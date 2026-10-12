param(
    [string] $ChildCase,
    [string] $FixtureRoot,
    [string] $InstallationId,
    [string] $LedgerId,
    [string] $DistroName,
    [string] $RunId,
    [string] $SessionId,
    [string] $Executable,
    [string] $SettingsFile
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 2.0

if ($env:OS -ne 'Windows_NT') {
    [Console]::Error.WriteLine('PENDING: native Windows 11/NTFS/Windows PowerShell 5.1 test gate requires Windows hardware')
    exit 3
}

$repositoryRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$filesystemPath = Join-Path $repositoryRoot 'orchestration\src\orchestration\windows\HostFilesystem.psm1'
$leasePath = Join-Path $repositoryRoot 'orchestration\src\orchestration\windows\HostLease.psm1'

function Assert-Equal {
    param($Actual, $Expected, [string] $Message = '')
    if ($Actual -ne $Expected) {
        throw "Assert-Equal failed: actual=[$Actual] expected=[$Expected] $Message"
    }
}

function Assert-True {
    param([bool] $Condition, [string] $Message)
    if (-not $Condition) {
        throw "Assert-True failed: $Message"
    }
}

function Get-ErrorCode {
    param([scriptblock] $Operation)
    try {
        & $Operation | Out-Null
    }
    catch {
        $code = $_.Exception.Data['code']
        if ($null -eq $code) {
            throw
        }
        return [string] $code
    }
    throw 'Expected operation to fail'
}

function Set-ModuleFault {
    param([System.Management.Automation.PSModuleInfo] $Module, [string] $Name)
    & $Module { param($FaultName) $script:TestFault = $FaultName } $Name
}

function Get-Sha256 {
    param([byte[]] $Bytes)
    $algorithm = [Security.Cryptography.SHA256]::Create()
    try {
        return ([BitConverter]::ToString($algorithm.ComputeHash($Bytes))).Replace('-', '').ToLowerInvariant()
    }
    finally {
        $algorithm.Dispose()
    }
}

function New-VersionedExecutable {
    param([string] $Path, [string] $Version)
    $source = @"
using System;
using System.Reflection;
using System.Threading;
[assembly: AssemblyTitle("QGroundControl")]
[assembly: AssemblyDescription("QGroundControl")]
[assembly: AssemblyProduct("QGroundControl")]
[assembly: AssemblyCompany("QGroundControl")]
[assembly: AssemblyFileVersion("$Version")]
[assembly: AssemblyInformationalVersion("$Version")]
[assembly: AssemblyVersion("$Version.0")]
public static class Program {
    [STAThread]
    public static void Main() { Thread.Sleep(30000); }
}
"@
    Add-Type -TypeDefinition $source -Language CSharp -OutputAssembly $Path -OutputType ConsoleApplication
}

function New-TestRegistryDistro {
    param([string] $Root, [string] $Name, [Guid] $Id)
    $key = Join-Path $Root ('Lxss\{' + $Id.ToString() + '}')
    New-Item -Path $key -Force | Out-Null
    New-ItemProperty -Path $key -Name DistributionName -Value $Name -PropertyType String -Force | Out-Null
}

function Start-TestChild {
    param([string] $Case, [hashtable] $Arguments)
    $argumentList = @('-NoLogo', '-NoProfile', '-NonInteractive', '-File', $PSCommandPath, '-ChildCase', $Case)
    foreach ($key in $Arguments.Keys) {
        $argumentList += "-$key"
        $argumentList += [string] $Arguments[$key]
    }
    return Start-Process -FilePath 'powershell.exe' -ArgumentList $argumentList -PassThru -WindowStyle Hidden
}

if ($ChildCase) {
    $env:DRONE_SIM_WINDOWS_NATIVE_TEST_MODE = '1'
    $env:DRONE_SIM_TEST_LOCALAPPDATA = $FixtureRoot
    $env:DRONE_SIM_TEST_LXSS_ROOT = $env:DRONE_SIM_PARENT_LXSS_ROOT
    $env:DRONE_SIM_TEST_MUTEX_SUFFIX = $env:DRONE_SIM_PARENT_MUTEX_SUFFIX
    $leaseModule = Import-Module $leasePath -Force -PassThru
    if ($ChildCase -eq 'CompeteForLease') {
        $code = Get-ErrorCode { Enter-ControllerLease -SessionId ([Guid]::NewGuid().ToString()) }
        [IO.File]::WriteAllText((Join-Path $FixtureRoot 'child-result.txt'), $code)
        exit 0
    }
    if ($ChildCase -eq 'AbandonActiveRun') {
        Enter-ControllerLease -SessionId $SessionId | Out-Null
        Register-Controller -InstallationId ([Guid] $InstallationId) -LedgerId ([Guid] $LedgerId) -DistroName $DistroName -Pristine $true | Out-Null
        Set-ActiveRun -RunId $RunId -SessionId $SessionId | Out-Null
        [IO.File]::WriteAllText((Join-Path $FixtureRoot 'child-result.txt'), 'active')
        exit 0
    }
    if ($ChildCase -eq 'PublishWithPause') {
        $filesystemModule = Import-Module $filesystemPath -Force -PassThru
        Set-ModuleFault $filesystemModule 'pause_before_rename'
        $bytes = [Text.Encoding]::UTF8.GetBytes('locked-path-publication')
        $receipt = Publish-QgcActions -Directory $FixtureRoot -Filename 'locked.json' -Bytes $bytes -Sha256 (Get-Sha256 $bytes)
        [IO.File]::WriteAllText((Join-Path ([IO.Path]::GetDirectoryName($FixtureRoot)) 'child-result.txt'), $receipt.sha256)
        exit 0
    }
    if ($ChildCase -eq 'OpenWithPause') {
        $filesystemModule = Import-Module $filesystemPath -Force -PassThru
        Set-ModuleFault $filesystemModule 'pause_before_qgc_open'
        $installation = Get-QgcInstallation -Executable $Executable -SettingsFile $SettingsFile
        $result = Open-Qgc -Installation $installation
        $selected = [IO.Path]::GetFullPath($Executable)
        foreach ($process in [Diagnostics.Process]::GetProcessesByName('QGroundControl')) {
            try {
                if ([string]::Equals([IO.Path]::GetFullPath($process.MainModule.FileName), $selected, [StringComparison]::OrdinalIgnoreCase)) {
                    $process.Kill()
                    $process.WaitForExit()
                }
            }
            finally { $process.Dispose() }
        }
        [IO.File]::WriteAllText((Join-Path $FixtureRoot 'child-result.txt'), $result.detail)
        exit 0
    }
    throw "Unknown child case: $ChildCase"
}

$testId = [Guid]::NewGuid().ToString('N')
$tempRoot = Join-Path ([IO.Path]::GetTempPath()) "drone-sim-windows-$testId"
$registryRoot = "HKCU:\Software\drone-sim-native-tests\$testId"
$env:DRONE_SIM_WINDOWS_NATIVE_TEST_MODE = '1'
$env:DRONE_SIM_TEST_LOCALAPPDATA = $tempRoot
$env:DRONE_SIM_TEST_LXSS_ROOT = (Join-Path $registryRoot 'Lxss')
$env:DRONE_SIM_TEST_MUTEX_SUFFIX = $testId
$env:DRONE_SIM_PARENT_LXSS_ROOT = $env:DRONE_SIM_TEST_LXSS_ROOT
$env:DRONE_SIM_PARENT_MUTEX_SUFFIX = $testId

$filesystemModule = $null
$leaseModule = $null
$passed = 0
$startedProcesses = New-Object 'Collections.Generic.List[Diagnostics.Process]'

function Invoke-Test {
    param([string] $Name, [scriptblock] $Body)
    & $Body
    $script:passed += 1
    Write-Output "PASS $Name"
}

try {
    New-Item -ItemType Directory -Path $tempRoot | Out-Null
    $volume = Get-Volume -FilePath $tempRoot
    Assert-Equal $volume.FileSystemType 'NTFS' 'native suite requires a temporary local NTFS directory'
    $windows = Get-CimInstance -ClassName Win32_OperatingSystem
    Write-Output "ENV Windows=$($windows.Caption) version=$($windows.Version) build=$($windows.BuildNumber)"
    Write-Output "ENV PowerShell=$($PSVersionTable.PSVersion.ToString()) edition=$($PSVersionTable.PSEdition)"
    Write-Output "ENV filesystem=$($volume.FileSystemType) driveType=$($volume.DriveType) path=$tempRoot"
    $wslVersion = (& wsl.exe --version 2>&1) -join '; '
    if ($LASTEXITCODE -ne 0) { throw "Unable to read WSL version: $wslVersion" }
    Write-Output "ENV WSL=$wslVersion"

    $filesystemModule = Import-Module $filesystemPath -Force -PassThru
    $leaseModule = Import-Module $leasePath -Force -PassThru

    $installDirectory = Join-Path $tempRoot 'installed'
    $otherInstallDirectory = Join-Path $tempRoot 'other-installed'
    $savePath = Join-Path $tempRoot 'QGC-évidence'
    $actionsDirectory = Join-Path $savePath 'MavlinkActions'
    New-Item -ItemType Directory -Path $installDirectory, $otherInstallDirectory, $actionsDirectory | Out-Null
    $executable = Join-Path $installDirectory 'QGroundControl.exe'
    $otherExecutable = Join-Path $otherInstallDirectory 'QGroundControl.exe'
    New-VersionedExecutable -Path $executable -Version '5.0.8'
    New-VersionedExecutable -Path $otherExecutable -Version '5.0.7'
    $settings = Join-Path $tempRoot 'QGroundControl.ini'
    $settingsText = "[General]`r`nSettingsVersion=9`r`nsavePath=$savePath`r`ngcsMavlinkSystemID=255`r`n`r`n[MavlinkActions]`r`nflyViewActionsFile=existing-é.json`r`n"
    $settingsBytes = (New-Object Text.UTF8Encoding($false)).GetBytes($settingsText)
    [IO.File]::WriteAllBytes($settings, $settingsBytes)

    Invoke-Test 'Test-DiscoveryReadsIdentityAndPreservesSettings' {
        $installation = Get-QgcInstallation -Executable $executable -SettingsFile $settings
        Assert-Equal $installation.version '5.0.8'
        Assert-Equal $installation.source.system_id 255
        Assert-Equal $installation.source.component_id 190
        Assert-Equal $installation.save_path $savePath
        Assert-Equal ([IO.File]::ReadAllBytes($settings) -join ',') ($settingsBytes -join ',')
    }

    Invoke-Test 'Test-UnicodePublicationAndUnchangedPreferences' {
        $bytes = (New-Object Text.UTF8Encoding($false)).GetBytes('{"label":"départ"}')
        $expectedHash = Get-Sha256 $bytes
        $receipt = Publish-QgcActions -Directory $actionsDirectory -Filename 'drone-sim-é.json' -Bytes $bytes -Sha256 $expectedHash
        Assert-Equal $receipt.sha256 $expectedHash
        Assert-Equal ([IO.File]::ReadAllBytes($settings) -join ',') ($settingsBytes -join ',')
        Assert-Equal ([IO.File]::ReadAllBytes($receipt.native_path) -join ',') ($bytes -join ',')
        $duplicate = Publish-QgcActions -Directory $actionsDirectory -Filename 'drone-sim-é.json' -Bytes $bytes -Sha256 $expectedHash
        Assert-Equal $duplicate.sha256 $expectedHash
    }

    Invoke-Test 'Test-ConflictingDuplicateIsRejected' {
        $bytes = [Text.Encoding]::UTF8.GetBytes('first')
        $hash = Get-Sha256 $bytes
        Publish-QgcActions -Directory $actionsDirectory -Filename 'conflict.json' -Bytes $bytes -Sha256 $hash | Out-Null
        $other = [Text.Encoding]::UTF8.GetBytes('second')
        $code = Get-ErrorCode { Publish-QgcActions -Directory $actionsDirectory -Filename 'conflict.json' -Bytes $other -Sha256 (Get-Sha256 $other) }
        Assert-Equal $code 'publication_conflict'
    }

    Invoke-Test 'Test-ReparseParentAndTargetAreRejected' {
        $real = Join-Path $tempRoot 'real-actions'
        $junction = Join-Path $tempRoot 'junction-actions'
        New-Item -ItemType Directory -Path $real | Out-Null
        $junctionOutput = & cmd.exe /d /c mklink /J $junction $real 2>&1
        if ($LASTEXITCODE -ne 0) { throw "Unable to create test junction: $junctionOutput" }
        $bytes = [Text.Encoding]::UTF8.GetBytes('safe')
        Assert-Equal (Get-ErrorCode { Publish-QgcActions -Directory $junction -Filename 'parent.json' -Bytes $bytes -Sha256 (Get-Sha256 $bytes) }) 'unsafe_publication_path'

        $target = Join-Path $actionsDirectory 'target-link.json'
        $backing = Join-Path $tempRoot 'target-backing-directory'
        New-Item -ItemType Directory -Path $backing | Out-Null
        $linkOutput = & cmd.exe /d /c mklink /J $target $backing 2>&1
        if ($LASTEXITCODE -ne 0) { throw "Unable to create target test junction: $linkOutput" }
        Assert-Equal (Get-ErrorCode { Publish-QgcActions -Directory $actionsDirectory -Filename 'target-link.json' -Bytes $bytes -Sha256 (Get-Sha256 $bytes) }) 'unsafe_publication_path'
    }

    foreach ($storageFault in @('network_storage', 'non_ntfs_storage')) {
        Invoke-Test "Test-UnsupportedStorage-$storageFault" {
            Set-ModuleFault $filesystemModule $storageFault
            $bytes = [Text.Encoding]::UTF8.GetBytes("storage-$storageFault")
            Assert-Equal (Get-ErrorCode { Publish-QgcActions -Directory $actionsDirectory -Filename "storage-$storageFault.json" -Bytes $bytes -Sha256 (Get-Sha256 $bytes) }) 'unsupported_publication_storage'
            Set-ModuleFault $filesystemModule ''
        }
    }

    Invoke-Test 'Test-ValidatedDirectoryCannotBeRenamedOrReplacedDuringPublication' {
        $raceDirectory = Join-Path $tempRoot 'race-actions'
        $raceReplacement = Join-Path $tempRoot 'race-replacement'
        $raceMoved = Join-Path $tempRoot 'race-actions-moved'
        New-Item -ItemType Directory -Path $raceDirectory, $raceReplacement | Out-Null
        $env:DRONE_SIM_TEST_PAUSE_READY = Join-Path $tempRoot 'pause-ready'
        $env:DRONE_SIM_TEST_PAUSE_CONTINUE = Join-Path $tempRoot 'pause-continue'
        Remove-Item -LiteralPath $env:DRONE_SIM_TEST_PAUSE_READY, $env:DRONE_SIM_TEST_PAUSE_CONTINUE -Force -ErrorAction SilentlyContinue
        $child = Start-TestChild -Case 'PublishWithPause' -Arguments @{ FixtureRoot = $raceDirectory }
        $deadline = [DateTime]::UtcNow.AddSeconds(10)
        while (-not [IO.File]::Exists($env:DRONE_SIM_TEST_PAUSE_READY)) {
            if ([DateTime]::UtcNow -ge $deadline) { throw 'Publisher child did not reach locked rename boundary' }
            Start-Sleep -Milliseconds 25
        }
        try {
            $renameBlocked = $false
            try { Move-Item -LiteralPath $raceDirectory -Destination $raceMoved -ErrorAction Stop }
            catch { $renameBlocked = $true }
            Assert-True $renameBlocked 'validated directory rename must be denied while native handles are held without delete sharing'
            $junctionOutput = & cmd.exe /d /c mklink /J $raceDirectory $raceReplacement 2>&1
            Assert-True ($LASTEXITCODE -ne 0) "junction replacement unexpectedly succeeded: $junctionOutput"
        }
        finally {
            [IO.File]::WriteAllText($env:DRONE_SIM_TEST_PAUSE_CONTINUE, 'continue')
        }
        $child.WaitForExit()
        Assert-Equal $child.ExitCode 0
        Assert-True ([IO.File]::Exists((Join-Path $raceDirectory 'locked.json'))) 'child publication did not finish in the held directory'
    }

    foreach ($faultCase in @(
        @('before_flush', 'publication_interrupted'),
        @('flush', 'publication_flush_failed'),
        @('before_rename', 'publication_interrupted'),
        @('after_rename', 'publication_uncertain'),
        @('readback', 'publication_readback_failed')
    )) {
        $faultName = $faultCase[0]
        $expectedCode = $faultCase[1]
        Invoke-Test "Test-PublicationFault-$faultName" {
            Set-ModuleFault $filesystemModule $faultName
            $bytes = [Text.Encoding]::UTF8.GetBytes("fault-$faultName")
            $code = Get-ErrorCode { Publish-QgcActions -Directory $actionsDirectory -Filename "fault-$faultName.json" -Bytes $bytes -Sha256 (Get-Sha256 $bytes) }
            Assert-Equal $code $expectedCode
            Set-ModuleFault $filesystemModule ''
            $evidence = @(Get-ChildItem -LiteralPath $actionsDirectory -Force | Where-Object { $_.Name -like "fault-$faultName.json*" })
            Assert-True ($evidence.Count -ge 1) 'uncertain write must retain destination or temporary evidence'
        }
    }

    Invoke-Test 'Test-RunningQgcExecutableVersionMismatchIsRejected' {
        Assert-Equal (Get-ErrorCode { Get-QgcInstallation -Executable $otherExecutable -SettingsFile $settings }) 'qgc_version_mismatch'
        $process = Start-Process -FilePath $otherExecutable -PassThru
        $startedProcesses.Add($process)
        Start-Sleep -Milliseconds 300
        $code = Get-ErrorCode { Get-QgcInstallation -Executable $executable -SettingsFile $settings }
        Assert-Equal $code 'qgc_running_installation_mismatch'
        $process.Kill()
        $process.WaitForExit()
    }

    Invoke-Test 'Test-ValidatedQgcExecutableCannotBeReplacedBeforeLaunch' {
        $env:DRONE_SIM_TEST_PAUSE_READY = Join-Path $tempRoot 'qgc-open-pause-ready'
        $env:DRONE_SIM_TEST_PAUSE_CONTINUE = Join-Path $tempRoot 'qgc-open-pause-continue'
        Remove-Item -LiteralPath $env:DRONE_SIM_TEST_PAUSE_READY, $env:DRONE_SIM_TEST_PAUSE_CONTINUE -Force -ErrorAction SilentlyContinue
        $child = Start-TestChild -Case 'OpenWithPause' -Arguments @{ FixtureRoot = $tempRoot; Executable = $executable; SettingsFile = $settings }
        $deadline = [DateTime]::UtcNow.AddSeconds(10)
        while (-not [IO.File]::Exists($env:DRONE_SIM_TEST_PAUSE_READY)) {
            if ([DateTime]::UtcNow -ge $deadline) { throw 'Open-Qgc child did not reach locked launch boundary' }
            Start-Sleep -Milliseconds 25
        }
        try {
            $writeBlocked = $false
            try { [IO.File]::WriteAllBytes($executable, [byte[]] @(0, 1, 2)) }
            catch { $writeBlocked = $true }
            Assert-True $writeBlocked 'validated QGC executable write must be denied through launch'

            $renameBlocked = $false
            try { Move-Item -LiteralPath $executable -Destination ($executable + '.replaced') -ErrorAction Stop }
            catch { $renameBlocked = $true }
            Assert-True $renameBlocked 'validated QGC executable replacement must be denied through launch'

            $ancestorRenameBlocked = $false
            try { Move-Item -LiteralPath $installDirectory -Destination ($installDirectory + '-replaced') -ErrorAction Stop }
            catch { $ancestorRenameBlocked = $true }
            Assert-True $ancestorRenameBlocked 'validated QGC ancestor replacement must be denied through launch'
        }
        finally {
            [IO.File]::WriteAllText($env:DRONE_SIM_TEST_PAUSE_CONTINUE, 'continue')
        }
        $child.WaitForExit()
        Assert-Equal $child.ExitCode 0
        Assert-True ([IO.File]::ReadAllText((Join-Path $tempRoot 'child-result.txt')).StartsWith('started QGroundControl process ')) 'validated fake QGC did not launch'
    }

    $distroA = 'Ubuntu-24.04-test-a'
    $distroB = 'Ubuntu-24.04-test-b'
    $distroAId = [Guid]::NewGuid()
    $distroBId = [Guid]::NewGuid()
    New-TestRegistryDistro -Root $registryRoot -Name $distroA -Id $distroAId
    New-TestRegistryDistro -Root $registryRoot -Name $distroB -Id $distroBId
    $installationGuid = [Guid]::NewGuid()
    $ledgerGuid = [Guid]::NewGuid()

    Invoke-Test 'Test-MissingAndAmbiguousDistroIdentityFailClosed' {
        Enter-ControllerLease -SessionId ([Guid]::NewGuid().ToString()) | Out-Null
        try {
            Assert-Equal (Get-ErrorCode { Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName 'missing-distro' -Pristine $true }) 'controller_distro_identity_missing'
            $duplicateId = [Guid]::NewGuid()
            New-TestRegistryDistro -Root $registryRoot -Name $distroA -Id $duplicateId
            try {
                Assert-Equal (Get-ErrorCode { Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroA -Pristine $true }) 'controller_distro_identity_ambiguous'
            }
            finally {
                Remove-Item -LiteralPath (Join-Path $registryRoot ('Lxss\{' + $duplicateId.ToString() + '}')) -Recurse -Force
            }
        }
        finally { Exit-ControllerLease }
    }

    Invoke-Test 'Test-MissingRegistrationWithExistingLedgerFails' {
        Enter-ControllerLease -SessionId ([Guid]::NewGuid().ToString()) | Out-Null
        try {
            $code = Get-ErrorCode { Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroA -Pristine $false }
            Assert-Equal $code 'controller_registration_missing'
        }
        finally { Exit-ControllerLease }
    }

    Invoke-Test 'Test-PartialEnrollmentFailsClosed' {
        $registrationDirectory = Join-Path $tempRoot 'drone-sim'
        New-Item -ItemType Directory -Path $registrationDirectory -Force | Out-Null
        [IO.File]::WriteAllText((Join-Path $registrationDirectory 'controller-registration-v1.json'), '{"version":1}')
        Enter-ControllerLease -SessionId ([Guid]::NewGuid().ToString()) | Out-Null
        try {
            Assert-Equal (Get-ErrorCode { Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroA -Pristine $true }) 'controller_registration_partial'
        }
        finally { Exit-ControllerLease }
        Remove-Item -LiteralPath (Join-Path $registrationDirectory 'controller-registration-v1.json') -Force
    }

    foreach ($registrationFault in @('registration_flush', 'registration_replace')) {
        Invoke-Test "Test-FirstEnrollmentRetryBlockedByOrphan-$registrationFault" {
            $registrationDirectory = Join-Path $tempRoot 'drone-sim'
            Get-ChildItem -LiteralPath $registrationDirectory -Filter 'controller-registration-v1.json*' -Force -ErrorAction SilentlyContinue | Remove-Item -Force
            Enter-ControllerLease -SessionId ([Guid]::NewGuid().ToString()) | Out-Null
            try {
                Set-ModuleFault $leaseModule $registrationFault
                $code = Get-ErrorCode { Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroA -Pristine $true }
                Assert-Equal $code 'controller_registration_write_failed'
                Set-ModuleFault $leaseModule ''
                $evidence = @(Get-ChildItem -LiteralPath $registrationDirectory -Filter 'controller-registration-v1.json.tmp-*' -Force)
                Assert-True ($evidence.Count -ge 1) 'failed or uncertain registration update must retain evidence'
                Assert-Equal (Get-ErrorCode { Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroA -Pristine $true }) 'controller_recovery_required'
            }
            finally {
                Set-ModuleFault $leaseModule ''
                Exit-ControllerLease
                Get-ChildItem -LiteralPath $registrationDirectory -Filter 'controller-registration-v1.json*' -Force -ErrorAction SilentlyContinue | Remove-Item -Force
            }
        }
    }

    foreach ($registrationFault in @('registration_flush', 'registration_replace')) {
        Invoke-Test "Test-ExistingRegistrationUpdateRetryBlockedByOrphan-$registrationFault" {
            $registrationDirectory = Join-Path $tempRoot 'drone-sim'
            Get-ChildItem -LiteralPath $registrationDirectory -Filter 'controller-registration-v1.json*' -Force -ErrorAction SilentlyContinue | Remove-Item -Force
            $activeSession = [Guid]::NewGuid().ToString()
            Enter-ControllerLease -SessionId $activeSession | Out-Null
            try {
                Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroA -Pristine $true | Out-Null
                Set-ModuleFault $leaseModule $registrationFault
                Assert-Equal (Get-ErrorCode { Set-ActiveRun -RunId ([Guid]::NewGuid().ToString()) -SessionId $activeSession }) 'controller_registration_write_failed'
                Set-ModuleFault $leaseModule ''
                $orphanEvidence = @(Get-ChildItem -LiteralPath $registrationDirectory -Filter 'controller-registration-v1.json.tmp-*' -Force)
                Assert-True ($orphanEvidence.Count -ge 1) 'failed active-run update must retain temporary evidence beside the old final registration'
                Assert-Equal (Get-ErrorCode { Set-ActiveRun -RunId ([Guid]::NewGuid().ToString()) -SessionId $activeSession }) 'controller_recovery_required'
            }
            finally {
                Set-ModuleFault $leaseModule ''
                Exit-ControllerLease
                Get-ChildItem -LiteralPath $registrationDirectory -Filter 'controller-registration-v1.json*' -Force -ErrorAction SilentlyContinue | Remove-Item -Force
            }
        }
    }

    Invoke-Test 'Test-DurableRegistrationReadbackFailureIsVisible' {
        $registrationDirectory = Join-Path $tempRoot 'drone-sim'
        Get-ChildItem -LiteralPath $registrationDirectory -Filter 'controller-registration-v1.json*' -Force -ErrorAction SilentlyContinue | Remove-Item -Force
        Enter-ControllerLease -SessionId ([Guid]::NewGuid().ToString()) | Out-Null
        try {
            Set-ModuleFault $leaseModule 'registration_readback'
            Assert-Equal (Get-ErrorCode { Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroA -Pristine $true }) 'controller_registration_readback_failed'
        }
        finally {
            Set-ModuleFault $leaseModule ''
            Exit-ControllerLease
        }
    }

    Invoke-Test 'Test-ClonedDistroCannotReuseLedger' {
        Enter-ControllerLease -SessionId ([Guid]::NewGuid().ToString()) | Out-Null
        try { Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroA -Pristine $true | Out-Null }
        finally { Exit-ControllerLease }
        Enter-ControllerLease -SessionId ([Guid]::NewGuid().ToString()) | Out-Null
        try {
            $result = [pscustomobject] @{ code = (Get-ErrorCode { Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroB -Pristine $false }) }
            Assert-Equal $result.code 'controller_identity_mismatch'
        }
        finally { Exit-ControllerLease }
    }

    Invoke-Test 'Test-CompetingWindowsMutexFails' {
        Enter-ControllerLease -SessionId ([Guid]::NewGuid().ToString()) | Out-Null
        try {
            Remove-Item -LiteralPath (Join-Path $tempRoot 'child-result.txt') -Force -ErrorAction SilentlyContinue
            $child = Start-TestChild -Case 'CompeteForLease' -Arguments @{ FixtureRoot = $tempRoot }
            $child.WaitForExit()
            Assert-Equal $child.ExitCode 0
            Assert-Equal ([IO.File]::ReadAllText((Join-Path $tempRoot 'child-result.txt'))) 'controller_lease_unavailable'
        }
        finally { Exit-ControllerLease }
    }

    Invoke-Test 'Test-LiveFormerWindowsProcessBlocksRecovery' {
        $ownedRun = [Guid]::NewGuid().ToString()
        $ownedSession = [Guid]::NewGuid().ToString()
        Enter-ControllerLease -SessionId $ownedSession | Out-Null
        try {
            Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroA -Pristine $false | Out-Null
            Set-ActiveRun -RunId $ownedRun -SessionId $ownedSession | Out-Null
        }
        finally { Exit-ControllerLease }

        $bytes = [Text.Encoding]::UTF8.GetBytes('live-owner')
        $filename = "drone-sim-$ownedRun.json"
        $receipt = Publish-QgcActions -Directory $actionsDirectory -Filename $filename -Bytes $bytes -Sha256 (Get-Sha256 $bytes)
        Enter-ControllerLease -SessionId ([Guid]::NewGuid().ToString()) | Out-Null
        try {
            Assert-Equal (Get-ErrorCode { Confirm-RunRecovery -RunId $ownedRun -Publication $receipt }) 'controller_helper_still_running'
        }
        finally { Exit-ControllerLease }

        Enter-ControllerLease -SessionId $ownedSession | Out-Null
        try {
            Confirm-RunRecovery -RunId $ownedRun -Publication $receipt | Out-Null
            Assert-Equal (Get-ErrorCode { Complete-ActiveRun -RunId $ownedRun -VerifiedCleanup $false }) 'active_run_cleanup_unverified'
            $retained = Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroA -Pristine $false
            Assert-Equal $retained.active_run_id $ownedRun
            Complete-ActiveRun -RunId $ownedRun -VerifiedCleanup $true | Out-Null
        }
        finally { Exit-ControllerLease }
    }

    Invoke-Test 'Test-AbandonedMutexRetainsActiveRun' {
        $registrationFile = Join-Path (Join-Path $tempRoot 'drone-sim') 'controller-registration-v1.json'
        Remove-Item -LiteralPath $registrationFile -Force
        $runGuid = [Guid]::NewGuid().ToString()
        $sessionGuid = [Guid]::NewGuid().ToString()
        Remove-Item -LiteralPath (Join-Path $tempRoot 'child-result.txt') -Force -ErrorAction SilentlyContinue
        $child = Start-TestChild -Case 'AbandonActiveRun' -Arguments @{
            FixtureRoot = $tempRoot; InstallationId = $installationGuid; LedgerId = $ledgerGuid
            DistroName = $distroA; RunId = $runGuid; SessionId = $sessionGuid
        }
        $child.WaitForExit()
        Assert-Equal $child.ExitCode 0
        Assert-Equal ([IO.File]::ReadAllText((Join-Path $tempRoot 'child-result.txt'))) 'active'

        $recoverySession = [Guid]::NewGuid().ToString()
        $lease = Enter-ControllerLease -SessionId $recoverySession
        Assert-True $lease.abandoned 'child exit without ReleaseMutex must report abandonment'
        try {
            $registration = Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroA -Pristine $false
            Assert-Equal $registration.active_run_id $runGuid
            Assert-True $registration.recovery_required 'abandoned active owner requires recovery'
        }
        finally { Exit-ControllerLease }
    }

    Invoke-Test 'Test-RecoveryVerifiesPublicationAndRebindsWindowsProcessIdentity' {
        $recoverySession = [Guid]::NewGuid().ToString()
        Enter-ControllerLease -SessionId $recoverySession | Out-Null
        try {
            $registration = Register-Controller -InstallationId $installationGuid -LedgerId $ledgerGuid -DistroName $distroA -Pristine $false
            $recoveryFilename = "drone-sim-$($registration.active_run_id).json"
            $receipt = Publish-QgcActions -Directory $actionsDirectory -Filename $recoveryFilename -Bytes ([Text.Encoding]::UTF8.GetBytes('recovery')) -Sha256 (Get-Sha256 ([Text.Encoding]::UTF8.GetBytes('recovery')))
            $recovered = Confirm-RunRecovery -RunId $registration.active_run_id -Publication $receipt
            Assert-Equal $recovered.helper_process_id $PID
            Assert-Equal $recovered.helper_session_id $recoverySession
            Complete-ActiveRun -RunId $registration.active_run_id -VerifiedCleanup $true | Out-Null
        }
        finally { Exit-ControllerLease }
    }

    Write-Output "RESULT: $passed native Windows host helper tests passed"
}
finally {
    foreach ($process in $startedProcesses) {
        try { if (-not $process.HasExited) { $process.Kill(); $process.WaitForExit() } } catch {}
    }
    try { if ($leaseModule) { & $leaseModule { if ($script:LeaseHeld) { Exit-ControllerLease } } } } catch {}
    Remove-Module HostFilesystem, HostLease -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $registryRoot -Recurse -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $tempRoot -Recurse -Force -ErrorAction SilentlyContinue
}
