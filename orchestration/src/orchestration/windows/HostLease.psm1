$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 2.0

$script:LeaseMutex = $null
$script:LeaseHeld = $false
$script:LeaseAbandoned = $false
$script:LeaseThreadId = $null
$script:LeaseSessionId = $null
$script:TestFault = ''

if (-not ('DroneSim.WindowsFileSystemNative' -as [type])) {
    Import-Module (Join-Path $PSScriptRoot 'HostFilesystem.psm1') -Force
}

if (-not ('DroneSim.WindowsLeaseNative' -as [type])) {
    Add-Type -TypeDefinition @'
using System;
using System.ComponentModel;
using System.IO;
using System.Runtime.InteropServices;

namespace DroneSim {
    public static class WindowsLeaseNative {
        private const uint MOVEFILE_REPLACE_EXISTING = 0x1;
        private const uint MOVEFILE_WRITE_THROUGH = 0x8;

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool MoveFileExW(string existingName, string newName, uint flags);

        public static void ReplaceWriteThrough(string source, string destination) {
            if (!MoveFileExW(source, destination, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
                throw new Win32Exception(Marshal.GetLastWin32Error(), "MoveFileExW durable registration replacement failed");
            }
        }
    }
}
'@
}

function Throw-HostLeaseError {
    param([string] $Code, [string] $Message, [Exception] $InnerException)
    if ($InnerException) {
        $exception = New-Object InvalidOperationException($Message, $InnerException)
    }
    else {
        $exception = New-Object InvalidOperationException($Message)
    }
    $exception.Data['code'] = $Code
    throw $exception
}

function Assert-NativeTestOverride {
    if ($env:DRONE_SIM_WINDOWS_NATIVE_TEST_MODE -ne '1') {
        Throw-HostLeaseError 'controller_test_override_rejected' 'Windows controller test overrides require explicit native test mode' $null
    }
}

function ConvertTo-CanonicalGuid {
    param([object] $Value, [string] $Code, [string] $Label)
    $parsed = [Guid]::Empty
    if (-not [Guid]::TryParse([string] $Value, [ref] $parsed) -or $parsed -eq [Guid]::Empty) {
        Throw-HostLeaseError $Code "$Label must be a nonzero UUID" $null
    }
    return $parsed.ToString()
}

function Get-CurrentProcessIdentity {
    $process = [Diagnostics.Process]::GetCurrentProcess()
    try {
        return [pscustomobject] @{
            process_id = $process.Id
            process_start_time_utc = $process.StartTime.ToUniversalTime().ToString('o')
        }
    }
    finally {
        $process.Dispose()
    }
}

function Get-RegistrationPath {
    if ($env:DRONE_SIM_TEST_LOCALAPPDATA) {
        Assert-NativeTestOverride
        $localAppData = [IO.Path]::GetFullPath($env:DRONE_SIM_TEST_LOCALAPPDATA)
    }
    else {
        $localAppData = [Environment]::GetFolderPath([Environment+SpecialFolder]::LocalApplicationData)
    }
    if ([string]::IsNullOrWhiteSpace($localAppData) -or -not [IO.Path]::IsPathRooted($localAppData)) {
        Throw-HostLeaseError 'controller_registration_path_invalid' 'Windows Local AppData is unavailable' $null
    }
    return Join-Path (Join-Path $localAppData 'drone-sim') 'controller-registration-v1.json'
}

function Get-ControllerMutexName {
    try {
        $sid = [Security.Principal.WindowsIdentity]::GetCurrent().User.Value
    }
    catch {
        Throw-HostLeaseError 'controller_windows_identity_missing' 'Cannot resolve the current Windows user SID' $_.Exception
    }
    $suffix = ''
    if ($env:DRONE_SIM_TEST_MUTEX_SUFFIX) {
        Assert-NativeTestOverride
        if ($env:DRONE_SIM_TEST_MUTEX_SUFFIX -notmatch '^[A-Za-z0-9-]+$') {
            Throw-HostLeaseError 'controller_test_override_rejected' 'Test mutex suffix contains unsupported characters' $null
        }
        $suffix = '-' + $env:DRONE_SIM_TEST_MUTEX_SUFFIX
    }
    return "Local\drone-sim-controller-$sid$suffix"
}

function Assert-LeaseHeld {
    if (-not $script:LeaseHeld -or $null -eq $script:LeaseMutex) {
        Throw-HostLeaseError 'controller_lease_required' 'The Windows controller lease is not held' $null
    }
    $threadId = [Threading.Thread]::CurrentThread.ManagedThreadId
    if ($threadId -ne $script:LeaseThreadId) {
        Throw-HostLeaseError 'controller_lease_thread_mismatch' 'The Windows controller lease must be used and released on its acquiring helper event-loop thread' $null
    }
}

function Enter-ControllerLease {
    [CmdletBinding()]
    param([Parameter(Mandatory = $true)][string] $SessionId)

    if ($script:LeaseHeld) {
        Throw-HostLeaseError 'controller_lease_already_held' 'This helper already holds the Windows controller lease' $null
    }
    $canonicalSessionId = ConvertTo-CanonicalGuid $SessionId 'controller_session_invalid' 'Helper session ID'
    if ($SessionId -cne $canonicalSessionId) {
        Throw-HostLeaseError 'controller_session_invalid' 'Helper session ID must use canonical lowercase UUID text' $null
    }
    $name = Get-ControllerMutexName
    $mutex = New-Object Threading.Mutex($false, $name)
    $acquired = $false
    $abandoned = $false
    try {
        try {
            $acquired = $mutex.WaitOne(0)
        }
        catch [Threading.AbandonedMutexException] {
            $acquired = $true
            $abandoned = $true
        }
        if (-not $acquired) {
            Throw-HostLeaseError 'controller_lease_unavailable' 'Another helper holds the Windows controller lease' $null
        }
        $script:LeaseMutex = $mutex
        $script:LeaseHeld = $true
        $script:LeaseAbandoned = $abandoned
        $script:LeaseThreadId = [Threading.Thread]::CurrentThread.ManagedThreadId
        $script:LeaseSessionId = $canonicalSessionId
        $identity = Get-CurrentProcessIdentity
        return [pscustomobject] @{
            mutex_name = $name
            abandoned = $abandoned
            session_id = $canonicalSessionId
            helper_process_id = $identity.process_id
            helper_process_start_time_utc = $identity.process_start_time_utc
            owner_thread_id = $script:LeaseThreadId
        }
    }
    catch {
        if (-not $script:LeaseHeld) { $mutex.Dispose() }
        throw
    }
}

function Exit-ControllerLease {
    [CmdletBinding()]
    param()

    Assert-LeaseHeld
    $mutex = $script:LeaseMutex
    try {
        $mutex.ReleaseMutex()
    }
    finally {
        $script:LeaseMutex = $null
        $script:LeaseHeld = $false
        $script:LeaseAbandoned = $false
        $script:LeaseThreadId = $null
        $script:LeaseSessionId = $null
        $mutex.Dispose()
    }
}

function Get-LxssDistroId {
    param([string] $DistroName)
    if ([string]::IsNullOrWhiteSpace($DistroName)) {
        Throw-HostLeaseError 'controller_distro_identity_missing' 'WSL distribution name is missing' $null
    }
    if ($env:DRONE_SIM_TEST_LXSS_ROOT) {
        Assert-NativeTestOverride
        $root = $env:DRONE_SIM_TEST_LXSS_ROOT
    }
    else {
        $root = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Lxss'
        if ($env:WSL_DISTRO_NAME -ne $DistroName) {
            Throw-HostLeaseError 'controller_distro_identity_mismatch' 'Requested WSL distribution does not match WSL_DISTRO_NAME' $null
        }
    }
    if (-not (Test-Path -LiteralPath $root -PathType Container)) {
        Throw-HostLeaseError 'controller_distro_identity_missing' 'Expected Windows Lxss registration key is missing' $null
    }
    $matches = New-Object 'Collections.Generic.List[Guid]'
    foreach ($child in Get-ChildItem -LiteralPath $root) {
        $candidateId = [Guid]::Empty
        $leaf = $child.PSChildName.Trim('{', '}')
        if (-not [Guid]::TryParse($leaf, [ref] $candidateId)) { continue }
        try {
            $properties = Get-ItemProperty -LiteralPath $child.PSPath -ErrorAction Stop
            if ([string] $properties.DistributionName -ceq $DistroName) {
                $matches.Add($candidateId)
            }
        }
        catch {
            Throw-HostLeaseError 'controller_distro_identity_unknown' "Cannot inspect WSL registration $($child.PSChildName)" $_.Exception
        }
    }
    if ($matches.Count -eq 0) {
        Throw-HostLeaseError 'controller_distro_identity_missing' "No Windows Lxss registration uniquely identifies distribution $DistroName" $null
    }
    if ($matches.Count -ne 1) {
        Throw-HostLeaseError 'controller_distro_identity_ambiguous' "Multiple Windows Lxss registrations identify distribution $DistroName" $null
    }
    return $matches[0].ToString()
}

function Test-ObjectProperty {
    param([object] $Object, [string] $Name)
    return $null -ne $Object.PSObject.Properties[$Name]
}

function Read-ControllerRegistration {
    $path = Get-RegistrationPath
    $directory = [IO.Path]::GetDirectoryName($path)
    if (-not [IO.Directory]::Exists($directory)) { return $null }
    $directoryLock = $null
    $stream = $null
    try {
        $directoryLock = [DroneSim.WindowsFileSystemNative]::LockDirectoryPath($directory)
        [DroneSim.WindowsFileSystemNative]::ValidateLocalNtfs($directory)
    }
    catch {
        if ($null -ne $directoryLock) { $directoryLock.Dispose() }
        Throw-HostLeaseError 'controller_registration_path_invalid' 'Controller registration directory must be local NTFS without reparse components' $_.Exception
    }
    if (-not [DroneSim.WindowsFileSystemNative]::PathExistsNoFollow($path)) {
        $directoryLock.Dispose()
        return $null
    }
    try {
        $stream = [DroneSim.WindowsFileSystemNative]::OpenRegularReadLocked($path)
        $memory = New-Object IO.MemoryStream
        try { $stream.CopyTo($memory); $rawBytes = $memory.ToArray() }
        finally { $memory.Dispose() }
        $encoding = New-Object Text.UTF8Encoding($false, $true)
        $text = $encoding.GetString($rawBytes)
        $registration = $text | ConvertFrom-Json
    }
    catch {
        Throw-HostLeaseError 'controller_registration_partial' 'Controller registration is not valid UTF-8 JSON' $_.Exception
    }
    finally {
        if ($null -ne $stream) { $stream.Dispose() }
        if ($null -ne $directoryLock) { $directoryLock.Dispose() }
    }
    $required = @(
        'version', 'distro_id', 'installation_id', 'ledger_id', 'active_run_id',
        'helper_process_id', 'helper_process_start_time_utc', 'helper_session_id'
    )
    foreach ($name in $required) {
        if (-not (Test-ObjectProperty $registration $name)) {
            Throw-HostLeaseError 'controller_registration_partial' "Controller registration is missing $name" $null
        }
    }
    if ($registration.version -ne 1) {
        Throw-HostLeaseError 'controller_registration_version_unsupported' "Controller registration version $($registration.version) is unsupported" $null
    }
    $registration.distro_id = ConvertTo-CanonicalGuid $registration.distro_id 'controller_registration_partial' 'Registered distro ID'
    $registration.installation_id = ConvertTo-CanonicalGuid $registration.installation_id 'controller_registration_partial' 'Registered installation ID'
    $registration.ledger_id = ConvertTo-CanonicalGuid $registration.ledger_id 'controller_registration_partial' 'Registered ledger ID'
    $hasRun = $null -ne $registration.active_run_id
    $helperValues = @($registration.helper_process_id, $registration.helper_process_start_time_utc, $registration.helper_session_id)
    $helperPresent = @($helperValues | Where-Object { $null -ne $_ }).Count
    if (-not $hasRun -and $helperPresent -ne 0) {
        Throw-HostLeaseError 'controller_registration_partial' 'Inactive controller registration retains a partial helper identity' $null
    }
    if ($hasRun) {
        if ([string]::IsNullOrWhiteSpace([string] $registration.active_run_id) -or $helperPresent -ne 3) {
            Throw-HostLeaseError 'controller_registration_partial' 'Active controller registration has a partial run/helper identity' $null
        }
        $pidValue = 0
        if (-not [int]::TryParse([string] $registration.helper_process_id, [ref] $pidValue) -or $pidValue -le 0) {
            Throw-HostLeaseError 'controller_registration_partial' 'Registered helper process ID is invalid' $null
        }
        $timestamp = [DateTime]::MinValue
        if (-not [DateTime]::TryParse(
            [string] $registration.helper_process_start_time_utc,
            [Globalization.CultureInfo]::InvariantCulture,
            [Globalization.DateTimeStyles]::RoundtripKind,
            [ref] $timestamp)) {
            Throw-HostLeaseError 'controller_registration_partial' 'Registered helper process start time is invalid' $null
        }
        $registration.helper_session_id = ConvertTo-CanonicalGuid $registration.helper_session_id 'controller_registration_partial' 'Registered helper session ID'
        $registration.helper_process_id = $pidValue
        $registration.helper_process_start_time_utc = $timestamp.ToUniversalTime().ToString('o')
    }
    return $registration
}

function Write-ControllerRegistration {
    param([object] $Registration)
    Assert-LeaseHeld
    $path = Get-RegistrationPath
    $directory = [IO.Path]::GetDirectoryName($path)
    $parent = [IO.Directory]::GetParent($directory)
    if ($null -eq $parent -or -not $parent.Exists) {
        Throw-HostLeaseError 'controller_registration_path_invalid' 'Controller registration parent directory is missing' $null
    }
    $parentLock = $null
    $directoryLock = $null
    try {
        $parentLock = [DroneSim.WindowsFileSystemNative]::LockDirectoryPath($parent.FullName)
        [IO.Directory]::CreateDirectory($directory) | Out-Null
        $directoryLock = [DroneSim.WindowsFileSystemNative]::LockDirectoryPath($directory)
        [DroneSim.WindowsFileSystemNative]::ValidateLocalNtfs($directory)
    }
    catch {
        if ($null -ne $directoryLock) { $directoryLock.Dispose() }
        if ($null -ne $parentLock) { $parentLock.Dispose() }
        Throw-HostLeaseError 'controller_registration_path_invalid' 'Controller registration directory must be local NTFS without reparse components' $_.Exception
    }
    $temporary = Join-Path $directory ('controller-registration-v1.json.tmp-' + [Guid]::NewGuid().ToString('N'))
    $encoding = New-Object Text.UTF8Encoding($false)
    $bytes = $encoding.GetBytes(($Registration | ConvertTo-Json -Depth 4 -Compress))
    $stream = $null
    try {
        $stream = New-Object IO.FileStream(
            $temporary,
            [IO.FileMode]::CreateNew,
            [IO.FileAccess]::Write,
            [IO.FileShare]::None,
            4096,
            [IO.FileOptions]::WriteThrough)
        $stream.Write($bytes, 0, $bytes.Length)
        if ($script:TestFault -eq 'registration_flush') {
            Throw-HostLeaseError 'controller_registration_write_failed' 'Injected registration flush failure' $null
        }
        $stream.Flush($true)
        $stream.Dispose()
        $stream = $null
        if ($script:TestFault -eq 'registration_replace') {
            Throw-HostLeaseError 'controller_registration_write_failed' 'Injected registration replacement failure' $null
        }
        [DroneSim.WindowsLeaseNative]::ReplaceWriteThrough($temporary, $path)
        if ($script:TestFault -eq 'registration_readback') {
            Throw-HostLeaseError 'controller_registration_readback_failed' 'Injected registration readback failure' $null
        }
        $readbackStream = $null
        try {
            $readbackStream = [DroneSim.WindowsFileSystemNative]::OpenRegularReadLocked($path)
            $readbackMemory = New-Object IO.MemoryStream
            try { $readbackStream.CopyTo($readbackMemory); $readback = $readbackMemory.ToArray() }
            finally { $readbackMemory.Dispose() }
        }
        finally {
            if ($null -ne $readbackStream) { $readbackStream.Dispose() }
        }
        if ($readback.Length -ne $bytes.Length) {
            Throw-HostLeaseError 'controller_registration_readback_failed' 'Durable controller registration readback length differs' $null
        }
        for ($index = 0; $index -lt $bytes.Length; $index++) {
            if ($readback[$index] -ne $bytes[$index]) {
                Throw-HostLeaseError 'controller_registration_readback_failed' 'Durable controller registration readback bytes differ' $null
            }
        }
    }
    catch {
        if ($_.Exception.Data['code']) { throw }
        Throw-HostLeaseError 'controller_registration_write_failed' "Failed to write durable Windows controller registration; evidence retained at $temporary" $_.Exception
    }
    finally {
        if ($null -ne $stream) { $stream.Dispose() }
        if ($null -ne $directoryLock) { $directoryLock.Dispose() }
        if ($null -ne $parentLock) { $parentLock.Dispose() }
    }
}

function Copy-RegistrationResult {
    param([object] $Registration, [bool] $RecoveryRequired)
    return [pscustomobject] @{
        version = [int] $Registration.version
        distro_id = [string] $Registration.distro_id
        installation_id = [string] $Registration.installation_id
        ledger_id = [string] $Registration.ledger_id
        active_run_id = $Registration.active_run_id
        helper_process_id = $Registration.helper_process_id
        helper_process_start_time_utc = $Registration.helper_process_start_time_utc
        helper_session_id = $Registration.helper_session_id
        recovery_required = $RecoveryRequired
    }
}

function Register-Controller {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][Guid] $InstallationId,
        [Parameter(Mandatory = $true)][Guid] $LedgerId,
        [Parameter(Mandatory = $true)][string] $DistroName,
        [Parameter(Mandatory = $true)][bool] $Pristine
    )

    Assert-LeaseHeld
    if ($InstallationId -eq [Guid]::Empty -or $LedgerId -eq [Guid]::Empty) {
        Throw-HostLeaseError 'controller_identity_invalid' 'Controller installation and ledger IDs must be nonzero UUIDs' $null
    }
    $distroId = Get-LxssDistroId $DistroName
    $registration = Read-ControllerRegistration
    if ($null -eq $registration) {
        if ($script:LeaseAbandoned) {
            Throw-HostLeaseError 'controller_recovery_required' 'An abandoned Windows lease cannot enroll a missing controller registration' $null
        }
        if (-not $Pristine) {
            Throw-HostLeaseError 'controller_registration_missing' 'Initialized Linux state has no Windows controller registration; recovery is required' $null
        }
        $registration = [pscustomobject] @{
            version = 1
            distro_id = $distroId
            installation_id = $InstallationId.ToString()
            ledger_id = $LedgerId.ToString()
            active_run_id = $null
            helper_process_id = $null
            helper_process_start_time_utc = $null
            helper_session_id = $null
        }
        Write-ControllerRegistration $registration
        return Copy-RegistrationResult $registration $false
    }
    if (
        $registration.distro_id -cne $distroId -or
        $registration.installation_id -cne $InstallationId.ToString() -or
        $registration.ledger_id -cne $LedgerId.ToString()
    ) {
        Throw-HostLeaseError 'controller_identity_mismatch' 'Windows controller registration belongs to a different WSL distribution, installation, or ledger' $null
    }
    $recoveryRequired = $script:LeaseAbandoned -or $null -ne $registration.active_run_id
    return Copy-RegistrationResult $registration $recoveryRequired
}

function Set-ActiveRun {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string] $RunId,
        [Parameter(Mandatory = $true)][string] $SessionId
    )

    Assert-LeaseHeld
    if ([string]::IsNullOrWhiteSpace($RunId)) {
        Throw-HostLeaseError 'active_run_invalid' 'Active run ID is missing' $null
    }
    $canonicalSessionId = ConvertTo-CanonicalGuid $SessionId 'controller_session_invalid' 'Helper session ID'
    if ($canonicalSessionId -cne $script:LeaseSessionId) {
        Throw-HostLeaseError 'controller_session_mismatch' 'Active run session ID does not match the held Windows lease session' $null
    }
    $registration = Read-ControllerRegistration
    if ($null -eq $registration) {
        Throw-HostLeaseError 'controller_registration_missing' 'Cannot bind an active run without a controller registration' $null
    }
    $identity = Get-CurrentProcessIdentity
    if ($null -ne $registration.active_run_id) {
        if (
            [string] $registration.active_run_id -ceq $RunId -and
            [int] $registration.helper_process_id -eq $identity.process_id -and
            [string] $registration.helper_process_start_time_utc -ceq $identity.process_start_time_utc -and
            [string] $registration.helper_session_id -ceq $canonicalSessionId
        ) {
            return Copy-RegistrationResult $registration $false
        }
        Throw-HostLeaseError 'active_run_recovery_required' "Controller registration retains active run $($registration.active_run_id); recovery is required" $null
    }
    if ($script:LeaseAbandoned) {
        Throw-HostLeaseError 'controller_recovery_required' 'An abandoned Windows lease requires recovery inspection before binding a run' $null
    }
    $registration.active_run_id = $RunId
    $registration.helper_process_id = $identity.process_id
    $registration.helper_process_start_time_utc = $identity.process_start_time_utc
    $registration.helper_session_id = $canonicalSessionId
    Write-ControllerRegistration $registration
    return Copy-RegistrationResult $registration $false
}

function Test-FormerProcessIdentityGone {
    param([int] $ProcessId, [string] $StartTimeUtc)
    $process = $null
    try {
        $process = [Diagnostics.Process]::GetProcessById($ProcessId)
    }
    catch [ArgumentException] {
        return $true
    }
    catch {
        Throw-HostLeaseError 'controller_helper_identity_unknown' "Cannot inspect former Windows helper process $ProcessId" $_.Exception
    }
    try {
        try {
            $actualStart = $process.StartTime.ToUniversalTime().ToString('o')
        }
        catch {
            Throw-HostLeaseError 'controller_helper_identity_unknown' "Cannot read creation time for former Windows helper process $ProcessId" $_.Exception
        }
        if ($actualStart -cne $StartTimeUtc) {
            return $true
        }
        if ($process.HasExited) { return $true }
        return $false
    }
    finally {
        if ($null -ne $process) { $process.Dispose() }
    }
}

function Get-StreamSha256 {
    param([IO.Stream] $Stream)
    $algorithm = [Security.Cryptography.SHA256]::Create()
    try {
        return ([BitConverter]::ToString($algorithm.ComputeHash($Stream))).Replace('-', '').ToLowerInvariant()
    }
    finally {
        $algorithm.Dispose()
    }
}

function Confirm-RunRecovery {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string] $RunId,
        [Parameter(Mandatory = $true)][object] $Publication
    )

    Assert-LeaseHeld
    $registration = Read-ControllerRegistration
    if ($null -eq $registration -or [string] $registration.active_run_id -cne $RunId) {
        Throw-HostLeaseError 'active_run_mismatch' 'Recovery run does not match the durable Windows active run' $null
    }
    $current = Get-CurrentProcessIdentity
    $sameHelper = (
        [int] $registration.helper_process_id -eq $current.process_id -and
        [string] $registration.helper_process_start_time_utc -ceq $current.process_start_time_utc -and
        [string] $registration.helper_session_id -ceq $script:LeaseSessionId
    )
    if (-not $sameHelper) {
        if (-not (Test-FormerProcessIdentityGone ([int] $registration.helper_process_id) ([string] $registration.helper_process_start_time_utc))) {
            Throw-HostLeaseError 'controller_helper_still_running' 'Former Windows helper process identity is still live; recovery traffic remains blocked' $null
        }
    }
    foreach ($property in @('native_path', 'filename', 'sha256')) {
        if (-not (Test-ObjectProperty $Publication $property)) {
            Throw-HostLeaseError 'recovery_publication_invalid' "Recovery publication is missing $property" $null
        }
    }
    $expectedFilename = "drone-sim-$RunId.json"
    if ([string] $Publication.filename -cne $expectedFilename) {
        Throw-HostLeaseError 'recovery_publication_mismatch' "Recovery publication filename must be $expectedFilename" $null
    }
    $nativePath = [IO.Path]::GetFullPath([string] $Publication.native_path)
    if ([IO.Path]::GetFileName($nativePath) -cne $expectedFilename -or -not [DroneSim.WindowsFileSystemNative]::PathExistsNoFollow($nativePath)) {
        Throw-HostLeaseError 'recovery_publication_mismatch' 'Recovery publication native path is missing or does not match its run filename' $null
    }
    $expectedHash = ([string] $Publication.sha256).ToLowerInvariant()
    $publicationDirectoryLock = $null
    $publicationStream = $null
    try {
        $publicationDirectoryLock = [DroneSim.WindowsFileSystemNative]::LockDirectoryPath([IO.Path]::GetDirectoryName($nativePath))
        $publicationStream = [DroneSim.WindowsFileSystemNative]::OpenRegularReadLocked($nativePath)
        if ($expectedHash -notmatch '^[0-9a-f]{64}$' -or (Get-StreamSha256 $publicationStream) -cne $expectedHash) {
            Throw-HostLeaseError 'recovery_publication_mismatch' 'Recovery publication bytes do not match the immutable receipt' $null
        }
    }
    catch {
        if ($_.Exception.Data['code']) { throw }
        Throw-HostLeaseError 'recovery_publication_mismatch' 'Recovery publication cannot be verified through no-follow native handles' $_.Exception
    }
    finally {
        if ($null -ne $publicationStream) { $publicationStream.Dispose() }
        if ($null -ne $publicationDirectoryLock) { $publicationDirectoryLock.Dispose() }
    }
    $registration.helper_process_id = $current.process_id
    $registration.helper_process_start_time_utc = $current.process_start_time_utc
    $registration.helper_session_id = $script:LeaseSessionId
    Write-ControllerRegistration $registration
    return Copy-RegistrationResult $registration $false
}

function Complete-ActiveRun {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string] $RunId,
        [Parameter(Mandatory = $true)][bool] $VerifiedCleanup
    )

    Assert-LeaseHeld
    $registration = Read-ControllerRegistration
    if ($null -eq $registration -or [string] $registration.active_run_id -cne $RunId) {
        Throw-HostLeaseError 'active_run_mismatch' 'Cleanup run does not match the durable Windows active run' $null
    }
    if (-not $VerifiedCleanup) {
        Throw-HostLeaseError 'active_run_cleanup_unverified' 'Windows active ownership remains until exact run cleanup is verified' $null
    }
    $registration.active_run_id = $null
    $registration.helper_process_id = $null
    $registration.helper_process_start_time_utc = $null
    $registration.helper_session_id = $null
    Write-ControllerRegistration $registration
    return Copy-RegistrationResult $registration $false
}

Export-ModuleMember -Function Enter-ControllerLease, Register-Controller, Set-ActiveRun, Confirm-RunRecovery, Complete-ActiveRun, Exit-ControllerLease
