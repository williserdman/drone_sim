[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string] $TestId,
    [Parameter(Mandatory = $true)][string] $DistroName
)

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
Set-StrictMode -Version 2.0
$utf8 = New-Object Text.UTF8Encoding($false, $true)
[Console]::InputEncoding = $utf8
[Console]::OutputEncoding = $utf8

function Write-WireValue {
    param([object] $Value)
    [Console]::Out.WriteLine((ConvertTo-Json $Value -Compress -Depth 6))
    [Console]::Out.Flush()
}

function Write-WireResponse {
    param([string] $RequestId, [bool] $Ok, [object] $Result, [string] $Message = '')
    if ($Ok) {
        Write-WireValue ([ordered] @{ request_id = $RequestId; ok = $true; result = $Result })
    }
    else {
        Write-WireValue ([ordered] @{ request_id = $RequestId; ok = $false; error = $Message })
    }
}

if ($TestId -notmatch '^[a-f0-9]{32}$') {
    throw 'TestId must be 32 lowercase hexadecimal characters'
}
if ([string]::IsNullOrWhiteSpace($DistroName)) {
    throw 'DistroName is required'
}

$tempRoot = Join-Path ([IO.Path]::GetTempPath()) "drone-sim-wsl-integration-$TestId"
$registryRoot = "HKCU:\Software\drone-sim-wsl-integration-tests\$TestId"
$lxssRoot = Join-Path $registryRoot 'Lxss'
$distroId = [Guid]::NewGuid()
$distroKey = Join-Path $lxssRoot ('{' + $distroId.ToString() + '}')
$savePath = Join-Path $tempRoot 'QGC'
$actionsDirectory = Join-Path $savePath 'MavlinkActions'
$trusted = $null
$spoof = $null
$helperPeer = $null

try {
    New-Item -ItemType Directory -Path $actionsDirectory -Force | Out-Null
    $volume = Get-Volume -FilePath $tempRoot
    if ($volume.FileSystemType -ne 'NTFS' -or $volume.DriveType -ne 'Fixed') {
        throw "Integration fixture requires local fixed NTFS, got $($volume.FileSystemType)/$($volume.DriveType)"
    }
    New-Item -Path $distroKey -Force | Out-Null
    New-ItemProperty -Path $distroKey -Name DistributionName -Value $DistroName -PropertyType String -Force | Out-Null

    $trusted = New-Object Net.Sockets.UdpClient((New-Object Net.IPEndPoint([Net.IPAddress]::Loopback, 14550)))
    $trusted.Client.ReceiveTimeout = 10000
    $spoof = New-Object Net.Sockets.UdpClient((New-Object Net.IPEndPoint([Net.IPAddress]::Loopback, 0)))

    Write-WireValue ([ordered] @{
        state = 'ready'
        powershell_version = $PSVersionTable.PSVersion.ToString()
        filesystem = [string] $volume.FileSystemType
        temp_root = $tempRoot
        registry_root = $lxssRoot
        save_path = $savePath
    })

    while ($true) {
        $line = [Console]::In.ReadLine()
        if ($null -eq $line) { break }
        $request = $line | ConvertFrom-Json
        $requestId = [string] $request.request_id
        try {
            switch ([string] $request.operation) {
                'receive' {
                    $sender = New-Object Net.IPEndPoint([Net.IPAddress]::Any, 0)
                    $bytes = $trusted.Receive([ref] $sender)
                    $helperPeer = $sender
                    Write-WireResponse $requestId $true ([ordered] @{
                        data_base64 = [Convert]::ToBase64String($bytes)
                        sender_address = $sender.Address.ToString()
                        sender_port = $sender.Port
                    })
                }
                'send' {
                    if ($null -eq $helperPeer) { throw 'Helper peer is unknown until receive succeeds' }
                    $bytes = [Convert]::FromBase64String([string] $request.data_base64)
                    $trusted.Send($bytes, $bytes.Length, $helperPeer) | Out-Null
                    Write-WireResponse $requestId $true ([ordered] @{})
                }
                'spoof' {
                    if ($null -eq $helperPeer) { throw 'Helper peer is unknown until receive succeeds' }
                    $bytes = [Convert]::FromBase64String([string] $request.data_base64)
                    $spoof.Send($bytes, $bytes.Length, $helperPeer) | Out-Null
                    Write-WireResponse $requestId $true ([ordered] @{})
                }
                'terminate_process' {
                    $processId = [int] $request.process_id
                    $expectedStart = [string] $request.process_start_time_utc
                    $process = [Diagnostics.Process]::GetProcessById($processId)
                    try {
                        $actualStart = $process.StartTime.ToUniversalTime().ToString('o')
                        if ($actualStart -cne $expectedStart) {
                            throw "Refusing to terminate reused Windows process ID $processId"
                        }
                        $process.Kill()
                        if (-not $process.WaitForExit(10000)) {
                            throw "Windows helper process $processId did not terminate"
                        }
                    }
                    finally {
                        $process.Dispose()
                    }
                    Write-WireResponse $requestId $true ([ordered] @{})
                }
                'shutdown' {
                    Write-WireResponse $requestId $true ([ordered] @{})
                    return
                }
                default { throw "Unknown fixture operation: $($request.operation)" }
            }
        }
        catch {
            Write-WireResponse $requestId $false $null $_.Exception.Message
        }
    }
}
finally {
    if ($null -ne $spoof) { $spoof.Dispose() }
    if ($null -ne $trusted) { $trusted.Dispose() }
    Remove-Item -LiteralPath $registryRoot -Recurse -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $tempRoot -Recurse -Force -ErrorAction SilentlyContinue
}
