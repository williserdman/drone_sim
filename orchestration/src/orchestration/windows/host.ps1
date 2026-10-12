[CmdletBinding()]
param([Parameter(Mandatory = $true)][string] $SessionId)

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$WarningPreference = 'SilentlyContinue'
$VerbosePreference = 'SilentlyContinue'
$InformationPreference = 'SilentlyContinue'
Set-StrictMode -Version 2.0

$script:ProtocolVersion = 1
$script:HelperVersion = '1'
$script:MaxFrameBytes = 131072
$script:MaxDatagramBytes = 65507
$script:QgcPeerAddress = [Net.IPAddress]::Parse('127.0.0.1')
$script:QgcPeerPort = 14550
$script:LeaseHeld = $false
$script:BoundRunId = $null
$script:PublishedReceipt = $null
$script:Transport = $null
$script:ShutdownRequested = $false

function Throw-ProtocolError {
    param([string] $Message)
    throw (New-Object InvalidDataException($Message))
}

function ConvertTo-CanonicalGuidText {
    param([string] $Value, [string] $Label)
    $parsed = [Guid]::Empty
    if (-not [Guid]::TryParse($Value, [ref] $parsed) -or $parsed -eq [Guid]::Empty -or $parsed.ToString() -cne $Value) {
        Throw-ProtocolError "$Label must be a canonical lowercase nonzero UUID"
    }
    return $Value
}

function Assert-ExactProperties {
    param([object] $Value, [string[]] $Names, [string] $Label)
    if ($null -eq $Value -or $Value -isnot [psobject]) {
        Throw-ProtocolError "$Label must be an object"
    }
    $actual = @($Value.PSObject.Properties | ForEach-Object { $_.Name } | Sort-Object)
    $expected = @($Names | Sort-Object)
    if ($actual.Count -ne $expected.Count) {
        Throw-ProtocolError "$Label has unknown or missing fields"
    }
    for ($index = 0; $index -lt $actual.Count; $index += 1) {
        if ($actual[$index] -cne $expected[$index]) {
            Throw-ProtocolError "$Label has unknown or missing fields"
        }
    }
}

function Assert-StringProperty {
    param([object] $Value, [string] $Name, [string] $Label, [bool] $AllowEmpty = $false)
    $selected = $Value.$Name
    if ($selected -isnot [string] -or (-not $AllowEmpty -and [string]::IsNullOrWhiteSpace($selected))) {
        Throw-ProtocolError "$Label field $Name must be a string"
    }
}

function Assert-BooleanProperty {
    param([object] $Value, [string] $Name, [string] $Label)
    if ($Value.$Name -isnot [bool]) {
        Throw-ProtocolError "$Label field $Name must be boolean"
    }
}

function Assert-Publication {
    param([object] $Publication, [string] $Label)
    Assert-ExactProperties $Publication @('native_path', 'filename', 'sha256') $Label
    Assert-StringProperty $Publication 'native_path' $Label
    Assert-StringProperty $Publication 'filename' $Label
    Assert-StringProperty $Publication 'sha256' $Label
    if ([string] $Publication.sha256 -cnotmatch '^[0-9a-f]{64}$') {
        Throw-ProtocolError "$Label field sha256 must be lowercase SHA-256 text"
    }
}

function Assert-Installation {
    param([object] $Installation, [string] $Label)
    Assert-ExactProperties $Installation @('executable', 'settings_file', 'save_path', 'version', 'source') $Label
    foreach ($field in @('executable', 'settings_file', 'save_path', 'version')) {
        Assert-StringProperty $Installation $field $Label
    }
    Assert-ExactProperties $Installation.source @('system_id', 'component_id') "$Label source"
    if (
        ($Installation.source.system_id -isnot [int] -and $Installation.source.system_id -isnot [long]) -or
        ($Installation.source.component_id -isnot [int] -and $Installation.source.component_id -isnot [long])
    ) {
        Throw-ProtocolError "$Label source IDs must be integers"
    }
}

function Assert-RequestPayload {
    param([string] $Operation, [object] $Payload)
    switch ($Operation) {
        'probe' { Assert-ExactProperties $Payload @() 'probe payload' }
        'inspect_qgc' {
            Assert-ExactProperties $Payload @('executable', 'settings_file') 'inspect_qgc payload'
            Assert-StringProperty $Payload 'executable' 'inspect_qgc payload' $true
            Assert-StringProperty $Payload 'settings_file' 'inspect_qgc payload' $true
        }
        'acquire_lease' { Assert-ExactProperties $Payload @() 'acquire_lease payload' }
        'register_controller' {
            Assert-ExactProperties $Payload @('installation_id', 'ledger_id', 'distro_name', 'pristine') 'register_controller payload'
            Assert-StringProperty $Payload 'installation_id' 'register_controller payload'
            Assert-StringProperty $Payload 'ledger_id' 'register_controller payload'
            Assert-StringProperty $Payload 'distro_name' 'register_controller payload'
            Assert-BooleanProperty $Payload 'pristine' 'register_controller payload'
            ConvertTo-CanonicalGuidText ([string] $Payload.installation_id) 'installation ID' | Out-Null
            ConvertTo-CanonicalGuidText ([string] $Payload.ledger_id) 'ledger ID' | Out-Null
        }
        'begin_run' {
            Assert-ExactProperties $Payload @('run_id') 'begin_run payload'
            Assert-StringProperty $Payload 'run_id' 'begin_run payload'
            ConvertTo-CanonicalGuidText ([string] $Payload.run_id) 'run ID' | Out-Null
        }
        'verify_resume' {
            Assert-ExactProperties $Payload @('run_id', 'publication') 'verify_resume payload'
            Assert-StringProperty $Payload 'run_id' 'verify_resume payload'
            ConvertTo-CanonicalGuidText ([string] $Payload.run_id) 'run ID' | Out-Null
            Assert-Publication $Payload.publication 'verify_resume publication'
        }
        'finish_run' {
            Assert-ExactProperties $Payload @('run_id', 'verified_cleanup') 'finish_run payload'
            Assert-StringProperty $Payload 'run_id' 'finish_run payload'
            ConvertTo-CanonicalGuidText ([string] $Payload.run_id) 'run ID' | Out-Null
            Assert-BooleanProperty $Payload 'verified_cleanup' 'finish_run payload'
        }
        'publish_actions' {
            Assert-ExactProperties $Payload @('directory', 'filename', 'data_base64', 'sha256') 'publish_actions payload'
            foreach ($field in @('directory', 'filename', 'data_base64', 'sha256')) {
                Assert-StringProperty $Payload $field 'publish_actions payload'
            }
            if ([string] $Payload.sha256 -cnotmatch '^[0-9a-f]{64}$') {
                Throw-ProtocolError 'publish_actions payload SHA-256 is invalid'
            }
        }
        'open_qgc' {
            Assert-ExactProperties $Payload @('installation') 'open_qgc payload'
            Assert-Installation $Payload.installation 'open_qgc installation'
        }
        'open_browser' {
            Assert-ExactProperties $Payload @('url') 'open_browser payload'
            Assert-StringProperty $Payload 'url' 'open_browser payload'
        }
        'open_transport' {
            Assert-ExactProperties $Payload @('run_id', 'publication') 'open_transport payload'
            Assert-StringProperty $Payload 'run_id' 'open_transport payload'
            ConvertTo-CanonicalGuidText ([string] $Payload.run_id) 'run ID' | Out-Null
            Assert-Publication $Payload.publication 'open_transport publication'
        }
        'close_transport' { Assert-ExactProperties $Payload @() 'close_transport payload' }
        'release_lease' { Assert-ExactProperties $Payload @() 'release_lease payload' }
        'shutdown' { Assert-ExactProperties $Payload @() 'shutdown payload' }
        default { Throw-ProtocolError "Helper operation is not allowed: $Operation" }
    }
}

function Write-WireFrame {
    param([object] $Value)
    $json = ConvertTo-Json $Value -Compress -Depth 8
    $bytes = (New-Object Text.UTF8Encoding($false, $true)).GetBytes($json)
    if ($bytes.Length -gt $script:MaxFrameBytes) {
        Throw-ProtocolError 'Outbound helper protocol line exceeds 128 KiB'
    }
    $output = [Console]::OpenStandardOutput()
    $output.Write($bytes, 0, $bytes.Length)
    $output.WriteByte(10)
    $output.Flush()
}

function Write-Success {
    param([string] $RequestId, [object] $Result)
    Write-WireFrame ([ordered] @{
        version = $script:ProtocolVersion
        kind = 'response'
        session_id = $SessionId
        request_id = $RequestId
        ok = $true
        result = $Result
    })
}

function Write-Failure {
    param([string] $RequestId, [Exception] $Exception)
    $code = 'helper_operation_failed'
    if ($null -ne $Exception.Data['code'] -and -not [string]::IsNullOrWhiteSpace([string] $Exception.Data['code'])) {
        $code = [string] $Exception.Data['code']
    }
    Write-WireFrame ([ordered] @{
        version = $script:ProtocolVersion
        kind = 'response'
        session_id = $SessionId
        request_id = $RequestId
        ok = $false
        error = [ordered] @{ code = $code; message = $Exception.Message }
    })
}

function Close-DatagramTransport {
    if ($null -ne $script:Transport) {
        $script:Transport.Dispose()
        $script:Transport = $null
    }
}

function Invoke-Request {
    param([string] $Operation, [object] $Payload, [string] $SourceDigest)
    switch ($Operation) {
        'probe' {
            return [ordered] @{ helper_version = $script:HelperVersion; source_sha256 = $SourceDigest }
        }
        'inspect_qgc' {
            $executable = [string] $Payload.executable
            $settingsFile = [string] $Payload.settings_file
            if ([string]::IsNullOrWhiteSpace($executable)) {
                $executable = Join-Path $env:ProgramFiles 'QGroundControl\bin\QGroundControl.exe'
            }
            if ([string]::IsNullOrWhiteSpace($settingsFile)) {
                $settingsFile = Join-Path $env:APPDATA 'QGroundControl\QGroundControl.ini'
            }
            return Get-QgcInstallation -Executable $executable -SettingsFile $settingsFile
        }
        'acquire_lease' {
            $result = Enter-ControllerLease -SessionId $SessionId
            $script:LeaseHeld = $true
            return $result
        }
        'register_controller' {
            return Register-Controller -InstallationId ([Guid] $Payload.installation_id) -LedgerId ([Guid] $Payload.ledger_id) -DistroName ([string] $Payload.distro_name) -Pristine ([bool] $Payload.pristine)
        }
        'begin_run' {
            $result = Set-ActiveRun -RunId ([string] $Payload.run_id) -SessionId $SessionId
            $script:BoundRunId = [string] $Payload.run_id
            $script:PublishedReceipt = $null
            return $result
        }
        'verify_resume' {
            $result = Confirm-RunRecovery -RunId ([string] $Payload.run_id) -Publication $Payload.publication
            $script:BoundRunId = [string] $Payload.run_id
            $script:PublishedReceipt = $Payload.publication
            return $result
        }
        'finish_run' {
            if ($null -ne $script:Transport) {
                Throw-ProtocolError 'Datagram transport must close before verified run cleanup'
            }
            $result = Complete-ActiveRun -RunId ([string] $Payload.run_id) -VerifiedCleanup ([bool] $Payload.verified_cleanup)
            $script:BoundRunId = $null
            $script:PublishedReceipt = $null
            return $result
        }
        'publish_actions' {
            if (-not $script:LeaseHeld -or $null -eq $script:BoundRunId) {
                Throw-ProtocolError 'Action publication requires an active run under the held Windows lease'
            }
            if ([string] $Payload.filename -cne "drone-sim-$($script:BoundRunId).json") {
                Throw-ProtocolError 'Action publication filename does not match the active run'
            }
            $bytes = [Convert]::FromBase64String([string] $Payload.data_base64)
            $receipt = Publish-QgcActions -Directory ([string] $Payload.directory) -Filename ([string] $Payload.filename) -Bytes $bytes -Sha256 ([string] $Payload.sha256)
            $script:PublishedReceipt = $receipt
            return $receipt
        }
        'open_qgc' {
            $opened = Open-Qgc -Installation $Payload.installation
            return [ordered] @{ opened = [bool] $opened.opened; detail = [string] $opened.detail }
        }
        'open_browser' {
            $uri = $null
            if (-not [Uri]::TryCreate([string] $Payload.url, [UriKind]::Absolute, [ref] $uri) -or ($uri.Scheme -ne 'http' -and $uri.Scheme -ne 'https')) {
                Throw-ProtocolError 'Browser URL must be absolute HTTP or HTTPS'
            }
            $start = New-Object Diagnostics.ProcessStartInfo
            $start.FileName = $uri.AbsoluteUri
            $start.UseShellExecute = $true
            $process = [Diagnostics.Process]::Start($start)
            if ($null -ne $process) { $process.Dispose() }
            return [ordered] @{ opened = $true; detail = 'browser open request completed' }
        }
        'open_transport' {
            if (-not $script:LeaseHeld -or $script:BoundRunId -cne [string] $Payload.run_id) {
                Throw-ProtocolError 'Datagram transport requires the matching active or recovered run under the held lease'
            }
            if (
                $null -eq $script:PublishedReceipt -or
                [string] $script:PublishedReceipt.native_path -cne [string] $Payload.publication.native_path -or
                [string] $script:PublishedReceipt.filename -cne [string] $Payload.publication.filename -or
                [string] $script:PublishedReceipt.sha256 -cne [string] $Payload.publication.sha256
            ) {
                Throw-ProtocolError 'Datagram transport publication does not match the verified run publication'
            }
            if ($null -ne $script:Transport) {
                Throw-ProtocolError 'Datagram transport is already open'
            }
            $binding = New-Object Net.IPEndPoint($script:QgcPeerAddress, 0)
            $client = New-Object Net.Sockets.UdpClient($binding)
            try {
                $client.Connect($script:QgcPeerAddress, $script:QgcPeerPort)
                $script:Transport = $client
            }
            catch {
                $client.Dispose()
                throw
            }
            $local = [Net.IPEndPoint] $client.Client.LocalEndPoint
            return [ordered] @{ local_port = $local.Port; peer = '127.0.0.1:14550' }
        }
        'close_transport' {
            Close-DatagramTransport
            return [ordered] @{}
        }
        'release_lease' {
            if ($null -ne $script:Transport) {
                Throw-ProtocolError 'Datagram transport must close before releasing the Windows lease'
            }
            Exit-ControllerLease
            $script:LeaseHeld = $false
            $script:BoundRunId = $null
            $script:PublishedReceipt = $null
            return [ordered] @{}
        }
        'shutdown' {
            Close-DatagramTransport
            if ($script:LeaseHeld) {
                Exit-ControllerLease
                $script:LeaseHeld = $false
                $script:BoundRunId = $null
                $script:PublishedReceipt = $null
            }
            $script:ShutdownRequested = $true
            return [ordered] @{}
        }
    }
}

function Invoke-WireFrame {
    param([byte[]] $Bytes, [string] $SourceDigest)
    if ($Bytes.Length -gt $script:MaxFrameBytes) {
        Throw-ProtocolError 'Inbound helper protocol line exceeds 128 KiB'
    }
    if ($Bytes.Length -gt 0 -and $Bytes[$Bytes.Length - 1] -eq 13) {
        $trimmed = New-Object byte[] ($Bytes.Length - 1)
        [Array]::Copy($Bytes, $trimmed, $trimmed.Length)
        $Bytes = $trimmed
    }
    try {
        $text = (New-Object Text.UTF8Encoding($false, $true)).GetString($Bytes)
        $frame = ConvertFrom-Json $text
    }
    catch {
        Throw-ProtocolError "Inbound helper protocol line is not valid UTF-8 JSON: $($_.Exception.Message)"
    }
    if ($null -eq $frame -or $frame -isnot [psobject]) {
        Throw-ProtocolError 'Inbound helper protocol line must contain an object'
    }
    if (
        ($frame.version -isnot [int] -and $frame.version -isnot [long]) -or
        [long] $frame.version -ne $script:ProtocolVersion -or
        $frame.session_id -isnot [string] -or
        [string] $frame.session_id -cne $SessionId
    ) {
        Throw-ProtocolError 'Inbound helper protocol version or session ID mismatched'
    }
    if ([string] $frame.kind -ceq 'request') {
        Assert-ExactProperties $frame @('version', 'kind', 'session_id', 'request_id', 'operation', 'payload') 'request envelope'
        Assert-StringProperty $frame 'request_id' 'request envelope'
        Assert-StringProperty $frame 'operation' 'request envelope'
        ConvertTo-CanonicalGuidText ([string] $frame.request_id) 'request ID' | Out-Null
        $requestId = [string] $frame.request_id
        Assert-RequestPayload ([string] $frame.operation) $frame.payload
        try {
            $result = Invoke-Request ([string] $frame.operation) $frame.payload $SourceDigest
            Write-Success $requestId $result
        }
        catch {
            Write-Failure $requestId $_.Exception
        }
        return
    }
    if ([string] $frame.kind -ceq 'datagram') {
        Assert-ExactProperties $frame @('version', 'kind', 'session_id', 'data_base64') 'datagram envelope'
        Assert-StringProperty $frame 'data_base64' 'datagram envelope' $true
        if ($null -eq $script:Transport) {
            Throw-ProtocolError 'Inbound datagram arrived while transport is closed'
        }
        try { $payload = [Convert]::FromBase64String([string] $frame.data_base64) }
        catch { Throw-ProtocolError 'Inbound datagram contains invalid base64' }
        if ($payload.Length -gt $script:MaxDatagramBytes) {
            Throw-ProtocolError 'Inbound datagram exceeds 65,507 bytes'
        }
        $sent = $script:Transport.Send($payload, $payload.Length)
        if ($sent -ne $payload.Length) {
            Throw-ProtocolError 'Windows UDP datagram send was incomplete'
        }
        return
    }
    Throw-ProtocolError 'Inbound helper protocol kind is unknown'
}

$SessionId = ConvertTo-CanonicalGuidText $SessionId 'helper session ID'
$sourceDigest = (Get-FileHash -LiteralPath $MyInvocation.MyCommand.Path -Algorithm SHA256).Hash.ToLowerInvariant()
$filesystemModule = Join-Path $PSScriptRoot 'HostFilesystem.psm1'
$leaseModule = Join-Path $PSScriptRoot 'HostLease.psm1'
Import-Module $filesystemModule -Force | Out-Null
Import-Module $leaseModule -Force | Out-Null

$inputStream = [Console]::OpenStandardInput()
$readBuffer = New-Object byte[] 4096
$lineBuffer = New-Object 'Collections.Generic.List[byte]'
$readTask = $inputStream.ReadAsync($readBuffer, 0, $readBuffer.Length)

try {
    Write-WireFrame ([ordered] @{
        version = $script:ProtocolVersion
        kind = 'status'
        session_id = $SessionId
        state = 'ready'
        helper_version = $script:HelperVersion
        source_sha256 = $sourceDigest
    })

    while (-not $script:ShutdownRequested) {
        if ($readTask.IsCompleted) {
            $count = $readTask.GetAwaiter().GetResult()
            if ($count -eq 0) {
                if ($lineBuffer.Count -ne 0) { Throw-ProtocolError 'Helper stdin ended with a partial protocol line' }
                break
            }
            for ($index = 0; $index -lt $count; $index += 1) {
                $value = $readBuffer[$index]
                if ($value -eq 10) {
                    Invoke-WireFrame $lineBuffer.ToArray() $sourceDigest
                    $lineBuffer.Clear()
                }
                else {
                    if ($lineBuffer.Count -ge $script:MaxFrameBytes) {
                        Throw-ProtocolError 'Inbound helper protocol line exceeds 128 KiB'
                    }
                    $lineBuffer.Add($value)
                }
            }
            $readTask = $inputStream.ReadAsync($readBuffer, 0, $readBuffer.Length)
        }

        if ($null -ne $script:Transport) {
            $drained = 0
            while ($script:Transport.Client.Available -gt 0 -and $drained -lt 32) {
                $peer = New-Object Net.IPEndPoint([Net.IPAddress]::Any, 0)
                $payload = $script:Transport.Receive([ref] $peer)
                $drained += 1
                if (-not $peer.Address.Equals($script:QgcPeerAddress) -or $peer.Port -ne $script:QgcPeerPort) {
                    continue
                }
                if ($payload.Length -gt $script:MaxDatagramBytes) {
                    Throw-ProtocolError 'Windows UDP datagram exceeds 65,507 bytes'
                }
                Write-WireFrame ([ordered] @{
                    version = $script:ProtocolVersion
                    kind = 'datagram'
                    session_id = $SessionId
                    data_base64 = [Convert]::ToBase64String($payload)
                })
            }
        }
        [Threading.Thread]::Sleep(2)
    }
}
catch {
    [Console]::Error.WriteLine($_.Exception.Message)
    exit 1
}
finally {
    Close-DatagramTransport
    if ($script:LeaseHeld) {
        try { Exit-ControllerLease } catch { [Console]::Error.WriteLine($_.Exception.Message) }
        $script:LeaseHeld = $false
    }
}
