$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 2.0

$script:TestFault = ''
$script:ExpectedQgcVersion = '5.0.8'

if (-not ('DroneSim.WindowsFileSystemNative' -as [type])) {
    Add-Type -TypeDefinition @'
using System;
using System.ComponentModel;
using System.IO;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
using System.Text;

namespace DroneSim {
    public static class WindowsFileSystemNative {
        private const uint FILE_READ_ATTRIBUTES = 0x80;
        private const uint GENERIC_READ = 0x80000000;
        private const uint FILE_SHARE_READ = 0x1;
        private const uint FILE_SHARE_WRITE = 0x2;
        private const uint OPEN_EXISTING = 3;
        private const uint FILE_FLAG_BACKUP_SEMANTICS = 0x02000000;
        private const uint FILE_FLAG_OPEN_REPARSE_POINT = 0x00200000;
        private const uint FILE_ATTRIBUTE_REPARSE_POINT = 0x400;
        private const uint FILE_ATTRIBUTE_DIRECTORY = 0x10;
        private const uint MOVEFILE_WRITE_THROUGH = 0x8;
        private const uint DRIVE_FIXED = 3;
        private const int FileAttributeTagInfo = 9;
        private const uint INVALID_FILE_ATTRIBUTES = 0xFFFFFFFF;

        [StructLayout(LayoutKind.Sequential)]
        private struct FILE_ATTRIBUTE_TAG_INFO {
            public uint FileAttributes;
            public uint ReparseTag;
        }

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern SafeFileHandle CreateFileW(
            string fileName,
            uint desiredAccess,
            uint shareMode,
            IntPtr securityAttributes,
            uint creationDisposition,
            uint flagsAndAttributes,
            IntPtr templateFile);

        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool GetFileInformationByHandleEx(
            SafeFileHandle file,
            int infoClass,
            out FILE_ATTRIBUTE_TAG_INFO info,
            uint size);

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool GetVolumePathNameW(
            string fileName,
            StringBuilder volumePathName,
            uint bufferLength);

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode)]
        private static extern uint GetDriveTypeW(string rootPathName);

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool GetVolumeInformationW(
            string rootPathName,
            StringBuilder volumeName,
            uint volumeNameSize,
            out uint serialNumber,
            out uint maximumComponentLength,
            out uint fileSystemFlags,
            StringBuilder fileSystemName,
            uint fileSystemNameSize);

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool MoveFileExW(string existingName, string newName, uint flags);

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern uint GetFileAttributesW(string fileName);

        [DllImport("user32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool SetForegroundWindow(IntPtr windowHandle);

        [DllImport("user32.dll")]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool ShowWindowAsync(IntPtr windowHandle, int command);

        public sealed class LockedPath : IDisposable {
            private List<SafeFileHandle> handles;

            internal LockedPath(List<SafeFileHandle> handles) {
                this.handles = handles;
            }

            public void Dispose() {
                if (handles == null) return;
                for (int index = handles.Count - 1; index >= 0; index--) {
                    handles[index].Dispose();
                }
                handles = null;
            }
        }

        private static SafeFileHandle OpenPathComponent(string path, bool requireDirectory) {
            SafeFileHandle handle = CreateFileW(
                path,
                FILE_READ_ATTRIBUTES,
                FILE_SHARE_READ | FILE_SHARE_WRITE,
                IntPtr.Zero,
                OPEN_EXISTING,
                FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OPEN_REPARSE_POINT,
                IntPtr.Zero);
            if (handle.IsInvalid) {
                int error = Marshal.GetLastWin32Error();
                handle.Dispose();
                throw new Win32Exception(error, "Cannot lock path component: " + path);
            }
            try {
                FILE_ATTRIBUTE_TAG_INFO info;
                if (!GetFileInformationByHandleEx(
                    handle,
                    FileAttributeTagInfo,
                    out info,
                    (uint)Marshal.SizeOf(typeof(FILE_ATTRIBUTE_TAG_INFO)))) {
                    throw new Win32Exception(Marshal.GetLastWin32Error(), "Cannot inspect path attributes: " + path);
                }
                if ((info.FileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) != 0) {
                    throw new InvalidOperationException("reparse:" + path);
                }
                if (requireDirectory && (info.FileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0) {
                    throw new InvalidOperationException("notdirectory:" + path);
                }
                return handle;
            } catch {
                handle.Dispose();
                throw;
            }
        }

        private static LockedPath LockPathInternal(string path, bool requireFinalDirectory) {
            string fullPath = Path.GetFullPath(path);
            string root = Path.GetPathRoot(fullPath);
            if (String.IsNullOrEmpty(root)) {
                throw new InvalidOperationException("unrooted:" + path);
            }
            List<SafeFileHandle> handles = new List<SafeFileHandle>();
            try {
                handles.Add(OpenPathComponent(root, true));
                string remainder = fullPath.Substring(root.Length);
                string current = root;
                string[] parts = remainder.Split(new char[] { Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar }, StringSplitOptions.RemoveEmptyEntries);
                for (int index = 0; index < parts.Length; index++) {
                    current = Path.Combine(current, parts[index]);
                    handles.Add(OpenPathComponent(current, index < parts.Length - 1 || requireFinalDirectory));
                }
                return new LockedPath(handles);
            } catch {
                for (int index = handles.Count - 1; index >= 0; index--) handles[index].Dispose();
                throw;
            }
        }

        public static LockedPath LockPath(string path) {
            return LockPathInternal(path, false);
        }

        public static LockedPath LockDirectoryPath(string path) {
            return LockPathInternal(path, true);
        }

        public static bool PathExistsNoFollow(string path) {
            uint attributes = GetFileAttributesW(Path.GetFullPath(path));
            if (attributes != INVALID_FILE_ATTRIBUTES) return true;
            int error = Marshal.GetLastWin32Error();
            if (error == 2 || error == 3) return false;
            throw new Win32Exception(error, "Cannot inspect path existence: " + path);
        }

        public static FileStream OpenRegularReadLocked(string path) {
            SafeFileHandle handle = CreateFileW(
                Path.GetFullPath(path),
                GENERIC_READ | FILE_READ_ATTRIBUTES,
                FILE_SHARE_READ,
                IntPtr.Zero,
                OPEN_EXISTING,
                FILE_FLAG_OPEN_REPARSE_POINT,
                IntPtr.Zero);
            if (handle.IsInvalid) {
                int error = Marshal.GetLastWin32Error();
                handle.Dispose();
                throw new Win32Exception(error, "Cannot open regular file without following reparse points: " + path);
            }
            try {
                FILE_ATTRIBUTE_TAG_INFO info;
                if (!GetFileInformationByHandleEx(handle, FileAttributeTagInfo, out info, (uint)Marshal.SizeOf(typeof(FILE_ATTRIBUTE_TAG_INFO)))) {
                    throw new Win32Exception(Marshal.GetLastWin32Error(), "Cannot inspect regular file handle: " + path);
                }
                if ((info.FileAttributes & (FILE_ATTRIBUTE_REPARSE_POINT | FILE_ATTRIBUTE_DIRECTORY)) != 0) {
                    throw new InvalidOperationException("notregular:" + path);
                }
                return new FileStream(handle, FileAccess.Read);
            } catch {
                handle.Dispose();
                throw;
            }
        }

        public static void ValidateLocalNtfs(string path) {
            StringBuilder volumePath = new StringBuilder(512);
            if (!GetVolumePathNameW(Path.GetFullPath(path), volumePath, (uint)volumePath.Capacity)) {
                throw new Win32Exception(Marshal.GetLastWin32Error(), "Cannot resolve publication volume");
            }
            if (GetDriveTypeW(volumePath.ToString()) != DRIVE_FIXED) {
                throw new InvalidOperationException("nonlocal:" + volumePath);
            }
            StringBuilder volumeName = new StringBuilder(512);
            StringBuilder fileSystemName = new StringBuilder(64);
            uint serialNumber;
            uint maximumComponentLength;
            uint fileSystemFlags;
            if (!GetVolumeInformationW(
                volumePath.ToString(), volumeName, (uint)volumeName.Capacity,
                out serialNumber, out maximumComponentLength, out fileSystemFlags,
                fileSystemName, (uint)fileSystemName.Capacity)) {
                throw new Win32Exception(Marshal.GetLastWin32Error(), "Cannot inspect publication volume");
            }
            if (!String.Equals(fileSystemName.ToString(), "NTFS", StringComparison.OrdinalIgnoreCase)) {
                throw new InvalidOperationException("nonntfs:" + fileSystemName);
            }
        }

        public static void MoveNoReplaceWriteThrough(string source, string destination) {
            if (!MoveFileExW(source, destination, MOVEFILE_WRITE_THROUGH)) {
                throw new Win32Exception(Marshal.GetLastWin32Error(), "MoveFileExW no-replace publication failed");
            }
        }
    }
}
'@
}

function Throw-HostFilesystemError {
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

function Get-BytesSha256 {
    param([byte[]] $Bytes)
    $algorithm = [Security.Cryptography.SHA256]::Create()
    try {
        return ([BitConverter]::ToString($algorithm.ComputeHash($Bytes))).Replace('-', '').ToLowerInvariant()
    }
    finally {
        $algorithm.Dispose()
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

function Read-StableBytes {
    param([string] $Path)
    $stream = New-Object IO.FileStream($Path, [IO.FileMode]::Open, [IO.FileAccess]::Read, [IO.FileShare]::Read)
    try {
        if ($stream.Length -gt [int]::MaxValue) {
            Throw-HostFilesystemError 'qgc_settings_invalid' 'QGroundControl settings file is too large' $null
        }
        $bytes = New-Object byte[] ([int] $stream.Length)
        $offset = 0
        while ($offset -lt $bytes.Length) {
            $read = $stream.Read($bytes, $offset, $bytes.Length - $offset)
            if ($read -eq 0) {
                Throw-HostFilesystemError 'qgc_settings_invalid' 'QGroundControl settings file ended during its read-only snapshot' $null
            }
            $offset += $read
        }
        return ,$bytes
    }
    finally {
        $stream.Dispose()
    }
}

function Read-QgcIni {
    param([byte[]] $Bytes)
    $encoding = New-Object Text.UTF8Encoding($false, $true)
    try {
        $text = $encoding.GetString($Bytes)
    }
    catch {
        Throw-HostFilesystemError 'qgc_settings_invalid' 'QGroundControl settings are not valid UTF-8' $_.Exception
    }
    if ($text.Length -gt 0 -and $text[0] -eq [char] 0xFEFF) {
        $text = $text.Substring(1)
    }
    $values = New-Object 'Collections.Generic.Dictionary[string,string]' ([StringComparer]::Ordinal)
    $section = ''
    foreach ($rawLine in [Text.RegularExpressions.Regex]::Split($text, "`r?`n")) {
        $line = $rawLine.Trim()
        if ($line.Length -eq 0 -or $line.StartsWith(';') -or $line.StartsWith('#')) { continue }
        if ($line.StartsWith('[') -and $line.EndsWith(']')) {
            $section = $line.Substring(1, $line.Length - 2)
            continue
        }
        $separator = $line.IndexOf('=')
        if ($separator -lt 1) { continue }
        $key = $section + '/' + $line.Substring(0, $separator).Trim()
        $value = $line.Substring($separator + 1).Trim()
        if ($values.ContainsKey($key)) {
            Throw-HostFilesystemError 'qgc_settings_invalid' "QGroundControl settings contain duplicate key $key" $null
        }
        $values.Add($key, $value)
    }
    return $values
}

function Get-QgcFileVersion {
    param([string] $Executable)
    $version = [Diagnostics.FileVersionInfo]::GetVersionInfo($Executable)
    $identityValues = @($version.ProductName, $version.FileDescription)
    foreach ($identity in $identityValues) {
        if ($identity -ne 'QGroundControl') {
            Throw-HostFilesystemError 'qgc_executable_identity_mismatch' "Selected executable has unexpected QGroundControl version-resource identity: $identity" $null
        }
    }
    if ($version.InternalName -ne 'QGroundControl' -and $version.InternalName -ne 'QGroundControl.exe') {
        Throw-HostFilesystemError 'qgc_executable_identity_mismatch' "Selected executable has unexpected internal filename: $($version.InternalName)" $null
    }
    if ($version.OriginalFilename -ne 'QGroundControl.exe') {
        Throw-HostFilesystemError 'qgc_executable_identity_mismatch' "Selected executable has unexpected original filename: $($version.OriginalFilename)" $null
    }
    if (
        $version.ProductVersion -ne $script:ExpectedQgcVersion -or
        $version.FileVersion -ne $script:ExpectedQgcVersion -or
        $version.FileMajorPart -ne 5 -or
        $version.FileMinorPart -ne 0 -or
        $version.FileBuildPart -ne 8 -or
        $version.FilePrivatePart -ne 0 -or
        $version.ProductMajorPart -ne 5 -or
        $version.ProductMinorPart -ne 0 -or
        $version.ProductBuildPart -ne 8 -or
        $version.ProductPrivatePart -ne 0
    ) {
        Throw-HostFilesystemError 'qgc_version_mismatch' "QGroundControl $script:ExpectedQgcVersion is required; selected version resources are product=$($version.ProductVersion), file=$($version.FileVersion)" $null
    }
    return $script:ExpectedQgcVersion
}

function Get-RunningQgcProcesses {
    param([string] $Executable)
    $selected = [IO.Path]::GetFullPath($Executable)
    $matching = New-Object 'Collections.Generic.List[Diagnostics.Process]'
    foreach ($process in [Diagnostics.Process]::GetProcessesByName('QGroundControl')) {
        try {
            $runningPath = $process.MainModule.FileName
            if (-not [string]::Equals([IO.Path]::GetFullPath($runningPath), $selected, [StringComparison]::OrdinalIgnoreCase)) {
                Throw-HostFilesystemError 'qgc_running_installation_mismatch' "Running QGroundControl process $($process.Id) uses a different executable: $runningPath" $null
            }
            Get-QgcFileVersion $runningPath | Out-Null
            $matching.Add($process)
        }
        catch {
            if ($_.Exception.Data['code']) { throw }
            Throw-HostFilesystemError 'qgc_running_installation_mismatch' "Cannot establish the executable and version of running QGroundControl process $($process.Id)" $_.Exception
        }
    }
    return ,$matching.ToArray()
}

function Get-QgcInstallation {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string] $Executable,
        [Parameter(Mandatory = $true)][string] $SettingsFile
    )

    $executablePath = [IO.Path]::GetFullPath($Executable)
    $settingsPath = [IO.Path]::GetFullPath($SettingsFile)
    if (-not [IO.File]::Exists($executablePath)) {
        Throw-HostFilesystemError 'qgc_executable_missing' "QGroundControl executable does not exist: $executablePath" $null
    }
    if (-not [IO.File]::Exists($settingsPath)) {
        Throw-HostFilesystemError 'qgc_settings_missing' "QGroundControl settings do not exist: $settingsPath" $null
    }
    $executableLock = $null
    $settingsLock = $null
    try {
        $executableLock = [DroneSim.WindowsFileSystemNative]::LockPath($executablePath)
        $settingsLock = [DroneSim.WindowsFileSystemNative]::LockPath($settingsPath)
    }
    catch {
        if ($null -ne $settingsLock) { $settingsLock.Dispose() }
        if ($null -ne $executableLock) { $executableLock.Dispose() }
        Throw-HostFilesystemError 'qgc_path_unsafe' 'QGroundControl executable and settings must be regular paths without reparse components' $_.Exception
    }
    try {
        $executableAttributes = [IO.File]::GetAttributes($executablePath)
        $settingsAttributes = [IO.File]::GetAttributes($settingsPath)
        if (($executableAttributes -band [IO.FileAttributes]::Directory) -ne 0 -or ($settingsAttributes -band [IO.FileAttributes]::Directory) -ne 0) {
            Throw-HostFilesystemError 'qgc_path_unsafe' 'QGroundControl executable and settings must be regular files' $null
        }
        $version = Get-QgcFileVersion $executablePath
        Get-RunningQgcProcesses $executablePath | Out-Null
        $settingsBytes = Read-StableBytes $settingsPath
        $settings = Read-QgcIni $settingsBytes
        if ($settings.ContainsKey('General/SettingsVersion') -and $settings['General/SettingsVersion'] -ne '9') {
            Throw-HostFilesystemError 'qgc_settings_version_mismatch' "QGroundControl SettingsVersion 9 is required; found $($settings['General/SettingsVersion'])" $null
        }
        if (-not $settings.ContainsKey('General/savePath') -or [string]::IsNullOrWhiteSpace($settings['General/savePath'])) {
            Throw-HostFilesystemError 'qgc_save_path_missing' 'QGroundControl General/savePath is missing' $null
        }
        $savePath = $settings['General/savePath']
        if (-not [IO.Path]::IsPathRooted($savePath)) {
            Throw-HostFilesystemError 'qgc_save_path_invalid' "QGroundControl savePath is not absolute: $savePath" $null
        }
        $systemId = 255
        $systemIdText = $null
        if ($settings.ContainsKey('General/gcsMavlinkSystemID')) {
            $systemIdText = $settings['General/gcsMavlinkSystemID']
        }
        elseif ($settings.ContainsKey('QGC_MAVLINK_PROTOCOL/GCS_SYSTEM_ID')) {
            $systemIdText = $settings['QGC_MAVLINK_PROTOCOL/GCS_SYSTEM_ID']
        }
        if ($null -ne $systemIdText) {
            $parsedSystemId = 0
            if (-not [int]::TryParse($systemIdText, [ref] $parsedSystemId) -or $parsedSystemId -lt 1 -or $parsedSystemId -gt 255) {
                Throw-HostFilesystemError 'qgc_identity_invalid' "QGroundControl MAVLink system ID is invalid: $systemIdText" $null
            }
            $systemId = $parsedSystemId
        }
        if ($systemId -ne 255) {
            Throw-HostFilesystemError 'qgc_identity_mismatch' "QGroundControl MAVLink source must be 255/190; settings select $systemId/190" $null
        }
        return [pscustomobject] @{
            executable = $executablePath
            settings_file = $settingsPath
            save_path = $savePath
            version = $version
            source = [pscustomobject] @{ system_id = 255; component_id = 190 }
        }
    }
    finally {
        $settingsLock.Dispose()
        $executableLock.Dispose()
    }
}

function Test-ExistingPublication {
    param([string] $Target, [string] $ExpectedSha256)
    if (-not [DroneSim.WindowsFileSystemNative]::PathExistsNoFollow($Target)) { return $false }
    $stream = $null
    try {
        $stream = [DroneSim.WindowsFileSystemNative]::OpenRegularReadLocked($Target)
    }
    catch {
        Throw-HostFilesystemError 'unsafe_publication_path' "Existing publication target is unsafe: $Target" $_.Exception
    }
    try {
        $actual = Get-StreamSha256 $stream
        if ($actual -ne $ExpectedSha256) {
            Throw-HostFilesystemError 'publication_conflict' "Existing QGroundControl action file has different bytes: $Target" $null
        }
    }
    finally {
        $stream.Dispose()
    }
    return $true
}

function Publish-QgcActions {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string] $Directory,
        [Parameter(Mandatory = $true)][string] $Filename,
        [Parameter(Mandatory = $true)][byte[]] $Bytes,
        [Parameter(Mandatory = $true)][string] $Sha256
    )

    if ([string]::IsNullOrWhiteSpace($Filename) -or [IO.Path]::GetFileName($Filename) -ne $Filename -or $Filename.IndexOfAny([IO.Path]::GetInvalidFileNameChars()) -ge 0) {
        Throw-HostFilesystemError 'publication_filename_invalid' "QGroundControl action filename must be one safe path component: $Filename" $null
    }
    $expectedHash = $Sha256.ToLowerInvariant()
    if ($expectedHash -notmatch '^[0-9a-f]{64}$' -or (Get-BytesSha256 $Bytes) -ne $expectedHash) {
        Throw-HostFilesystemError 'publication_hash_mismatch' 'Action bytes do not match the supplied SHA-256' $null
    }
    $directoryPath = [IO.Path]::GetFullPath($Directory)
    $parentPath = [IO.Directory]::GetParent($directoryPath)
    if ($null -eq $parentPath -or -not $parentPath.Exists) {
        Throw-HostFilesystemError 'unsafe_publication_path' "Publication directory parent does not exist: $directoryPath" $null
    }
    $parentLock = $null
    $directoryLock = $null
    try {
        $parentLock = [DroneSim.WindowsFileSystemNative]::LockDirectoryPath($parentPath.FullName)
        if (-not [IO.Directory]::Exists($directoryPath)) {
            [IO.Directory]::CreateDirectory($directoryPath) | Out-Null
        }
        $directoryLock = [DroneSim.WindowsFileSystemNative]::LockDirectoryPath($directoryPath)
        [DroneSim.WindowsFileSystemNative]::ValidateLocalNtfs($directoryPath)
        if ($script:TestFault -eq 'unsupported_storage' -or $script:TestFault -eq 'network_storage' -or $script:TestFault -eq 'non_ntfs_storage') {
            Throw-HostFilesystemError 'unsupported_publication_storage' 'Injected unsupported publication storage' $null
        }
    }
    catch {
        if ($null -ne $directoryLock) { $directoryLock.Dispose() }
        if ($null -ne $parentLock) { $parentLock.Dispose() }
        if ($_.Exception.Data['code']) { throw }
        $text = $_.Exception.Message
        if ($text.StartsWith('nonlocal:') -or $text.StartsWith('nonntfs:')) {
            Throw-HostFilesystemError 'unsupported_publication_storage' 'QGroundControl actions must be published to a local NTFS directory' $_.Exception
        }
        Throw-HostFilesystemError 'unsafe_publication_path' "Publication directory contains an unsafe reparse component: $directoryPath" $_.Exception
    }
    $target = Join-Path $directoryPath $Filename
    try {
        if (Test-ExistingPublication $target $expectedHash) {
            $directoryLock.Dispose()
            $directoryLock = $null
            $parentLock.Dispose()
            $parentLock = $null
            return [pscustomobject] @{ native_path = $target; filename = $Filename; sha256 = $expectedHash }
        }
    }
    catch {
        if ($null -ne $directoryLock) { $directoryLock.Dispose(); $directoryLock = $null }
        if ($null -ne $parentLock) { $parentLock.Dispose(); $parentLock = $null }
        throw
    }

    $temporary = Join-Path $directoryPath ($Filename + '.tmp-' + [Guid]::NewGuid().ToString('N'))
    $stream = $null
    $renamed = $false
    try {
        $stream = New-Object IO.FileStream(
            $temporary,
            [IO.FileMode]::CreateNew,
            [IO.FileAccess]::Write,
            [IO.FileShare]::None,
            4096,
            [IO.FileOptions]::WriteThrough)
        $stream.Write($Bytes, 0, $Bytes.Length)
        if ($script:TestFault -eq 'before_flush') {
            Throw-HostFilesystemError 'publication_interrupted' 'Injected interruption before durable flush' $null
        }
        if ($script:TestFault -eq 'flush') {
            Throw-HostFilesystemError 'publication_flush_failed' 'Injected durable flush failure' $null
        }
        $stream.Flush($true)
        $stream.Dispose()
        $stream = $null
        if ($script:TestFault -eq 'before_rename') {
            Throw-HostFilesystemError 'publication_interrupted' 'Injected interruption before publication rename' $null
        }
        if ($script:TestFault -eq 'pause_before_rename') {
            if ($env:DRONE_SIM_WINDOWS_NATIVE_TEST_MODE -ne '1' -or -not $env:DRONE_SIM_TEST_PAUSE_READY -or -not $env:DRONE_SIM_TEST_PAUSE_CONTINUE) {
                Throw-HostFilesystemError 'publication_test_hook_rejected' 'Publication pause fault is restricted to the native test harness' $null
            }
            [IO.File]::WriteAllText($env:DRONE_SIM_TEST_PAUSE_READY, 'ready')
            $deadline = [DateTime]::UtcNow.AddSeconds(15)
            while (-not [IO.File]::Exists($env:DRONE_SIM_TEST_PAUSE_CONTINUE)) {
                if ([DateTime]::UtcNow -ge $deadline) {
                    Throw-HostFilesystemError 'publication_test_hook_timeout' 'Timed out waiting for native path-race test continuation' $null
                }
                [Threading.Thread]::Sleep(25)
            }
        }
        try {
            [DroneSim.WindowsFileSystemNative]::MoveNoReplaceWriteThrough($temporary, $target)
            $renamed = $true
        }
        catch {
            if ([DroneSim.WindowsFileSystemNative]::PathExistsNoFollow($target)) {
                if (Test-ExistingPublication $target $expectedHash) { return [pscustomobject] @{ native_path = $target; filename = $Filename; sha256 = $expectedHash } }
            }
            Throw-HostFilesystemError 'publication_move_failed' "QGroundControl action publication failed and retained temporary evidence: $temporary" $_.Exception
        }
        if ($script:TestFault -eq 'after_rename') {
            Throw-HostFilesystemError 'publication_uncertain' "Injected interruption after rename; destination retained for recovery: $target" $null
        }
        if ($script:TestFault -eq 'readback') {
            Throw-HostFilesystemError 'publication_readback_failed' "Injected readback failure; destination retained for recovery: $target" $null
        }
        $targetStream = $null
        try {
            $targetStream = [DroneSim.WindowsFileSystemNative]::OpenRegularReadLocked($target)
            $actualHash = Get-StreamSha256 $targetStream
            if ($actualHash -ne $expectedHash) {
                Throw-HostFilesystemError 'publication_readback_failed' "Published action readback differs; destination retained for recovery: $target" $null
            }
        }
        catch {
            if ($_.Exception.Data['code']) { throw }
            Throw-HostFilesystemError 'publication_readback_failed' "Published action cannot be safely reopened; destination retained for recovery: $target" $_.Exception
        }
        finally {
            if ($null -ne $targetStream) { $targetStream.Dispose() }
        }
        return [pscustomobject] @{ native_path = $target; filename = $Filename; sha256 = $actualHash }
    }
    catch {
        if ($_.Exception.Data['code']) { throw }
        Throw-HostFilesystemError 'publication_write_failed' "QGroundControl action write failed; evidence is retained at $temporary" $_.Exception
    }
    finally {
        if ($null -ne $stream) { $stream.Dispose() }
        if ($null -ne $directoryLock) { $directoryLock.Dispose() }
        if ($null -ne $parentLock) { $parentLock.Dispose() }
        # Deliberately retain a failed temporary file or renamed destination as evidence.
        $null = $renamed
    }
}

function Open-Qgc {
    [CmdletBinding()]
    param([Parameter(Mandatory = $true)][object] $Installation)

    $verified = Get-QgcInstallation -Executable ([string] $Installation.executable) -SettingsFile ([string] $Installation.settings_file)
    $running = @(Get-RunningQgcProcesses $verified.executable)
    if ($running.Count -gt 0) {
        $window = $running[0].MainWindowHandle
        if ($window -eq [IntPtr]::Zero) {
            Throw-HostFilesystemError 'qgc_focus_failed' 'Running QGroundControl has no focusable main window' $null
        }
        [DroneSim.WindowsFileSystemNative]::ShowWindowAsync($window, 9) | Out-Null
        if (-not [DroneSim.WindowsFileSystemNative]::SetForegroundWindow($window)) {
            Throw-HostFilesystemError 'qgc_focus_failed' 'Windows refused to focus the validated QGroundControl window' $null
        }
        return [pscustomobject] @{ opened = $true; detail = 'focused existing QGroundControl' }
    }
    $start = New-Object Diagnostics.ProcessStartInfo
    $start.FileName = $verified.executable
    $start.WorkingDirectory = [IO.Path]::GetDirectoryName($verified.executable)
    $start.UseShellExecute = $true
    $process = [Diagnostics.Process]::Start($start)
    if ($null -eq $process) {
        Throw-HostFilesystemError 'qgc_launch_failed' 'Windows did not start QGroundControl' $null
    }
    return [pscustomobject] @{ opened = $true; detail = "started QGroundControl process $($process.Id)" }
}

Export-ModuleMember -Function Get-QgcInstallation, Publish-QgcActions, Open-Qgc
