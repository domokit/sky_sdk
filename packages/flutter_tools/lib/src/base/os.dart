// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi' show Abi;

import 'package:archive/archive.dart';
import 'package:file/file.dart';
import 'package:meta/meta.dart';
import 'package:process/process.dart';

import 'common.dart';
import 'file_system.dart';
import 'io.dart';
import 'logger.dart';
import 'platform.dart';
import 'process.dart';

abstract class OperatingSystemUtils {
  factory OperatingSystemUtils({
    required FileSystem fileSystem,
    required Logger logger,
    required Platform platform,
    required ProcessManager processManager,
  }) {
    if (platform.isWindows) {
      return _WindowsUtils(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        processManager: processManager,
      );
    } else if (platform.isMacOS) {
      return _MacOSUtils(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        processManager: processManager,
      );
    } else if (platform.isLinux) {
      return _LinuxUtils(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        processManager: processManager,
      );
    } else {
      return _PosixUtils(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        processManager: processManager,
      );
    }
  }

  OperatingSystemUtils._private({
    required FileSystem fileSystem,
    required Logger logger,
    required Platform platform,
    required ProcessManager processManager,
  }) : _fileSystem = fileSystem,
       _logger = logger,
       _platform = platform,
       _processManager = processManager,
       _processUtils = ProcessUtils(logger: logger, processManager: processManager);

  @visibleForTesting
  static final GZipCodec gzipLevel1 = GZipCodec(level: 1);

  final FileSystem _fileSystem;
  final Logger _logger;
  final Platform _platform;
  final ProcessManager _processManager;
  final ProcessUtils _processUtils;

  /// Make the given file executable. This may be a no-op on some platforms.
  void makeExecutable(File file);

  /// Updates the specified file system [entity] to have the file mode
  /// bits set to the value defined by [mode], which can be specified in octal
  /// (e.g. `644`) or symbolically (e.g. `u+x`).
  ///
  /// On operating systems that do not support file mode bits, this will be a
  /// no-op.
  void chmod(FileSystemEntity entity, String mode);

  /// Return the path (with symlinks resolved) to the given executable, or null
  /// if `which` was not able to locate the binary.
  File? which(String execName) {
    final List<File> result = _which(execName);
    if (result.isEmpty) {
      return null;
    }
    return result.first;
  }

  /// Return a list of all paths to `execName` found on the system. Uses the
  /// PATH environment variable.
  List<File> whichAll(String execName) => _which(execName, all: true);

  /// Return the File representing a new pipe.
  File makePipe(String path);

  /// Return a directory's total size in bytes.
  int? getDirectorySize(Directory directory) {
    int? size;
    for (final FileSystemEntity entity in directory.listSync(recursive: true, followLinks: false)) {
      if (entity is File) {
        size ??= 0;
        size += entity.lengthSync();
      }
    }
    return size;
  }

  void unzip(File file, Directory targetDirectory);

  void unpack(File gzippedTarFile, Directory targetDirectory);

  /// Compresses a stream using gzip level 1 (faster but larger).
  Stream<List<int>> gzipLevel1Stream(Stream<List<int>> stream) {
    return stream.cast<List<int>>().transform<List<int>>(gzipLevel1.encoder);
  }

  /// Returns a pretty name string for the current operating system.
  ///
  /// If available, the detailed version of the OS is included.
  String get name {
    const Map<String, String> osNames = <String, String>{
      'macos': 'Mac OS',
      'linux': 'Linux',
      'windows': 'Windows',
    };
    final String osName = _platform.operatingSystem;
    return osNames[osName] ?? osName;
  }

  HostPlatform get hostPlatform;

  List<File> _which(String execName, {bool all = false});

  /// Returns the separator between items in the PATH environment variable.
  String get pathVarSeparator;

  /// Returns an unused network port.
  ///
  /// Returns 0 if an unused port cannot be found.
  ///
  /// The port returned by this function may become used before it is bound by
  /// its intended user.
  Future<int> findFreePort({bool ipv6 = false}) async {
    int port = 0;
    ServerSocket? serverSocket;
    final InternetAddress loopback =
        ipv6 ? InternetAddress.loopbackIPv6 : InternetAddress.loopbackIPv4;
    try {
      serverSocket = await ServerSocket.bind(loopback, 0);
      port = serverSocket.port;
    } on SocketException catch (e) {
      // If ipv4 loopback bind fails, try ipv6.
      if (!ipv6) {
        return findFreePort(ipv6: true);
      }
      _logger.printTrace('findFreePort failed: $e');
    } on Exception catch (e) {
      // Failures are signaled by a return value of 0 from this function.
      _logger.printTrace('findFreePort failed: $e');
    } finally {
      if (serverSocket != null) {
        await serverSocket.close();
      }
    }
    return port;
  }
}

class _PosixUtils extends OperatingSystemUtils {
  _PosixUtils({
    required super.fileSystem,
    required super.logger,
    required super.platform,
    required super.processManager,
  }) : super._private();

  @override
  void makeExecutable(File file) {
    chmod(file, 'a+x');
  }

  @override
  void chmod(FileSystemEntity entity, String mode) {
    // Errors here are silently ignored (except when tracing).
    try {
      final ProcessResult result = _processManager.runSync(<String>['chmod', mode, entity.path]);
      if (result.exitCode != 0) {
        _logger.printTrace(
          'Error trying to run "chmod $mode ${entity.path}":\n'
          '  exit code: ${result.exitCode}\n'
          '  stdout: ${result.stdout.toString().trimRight()}\n'
          '  stderr: ${result.stderr.toString().trimRight()}',
        );
      }
    } on ProcessException catch (error) {
      _logger.printTrace('Error trying to run "chmod $mode ${entity.path}": $error');
    }
  }

  @override
  List<File> _which(String execName, {bool all = false}) {
    final List<String> command = <String>['which', if (all) '-a', execName];
    final ProcessResult result = _processManager.runSync(command);
    if (result.exitCode != 0) {
      return const <File>[];
    }
    final String stdout = result.stdout as String;
    return stdout
        .trim()
        .split('\n')
        .map<File>((String path) => _fileSystem.file(path.trim()))
        .toList();
  }

  // unzip -o -q zipfile -d dest
  @override
  void unzip(File file, Directory targetDirectory) {
    if (!_processManager.canRun('unzip')) {
      // unzip is not available. this error message is modeled after the download
      // error in bin/internal/update_dart_sdk.sh
      String message = 'Please install unzip.';
      if (_platform.isMacOS) {
        message = 'Consider running "brew install unzip".';
      } else if (_platform.isLinux) {
        message = 'Consider running "sudo apt-get install unzip".';
      }
      throwToolExit('Missing "unzip" tool. Unable to extract ${file.path}.\n$message');
    }
    _processUtils.runSync(
      <String>['unzip', '-o', '-q', file.path, '-d', targetDirectory.path],
      throwOnError: true,
      verboseExceptions: true,
    );
  }

  // tar -xzf tarball -C dest
  @override
  void unpack(File gzippedTarFile, Directory targetDirectory) {
    _processUtils.runSync(<String>[
      'tar',
      '-xzf',
      gzippedTarFile.path,
      '-C',
      targetDirectory.path,
    ], throwOnError: true);
  }

  @override
  File makePipe(String path) {
    _processUtils.runSync(<String>['mkfifo', path], throwOnError: true);
    return _fileSystem.file(path);
  }

  @override
  String get pathVarSeparator => ':';

  HostPlatform? _hostPlatform;

  @override
  HostPlatform get hostPlatform {
    if (_hostPlatform == null) {
      final RunResult hostPlatformCheck = _processUtils.runSync(<String>['uname', '-m']);
      // On x64 stdout is "uname -m: x86_64"
      // On arm64 stdout is "uname -m: aarch64, arm64_v8a"
      if (hostPlatformCheck.exitCode != 0) {
        _hostPlatform = HostPlatform.linux_x64;
        _logger.printError(
          'Encountered an error trying to run "uname -m":\n'
          '  exit code: ${hostPlatformCheck.exitCode}\n'
          '  stdout: ${hostPlatformCheck.stdout.trimRight()}\n'
          '  stderr: ${hostPlatformCheck.stderr.trimRight()}\n'
          'Assuming host platform is ${getNameForHostPlatform(_hostPlatform!)}.',
        );
      } else if (hostPlatformCheck.stdout.trim().endsWith('x86_64')) {
        _hostPlatform = HostPlatform.linux_x64;
      } else {
        // We default to ARM if it's not x86_64 and we did not get an error.
        _hostPlatform = HostPlatform.linux_arm64;
      }
    }
    return _hostPlatform!;
  }
}

class _LinuxUtils extends _PosixUtils {
  _LinuxUtils({
    required super.fileSystem,
    required super.logger,
    required super.platform,
    required super.processManager,
  });

  String? _name;

  @override
  String get name {
    if (_name == null) {
      const String prettyNameKey = 'PRETTY_NAME';
      // If "/etc/os-release" doesn't exist, fallback to "/usr/lib/os-release".
      final String osReleasePath =
          _fileSystem.file('/etc/os-release').existsSync()
              ? '/etc/os-release'
              : '/usr/lib/os-release';
      String prettyName;
      String kernelRelease;
      try {
        final String osRelease = _fileSystem.file(osReleasePath).readAsStringSync();
        prettyName = _getOsReleaseValueForKey(osRelease, prettyNameKey);
      } on Exception catch (e) {
        _logger.printTrace('Failed obtaining PRETTY_NAME for Linux: $e');
        prettyName = '';
      }
      try {
        // Split the operating system version which should be formatted as
        // "Linux kernelRelease build", by spaces.
        final List<String> osVersionSplit = _platform.operatingSystemVersion.split(' ');
        if (osVersionSplit.length < 3) {
          // The operating system version didn't have the expected format.
          // Initialize as an empty string.
          kernelRelease = '';
        } else {
          kernelRelease = ' ${osVersionSplit[1]}';
        }
      } on Exception catch (e) {
        _logger.printTrace('Failed obtaining kernel release for Linux: $e');
        kernelRelease = '';
      }
      _name = '${prettyName.isEmpty ? super.name : prettyName}$kernelRelease';
    }
    return _name!;
  }

  String _getOsReleaseValueForKey(String osRelease, String key) {
    final List<String> osReleaseSplit = osRelease.split('\n');
    for (String entry in osReleaseSplit) {
      entry = entry.trim();
      final List<String> entryKeyValuePair = entry.split('=');
      if (entryKeyValuePair[0] == key) {
        final String value = entryKeyValuePair[1];
        // Remove quotes from either end of the value if they exist
        final String quote = value[0];
        if (quote == "'" || quote == '"') {
          return value.substring(0, value.length - 1).substring(1);
        } else {
          return value;
        }
      }
    }
    return '';
  }
}

class _MacOSUtils extends _PosixUtils {
  _MacOSUtils({
    required super.fileSystem,
    required super.logger,
    required super.platform,
    required super.processManager,
  });

  String? _name;

  @override
  String get name {
    if (_name == null) {
      final List<RunResult> results = <RunResult>[
        _processUtils.runSync(<String>['sw_vers', '-productName']),
        _processUtils.runSync(<String>['sw_vers', '-productVersion']),
        _processUtils.runSync(<String>['sw_vers', '-buildVersion']),
        _processUtils.runSync(<String>['uname', '-m']),
      ];
      if (results.every((RunResult result) => result.exitCode == 0)) {
        String osName = getNameForHostPlatform(hostPlatform);
        // If the script is running in Rosetta, "uname -m" will return x86_64.
        if (hostPlatform == HostPlatform.darwin_arm64 && results[3].stdout.contains('x86_64')) {
          osName = '$osName (Rosetta)';
        }
        _name =
            '${results[0].stdout.trim()} ${results[1].stdout.trim()} ${results[2].stdout.trim()} $osName';
      }
      _name ??= super.name;
    }
    return _name!;
  }

  // On ARM returns arm64, even when this process is running in Rosetta.
  @override
  HostPlatform get hostPlatform {
    if (_hostPlatform == null) {
      String? sysctlPath;
      if (which('sysctl') == null) {
        // Fallback to known install locations.
        for (final String path in <String>['/usr/sbin/sysctl', '/sbin/sysctl']) {
          if (_fileSystem.isFileSync(path)) {
            sysctlPath = path;
          }
        }
      } else {
        sysctlPath = 'sysctl';
      }

      if (sysctlPath == null) {
        throwToolExit('sysctl not found. Try adding it to your PATH environment variable.');
      }
      final RunResult arm64Check = _processUtils.runSync(<String>[sysctlPath, 'hw.optional.arm64']);
      // On arm64 stdout is "sysctl hw.optional.arm64: 1"
      // On x86 hw.optional.arm64 is unavailable and exits with 1.
      if (arm64Check.exitCode == 0 && arm64Check.stdout.trim().endsWith('1')) {
        _hostPlatform = HostPlatform.darwin_arm64;
      } else {
        _hostPlatform = HostPlatform.darwin_x64;
      }
    }
    return _hostPlatform!;
  }

  // unzip, then rsync
  @override
  void unzip(File file, Directory targetDirectory) {
    if (!_processManager.canRun('unzip')) {
      // unzip is not available. this error message is modeled after the download
      // error in bin/internal/update_dart_sdk.sh
      throwToolExit(
        'Missing "unzip" tool. Unable to extract ${file.path}.\nConsider running "brew install unzip".',
      );
    }
    if (_processManager.canRun('rsync')) {
      final Directory tempDirectory = _fileSystem.systemTempDirectory.createTempSync(
        'flutter_${file.basename}.',
      );
      try {
        // Unzip to a temporary directory.
        _processUtils.runSync(
          <String>['unzip', '-o', '-q', file.path, '-d', tempDirectory.path],
          throwOnError: true,
          verboseExceptions: true,
        );
        for (final FileSystemEntity unzippedFile in tempDirectory.listSync(followLinks: false)) {
          // rsync --delete the unzipped files so files removed from the archive are also removed from the target.
          // Add the '-8' parameter to avoid mangling filenames with encodings that do not match the current locale.
          _processUtils.runSync(
            <String>['rsync', '-8', '-av', '--delete', unzippedFile.path, targetDirectory.path],
            throwOnError: true,
            verboseExceptions: true,
          );
        }
      } finally {
        tempDirectory.deleteSync(recursive: true);
      }
    } else {
      // Fall back to just unzipping.
      _logger.printTrace('Unable to find rsync, falling back to direct unzipping.');
      _processUtils.runSync(
        <String>['unzip', '-o', '-q', file.path, '-d', targetDirectory.path],
        throwOnError: true,
        verboseExceptions: true,
      );
    }
  }
}

class _WindowsUtils extends OperatingSystemUtils {
  _WindowsUtils({
    required super.fileSystem,
    required super.logger,
    required super.platform,
    required super.processManager,
  }) : super._private();

  HostPlatform? _hostPlatform;

  @override
  HostPlatform get hostPlatform {
    if (_hostPlatform == null) {
      final Abi abi = Abi.current();
      _hostPlatform =
          (abi == Abi.windowsArm64) ? HostPlatform.windows_arm64 : HostPlatform.windows_x64;
    }
    return _hostPlatform!;
  }

  @override
  void makeExecutable(File file) {}

  @override
  void chmod(FileSystemEntity entity, String mode) {}

  @override
  List<File> _which(String execName, {bool all = false}) {
    if (!_processManager.canRun('where')) {
      // `where` could be missing if system32 is not on the PATH.
      throwToolExit(
        'Cannot find the executable for `where`. This can happen if the System32 '
        r'folder (e.g. C:\Windows\System32 ) is removed from the PATH environment '
        'variable. Ensure that this is present and then try again after restarting '
        'the terminal and/or IDE.',
      );
    }
    // `where` always returns all matches, not just the first one.
    final ProcessResult result = _processManager.runSync(<String>['where', execName]);
    if (result.exitCode != 0) {
      return const <File>[];
    }
    final List<String> lines = (result.stdout as String).trim().split('\n');
    if (all) {
      return lines.map<File>((String path) => _fileSystem.file(path.trim())).toList();
    }
    return <File>[_fileSystem.file(lines.first.trim())];
  }

  @override
  void unzip(File file, Directory targetDirectory) {
    final Archive archive = ZipDecoder().decodeBytes(file.readAsBytesSync());
    _unpackArchive(archive, targetDirectory);
  }

  @override
  void unpack(File gzippedTarFile, Directory targetDirectory) {
    final Archive archive = TarDecoder().decodeBytes(
      GZipDecoder().decodeBytes(gzippedTarFile.readAsBytesSync()),
    );
    _unpackArchive(archive, targetDirectory);
  }

  void _unpackArchive(Archive archive, Directory targetDirectory) {
    for (final ArchiveFile archiveFile in archive.files) {
      // The archive package doesn't correctly set isFile.
      if (!archiveFile.isFile || archiveFile.name.endsWith('/')) {
        continue;
      }

      final File destFile = _fileSystem.file(
        _fileSystem.path.canonicalize(
          _fileSystem.path.join(targetDirectory.path, archiveFile.name),
        ),
      );

      // Validate that the destFile is within the targetDirectory we want to
      // extract to.
      //
      // See https://snyk.io/research/zip-slip-vulnerability for more context.
      final String destinationFileCanonicalPath = _fileSystem.path.canonicalize(destFile.path);
      final String targetDirectoryCanonicalPath = _fileSystem.path.canonicalize(
        targetDirectory.path,
      );
      if (!destinationFileCanonicalPath.startsWith(targetDirectoryCanonicalPath)) {
        throw StateError(
          'Tried to extract the file $destinationFileCanonicalPath outside of the '
          'target directory $targetDirectoryCanonicalPath',
        );
      }

      if (!destFile.parent.existsSync()) {
        destFile.parent.createSync(recursive: true);
      }
      destFile.writeAsBytesSync(archiveFile.content as List<int>);
    }
  }

  @override
  File makePipe(String path) {
    throw UnsupportedError('makePipe is not implemented on Windows.');
  }

  String? _name;

  @override
  String get name {
    if (_name == null) {
      final ProcessResult result = _processManager.runSync(<String>['ver'], runInShell: true);
      if (result.exitCode == 0) {
        _name = (result.stdout as String).trim();
      } else {
        _name = super.name;
      }
    }
    return _name!;
  }

  @override
  String get pathVarSeparator => ';';
}

/// Find and return the project root directory relative to the specified
/// directory or the current working directory if none specified.
/// Return null if the project root could not be found
/// or if the project root is the flutter repository root.
String? findProjectRoot(FileSystem fileSystem, [String? directory]) {
  const String kProjectRootSentinel = 'pubspec.yaml';
  directory ??= fileSystem.currentDirectory.path;
  Directory currentDirectory = fileSystem.directory(directory).absolute;
  while (true) {
    if (currentDirectory.childFile(kProjectRootSentinel).existsSync()) {
      return currentDirectory.path;
    }
    if (!currentDirectory.existsSync() || currentDirectory.parent.path == currentDirectory.path) {
      return null;
    }
    currentDirectory = currentDirectory.parent;
  }
}

enum HostPlatform {
  darwin_x64,
  darwin_arm64,
  linux_x64,
  linux_arm64,
  windows_x64,
  windows_arm64;

  String get platformName => switch (this) {
    HostPlatform.darwin_x64 => 'x64',
    HostPlatform.darwin_arm64 => 'arm64',
    HostPlatform.linux_x64 => 'x64',
    HostPlatform.linux_arm64 => 'arm64',
    HostPlatform.windows_x64 => 'x64',
    HostPlatform.windows_arm64 => 'arm64',
  };
}

String getNameForHostPlatform(HostPlatform platform) {
  return switch (platform) {
    HostPlatform.darwin_x64 => 'darwin-x64',
    HostPlatform.darwin_arm64 => 'darwin-arm64',
    HostPlatform.linux_x64 => 'linux-x64',
    HostPlatform.linux_arm64 => 'linux-arm64',
    HostPlatform.windows_x64 => 'windows-x64',
    HostPlatform.windows_arm64 => 'windows-arm64',
  };
}
