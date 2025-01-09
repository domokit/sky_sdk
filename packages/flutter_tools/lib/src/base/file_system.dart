// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/local.dart' as local_fs;
import 'package:meta/meta.dart';

import 'common.dart';
import 'io.dart';
import 'platform.dart';
import 'process.dart';
import 'signals.dart';

// package:file/local.dart must not be exported. This exposes LocalFileSystem,
// which we override to ensure that temporary directories are cleaned up when
// the tool is killed by a signal.
export 'package:file/file.dart';

/// Exception indicating that a file that was expected to exist was not found.
class FileNotFoundException implements IOException {
  const FileNotFoundException(this.path);

  final String path;

  @override
  String toString() => 'File not found: $path';
}

/// Various convenience file system methods.
class FileSystemUtils {
  FileSystemUtils({required FileSystem fileSystem, required Platform platform})
    : _fileSystem = fileSystem,
      _platform = platform;

  final FileSystem _fileSystem;

  final Platform _platform;

  /// Appends a number to a filename in order to make it unique under a
  /// directory.
  File getUniqueFile(Directory dir, String baseName, String ext) {
    return _getUniqueFile(dir, baseName, ext);
  }

  /// Appends a number to a directory name in order to make it unique under a
  /// directory.
  Directory getUniqueDirectory(Directory dir, String baseName) {
    final FileSystem fs = dir.fileSystem;
    int i = 1;

    while (true) {
      final String name = '${baseName}_${i.toString().padLeft(2, '0')}';
      final Directory directory = fs.directory(_fileSystem.path.join(dir.path, name));
      if (!directory.existsSync()) {
        return directory;
      }
      i += 1;
    }
  }

  /// Escapes [path].
  ///
  /// On Windows it replaces all '\' with '\\'. On other platforms, it returns the
  /// path unchanged.
  String escapePath(String path) => _platform.isWindows ? path.replaceAll(r'\', r'\\') : path;

  /// Returns true if the file system [entity] has not been modified since the
  /// latest modification to [referenceFile].
  ///
  /// Returns true, if [entity] does not exist.
  ///
  /// Returns false, if [entity] exists, but [referenceFile] does not.
  bool isOlderThanReference({required FileSystemEntity entity, required File referenceFile}) {
    if (!entity.existsSync()) {
      return true;
    }
    return referenceFile.existsSync() &&
        referenceFile.statSync().modified.isAfter(entity.statSync().modified);
  }

  /// Return the absolute path of the user's home directory.
  String? get homeDirPath {
    String? path =
        _platform.isWindows ? _platform.environment['USERPROFILE'] : _platform.environment['HOME'];
    if (path != null) {
      path = _fileSystem.path.absolute(path);
    }
    return path;
  }
}

/// Return a relative path if [fullPath] is contained by the cwd, else return an
/// absolute path.
String getDisplayPath(String fullPath, FileSystem fileSystem) {
  final String cwd = fileSystem.currentDirectory.path + fileSystem.path.separator;
  return fullPath.startsWith(cwd) ? fullPath.substring(cwd.length) : fullPath;
}

/// Creates `destDir` if needed, then recursively copies `srcDir` to
/// `destDir`, invoking [onFileCopied], if specified, for each
/// source/destination file pair.
///
/// Skips files if [shouldCopyFile] returns `false`.
/// Does not recurse over directories if [shouldCopyDirectory] returns `false`.
///
/// If [followLinks] is false, then any symbolic links found are reported as
/// [Link] objects, rather than as directories or files, and are not recursed into.
///
/// If [followLinks] is true, then working links are reported as directories or
/// files, depending on what they point to.
void copyDirectory(
  Directory srcDir,
  Directory destDir, {
  bool Function(File srcFile, File destFile)? shouldCopyFile,
  bool Function(Directory)? shouldCopyDirectory,
  void Function(File srcFile, File destFile)? onFileCopied,
  bool followLinks = true,
}) {
  if (!srcDir.existsSync()) {
    throw Exception('Source directory "${srcDir.path}" does not exist, nothing to copy');
  }

  if (!destDir.existsSync()) {
    destDir.createSync(recursive: true);
  }

  for (final FileSystemEntity entity in srcDir.listSync(followLinks: followLinks)) {
    final String newPath = destDir.fileSystem.path.join(destDir.path, entity.basename);
    if (entity is Link) {
      final Link newLink = destDir.fileSystem.link(newPath);
      newLink.createSync(entity.targetSync());
    } else if (entity is File) {
      final File newFile = destDir.fileSystem.file(newPath);
      if (shouldCopyFile != null && !shouldCopyFile(entity, newFile)) {
        continue;
      }
      newFile.writeAsBytesSync(entity.readAsBytesSync());
      onFileCopied?.call(entity, newFile);
    } else if (entity is Directory) {
      if (shouldCopyDirectory != null && !shouldCopyDirectory(entity)) {
        continue;
      }
      copyDirectory(
        entity,
        destDir.fileSystem.directory(newPath),
        shouldCopyFile: shouldCopyFile,
        onFileCopied: onFileCopied,
        followLinks: followLinks,
      );
    } else {
      throw Exception('${entity.path} is neither File nor Directory, was ${entity.runtimeType}');
    }
  }
}

File _getUniqueFile(Directory dir, String baseName, String ext) {
  final FileSystem fs = dir.fileSystem;
  int i = 1;

  while (true) {
    final String name = '${baseName}_${i.toString().padLeft(2, '0')}.$ext';
    final File file = fs.file(dir.fileSystem.path.join(dir.path, name));
    if (!file.existsSync()) {
      file.createSync(recursive: true);
      return file;
    }
    i += 1;
  }
}

/// Appends a number to a filename in order to make it unique under a
/// directory.
File getUniqueFile(Directory dir, String baseName, String ext) {
  return _getUniqueFile(dir, baseName, ext);
}

/// This class extends [local_fs.LocalFileSystem] in order to clean up
/// directories and files that the tool creates under the system temporary
/// directory when the tool exits either normally or when killed by a signal.
class LocalFileSystem extends local_fs.LocalFileSystem {
  LocalFileSystem(this._signals, this._fatalSignals, this.shutdownHooks);

  @visibleForTesting
  LocalFileSystem.test({
    required Signals signals,
    List<ProcessSignal> fatalSignals = Signals.defaultExitSignals,
  }) : this(signals, fatalSignals, ShutdownHooks());

  Directory? _systemTemp;
  final Map<ProcessSignal, Object> _signalTokens = <ProcessSignal, Object>{};

  final ShutdownHooks shutdownHooks;

  // Indicates that `dispose()` has been invoked or some shutdown hook has executed,
  // resulting in the underlying temporary directory being cleaned up.
  bool get disposed => _disposed;
  bool _disposed = false;

  Future<void> dispose() async {
    _tryToDeleteTemp();
    for (final MapEntry<ProcessSignal, Object> signalToken in _signalTokens.entries) {
      await _signals.removeHandler(signalToken.key, signalToken.value);
    }
    _signalTokens.clear();
  }

  final Signals _signals;
  final List<ProcessSignal> _fatalSignals;

  void _tryToDeleteTemp() {
    _disposed = true;
    try {
      if (_systemTemp?.existsSync() ?? false) {
        _systemTemp?.deleteSync(recursive: true);
      }
    } on FileSystemException {
      // ignore
    }
    _systemTemp = null;
  }

  // This getter returns a fresh entry under /tmp, like
  // /tmp/flutter_tools.abcxyz, then the rest of the tool creates /tmp entries
  // under that, like /tmp/flutter_tools.abcxyz/flutter_build_stuff.123456.
  // Right before exiting because of a signal or otherwise, we delete
  // /tmp/flutter_tools.abcxyz, not the whole of /tmp.
  @override
  Directory get systemTempDirectory {
    if (_systemTemp == null) {
      if (!superSystemTempDirectory.existsSync()) {
        throwToolExit(
          'Your system temp directory (${superSystemTempDirectory.path}) does not exist. '
          'Did you set an invalid override in your environment? See issue https://github.com/flutter/flutter/issues/74042 for more context.',
        );
      }
      _systemTemp = superSystemTempDirectory.createTempSync('flutter_tools.')
        ..createSync(recursive: true);
      // Make sure that the temporary directory is cleaned up if the tool is
      // killed by a signal.
      for (final ProcessSignal signal in _fatalSignals) {
        final Object token = _signals.addHandler(signal, (ProcessSignal _) {
          _tryToDeleteTemp();
        });
        _signalTokens[signal] = token;
      }
      // Make sure that the temporary directory is cleaned up when the tool
      // exits normally.
      shutdownHooks.addShutdownHook(_tryToDeleteTemp);
    }
    return _systemTemp!;
  }

  // This only exist because the memory file system does not support a systemTemp that does not exists #74042
  @visibleForTesting
  Directory get superSystemTempDirectory => super.systemTempDirectory;
}
