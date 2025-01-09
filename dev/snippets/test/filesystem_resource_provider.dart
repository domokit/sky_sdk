// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;
import 'dart:typed_data';

import 'package:analyzer/file_system/file_system.dart';
import 'package:analyzer/source/file_source.dart';
import 'package:analyzer/source/source.dart';
import 'package:file/file.dart' as file;
import 'package:file/local.dart' as file;
import 'package:meta/meta.dart';
import 'package:path/path.dart';
import 'package:watcher/watcher.dart';

/// The name of the directory containing plugin specific subfolders used to
/// store data across sessions.
const String _SERVER_DIR = '.dartServer';

/// Returns the path to default state location.
///
/// Generally this is ~/.dartServer. It can be overridden via the
/// ANALYZER_STATE_LOCATION_OVERRIDE environment variable, in which case this
/// method will return the contents of that environment variable.
String? _getStandardStateLocation() {
  final Map<String, String> env = io.Platform.environment;
  if (env.containsKey('ANALYZER_STATE_LOCATION_OVERRIDE')) {
    return env['ANALYZER_STATE_LOCATION_OVERRIDE'];
  }

  final String? home = io.Platform.isWindows ? env['LOCALAPPDATA'] : env['HOME'];
  return home != null && io.FileSystemEntity.isDirectorySync(home) ? join(home, _SERVER_DIR) : null;
}

/// A `dart:io` based implementation of [ResourceProvider].
class FileSystemResourceProvider implements ResourceProvider {
  FileSystemResourceProvider(this.filesystem, {String? stateLocation})
    : _stateLocation = stateLocation ?? _getStandardStateLocation();

  static final FileSystemResourceProvider instance = FileSystemResourceProvider(
    const file.LocalFileSystem(),
  );

  /// The path to the base folder where state is stored.
  final String? _stateLocation;

  final file.FileSystem filesystem;

  @override
  Context get pathContext => context;

  @override
  File getFile(String path) {
    _ensureAbsoluteAndNormalized(path);
    return _PhysicalFile(filesystem.file(path));
  }

  @override
  Folder getFolder(String path) {
    _ensureAbsoluteAndNormalized(path);
    return _PhysicalFolder(filesystem.directory(path));
  }

  @override
  Resource getResource(String path) {
    _ensureAbsoluteAndNormalized(path);
    if (filesystem.isDirectorySync(path)) {
      return getFolder(path);
    } else {
      return getFile(path);
    }
  }

  @override
  Folder? getStateLocation(String pluginId) {
    if (_stateLocation != null) {
      final file.Directory directory = filesystem.directory(join(_stateLocation, pluginId));
      directory.createSync(recursive: true);
      return _PhysicalFolder(directory);
    }
    return null;
  }

  /// The file system abstraction supports only absolute and normalized paths.
  /// This method is used to validate any input paths to prevent errors later.
  void _ensureAbsoluteAndNormalized(String path) {
    assert(() {
      if (!pathContext.isAbsolute(path)) {
        throw ArgumentError('Path must be absolute : $path');
      }
      if (pathContext.normalize(path) != path) {
        throw ArgumentError('Path must be normalized : $path');
      }
      return true;
    }());
  }

  @override
  Link getLink(String path) {
    throw UnimplementedError('getLink Not Implemented');
  }
}

/// A `dart:io` based implementation of [File].
class _PhysicalFile extends _PhysicalResource implements File {
  const _PhysicalFile(io.File super.file);

  @override
  Stream<WatchEvent> get changes => FileWatcher(_entry.path).events;

  @override
  int get lengthSync {
    try {
      return _file.lengthSync();
    } on io.FileSystemException catch (exception) {
      throw _wrapException(exception);
    }
  }

  @override
  int get modificationStamp {
    try {
      return _file.lastModifiedSync().millisecondsSinceEpoch;
    } on io.FileSystemException catch (exception) {
      throw _wrapException(exception);
    }
  }

  /// Return the underlying file being represented by this wrapper.
  io.File get _file => _entry as io.File;

  @override
  File copyTo(Folder parentFolder) {
    parentFolder.create();
    final File destination = parentFolder.getChildAssumingFile(shortName);
    destination.writeAsBytesSync(readAsBytesSync());
    return destination;
  }

  @override
  Source createSource([Uri? uri]) {
    return FileSource(this, uri ?? pathContext.toUri(path));
  }

  @override
  bool isOrContains(String path) {
    return path == this.path;
  }

  @override
  Uint8List readAsBytesSync() {
    _throwIfWindowsDeviceDriver();
    try {
      return _file.readAsBytesSync();
    } on io.FileSystemException catch (exception) {
      throw _wrapException(exception);
    }
  }

  @override
  String readAsStringSync() {
    _throwIfWindowsDeviceDriver();
    try {
      return _file.readAsStringSync();
    } on io.FileSystemException catch (exception) {
      throw _wrapException(exception);
    }
  }

  @override
  File renameSync(String newPath) {
    try {
      return _PhysicalFile(_file.renameSync(newPath));
    } on io.FileSystemException catch (exception) {
      throw _wrapException(exception);
    }
  }

  @override
  File resolveSymbolicLinksSync() {
    try {
      return _PhysicalFile(io.File(_file.resolveSymbolicLinksSync()));
    } on io.FileSystemException catch (exception) {
      throw _wrapException(exception);
    }
  }

  @override
  Uri toUri() => Uri.file(path);

  @override
  void writeAsBytesSync(List<int> bytes) {
    try {
      _file.writeAsBytesSync(bytes);
    } on io.FileSystemException catch (exception) {
      throw _wrapException(exception);
    }
  }

  @override
  void writeAsStringSync(String content) {
    try {
      _file.writeAsStringSync(content);
    } on io.FileSystemException catch (exception) {
      throw _wrapException(exception);
    }
  }

  @override
  ResourceWatcher watch() {
    throw UnimplementedError();
  }
}

/// A `dart:io` based implementation of [Folder].
class _PhysicalFolder extends _PhysicalResource implements Folder {
  const _PhysicalFolder(io.Directory super.directory);

  @override
  Stream<WatchEvent> get changes => DirectoryWatcher(_entry.path).events.handleError(
    (Object error) {},
    test:
        (dynamic error) =>
            error is io.FileSystemException &&
            // Don't suppress "Directory watcher closed," so the outer
            // listener can see the interruption & act on it.
            !error.message.startsWith('Directory watcher closed unexpectedly'),
  );

  @override
  bool get isRoot {
    final String parentPath = provider.pathContext.dirname(path);
    return parentPath == path;
  }

  /// Return the underlying file being represented by this wrapper.
  io.Directory get _directory => _entry as io.Directory;

  @override
  String canonicalizePath(String relPath) {
    return normalize(join(path, relPath));
  }

  @override
  bool contains(String path) {
    FileSystemResourceProvider.instance._ensureAbsoluteAndNormalized(path);
    return pathContext.isWithin(this.path, path);
  }

  @override
  Folder copyTo(Folder parentFolder) {
    final Folder destination = parentFolder.getChildAssumingFolder(shortName);
    destination.create();
    for (final Resource child in getChildren()) {
      child.copyTo(destination);
    }
    return destination;
  }

  @override
  void create() {
    _directory.createSync(recursive: true);
  }

  @override
  Resource getChild(String relPath) {
    final String canonicalPath = canonicalizePath(relPath);
    return FileSystemResourceProvider.instance.getResource(canonicalPath);
  }

  @override
  _PhysicalFile getChildAssumingFile(String relPath) {
    final String canonicalPath = canonicalizePath(relPath);
    final io.File file = io.File(canonicalPath);
    return _PhysicalFile(file);
  }

  @override
  _PhysicalFolder getChildAssumingFolder(String relPath) {
    final String canonicalPath = canonicalizePath(relPath);
    final io.Directory directory = io.Directory(canonicalPath);
    return _PhysicalFolder(directory);
  }

  @override
  List<Resource> getChildren() {
    try {
      final List<Resource> children = <Resource>[];
      final io.Directory directory = _entry as io.Directory;
      final List<io.FileSystemEntity> entries = directory.listSync();
      final int numEntries = entries.length;
      for (int i = 0; i < numEntries; i++) {
        final io.FileSystemEntity entity = entries[i];
        if (entity is io.Directory) {
          children.add(_PhysicalFolder(entity));
        } else if (entity is io.File) {
          children.add(_PhysicalFile(entity));
        }
      }
      return children;
    } on io.FileSystemException catch (exception) {
      throw _wrapException(exception);
    }
  }

  @override
  bool isOrContains(String path) {
    if (path == this.path) {
      return true;
    }
    return contains(path);
  }

  @override
  Folder resolveSymbolicLinksSync() {
    try {
      return _PhysicalFolder(io.Directory(_directory.resolveSymbolicLinksSync()));
    } on io.FileSystemException catch (exception) {
      throw _wrapException(exception);
    }
  }

  @override
  Uri toUri() => Uri.directory(path);

  @override
  ResourceWatcher watch() {
    throw UnimplementedError();
  }
}

/// A `dart:io` based implementation of [Resource].
@immutable
abstract class _PhysicalResource implements Resource {
  const _PhysicalResource(this._entry);

  final io.FileSystemEntity _entry;

  @override
  bool get exists {
    try {
      return _entry.existsSync();
    } on FileSystemException {
      return false;
    }
  }

  @override
  int get hashCode => path.hashCode;

  @override
  Folder get parent {
    final String parentPath = pathContext.dirname(path);
    return _PhysicalFolder(io.Directory(parentPath));
  }

  @override
  Folder get parent2 {
    final String parentPath = pathContext.dirname(path);
    return _PhysicalFolder(io.Directory(parentPath));
  }

  @override
  String get path => _entry.path;

  /// Return the path context used by this resource provider.
  Context get pathContext => io.Platform.isWindows ? windows : posix;

  @override
  ResourceProvider get provider => FileSystemResourceProvider.instance;

  @override
  String get shortName => pathContext.basename(path);

  @override
  bool operator ==(Object other) {
    if (runtimeType != other.runtimeType) {
      return false;
    }
    // ignore: test_types_in_equals
    return path == (other as _PhysicalResource).path;
  }

  @override
  void delete() {
    try {
      _entry.deleteSync(recursive: true);
    } on io.FileSystemException catch (exception) {
      throw _wrapException(exception);
    }
  }

  @override
  String toString() => path;

  /// If the operating system is Windows and the resource references one of the
  /// device drivers, throw a [FileSystemException].
  ///
  /// https://support.microsoft.com/en-us/kb/74496
  void _throwIfWindowsDeviceDriver() {
    if (io.Platform.isWindows) {
      final String shortName = this.shortName.toUpperCase();
      if (shortName == r'CON' ||
          shortName == r'PRN' ||
          shortName == r'AUX' ||
          shortName == r'CLOCK$' ||
          shortName == r'NUL' ||
          shortName == r'COM1' ||
          shortName == r'LPT1' ||
          shortName == r'LPT2' ||
          shortName == r'LPT3' ||
          shortName == r'COM2' ||
          shortName == r'COM3' ||
          shortName == r'COM4') {
        throw FileSystemException(path, 'Windows device drivers cannot be read.');
      }
    }
  }

  FileSystemException _wrapException(io.FileSystemException e) {
    return FileSystemException(e.path ?? path, e.message);
  }
}
