// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io show Directory, File, IOOverrides, Link;

import 'package:flutter_tools/src/base/file_system.dart';

/// An [IOOverrides] that can delegate to [FileSystem] implementation if provided.
///
/// Does not override any of the socket facilities.
///
/// Do not provide a [LocalFileSystem] as a delegate. Since internally this calls
/// out to `dart:io` classes, it will result in a stack overflow error as the
/// IOOverrides and LocalFileSystem call each other endlessly.
///
/// The only safe delegate types are those that do not call out to `dart:io`,
/// like the [MemoryFileSystem].
class FlutterIOOverrides extends io.IOOverrides {
  FlutterIOOverrides({FileSystem? fileSystem}) : _fileSystemDelegate = fileSystem;

  final FileSystem? _fileSystemDelegate;

  @override
  io.Directory createDirectory(String path) {
    return _fileSystemDelegate?.directory(path) ?? super.createDirectory(path);
  }

  @override
  io.File createFile(String path) {
    return _fileSystemDelegate?.file(path) ?? super.createFile(path);
  }

  @override
  io.Link createLink(String path) {
    return _fileSystemDelegate?.link(path) ?? super.createLink(path);
  }

  @override
  Stream<FileSystemEvent> fsWatch(String path, int events, bool recursive) {
    return _fileSystemDelegate?.file(path).watch(events: events, recursive: recursive) ??
        super.fsWatch(path, events, recursive);
  }

  @override
  bool fsWatchIsSupported() {
    return _fileSystemDelegate?.isWatchSupported ?? super.fsWatchIsSupported();
  }

  @override
  Future<FileSystemEntityType> fseGetType(String path, bool followLinks) {
    return _fileSystemDelegate?.type(path, followLinks: followLinks) ??
        super.fseGetType(path, followLinks);
  }

  @override
  FileSystemEntityType fseGetTypeSync(String path, bool followLinks) {
    return _fileSystemDelegate?.typeSync(path, followLinks: followLinks) ??
        super.fseGetTypeSync(path, followLinks);
  }

  @override
  Future<bool> fseIdentical(String path1, String path2) {
    return _fileSystemDelegate?.identical(path1, path2) ?? super.fseIdentical(path1, path2);
  }

  @override
  bool fseIdenticalSync(String path1, String path2) {
    return _fileSystemDelegate?.identicalSync(path1, path2) ?? super.fseIdenticalSync(path1, path2);
  }

  @override
  io.Directory getCurrentDirectory() {
    return _fileSystemDelegate?.currentDirectory ?? super.getCurrentDirectory();
  }

  @override
  io.Directory getSystemTempDirectory() {
    return _fileSystemDelegate?.systemTempDirectory ?? super.getSystemTempDirectory();
  }

  @override
  void setCurrentDirectory(String path) {
    if (_fileSystemDelegate == null) {
      return super.setCurrentDirectory(path);
    }
    _fileSystemDelegate.currentDirectory = path;
  }

  @override
  Future<FileStat> stat(String path) {
    return _fileSystemDelegate?.stat(path) ?? super.stat(path);
  }

  @override
  FileStat statSync(String path) {
    return _fileSystemDelegate?.statSync(path) ?? super.statSync(path);
  }
}
