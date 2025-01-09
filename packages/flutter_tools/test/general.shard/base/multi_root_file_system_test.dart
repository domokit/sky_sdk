// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/multi_root_file_system.dart';

import '../../src/common.dart';

void setupFileSystem({
  required MemoryFileSystem fs,
  required List<String> directories,
  required List<String> files,
}) {
  for (final String directory in directories) {
    fs.directory(directory).createSync(recursive: true);
  }

  for (final String file in files) {
    fs.file(file).writeAsStringSync('Content: $file');
  }
}

void main() {
  group('Posix style', () {
    runTest(FileSystemStyle.posix);
  });

  group('Windows style', () {
    runTest(FileSystemStyle.windows);
  });
}

void runTest(FileSystemStyle style) {
  final String sep = style == FileSystemStyle.windows ? r'\' : '/';
  final String root = style == FileSystemStyle.windows ? r'C:\' : '/';
  final String rootUri = style == FileSystemStyle.windows ? 'C:/' : '';

  late MultiRootFileSystem fs;

  setUp(() {
    final MemoryFileSystem memory = MemoryFileSystem(style: style);
    setupFileSystem(
      fs: memory,
      directories: <String>[
        '${root}foo${sep}subdir',
        '${root}bar',
        '${root}bar${sep}bar_subdir',
        '${root}other${sep}directory',
      ],
      files: <String>[
        '${root}foo${sep}only_in_foo',
        '${root}foo${sep}in_both',
        '${root}foo${sep}subdir${sep}in_subdir',
        '${root}bar${sep}only_in_bar',
        '${root}bar${sep}in_both',
        '${root}bar${sep}bar_subdir${sep}in_subdir',
        '${root}other${sep}directory${sep}file',
      ],
    );

    fs = MultiRootFileSystem(
      delegate: memory,
      scheme: 'scheme',
      roots: <String>['${root}foo$sep', '${root}bar'],
    );
  });

  testWithoutContext('file inside root', () {
    final File file = fs.file('${root}foo${sep}only_in_foo');
    expect(file.readAsStringSync(), 'Content: ${root}foo${sep}only_in_foo');
    expect(file.path, '${root}foo${sep}only_in_foo');
    expect(file.uri, Uri.parse('scheme:///only_in_foo'));
  });

  testWithoutContext('file inside second root', () {
    final File file = fs.file('${root}bar${sep}only_in_bar');
    expect(file.readAsStringSync(), 'Content: ${root}bar${sep}only_in_bar');
    expect(file.path, '${root}bar${sep}only_in_bar');
    expect(file.uri, Uri.parse('scheme:///only_in_bar'));
  });

  testWithoutContext('file outside root', () {
    final File file = fs.file('${root}other${sep}directory${sep}file');
    expect(file.readAsStringSync(), 'Content: ${root}other${sep}directory${sep}file');
    expect(file.path, '${root}other${sep}directory${sep}file');
    expect(file.uri, Uri.parse('file:///${rootUri}other/directory/file'));
  });

  testWithoutContext('file with file system scheme', () {
    final File file = fs.file('scheme:///only_in_foo');
    expect(file.readAsStringSync(), 'Content: ${root}foo${sep}only_in_foo');
    expect(file.path, '${root}foo${sep}only_in_foo');
    expect(file.uri, Uri.parse('scheme:///only_in_foo'));
  });

  testWithoutContext('file with file system scheme URI', () {
    final File file = fs.file(Uri.parse('scheme:///only_in_foo'));
    expect(file.readAsStringSync(), 'Content: ${root}foo${sep}only_in_foo');
    expect(file.path, '${root}foo${sep}only_in_foo');
    expect(file.uri, Uri.parse('scheme:///only_in_foo'));
  });

  testWithoutContext('file in second root with file system scheme', () {
    final File file = fs.file('scheme:///only_in_bar');
    expect(file.readAsStringSync(), 'Content: ${root}bar${sep}only_in_bar');
    expect(file.path, '${root}bar${sep}only_in_bar');
    expect(file.uri, Uri.parse('scheme:///only_in_bar'));
  });

  testWithoutContext('file in second root with file system scheme URI', () {
    final File file = fs.file(Uri.parse('scheme:///only_in_bar'));
    expect(file.readAsStringSync(), 'Content: ${root}bar${sep}only_in_bar');
    expect(file.path, '${root}bar${sep}only_in_bar');
    expect(file.uri, Uri.parse('scheme:///only_in_bar'));
  });

  testWithoutContext('file in both roots', () {
    final File file = fs.file(Uri.parse('scheme:///in_both'));
    expect(file.readAsStringSync(), 'Content: ${root}foo${sep}in_both');
    expect(file.path, '${root}foo${sep}in_both');
    expect(file.uri, Uri.parse('scheme:///in_both'));
  });

  testWithoutContext('file with scheme in subdirectory', () {
    final File file = fs.file(Uri.parse('scheme:///subdir/in_subdir'));
    expect(file.readAsStringSync(), 'Content: ${root}foo${sep}subdir${sep}in_subdir');
    expect(file.path, '${root}foo${sep}subdir${sep}in_subdir');
    expect(file.uri, Uri.parse('scheme:///subdir/in_subdir'));
  });

  testWithoutContext('file in second root with scheme in subdirectory', () {
    final File file = fs.file(Uri.parse('scheme:///bar_subdir/in_subdir'));
    expect(file.readAsStringSync(), 'Content: ${root}bar${sep}bar_subdir${sep}in_subdir');
    expect(file.path, '${root}bar${sep}bar_subdir${sep}in_subdir');
    expect(file.uri, Uri.parse('scheme:///bar_subdir/in_subdir'));
  });

  testWithoutContext('non-existent file with scheme', () {
    final File file = fs.file(Uri.parse('scheme:///not_exist'));
    expect(file.uri, Uri.parse('scheme:///not_exist'));
    expect(file.path, '${root}foo${sep}not_exist');
  });

  testWithoutContext('stat', () async {
    expect((await fs.stat('${root}foo${sep}only_in_foo')).type, io.FileSystemEntityType.file);
    expect((await fs.stat('scheme:///only_in_foo')).type, io.FileSystemEntityType.file);
    expect(fs.statSync('${root}foo${sep}only_in_foo').type, io.FileSystemEntityType.file);
    expect(fs.statSync('scheme:///only_in_foo').type, io.FileSystemEntityType.file);
  });

  testWithoutContext('type', () async {
    expect(await fs.type('${root}foo${sep}only_in_foo'), io.FileSystemEntityType.file);
    expect(await fs.type('scheme:///only_in_foo'), io.FileSystemEntityType.file);
    expect(await fs.type('${root}foo${sep}subdir'), io.FileSystemEntityType.directory);
    expect(await fs.type('scheme:///subdir'), io.FileSystemEntityType.directory);
    expect(await fs.type('${root}foo${sep}not_found'), io.FileSystemEntityType.notFound);
    expect(await fs.type('scheme:///not_found'), io.FileSystemEntityType.notFound);

    expect(fs.typeSync('${root}foo${sep}only_in_foo'), io.FileSystemEntityType.file);
    expect(fs.typeSync('scheme:///only_in_foo'), io.FileSystemEntityType.file);
    expect(fs.typeSync('${root}foo${sep}subdir'), io.FileSystemEntityType.directory);
    expect(fs.typeSync('scheme:///subdir'), io.FileSystemEntityType.directory);
    expect(fs.typeSync('${root}foo${sep}not_found'), io.FileSystemEntityType.notFound);
    expect(fs.typeSync('scheme:///not_found'), io.FileSystemEntityType.notFound);
  });

  testWithoutContext('identical', () async {
    expect(await fs.identical('${root}foo${sep}in_both', '${root}foo${sep}in_both'), true);
    expect(await fs.identical('${root}foo${sep}in_both', 'scheme:///in_both'), true);
    expect(await fs.identical('${root}foo${sep}in_both', 'scheme:///in_both'), true);
    expect(await fs.identical('${root}bar${sep}in_both', 'scheme:///in_both'), false);

    expect(fs.identicalSync('${root}foo${sep}in_both', '${root}foo${sep}in_both'), true);
    expect(fs.identicalSync('${root}foo${sep}in_both', 'scheme:///in_both'), true);
    expect(fs.identicalSync('${root}foo${sep}in_both', 'scheme:///in_both'), true);
    expect(fs.identicalSync('${root}bar${sep}in_both', 'scheme:///in_both'), false);
  });
}
