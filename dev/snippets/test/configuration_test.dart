// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:snippets/snippets.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

void main() {
  group('Configuration', () {
    final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
    late SnippetConfiguration config;

    setUp(() {
      config = FlutterRepoSnippetConfiguration(
        flutterRoot: memoryFileSystem.directory('/flutter sdk'),
        filesystem: memoryFileSystem,
      );
    });
    test('config directory is correct', () async {
      expect(
        config.configDirectory.path,
        matches(RegExp(r'[/\\]flutter sdk[/\\]dev[/\\]snippets[/\\]config')),
      );
    });
    test('skeleton directory is correct', () async {
      expect(
        config.skeletonsDirectory.path,
        matches(RegExp(r'[/\\]flutter sdk[/\\]dev[/\\]snippets[/\\]config[/\\]skeletons')),
      );
    });
    test('html skeleton file for sample is correct', () async {
      expect(
        config.getHtmlSkeletonFile('snippet').path,
        matches(
          RegExp(
            r'[/\\]flutter sdk[/\\]dev[/\\]snippets[/\\]config[/\\]skeletons[/\\]snippet.html',
          ),
        ),
      );
    });
    test('html skeleton file for app with no dartpad is correct', () async {
      expect(
        config.getHtmlSkeletonFile('sample').path,
        matches(
          RegExp(r'[/\\]flutter sdk[/\\]dev[/\\]snippets[/\\]config[/\\]skeletons[/\\]sample.html'),
        ),
      );
    });
    test('html skeleton file for app with dartpad is correct', () async {
      expect(
        config.getHtmlSkeletonFile('dartpad').path,
        matches(
          RegExp(
            r'[/\\]flutter sdk[/\\]dev[/\\]snippets[/\\]config[/\\]skeletons[/\\]dartpad-sample.html',
          ),
        ),
      );
    });
  });
}
