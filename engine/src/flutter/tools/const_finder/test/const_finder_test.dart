// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' as io;

import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:kernel/const_finder.dart';
import 'package:path/path.dart' as path;
import 'package:test/test.dart';

void main() {
  final engine = Engine.findWithin();
  var buildDir = io.Platform.environment['FLUTTER_BUILD_DIRECTORY'];
  buildDir ??= engine.latestOutput()?.path.path;
  if (buildDir == null) {
    fail('No build directory found. Set FLUTTER_BUILD_DIRECTORY');
  }

  final fixturesPath = path.join(
    engine.flutterDir.path,
    'tools',
    'const_finder',
    'test',
    'fixtures',
  );
  final String fixturesUrl =
      io.Platform.isWindows
          ? '/$fixturesPath'.replaceAll(io.Platform.pathSeparator, '/')
          : fixturesPath;

  final frontendServerSnapshot = path.join(buildDir, 'gen', 'frontend_server_aot.dart.snapshot');
  final flutterPatchedSdk = path.join(buildDir, 'flutter_patched_sdk');
  final librariesDotJson = path.join(flutterPatchedSdk, 'lib', 'libraries.json');
  final String packageConfig = path.join(fixturesPath, '.dart_tool', 'package_config.json');

  final dart = io.Platform.resolvedExecutable;
  final dartaotruntime = path.join(path.dirname(io.Platform.resolvedExecutable), 'dartaotruntime');

  void compileAOTDill({required String sourcePath, required String dillPath}) {
    final result = io.Process.runSync(dartaotruntime, [
      frontendServerSnapshot,
      '--sdk-root=$flutterPatchedSdk',
      '--target=flutter',
      '--aot',
      '--tfa',
      '--packages=$packageConfig',
      '--output-dill=$dillPath',
      sourcePath,
    ]);
    printOnFailure(result.stdout.toString());
    printOnFailure(result.stderr.toString());
    if (result.exitCode != 0) {
      fail('Failed to compile AOT dill');
    }
    addTearDown(() => io.File(dillPath).deleteSync());
  }

  void compileDart2JSDill({required String sourcePath, required String dillPath}) {
    final result = io.Process.runSync(dart, [
      'compile',
      'js',
      '--libraries-spec=$librariesDotJson',
      '-Ddart.vm.product=true',
      '-o',
      dillPath,
      '--packages=$packageConfig',
      '--cfe-only',
      sourcePath,
    ]);
    printOnFailure(result.stdout.toString());
    printOnFailure(result.stderr.toString());
    if (result.exitCode != 0) {
      fail('Failed to compile Dart2JS dill');
    }
    addTearDown(() => io.File(dillPath).deleteSync());
  }

  test('box_frontend (aot)', () {
    final sourcePath = path.join(fixturesPath, 'lib', 'box.dart');
    final dillPath = path.join(fixturesPath, 'box_frontend.dill');
    compileAOTDill(sourcePath: sourcePath, dillPath: dillPath);
    final finder = ConstFinder(
      kernelFilePath: dillPath,
      classLibraryUri: 'package:const_finder_fixtures/box.dart',
      className: 'Box',
    );

    // Will timeout if we did things wrong.
    jsonEncode(finder.findInstances());
  });

  test('box_web (dart2js)', () {
    final sourcePath = path.join(fixturesPath, 'lib', 'box.dart');
    final dillPath = path.join(fixturesPath, 'box_web.dill');
    compileDart2JSDill(sourcePath: sourcePath, dillPath: dillPath);
    final finder = ConstFinder(
      kernelFilePath: dillPath,
      classLibraryUri: 'package:const_finder_fixtures/box.dart',
      className: 'Box',
    );

    // Will timeout if we did things wrong.
    jsonEncode(finder.findInstances());
  });

  test('consts_frontend (aot)', () {
    final sourcePath = path.join(fixturesPath, 'lib', 'consts.dart');
    final dillPath = path.join(fixturesPath, 'consts_frontend.dill');
    compileAOTDill(sourcePath: sourcePath, dillPath: dillPath);

    final {
      'constantInstances': List<Object?> constantInstances,
      'nonConstantLocations': List<Object?> nonConstantLocations,
    } = ConstFinder(
          kernelFilePath: dillPath,
          classLibraryUri: 'package:const_finder_fixtures/target.dart',
          className: 'Target',
        ).findInstances();

    expect(
      constantInstances,
      unorderedEquals([
        {'stringValue': '100', 'intValue': 100, 'targetValue': null},
        {'stringValue': '102', 'intValue': 102, 'targetValue': null},
        {'stringValue': '101', 'intValue': 101},
        {'stringValue': '103', 'intValue': 103, 'targetValue': null},
        {'stringValue': '105', 'intValue': 105, 'targetValue': null},
        {'stringValue': '104', 'intValue': 104},
        {'stringValue': '106', 'intValue': 106, 'targetValue': null},
        {'stringValue': '108', 'intValue': 108, 'targetValue': null},
        {'stringValue': '107', 'intValue': 107},
        {'stringValue': '1', 'intValue': 1, 'targetValue': null},
        {'stringValue': '4', 'intValue': 4, 'targetValue': null},
        {'stringValue': '2', 'intValue': 2},
        {'stringValue': '6', 'intValue': 6, 'targetValue': null},
        {'stringValue': '8', 'intValue': 8, 'targetValue': null},
        {'stringValue': '10', 'intValue': 10, 'targetValue': null},
        {'stringValue': '9', 'intValue': 9},
        {'stringValue': '7', 'intValue': 7, 'targetValue': null},
        {'stringValue': '11', 'intValue': 11, 'targetValue': null},
        {'stringValue': '12', 'intValue': 12, 'targetValue': null},
        {'stringValue': 'package', 'intValue': -1, 'targetValue': null},
      ]),
    );
    expect(nonConstantLocations, isEmpty);
  });

  test('consts_web (dart2js)', () {
    final sourcePath = path.join(fixturesPath, 'lib', 'consts.dart');
    final dillPath = path.join(fixturesPath, 'consts_web.dill');
    compileDart2JSDill(sourcePath: sourcePath, dillPath: dillPath);

    final {
      'constantInstances': List<Object?> constantInstances,
      'nonConstantLocations': List<Object?> nonConstantLocations,
    } = ConstFinder(
          kernelFilePath: dillPath,
          classLibraryUri: 'package:const_finder_fixtures/target.dart',
          className: 'Target',
        ).findInstances();

    expect(
      constantInstances,
      unorderedEquals([
        {'stringValue': '100', 'intValue': 100, 'targetValue': null},
        {'stringValue': '102', 'intValue': 102, 'targetValue': null},
        {'stringValue': '101', 'intValue': 101},
        {'stringValue': '103', 'intValue': 103, 'targetValue': null},
        {'stringValue': '105', 'intValue': 105, 'targetValue': null},
        {'stringValue': '104', 'intValue': 104},
        {'stringValue': '106', 'intValue': 106, 'targetValue': null},
        {'stringValue': '108', 'intValue': 108, 'targetValue': null},
        {'stringValue': '107', 'intValue': 107},
        {'stringValue': '1', 'intValue': 1, 'targetValue': null},
        {'stringValue': '4', 'intValue': 4, 'targetValue': null},
        {'stringValue': '2', 'intValue': 2},
        {'stringValue': '6', 'intValue': 6, 'targetValue': null},
        {'stringValue': '8', 'intValue': 8, 'targetValue': null},
        {'stringValue': '10', 'intValue': 10, 'targetValue': null},
        {'stringValue': '9', 'intValue': 9},
        {'stringValue': '7', 'intValue': 7, 'targetValue': null},
        {'stringValue': '11', 'intValue': 11, 'targetValue': null},
        {'stringValue': '12', 'intValue': 12, 'targetValue': null},
        {'stringValue': 'package', 'intValue': -1, 'targetValue': null},
      ]),
    );

    expect(nonConstantLocations, [
      {'file': 'file://$fixturesUrl/pkg/package.dart', 'line': 14, 'column': 25},
    ]);
  });

  test('consts_and_non_frontend (aot)', () {
    final sourcePath = path.join(fixturesPath, 'lib', 'consts_and_non.dart');
    final dillPath = path.join(fixturesPath, 'consts_and_non_frontend.dill');
    compileAOTDill(sourcePath: sourcePath, dillPath: dillPath);

    final {
      'constantInstances': List<Object?> constantInstances,
      'nonConstantLocations': List<Object?> nonConstantLocations,
    } = ConstFinder(
          kernelFilePath: dillPath,
          classLibraryUri: 'package:const_finder_fixtures/target.dart',
          className: 'Target',
        ).findInstances();

    expect(
      constantInstances,
      unorderedEquals([
        {'stringValue': '1', 'intValue': 1, 'targetValue': null},
        {'stringValue': '4', 'intValue': 4, 'targetValue': null},
        {'stringValue': '6', 'intValue': 6, 'targetValue': null},
        {'stringValue': '8', 'intValue': 8, 'targetValue': null},
        {'stringValue': '10', 'intValue': 10, 'targetValue': null},
        {'stringValue': '9', 'intValue': 9},
        {'stringValue': '7', 'intValue': 7, 'targetValue': null},
      ]),
    );
    expect(nonConstantLocations, [
      {'file': 'file://$fixturesUrl/lib/consts_and_non.dart', 'line': 14, 'column': 26},
      {'file': 'file://$fixturesUrl/lib/consts_and_non.dart', 'line': 16, 'column': 26},
      {'file': 'file://$fixturesUrl/lib/consts_and_non.dart', 'line': 16, 'column': 41},
      {'file': 'file://$fixturesUrl/lib/consts_and_non.dart', 'line': 17, 'column': 26},
      {'file': 'file://$fixturesUrl/pkg/package.dart', 'line': 14, 'column': 25},
    ]);
  });

  test('consts_and_non_web (dart2js)', () {
    final sourcePath = path.join(fixturesPath, 'lib', 'consts_and_non.dart');
    final dillPath = path.join(fixturesPath, 'consts_and_non_web.dill');
    compileDart2JSDill(sourcePath: sourcePath, dillPath: dillPath);

    final {
      'constantInstances': List<Object?> constantInstances,
      'nonConstantLocations': List<Object?> nonConstantLocations,
    } = ConstFinder(
          kernelFilePath: dillPath,
          classLibraryUri: 'package:const_finder_fixtures/target.dart',
          className: 'Target',
        ).findInstances();

    expect(
      constantInstances,
      unorderedEquals([
        {'stringValue': '1', 'intValue': 1, 'targetValue': null},
        {'stringValue': '4', 'intValue': 4, 'targetValue': null},
        {'stringValue': '6', 'intValue': 6, 'targetValue': null},
        {'stringValue': '8', 'intValue': 8, 'targetValue': null},
        {'stringValue': '10', 'intValue': 10, 'targetValue': null},
        {'stringValue': '9', 'intValue': 9},
        {'stringValue': '7', 'intValue': 7, 'targetValue': null},
        {'stringValue': 'package', 'intValue': -1, 'targetValue': null},
      ]),
    );

    expect(nonConstantLocations, [
      {'file': 'file://$fixturesUrl/lib/consts_and_non.dart', 'line': 14, 'column': 26},
      {'file': 'file://$fixturesUrl/lib/consts_and_non.dart', 'line': 16, 'column': 26},
      {'file': 'file://$fixturesUrl/lib/consts_and_non.dart', 'line': 16, 'column': 41},
      {'file': 'file://$fixturesUrl/lib/consts_and_non.dart', 'line': 17, 'column': 26},
      {'file': 'file://$fixturesUrl/pkg/package.dart', 'line': 14, 'column': 25},
    ]);
  });

  test('static_icon_provider_frontend (aot)', () {
    final sourcePath = path.join(fixturesPath, 'lib', 'static_icon_provider.dart');
    final dillPath = path.join(fixturesPath, 'static_icon_provider_frontend.dill');
    compileAOTDill(sourcePath: sourcePath, dillPath: dillPath);
    final finder = ConstFinder(
      kernelFilePath: dillPath,
      classLibraryUri: 'package:const_finder_fixtures/target.dart',
      className: 'Target',
      annotationClassName: 'StaticIconProvider',
      annotationClassLibraryUri: 'package:const_finder_fixtures/static_icon_provider.dart',
    );

    final {
      'constantInstances': List<Object?> constantInstances,
      'nonConstantLocations': List<Object?> nonConstantLocations,
    } = finder.findInstances();
    expect(
      constantInstances,
      unorderedEquals([
        {'stringValue': 'used1', 'intValue': 1, 'targetValue': null},
        {'stringValue': 'used2', 'intValue': 2, 'targetValue': null},
      ]),
    );

    // TODO(fujino): This should have non-constant locations from the use of
    // a tear-off, see https://github.com/flutter/flutter/issues/116797
    expect(nonConstantLocations, isEmpty);
  });

  test('static_icon_provider_web (dart2js)', () {
    final sourcePath = path.join(fixturesPath, 'lib', 'static_icon_provider.dart');
    final dillPath = path.join(fixturesPath, 'static_icon_provider_web.dill');
    compileDart2JSDill(sourcePath: sourcePath, dillPath: dillPath);
    final finder = ConstFinder(
      kernelFilePath: dillPath,
      classLibraryUri: 'package:const_finder_fixtures/target.dart',
      className: 'Target',
      annotationClassName: 'StaticIconProvider',
      annotationClassLibraryUri: 'package:const_finder_fixtures/static_icon_provider.dart',
    );

    final {
      'constantInstances': List<Object?> constantInstances,
      'nonConstantLocations': List<Object?> nonConstantLocations,
    } = finder.findInstances();
    expect(
      constantInstances,
      unorderedEquals([
        {'stringValue': 'used1', 'intValue': 1, 'targetValue': null},
        {'stringValue': 'used2', 'intValue': 2, 'targetValue': null},
      ]),
    );

    // TODO(fujino): This should have non-constant locations from the use of
    // a tear-off, see https://github.com/flutter/flutter/issues/116797
    expect(nonConstantLocations, isEmpty);
  });
}
