// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_data/project.dart';
import 'test_utils.dart';

final class _AssetTransformationTestProject extends Project {
  @override
  final String pubspec = '''
name: test
environment:
  sdk: ^3.7.0-0
dependencies:
  flutter:
    sdk: flutter
dev_dependencies:
  capitalizer_transformer:
    path: ./capitalizer_transformer
flutter:
  assets:
    - path: assets/text_asset.txt
      transformers:
        - package: capitalizer_transformer
''';

  @override
  final String main = '''
import 'package:flutter/material.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      home: Scaffold(
        body: Center(
          child: Text('Hello, World!'),
        ),
      ),
    );
  }
}
''';
}

Future<void> main() async {
  testWithoutContext('asset is transformed when declared with a transformation', () async {
    final Directory tempProjectDirectory = createResolvedTempDirectorySync(
      'asset_transformation_test.',
    );

    try {
      _setUpCapitalizerTransformer(tempProjectDirectory);
      await _AssetTransformationTestProject().setUpIn(tempProjectDirectory);
      tempProjectDirectory.childDirectory('assets').childFile('text_asset.txt')
        ..createSync(recursive: true)
        ..writeAsStringSync('abc');

      final ProcessResult result = await processManager.run(<String>[
        flutterBin,
        'build',
        'web',
      ], workingDirectory: tempProjectDirectory.path);

      expect(result.exitCode, 0, reason: result.stderr as String);

      final File asset = fileSystem.file(
        fileSystem.path.join(
          tempProjectDirectory.path,
          'build',
          'web',
          'assets',
          'assets',
          'text_asset.txt',
        ),
      );

      expect(asset, exists);

      expect(
        asset.readAsStringSync(),
        equals('ABC'),
        reason:
            "The original contents of the asset (which should be 'abc') should "
            "have been transformed to 'ABC' by the capitalizer_transformer as "
            'configured in the pubspec.',
      );
    } finally {
      tryToDelete(tempProjectDirectory);
    }
  });
}

void _setUpCapitalizerTransformer(Directory projectDir) {
  final Directory targetDir = projectDir.childDirectory('capitalizer_transformer');
  targetDir.createSync(recursive: true);

  targetDir.childFile('pubspec.yaml')
    ..createSync()
    ..writeAsStringSync('''
name: capitalizer_transformer
version: 1.0.0

environment:
  sdk: ^3.7.0-0

dependencies:
  args: ^2.4.2
''');

  targetDir.childDirectory('bin').childFile('capitalizer_transformer.dart')
    ..createSync(recursive: true)
    ..writeAsStringSync('''
import 'dart:io';

import 'package:args/args.dart';

void main(List<String> args) {
  final ArgParser parser = ArgParser()
    ..addOption('input')
    ..addOption('output');

  final ArgResults parsedArgs = parser.parse(args);

  final String inputFilePath = parsedArgs['input'] as String;
  final String outputFilePath = parsedArgs['output'] as String;

  final String input = File(inputFilePath).readAsStringSync();
  File(outputFilePath)
    ..createSync(recursive: true)
    ..writeAsStringSync(input.toUpperCase());
}
''');
}
