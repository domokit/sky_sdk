// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:archive/archive.dart';
import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/intellij/intellij.dart';

import '../../src/common.dart';

void main() {
  late FileSystem fileSystem;

  void writeFileCreatingDirectories(String path, List<int> bytes) {
    final File file = fileSystem.file(path);
    file.parent.createSync(recursive: true);
    file.writeAsBytesSync(bytes);
  }

  setUp(() {
    fileSystem = MemoryFileSystem.test();
  });

  testWithoutContext('IntelliJPlugins found', () async {
    final IntelliJPlugins plugins = IntelliJPlugins(_kPluginsPath, fileSystem: fileSystem);

    final Archive dartJarArchive = buildSingleFileArchive('META-INF/plugin.xml', r'''
<idea-plugin version="2">
<name>Dart</name>
<version>162.2485</version>
</idea-plugin>
''');
    writeFileCreatingDirectories(
      fileSystem.path.join(_kPluginsPath, 'Dart', 'lib', 'Dart.jar'),
      ZipEncoder().encode(dartJarArchive)!,
    );

    final Archive flutterJarArchive = buildSingleFileArchive('META-INF/plugin.xml', r'''
<idea-plugin version="2">
<name>Flutter</name>
<version>0.1.3</version>
</idea-plugin>
''');
    writeFileCreatingDirectories(
      fileSystem.path.join(_kPluginsPath, 'flutter-intellij.jar'),
      ZipEncoder().encode(flutterJarArchive)!,
    );

    final List<ValidationMessage> messages = <ValidationMessage>[];
    plugins.validatePackage(messages, <String>['Dart', 'dart'], 'Dart', 'download-Dart');
    plugins.validatePackage(
      messages,
      <String>['flutter-intellij', 'flutter-intellij.jar'],
      'Flutter',
      'download-Flutter',
      minVersion: IntelliJPlugins.kMinFlutterPluginVersion,
    );

    ValidationMessage message = messages.firstWhere(
      (ValidationMessage m) => m.message.startsWith('Dart '),
    );
    expect(message.message, 'Dart plugin version 162.2485');

    message = messages.firstWhere((ValidationMessage m) => m.message.startsWith('Flutter '));
    expect(message.message, contains('Flutter plugin version 0.1.3'));
    expect(message.message, contains('recommended minimum version'));
  });

  testWithoutContext(
    'IntelliJPlugins can read the package version of the flutter-intellij 50.0+/IntelliJ 2020.2+ layout',
    () async {
      final IntelliJPlugins plugins = IntelliJPlugins(_kPluginsPath, fileSystem: fileSystem);

      final Archive flutterIdeaJarArchive = buildSingleFileArchive('META-INF/plugin.xml', r'''
<idea-plugin version="2">
<name>Flutter</name>
<version>50.0</version>
</idea-plugin>
''');
      writeFileCreatingDirectories(
        fileSystem.path.join(_kPluginsPath, 'flutter-intellij', 'lib', 'flutter-idea-50.0.jar'),
        ZipEncoder().encode(flutterIdeaJarArchive)!,
      );
      final Archive flutterIntellijJarArchive = buildSingleFileArchive('META-INF/MANIFEST.MF', r'''
Manifest-Version: 1.0
''');
      writeFileCreatingDirectories(
        fileSystem.path.join(_kPluginsPath, 'flutter-intellij', 'lib', 'flutter-intellij-50.0.jar'),
        ZipEncoder().encode(flutterIntellijJarArchive)!,
      );

      final List<ValidationMessage> messages = <ValidationMessage>[];
      plugins.validatePackage(
        messages,
        <String>['flutter-intellij', 'flutter-intellij.jar'],
        'Flutter',
        'download-Flutter',
        minVersion: IntelliJPlugins.kMinFlutterPluginVersion,
      );

      final ValidationMessage message = messages.firstWhere(
        (ValidationMessage m) => m.message.startsWith('Flutter '),
      );
      expect(message.message, contains('Flutter plugin version 50.0'));
    },
  );

  testWithoutContext(
    'IntelliJPlugins can read the package version of the flutter-intellij 50.0+/IntelliJ 2020.2+ layout(priority is given to packages with the same prefix as packageName)',
    () async {
      final IntelliJPlugins plugins = IntelliJPlugins(_kPluginsPath, fileSystem: fileSystem);

      final Archive flutterIdeaJarArchive = buildSingleFileArchive('META-INF/plugin.xml', r'''
<idea-plugin version="2">
<name>Flutter</name>
<version>50.0</version>
</idea-plugin>
''');
      writeFileCreatingDirectories(
        fileSystem.path.join(_kPluginsPath, 'flutter-intellij', 'lib', 'flutter-idea-50.0.jar'),
        ZipEncoder().encode(flutterIdeaJarArchive)!,
      );
      final Archive flutterIntellijJarArchive = buildSingleFileArchive('META-INF/plugin.xml', r'''
<idea-plugin version="2">
<name>Flutter</name>
<version>51.0</version>
</idea-plugin>
''');
      writeFileCreatingDirectories(
        fileSystem.path.join(_kPluginsPath, 'flutter-intellij', 'lib', 'flutter-intellij-50.0.jar'),
        ZipEncoder().encode(flutterIntellijJarArchive)!,
      );

      final List<ValidationMessage> messages = <ValidationMessage>[];
      plugins.validatePackage(
        messages,
        <String>['flutter-intellij', 'flutter-intellij.jar'],
        'Flutter',
        'download-Flutter',
        minVersion: IntelliJPlugins.kMinFlutterPluginVersion,
      );

      final ValidationMessage message = messages.firstWhere(
        (ValidationMessage m) => m.message.startsWith('Flutter '),
      );
      expect(message.message, contains('Flutter plugin version 51.0'));
    },
  );

  testWithoutContext('IntelliJPlugins not found displays a link to their download site', () async {
    final IntelliJPlugins plugins = IntelliJPlugins(_kPluginsPath, fileSystem: fileSystem);

    final List<ValidationMessage> messages = <ValidationMessage>[];
    plugins.validatePackage(messages, <String>['Dart', 'dart'], 'Dart', 'download-Dart');
    plugins.validatePackage(
      messages,
      <String>['flutter-intellij', 'flutter-intellij.jar'],
      'Flutter',
      'download-Flutter',
      minVersion: IntelliJPlugins.kMinFlutterPluginVersion,
    );

    ValidationMessage message = messages.firstWhere(
      (ValidationMessage m) => m.message.startsWith('Dart '),
    );
    expect(message.message, contains('Dart plugin can be installed from'));
    expect(message.contextUrl, isNotNull);

    message = messages.firstWhere((ValidationMessage m) => m.message.startsWith('Flutter '));
    expect(message.message, contains('Flutter plugin can be installed from'));
    expect(message.contextUrl, isNotNull);
  });

  testWithoutContext('IntelliJPlugins does not crash if no plugin file found', () async {
    final IntelliJPlugins plugins = IntelliJPlugins(_kPluginsPath, fileSystem: fileSystem);

    final Archive dartJarArchive = buildSingleFileArchive('META-INF/MANIFEST.MF', r'''
Manifest-Version: 1.0
''');
    writeFileCreatingDirectories(
      fileSystem.path.join(_kPluginsPath, 'Dart', 'lib', 'Other.jar'),
      ZipEncoder().encode(dartJarArchive)!,
    );

    expect(
      () => plugins.validatePackage(
        <ValidationMessage>[],
        <String>['Dart', 'dart'],
        'Dart',
        'download-Dart',
      ),
      returnsNormally,
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/163214
  testWithoutContext(
    'IntelliJPlugins can find the Dart plugin with a lowercase package name',
    () async {
      final IntelliJPlugins plugins = IntelliJPlugins(_kPluginsPath, fileSystem: fileSystem);
      final Archive dartJarArchive = buildSingleFileArchive('META-INF/plugin.xml', r'''
<idea-plugin>
  <name>Dart</name>
  <version>242.24931</version>
</idea-plugin>''');
      writeFileCreatingDirectories(
        fileSystem.path.join(_kPluginsPath, 'dart', 'lib', 'dart.jar'),
        ZipEncoder().encode(dartJarArchive)!,
      );

      final List<ValidationMessage> messages = <ValidationMessage>[];
      plugins.validatePackage(messages, <String>['Dart', 'dart'], 'Dart', 'download-Dart');

      expect(messages.length, equals(1));
      expect(messages.single.message, equals('Dart plugin version 242.24931'));
    },
  );
}

const String _kPluginsPath = '/data/intellij/plugins';

Archive buildSingleFileArchive(String path, String content) {
  final Archive archive = Archive();

  final List<int> bytes = utf8.encode(content);
  archive.addFile(ArchiveFile(path, bytes.length, bytes));

  return archive;
}
