// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/ide_config.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/template.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  group('ide_config', () {
    late Directory tempDir;
    late Directory templateDir;
    late Directory intellijDir;
    late Directory toolsDir;

    Map<String, String> getFilesystemContents([Directory? root]) {
      final String tempPath = tempDir.absolute.path;
      final List<String> paths =
          (root ?? tempDir).listSync(recursive: true).map((FileSystemEntity entity) {
            final String relativePath = globals.fs.path.relative(entity.path, from: tempPath);
            return relativePath;
          }).toList();
      final Map<String, String> contents = <String, String>{};
      for (final String path in paths) {
        final String absPath = globals.fs.path.join(tempPath, path);
        if (globals.fs.isDirectorySync(absPath)) {
          contents[path] = 'dir';
        } else if (globals.fs.isFileSync(absPath)) {
          contents[path] = globals.fs.file(absPath).readAsStringSync();
        }
      }
      return contents;
    }

    Map<String, String> getManifest(Directory base, String marker, {bool isTemplate = false}) {
      final String basePath = globals.fs.path.relative(base.path, from: tempDir.absolute.path);
      final String suffix = isTemplate ? Template.copyTemplateExtension : '';
      return <String, String>{
        globals.fs.path.join(basePath, '.idea'): 'dir',
        globals.fs.path.join(basePath, '.idea', 'modules.xml$suffix'): 'modules $marker',
        globals.fs.path.join(basePath, '.idea', 'vcs.xml$suffix'): 'vcs $marker',
        globals.fs.path.join(basePath, '.idea', '.name$suffix'): 'codeStyleSettings $marker',
        globals.fs.path.join(basePath, '.idea', 'runConfigurations'): 'dir',
        globals.fs.path.join(basePath, '.idea', 'runConfigurations', 'hello_world.xml$suffix'):
            'hello_world $marker',
        globals.fs.path.join(basePath, 'flutter.iml$suffix'): 'flutter $marker',
        globals.fs.path.join(basePath, 'packages', 'new', 'deep.iml$suffix'): 'deep $marker',
        globals.fs.path.join(basePath, 'example', 'gallery', 'android.iml$suffix'):
            'android $marker',
      };
    }

    void populateDir(Map<String, String> manifest) {
      for (final String key in manifest.keys) {
        if (manifest[key] == 'dir') {
          tempDir.childDirectory(key).createSync(recursive: true);
        }
      }
      for (final String key in manifest.keys) {
        if (manifest[key] != 'dir') {
          tempDir.childFile(key)
            ..createSync(recursive: true)
            ..writeAsStringSync(manifest[key]!);
        }
      }
    }

    bool fileOrDirectoryExists(String path) {
      final String absPath = globals.fs.path.join(tempDir.absolute.path, path);
      return globals.fs.file(absPath).existsSync() || globals.fs.directory(absPath).existsSync();
    }

    Future<void> updateIdeConfig({
      Directory? dir,
      List<String> args = const <String>[],
      Map<String, String> expectedContents = const <String, String>{},
      List<String> unexpectedPaths = const <String>[],
    }) async {
      dir ??= tempDir;
      Cache.flutterRoot = tempDir.absolute.path;
      final IdeConfigCommand command = IdeConfigCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>['ide-config', ...args]);

      for (final String path in expectedContents.keys) {
        final String absPath = globals.fs.path.join(tempDir.absolute.path, path);
        expect(
          fileOrDirectoryExists(globals.fs.path.join(dir.path, path)),
          true,
          reason: "$path doesn't exist",
        );
        if (globals.fs.file(absPath).existsSync()) {
          expect(
            globals.fs.file(absPath).readAsStringSync(),
            equals(expectedContents[path]),
            reason: "$path contents don't match",
          );
        }
      }
      for (final String path in unexpectedPaths) {
        expect(
          fileOrDirectoryExists(globals.fs.path.join(dir.path, path)),
          false,
          reason: '$path exists',
        );
      }
    }

    setUpAll(() {
      Cache.disableLocking();
    });

    setUp(() {
      tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_ide_config_test.');
      final Directory packagesDir = tempDir.childDirectory('packages')..createSync(recursive: true);
      toolsDir = packagesDir.childDirectory('flutter_tools')..createSync();
      templateDir = toolsDir.childDirectory('ide_templates')..createSync();
      intellijDir = templateDir.childDirectory('intellij')..createSync();
    });

    tearDown(() {
      tryToDelete(tempDir);
    });

    testUsingContext("doesn't touch existing files without --overwrite", () async {
      final Map<String, String> templateManifest = getManifest(
        intellijDir,
        'template',
        isTemplate: true,
      );
      final Map<String, String> flutterManifest = getManifest(tempDir, 'existing');
      populateDir(templateManifest);
      populateDir(flutterManifest);
      final Map<String, String> expectedContents = getFilesystemContents();
      return updateIdeConfig(expectedContents: expectedContents);
    });

    testUsingContext('creates non-existent files', () async {
      final Map<String, String> templateManifest = getManifest(
        intellijDir,
        'template',
        isTemplate: true,
      );
      final Map<String, String> flutterManifest = getManifest(tempDir, 'template');
      populateDir(templateManifest);
      final Map<String, String> expectedContents = <String, String>{
        ...templateManifest,
        ...flutterManifest,
      };
      return updateIdeConfig(expectedContents: expectedContents);
    });

    testUsingContext('overwrites existing files with --overwrite', () async {
      final Map<String, String> templateManifest = getManifest(
        intellijDir,
        'template',
        isTemplate: true,
      );
      final Map<String, String> flutterManifest = getManifest(tempDir, 'existing');
      populateDir(templateManifest);
      populateDir(flutterManifest);
      final Map<String, String> overwrittenManifest = getManifest(tempDir, 'template');
      final Map<String, String> expectedContents = <String, String>{
        ...templateManifest,
        ...overwrittenManifest,
      };
      return updateIdeConfig(args: <String>['--overwrite'], expectedContents: expectedContents);
    });

    testUsingContext('only adds new templates without --overwrite', () async {
      final Map<String, String> templateManifest = getManifest(
        intellijDir,
        'template',
        isTemplate: true,
      );
      final String flutterIml = globals.fs.path.join(
        'packages',
        'flutter_tools',
        'ide_templates',
        'intellij',
        'flutter.iml${Template.copyTemplateExtension}',
      );
      templateManifest.remove(flutterIml);
      populateDir(templateManifest);
      templateManifest[flutterIml] = 'flutter existing';
      final Map<String, String> flutterManifest = getManifest(tempDir, 'existing');
      populateDir(flutterManifest);
      final Map<String, String> expectedContents = <String, String>{
        ...flutterManifest,
        ...templateManifest,
      };
      return updateIdeConfig(
        args: <String>['--update-templates'],
        expectedContents: expectedContents,
      );
    });

    testUsingContext('update all templates with --overwrite', () async {
      final Map<String, String> templateManifest = getManifest(
        intellijDir,
        'template',
        isTemplate: true,
      );
      populateDir(templateManifest);
      final Map<String, String> flutterManifest = getManifest(tempDir, 'existing');
      populateDir(flutterManifest);
      final Map<String, String> updatedTemplates = getManifest(
        intellijDir,
        'existing',
        isTemplate: true,
      );
      final Map<String, String> expectedContents = <String, String>{
        ...flutterManifest,
        ...updatedTemplates,
      };
      return updateIdeConfig(
        args: <String>['--update-templates', '--overwrite'],
        expectedContents: expectedContents,
      );
    });

    testUsingContext('removes deleted imls with --overwrite', () async {
      final Map<String, String> templateManifest = getManifest(
        intellijDir,
        'template',
        isTemplate: true,
      );
      populateDir(templateManifest);
      final Map<String, String> flutterManifest = getManifest(tempDir, 'existing');
      flutterManifest.remove('flutter.iml');
      populateDir(flutterManifest);
      final Map<String, String> updatedTemplates = getManifest(
        intellijDir,
        'existing',
        isTemplate: true,
      );
      final String flutterIml = globals.fs.path.join(
        'packages',
        'flutter_tools',
        'ide_templates',
        'intellij',
        'flutter.iml${Template.copyTemplateExtension}',
      );
      updatedTemplates.remove(flutterIml);
      final Map<String, String> expectedContents = <String, String>{
        ...flutterManifest,
        ...updatedTemplates,
      };
      return updateIdeConfig(
        args: <String>['--update-templates', '--overwrite'],
        expectedContents: expectedContents,
      );
    });

    testUsingContext(
      'removes deleted imls with --overwrite, including empty parent dirs',
      () async {
        final Map<String, String> templateManifest = getManifest(
          intellijDir,
          'template',
          isTemplate: true,
        );
        populateDir(templateManifest);
        final Map<String, String> flutterManifest = getManifest(tempDir, 'existing');
        flutterManifest.remove(globals.fs.path.join('packages', 'new', 'deep.iml'));
        populateDir(flutterManifest);
        final Map<String, String> updatedTemplates = getManifest(
          intellijDir,
          'existing',
          isTemplate: true,
        );
        String deepIml = globals.fs.path.join(
          'packages',
          'flutter_tools',
          'ide_templates',
          'intellij',
        );
        // Remove the all the dir entries too.
        updatedTemplates.remove(deepIml);
        deepIml = globals.fs.path.join(deepIml, 'packages');
        updatedTemplates.remove(deepIml);
        deepIml = globals.fs.path.join(deepIml, 'new');
        updatedTemplates.remove(deepIml);
        deepIml = globals.fs.path.join(deepIml, 'deep.iml');
        updatedTemplates.remove(deepIml);
        final Map<String, String> expectedContents = <String, String>{
          ...flutterManifest,
          ...updatedTemplates,
        };
        return updateIdeConfig(
          args: <String>['--update-templates', '--overwrite'],
          expectedContents: expectedContents,
        );
      },
    );
  });
}
