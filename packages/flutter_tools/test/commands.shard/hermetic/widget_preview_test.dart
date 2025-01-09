// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/commands/widget_preview.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/test.dart';
import 'package:test_api/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';

void main() {
  group('WidgetPreviewStartCommand', () {
    late MemoryFileSystem fileSystem;
    late ProcessManager processManager;
    late WidgetPreviewStartCommand command;
    late FlutterProject rootProject;

    setUp(() {
      fileSystem = MemoryFileSystem.test();
      processManager = FakeProcessManager.any();
      command = WidgetPreviewStartCommand();
      rootProject = FakeFlutterProject(
        projectRoot: 'some_project',
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
      );
    });

    testUsingContext(
      'can create a pubspec.yaml for the preview scaffold including root project assets',
      () {
        final FlutterManifest root = rootProject.manifest;
        final FlutterManifest updated = command.buildPubspec(
          rootManifest: rootProject.manifest,
          widgetPreviewManifest: rootProject.widgetPreviewScaffoldProject.manifest,
        );

        final List<AssetsEntry> rootAssets = root.assets;
        final List<AssetsEntry> updatedAssets = updated.assets;
        expect(updatedAssets.length, rootAssets.length);
        for (int i = 0; i < rootAssets.length; ++i) {
          final AssetsEntry rootEntry = rootAssets[i];
          final AssetsEntry updatedEntry = updatedAssets[i];
          expect(updatedEntry, WidgetPreviewStartCommand.transformAssetsEntry(rootEntry));
        }

        expect(updated.fonts.length, root.fonts.length);
        for (int i = 0; i < root.fonts.length; ++i) {
          final Font rootFont = root.fonts[i];
          final Font updatedFont = updated.fonts[i];
          expect(updatedFont.familyName, rootFont.familyName);
          expect(updatedFont.fontAssets.length, rootFont.fontAssets.length);
          for (int j = 0; j < rootFont.fontAssets.length; ++j) {
            final FontAsset rootFontAsset = rootFont.fontAssets[j];
            final FontAsset updatedFontAsset = updatedFont.fontAssets[j];
            expect(
              updatedFontAsset.descriptor,
              WidgetPreviewStartCommand.transformFontAsset(rootFontAsset).descriptor,
            );
          }
        }

        expect(updated.shaders, root.shaders.map(WidgetPreviewStartCommand.transformAssetUri));
        expect(updated.models, root.models.map(WidgetPreviewStartCommand.transformAssetUri));

        expect(updated.deferredComponents?.length, root.deferredComponents?.length);
        if (root.deferredComponents != null) {
          for (int i = 0; i < root.deferredComponents!.length; ++i) {
            expect(
              updated.deferredComponents![i].toString(),
              WidgetPreviewStartCommand.transformDeferredComponent(
                root.deferredComponents![i],
              ).toString(),
            );
          }
        }
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );

    testUsingContext(
      'can add flutter_gen to package_config.json if generate is set in the parent project',
      () async {
        command.maybeAddFlutterGenToPackageConfig(rootProject: rootProject);
        final Map<String, Object?> packageConfig =
            jsonDecode(rootProject.widgetPreviewScaffoldProject.packageConfig.readAsStringSync())
                as Map<String, Object?>;
        expect(packageConfig.containsKey('packages'), true);
        final List<Map<String, Object?>> packages =
            (packageConfig['packages']! as List<dynamic>).cast<Map<String, Object?>>();
        expect(packages.length, 2);
        expect(packages.last, WidgetPreviewStartCommand.flutterGenPackageConfigEntry);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );
  });
}

class FakeFlutterProject extends Fake implements FlutterProject {
  FakeFlutterProject({required this.projectRoot, required this.fileSystem, required this.logger});

  static const String complexPubspec = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  generate: true
  assets:
    - path: foo
      flavors:
        - flavor
      transformers:
        - package: package:foo
          args:
            - arg
    - path: package/foo
  fonts:
    - family: fontFamily
      fonts:
        - weight: 100
          style: normal
          asset: assetUri
        - weight: 200
          style: italic
          asset: package/assetUri
  shaders:
    - shaderUri
  models:
    - modelUri
  deferred-components:
    - name: deferredComponent
      libraries:
        - deferredComponentLibrary
      assets:
        - path: deferredComponentUri
          flavors:
            - deferredComponentFlavor
          transformers:
            - package: package:deferredComponent
              args:
                - deferredComponentArg
        - path: package/deferredComponentUri''';

  static const String basicPackageConfig = '''
{
  "configVersion": 2,
  "packages": [
    {
      "name": "test",
      "rootUri": "fileSystem.currentDirectory.path",
      "packageUri": "lib/"
    }
  ]
}
''';

  final String projectRoot;
  final FileSystem fileSystem;
  final Logger logger;

  @override
  late final FlutterManifest manifest =
      FlutterManifest.createFromPath(pubspecFile.path, fileSystem: fileSystem, logger: logger)!;

  @override
  late FlutterProject widgetPreviewScaffoldProject = FakeFlutterProject(
    projectRoot: fileSystem.path.join(projectRoot, '.dart_tool', 'widget_preview_scaffold'),
    fileSystem: fileSystem,
    logger: logger,
  );

  @override
  late final File pubspecFile = () {
    final File file = fileSystem.directory(projectRoot).childFile('pubspec.yaml')
      ..createSync(recursive: true);
    file.writeAsStringSync(complexPubspec);
    return file;
  }();

  @override
  late final File packageConfig = () {
    final File file = fileSystem
      .directory(fileSystem.path.join(projectRoot, '.dart_tool'))
      .childFile('package_config.json')..createSync(recursive: true);
    file.writeAsStringSync(basicPackageConfig);
    return file;
  }();
}
