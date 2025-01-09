// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:meta/meta.dart';

import '../base/common.dart';
import '../base/deferred_component.dart';
import '../base/file_system.dart';
import '../base/platform.dart';
import '../cache.dart';
import '../convert.dart';
import '../dart/pub.dart';
import '../flutter_manifest.dart';

import '../globals.dart' as globals;
import '../project.dart';
import '../runner/flutter_command.dart';
import 'create_base.dart';

class WidgetPreviewCommand extends FlutterCommand {
  WidgetPreviewCommand() {
    addSubcommand(WidgetPreviewStartCommand());
    addSubcommand(WidgetPreviewCleanCommand());
  }

  @override
  String get description => 'Manage the widget preview environment.';

  @override
  String get name => 'widget-preview';

  @override
  String get category => FlutterCommandCategory.tools;

  // TODO(bkonyi): show when --verbose is not provided when this feature is
  // ready to ship.
  @override
  bool get hidden => true;

  @override
  Future<FlutterCommandResult> runCommand() async => FlutterCommandResult.fail();
}

/// Common utilities for the 'start' and 'clean' commands.
mixin WidgetPreviewSubCommandMixin on FlutterCommand {
  FlutterProject getRootProject() {
    final ArgResults results = argResults!;
    final Directory projectDir;
    if (results.rest case <String>[final String directory]) {
      projectDir = globals.fs.directory(directory);
      if (!projectDir.existsSync()) {
        throwToolExit('Could not find ${projectDir.path}.');
      }
    } else if (results.rest.length > 1) {
      throwToolExit('Only one directory should be provided.');
    } else {
      projectDir = globals.fs.currentDirectory;
    }
    return validateFlutterProjectForPreview(projectDir);
  }

  FlutterProject validateFlutterProjectForPreview(Directory directory) {
    globals.logger.printTrace('Verifying that ${directory.path} is a Flutter project.');
    final FlutterProject flutterProject = globals.projectFactory.fromDirectory(directory);
    if (!flutterProject.dartTool.existsSync()) {
      throwToolExit('${flutterProject.directory.path} is not a valid Flutter project.');
    }
    return flutterProject;
  }
}

class WidgetPreviewStartCommand extends FlutterCommand
    with CreateBase, WidgetPreviewSubCommandMixin {
  WidgetPreviewStartCommand() {
    addPubOptions();
  }

  @override
  String get description => 'Starts the widget preview environment.';

  @override
  String get name => 'start';

  @override
  Future<FlutterCommandResult> runCommand() async {
    final FlutterProject rootProject = getRootProject();
    final Directory widgetPreviewScaffold = rootProject.widgetPreviewScaffold;

    // Check to see if a preview scaffold has already been generated. If not,
    // generate one.
    if (!widgetPreviewScaffold.existsSync()) {
      globals.logger.printStatus(
        'Creating widget preview scaffolding at: ${widgetPreviewScaffold.path}',
      );
      await generateApp(
        <String>['widget_preview_scaffold'],
        widgetPreviewScaffold,
        createTemplateContext(
          organization: 'flutter',
          projectName: 'widget_preview_scaffold',
          titleCaseProjectName: 'Widget Preview Scaffold',
          flutterRoot: Cache.flutterRoot!,
          dartSdkVersionBounds: '^${globals.cache.dartSdkBuild}',
          linux: const LocalPlatform().isLinux,
          macos: const LocalPlatform().isMacOS,
          windows: const LocalPlatform().isWindows,
        ),
        overwrite: true,
        generateMetadata: false,
      );
      await _populatePreviewPubspec(rootProject: rootProject);
    }
    return FlutterCommandResult.success();
  }

  @visibleForTesting
  static const Map<String, String> flutterGenPackageConfigEntry = <String, String>{
    'name': 'flutter_gen',
    'rootUri': '../../flutter_gen',
    'languageVersion': '2.12',
  };

  /// Maps asset URIs to relative paths for the widget preview project to
  /// include.
  @visibleForTesting
  static Uri transformAssetUri(Uri uri) {
    // Assets provided by packages always start with 'packages' and do not
    // require their URIs to be updated.
    if (uri.path.startsWith('packages')) {
      return uri;
    }
    // Otherwise, the asset is contained within the root project and needs
    // to be referenced from the widget preview scaffold project's pubspec.
    return Uri(path: '../../${uri.path}');
  }

  @visibleForTesting
  static AssetsEntry transformAssetsEntry(AssetsEntry asset) {
    return AssetsEntry(
      uri: transformAssetUri(asset.uri),
      flavors: asset.flavors,
      transformers: asset.transformers,
    );
  }

  @visibleForTesting
  static FontAsset transformFontAsset(FontAsset asset) {
    return FontAsset(transformAssetUri(asset.assetUri), weight: asset.weight, style: asset.style);
  }

  @visibleForTesting
  static DeferredComponent transformDeferredComponent(DeferredComponent component) {
    return DeferredComponent(
      name: component.name,
      // TODO(bkonyi): verify these library paths are always package: paths from the parent project.
      libraries: component.libraries,
      assets: component.assets.map(transformAssetsEntry).toList(),
    );
  }

  @visibleForTesting
  FlutterManifest buildPubspec({
    required FlutterManifest rootManifest,
    required FlutterManifest widgetPreviewManifest,
  }) {
    final List<AssetsEntry> assets = rootManifest.assets.map(transformAssetsEntry).toList();

    final List<Font> fonts =
        rootManifest.fonts.map((Font font) {
          return Font(font.familyName, font.fontAssets.map(transformFontAsset).toList());
        }).toList();

    final List<Uri> shaders = rootManifest.shaders.map(transformAssetUri).toList();

    final List<Uri> models = rootManifest.models.map(transformAssetUri).toList();

    final List<DeferredComponent>? deferredComponents =
        rootManifest.deferredComponents?.map(transformDeferredComponent).toList();

    return widgetPreviewManifest.copyWith(
      logger: globals.logger,
      assets: assets,
      fonts: fonts,
      shaders: shaders,
      models: models,
      deferredComponents: deferredComponents,
    );
  }

  Future<void> _populatePreviewPubspec({required FlutterProject rootProject}) async {
    final FlutterProject widgetPreviewScaffoldProject = rootProject.widgetPreviewScaffoldProject;

    // Overwrite the pubspec for the preview scaffold project to include assets
    // from the root project.
    widgetPreviewScaffoldProject.replacePubspec(
      buildPubspec(
        rootManifest: rootProject.manifest,
        widgetPreviewManifest: widgetPreviewScaffoldProject.manifest,
      ),
    );

    // Adds a path dependency on the parent project so previews can be
    // imported directly into the preview scaffold.
    const String pubAdd = 'add';
    await pub.interactively(
      <String>[
        pubAdd,
        '--directory',
        widgetPreviewScaffoldProject.directory.path,
        '${rootProject.manifest.appName}:{"path":${rootProject.directory.path}}',
      ],
      context: PubContext.pubAdd,
      command: pubAdd,
      touchesPackageConfig: true,
    );

    // Generate package_config.json.
    await pub.get(
      context: PubContext.create,
      project: widgetPreviewScaffoldProject,
      offline: offline,
      outputMode: PubOutputMode.summaryOnly,
    );

    maybeAddFlutterGenToPackageConfig(rootProject: rootProject);
  }

  /// Manually adds an entry for package:flutter_gen to the preview scaffold's
  /// package_config.json if the target project makes use of localization.
  ///
  /// The Flutter Tool does this when running a Flutter project with
  /// localization instead of modifying the user's pubspec.yaml to depend on it
  /// as a path dependency. Unfortunately, the preview scaffold still needs to
  /// add it directly to its package_config.json as the generated package name
  /// isn't actually flutter_gen, which pub doesn't really like, and using the
  /// actual package name will break applications which import
  /// package:flutter_gen.
  @visibleForTesting
  void maybeAddFlutterGenToPackageConfig({required FlutterProject rootProject}) {
    // TODO(matanlurey): Remove this once flutter_gen is removed.
    //
    // This is actually incorrect logic; the presence of a `generate: true`
    // does *NOT* mean that we need to add `flutter_gen` to the package config,
    // and never did, but the name of the manifest field was labeled and
    // described incorrectly.
    //
    // Tracking removal: https://github.com/flutter/flutter/issues/102983.
    if (!rootProject.manifest.generateLocalizations) {
      return;
    }
    final FlutterProject widgetPreviewScaffoldProject = rootProject.widgetPreviewScaffoldProject;
    final File packageConfig = widgetPreviewScaffoldProject.packageConfig;
    final String previewPackageConfigPath = packageConfig.path;
    if (!packageConfig.existsSync()) {
      throw StateError(
        "Could not find preview project's package_config.json at "
        '$previewPackageConfigPath',
      );
    }
    final Map<String, Object?> packageConfigJson =
        json.decode(packageConfig.readAsStringSync()) as Map<String, Object?>;
    (packageConfigJson['packages'] as List<dynamic>?)!.cast<Map<String, String>>().add(
      flutterGenPackageConfigEntry,
    );
    packageConfig.writeAsStringSync(json.encode(packageConfigJson));
    globals.logger.printStatus('Added flutter_gen dependency to $previewPackageConfigPath');
  }
}

class WidgetPreviewCleanCommand extends FlutterCommand with WidgetPreviewSubCommandMixin {
  @override
  String get description => 'Cleans up widget preview state.';

  @override
  String get name => 'clean';

  @override
  Future<FlutterCommandResult> runCommand() async {
    final Directory widgetPreviewScaffold = getRootProject().widgetPreviewScaffold;
    if (widgetPreviewScaffold.existsSync()) {
      final String scaffoldPath = widgetPreviewScaffold.path;
      globals.logger.printStatus('Deleting widget preview scaffold at $scaffoldPath.');
      widgetPreviewScaffold.deleteSync(recursive: true);
    } else {
      globals.logger.printStatus('Nothing to clean up.');
    }
    return FlutterCommandResult.success();
  }
}
