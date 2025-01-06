// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:process/process.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../artifacts.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import '../base/project_migrator.dart';
import '../base/terminal.dart';
import '../build_info.dart';
import '../build_system/build_system.dart';
import '../cache.dart';
import '../flutter_plugins.dart';
import '../globals.dart' as globals;
import '../platform_plugins.dart';
import '../plugins.dart';
import '../project.dart';
import '../reporting/reporting.dart';
import '../version.dart';
import 'compiler_config.dart';
import 'file_generators/flutter_service_worker_js.dart';
import 'migrations/scrub_generated_plugin_registrant.dart';

export 'compiler_config.dart';

/// Whether the application has web plugins.
const String kHasWebPlugins = 'HasWebPlugins';

/// Base href to set in index.html in flutter build command
const String kBaseHref = 'baseHref';

/// The caching strategy to use for service worker generation.
const String kServiceWorkerStrategy = 'ServiceWorkerStrategy';

class WebBuilder {
  WebBuilder({
    required Logger logger,
    required ProcessManager processManager,
    required BuildSystem buildSystem,
    required Usage usage,
    required Analytics analytics,
    required FlutterVersion flutterVersion,
    required FileSystem fileSystem,
  }) : _logger = logger,
       _processManager = processManager,
       _buildSystem = buildSystem,
       _flutterUsage = usage,
       _analytics = analytics,
       _flutterVersion = flutterVersion,
       _fileSystem = fileSystem;

  final Logger _logger;
  final ProcessManager _processManager;
  final BuildSystem _buildSystem;
  final Usage _flutterUsage;
  final Analytics _analytics;
  final FlutterVersion _flutterVersion;
  final FileSystem _fileSystem;

  Future<void> buildWeb(
    FlutterProject flutterProject,
    String target,
    BuildInfo buildInfo,
    ServiceWorkerStrategy serviceWorkerStrategy, {
    required List<WebCompilerConfig> compilerConfigs,
    String? baseHref,
    String? outputDirectoryPath,
  }) async {
    final bool hasWebPlugins = (await findPlugins(
      flutterProject,
    )).any((Plugin p) => p.platforms.containsKey(WebPlugin.kConfigKey));
    final Directory outputDirectory =
        outputDirectoryPath == null
            ? _fileSystem.directory(getWebBuildDirectory())
            : _fileSystem.directory(outputDirectoryPath);
    outputDirectory.createSync(recursive: true);

    // The migrators to apply to a Web project.
    final List<ProjectMigrator> migrators = <ProjectMigrator>[
      ScrubGeneratedPluginRegistrant(flutterProject.web, _logger),
    ];

    final ProjectMigration migration = ProjectMigration(migrators);
    await migration.run();

    final Status status = _logger.startProgress('Compiling $target for the Web...');
    final Stopwatch sw = Stopwatch()..start();
    try {
      final BuildResult result = await _buildSystem.build(
        globals.buildTargets.webServiceWorker(_fileSystem, compilerConfigs),
        Environment(
          projectDir: _fileSystem.currentDirectory,
          outputDir: outputDirectory,
          buildDir: flutterProject.directory
              .childDirectory('.dart_tool')
              .childDirectory('flutter_build'),
          defines: <String, String>{
            kTargetFile: target,
            kHasWebPlugins: hasWebPlugins.toString(),
            if (baseHref != null) kBaseHref: baseHref,
            kServiceWorkerStrategy: serviceWorkerStrategy.cliName,
            ...buildInfo.toBuildSystemEnvironment(),
          },
          packageConfigPath: buildInfo.packageConfigPath,
          artifacts: globals.artifacts!,
          fileSystem: _fileSystem,
          logger: _logger,
          processManager: _processManager,
          platform: globals.platform,
          usage: _flutterUsage,
          analytics: _analytics,
          cacheDir: globals.cache.getRoot(),
          engineVersion:
              globals.artifacts!.usesLocalArtifacts ? null : _flutterVersion.engineRevision,
          flutterRootDir: _fileSystem.directory(Cache.flutterRoot),
          // Web uses a different Dart plugin registry.
          // https://github.com/flutter/flutter/issues/80406
          generateDartPluginRegistry: false,
        ),
      );
      if (!result.success) {
        for (final ExceptionMeasurement measurement in result.exceptions.values) {
          _logger.printError(
            'Target ${measurement.target} failed: ${measurement.exception}',
            stackTrace: measurement.fatal ? measurement.stackTrace : null,
          );
        }
        throwToolExit('Failed to compile application for the Web.');
      }
    } on Exception catch (err) {
      throwToolExit(err.toString());
    } finally {
      status.stop();
    }

    // We don't print a size because the output directory can contain
    // optional files not needed by the user.
    globals.printStatus(
      '${globals.terminal.successMark} '
      'Built ${globals.fs.path.relative(outputDirectory.path)}',
      color: TerminalColor.green,
    );

    final String buildSettingsString = _buildEventAnalyticsSettings(configs: compilerConfigs);

    BuildEvent(
      'web-compile',
      type: 'web',
      settings: buildSettingsString,
      flutterUsage: _flutterUsage,
    ).send();
    _analytics.send(
      Event.flutterBuildInfo(label: 'web-compile', buildType: 'web', settings: buildSettingsString),
    );

    final Duration elapsedDuration = sw.elapsed;
    final String variableName = compilerConfigs.length > 1 ? 'dual-compile' : 'dart2js';
    _flutterUsage.sendTiming('build', variableName, elapsedDuration);
    _analytics.send(
      Event.timing(
        workflow: 'build',
        variableName: variableName,
        elapsedMilliseconds: elapsedDuration.inMilliseconds,
      ),
    );
  }
}

/// Web rendering backend mode.
enum WebRendererMode {
  /// Auto detects which rendering backend to use.
  auto,

  /// Always uses canvaskit.
  canvaskit,

  /// Always uses html.
  html,

  /// Always use skwasm.
  skwasm;

  factory WebRendererMode.fromDartDefines(Iterable<String> defines, {required bool useWasm}) {
    if (defines.contains('FLUTTER_WEB_AUTO_DETECT=true')) {
      return auto;
    } else if (defines.contains('FLUTTER_WEB_USE_SKIA=false') &&
        defines.contains('FLUTTER_WEB_USE_SKWASM=true')) {
      return skwasm;
    } else if (defines.contains('FLUTTER_WEB_USE_SKIA=true') &&
        defines.contains('FLUTTER_WEB_USE_SKWASM=false')) {
      return canvaskit;
    } else if (defines.contains('FLUTTER_WEB_USE_SKIA=false') &&
        defines.contains('FLUTTER_WEB_USE_SKWASM=false')) {
      return html; // The horror!
    }
    return getDefault(useWasm: useWasm);
  }

  static WebRendererMode getDefault({required bool useWasm}) {
    return useWasm ? defaultForWasm : defaultForJs;
  }

  static const WebRendererMode defaultForJs = WebRendererMode.canvaskit;
  static const WebRendererMode defaultForWasm = WebRendererMode.skwasm;

  /// Returns whether the WebRendererMode is considered deprecated or not.
  ///
  /// Deprecated modes: auto, html.
  bool get isDeprecated => switch (this) {
    auto => true,
    canvaskit => false,
    html => true,
    skwasm => false,
  };

  /// Returns a consistent deprecation warning for the WebRendererMode.
  String get deprecationWarning =>
      'The HTML Renderer is deprecated and will be removed. Please, stop using it.'
      '\nSee: https://docs.flutter.dev/to/web-html-renderer-deprecation';

  String get helpText => switch (this) {
    auto => 'Use the HTML renderer on mobile devices, and CanvasKit on desktop devices.',
    canvaskit =>
      'Always use the CanvasKit renderer. This renderer uses WebGL and WebAssembly to render graphics.',
    html =>
      'Always use the HTML renderer. This renderer uses a combination of HTML, CSS, SVG, 2D Canvas, and WebGL.',
    skwasm => 'Always use the experimental skwasm renderer.',
  };

  /// Returns [dartDefines] in a way usable from the CLI.
  ///
  /// This is used to start integration tests.
  Iterable<String> get toCliDartDefines =>
      dartDefines.map((String define) => '--dart-define=$define');

  Iterable<String> get dartDefines => switch (this) {
    auto => const <String>{'FLUTTER_WEB_AUTO_DETECT=true'},
    canvaskit => const <String>{'FLUTTER_WEB_USE_SKIA=true', 'FLUTTER_WEB_USE_SKWASM=false'},
    html => const <String>{'FLUTTER_WEB_USE_SKIA=false', 'FLUTTER_WEB_USE_SKWASM=false'},
    skwasm => const <String>{'FLUTTER_WEB_USE_SKIA=false', 'FLUTTER_WEB_USE_SKWASM=true'},
  };

  /// Sets the dart defines for the currently selected WebRendererMode
  List<String> updateDartDefines(List<String> inputDefines) {
    final Set<String> dartDefinesSet = inputDefines.toSet();

    dartDefinesSet
      ..removeWhere((String d) {
        return d.startsWith('FLUTTER_WEB_AUTO_DETECT=') ||
            d.startsWith('FLUTTER_WEB_USE_SKIA=') ||
            d.startsWith('FLUTTER_WEB_USE_SKWASM=');
      })
      ..addAll(dartDefines);

    return dartDefinesSet.toList();
  }
}

/// The correct precompiled artifact to use for each build and render mode for DDC with AMD modules.
// TODO(markzipan): delete this when DDC's AMD module system is deprecated, https://github.com/flutter/flutter/issues/142060.
const Map<WebRendererMode, Map<NullSafetyMode, HostArtifact>> kAmdDartSdkJsArtifactMap =
    <WebRendererMode, Map<NullSafetyMode, HostArtifact>>{
      WebRendererMode.auto: <NullSafetyMode, HostArtifact>{
        NullSafetyMode.sound: HostArtifact.webPrecompiledAmdCanvaskitAndHtmlSoundSdk,
        NullSafetyMode.unsound: HostArtifact.webPrecompiledAmdCanvaskitAndHtmlSdk,
      },
      WebRendererMode.canvaskit: <NullSafetyMode, HostArtifact>{
        NullSafetyMode.sound: HostArtifact.webPrecompiledAmdCanvaskitSoundSdk,
        NullSafetyMode.unsound: HostArtifact.webPrecompiledAmdCanvaskitSdk,
      },
      WebRendererMode.html: <NullSafetyMode, HostArtifact>{
        NullSafetyMode.sound: HostArtifact.webPrecompiledAmdSoundSdk,
        NullSafetyMode.unsound: HostArtifact.webPrecompiledAmdSdk,
      },
    };

/// The correct source map artifact to use for each build and render mode for DDC with AMD modules.
// TODO(markzipan): delete this when DDC's AMD module system is deprecated, https://github.com/flutter/flutter/issues/142060.
const Map<WebRendererMode, Map<NullSafetyMode, HostArtifact>> kAmdDartSdkJsMapArtifactMap =
    <WebRendererMode, Map<NullSafetyMode, HostArtifact>>{
      WebRendererMode.auto: <NullSafetyMode, HostArtifact>{
        NullSafetyMode.sound: HostArtifact.webPrecompiledAmdCanvaskitAndHtmlSoundSdkSourcemaps,
        NullSafetyMode.unsound: HostArtifact.webPrecompiledAmdCanvaskitAndHtmlSdkSourcemaps,
      },
      WebRendererMode.canvaskit: <NullSafetyMode, HostArtifact>{
        NullSafetyMode.sound: HostArtifact.webPrecompiledAmdCanvaskitSoundSdkSourcemaps,
        NullSafetyMode.unsound: HostArtifact.webPrecompiledAmdCanvaskitSdkSourcemaps,
      },
      WebRendererMode.html: <NullSafetyMode, HostArtifact>{
        NullSafetyMode.sound: HostArtifact.webPrecompiledAmdSoundSdkSourcemaps,
        NullSafetyMode.unsound: HostArtifact.webPrecompiledAmdSdkSourcemaps,
      },
    };

/// The correct precompiled artifact to use for each build and render mode for DDC with DDC modules.
const Map<WebRendererMode, Map<NullSafetyMode, HostArtifact>> kDdcDartSdkJsArtifactMap =
    <WebRendererMode, Map<NullSafetyMode, HostArtifact>>{
      WebRendererMode.auto: <NullSafetyMode, HostArtifact>{
        NullSafetyMode.sound: HostArtifact.webPrecompiledDdcCanvaskitAndHtmlSoundSdk,
        NullSafetyMode.unsound: HostArtifact.webPrecompiledDdcCanvaskitAndHtmlSdk,
      },
      WebRendererMode.canvaskit: <NullSafetyMode, HostArtifact>{
        NullSafetyMode.sound: HostArtifact.webPrecompiledDdcCanvaskitSoundSdk,
        NullSafetyMode.unsound: HostArtifact.webPrecompiledDdcCanvaskitSdk,
      },
      WebRendererMode.html: <NullSafetyMode, HostArtifact>{
        NullSafetyMode.sound: HostArtifact.webPrecompiledDdcSoundSdk,
        NullSafetyMode.unsound: HostArtifact.webPrecompiledDdcSdk,
      },
    };

/// The correct source map artifact to use for each build and render mode for DDC with DDC modules.
const Map<WebRendererMode, Map<NullSafetyMode, HostArtifact>> kDdcDartSdkJsMapArtifactMap =
    <WebRendererMode, Map<NullSafetyMode, HostArtifact>>{
      WebRendererMode.auto: <NullSafetyMode, HostArtifact>{
        NullSafetyMode.sound: HostArtifact.webPrecompiledDdcCanvaskitAndHtmlSoundSdkSourcemaps,
        NullSafetyMode.unsound: HostArtifact.webPrecompiledDdcCanvaskitAndHtmlSdkSourcemaps,
      },
      WebRendererMode.canvaskit: <NullSafetyMode, HostArtifact>{
        NullSafetyMode.sound: HostArtifact.webPrecompiledDdcCanvaskitSoundSdkSourcemaps,
        NullSafetyMode.unsound: HostArtifact.webPrecompiledDdcCanvaskitSdkSourcemaps,
      },
      WebRendererMode.html: <NullSafetyMode, HostArtifact>{
        NullSafetyMode.sound: HostArtifact.webPrecompiledDdcSoundSdkSourcemaps,
        NullSafetyMode.unsound: HostArtifact.webPrecompiledDdcSdkSourcemaps,
      },
    };

String _buildEventAnalyticsSettings({required List<WebCompilerConfig> configs}) {
  final Map<String, Object> values = <String, Object>{};
  final List<String> renderers = <String>[];
  final List<String> targets = <String>[];
  for (final WebCompilerConfig config in configs) {
    values.addAll(config.buildEventAnalyticsValues);
    renderers.add(config.renderer.name);
    targets.add(config.compileTarget.name);
  }
  values['web-renderer'] = renderers.join(',');
  values['web-target'] = targets.join(',');

  final List<String> sortedList =
      values.entries.map((MapEntry<String, Object> e) => '${e.key}: ${e.value};').toList()..sort();

  return sortedList.join(' ');
}
