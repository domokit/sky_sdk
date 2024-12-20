// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:package_config/package_config.dart';

import '../../base/file_system.dart';
import '../../build_info.dart';
import '../../dart/package_map.dart';
import '../../flutter_plugins.dart';
import '../../project.dart';
import '../build_system.dart';

/// Generates a new `./dart_tool/flutter_build/dart_plugin_registrant.dart`
/// based on the current dependency map in `pubspec.lock`.
class DartPluginRegistrantTarget extends Target {
  /// Construct a [DartPluginRegistrantTarget].
  const DartPluginRegistrantTarget() : _project = null;

  /// Construct a [DartPluginRegistrantTarget].
  ///
  /// If `project` is unset, a [FlutterProject] based on environment is used.
  @visibleForTesting
  const DartPluginRegistrantTarget.test(FlutterProject project) : _project = project;

  final FlutterProject? _project;

  @override
  Future<void> build(Environment environment) async {
    assert(environment.generateDartPluginRegistry);
    final FlutterProject project = _project ?? FlutterProject.fromDirectory(environment.projectDir);
    final File packageConfigFile = findPackageConfigFileOrDefault(environment.projectDir);
    final PackageConfig packageConfig = await loadPackageConfigWithLogging(
      packageConfigFile,
      logger: environment.logger,
    );
    final String targetFilePath =
        environment.defines[kTargetFile] ?? environment.fileSystem.path.join('lib', 'main.dart');
    final File mainFile = environment.fileSystem.file(targetFilePath);
    final Uri mainFileUri = mainFile.absolute.uri;
    final String mainFileUriString =
        packageConfig.toPackageUri(mainFileUri)?.toString() ?? mainFileUri.toString();

    await generateMainDartWithPluginRegistrant(project, packageConfig, mainFileUriString, mainFile);
  }

  @override
  bool canSkip(Environment environment) {
    if (!environment.generateDartPluginRegistry) {
      return true;
    }
    final String? platformName = environment.defines[kTargetPlatform];
    final TargetPlatform? targetPlatform =
        platformName == null ? null : getTargetPlatformForName(platformName);
    // TODO(stuartmorgan): Investigate removing this check entirely; ideally the
    // source generation step shouldn't be platform dependent, and the generated
    // code should just do the right thing on every platform.
    // Failing that, consider throwing if `targetPlatform` isn't set and finding
    // all violations, as it's not consistently set here.
    return targetPlatform == TargetPlatform.fuchsia_arm64 ||
        targetPlatform == TargetPlatform.fuchsia_x64 ||
        targetPlatform == TargetPlatform.web_javascript;
  }

  @override
  List<Target> get dependencies => <Target>[];

  @override
  List<Source> get inputs => <Source>[
    const Source.pattern('{WORKSPACE_DIR}/.dart_tool/package_config_subset'),
  ];

  @override
  String get name => 'gen_dart_plugin_registrant';

  @override
  List<Source> get outputs => <Source>[
    const Source.pattern(
      '{PROJECT_DIR}/.dart_tool/flutter_build/dart_plugin_registrant.dart',
      optional: true,
    ),
  ];
}
