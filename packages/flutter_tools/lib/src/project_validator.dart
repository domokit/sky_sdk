// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/platform.dart';
import 'cache.dart';
import 'flutter_manifest.dart';
import 'project.dart';
import 'project_validator_result.dart';
import 'version.dart';

abstract class ProjectValidator {
  const ProjectValidator();
  String get title;
  bool get machineOutput => false;
  bool supportsProject(FlutterProject project);

  /// Can return more than one result in case a file/command have a lot of info to share to the user
  Future<List<ProjectValidatorResult>> start(FlutterProject project);
}

abstract class MachineProjectValidator extends ProjectValidator {
  const MachineProjectValidator();

  @override
  bool get machineOutput => true;
}

/// Validator run for all platforms that extract information from the pubspec.yaml.
///
/// Specific info from different platforms should be written in their own ProjectValidator.
class VariableDumpMachineProjectValidator extends MachineProjectValidator {
  VariableDumpMachineProjectValidator({
    required this.logger,
    required this.fileSystem,
    required this.platform,
  });

  final Logger logger;
  final FileSystem fileSystem;
  final Platform platform;

  String _toJsonValue(Object? obj) {
    String value = obj.toString();
    if (obj is String) {
      value = '"$obj"';
    }
    value = value.replaceAll(r'\', r'\\');
    return value;
  }

  @override
  Future<List<ProjectValidatorResult>> start(FlutterProject project) async {
    final FlutterVersion version = FlutterVersion(flutterRoot: Cache.flutterRoot!, fs: fileSystem);
    final Map<String, Object?> result = <String, Object?>{
      'FlutterProject.directory': project.directory.absolute.path,
      'FlutterProject.metadataFile': project.metadataFile.absolute.path,
      'FlutterProject.android.exists': project.android.existsSync(),
      'FlutterProject.ios.exists': project.ios.exists,
      'FlutterProject.web.exists': project.web.existsSync(),
      'FlutterProject.macos.exists': project.macos.existsSync(),
      'FlutterProject.linux.exists': project.linux.existsSync(),
      'FlutterProject.windows.exists': project.windows.existsSync(),
      'FlutterProject.fuchsia.exists': project.fuchsia.existsSync(),

      'FlutterProject.android.isKotlin': project.android.isKotlin,
      'FlutterProject.ios.isSwift': project.ios.isSwift,

      'FlutterProject.isModule': project.isModule,
      'FlutterProject.isPlugin': project.isPlugin,

      'FlutterProject.manifest.appname': project.manifest.appName,

      // FlutterVersion
      'FlutterVersion.frameworkRevision': version.frameworkRevision,

      // Platform
      'Platform.operatingSystem': platform.operatingSystem,
      'Platform.isAndroid': platform.isAndroid,
      'Platform.isIOS': platform.isIOS,
      'Platform.isWindows': platform.isWindows,
      'Platform.isMacOS': platform.isMacOS,
      'Platform.isFuchsia': platform.isFuchsia,
      'Platform.pathSeparator': platform.pathSeparator,

      // Cache
      'Cache.flutterRoot': Cache.flutterRoot,
    };

    return <ProjectValidatorResult>[
      for (final String key in result.keys)
        ProjectValidatorResult(
          name: key,
          value: _toJsonValue(result[key]),
          status: StatusProjectValidator.info,
        ),
    ];
  }

  @override
  bool supportsProject(FlutterProject project) {
    // this validator will run for any type of project
    return true;
  }

  @override
  String get title => 'Machine JSON variable dump';
}

/// Validator run for all platforms that extract information from the pubspec.yaml.
///
/// Specific info from different platforms should be written in their own ProjectValidator.
class GeneralInfoProjectValidator extends ProjectValidator {
  @override
  Future<List<ProjectValidatorResult>> start(FlutterProject project) async {
    final FlutterManifest flutterManifest = project.manifest;
    final List<ProjectValidatorResult> result = <ProjectValidatorResult>[];
    final ProjectValidatorResult appNameValidatorResult = _getAppNameResult(flutterManifest);
    result.add(appNameValidatorResult);
    final String supportedPlatforms = _getSupportedPlatforms(project);
    if (supportedPlatforms.isEmpty) {
      return result;
    }
    final ProjectValidatorResult supportedPlatformsResult = ProjectValidatorResult(
      name: 'Supported Platforms',
      value: supportedPlatforms,
      status: StatusProjectValidator.success,
    );
    final ProjectValidatorResult isFlutterPackage = _isFlutterPackageValidatorResult(
      flutterManifest,
    );
    result.addAll(<ProjectValidatorResult>[supportedPlatformsResult, isFlutterPackage]);
    if (flutterManifest.flutterDescriptor.isNotEmpty) {
      result.add(_materialDesignResult(flutterManifest));
      result.add(_pluginValidatorResult(flutterManifest));
    }
    result.add(await project.android.validateJavaAndGradleAgpVersions());
    return result;
  }

  ProjectValidatorResult _getAppNameResult(FlutterManifest flutterManifest) {
    final String appName = flutterManifest.appName;
    const String name = 'App Name';
    if (appName.isEmpty) {
      return const ProjectValidatorResult(
        name: name,
        value: 'name not found',
        status: StatusProjectValidator.error,
      );
    }
    return ProjectValidatorResult(
      name: name,
      value: appName,
      status: StatusProjectValidator.success,
    );
  }

  ProjectValidatorResult _isFlutterPackageValidatorResult(FlutterManifest flutterManifest) {
    final String value;
    final StatusProjectValidator status;
    if (flutterManifest.flutterDescriptor.isNotEmpty) {
      value = 'yes';
      status = StatusProjectValidator.success;
    } else {
      value = 'no';
      status = StatusProjectValidator.warning;
    }

    return ProjectValidatorResult(name: 'Is Flutter Package', value: value, status: status);
  }

  ProjectValidatorResult _materialDesignResult(FlutterManifest flutterManifest) {
    return ProjectValidatorResult(
      name: 'Uses Material Design',
      value: flutterManifest.usesMaterialDesign ? 'yes' : 'no',
      status: StatusProjectValidator.success,
    );
  }

  String _getSupportedPlatforms(FlutterProject project) {
    return project
        .getSupportedPlatforms()
        .map((SupportedPlatform platform) => platform.name)
        .join(', ');
  }

  ProjectValidatorResult _pluginValidatorResult(FlutterManifest flutterManifest) {
    return ProjectValidatorResult(
      name: 'Is Plugin',
      value: flutterManifest.isPlugin ? 'yes' : 'no',
      status: StatusProjectValidator.success,
    );
  }

  @override
  bool supportsProject(FlutterProject project) {
    // this validator will run for any type of project
    return true;
  }

  @override
  String get title => 'General Info';
}
