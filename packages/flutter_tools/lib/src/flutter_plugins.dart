// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:package_config/package_config.dart';
import 'package:path/path.dart' as path; // flutter_ignore: package_path_import
import 'package:pub_semver/pub_semver.dart' as semver;
import 'package:yaml/yaml.dart';

import 'android/gradle.dart';
import 'base/common.dart';
import 'base/error_handling_io.dart';
import 'base/file_system.dart';
import 'base/os.dart';
import 'base/platform.dart';
import 'base/template.dart';
import 'base/version.dart';
import 'cache.dart';
import 'compute_dev_dependencies.dart';
import 'convert.dart';
import 'dart/language_version.dart';
import 'dart/package_map.dart';
import 'dart/pub.dart';
import 'features.dart';
import 'globals.dart' as globals;
import 'macos/darwin_dependency_management.dart';
import 'macos/swift_package_manager.dart';
import 'platform_plugins.dart';
import 'plugins.dart';
import 'project.dart';

Future<void> _renderTemplateToFile(
  String template,
  Object? context,
  File file,
  TemplateRenderer templateRenderer,
) async {
  final String renderedTemplate = templateRenderer.renderString(template, context);
  await file.create(recursive: true);
  await file.writeAsString(renderedTemplate);
}

Future<Plugin?> _pluginFromPackage(
  String name,
  Uri packageRoot,
  Set<String> appDependencies, {
  required Set<String> devDependencies,
  FileSystem? fileSystem,
}) async {
  final FileSystem fs = fileSystem ?? globals.fs;
  final File pubspecFile = fs.file(packageRoot.resolve('pubspec.yaml'));
  if (!pubspecFile.existsSync()) {
    return null;
  }
  Object? pubspec;

  try {
    pubspec = loadYaml(await pubspecFile.readAsString());
  } on YamlException catch (err) {
    globals.printTrace('Failed to parse plugin manifest for $name: $err');
    // Do nothing, potentially not a plugin.
  }
  if (pubspec == null || pubspec is! YamlMap) {
    return null;
  }
  final Object? flutterConfig = pubspec['flutter'];
  if (flutterConfig == null || flutterConfig is! YamlMap || !flutterConfig.containsKey('plugin')) {
    return null;
  }
  final String? flutterConstraintText = (pubspec['environment'] as YamlMap?)?['flutter'] as String?;
  final semver.VersionConstraint? flutterConstraint =
      flutterConstraintText == null ? null : semver.VersionConstraint.parse(flutterConstraintText);
  final String packageRootPath = fs.path.fromUri(packageRoot);
  final YamlMap? dependencies = pubspec['dependencies'] as YamlMap?;
  globals.printTrace('Found plugin $name at $packageRootPath');
  return Plugin.fromYaml(
    name,
    packageRootPath,
    flutterConfig['plugin'] as YamlMap?,
    flutterConstraint,
    dependencies == null ? <String>[] : <String>[...dependencies.keys.cast<String>()],
    fileSystem: fs,
    appDependencies: appDependencies,
    isDevDependency: devDependencies.contains(name),
  );
}

/// Returns a list of all plugins to be registered with the provided [project].
///
/// If [throwOnError] is `true`, an empty package configuration is an error.
Future<List<Plugin>> findPlugins(
  FlutterProject project, {
  bool throwOnError = true,
  bool? determineDevDependencies,
}) async {
  determineDevDependencies ??= featureFlags.isExplicitPackageDependenciesEnabled;
  final List<Plugin> plugins = <Plugin>[];
  final FileSystem fs = project.directory.fileSystem;
  final File packageConfigFile = findPackageConfigFileOrDefault(project.directory);
  final PackageConfig packageConfig = await loadPackageConfigWithLogging(
    packageConfigFile,
    logger: globals.logger,
    throwOnError: throwOnError,
  );
  final Set<String> devDependencies;
  if (!determineDevDependencies) {
    devDependencies = <String>{};
  } else {
    devDependencies = await computeExclusiveDevDependencies(
      pub,
      logger: globals.logger,
      project: project,
    );
  }
  for (final Package package in packageConfig.packages) {
    final Uri packageRoot = package.packageUriRoot.resolve('..');
    final Plugin? plugin = await _pluginFromPackage(
      package.name,
      packageRoot,
      project.manifest.dependencies,
      devDependencies: devDependencies,
      fileSystem: fs,
    );
    if (plugin != null) {
      plugins.add(plugin);
    }
  }
  return plugins;
}

/// Plugin resolution type to determine the injection mechanism.
enum _PluginResolutionType { dart, nativeOrDart }

// Key strings for the .flutter-plugins-dependencies file.
const String _kFlutterPluginsPluginListKey = 'plugins';
const String _kFlutterPluginsNameKey = 'name';
const String _kFlutterPluginsPathKey = 'path';
const String _kFlutterPluginsDependenciesKey = 'dependencies';
const String _kFlutterPluginsHasNativeBuildKey = 'native_build';
const String _kFlutterPluginsSharedDarwinSource = 'shared_darwin_source';
const String _kFlutterPluginsDevDependencyKey = 'dev_dependency';

/// Writes the .flutter-plugins-dependencies file based on the list of plugins.
/// If there aren't any plugins, then the files aren't written to disk. The resulting
/// file looks something like this (order of keys is not guaranteed):
/// {
///   "info": "This is a generated file; do not edit or check into version control.",
///   "plugins": {
///     "ios": [
///       {
///         "name": "test",
///         "path": "test_path",
///         "dependencies": [
///           "plugin-a",
///           "plugin-b"
///         ],
///         "native_build": true
///       }
///     ],
///     "android": [],
///     "macos": [],
///     "linux": [],
///     "windows": [],
///     "web": []
///   },
///   "dependencyGraph": [
///     {
///       "name": "plugin-a",
///       "dependencies": [
///         "plugin-b",
///         "plugin-c"
///       ]
///     },
///     {
///       "name": "plugin-b",
///       "dependencies": [
///         "plugin-c"
///       ]
///     },
///     {
///       "name": "plugin-c",
///       "dependencies": []
///     }
///   ],
///   "date_created": "1970-01-01 00:00:00.000",
///   "version": "0.0.0-unknown"
/// }
///
///
/// Finally, returns [true] if the plugins list has changed, otherwise returns [false].
bool _writeFlutterPluginsList(
  FlutterProject project,
  List<Plugin> plugins, {
  required bool swiftPackageManagerEnabledIos,
  required bool swiftPackageManagerEnabledMacos,
}) {
  final File pluginsFile = project.flutterPluginsDependenciesFile;
  if (plugins.isEmpty) {
    return ErrorHandlingFileSystem.deleteIfExists(pluginsFile);
  }

  final Iterable<String> platformKeys = <String>[
    project.ios.pluginConfigKey,
    project.android.pluginConfigKey,
    project.macos.pluginConfigKey,
    project.linux.pluginConfigKey,
    project.windows.pluginConfigKey,
    project.web.pluginConfigKey,
  ];

  final Map<String, List<Plugin>> resolvedPlatformPlugins = _resolvePluginImplementations(
    plugins,
    pluginResolutionType: _PluginResolutionType.nativeOrDart,
  );

  final Map<String, Object> pluginsMap = <String, Object>{};
  for (final String platformKey in platformKeys) {
    pluginsMap[platformKey] = _createPluginMapOfPlatform(
      resolvedPlatformPlugins[platformKey] ?? <Plugin>[],
      platformKey,
    );
  }

  final Map<String, Object> result = <String, Object>{};

  result['info'] = 'This is a generated file; do not edit or check into version control.';
  result[_kFlutterPluginsPluginListKey] = pluginsMap;

  /// The dependencyGraph object is kept for backwards compatibility, but
  /// should be removed once migration is complete.
  /// https://github.com/flutter/flutter/issues/48918
  result['dependencyGraph'] = _createPluginLegacyDependencyGraph(plugins);
  result['date_created'] = globals.systemClock.now().toString();
  result['version'] = globals.flutterVersion.frameworkVersion;

  result['swift_package_manager_enabled'] = <String, bool>{
    'ios': swiftPackageManagerEnabledIos,
    'macos': swiftPackageManagerEnabledMacos,
  };

  // Only notify if the plugins list has changed. [date_created] will always be different,
  // [version] is not relevant for this check.
  final String? oldPluginsFileStringContent = _readFileContent(pluginsFile);
  bool pluginsChanged = true;
  if (oldPluginsFileStringContent != null) {
    pluginsChanged = oldPluginsFileStringContent.contains(pluginsMap.toString());
  }
  final String pluginFileContent = json.encode(result);
  pluginsFile.writeAsStringSync(pluginFileContent, flush: true);

  return pluginsChanged;
}

/// Creates a map representation of the [plugins] for those supported by [platformKey].
/// All given [plugins] must provide an implementation for the [platformKey].
List<Map<String, Object>> _createPluginMapOfPlatform(List<Plugin> plugins, String platformKey) {
  final Set<String> pluginNames = plugins.map((Plugin plugin) => plugin.name).toSet();
  final List<Map<String, Object>> pluginInfo = <Map<String, Object>>[];
  for (final Plugin plugin in plugins) {
    assert(
      plugin.platforms[platformKey] != null,
      'Plugin ${plugin.name} does not provide an implementation for $platformKey.',
    );
    final PluginPlatform platformPlugin = plugin.platforms[platformKey]!;
    pluginInfo.add(<String, Object>{
      _kFlutterPluginsNameKey: plugin.name,
      _kFlutterPluginsPathKey: globals.fsUtils.escapePath(plugin.path),
      if (platformPlugin is DarwinPlugin && (platformPlugin as DarwinPlugin).sharedDarwinSource)
        _kFlutterPluginsSharedDarwinSource: (platformPlugin as DarwinPlugin).sharedDarwinSource,
      if (platformPlugin is NativeOrDartPlugin)
        _kFlutterPluginsHasNativeBuildKey:
            (platformPlugin as NativeOrDartPlugin).hasMethodChannel() ||
            (platformPlugin as NativeOrDartPlugin).hasFfi(),
      _kFlutterPluginsDependenciesKey: <String>[...plugin.dependencies.where(pluginNames.contains)],
      _kFlutterPluginsDevDependencyKey: plugin.isDevDependency,
    });
  }
  return pluginInfo;
}

List<Object?> _createPluginLegacyDependencyGraph(List<Plugin> plugins) {
  final List<Object> directAppDependencies = <Object>[];

  final Set<String> pluginNames = plugins.map((Plugin plugin) => plugin.name).toSet();
  for (final Plugin plugin in plugins) {
    directAppDependencies.add(<String, Object>{
      'name': plugin.name,
      // Extract the plugin dependencies which happen to be plugins.
      'dependencies': <String>[...plugin.dependencies.where(pluginNames.contains)],
    });
  }
  return directAppDependencies;
}

// The .flutter-plugins file will be DEPRECATED in favor of .flutter-plugins-dependencies.
// TODO(franciscojma): Remove this method once deprecated.
// https://github.com/flutter/flutter/issues/48918
//
/// Writes the .flutter-plugins files based on the list of plugins.
/// If there aren't any plugins, then the files aren't written to disk.
///
/// Finally, returns [true] if .flutter-plugins has changed, otherwise returns [false].
bool _writeFlutterPluginsListLegacy(FlutterProject project, List<Plugin> plugins) {
  final File pluginsFile = project.flutterPluginsFile;
  if (plugins.isEmpty) {
    return ErrorHandlingFileSystem.deleteIfExists(pluginsFile);
  }

  const String info = 'This is a generated file; do not edit or check into version control.';
  final StringBuffer flutterPluginsBuffer = StringBuffer('# $info\n');

  for (final Plugin plugin in plugins) {
    flutterPluginsBuffer.write('${plugin.name}=${globals.fsUtils.escapePath(plugin.path)}\n');
  }
  final String? oldPluginFileContent = _readFileContent(pluginsFile);
  final String pluginFileContent = flutterPluginsBuffer.toString();
  pluginsFile.writeAsStringSync(pluginFileContent, flush: true);

  return oldPluginFileContent != _readFileContent(pluginsFile);
}

/// Returns the contents of [File] or [null] if that file does not exist.
String? _readFileContent(File file) {
  return file.existsSync() ? file.readAsStringSync() : null;
}

const String _androidPluginRegistryTemplateNewEmbedding = '''
package io.flutter.plugins;

import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import io.flutter.Log;

import io.flutter.embedding.engine.FlutterEngine;

/**
 * Generated file. Do not edit.
 * This file is generated by the Flutter tool based on the
 * plugins that support the Android platform.
 */
@Keep
public final class GeneratedPluginRegistrant {
  private static final String TAG = "GeneratedPluginRegistrant";
  public static void registerWith(@NonNull FlutterEngine flutterEngine) {
{{#methodChannelPlugins}}
  {{#supportsEmbeddingV2}}
    try {
      flutterEngine.getPlugins().add(new {{package}}.{{class}}());
    } catch (Exception e) {
      Log.e(TAG, "Error registering plugin {{name}}, {{package}}.{{class}}", e);
    }
  {{/supportsEmbeddingV2}}
{{/methodChannelPlugins}}
  }
}
''';

List<Map<String, Object?>> _extractPlatformMaps(List<Plugin> plugins, String type) {
  return <Map<String, Object?>>[
    for (final Plugin plugin in plugins)
      if (plugin.platforms[type] case final PluginPlatform platformPlugin) platformPlugin.toMap(),
  ];
}

Future<void> _writeAndroidPluginRegistrant(FlutterProject project, List<Plugin> plugins) async {
  final List<Plugin> methodChannelPlugins = _filterMethodChannelPlugins(
    plugins,
    AndroidPlugin.kConfigKey,
  );
  final List<Map<String, Object?>> androidPlugins = _extractPlatformMaps(
    methodChannelPlugins,
    AndroidPlugin.kConfigKey,
  );

  final Map<String, Object> templateContext = <String, Object>{
    'methodChannelPlugins': androidPlugins,
    'androidX': isAppUsingAndroidX(project.android.hostAppGradleRoot),
  };
  final String javaSourcePath = globals.fs.path.join(
    project.android.pluginRegistrantHost.path,
    'src',
    'main',
    'java',
  );
  final String registryPath = globals.fs.path.join(
    javaSourcePath,
    'io',
    'flutter',
    'plugins',
    'GeneratedPluginRegistrant.java',
  );
  const String templateContent = _androidPluginRegistryTemplateNewEmbedding;
  globals.printTrace('Generating $registryPath');
  await _renderTemplateToFile(
    templateContent,
    templateContext,
    globals.fs.file(registryPath),
    globals.templateRenderer,
  );
}

const String _objcPluginRegistryHeaderTemplate = '''
//
//  Generated file. Do not edit.
//

// clang-format off

#ifndef GeneratedPluginRegistrant_h
#define GeneratedPluginRegistrant_h

#import <{{framework}}/{{framework}}.h>

NS_ASSUME_NONNULL_BEGIN

@interface GeneratedPluginRegistrant : NSObject
+ (void)registerWithRegistry:(NSObject<FlutterPluginRegistry>*)registry;
@end

NS_ASSUME_NONNULL_END
#endif /* GeneratedPluginRegistrant_h */
''';

const String _objcPluginRegistryImplementationTemplate = '''
//
//  Generated file. Do not edit.
//

// clang-format off

#import "GeneratedPluginRegistrant.h"

{{#methodChannelPlugins}}
#if __has_include(<{{name}}/{{class}}.h>)
#import <{{name}}/{{class}}.h>
#else
@import {{name}};
#endif

{{/methodChannelPlugins}}
@implementation GeneratedPluginRegistrant

+ (void)registerWithRegistry:(NSObject<FlutterPluginRegistry>*)registry {
{{#methodChannelPlugins}}
  [{{prefix}}{{class}} registerWithRegistrar:[registry registrarForPlugin:@"{{prefix}}{{class}}"]];
{{/methodChannelPlugins}}
}

@end
''';

const String _swiftPluginRegistryTemplate = '''
//
//  Generated file. Do not edit.
//

import {{framework}}
import Foundation

{{#methodChannelPlugins}}
import {{name}}
{{/methodChannelPlugins}}

func RegisterGeneratedPlugins(registry: FlutterPluginRegistry) {
  {{#methodChannelPlugins}}
  {{class}}.register(with: registry.registrar(forPlugin: "{{class}}"))
{{/methodChannelPlugins}}
}
''';

const String _pluginRegistrantPodspecTemplate = '''
#
# Generated file, do not edit.
#

Pod::Spec.new do |s|
  s.name             = 'FlutterPluginRegistrant'
  s.version          = '0.0.1'
  s.summary          = 'Registers plugins with your Flutter app'
  s.description      = <<-DESC
Depends on all your plugins, and provides a function to register them.
                       DESC
  s.homepage         = 'https://flutter.dev'
  s.license          = { :type => 'BSD' }
  s.author           = { 'Flutter Dev Team' => 'flutter-dev@googlegroups.com' }
  s.{{os}}.deployment_target = '{{deploymentTarget}}'
  s.source_files =  "Classes", "Classes/**/*.{h,m}"
  s.source           = { :path => '.' }
  s.public_header_files = './Classes/**/*.h'
  s.static_framework    = true
  s.pod_target_xcconfig = { 'DEFINES_MODULE' => 'YES' }
  s.dependency '{{framework}}'
  {{#methodChannelPlugins}}
  s.dependency '{{name}}'
  {{/methodChannelPlugins}}
end
''';

const String _noopDartPluginRegistryTemplate = '''
// Flutter web plugin registrant file.
//
// Generated file. Do not edit.
//

// ignore_for_file: type=lint

void registerPlugins() {}
''';

const String _dartPluginRegistryTemplate = '''
// Flutter web plugin registrant file.
//
// Generated file. Do not edit.
//

// @dart = 2.13
// ignore_for_file: type=lint

{{#methodChannelPlugins}}
import 'package:{{name}}/{{file}}';
{{/methodChannelPlugins}}
import 'package:flutter_web_plugins/flutter_web_plugins.dart';

void registerPlugins([final Registrar? pluginRegistrar]) {
  final Registrar registrar = pluginRegistrar ?? webPluginRegistrar;
{{#methodChannelPlugins}}
  {{class}}.registerWith(registrar);
{{/methodChannelPlugins}}
  registrar.registerMessageHandler();
}
''';

const String _cppPluginRegistryHeaderTemplate = '''
//
//  Generated file. Do not edit.
//

// clang-format off

#ifndef GENERATED_PLUGIN_REGISTRANT_
#define GENERATED_PLUGIN_REGISTRANT_

#include <flutter/plugin_registry.h>

// Registers Flutter plugins.
void RegisterPlugins(flutter::PluginRegistry* registry);

#endif  // GENERATED_PLUGIN_REGISTRANT_
''';

const String _cppPluginRegistryImplementationTemplate = '''
//
//  Generated file. Do not edit.
//

// clang-format off

#include "generated_plugin_registrant.h"

{{#methodChannelPlugins}}
#include <{{name}}/{{filename}}.h>
{{/methodChannelPlugins}}

void RegisterPlugins(flutter::PluginRegistry* registry) {
{{#methodChannelPlugins}}
  {{class}}RegisterWithRegistrar(
      registry->GetRegistrarForPlugin("{{class}}"));
{{/methodChannelPlugins}}
}
''';

const String _linuxPluginRegistryHeaderTemplate = '''
//
//  Generated file. Do not edit.
//

// clang-format off

#ifndef GENERATED_PLUGIN_REGISTRANT_
#define GENERATED_PLUGIN_REGISTRANT_

#include <flutter_linux/flutter_linux.h>

// Registers Flutter plugins.
void fl_register_plugins(FlPluginRegistry* registry);

#endif  // GENERATED_PLUGIN_REGISTRANT_
''';

const String _linuxPluginRegistryImplementationTemplate = '''
//
//  Generated file. Do not edit.
//

// clang-format off

#include "generated_plugin_registrant.h"

{{#methodChannelPlugins}}
#include <{{name}}/{{filename}}.h>
{{/methodChannelPlugins}}

void fl_register_plugins(FlPluginRegistry* registry) {
{{#methodChannelPlugins}}
  g_autoptr(FlPluginRegistrar) {{name}}_registrar =
      fl_plugin_registry_get_registrar_for_plugin(registry, "{{class}}");
  {{filename}}_register_with_registrar({{name}}_registrar);
{{/methodChannelPlugins}}
}
''';

const String _pluginCmakefileTemplate = r'''
#
# Generated file, do not edit.
#

list(APPEND FLUTTER_PLUGIN_LIST
{{#methodChannelPlugins}}
  {{name}}
{{/methodChannelPlugins}}
)

list(APPEND FLUTTER_FFI_PLUGIN_LIST
{{#ffiPlugins}}
  {{name}}
{{/ffiPlugins}}
)

set(PLUGIN_BUNDLED_LIBRARIES)

foreach(plugin ${FLUTTER_PLUGIN_LIST})
  add_subdirectory({{pluginsDir}}/${plugin}/{{os}} plugins/${plugin})
  target_link_libraries(${BINARY_NAME} PRIVATE ${plugin}_plugin)
  list(APPEND PLUGIN_BUNDLED_LIBRARIES $<TARGET_FILE:${plugin}_plugin>)
  list(APPEND PLUGIN_BUNDLED_LIBRARIES ${${plugin}_bundled_libraries})
endforeach(plugin)

foreach(ffi_plugin ${FLUTTER_FFI_PLUGIN_LIST})
  add_subdirectory({{pluginsDir}}/${ffi_plugin}/{{os}} plugins/${ffi_plugin})
  list(APPEND PLUGIN_BUNDLED_LIBRARIES ${${ffi_plugin}_bundled_libraries})
endforeach(ffi_plugin)
''';

const String _dartPluginRegisterWith = r'''
      try {
        {{dartClass}}.registerWith();
      } catch (err) {
        print(
          '`{{pluginName}}` threw an error: $err. '
          'The app may not function as expected until you remove this plugin from pubspec.yaml'
        );
      }
''';

// TODO(egarciad): Evaluate merging the web and non-web plugin registry templates.
// https://github.com/flutter/flutter/issues/80406
const String _dartPluginRegistryForNonWebTemplate = '''
//
// Generated file. Do not edit.
// This file is generated from template in file `flutter_tools/lib/src/flutter_plugins.dart`.
//

// @dart = {{dartLanguageVersion}}

import 'dart:io'; // flutter_ignore: dart_io_import.
{{#android}}
import 'package:{{pluginName}}/{{dartFileName}}';
{{/android}}
{{#ios}}
import 'package:{{pluginName}}/{{dartFileName}}';
{{/ios}}
{{#linux}}
import 'package:{{pluginName}}/{{dartFileName}}';
{{/linux}}
{{#macos}}
import 'package:{{pluginName}}/{{dartFileName}}';
{{/macos}}
{{#windows}}
import 'package:{{pluginName}}/{{dartFileName}}';
{{/windows}}

@pragma('vm:entry-point')
class _PluginRegistrant {

  @pragma('vm:entry-point')
  static void register() {
    if (Platform.isAndroid) {
      {{#android}}
$_dartPluginRegisterWith
      {{/android}}
    } else if (Platform.isIOS) {
      {{#ios}}
$_dartPluginRegisterWith
      {{/ios}}
    } else if (Platform.isLinux) {
      {{#linux}}
$_dartPluginRegisterWith
      {{/linux}}
    } else if (Platform.isMacOS) {
      {{#macos}}
$_dartPluginRegisterWith
      {{/macos}}
    } else if (Platform.isWindows) {
      {{#windows}}
$_dartPluginRegisterWith
      {{/windows}}
    }
  }
}
''';

Future<void> _writeIOSPluginRegistrant(FlutterProject project, List<Plugin> plugins) async {
  final List<Plugin> methodChannelPlugins = _filterMethodChannelPlugins(
    plugins,
    IOSPlugin.kConfigKey,
  );
  final List<Map<String, Object?>> iosPlugins = _extractPlatformMaps(
    methodChannelPlugins,
    IOSPlugin.kConfigKey,
  );
  final Map<String, Object> context = <String, Object>{
    'os': 'ios',
    'deploymentTarget': '12.0',
    'framework': 'Flutter',
    'methodChannelPlugins': iosPlugins,
  };
  if (project.isModule) {
    final Directory registryDirectory = project.ios.pluginRegistrantHost;
    await _renderTemplateToFile(
      _pluginRegistrantPodspecTemplate,
      context,
      registryDirectory.childFile('FlutterPluginRegistrant.podspec'),
      globals.templateRenderer,
    );
  }
  await _renderTemplateToFile(
    _objcPluginRegistryHeaderTemplate,
    context,
    project.ios.pluginRegistrantHeader,
    globals.templateRenderer,
  );
  await _renderTemplateToFile(
    _objcPluginRegistryImplementationTemplate,
    context,
    project.ios.pluginRegistrantImplementation,
    globals.templateRenderer,
  );
}

/// The relative path from a project's main CMake file to the plugin symlink
/// directory to use in the generated plugin CMake file.
///
/// Because the generated file is checked in, it can't use absolute paths. It is
/// designed to be included by the main CMakeLists.txt, so it relative to
/// that file, rather than the generated file.
String _cmakeRelativePluginSymlinkDirectoryPath(CmakeBasedProject project) {
  final FileSystem fileSystem = project.pluginSymlinkDirectory.fileSystem;
  final String makefileDirPath = project.cmakeFile.parent.absolute.path;
  // CMake always uses posix-style path separators, regardless of the platform.
  final path.Context cmakePathContext = path.Context(style: path.Style.posix);
  final List<String> relativePathComponents = fileSystem.path.split(
    fileSystem.path.relative(project.pluginSymlinkDirectory.absolute.path, from: makefileDirPath),
  );
  return cmakePathContext.joinAll(relativePathComponents);
}

Future<void> _writeLinuxPluginFiles(FlutterProject project, List<Plugin> plugins) async {
  final List<Plugin> methodChannelPlugins = _filterMethodChannelPlugins(
    plugins,
    LinuxPlugin.kConfigKey,
  );
  final List<Map<String, Object?>> linuxMethodChannelPlugins = _extractPlatformMaps(
    methodChannelPlugins,
    LinuxPlugin.kConfigKey,
  );
  final List<Plugin> ffiPlugins = _filterFfiPlugins(plugins, LinuxPlugin.kConfigKey)
    ..removeWhere(methodChannelPlugins.contains);
  final List<Map<String, Object?>> linuxFfiPlugins = _extractPlatformMaps(
    ffiPlugins,
    LinuxPlugin.kConfigKey,
  );
  final Map<String, Object> context = <String, Object>{
    'os': 'linux',
    'methodChannelPlugins': linuxMethodChannelPlugins,
    'ffiPlugins': linuxFfiPlugins,
    'pluginsDir': _cmakeRelativePluginSymlinkDirectoryPath(project.linux),
  };
  await _writeLinuxPluginRegistrant(project.linux.managedDirectory, context);
  await _writePluginCmakefile(
    project.linux.generatedPluginCmakeFile,
    context,
    globals.templateRenderer,
  );
}

Future<void> _writeLinuxPluginRegistrant(
  Directory destination,
  Map<String, Object> templateContext,
) async {
  await _renderTemplateToFile(
    _linuxPluginRegistryHeaderTemplate,
    templateContext,
    destination.childFile('generated_plugin_registrant.h'),
    globals.templateRenderer,
  );
  await _renderTemplateToFile(
    _linuxPluginRegistryImplementationTemplate,
    templateContext,
    destination.childFile('generated_plugin_registrant.cc'),
    globals.templateRenderer,
  );
}

Future<void> _writePluginCmakefile(
  File destinationFile,
  Map<String, Object> templateContext,
  TemplateRenderer templateRenderer,
) async {
  await _renderTemplateToFile(
    _pluginCmakefileTemplate,
    templateContext,
    destinationFile,
    templateRenderer,
  );
}

Future<void> _writeMacOSPluginRegistrant(FlutterProject project, List<Plugin> plugins) async {
  final List<Plugin> methodChannelPlugins = _filterMethodChannelPlugins(
    plugins,
    MacOSPlugin.kConfigKey,
  );
  final List<Map<String, Object?>> macosMethodChannelPlugins = _extractPlatformMaps(
    methodChannelPlugins,
    MacOSPlugin.kConfigKey,
  );
  final Map<String, Object> context = <String, Object>{
    'os': 'macos',
    'framework': 'FlutterMacOS',
    'methodChannelPlugins': macosMethodChannelPlugins,
  };
  await _renderTemplateToFile(
    _swiftPluginRegistryTemplate,
    context,
    project.macos.managedDirectory.childFile('GeneratedPluginRegistrant.swift'),
    globals.templateRenderer,
  );
}

/// Filters out any plugins that don't use method channels, and thus shouldn't be added to the native generated registrants.
List<Plugin> _filterMethodChannelPlugins(List<Plugin> plugins, String platformKey) {
  return plugins.where((Plugin element) {
    final PluginPlatform? plugin = element.platforms[platformKey];
    if (plugin == null) {
      return false;
    }
    if (plugin is NativeOrDartPlugin) {
      return (plugin as NativeOrDartPlugin).hasMethodChannel();
    }
    // Not all platforms have the ability to create Dart-only plugins. Therefore, any plugin that doesn't
    // implement NativeOrDartPlugin is always native.
    return true;
  }).toList();
}

/// Filters out Dart-only and method channel plugins.
///
/// FFI plugins do not need native code registration, but their binaries need to be bundled.
List<Plugin> _filterFfiPlugins(List<Plugin> plugins, String platformKey) {
  return plugins.where((Plugin element) {
    final PluginPlatform? plugin = element.platforms[platformKey];
    if (plugin == null) {
      return false;
    }
    if (plugin is NativeOrDartPlugin) {
      final NativeOrDartPlugin plugin_ = plugin as NativeOrDartPlugin;
      return plugin_.hasFfi();
    }
    return false;
  }).toList();
}

/// Returns only the plugins with the given platform variant.
List<Plugin> _filterPluginsByVariant(
  List<Plugin> plugins,
  String platformKey,
  PluginPlatformVariant variant,
) {
  return plugins.where((Plugin element) {
    final PluginPlatform? platformPlugin = element.platforms[platformKey];
    if (platformPlugin == null) {
      return false;
    }
    assert(platformPlugin is VariantPlatformPlugin);
    return (platformPlugin as VariantPlatformPlugin).supportedVariants.contains(variant);
  }).toList();
}

@visibleForTesting
Future<void> writeWindowsPluginFiles(
  FlutterProject project,
  List<Plugin> plugins,
  TemplateRenderer templateRenderer, {
  Iterable<String>? allowedPlugins,
}) async {
  final List<Plugin> methodChannelPlugins = _filterMethodChannelPlugins(
    plugins,
    WindowsPlugin.kConfigKey,
  );
  if (allowedPlugins != null) {
    final List<Plugin> disallowedPlugins =
        methodChannelPlugins.toList()
          ..removeWhere((Plugin plugin) => allowedPlugins.contains(plugin.name));
    if (disallowedPlugins.isNotEmpty) {
      final StringBuffer buffer = StringBuffer();
      buffer.writeln(
        'The Flutter Preview device does not support the following plugins from your pubspec.yaml:',
      );
      buffer.writeln();
      buffer.writeln(disallowedPlugins.map((Plugin p) => p.name).toList().toString());
      buffer.writeln();
      buffer.writeln(
        'In order to build a Flutter app with plugins, you must use another target platform,',
      );
      buffer.writeln(
        'such as Windows. Type `flutter doctor` into your terminal to see which target platforms',
      );
      buffer.writeln(
        'are ready to be used, and how to get required dependencies for other platforms.',
      );
      throwToolExit(buffer.toString());
    }
  }
  final List<Plugin> win32Plugins = _filterPluginsByVariant(
    methodChannelPlugins,
    WindowsPlugin.kConfigKey,
    PluginPlatformVariant.win32,
  );
  final List<Map<String, Object?>> windowsMethodChannelPlugins = _extractPlatformMaps(
    win32Plugins,
    WindowsPlugin.kConfigKey,
  );
  final List<Plugin> ffiPlugins = _filterFfiPlugins(plugins, WindowsPlugin.kConfigKey)
    ..removeWhere(methodChannelPlugins.contains);
  final List<Map<String, Object?>> windowsFfiPlugins = _extractPlatformMaps(
    ffiPlugins,
    WindowsPlugin.kConfigKey,
  );
  final Map<String, Object> context = <String, Object>{
    'os': 'windows',
    'methodChannelPlugins': windowsMethodChannelPlugins,
    'ffiPlugins': windowsFfiPlugins,
    'pluginsDir': _cmakeRelativePluginSymlinkDirectoryPath(project.windows),
  };
  await _writeCppPluginRegistrant(project.windows.managedDirectory, context, templateRenderer);
  await _writePluginCmakefile(project.windows.generatedPluginCmakeFile, context, templateRenderer);
}

Future<void> _writeCppPluginRegistrant(
  Directory destination,
  Map<String, Object> templateContext,
  TemplateRenderer templateRenderer,
) async {
  await _renderTemplateToFile(
    _cppPluginRegistryHeaderTemplate,
    templateContext,
    destination.childFile('generated_plugin_registrant.h'),
    templateRenderer,
  );
  await _renderTemplateToFile(
    _cppPluginRegistryImplementationTemplate,
    templateContext,
    destination.childFile('generated_plugin_registrant.cc'),
    templateRenderer,
  );
}

Future<void> _writeWebPluginRegistrant(
  FlutterProject project,
  List<Plugin> plugins,
  Directory destination,
) async {
  final List<Map<String, Object?>> webPlugins = _extractPlatformMaps(plugins, WebPlugin.kConfigKey);
  final Map<String, Object> context = <String, Object>{'methodChannelPlugins': webPlugins};

  final File pluginFile = destination.childFile('web_plugin_registrant.dart');

  final String template =
      webPlugins.isEmpty ? _noopDartPluginRegistryTemplate : _dartPluginRegistryTemplate;

  await _renderTemplateToFile(template, context, pluginFile, globals.templateRenderer);
}

/// For each platform that uses them, creates symlinks within the platform
/// directory to each plugin used on that platform.
///
/// If |force| is true, the symlinks will be recreated, otherwise they will
/// be created only if missing.
///
/// This uses [project.flutterPluginsDependenciesFile], so it should only be
/// run after [refreshPluginsList] has been run since the last plugin change.
void createPluginSymlinks(
  FlutterProject project, {
  bool force = false,
  @visibleForTesting FeatureFlags? featureFlagsOverride,
}) {
  final FeatureFlags localFeatureFlags = featureFlagsOverride ?? featureFlags;
  Map<String, Object?>? platformPlugins;
  final String? pluginFileContent = _readFileContent(project.flutterPluginsDependenciesFile);
  if (pluginFileContent != null) {
    final Map<String, Object?>? pluginInfo =
        json.decode(pluginFileContent) as Map<String, Object?>?;
    platformPlugins = pluginInfo?[_kFlutterPluginsPluginListKey] as Map<String, Object?>?;
  }
  platformPlugins ??= <String, Object?>{};

  if (localFeatureFlags.isWindowsEnabled && project.windows.existsSync()) {
    _createPlatformPluginSymlinks(
      project.windows.pluginSymlinkDirectory,
      platformPlugins[project.windows.pluginConfigKey] as List<Object?>?,
      force: force,
    );
  }
  if (localFeatureFlags.isLinuxEnabled && project.linux.existsSync()) {
    _createPlatformPluginSymlinks(
      project.linux.pluginSymlinkDirectory,
      platformPlugins[project.linux.pluginConfigKey] as List<Object?>?,
      force: force,
    );
  }
}

/// Handler for symlink failures which provides specific instructions for known
/// failure cases.
@visibleForTesting
void handleSymlinkException(
  FileSystemException e, {
  required Platform platform,
  required OperatingSystemUtils os,
  required String destination,
  required String source,
}) {
  if (platform.isWindows) {
    // ERROR_ACCESS_DENIED
    if (e.osError?.errorCode == 5) {
      throwToolExit(
        'ERROR_ACCESS_DENIED file system exception thrown while trying to '
        'create a symlink from $source to $destination',
      );
    }
    // ERROR_PRIVILEGE_NOT_HELD, user cannot symlink
    if (e.osError?.errorCode == 1314) {
      final String? versionString = RegExp(r'[\d.]+').firstMatch(os.name)?.group(0);
      final Version? version = Version.parse(versionString);
      // Windows 10 14972 is the oldest version that allows creating symlinks
      // just by enabling developer mode; before that it requires running the
      // terminal as Administrator.
      // https://blogs.windows.com/windowsdeveloper/2016/12/02/symlinks-windows-10/
      final String instructions =
          (version != null && version >= Version(10, 0, 14972))
              ? 'Please enable Developer Mode in your system settings. Run\n'
                  '  start ms-settings:developers\n'
                  'to open settings.'
              : 'You must build from a terminal run as administrator.';
      throwToolExit('Building with plugins requires symlink support.\n\n$instructions');
    }
    // ERROR_INVALID_FUNCTION, trying to link across drives, which is not supported
    if (e.osError?.errorCode == 1) {
      throwToolExit(
        'Creating symlink from $source to $destination failed with '
        'ERROR_INVALID_FUNCTION. Try moving your Flutter project to the same '
        'drive as your Flutter SDK.',
      );
    }
  }
}

/// Creates [symlinkDirectory] containing symlinks to each plugin listed in [platformPlugins].
///
/// If [force] is true, the directory will be created only if missing.
void _createPlatformPluginSymlinks(
  Directory symlinkDirectory,
  List<Object?>? platformPlugins, {
  bool force = false,
}) {
  if (force) {
    // Start fresh to avoid stale links.
    ErrorHandlingFileSystem.deleteIfExists(symlinkDirectory, recursive: true);
  }
  symlinkDirectory.createSync(recursive: true);
  if (platformPlugins == null) {
    return;
  }
  for (final Map<String, Object?> pluginInfo in platformPlugins.cast<Map<String, Object?>>()) {
    final String name = pluginInfo[_kFlutterPluginsNameKey]! as String;
    final String path = pluginInfo[_kFlutterPluginsPathKey]! as String;
    final Link link = symlinkDirectory.childLink(name);
    if (link.existsSync()) {
      continue;
    }
    try {
      link.createSync(path);
    } on FileSystemException catch (e) {
      handleSymlinkException(
        e,
        platform: globals.platform,
        os: globals.os,
        destination: link.path,
        source: path,
      );
      rethrow;
    }
  }
}

/// Rewrites the `.flutter-plugins` file of [project] based on the plugin
/// dependencies declared in `pubspec.yaml`.
///
/// Assumes `pub get` has been executed since last change to `pubspec.yaml`.
Future<void> refreshPluginsList(
  FlutterProject project, {
  bool iosPlatform = false,
  bool macOSPlatform = false,
  bool forceCocoaPodsOnly = false,
  bool? determineDevDependencies,
  bool? generateLegacyPlugins,
}) async {
  final List<Plugin> plugins = await findPlugins(
    project,
    determineDevDependencies: determineDevDependencies,
  );
  // Sort the plugins by name to keep ordering stable in generated files.
  plugins.sort((Plugin left, Plugin right) => left.name.compareTo(right.name));
  // TODO(matanlurey): Remove once migration is complete.
  // Write the legacy plugin files to avoid breaking existing apps.
  generateLegacyPlugins ??= !featureFlags.isExplicitPackageDependenciesEnabled;
  final bool legacyChanged =
      generateLegacyPlugins && _writeFlutterPluginsListLegacy(project, plugins);

  bool swiftPackageManagerEnabledIos = false;
  bool swiftPackageManagerEnabledMacos = false;
  if (!forceCocoaPodsOnly) {
    if (iosPlatform) {
      swiftPackageManagerEnabledIos = project.ios.usesSwiftPackageManager;
    }
    if (macOSPlatform) {
      swiftPackageManagerEnabledMacos = project.macos.usesSwiftPackageManager;
    }
  }

  final bool changed = _writeFlutterPluginsList(
    project,
    plugins,
    swiftPackageManagerEnabledIos: swiftPackageManagerEnabledIos,
    swiftPackageManagerEnabledMacos: swiftPackageManagerEnabledMacos,
  );
  if (changed || legacyChanged || forceCocoaPodsOnly) {
    createPluginSymlinks(project, force: true);
    if (iosPlatform) {
      globals.cocoaPods?.invalidatePodInstallOutput(project.ios);
    }
    if (macOSPlatform) {
      globals.cocoaPods?.invalidatePodInstallOutput(project.macos);
    }
  }
}

/// Injects plugins found in `pubspec.yaml` into the platform-specific projects
/// only at build-time.
///
/// This method is similar to [injectPlugins], but used only for platforms where
/// the plugin files are not required when the app is created (currently: Web).
///
/// This method will create files in the temporary flutter build directory
/// specified by `destination`.
///
/// In the Web platform, `destination` can point to a real filesystem (`flutter build`)
/// or an in-memory filesystem (`flutter run`).
///
/// This method is also used by [WebProject.ensureReadyForPlatformSpecificTooling]
/// to inject a copy of the plugin registrant for web into .dart_tool/dartpad so
/// dartpad can get the plugin registrant without needing to build the complete
/// project. See: https://github.com/dart-lang/dart-services/pull/874
Future<void> injectBuildTimePluginFilesForWebPlatform(
  FlutterProject project, {
  required Directory destination,
}) async {
  final List<Plugin> plugins = await findPlugins(project);
  final Map<String, List<Plugin>> pluginsByPlatform = _resolvePluginImplementations(
    plugins,
    pluginResolutionType: _PluginResolutionType.nativeOrDart,
  );
  await _writeWebPluginRegistrant(project, pluginsByPlatform[WebPlugin.kConfigKey]!, destination);
}

/// Injects plugins found in `pubspec.yaml` into the platform-specific projects.
///
/// The injected files are required by the flutter app as soon as possible, so
/// it can be built.
///
/// Files written by this method end up in platform-specific locations that are
/// configured by each [FlutterProject] subclass (except for the Web).
///
/// Web tooling uses [injectBuildTimePluginFilesForWebPlatform] instead, which places files in the
/// current build (temp) directory, and doesn't modify the users' working copy.
///
/// Assumes [refreshPluginsList] has been called since last change to `pubspec.yaml`.
Future<void> injectPlugins(
  FlutterProject project, {
  bool androidPlatform = false,
  bool iosPlatform = false,
  bool linuxPlatform = false,
  bool macOSPlatform = false,
  bool windowsPlatform = false,
  Iterable<String>? allowedPlugins,
  DarwinDependencyManagement? darwinDependencyManagement,
}) async {
  final List<Plugin> plugins = await findPlugins(project);
  final Map<String, List<Plugin>> pluginsByPlatform = _resolvePluginImplementations(
    plugins,
    pluginResolutionType: _PluginResolutionType.nativeOrDart,
  );

  if (androidPlatform) {
    await _writeAndroidPluginRegistrant(project, pluginsByPlatform[AndroidPlugin.kConfigKey]!);
  }
  if (iosPlatform) {
    await _writeIOSPluginRegistrant(project, pluginsByPlatform[IOSPlugin.kConfigKey]!);
  }
  if (linuxPlatform) {
    await _writeLinuxPluginFiles(project, pluginsByPlatform[LinuxPlugin.kConfigKey]!);
  }
  if (macOSPlatform) {
    await _writeMacOSPluginRegistrant(project, pluginsByPlatform[MacOSPlugin.kConfigKey]!);
  }
  if (windowsPlatform) {
    await writeWindowsPluginFiles(
      project,
      pluginsByPlatform[WindowsPlugin.kConfigKey]!,
      globals.templateRenderer,
      allowedPlugins: allowedPlugins,
    );
  }
  if (iosPlatform || macOSPlatform) {
    final DarwinDependencyManagement darwinDependencyManagerSetup =
        darwinDependencyManagement ??
        DarwinDependencyManagement(
          project: project,
          plugins: plugins,
          cocoapods: globals.cocoaPods!,
          swiftPackageManager: SwiftPackageManager(
            fileSystem: globals.fs,
            templateRenderer: globals.templateRenderer,
          ),
          fileSystem: globals.fs,
          logger: globals.logger,
        );
    if (iosPlatform) {
      await darwinDependencyManagerSetup.setUp(platform: SupportedPlatform.ios);
    }
    if (macOSPlatform) {
      await darwinDependencyManagerSetup.setUp(platform: SupportedPlatform.macos);
    }
  }
}

/// Returns whether the specified Flutter [project] has any plugin dependencies.
///
/// Assumes [refreshPluginsList] has been called since last change to `pubspec.yaml`.
bool hasPlugins(FlutterProject project) {
  return _readFileContent(project.flutterPluginsDependenciesFile) != null;
}

/// Resolves the plugin implementations for all platforms.
///
///   * If there is only one dependency on a package that implements the
///     frontend plugin for the current platform, use that.
///   * If there is a single direct dependency on a package that implements the
///     frontend plugin for the current platform, use that.
///   * If there is no direct dependency on a package that implements the
///     frontend plugin, but there is a default for the current platform,
///     use that.
///   * Else fail.
///
///  For more details, https://flutter.dev/go/federated-plugins.
///
/// If [selectDartPluginsOnly] is enabled, only Dart plugin implementations are
/// considered. Else, native and Dart plugin implementations are considered.
List<PluginInterfaceResolution> resolvePlatformImplementation(
  List<Plugin> plugins, {
  required bool selectDartPluginsOnly,
}) {
  final Map<String, List<Plugin>> resolution = _resolvePluginImplementations(
    plugins,
    pluginResolutionType:
        selectDartPluginsOnly ? _PluginResolutionType.dart : _PluginResolutionType.nativeOrDart,
  );
  return resolution.entries.expand((MapEntry<String, List<Plugin>> entry) {
    return entry.value.map((Plugin plugin) {
      return PluginInterfaceResolution(plugin: plugin, platform: entry.key);
    });
  }).toList();
}

/// Resolves the plugin implementations for all platforms,
/// see [resolvePlatformImplementation].
///
/// Only plugins which provide the according platform implementation are returned.
Map<String, List<Plugin>> _resolvePluginImplementations(
  List<Plugin> plugins, {
  required _PluginResolutionType pluginResolutionType,
}) {
  final Map<String, List<Plugin>> pluginsByPlatform = <String, List<Plugin>>{
    AndroidPlugin.kConfigKey: <Plugin>[],
    IOSPlugin.kConfigKey: <Plugin>[],
    LinuxPlugin.kConfigKey: <Plugin>[],
    MacOSPlugin.kConfigKey: <Plugin>[],
    WindowsPlugin.kConfigKey: <Plugin>[],
    WebPlugin.kConfigKey: <Plugin>[],
  };

  bool hasPluginPubspecError = false;
  bool hasResolutionError = false;

  for (final String platformKey in pluginsByPlatform.keys) {
    final (
      List<Plugin> platformPluginResolutions,
      bool hasPlatformPluginPubspecError,
      bool hasPlatformResolutionError,
    ) = _resolvePluginImplementationsByPlatform(
      plugins,
      platformKey,
      pluginResolutionType: pluginResolutionType,
    );

    if (hasPlatformPluginPubspecError) {
      hasPluginPubspecError = true;
    } else if (hasPlatformResolutionError) {
      hasResolutionError = true;
    } else {
      pluginsByPlatform[platformKey] = platformPluginResolutions;
    }
  }
  if (hasPluginPubspecError) {
    throwToolExit('Please resolve the plugin pubspec errors');
  }
  if (hasResolutionError) {
    throwToolExit('Please resolve the plugin implementation selection errors');
  }
  return pluginsByPlatform;
}

/// Resolves the plugins for the given [platformKey] (Dart-only or native
/// implementations).
(List<Plugin> pluginImplementations, bool hasPluginPubspecError, bool hasResolutionError)
_resolvePluginImplementationsByPlatform(
  Iterable<Plugin> plugins,
  String platformKey, {
  _PluginResolutionType pluginResolutionType = _PluginResolutionType.nativeOrDart,
}) {
  bool hasPluginPubspecError = false;
  bool hasResolutionError = false;

  // Key: the plugin name, value: the list of plugin candidates for the implementation of [platformKey].
  final Map<String, List<Plugin>> pluginImplCandidates = <String, List<Plugin>>{};

  // Key: the plugin name, value: the plugin of the default implementation of [platformKey].
  final Map<String, Plugin> defaultImplementations = <String, Plugin>{};

  for (final Plugin plugin in plugins) {
    final String? error = _validatePlugin(
      plugin,
      platformKey,
      pluginResolutionType: pluginResolutionType,
    );
    if (error != null) {
      globals.printError(error);
      hasPluginPubspecError = true;
      continue;
    }
    final String? implementsPluginName = _getImplementedPlugin(
      plugin,
      platformKey,
      pluginResolutionType: pluginResolutionType,
    );
    final String? defaultImplPluginName = _getDefaultImplPlugin(
      plugin,
      platformKey,
      pluginResolutionType: pluginResolutionType,
    );

    if (defaultImplPluginName != null) {
      final Plugin? defaultPackage =
          plugins.where((Plugin plugin) => plugin.name == defaultImplPluginName).firstOrNull;
      if (defaultPackage != null) {
        if (_hasPluginInlineImpl(
          defaultPackage,
          platformKey,
          pluginResolutionType: _PluginResolutionType.nativeOrDart,
        )) {
          if (pluginResolutionType == _PluginResolutionType.nativeOrDart ||
              _hasPluginInlineImpl(
                defaultPackage,
                platformKey,
                pluginResolutionType: pluginResolutionType,
              )) {
            // Each plugin can only have one default implementation for this [platformKey].
            defaultImplementations[plugin.name] = defaultPackage;
            // No need to add the default plugin to `pluginImplCandidates`,
            // as if the plugin is present and provides an implementation
            // it is added via `_getImplementedPlugin`.
          }
        } else {
          // Only warn, if neither an implementation for native nor for Dart is given.
          globals.printWarning(
            'Package ${plugin.name}:$platformKey references $defaultImplPluginName:$platformKey as the default plugin, but it does not provide an inline implementation.\n'
            'Ask the maintainers of ${plugin.name} to either avoid referencing a default implementation via `platforms: $platformKey: default_package: $defaultImplPluginName` '
            'or add an inline implementation to $defaultImplPluginName via `platforms: $platformKey:` `pluginClass` or `dartPluginClass`.\n',
          );
        }
      } else {
        globals.printWarning(
          'Package ${plugin.name}:$platformKey references $defaultImplPluginName:$platformKey as the default plugin, but the package does not exist, or is not a plugin package.\n'
          'Ask the maintainers of ${plugin.name} to either avoid referencing a default implementation via `platforms: $platformKey: default_package: $defaultImplPluginName` '
          'or create a plugin named $defaultImplPluginName.\n',
        );
      }
    }
    if (implementsPluginName != null) {
      pluginImplCandidates.putIfAbsent(implementsPluginName, () => <Plugin>[]);
      pluginImplCandidates[implementsPluginName]!.add(plugin);
    }
  }

  // Key: the plugin name, value: the plugin which provides an implementation for [platformKey].
  final Map<String, Plugin> pluginResolution = <String, Plugin>{};

  // Now resolve all the possible resolutions to a single option for each
  // plugin, or throw if that's not possible.
  for (final MapEntry<String, List<Plugin>> implCandidatesEntry in pluginImplCandidates.entries) {
    final (Plugin? resolution, String? error) = _resolveImplementationOfPlugin(
      platformKey: platformKey,
      pluginResolutionType: pluginResolutionType,
      pluginName: implCandidatesEntry.key,
      candidates: implCandidatesEntry.value,
      defaultPackage: defaultImplementations[implCandidatesEntry.key],
    );
    if (error != null) {
      globals.printError(error);
      hasResolutionError = true;
    } else if (resolution != null) {
      pluginResolution[implCandidatesEntry.key] = resolution;
    }
  }

  // Sort the plugins by name to keep ordering stable in generated files.
  final List<Plugin> pluginImplementations =
      pluginResolution.values.toList()
        ..sort((Plugin left, Plugin right) => left.name.compareTo(right.name));
  return (pluginImplementations, hasPluginPubspecError, hasResolutionError);
}

/// Validates conflicting plugin parameters in pubspec, such as
/// `dartPluginClass`, `default_package` and `implements`.
///
/// Returns an error, if failing.
String? _validatePlugin(
  Plugin plugin,
  String platformKey, {
  required _PluginResolutionType pluginResolutionType,
}) {
  final String? implementsPackage = plugin.implementsPackage;
  final String? defaultImplPluginName = plugin.defaultPackagePlatforms[platformKey];

  if (plugin.name == implementsPackage && plugin.name == defaultImplPluginName) {
    // Allow self implementing and self as platform default.
    return null;
  }

  if (defaultImplPluginName != null) {
    if (implementsPackage != null && implementsPackage.isNotEmpty) {
      return 'Plugin ${plugin.name}:$platformKey provides an implementation for $implementsPackage '
          'and also references a default implementation for $defaultImplPluginName, which is currently not supported. '
          'Ask the maintainers of ${plugin.name} to either remove the implementation via `implements: $implementsPackage` '
          'or avoid referencing a default implementation via `platforms: $platformKey: default_package: $defaultImplPluginName`.\n';
    }

    if (_hasPluginInlineImpl(plugin, platformKey, pluginResolutionType: pluginResolutionType)) {
      return 'Plugin ${plugin.name}:$platformKey which provides an inline implementation '
          'cannot also reference a default implementation for $defaultImplPluginName. '
          'Ask the maintainers of ${plugin.name} to either remove the implementation via `platforms: $platformKey:${pluginResolutionType == _PluginResolutionType.dart ? ' dartPluginClass' : '` `pluginClass` or `dartPluginClass'}` '
          'or avoid referencing a default implementation via `platforms: $platformKey: default_package: $defaultImplPluginName`.\n';
    }
  }
  return null;
}

/// Determine if this [plugin] serves as implementation for an app-facing
/// package for the given platform [platformKey].
///
/// If so, return the package name, which the [plugin] implements.
///
/// Options:
///   * The [plugin] (e.g. 'url_launcher_linux') serves as implementation for
///     an app-facing package (e.g. 'url_launcher').
///   * The [plugin] (e.g. 'url_launcher') implements itself and then also
///     serves as its own default implementation.
///   * The [plugin] does not provide an implementation.
String? _getImplementedPlugin(
  Plugin plugin,
  String platformKey, {
  _PluginResolutionType pluginResolutionType = _PluginResolutionType.nativeOrDart,
}) {
  if (_hasPluginInlineImpl(plugin, platformKey, pluginResolutionType: pluginResolutionType)) {
    // Only can serve, if the plugin has an inline implementation.

    final String? implementsPackage = plugin.implementsPackage;
    if (implementsPackage != null && implementsPackage.isNotEmpty) {
      // The inline plugin implements another package.
      return implementsPackage;
    }

    if (pluginResolutionType == _PluginResolutionType.nativeOrDart ||
        _isEligibleDartSelfImpl(plugin, platformKey)) {
      // The inline plugin implements itself.
      return plugin.name;
    }
  }

  return null;
}

/// Determine if this [plugin] (or package) references a default plugin with an
/// implementation for the given platform [platformKey].
///
/// If so, return the plugin name, which provides the default implementation.
///
/// Options:
///   * The [plugin] (e.g. 'url_launcher') references a default implementation
///     (e.g. 'url_launcher_linux').
///   * The [plugin] (e.g. 'url_launcher') implements itself and then also
///     serves as its own default implementation.
///   * The [plugin] does not reference a default implementation.
String? _getDefaultImplPlugin(
  Plugin plugin,
  String platformKey, {
  _PluginResolutionType pluginResolutionType = _PluginResolutionType.nativeOrDart,
}) {
  final String? defaultImplPluginName = plugin.defaultPackagePlatforms[platformKey];
  if (defaultImplPluginName != null) {
    return defaultImplPluginName;
  }

  if (_hasPluginInlineImpl(plugin, platformKey, pluginResolutionType: pluginResolutionType) &&
      (pluginResolutionType == _PluginResolutionType.nativeOrDart ||
          _isEligibleDartSelfImpl(plugin, platformKey))) {
    // The inline plugin serves as its own default implementation.
    return plugin.name;
  }

  return null;
}

/// Determine if the [plugin]'s inline dart implementation for the
/// [platformKey] is eligible to serve as its own default.
///
/// An app-facing package (i.e., one with no 'implements') with an
/// inline implementation should be its own default implementation.
/// Desktop platforms originally did not work that way, and enabling
/// it unconditionally would break existing published plugins, so
/// only treat it as such if either:
/// - the platform is not desktop, or
/// - the plugin requires at least Flutter 2.11 (when this opt-in logic
///   was added), so that existing plugins continue to work.
/// See https://github.com/flutter/flutter/issues/87862 for details.
bool _isEligibleDartSelfImpl(Plugin plugin, String platformKey) {
  final bool isDesktop =
      platformKey == 'linux' || platformKey == 'macos' || platformKey == 'windows';
  final semver.VersionConstraint? flutterConstraint = plugin.flutterConstraint;
  final semver.Version? minFlutterVersion =
      flutterConstraint != null && flutterConstraint is semver.VersionRange
          ? flutterConstraint.min
          : null;
  final bool hasMinVersionForImplementsRequirement =
      minFlutterVersion != null && minFlutterVersion.compareTo(semver.Version(2, 11, 0)) >= 0;
  return !isDesktop || hasMinVersionForImplementsRequirement;
}

/// Determine if the plugin provides an inline implementation.
bool _hasPluginInlineImpl(
  Plugin plugin,
  String platformKey, {
  required _PluginResolutionType pluginResolutionType,
}) {
  return pluginResolutionType == _PluginResolutionType.nativeOrDart &&
          plugin.platforms[platformKey] != null ||
      pluginResolutionType == _PluginResolutionType.dart &&
          _hasPluginInlineDartImpl(plugin, platformKey);
}

/// Determine if the plugin provides an inline Dart implementation.
bool _hasPluginInlineDartImpl(Plugin plugin, String platformKey) {
  final DartPluginClassAndFilePair? platformInfo = plugin.pluginDartClassPlatforms[platformKey];
  return platformInfo != null && platformInfo.dartClass != 'none';
}

/// Get the resolved plugin [resolution] from the [candidates] serving as implementation for
/// [pluginName].
///
/// Returns an [error] string, if failing.
(Plugin? resolution, String? error) _resolveImplementationOfPlugin({
  required String platformKey,
  required _PluginResolutionType pluginResolutionType,
  required String pluginName,
  required List<Plugin> candidates,
  Plugin? defaultPackage,
}) {
  // If there's only one candidate, use it.
  if (candidates.length == 1) {
    return (candidates.first, null);
  }
  // Next, try direct dependencies of the resolving application.
  final Iterable<Plugin> directDependencies = candidates.where((Plugin plugin) {
    return plugin.isDirectDependency;
  });
  if (directDependencies.isNotEmpty) {
    if (directDependencies.length > 1) {
      // Allow overriding an app-facing package with an inline implementation (which is a direct dependency)
      // with another direct dependency which implements the app-facing package.
      final Iterable<Plugin> implementingPackage = directDependencies.where(
        (Plugin plugin) => plugin.implementsPackage != null && plugin.implementsPackage!.isNotEmpty,
      );
      final Set<Plugin> appFacingPackage =
          directDependencies.toSet()..removeAll(implementingPackage);
      if (implementingPackage.length == 1 && appFacingPackage.length == 1) {
        return (implementingPackage.first, null);
      }

      return (
        null,
        'Plugin $pluginName:$platformKey has conflicting direct dependency implementations:\n'
            '${directDependencies.map((Plugin plugin) => '  ${plugin.name}\n').join()}'
            'To fix this issue, remove all but one of these dependencies from pubspec.yaml.\n',
      );
    } else {
      return (directDependencies.first, null);
    }
  }
  // Next, defer to the default implementation if there is one.
  if (defaultPackage != null && candidates.contains(defaultPackage)) {
    // By definition every candidate has an inline implementation
    assert(
      _hasPluginInlineImpl(defaultPackage, platformKey, pluginResolutionType: pluginResolutionType),
    );
    return (defaultPackage, null);
  }
  // Otherwise, require an explicit choice.
  if (candidates.length > 1) {
    return (
      null,
      'Plugin $pluginName:$platformKey has multiple possible implementations:\n'
          '${candidates.map((Plugin plugin) => '  ${plugin.name}\n').join()}'
          'To fix this issue, add one of these dependencies to pubspec.yaml.\n',
    );
  }
  // No implementation provided
  return (null, null);
}

/// Generates the Dart plugin registrant, which allows to bind a platform
/// implementation of a Dart only plugin to its interface.
/// The new entrypoint wraps [currentMainUri], adds the [_PluginRegistrant] class,
/// and writes the file to [newMainDart].
///
/// [mainFile] is the main entrypoint file. e.g. /<app>/lib/main.dart.
///
/// A successful run will create a new generate_main.dart file or update the existing file.
/// Throws [ToolExit] if unable to generate the file.
///
/// For more details, see https://flutter.dev/go/federated-plugins.
Future<void> generateMainDartWithPluginRegistrant(
  FlutterProject rootProject,
  PackageConfig packageConfig,
  String currentMainUri,
  File mainFile,
) async {
  final List<Plugin> plugins = await findPlugins(rootProject);
  final List<PluginInterfaceResolution> resolutions = resolvePlatformImplementation(
    plugins,
    selectDartPluginsOnly: true,
  );
  final LanguageVersion entrypointVersion = determineLanguageVersion(
    mainFile,
    packageConfig.packageOf(mainFile.absolute.uri),
    Cache.flutterRoot!,
  );
  final Map<String, Object> templateContext = <String, Object>{
    'mainEntrypoint': currentMainUri,
    'dartLanguageVersion': entrypointVersion.toString(),
    AndroidPlugin.kConfigKey: <Object?>[],
    IOSPlugin.kConfigKey: <Object?>[],
    LinuxPlugin.kConfigKey: <Object?>[],
    MacOSPlugin.kConfigKey: <Object?>[],
    WindowsPlugin.kConfigKey: <Object?>[],
  };
  final File newMainDart = rootProject.dartPluginRegistrant;
  if (resolutions.isEmpty) {
    try {
      if (await newMainDart.exists()) {
        await newMainDart.delete();
      }
    } on FileSystemException catch (error) {
      globals.printWarning(
        'Unable to remove ${newMainDart.path}, received error: $error.\n'
        'You might need to run flutter clean.',
      );
      rethrow;
    }
    return;
  }
  for (final PluginInterfaceResolution resolution in resolutions) {
    assert(templateContext.containsKey(resolution.platform));
    (templateContext[resolution.platform] as List<Object?>?)?.add(resolution.toMap());
  }
  try {
    await _renderTemplateToFile(
      _dartPluginRegistryForNonWebTemplate,
      templateContext,
      newMainDart,
      globals.templateRenderer,
    );
  } on FileSystemException catch (error) {
    globals.printError('Unable to write ${newMainDart.path}, received error: $error');
    rethrow;
  }
}
