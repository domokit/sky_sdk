// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'localizations/gen_l10n.dart' as gen_l10n;
library;

import 'package:meta/meta.dart';
import 'package:pub_semver/pub_semver.dart';
import 'package:yaml/yaml.dart';

import 'base/deferred_component.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/utils.dart';
import 'globals.dart' as globals;
import 'plugins.dart';

/// Whether or not Impeller Scene 3D model import is enabled.
const bool kIs3dSceneSupported = true;

const Set<String> _kValidPluginPlatforms = <String>{
  'android',
  'ios',
  'web',
  'windows',
  'linux',
  'macos',
};

/// A wrapper around the `flutter` section in the `pubspec.yaml` file.
class FlutterManifest {
  FlutterManifest._({required Logger logger}) : _logger = logger;

  /// Returns an empty manifest.
  factory FlutterManifest.empty({required Logger logger}) = FlutterManifest._;

  /// Returns null on invalid manifest. Returns empty manifest on missing file.
  static FlutterManifest? createFromPath(
    String path, {
    required FileSystem fileSystem,
    required Logger logger,
  }) {
    if (!fileSystem.isFileSync(path)) {
      return _createFromYaml(null, logger);
    }
    final String manifest = fileSystem.file(path).readAsStringSync();
    return FlutterManifest.createFromString(manifest, logger: logger);
  }

  /// Returns null on missing or invalid manifest.
  @visibleForTesting
  static FlutterManifest? createFromString(String manifest, {required Logger logger}) {
    return _createFromYaml(loadYaml(manifest), logger);
  }

  static FlutterManifest? _createFromYaml(Object? yamlDocument, Logger logger) {
    if (yamlDocument != null && !_validate(yamlDocument, logger)) {
      return null;
    }

    final FlutterManifest pubspec = FlutterManifest._(logger: logger);
    final Map<Object?, Object?>? yamlMap = yamlDocument as YamlMap?;
    if (yamlMap != null) {
      pubspec._descriptor = yamlMap.cast<String, Object?>();
    }

    final Map<Object?, Object?>? flutterMap =
        pubspec._descriptor['flutter'] as Map<Object?, Object?>?;
    if (flutterMap != null) {
      pubspec._flutterDescriptor = flutterMap.cast<String, Object?>();
    }

    return pubspec;
  }

  /// Creates a copy of the current manifest with some subset of properties
  /// modified.
  FlutterManifest copyWith({
    required Logger logger,
    List<AssetsEntry>? assets,
    List<Font>? fonts,
    List<Uri>? shaders,
    List<Uri>? models,
    List<DeferredComponent>? deferredComponents,
  }) {
    final FlutterManifest copy = FlutterManifest._(logger: _logger);
    copy._descriptor = <String, Object?>{..._descriptor};
    copy._flutterDescriptor = <String, Object?>{..._flutterDescriptor};

    if (assets != null && assets.isNotEmpty) {
      copy._flutterDescriptor['assets'] = YamlList.wrap(<Object?>[
        for (final AssetsEntry asset in assets) asset.descriptor,
      ]);
    }

    if (fonts != null && fonts.isNotEmpty) {
      copy._flutterDescriptor['fonts'] = YamlList.wrap(<Map<String, Object?>>[
        for (final Font font in fonts) font.descriptor,
      ]);
    }

    if (shaders != null && shaders.isNotEmpty) {
      copy._flutterDescriptor['shaders'] = YamlList.wrap(
        shaders.map((Uri uri) => uri.toString()).toList(),
      );
    }

    if (models != null && models.isNotEmpty) {
      copy._flutterDescriptor['models'] = YamlList.wrap(
        models.map((Uri uri) => uri.toString()).toList(),
      );
    }

    if (deferredComponents != null && deferredComponents.isNotEmpty) {
      copy._flutterDescriptor['deferred-components'] = YamlList.wrap(
        deferredComponents.map((DeferredComponent dc) => dc.descriptor).toList(),
      );
    }

    copy._descriptor['flutter'] = YamlMap.wrap(copy._flutterDescriptor);

    if (!_validate(YamlMap.wrap(copy._descriptor), logger)) {
      throw StateError('Generated invalid pubspec.yaml.');
    }

    return copy;
  }

  final Logger _logger;

  /// A map representation of the entire `pubspec.yaml` file.
  Map<String, Object?> _descriptor = <String, Object?>{};

  /// A map representation of the `flutter` section in the `pubspec.yaml` file.
  Map<String, Object?> _flutterDescriptor = <String, Object?>{};

  Map<String, Object?> get flutterDescriptor => _flutterDescriptor;

  /// True if the `pubspec.yaml` file does not exist.
  bool get isEmpty => _descriptor.isEmpty;

  /// The string value of the top-level `name` property in the `pubspec.yaml` file.
  String get appName => _descriptor['name'] as String? ?? '';

  /// Contains the name of the dependencies.
  /// These are the keys specified in the `dependency` map.
  Set<String> get dependencies {
    final YamlMap? dependencies = _descriptor['dependencies'] as YamlMap?;
    return dependencies != null ? <String>{...dependencies.keys.cast<String>()} : <String>{};
  }

  // Flag to avoid printing multiple invalid version messages.
  bool _hasShowInvalidVersionMsg = false;

  /// The version String from the `pubspec.yaml` file.
  /// Can be null if it isn't set or has a wrong format.
  String? get appVersion {
    final String? verStr = _descriptor['version']?.toString();
    if (verStr == null) {
      return null;
    }

    Version? version;
    try {
      version = Version.parse(verStr);
    } on Exception {
      if (!_hasShowInvalidVersionMsg) {
        _logger.printStatus(
          globals.userMessages.invalidVersionSettingHintMessage(verStr),
          emphasis: true,
        );
        _hasShowInvalidVersionMsg = true;
      }
    }
    return version?.toString();
  }

  /// The build version name from the `pubspec.yaml` file.
  /// Can be null if version isn't set or has a wrong format.
  String? get buildName {
    final String? version = appVersion;
    if (version != null && version.contains('+')) {
      return version.split('+').elementAt(0);
    }
    return version;
  }

  /// The build version number from the `pubspec.yaml` file.
  /// Can be null if version isn't set or has a wrong format.
  String? get buildNumber {
    final String? version = appVersion;
    if (version != null && version.contains('+')) {
      final String value = version.split('+').elementAt(1);
      return value;
    } else {
      return null;
    }
  }

  bool get usesMaterialDesign {
    return _flutterDescriptor['uses-material-design'] as bool? ?? false;
  }

  /// If true, does not use Swift Package Manager as a dependency manager.
  /// CocoaPods will be used instead.
  bool get disabledSwiftPackageManager {
    return _flutterDescriptor['disable-swift-package-manager'] as bool? ?? false;
  }

  /// True if this Flutter module should use AndroidX dependencies.
  ///
  /// If false the deprecated Android Support library will be used.
  bool get usesAndroidX {
    final Object? module = _flutterDescriptor['module'];
    if (module is YamlMap) {
      return module['androidX'] == true;
    }
    return false;
  }

  /// Any additional license files listed under the `flutter` key.
  ///
  /// This is expected to be a list of file paths that should be treated as
  /// relative to the pubspec in this directory.
  ///
  /// For example:
  ///
  /// ```yaml
  /// flutter:
  ///   licenses:
  ///     - assets/foo_license.txt
  /// ```
  List<String> get additionalLicenses {
    return <String>[
      if (_flutterDescriptor case {'licenses': final YamlList list})
        for (final Object? item in list) '$item',
    ];
  }

  /// True if this manifest declares a Flutter module project.
  ///
  /// A Flutter project is considered a module when it has a `module:`
  /// descriptor. A Flutter module project supports integration into an
  /// existing host app, and has managed platform host code.
  ///
  /// Such a project can be created using `flutter create -t module`.
  bool get isModule => _flutterDescriptor.containsKey('module');

  /// True if this manifest declares a Flutter plugin project.
  ///
  /// A Flutter project is considered a plugin when it has a `plugin:`
  /// descriptor. A Flutter plugin project wraps custom Android and/or
  /// iOS code in a Dart interface for consumption by other Flutter app
  /// projects.
  ///
  /// Such a project can be created using `flutter create -t plugin`.
  bool get isPlugin => _flutterDescriptor.containsKey('plugin');

  /// Returns the Android package declared by this manifest in its
  /// module or plugin descriptor. Returns null, if there is no
  /// such declaration.
  String? get androidPackage {
    if (isModule) {
      if (_flutterDescriptor case {'module': final YamlMap map}) {
        return map['androidPackage'] as String?;
      }
    }

    late final YamlMap? plugin = _flutterDescriptor['plugin'] as YamlMap?;

    return switch (supportedPlatforms) {
      {'android': final YamlMap map} => map['package'] as String?,
      // Pre-multi-platform plugin format
      null when isPlugin => plugin?['androidPackage'] as String?,
      _ => null,
    };
  }

  /// Returns the deferred components configuration if declared. Returns
  /// null if no deferred components are declared.
  late final List<DeferredComponent>? deferredComponents = computeDeferredComponents();
  List<DeferredComponent>? computeDeferredComponents() {
    if (!_flutterDescriptor.containsKey('deferred-components')) {
      return null;
    }
    final List<DeferredComponent> components = <DeferredComponent>[];
    final Object? deferredComponents = _flutterDescriptor['deferred-components'];
    if (deferredComponents is! YamlList) {
      return components;
    }
    for (final Object? component in deferredComponents) {
      if (component is! YamlMap) {
        _logger.printError('Expected deferred component manifest to be a map.');
        continue;
      }
      components.add(
        DeferredComponent(
          name: component['name'] as String,
          libraries:
              component['libraries'] == null
                  ? <String>[]
                  : (component['libraries'] as List<dynamic>).cast<String>(),
          assets: _computeAssets(component['assets']),
        ),
      );
    }
    return components;
  }

  /// Returns the iOS bundle identifier declared by this manifest in its
  /// module descriptor. Returns null if there is no such declaration.
  String? get iosBundleIdentifier {
    if (isModule) {
      if (_flutterDescriptor case {'module': final YamlMap map}) {
        return map['iosBundleIdentifier'] as String?;
      }
    }
    return null;
  }

  /// Gets the supported platforms. This only supports the new `platforms` format.
  ///
  /// If the plugin uses the legacy pubspec format, this method returns null.
  Map<String, Object?>? get supportedPlatforms {
    if (isPlugin) {
      final YamlMap? plugin = _flutterDescriptor['plugin'] as YamlMap?;
      if (plugin?.containsKey('platforms') ?? false) {
        final YamlMap? platformsMap = plugin!['platforms'] as YamlMap?;
        return platformsMap?.value.cast<String, Object?>();
      }
    }
    return null;
  }

  /// Like [supportedPlatforms], but only returns the valid platforms that are supported in flutter plugins.
  Map<String, Object?>? get validSupportedPlatforms {
    final Map<String, Object?>? allPlatforms = supportedPlatforms;
    if (allPlatforms == null) {
      return null;
    }
    final Map<String, Object?> platforms = <String, Object?>{}..addAll(allPlatforms);
    platforms.removeWhere((String key, Object? _) => !_kValidPluginPlatforms.contains(key));
    if (platforms.isEmpty) {
      return null;
    }
    return platforms;
  }

  List<Map<String, Object?>> get fontsDescriptor {
    return fonts.map((Font font) => font.descriptor).toList();
  }

  List<Map<String, Object?>> get _rawFontsDescriptor {
    final List<Object?>? fontList = _flutterDescriptor['fonts'] as List<Object?>?;
    return fontList == null
        ? const <Map<String, Object?>>[]
        : fontList
            .map<Map<String, Object?>?>(castStringKeyedMap)
            .whereType<Map<String, Object?>>()
            .toList();
  }

  late final List<AssetsEntry> assets = _computeAssets(_flutterDescriptor['assets']);

  late final List<Font> fonts = _extractFonts();

  List<Font> _extractFonts() {
    if (!_flutterDescriptor.containsKey('fonts')) {
      return <Font>[];
    }

    final List<Font> fonts = <Font>[];
    for (final Map<String, Object?> fontFamily in _rawFontsDescriptor) {
      final YamlList? fontFiles = fontFamily['fonts'] as YamlList?;
      final String? familyName = fontFamily['family'] as String?;
      if (familyName == null) {
        _logger.printWarning('Warning: Missing family name for font.', emphasis: true);
        continue;
      }
      if (fontFiles == null) {
        _logger.printWarning('Warning: No fonts specified for font $familyName', emphasis: true);
        continue;
      }

      final List<FontAsset> fontAssets = <FontAsset>[];
      for (final Map<Object?, Object?> fontFile in fontFiles.cast<Map<Object?, Object?>>()) {
        final String? asset = fontFile['asset'] as String?;
        if (asset == null) {
          _logger.printWarning('Warning: Missing asset in fonts for $familyName', emphasis: true);
          continue;
        }

        fontAssets.add(
          FontAsset(
            Uri.parse(asset),
            weight: fontFile['weight'] as int?,
            style: fontFile['style'] as String?,
          ),
        );
      }
      if (fontAssets.isNotEmpty) {
        fonts.add(Font(familyName, fontAssets));
      }
    }
    return fonts;
  }

  late final List<Uri> shaders = _extractAssetUris('shaders', 'Shader');
  late final List<Uri> models =
      kIs3dSceneSupported ? _extractAssetUris('models', 'Model') : <Uri>[];

  List<Uri> _extractAssetUris(String key, String singularName) {
    if (!_flutterDescriptor.containsKey(key)) {
      return <Uri>[];
    }

    final List<Object?>? items = _flutterDescriptor[key] as List<Object?>?;
    if (items == null) {
      return const <Uri>[];
    }
    final List<Uri> results = <Uri>[];
    for (final Object? item in items) {
      if (item is! String || item == '') {
        _logger.printError('$singularName manifest contains a null or empty uri.');
        continue;
      }
      try {
        results.add(Uri(pathSegments: item.split('/')));
      } on FormatException {
        _logger.printError('$singularName manifest contains invalid uri: $item.');
      }
    }
    return results;
  }

  /// Whether localization Dart files should be generated.
  ///
  /// **NOTE**: This method was previously called `generateSyntheticPackage`,
  /// which was incorrect; the presence of `generate: true` in `pubspec.yaml`
  /// does _not_ imply a synthetic package (and never did); additional
  /// introspection is required to determine whether a synthetic package is
  /// required.
  ///
  /// See also:
  ///
  ///   * [Deprecate and remove synthethic `package:flutter_gen`](https://github.com/flutter/flutter/issues/102983)
  ///   * [gen_l10n.generateLocalizations]
  late final bool generateLocalizations = _flutterDescriptor['generate'] == true;

  String? get defaultFlavor => _flutterDescriptor['default-flavor'] as String?;

  YamlMap toYaml() {
    return YamlMap.wrap(_descriptor);
  }
}

class Font {
  Font(this.familyName, this.fontAssets) : assert(fontAssets.isNotEmpty);

  final String familyName;
  final List<FontAsset> fontAssets;

  Map<String, Object?> get descriptor {
    return <String, Object?>{
      'family': familyName,
      'fonts': fontAssets.map<Map<String, Object?>>((FontAsset a) => a.descriptor).toList(),
    };
  }

  @override
  String toString() => '$runtimeType(family: $familyName, assets: $fontAssets)';
}

class FontAsset {
  FontAsset(this.assetUri, {this.weight, this.style});

  final Uri assetUri;
  final int? weight;
  final String? style;

  Map<String, Object?> get descriptor {
    final Map<String, Object?> descriptor = <String, Object?>{};
    if (weight != null) {
      descriptor['weight'] = weight;
    }

    if (style != null) {
      descriptor['style'] = style;
    }

    descriptor['asset'] = assetUri.path;
    return descriptor;
  }

  @override
  String toString() => '$runtimeType(asset: ${assetUri.path}, weight; $weight, style: $style)';
}

bool _validate(Object? manifest, Logger logger) {
  final List<String> errors = <String>[];
  if (manifest is! YamlMap) {
    errors.add('Expected YAML map');
  } else {
    for (final MapEntry<Object?, Object?> kvp in manifest.entries) {
      if (kvp.key is! String) {
        errors.add('Expected YAML key to be a string, but got ${kvp.key}.');
        continue;
      }
      switch (kvp.key as String?) {
        case 'name':
          if (kvp.value is! String) {
            errors.add('Expected "${kvp.key}" to be a string, but got ${kvp.value}.');
          }
        case 'flutter':
          if (kvp.value == null) {
            continue;
          }
          if (kvp.value is! YamlMap) {
            errors.add(
              'Expected "${kvp.key}" section to be an object or null, but got ${kvp.value}.',
            );
          } else {
            _validateFlutter(kvp.value as YamlMap?, errors);
          }
        default:
          // additionalProperties are allowed.
          break;
      }
    }
  }

  if (errors.isNotEmpty) {
    logger.printStatus('Error detected in pubspec.yaml:', emphasis: true);
    logger.printError(errors.join('\n'));
    return false;
  }

  return true;
}

void _validateFlutter(YamlMap? yaml, List<String> errors) {
  if (yaml == null) {
    return;
  }
  for (final MapEntry<Object?, Object?> kvp in yaml.entries) {
    final Object? yamlKey = kvp.key;
    final Object? yamlValue = kvp.value;
    if (yamlKey is! String) {
      errors.add('Expected YAML key to be a string, but got $yamlKey (${yamlValue.runtimeType}).');
      continue;
    }
    switch (yamlKey) {
      case 'uses-material-design':
        if (yamlValue is! bool) {
          errors.add(
            'Expected "$yamlKey" to be a bool, but got $yamlValue (${yamlValue.runtimeType}).',
          );
        }
      case 'assets':
        errors.addAll(_validateAssets(yamlValue));
      case 'shaders':
        if (yamlValue is! YamlList) {
          errors.add(
            'Expected "$yamlKey" to be a list, but got $yamlValue (${yamlValue.runtimeType}).',
          );
        } else if (yamlValue.isEmpty) {
          break;
        } else if (yamlValue[0] is! String) {
          errors.add(
            'Expected "$yamlKey" to be a list of strings, but the first element is $yamlValue (${yamlValue.runtimeType}).',
          );
        }
      case 'models':
        if (yamlValue is! YamlList) {
          errors.add(
            'Expected "$yamlKey" to be a list, but got $yamlValue (${yamlValue.runtimeType}).',
          );
        } else if (yamlValue.isEmpty) {
          break;
        } else if (yamlValue[0] is! String) {
          errors.add(
            'Expected "$yamlKey" to be a list of strings, but the first element is $yamlValue (${yamlValue.runtimeType}).',
          );
        }
      case 'fonts':
        if (yamlValue is! YamlList) {
          errors.add(
            'Expected "$yamlKey" to be a list, but got $yamlValue (${yamlValue.runtimeType}).',
          );
        } else if (yamlValue.isEmpty) {
          break;
        } else if (yamlValue.first is! YamlMap) {
          errors.add(
            'Expected "$yamlKey" to contain maps, but the first element is $yamlValue (${yamlValue.runtimeType}).',
          );
        } else {
          _validateFonts(yamlValue, errors);
        }
      case 'licenses':
        final (_, List<String> filesErrors) = _parseList<String>(yamlValue, '"$yamlKey"', 'files');
        errors.addAll(filesErrors);
      case 'module':
        if (yamlValue is! YamlMap) {
          errors.add(
            'Expected "$yamlKey" to be an object, but got $yamlValue (${yamlValue.runtimeType}).',
          );
          break;
        }

        if (yamlValue['androidX'] != null && yamlValue['androidX'] is! bool) {
          errors.add('The "androidX" value must be a bool if set.');
        }
        if (yamlValue['androidPackage'] != null && yamlValue['androidPackage'] is! String) {
          errors.add('The "androidPackage" value must be a string if set.');
        }
        if (yamlValue['iosBundleIdentifier'] != null &&
            yamlValue['iosBundleIdentifier'] is! String) {
          errors.add('The "iosBundleIdentifier" section must be a string if set.');
        }
      case 'plugin':
        if (yamlValue is! YamlMap) {
          errors.add(
            'Expected "$yamlKey" to be an object, but got $yamlValue (${yamlValue.runtimeType}).',
          );
          break;
        }
        final List<String> pluginErrors = Plugin.validatePluginYaml(yamlValue);
        errors.addAll(pluginErrors);
      case 'generate':
        break;
      case 'deferred-components':
        _validateDeferredComponents(kvp, errors);
      case 'disable-swift-package-manager':
        if (yamlValue is! bool) {
          errors.add(
            'Expected "$yamlKey" to be a bool, but got $yamlValue (${yamlValue.runtimeType}).',
          );
        }
      case 'default-flavor':
        if (yamlValue is! String) {
          errors.add(
            'Expected "$yamlKey" to be a string, but got $yamlValue (${yamlValue.runtimeType}).',
          );
        }
      default:
        errors.add('Unexpected child "$yamlKey" found under "flutter".');
    }
  }
}

(List<T>? result, List<String> errors) _parseList<T>(
  Object? yamlList,
  String context,
  String typeAlias,
) {
  final List<String> errors = <String>[];

  if (yamlList is! YamlList) {
    final String message =
        'Expected $context to be a list of $typeAlias, but got $yamlList (${yamlList.runtimeType}).';
    return (null, <String>[message]);
  }

  for (int i = 0; i < yamlList.length; i++) {
    if (yamlList[i] is! T) {
      // ignore: avoid_dynamic_calls
      errors.add(
        'Expected $context to be a list of $typeAlias, but element at index $i was a ${yamlList[i].runtimeType}.',
      );
    }
  }

  return errors.isEmpty ? (List<T>.from(yamlList), errors) : (null, errors);
}

void _validateDeferredComponents(MapEntry<Object?, Object?> kvp, List<String> errors) {
  final Object? yamlList = kvp.value;
  if (yamlList != null && (yamlList is! YamlList || yamlList[0] is! YamlMap)) {
    errors.add('Expected "${kvp.key}" to be a list, but got $yamlList (${yamlList.runtimeType}).');
  } else if (yamlList is YamlList) {
    for (int i = 0; i < yamlList.length; i++) {
      final Object? valueMap = yamlList[i];
      if (valueMap is! YamlMap) {
        // ignore: avoid_dynamic_calls
        errors.add(
          'Expected the $i element in "${kvp.key}" to be a map, but got ${yamlList[i]} (${yamlList[i].runtimeType}).',
        );
        continue;
      }
      if (!valueMap.containsKey('name') || valueMap['name'] is! String) {
        errors.add(
          'Expected the $i element in "${kvp.key}" to have required key "name" of type String',
        );
      }
      if (valueMap.containsKey('libraries')) {
        final (_, List<String> librariesErrors) = _parseList<String>(
          valueMap['libraries'],
          '"libraries" key in the element at index $i of "${kvp.key}"',
          'String',
        );
        errors.addAll(librariesErrors);
      }
      if (valueMap.containsKey('assets')) {
        errors.addAll(_validateAssets(valueMap['assets']));
      }
    }
  }
}

List<String> _validateAssets(Object? yaml) {
  final (_, List<String> errors) = _computeAssetsSafe(yaml);
  return errors;
}

// TODO(andrewkolos): We end up parsing the assets section twice, once during
// validation and once when the assets getter is called. We should consider
// refactoring this class to parse and store everything in the constructor.
// https://github.com/flutter/flutter/issues/139183
(List<AssetsEntry>, List<String> errors) _computeAssetsSafe(Object? yaml) {
  if (yaml == null) {
    return (const <AssetsEntry>[], const <String>[]);
  }
  if (yaml is! YamlList) {
    final String error = 'Expected "assets" to be a list, but got $yaml (${yaml.runtimeType}).';
    return (const <AssetsEntry>[], <String>[error]);
  }
  final List<AssetsEntry> results = <AssetsEntry>[];
  final List<String> errors = <String>[];
  for (final Object? rawAssetEntry in yaml) {
    final (AssetsEntry? parsed, String? error) = AssetsEntry.parseFromYamlSafe(rawAssetEntry);
    if (parsed != null) {
      results.add(parsed);
    }
    if (error != null) {
      errors.add(error);
    }
  }
  return (results, errors);
}

List<AssetsEntry> _computeAssets(Object? assetsSection) {
  final (List<AssetsEntry> result, List<String> errors) = _computeAssetsSafe(assetsSection);
  if (errors.isNotEmpty) {
    throw Exception(
      'Uncaught error(s) in assets section: '
      '${errors.join('\n')}',
    );
  }
  return result;
}

void _validateFonts(YamlList fonts, List<String> errors) {
  const Set<int> fontWeights = <int>{100, 200, 300, 400, 500, 600, 700, 800, 900};
  for (final Object? fontMap in fonts) {
    if (fontMap is! YamlMap) {
      errors.add('Unexpected child "$fontMap" found under "fonts". Expected a map.');
      continue;
    }
    for (final Object? key in fontMap.keys.where(
      (Object? key) => key != 'family' && key != 'fonts',
    )) {
      errors.add('Unexpected child "$key" found under "fonts".');
    }
    if (fontMap['family'] != null && fontMap['family'] is! String) {
      errors.add('Font family must either be null or a String.');
    }
    if (fontMap['fonts'] == null) {
      continue;
    } else if (fontMap['fonts'] is! YamlList) {
      errors.add('Expected "fonts" to either be null or a list.');
      continue;
    }
    for (final Object? fontMapList in fontMap['fonts'] as List<Object?>) {
      if (fontMapList is! YamlMap) {
        errors.add('Expected "fonts" to be a list of maps.');
        continue;
      }
      for (final MapEntry<Object?, Object?> kvp in fontMapList.entries) {
        final Object? fontKey = kvp.key;
        if (fontKey is! String) {
          errors.add('Expected "$fontKey" under "fonts" to be a string.');
        }
        switch (fontKey) {
          case 'asset':
            if (kvp.value is! String) {
              errors.add(
                'Expected font asset ${kvp.value} ((${kvp.value.runtimeType})) to be a string.',
              );
            }
          case 'weight':
            if (!fontWeights.contains(kvp.value)) {
              errors.add(
                'Invalid value ${kvp.value} ((${kvp.value.runtimeType})) for font -> weight.',
              );
            }
          case 'style':
            if (kvp.value != 'normal' && kvp.value != 'italic') {
              errors.add(
                'Invalid value ${kvp.value} ((${kvp.value.runtimeType})) for font -> style.',
              );
            }
          default:
            errors.add('Unexpected key $fontKey ((${kvp.value.runtimeType})) under font.');
        }
      }
    }
  }
}

/// Represents an entry under the `assets` section of a pubspec.
@immutable
class AssetsEntry {
  const AssetsEntry({
    required this.uri,
    this.flavors = const <String>{},
    this.transformers = const <AssetTransformerEntry>[],
  });

  final Uri uri;
  final Set<String> flavors;
  final List<AssetTransformerEntry> transformers;

  Object? get descriptor {
    if (transformers.isEmpty && flavors.isEmpty) {
      return uri.toString();
    }
    return <String, Object?>{
      _pathKey: uri.toString(),
      if (flavors.isNotEmpty) _flavorKey: flavors.toList(),
      if (transformers.isNotEmpty)
        _transformersKey: transformers.map((AssetTransformerEntry e) => e.descriptor).toList(),
    };
  }

  static const String _pathKey = 'path';
  static const String _flavorKey = 'flavors';
  static const String _transformersKey = 'transformers';

  static AssetsEntry? parseFromYaml(Object? yaml) {
    final (AssetsEntry? value, String? error) = parseFromYamlSafe(yaml);
    if (error != null) {
      throw Exception('Unexpected error when parsing assets entry');
    }
    return value!;
  }

  static (AssetsEntry? assetsEntry, String? error) parseFromYamlSafe(Object? yaml) {
    (Uri?, String?) tryParseUri(String uri) {
      try {
        return (Uri(pathSegments: uri.split('/')), null);
      } on FormatException {
        return (null, 'Asset manifest contains invalid uri: $uri.');
      }
    }

    if (yaml == null || yaml == '') {
      return (null, 'Asset manifest contains a null or empty uri.');
    }

    if (yaml is String) {
      final (Uri? uri, String? error) = tryParseUri(yaml);
      return uri == null ? (null, error) : (AssetsEntry(uri: uri), null);
    }

    if (yaml is Map) {
      if (yaml.keys.isEmpty) {
        return (null, null);
      }

      final Object? path = yaml[_pathKey];

      if (path == null || path is! String) {
        return (
          null,
          'Asset manifest entry is malformed. '
              'Expected asset entry to be either a string or a map '
              'containing a "$_pathKey" entry. Got ${path.runtimeType} instead.',
        );
      }

      final (List<String>? flavors, List<String> flavorsErrors) = _parseFlavorsSection(
        yaml[_flavorKey],
      );
      final (
        List<AssetTransformerEntry>? transformers,
        List<String> transformersErrors,
      ) = _parseTransformersSection(yaml[_transformersKey]);

      final List<String> errors = <String>[
        ...flavorsErrors.map((String e) => 'In $_flavorKey section of asset "$path": $e'),
        ...transformersErrors.map(
          (String e) => 'In $_transformersKey section of asset "$path": $e',
        ),
      ];
      if (errors.isNotEmpty) {
        return (null, <String>['Unable to parse assets section.', ...errors].join('\n'));
      }

      return (
        AssetsEntry(
          uri: Uri(pathSegments: path.split('/')),
          flavors: Set<String>.from(flavors ?? <String>[]),
          transformers: transformers ?? <AssetTransformerEntry>[],
        ),
        null,
      );
    }

    return (
      null,
      'Assets entry had unexpected shape. '
          'Expected a string or an object. Got ${yaml.runtimeType} instead.',
    );
  }

  static (List<String>? flavors, List<String> errors) _parseFlavorsSection(Object? yaml) {
    if (yaml == null) {
      return (null, <String>[]);
    }

    return _parseList<String>(yaml, _flavorKey, 'String');
  }

  static (List<AssetTransformerEntry>?, List<String> errors) _parseTransformersSection(
    Object? yaml,
  ) {
    if (yaml == null) {
      return (null, <String>[]);
    }
    final (List<YamlMap>? yamlObjects, List<String> listErrors) = _parseList<YamlMap>(
      yaml,
      '$_transformersKey list',
      'Map',
    );

    if (listErrors.isNotEmpty) {
      return (null, listErrors);
    }

    final List<AssetTransformerEntry> transformers = <AssetTransformerEntry>[];
    final List<String> errors = <String>[];
    for (final YamlMap yaml in yamlObjects!) {
      final (
        AssetTransformerEntry? transformerEntry,
        List<String> transformerErrors,
      ) = AssetTransformerEntry.tryParse(yaml);
      if (transformerEntry != null) {
        transformers.add(transformerEntry);
      } else {
        errors.addAll(transformerErrors);
      }
    }

    if (errors.isEmpty) {
      return (transformers, errors);
    }
    return (null, errors);
  }

  @override
  bool operator ==(Object other) {
    if (other is! AssetsEntry) {
      return false;
    }

    return uri == other.uri && setEquals(flavors, other.flavors);
  }

  @override
  int get hashCode => Object.hashAll(<Object?>[
    uri.hashCode,
    Object.hashAllUnordered(flavors),
    Object.hashAll(transformers),
  ]);

  @override
  String toString() => 'AssetsEntry(uri: $uri, flavors: $flavors, transformers: $transformers)';
}

/// Represents an entry in the "transformers" section of an asset.
@immutable
final class AssetTransformerEntry {
  const AssetTransformerEntry({required this.package, required List<String>? args})
    : args = args ?? const <String>[];

  final String package;
  final List<String>? args;

  Map<String, Object?> get descriptor {
    return <String, Object?>{_kPackage: package, if (args != null) _kArgs: args};
  }

  static const String _kPackage = 'package';
  static const String _kArgs = 'args';

  static (AssetTransformerEntry? entry, List<String> errors) tryParse(Object? yaml) {
    if (yaml == null) {
      return (null, <String>['Transformer entry is null.']);
    }
    if (yaml is! YamlMap) {
      return (null, <String>['Expected entry to be a map. Found ${yaml.runtimeType} instead']);
    }

    final Object? package = yaml['package'];
    if (package is! String || package.isEmpty) {
      return (
        null,
        <String>['Expected "package" to be a String. Found ${package.runtimeType} instead.'],
      );
    }

    final (List<String>? args, List<String> argsErrors) = _parseArgsSection(yaml['args']);
    if (argsErrors.isNotEmpty) {
      return (
        null,
        argsErrors
            .map((String e) => 'In args section of transformer using package "$package": $e')
            .toList(),
      );
    }

    return (AssetTransformerEntry(package: package, args: args), <String>[]);
  }

  static (List<String>? args, List<String> errors) _parseArgsSection(Object? yaml) {
    if (yaml == null) {
      return (null, <String>[]);
    }
    return _parseList(yaml, 'args', 'String');
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other is! AssetTransformerEntry) {
      return false;
    }

    final bool argsAreEqual =
        (() {
          if (args == null && other.args == null) {
            return true;
          }
          if (args?.length != other.args?.length) {
            return false;
          }

          for (int index = 0; index < args!.length; index += 1) {
            if (args![index] != other.args![index]) {
              return false;
            }
          }
          return true;
        })();

    return package == other.package && argsAreEqual;
  }

  @override
  int get hashCode =>
      Object.hashAll(<Object?>[package.hashCode, args?.map((String e) => e.hashCode)]);

  @override
  String toString() {
    return 'AssetTransformerEntry(package: $package, args: $args)';
  }
}
