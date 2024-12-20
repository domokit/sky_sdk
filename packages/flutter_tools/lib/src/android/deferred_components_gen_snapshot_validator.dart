// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:xml/xml.dart';
import 'package:yaml/yaml.dart';

import '../base/deferred_component.dart';
import '../base/error_handling_io.dart';
import '../base/file_system.dart';
import '../build_system/build_system.dart';
import 'deferred_components_validator.dart';

/// A class to configure and run deferred component setup verification checks
/// and tasks.
///
/// Once constructed, checks and tasks can be executed by calling the respective
/// methods. The results of the checks are stored internally and can be
/// displayed to the user by calling [displayResults].
class DeferredComponentsGenSnapshotValidator extends DeferredComponentsValidator {
  /// Constructs a validator instance.
  ///
  /// The [env] property is used to locate the project files that are checked.
  ///
  /// The [templatesDir] parameter is optional. If null, the tool's default
  /// templates directory will be used.
  ///
  /// When [exitOnFail] is set to true, the [handleResults] and [attemptToolExit]
  /// methods will exit the tool when this validator detects a recommended
  /// change. This defaults to true.
  DeferredComponentsGenSnapshotValidator(this.env, {bool exitOnFail = true, String? title})
    : super(env.projectDir, env.logger, env.platform, exitOnFail: exitOnFail, title: title);

  /// The build environment that should be used to find the input files to run
  /// checks against.
  ///
  /// The checks in this class are meant to be used as part of a build process,
  /// so an environment should be available.
  final Environment env;

  // The key used to identify the metadata element as the loading unit id to
  // deferred component mapping.
  static const String _mappingKey =
      'io.flutter.embedding.engine.deferredcomponents.DeferredComponentManager.loadingUnitMapping';

  /// Checks if the base module `app`'s `AndroidManifest.xml` contains the
  /// required meta-data that maps loading units to deferred components.
  ///
  /// Returns true if the check passed with no recommended changes, and false
  /// otherwise.
  ///
  /// Flutter engine uses a manifest meta-data mapping to determine which
  /// deferred component includes a particular loading unit id. This method
  /// checks if `app`'s `AndroidManifest.xml` contains this metadata. If not, it
  /// will generate a modified AndroidManifest.xml with the correct metadata
  /// entry.
  ///
  /// An example mapping:
  ///
  ///   2:componentA,3:componentB,4:componentC
  ///
  /// Where loading unit 2 is included in componentA, loading unit 3 is included
  /// in componentB, and loading unit 4 is included in componentC.
  bool checkAppAndroidManifestComponentLoadingUnitMapping(
    List<DeferredComponent> components,
    List<LoadingUnit> generatedLoadingUnits,
  ) {
    final Directory androidDir = projectDir.childDirectory('android');
    inputs.add(projectDir.childFile('pubspec.yaml'));

    // We do not use the Xml package to handle the writing, as we do not want to
    // erase any user applied formatting and comments. The changes can be
    // applied with dart io and custom parsing.
    final File appManifestFile = androidDir
        .childDirectory('app')
        .childDirectory('src')
        .childDirectory('main')
        .childFile('AndroidManifest.xml');
    inputs.add(appManifestFile);
    if (!appManifestFile.existsSync()) {
      invalidFiles[appManifestFile.path] =
          'Error: $appManifestFile does not '
          'exist or could not be found. Please ensure an AndroidManifest.xml '
          "exists for the app's base module.";
      return false;
    }
    XmlDocument document;
    try {
      document = XmlDocument.parse(appManifestFile.readAsStringSync());
    } on XmlException {
      invalidFiles[appManifestFile.path] =
          'Error parsing $appManifestFile '
          'Please ensure that the android manifest is a valid XML document and '
          'try again.';
      return false;
    } on FileSystemException {
      invalidFiles[appManifestFile.path] =
          'Error reading $appManifestFile '
          'even though it exists. Please ensure that you have read permission for '
          'this file and try again.';
      return false;
    }
    // Create loading unit mapping.
    final Map<int, String> mapping = <int, String>{};
    for (final DeferredComponent component in components) {
      component.assignLoadingUnits(generatedLoadingUnits);
      final Set<LoadingUnit>? loadingUnits = component.loadingUnits;
      if (loadingUnits == null) {
        continue;
      }
      for (final LoadingUnit unit in loadingUnits) {
        if (!mapping.containsKey(unit.id)) {
          mapping[unit.id] = component.name;
        }
      }
    }
    for (final LoadingUnit unit in generatedLoadingUnits) {
      if (!mapping.containsKey(unit.id)) {
        // Store an empty string for unassigned loading units,
        // indicating that it is in the base component.
        mapping[unit.id] = '';
      }
    }
    // Encode the mapping as a string.
    final StringBuffer mappingBuffer = StringBuffer();
    for (final int key in mapping.keys) {
      mappingBuffer.write('$key:${mapping[key]},');
    }
    String encodedMapping = mappingBuffer.toString();
    // remove trailing comma if any
    if (encodedMapping.endsWith(',')) {
      encodedMapping = encodedMapping.substring(0, encodedMapping.length - 1);
    }
    // Check for existing metadata entry and see if needs changes.
    bool exists = false;
    bool modified = false;
    for (final XmlElement application in document.findAllElements('application')) {
      for (final XmlElement metaData in application.findElements('meta-data')) {
        final String? name = metaData.getAttribute('android:name');
        if (name == _mappingKey) {
          exists = true;
          final String? storedMappingString = metaData.getAttribute('android:value');
          if (storedMappingString != encodedMapping) {
            metaData.setAttribute('android:value', encodedMapping);
            modified = true;
          }
        }
      }
    }
    if (!exists) {
      // Create an meta-data XmlElement that contains the mapping.
      final XmlElement mappingMetadataElement =
          XmlElement(XmlName.fromString('meta-data'), <XmlAttribute>[
            XmlAttribute(XmlName.fromString('android:name'), _mappingKey),
            XmlAttribute(XmlName.fromString('android:value'), encodedMapping),
          ]);
      for (final XmlElement application in document.findAllElements('application')) {
        application.children.add(mappingMetadataElement);
        break;
      }
    }
    if (!exists || modified) {
      final File manifestOutput = outputDir
          .childDirectory('app')
          .childDirectory('src')
          .childDirectory('main')
          .childFile('AndroidManifest.xml');
      ErrorHandlingFileSystem.deleteIfExists(manifestOutput);
      manifestOutput.createSync(recursive: true);
      manifestOutput.writeAsStringSync(document.toXmlString(pretty: true), flush: true);
      modifiedFiles.add(manifestOutput.path);
      return false;
    }
    return true;
  }

  /// Compares the provided loading units against the contents of the
  /// `deferred_components_loading_units.yaml` file.
  ///
  /// Returns true if a loading unit cache file exists and all loading units
  /// match, and false otherwise.
  ///
  /// This method will parse the cached loading units file if it exists and
  /// compare it to the provided generatedLoadingUnits. It will distinguish
  /// between newly added loading units and no longer existing loading units. If
  /// the cache file does not exist, then all generatedLoadingUnits will be
  /// considered new.
  bool checkAgainstLoadingUnitsCache(List<LoadingUnit> generatedLoadingUnits) {
    final List<LoadingUnit> cachedLoadingUnits = _parseLoadingUnitsCache(
      projectDir.childFile(DeferredComponentsValidator.kLoadingUnitsCacheFileName),
    );
    loadingUnitComparisonResults = <String, Object>{};
    final Set<LoadingUnit> unmatchedLoadingUnits = <LoadingUnit>{};
    final List<LoadingUnit> newLoadingUnits = <LoadingUnit>[];
    unmatchedLoadingUnits.addAll(cachedLoadingUnits);
    final Set<int> addedNewIds = <int>{};
    for (final LoadingUnit genUnit in generatedLoadingUnits) {
      bool matched = false;
      for (final LoadingUnit cacheUnit in cachedLoadingUnits) {
        if (genUnit.equalsIgnoringPath(cacheUnit)) {
          matched = true;
          unmatchedLoadingUnits.remove(cacheUnit);
          break;
        }
      }
      if (!matched && !addedNewIds.contains(genUnit.id)) {
        newLoadingUnits.add(genUnit);
        addedNewIds.add(genUnit.id);
      }
    }
    loadingUnitComparisonResults!['new'] = newLoadingUnits;
    loadingUnitComparisonResults!['missing'] = unmatchedLoadingUnits;
    loadingUnitComparisonResults!['match'] =
        newLoadingUnits.isEmpty && unmatchedLoadingUnits.isEmpty;
    return loadingUnitComparisonResults!['match']! as bool;
  }

  List<LoadingUnit> _parseLoadingUnitsCache(File cacheFile) {
    final List<LoadingUnit> loadingUnits = <LoadingUnit>[];
    inputs.add(cacheFile);
    if (!cacheFile.existsSync()) {
      return loadingUnits;
    }
    final YamlMap data = loadYaml(cacheFile.readAsStringSync()) as YamlMap;
    // validate yaml format.
    if (!data.containsKey('loading-units')) {
      invalidFiles[cacheFile.path] =
          "Invalid loading units yaml file, 'loading-units' "
          'entry did not exist.';
      return loadingUnits;
    } else {
      if (data['loading-units'] is! YamlList && data['loading-units'] != null) {
        invalidFiles[cacheFile.path] =
            "Invalid loading units yaml file, 'loading-units' "
            'is not a list.';
        return loadingUnits;
      }
      if (data['loading-units'] != null) {
        for (final Object? loadingUnitData in data['loading-units'] as List<Object?>) {
          if (loadingUnitData is! YamlMap) {
            invalidFiles[cacheFile.path] =
                "Invalid loading units yaml file, 'loading-units' "
                'is not a list of maps.';
            return loadingUnits;
          }
          final YamlMap loadingUnitDataMap = loadingUnitData;
          if (loadingUnitDataMap['id'] == null) {
            invalidFiles[cacheFile.path] =
                'Invalid loading units yaml file, all '
                "loading units must have an 'id'";
            return loadingUnits;
          }
          if (loadingUnitDataMap['libraries'] != null) {
            if (loadingUnitDataMap['libraries'] is! YamlList) {
              invalidFiles[cacheFile.path] =
                  "Invalid loading units yaml file, 'libraries' "
                  'is not a list.';
              return loadingUnits;
            }
            for (final Object? node in loadingUnitDataMap['libraries'] as YamlList) {
              if (node is! String) {
                invalidFiles[cacheFile.path] =
                    "Invalid loading units yaml file, 'libraries' "
                    'is not a list of strings.';
                return loadingUnits;
              }
            }
          }
        }
      }
    }

    // Parse out validated yaml.
    if (data.containsKey('loading-units')) {
      if (data['loading-units'] != null) {
        for (final Object? loadingUnitData in data['loading-units'] as List<Object?>) {
          final YamlMap? loadingUnitDataMap = loadingUnitData as YamlMap?;
          final List<String> libraries = <String>[];
          final YamlList? nodes = loadingUnitDataMap?['libraries'] as YamlList?;
          if (nodes != null) {
            for (final Object node in nodes.whereType<Object>()) {
              libraries.add(node as String);
            }
          }
          loadingUnits.add(LoadingUnit(id: loadingUnitDataMap!['id'] as int, libraries: libraries));
        }
      }
    }
    return loadingUnits;
  }

  /// Writes the provided generatedLoadingUnits as `deferred_components_loading_units.yaml`
  ///
  /// This cache file is used to detect any changes in the loading units
  /// produced by gen_snapshot. Running [checkAgainstLoadingUnitCache] with a
  /// mismatching or missing cache will result in a failed validation. This
  /// prevents unexpected changes in loading units causing misconfigured
  /// deferred components.
  void writeLoadingUnitsCache(List<LoadingUnit>? generatedLoadingUnits) {
    generatedLoadingUnits ??= <LoadingUnit>[];
    final File cacheFile = projectDir.childFile(
      DeferredComponentsValidator.kLoadingUnitsCacheFileName,
    );
    outputs.add(cacheFile);
    ErrorHandlingFileSystem.deleteIfExists(cacheFile);
    cacheFile.createSync(recursive: true);

    final StringBuffer buffer = StringBuffer();
    buffer.write('''
# ==============================================================================
# The contents of this file are automatically generated and it is not
# recommended to modify this file manually.
# ==============================================================================
#
# In order to prevent unexpected splitting of deferred apps, this file records
# the last generated set of loading units. It only possible to obtain the final
# configuration of loading units after compilation is complete. This means
# improperly setup deferred imports can only be detected after compilation.
#
# This file allows the build tool to detect any changes in the generated
# loading units. During the next build attempt, loading units in this file are
# compared against the newly generated loading units to check for any new or
# removed loading units. In the case where loading units do not match, the build
# will fail and ask the developer to verify that the `deferred-components`
# configuration in `pubspec.yaml` is correct. Developers should make any
# necessary changes to integrate new and changed loading units or remove no
# longer existing loading units from the configuration. The build command should
# then be re-run to continue the build process.
#
# Sometimes, changes to the generated loading units may be unintentional. If
# the list of loading units in this file is not what is expected, the app's
# deferred imports should be reviewed. Third party plugins and packages may
# also introduce deferred imports that result in unexpected loading units.
loading-units:
''');
    final Set<int> usedIds = <int>{};
    for (final LoadingUnit unit in generatedLoadingUnits) {
      if (usedIds.contains(unit.id)) {
        continue;
      }
      buffer.write('  - id: ${unit.id}\n');
      if (unit.libraries.isNotEmpty) {
        buffer.write('    libraries:\n');
        for (final String lib in unit.libraries) {
          buffer.write('      - $lib\n');
        }
      }
      usedIds.add(unit.id);
    }
    cacheFile.writeAsStringSync(buffer.toString(), flush: true);
  }
}
