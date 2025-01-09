// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:xml/xml.dart';

import '../base/deferred_component.dart';
import '../base/error_handling_io.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import '../globals.dart' as globals;
import '../project.dart';
import '../template.dart';
import 'deferred_components_validator.dart';

/// A class to configure and run deferred component setup verification checks
/// and tasks.
///
/// Once constructed, checks and tasks can be executed by calling the respective
/// methods. The results of the checks are stored internally and can be
/// displayed to the user by calling [displayResults].
class DeferredComponentsPrebuildValidator extends DeferredComponentsValidator {
  /// Constructs a validator instance.
  ///
  /// The [templatesDir] parameter is optional. If null, the tool's default
  /// templates directory will be used.
  ///
  /// When [exitOnFail] is set to true, the [handleResults] and [attemptToolExit]
  /// methods will exit the tool when this validator detects a recommended
  /// change. This defaults to true.
  DeferredComponentsPrebuildValidator(
    super.projectDir,
    super.logger,
    super.platform, {
    super.exitOnFail,
    super.title,
    Directory? templatesDir,
  }) : _templatesDir = templatesDir;

  final Directory? _templatesDir;

  /// Checks if an android dynamic feature module exists for each deferred
  /// component.
  ///
  /// Returns true if the check passed with no recommended changes, and false
  /// otherwise.
  ///
  /// This method looks for the existence of `android/<componentname>/build.gradle`
  /// and `android/<componentname>/src/main/AndroidManifest.xml`. If either of
  /// these files does not exist, it will generate it in the validator output
  /// directory based off of a template.
  ///
  /// This method does not check if the contents of either of the files are
  /// valid, as there are many ways that they can be validly configured.
  Future<bool> checkAndroidDynamicFeature(List<DeferredComponent> components) async {
    inputs.add(projectDir.childFile('pubspec.yaml'));
    if (components.isEmpty) {
      return false;
    }
    bool changesMade = false;
    for (final DeferredComponent component in components) {
      final _DeferredComponentAndroidFiles androidFiles = _DeferredComponentAndroidFiles(
        name: component.name,
        projectDir: projectDir,
        logger: logger,
        templatesDir: _templatesDir,
      );
      if (!androidFiles.verifyFilesExist()) {
        // generate into temp directory
        final Map<String, List<File>> results = await androidFiles.generateFiles(
          alternateAndroidDir: outputDir,
          clearAlternateOutputDir: true,
        );
        if (results.containsKey('outputs')) {
          for (final File file in results['outputs']!) {
            generatedFiles.add(file.path);
            changesMade = true;
          }
          outputs.addAll(results['outputs']!);
        }
        if (results.containsKey('inputs')) {
          inputs.addAll(results['inputs']!);
        }
      }
    }
    return !changesMade;
  }

  /// Checks if the base module `app`'s `strings.xml` contain string
  /// resources for each component's name.
  ///
  /// Returns true if the check passed with no recommended changes, and false
  /// otherwise.
  ///
  /// In each dynamic feature module's AndroidManifest.xml, the
  /// name of the module is a string resource. This checks if
  /// the needed string resources are in the base module `strings.xml`.
  /// If not, this method will generate a modified `strings.xml` (or a
  /// completely new one if the original file did not exist) in the
  /// validator's output directory.
  ///
  /// For example, if there is a deferred component named `component1`,
  /// there should be the following string resource:
  ///
  ///   <string name="component1Name">component1</string>
  ///
  /// The string element's name attribute should be the component name with
  /// `Name` as a suffix, and the text contents should be the component name.
  bool checkAndroidResourcesStrings(List<DeferredComponent> components) {
    final Directory androidDir = projectDir.childDirectory('android');
    inputs.add(projectDir.childFile('pubspec.yaml'));

    // Add component name mapping to strings.xml
    final File stringRes = androidDir
        .childDirectory('app')
        .childDirectory('src')
        .childDirectory('main')
        .childDirectory('res')
        .childDirectory('values')
        .childFile('strings.xml');
    inputs.add(stringRes);
    final File stringResOutput = outputDir
        .childDirectory('app')
        .childDirectory('src')
        .childDirectory('main')
        .childDirectory('res')
        .childDirectory('values')
        .childFile('strings.xml');
    ErrorHandlingFileSystem.deleteIfExists(stringResOutput);
    if (components.isEmpty) {
      return true;
    }
    final Map<String, String> requiredEntriesMap = <String, String>{};
    for (final DeferredComponent component in components) {
      requiredEntriesMap['${component.name}Name'] = component.name;
    }
    if (stringRes.existsSync()) {
      bool modified = false;
      XmlDocument document;
      try {
        document = XmlDocument.parse(stringRes.readAsStringSync());
      } on XmlException {
        invalidFiles[stringRes.path] =
            'Error parsing $stringRes '
            'Please ensure that the strings.xml is a valid XML document and '
            'try again.';
        return false;
      }
      // Check if all required lines are present, and fix if name exists, but
      // wrong string stored.
      for (final XmlElement resources in document.findAllElements('resources')) {
        for (final XmlElement element in resources.findElements('string')) {
          final String? name = element.getAttribute('name');
          if (requiredEntriesMap.containsKey(name)) {
            if (element.innerText != requiredEntriesMap[name]) {
              element.innerText = requiredEntriesMap[name]!;
              modified = true;
            }
            requiredEntriesMap.remove(name);
          }
        }
        requiredEntriesMap.forEach((String key, String value) {
          modified = true;
          final XmlElement newStringElement = XmlElement(
            XmlName.fromString('string'),
            <XmlAttribute>[XmlAttribute(XmlName.fromString('name'), key)],
            <XmlNode>[XmlText(value)],
          );
          resources.children.add(newStringElement);
        });
        break;
      }
      if (modified) {
        stringResOutput.createSync(recursive: true);
        stringResOutput.writeAsStringSync(document.toXmlString(pretty: true));
        modifiedFiles.add(stringResOutput.path);
        return false;
      }
      return true;
    }
    // strings.xml does not exist, generate completely new file.
    stringResOutput.createSync(recursive: true);
    final StringBuffer buffer = StringBuffer();
    buffer.writeln('''
<?xml version="1.0" encoding="utf-8"?>
<resources>
''');
    for (final String key in requiredEntriesMap.keys) {
      buffer.write('    <string name="$key">${requiredEntriesMap[key]}</string>\n');
    }
    buffer.write('''
</resources>

''');
    stringResOutput.writeAsStringSync(buffer.toString(), flush: true, mode: FileMode.append);
    generatedFiles.add(stringResOutput.path);
    return false;
  }

  /// Deletes all files inside of the validator's output directory.
  void clearOutputDir() {
    final Directory dir = projectDir
        .childDirectory('build')
        .childDirectory(DeferredComponentsValidator.kDeferredComponentsTempDirectory);
    ErrorHandlingFileSystem.deleteIfExists(dir, recursive: true);
  }
}

// Handles a single deferred component's android dynamic feature module
// directory.
class _DeferredComponentAndroidFiles {
  _DeferredComponentAndroidFiles({
    required this.name,
    required this.projectDir,
    required this.logger,
    Directory? templatesDir,
  }) : _templatesDir = templatesDir;

  // The name of the deferred component.
  final String name;
  final Directory projectDir;
  final Logger logger;
  final Directory? _templatesDir;

  Directory get androidDir => projectDir.childDirectory('android');
  Directory get componentDir => androidDir.childDirectory(name);

  File get androidManifestFile =>
      componentDir.childDirectory('src').childDirectory('main').childFile('AndroidManifest.xml');
  File get buildGradleFile {
    if (componentDir.childFile('build.gradle').existsSync()) {
      return componentDir.childFile('build.gradle');
    }
    return componentDir.childFile('build.gradle.kts');
  }

  // True when AndroidManifest.xml and build.gradle/build.gradle.kts exist for
  // the android dynamic feature.
  bool verifyFilesExist() {
    return androidManifestFile.existsSync() && buildGradleFile.existsSync();
  }

  // Generates any missing basic files for the dynamic feature into a temporary directory.
  Future<Map<String, List<File>>> generateFiles({
    Directory? alternateAndroidDir,
    bool clearAlternateOutputDir = false,
  }) async {
    final Directory outputDir = alternateAndroidDir?.childDirectory(name) ?? componentDir;
    if (clearAlternateOutputDir && alternateAndroidDir != null) {
      ErrorHandlingFileSystem.deleteIfExists(outputDir);
    }
    final List<File> inputs = <File>[];
    inputs.add(androidManifestFile);
    inputs.add(buildGradleFile);
    final Map<String, List<File>> results = <String, List<File>>{'inputs': inputs};
    results['outputs'] = await _setupComponentFiles(outputDir);
    return results;
  }

  // generates default build.gradle and AndroidManifest.xml for the deferred component.
  Future<List<File>> _setupComponentFiles(Directory outputDir) async {
    Template template;
    final Directory? templatesDir = _templatesDir;
    if (templatesDir != null) {
      final Directory templateComponentDir = templatesDir.childDirectory(
        'module${globals.fs.path.separator}android${globals.fs.path.separator}deferred_component',
      );
      template = Template(
        templateComponentDir,
        templateComponentDir,
        fileSystem: globals.fs,
        logger: logger,
        templateRenderer: globals.templateRenderer,
      );
    } else {
      template = await Template.fromName(
        'module${globals.fs.path.separator}android${globals.fs.path.separator}deferred_component',
        fileSystem: globals.fs,
        templateManifest: null,
        logger: logger,
        templateRenderer: globals.templateRenderer,
      );
    }
    final Map<String, Object> context = <String, Object>{
      'androidIdentifier':
          FlutterProject.current().manifest.androidPackage ??
          'com.example.${FlutterProject.current().manifest.appName}',
      'componentName': name,
    };

    template.render(outputDir, context);

    final List<File> generatedFiles = <File>[];

    final File tempBuildGradle = outputDir.childFile('build.gradle');
    if (!buildGradleFile.existsSync()) {
      generatedFiles.add(tempBuildGradle);
    } else {
      ErrorHandlingFileSystem.deleteIfExists(tempBuildGradle);
    }
    final File tempAndroidManifest = outputDir
        .childDirectory('src')
        .childDirectory('main')
        .childFile('AndroidManifest.xml');
    if (!androidManifestFile.existsSync()) {
      generatedFiles.add(tempAndroidManifest);
    } else {
      ErrorHandlingFileSystem.deleteIfExists(tempAndroidManifest);
    }
    return generatedFiles;
  }
}
