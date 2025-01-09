// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:collection';

import 'package:meta/meta.dart';
import 'package:process/process.dart';
import 'package:xml/xml.dart';

import '../application_package.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/process.dart';
import '../base/user_messages.dart';
import '../build_info.dart';
import '../project.dart';
import 'android_sdk.dart';
import 'gradle.dart';

/// An application package created from an already built Android APK.
class AndroidApk extends ApplicationPackage implements PrebuiltApplicationPackage {
  AndroidApk({
    required super.id,
    required this.applicationPackage,
    required this.versionCode,
    required this.launchActivity,
  });

  /// Creates a new AndroidApk from an existing APK.
  ///
  /// Returns `null` if the APK was invalid or any required tooling was missing.
  static AndroidApk? fromApk(
    File apk, {
    required AndroidSdk androidSdk,
    required ProcessManager processManager,
    required UserMessages userMessages,
    required Logger logger,
    required ProcessUtils processUtils,
  }) {
    final String? aaptPath = androidSdk.latestVersion?.aaptPath;
    if (aaptPath == null || !processManager.canRun(aaptPath)) {
      logger.printError(userMessages.aaptNotFound);
      return null;
    }

    String apptStdout;
    try {
      apptStdout =
          processUtils
              .runSync(<String>[
                aaptPath,
                'dump',
                'xmltree',
                apk.path,
                'AndroidManifest.xml',
              ], throwOnError: true)
              .stdout
              .trim();
    } on ProcessException catch (error) {
      logger.printError('Failed to extract manifest from APK: $error.');
      return null;
    }

    final ApkManifestData? data = ApkManifestData.parseFromXmlDump(apptStdout, logger);

    if (data == null) {
      logger.printError('Unable to read manifest info from ${apk.path}.');
      return null;
    }

    final String? packageName = data.packageName;
    if (packageName == null || data.launchableActivityName == null) {
      logger.printError('Unable to read manifest info from ${apk.path}.');
      return null;
    }

    return AndroidApk(
      id: packageName,
      applicationPackage: apk,
      versionCode: data.versionCode == null ? null : int.tryParse(data.versionCode!),
      launchActivity: '${data.packageName}/${data.launchableActivityName}',
    );
  }

  @override
  final FileSystemEntity applicationPackage;

  /// The path to the activity that should be launched.
  final String launchActivity;

  /// The version code of the APK.
  final int? versionCode;

  /// Creates a new AndroidApk based on the information in the Android manifest.
  static Future<AndroidApk?> fromAndroidProject(
    AndroidProject androidProject, {
    required AndroidSdk? androidSdk,
    required ProcessManager processManager,
    required UserMessages userMessages,
    required ProcessUtils processUtils,
    required Logger logger,
    required FileSystem fileSystem,
    BuildInfo? buildInfo,
  }) async {
    final File apkFile;
    final String filename;
    if (buildInfo == null) {
      filename = 'app.apk';
    } else if (buildInfo.flavor == null) {
      filename = 'app-${buildInfo.mode.cliName}.apk';
    } else {
      filename = 'app-${buildInfo.lowerCasedFlavor}-${buildInfo.mode.cliName}.apk';
    }

    if (androidProject.isUsingGradle && androidProject.isSupportedVersion) {
      Directory apkDirectory = getApkDirectory(androidProject.parent);
      if (androidProject.parent.isModule) {
        // Module builds output the apk in a subdirectory that corresponds
        // to the buildmode of the apk.
        apkDirectory = apkDirectory.childDirectory(buildInfo!.mode.cliName);
      }
      apkFile = apkDirectory.childFile(filename);
      if (apkFile.existsSync()) {
        // Grab information from the .apk. The gradle build script might alter
        // the application Id, so we need to look at what was actually built.
        return AndroidApk.fromApk(
          apkFile,
          androidSdk: androidSdk!,
          processManager: processManager,
          logger: logger,
          userMessages: userMessages,
          processUtils: processUtils,
        );
      }
      // The .apk hasn't been built yet, so we work with what we have. The run
      // command will grab a new AndroidApk after building, to get the updated
      // IDs.
    } else {
      apkFile = fileSystem.file(fileSystem.path.join(getAndroidBuildDirectory(), filename));
    }

    final File manifest = androidProject.appManifestFile;

    if (!manifest.existsSync()) {
      logger.printError('AndroidManifest.xml could not be found.');
      logger.printError('Please check ${manifest.path} for errors.');
      return null;
    }

    final String manifestString = manifest.readAsStringSync();
    XmlDocument document;
    try {
      document = XmlDocument.parse(manifestString);
    } on XmlException catch (exception) {
      String manifestLocation;
      if (androidProject.isUsingGradle) {
        manifestLocation = fileSystem.path.join(
          androidProject.hostAppGradleRoot.path,
          'app',
          'src',
          'main',
          'AndroidManifest.xml',
        );
      } else {
        manifestLocation = fileSystem.path.join(
          androidProject.hostAppGradleRoot.path,
          'AndroidManifest.xml',
        );
      }
      logger.printError('AndroidManifest.xml is not a valid XML document.');
      logger.printError('Please check $manifestLocation for errors.');
      throwToolExit('XML Parser error message: $exception');
    }

    final Iterable<XmlElement> manifests = document.findElements('manifest');
    if (manifests.isEmpty) {
      logger.printError('AndroidManifest.xml has no manifest element.');
      logger.printError('Please check ${manifest.path} for errors.');
      return null;
    }

    // Starting from AGP version 7.3, the `package` attribute in Manifest.xml
    // can be replaced with the `namespace` attribute under the `android` section in `android/app/build.gradle`.
    final String? packageId = manifests.first.getAttribute('package') ?? androidProject.namespace;

    String? launchActivity;
    for (final XmlElement activity in document.findAllElements('activity')) {
      final String? enabled = activity.getAttribute('android:enabled');
      if (enabled != null && enabled == 'false') {
        continue;
      }

      for (final XmlElement element in activity.findElements('intent-filter')) {
        String? actionName = '';
        String? categoryName = '';
        for (final XmlNode node in element.children) {
          if (node is! XmlElement) {
            continue;
          }
          final String? name = node.getAttribute('android:name');
          if (name == 'android.intent.action.MAIN') {
            actionName = name;
          } else if (name == 'android.intent.category.LAUNCHER') {
            categoryName = name;
          }
        }
        if (actionName != null &&
            categoryName != null &&
            actionName.isNotEmpty &&
            categoryName.isNotEmpty) {
          final String? activityName = activity.getAttribute('android:name');
          launchActivity = '$packageId/$activityName';
          break;
        }
      }
    }

    if (packageId == null || launchActivity == null) {
      logger.printError('package identifier or launch activity not found.');
      logger.printError('Please check ${manifest.path} for errors.');
      return null;
    }

    return AndroidApk(
      id: packageId,
      applicationPackage: apkFile,
      versionCode: null,
      launchActivity: launchActivity,
    );
  }

  @override
  String get name => applicationPackage.basename;
}

abstract class _Entry {
  const _Entry(this.parent, this.level);

  final _Element? parent;
  final int level;
}

class _Element extends _Entry {
  _Element._(this.name, _Element? parent, int level) : super(parent, level);

  factory _Element.fromLine(String line, _Element? parent) {
    //      E: application (line=29)
    final List<String> parts = line.trimLeft().split(' ');
    return _Element._(parts[1], parent, line.length - line.trimLeft().length);
  }

  final List<_Entry> children = <_Entry>[];
  final String? name;

  void addChild(_Entry child) {
    children.add(child);
  }

  _Attribute? firstAttribute(String name) {
    for (final _Attribute child in children.whereType<_Attribute>()) {
      if (child.key?.startsWith(name) ?? false) {
        return child;
      }
    }
    return null;
  }

  _Element? firstElement(String name) {
    for (final _Element child in children.whereType<_Element>()) {
      if (child.name?.startsWith(name) ?? false) {
        return child;
      }
    }
    return null;
  }

  Iterable<_Element> allElements(String name) {
    return children.whereType<_Element>().where((_Element e) => e.name?.startsWith(name) ?? false);
  }
}

class _Attribute extends _Entry {
  const _Attribute._(this.key, this.value, _Element? parent, int level) : super(parent, level);

  factory _Attribute.fromLine(String line, _Element parent) {
    //     A: android:label(0x01010001)="hello_world" (Raw: "hello_world")
    const String attributePrefix = 'A: ';
    final List<String> keyVal = line
        .substring(line.indexOf(attributePrefix) + attributePrefix.length)
        .split('=');
    return _Attribute._(keyVal[0], keyVal[1], parent, line.length - line.trimLeft().length);
  }

  final String? key;
  final String? value;
}

class ApkManifestData {
  ApkManifestData._(this._data);

  static bool _isAttributeWithValuePresent(
    _Element baseElement,
    String childElement,
    String attributeName,
    String attributeValue,
  ) {
    final Iterable<_Element> allElements = baseElement.allElements(childElement);
    for (final _Element oneElement in allElements) {
      final String? elementAttributeValue = oneElement.firstAttribute(attributeName)?.value;
      if (elementAttributeValue != null && elementAttributeValue.startsWith(attributeValue)) {
        return true;
      }
    }
    return false;
  }

  static ApkManifestData? parseFromXmlDump(String data, Logger logger) {
    if (data.trim().isEmpty) {
      return null;
    }

    final List<String> lines = data.split('\n');
    assert(lines.length > 3);

    final int manifestLine = lines.indexWhere((String line) => line.contains('E: manifest'));
    final _Element manifest = _Element.fromLine(lines[manifestLine], null);
    _Element currentElement = manifest;

    for (final String line in lines.skip(manifestLine)) {
      final String trimLine = line.trimLeft();
      final int level = line.length - trimLine.length;

      // Handle level out
      while (currentElement.parent != null && level <= currentElement.level) {
        currentElement = currentElement.parent!;
      }

      if (level > currentElement.level) {
        switch (trimLine[0]) {
          case 'A':
            currentElement.addChild(_Attribute.fromLine(line, currentElement));
          case 'E':
            final _Element element = _Element.fromLine(line, currentElement);
            currentElement.addChild(element);
            currentElement = element;
        }
      }
    }

    final _Element? application = manifest.firstElement('application');
    if (application == null) {
      return null;
    }

    final Iterable<_Element> activities = application.allElements('activity');

    _Element? launchActivity;
    for (final _Element activity in activities) {
      final _Attribute? enabled = activity.firstAttribute('android:enabled');
      final Iterable<_Element> intentFilters = activity.allElements('intent-filter');
      final bool isEnabledByDefault = enabled == null;
      final bool isExplicitlyEnabled =
          enabled != null && (enabled.value?.contains('0xffffffff') ?? false);
      if (!(isEnabledByDefault || isExplicitlyEnabled)) {
        continue;
      }

      for (final _Element element in intentFilters) {
        final bool isMainAction = _isAttributeWithValuePresent(
          element,
          'action',
          'android:name',
          '"android.intent.action.MAIN"',
        );
        if (!isMainAction) {
          continue;
        }
        final bool isLauncherCategory = _isAttributeWithValuePresent(
          element,
          'category',
          'android:name',
          '"android.intent.category.LAUNCHER"',
        );
        if (!isLauncherCategory) {
          continue;
        }
        launchActivity = activity;
        break;
      }
      if (launchActivity != null) {
        break;
      }
    }

    final _Attribute? package = manifest.firstAttribute('package');
    // "io.flutter.examples.hello_world" (Raw: "io.flutter.examples.hello_world")
    final String? packageName = package?.value?.substring(1, package.value?.indexOf('" '));

    if (launchActivity == null) {
      logger.printError('Error running $packageName. Default activity not found');
      return null;
    }

    final _Attribute? nameAttribute = launchActivity.firstAttribute('android:name');
    // "io.flutter.examples.hello_world.MainActivity" (Raw: "io.flutter.examples.hello_world.MainActivity")
    final String? activityName = nameAttribute?.value?.substring(
      1,
      nameAttribute.value?.indexOf('" '),
    );

    // Example format: (type 0x10)0x1
    final _Attribute? versionCodeAttr = manifest.firstAttribute('android:versionCode');
    if (versionCodeAttr == null) {
      logger.printError('Error running $packageName. Manifest versionCode not found');
      return null;
    }
    if (versionCodeAttr.value?.startsWith('(type 0x10)') != true) {
      logger.printError('Error running $packageName. Manifest versionCode invalid');
      return null;
    }
    final int? versionCode =
        versionCodeAttr.value == null ? null : int.tryParse(versionCodeAttr.value!.substring(11));
    if (versionCode == null) {
      logger.printError('Error running $packageName. Manifest versionCode invalid');
      return null;
    }

    final Map<String, Map<String, String>> map = <String, Map<String, String>>{
      if (packageName != null) 'package': <String, String>{'name': packageName},
      'version-code': <String, String>{'name': versionCode.toString()},
      if (activityName != null) 'launchable-activity': <String, String>{'name': activityName},
    };

    return ApkManifestData._(map);
  }

  final Map<String, Map<String, String>> _data;

  @visibleForTesting
  Map<String, Map<String, String>> get data =>
      UnmodifiableMapView<String, Map<String, String>>(_data);

  String? get packageName => _data['package'] == null ? null : _data['package']?['name'];

  String? get versionCode => _data['version-code'] == null ? null : _data['version-code']?['name'];

  String? get launchableActivityName {
    return _data['launchable-activity'] == null ? null : _data['launchable-activity']?['name'];
  }

  @override
  String toString() => _data.toString();
}
