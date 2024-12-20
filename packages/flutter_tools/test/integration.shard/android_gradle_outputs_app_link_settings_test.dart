// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' as io;

import 'package:file/file.dart';
import 'package:flutter_tools/src/android/gradle_utils.dart' show getGradlewFileName;
import 'package:flutter_tools/src/base/io.dart';
import 'package:xml/xml.dart';

import '../src/common.dart';
import 'test_utils.dart';

final XmlElement deeplinkFlagMetaData = XmlElement(XmlName('meta-data'), <XmlAttribute>[
  XmlAttribute(XmlName('name', 'android'), 'flutter_deeplinking_enabled'),
  XmlAttribute(XmlName('value', 'android'), 'true'),
]);
final XmlElement pureHttpIntentFilter = XmlElement(
  XmlName('intent-filter'),
  <XmlAttribute>[XmlAttribute(XmlName('autoVerify', 'android'), 'true')],
  <XmlElement>[
    XmlElement(XmlName('action'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.action.VIEW'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.DEFAULT'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.BROWSABLE'),
    ]),
    XmlElement(XmlName('data'), <XmlAttribute>[
      XmlAttribute(XmlName('scheme', 'android'), 'http'),
      XmlAttribute(XmlName('host', 'android'), 'pure-http.com'),
    ]),
  ],
);

final XmlElement nonHttpIntentFilter = XmlElement(
  XmlName('intent-filter'),
  <XmlAttribute>[XmlAttribute(XmlName('autoVerify', 'android'), 'true')],
  <XmlElement>[
    XmlElement(XmlName('action'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.action.VIEW'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.DEFAULT'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.BROWSABLE'),
    ]),
    XmlElement(XmlName('data'), <XmlAttribute>[
      XmlAttribute(XmlName('scheme', 'android'), 'custom'),
      XmlAttribute(XmlName('host', 'android'), 'custom.com'),
    ]),
  ],
);

final XmlElement hybridIntentFilter = XmlElement(
  XmlName('intent-filter'),
  <XmlAttribute>[XmlAttribute(XmlName('autoVerify', 'android'), 'true')],
  <XmlElement>[
    XmlElement(XmlName('action'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.action.VIEW'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.DEFAULT'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.BROWSABLE'),
    ]),
    XmlElement(XmlName('data'), <XmlAttribute>[
      XmlAttribute(XmlName('scheme', 'android'), 'custom'),
      XmlAttribute(XmlName('host', 'android'), 'hybrid.com'),
    ]),
    XmlElement(XmlName('data'), <XmlAttribute>[XmlAttribute(XmlName('scheme', 'android'), 'http')]),
  ],
);

final XmlElement nonAutoVerifyIntentFilter = XmlElement(
  XmlName('intent-filter'),
  <XmlAttribute>[],
  <XmlElement>[
    XmlElement(XmlName('action'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.action.VIEW'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.DEFAULT'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.BROWSABLE'),
    ]),
    XmlElement(XmlName('data'), <XmlAttribute>[
      XmlAttribute(XmlName('scheme', 'android'), 'http'),
      XmlAttribute(XmlName('host', 'android'), 'non-auto-verify.com'),
    ]),
  ],
);
final XmlElement nonActionIntentFilter = XmlElement(
  XmlName('intent-filter'),
  <XmlAttribute>[XmlAttribute(XmlName('autoVerify', 'android'), 'true')],
  <XmlElement>[
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.DEFAULT'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.BROWSABLE'),
    ]),
    XmlElement(XmlName('data'), <XmlAttribute>[
      XmlAttribute(XmlName('scheme', 'android'), 'http'),
      XmlAttribute(XmlName('host', 'android'), 'non-action.com'),
    ]),
  ],
);
final XmlElement nonDefaultCategoryIntentFilter = XmlElement(
  XmlName('intent-filter'),
  <XmlAttribute>[XmlAttribute(XmlName('autoVerify', 'android'), 'true')],
  <XmlElement>[
    XmlElement(XmlName('action'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.action.VIEW'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.BROWSABLE'),
    ]),
    XmlElement(XmlName('data'), <XmlAttribute>[
      XmlAttribute(XmlName('scheme', 'android'), 'http'),
      XmlAttribute(XmlName('host', 'android'), 'non-default-category.com'),
    ]),
  ],
);
final XmlElement nonBrowsableCategoryIntentFilter = XmlElement(
  XmlName('intent-filter'),
  <XmlAttribute>[XmlAttribute(XmlName('autoVerify', 'android'), 'true')],
  <XmlElement>[
    XmlElement(XmlName('action'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.action.VIEW'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.DEFAULT'),
    ]),
    XmlElement(XmlName('data'), <XmlAttribute>[
      XmlAttribute(XmlName('scheme', 'android'), 'http'),
      XmlAttribute(XmlName('host', 'android'), 'non-browsable-category.com'),
    ]),
  ],
);
final XmlElement nonSchemeCategoryIntentFilter = XmlElement(
  XmlName('intent-filter'),
  <XmlAttribute>[XmlAttribute(XmlName('autoVerify', 'android'), 'true')],
  <XmlElement>[
    XmlElement(XmlName('action'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.action.VIEW'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.DEFAULT'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.BROWSABLE'),
    ]),
    XmlElement(XmlName('data'), <XmlAttribute>[
      XmlAttribute(XmlName('host', 'android'), 'non-browsable-category.com'),
    ]),
  ],
);
final XmlElement nonHostCategoryIntentFilter = XmlElement(
  XmlName('intent-filter'),
  <XmlAttribute>[XmlAttribute(XmlName('autoVerify', 'android'), 'true')],
  <XmlElement>[
    XmlElement(XmlName('action'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.action.VIEW'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.DEFAULT'),
    ]),
    XmlElement(XmlName('category'), <XmlAttribute>[
      XmlAttribute(XmlName('name', 'android'), 'android.intent.category.BROWSABLE'),
    ]),
    XmlElement(XmlName('data'), <XmlAttribute>[
      XmlAttribute(XmlName('scheme', 'android'), 'http'),
      XmlAttribute(XmlName('path', 'android'), '/path1'),
    ]),
  ],
);

void main() {
  late Directory tempDir;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('run_test.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  void testDeeplink(
    dynamic deeplink,
    String? scheme,
    String? host,
    String path, {
    required bool hasAutoVerify,
    required bool hasActionView,
    required bool hasDefaultCategory,
    required bool hasBrowsableCategory,
  }) {
    deeplink as Map<String, dynamic>;
    expect(deeplink['scheme'], scheme);
    expect(deeplink['host'], host);
    expect(deeplink['path'], path);
    final Map<String, dynamic> intentFilterCheck =
        deeplink['intentFilterCheck'] as Map<String, dynamic>;
    expect(intentFilterCheck['hasAutoVerify'], hasAutoVerify);
    expect(intentFilterCheck['hasActionView'], hasActionView);
    expect(intentFilterCheck['hasDefaultCategory'], hasDefaultCategory);
    expect(intentFilterCheck['hasBrowsableCategory'], hasBrowsableCategory);
  }

  testWithoutContext(
    'gradle task outputs<mode>AppLinkSettings works when a project has app links',
    () async {
      // Create a new flutter project.
      ProcessResult result = await processManager.run(<String>[
        flutterBin,
        'create',
        tempDir.path,
        '--project-name=testapp',
      ], workingDirectory: tempDir.path);
      expect(result, const ProcessResultMatcher());
      // Adds intent filters for app links
      final String androidManifestPath = fileSystem.path.join(
        tempDir.path,
        'android',
        'app',
        'src',
        'main',
        'AndroidManifest.xml',
      );
      final io.File androidManifestFile = io.File(androidManifestPath);
      final XmlDocument androidManifest = XmlDocument.parse(androidManifestFile.readAsStringSync());
      final XmlElement activity = androidManifest.findAllElements('activity').first;
      activity.children.add(deeplinkFlagMetaData);
      activity.children.add(pureHttpIntentFilter);
      activity.children.add(nonHttpIntentFilter);
      activity.children.add(hybridIntentFilter);
      activity.children.add(nonAutoVerifyIntentFilter);
      activity.children.add(nonActionIntentFilter);
      activity.children.add(nonDefaultCategoryIntentFilter);
      activity.children.add(nonBrowsableCategoryIntentFilter);
      activity.children.add(nonSchemeCategoryIntentFilter);
      activity.children.add(nonHostCategoryIntentFilter);
      androidManifestFile.writeAsStringSync(androidManifest.toString(), flush: true);

      // Ensure that gradle files exists from templates.
      result = await processManager.run(<String>[
        flutterBin,
        'build',
        'apk',
        '--config-only',
      ], workingDirectory: tempDir.path);
      expect(result, const ProcessResultMatcher());

      final Directory androidApp = tempDir.childDirectory('android');
      final io.File fileDump = tempDir
          .childDirectory('build')
          .childDirectory('app')
          .childFile('app-link-settings-debug.json');
      result = await processManager.run(<String>[
        '.${platform.pathSeparator}${getGradlewFileName(platform)}',
        ...getLocalEngineArguments(),
        '-q', // quiet output.
        '-PoutputPath=${fileDump.path}',
        'outputDebugAppLinkSettings',
      ], workingDirectory: androidApp.path);

      expect(result, const ProcessResultMatcher());
      expect(fileDump.existsSync(), true);
      final Map<String, dynamic> json =
          jsonDecode(fileDump.readAsStringSync()) as Map<String, dynamic>;
      expect(json['applicationId'], 'com.example.testapp');
      expect(json['deeplinkingFlagEnabled'], true);
      final List<dynamic> deeplinks = json['deeplinks']! as List<dynamic>;
      expect(deeplinks.length, 10);
      testDeeplink(
        deeplinks[0],
        'http',
        'pure-http.com',
        '.*',
        hasAutoVerify: true,
        hasActionView: true,
        hasDefaultCategory: true,
        hasBrowsableCategory: true,
      );
      testDeeplink(
        deeplinks[1],
        'custom',
        'custom.com',
        '.*',
        hasAutoVerify: true,
        hasActionView: true,
        hasDefaultCategory: true,
        hasBrowsableCategory: true,
      );
      testDeeplink(
        deeplinks[2],
        'custom',
        'hybrid.com',
        '.*',
        hasAutoVerify: true,
        hasActionView: true,
        hasDefaultCategory: true,
        hasBrowsableCategory: true,
      );
      testDeeplink(
        deeplinks[3],
        'http',
        'hybrid.com',
        '.*',
        hasAutoVerify: true,
        hasActionView: true,
        hasDefaultCategory: true,
        hasBrowsableCategory: true,
      );
      testDeeplink(
        deeplinks[4],
        'http',
        'non-auto-verify.com',
        '.*',
        hasAutoVerify: false,
        hasActionView: true,
        hasDefaultCategory: true,
        hasBrowsableCategory: true,
      );
      testDeeplink(
        deeplinks[5],
        'http',
        'non-action.com',
        '.*',
        hasAutoVerify: true,
        hasActionView: false,
        hasDefaultCategory: true,
        hasBrowsableCategory: true,
      );
      testDeeplink(
        deeplinks[6],
        'http',
        'non-default-category.com',
        '.*',
        hasAutoVerify: true,
        hasActionView: true,
        hasDefaultCategory: false,
        hasBrowsableCategory: true,
      );
      testDeeplink(
        deeplinks[7],
        'http',
        'non-browsable-category.com',
        '.*',
        hasAutoVerify: true,
        hasActionView: true,
        hasDefaultCategory: true,
        hasBrowsableCategory: false,
      );
      testDeeplink(
        deeplinks[8],
        null,
        'non-browsable-category.com',
        '.*',
        hasAutoVerify: true,
        hasActionView: true,
        hasDefaultCategory: true,
        hasBrowsableCategory: true,
      );
      testDeeplink(
        deeplinks[9],
        'http',
        null,
        '/path1',
        hasAutoVerify: true,
        hasActionView: true,
        hasDefaultCategory: true,
        hasBrowsableCategory: true,
      );
    },
  );

  testWithoutContext(
    'gradle task outputs<mode>AppLinkSettings works when a project does not have app link and the flutter_deeplinking_enabled flag',
    () async {
      // Create a new flutter project.
      ProcessResult result = await processManager.run(<String>[
        flutterBin,
        'create',
        tempDir.path,
        '--project-name=testapp',
      ], workingDirectory: tempDir.path);
      expect(result, const ProcessResultMatcher());

      // Ensure that gradle files exists from templates.
      result = await processManager.run(<String>[
        flutterBin,
        'build',
        'apk',
        '--config-only',
      ], workingDirectory: tempDir.path);
      expect(result, const ProcessResultMatcher());

      final Directory androidApp = tempDir.childDirectory('android');
      final io.File fileDump = tempDir
          .childDirectory('build')
          .childDirectory('app')
          .childFile('app-link-settings-debug.json');
      result = await processManager.run(<String>[
        '.${platform.pathSeparator}${getGradlewFileName(platform)}',
        ...getLocalEngineArguments(),
        '-q', // quiet output.
        '-PoutputPath=${fileDump.path}',
        'outputDebugAppLinkSettings',
      ], workingDirectory: androidApp.path);

      expect(result, const ProcessResultMatcher());
      expect(fileDump.existsSync(), true);
      final Map<String, dynamic> json =
          jsonDecode(fileDump.readAsStringSync()) as Map<String, dynamic>;
      expect(json['applicationId'], 'com.example.testapp');
      expect(json['deeplinkingFlagEnabled'], false);
      final List<dynamic> deeplinks = json['deeplinks']! as List<dynamic>;
      expect(deeplinks.length, 0);
    },
  );
}
