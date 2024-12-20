// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/deferred_component.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';
import 'package:yaml_edit/yaml_edit.dart';

import '../src/common.dart';

void main() {
  late BufferLogger logger;

  setUpAll(() {
    Cache.flutterRoot = getFlutterRoot();
  });

  setUp(() {
    logger = BufferLogger.test();
  });

  testWithoutContext('FlutterManifest is empty when the pubspec.yaml file is empty', () async {
    final FlutterManifest flutterManifest = FlutterManifest.createFromString('', logger: logger)!;

    expect(flutterManifest.isEmpty, true);
    expect(flutterManifest.appName, '');
    expect(flutterManifest.usesMaterialDesign, false);
    expect(flutterManifest.fontsDescriptor, isEmpty);
    expect(flutterManifest.fonts, isEmpty);
    expect(flutterManifest.assets, isEmpty);
    expect(flutterManifest.additionalLicenses, isEmpty);
    expect(flutterManifest.defaultFlavor, null);
  });

  testWithoutContext('FlutterManifest is null when the pubspec.yaml file is not a map', () async {
    expect(FlutterManifest.createFromString('Not a map', logger: logger), isNull);

    expect(logger.errorText, contains('Expected YAML map'));
  });

  testWithoutContext(
    'FlutterManifest has no fonts or assets when the "flutter" section is empty',
    () async {
      const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
''';
      final FlutterManifest flutterManifest =
          FlutterManifest.createFromString(manifest, logger: logger)!;

      expect(flutterManifest, isNotNull);
      expect(flutterManifest.isEmpty, false);
      expect(flutterManifest.appName, 'test');
      expect(flutterManifest.usesMaterialDesign, false);
      expect(flutterManifest.fontsDescriptor, isEmpty);
      expect(flutterManifest.fonts, isEmpty);
      expect(flutterManifest.assets, isEmpty);
    },
  );

  testWithoutContext('FlutterManifest knows if Material Design is used', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
''';
    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.usesMaterialDesign, true);
  });

  testWithoutContext('FlutterManifest knows if generate is provided', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  generate: true
''';
    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.generateLocalizations, true);
  });

  testWithoutContext('FlutterManifest can parse invalid generate key', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  generate: "invalid"
''';
    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.generateLocalizations, false);
  });

  testWithoutContext('FlutterManifest knows if generate is disabled', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  generate: false
''';
    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.generateLocalizations, false);
  });

  testWithoutContext('FlutterManifest has one font family with one asset', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  fonts:
    - family: foo
      fonts:
        - asset: a/bar
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.fonts, hasLength(1));
    expect(
      flutterManifest.fonts.single,
      matchesFont(
        familyName: 'foo',
        descriptor: <String, Object>{
          'family': 'foo',
          'fonts': <Object>[
            <String, Object>{'asset': 'a/bar'},
          ],
        },
        fontAssets: <Matcher>[matchesFontAsset(assetUri: Uri.parse('a/bar'))],
      ),
    );
  });

  testWithoutContext('FlutterManifest has one font family with a simple asset '
      'and one with weight', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  fonts:
    - family: foo
      fonts:
        - asset: a/bar
        - asset: a/bar
          weight: 400
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.fonts, hasLength(1));
    expect(
      flutterManifest.fonts.single,
      matchesFont(
        familyName: 'foo',
        descriptor: <String, Object>{
          'family': 'foo',
          'fonts': <Object>[
            <String, Object>{'asset': 'a/bar'},
            <String, Object>{'weight': 400, 'asset': 'a/bar'},
          ],
        },
        fontAssets: <Matcher>[
          matchesFontAsset(assetUri: Uri.parse('a/bar')),
          matchesFontAsset(assetUri: Uri.parse('a/bar'), weight: 400),
        ],
      ),
    );
  });

  testWithoutContext('FlutterManifest has one font family with a simple asset '
      'and one with weight and style', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  fonts:
    - family: foo
      fonts:
        - asset: a/bar
        - asset: a/bar
          weight: 400
          style: italic
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.fonts, hasLength(1));
    expect(
      flutterManifest.fonts.single,
      matchesFont(
        familyName: 'foo',
        descriptor: <String, Object>{
          'family': 'foo',
          'fonts': <Object>[
            <String, Object>{'asset': 'a/bar'},
            <String, Object>{'weight': 400, 'style': 'italic', 'asset': 'a/bar'},
          ],
        },
        fontAssets: <Matcher>[
          matchesFontAsset(assetUri: Uri.parse('a/bar')),
          matchesFontAsset(assetUri: Uri.parse('a/bar'), weight: 400, style: 'italic'),
        ],
      ),
    );
  });

  testWithoutContext('FlutterManifest has two font families, each with one '
      'simple asset and one with weight and style', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  fonts:
    - family: foo
      fonts:
        - asset: a/bar
        - asset: a/bar
          weight: 400
          style: italic
    - family: bar
      fonts:
        - asset: a/baz
        - weight: 400
          asset: a/baz
          style: italic
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.fonts, hasLength(2));
    expect(
      flutterManifest.fonts,
      containsAll(<Matcher>[
        matchesFont(
          familyName: 'foo',
          descriptor: <String, Object>{
            'family': 'foo',
            'fonts': <Object>[
              <String, Object>{'asset': 'a/bar'},
              <String, Object>{'weight': 400, 'style': 'italic', 'asset': 'a/bar'},
            ],
          },
          fontAssets: <Matcher>[
            matchesFontAsset(assetUri: Uri.parse('a/bar')),
            matchesFontAsset(assetUri: Uri.parse('a/bar'), weight: 400, style: 'italic'),
          ],
        ),
        matchesFont(
          familyName: 'bar',
          descriptor: <String, Object>{
            'family': 'bar',
            'fonts': <Object>[
              <String, Object>{'asset': 'a/baz'},
              <String, Object>{'weight': 400, 'style': 'italic', 'asset': 'a/baz'},
            ],
          },
          fontAssets: <Matcher>[
            matchesFontAsset(assetUri: Uri.parse('a/baz')),
            matchesFontAsset(assetUri: Uri.parse('a/baz'), weight: 400, style: 'italic'),
          ],
        ),
      ]),
    );
  });

  testWithoutContext('FlutterManifest.fontsDescriptor combines descriptors from '
      'individual fonts', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  fonts:
    - family: foo
      fonts:
        - asset: a/bar
        - asset: a/bar
          weight: 400
          style: italic
    - family: bar
      fonts:
        - asset: a/baz
        - weight: 400
          asset: a/baz
          style: italic
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.fontsDescriptor, <Object>[
      <String, Object>{
        'family': 'foo',
        'fonts': <Object>[
          <String, Object>{'asset': 'a/bar'},
          <String, Object>{'weight': 400, 'style': 'italic', 'asset': 'a/bar'},
        ],
      },
      <String, Object>{
        'family': 'bar',
        'fonts': <Object>[
          <String, Object>{'asset': 'a/baz'},
          <String, Object>{'weight': 400, 'style': 'italic', 'asset': 'a/baz'},
        ],
      },
    ]);
  });

  testWithoutContext('FlutterManifest has only one of two font families when '
      'one declaration is missing the "family" option', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  fonts:
    - family: foo
      fonts:
        - asset: a/bar
        - asset: a/bar
          weight: 400
          style: italic
    - fonts:
        - asset: a/baz
        - asset: a/baz
          weight: 400
          style: italic
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.fonts, hasLength(1));
    expect(
      flutterManifest.fonts,
      containsAll(<Matcher>[
        matchesFont(
          familyName: 'foo',
          descriptor: <String, Object>{
            'family': 'foo',
            'fonts': <Object>[
              <String, Object>{'asset': 'a/bar'},
              <String, Object>{'weight': 400, 'style': 'italic', 'asset': 'a/bar'},
            ],
          },
          fontAssets: <Matcher>[
            matchesFontAsset(assetUri: Uri.parse('a/bar')),
            matchesFontAsset(assetUri: Uri.parse('a/bar'), weight: 400, style: 'italic'),
          ],
        ),
      ]),
    );
  });

  testWithoutContext('FlutterManifest has only one of two font families when '
      'one declaration is missing the "fonts" option', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  fonts:
    - family: foo
      fonts:
        - asset: a/bar
        - asset: a/bar
          weight: 400
          style: italic
    - family: bar
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.fonts, hasLength(1));
    expect(
      flutterManifest.fonts,
      containsAll(<Matcher>[
        matchesFont(
          familyName: 'foo',
          descriptor: <String, Object>{
            'family': 'foo',
            'fonts': <Object>[
              <String, Object>{'asset': 'a/bar'},
              <String, Object>{'weight': 400, 'style': 'italic', 'asset': 'a/bar'},
            ],
          },
          fontAssets: <Matcher>[
            matchesFontAsset(assetUri: Uri.parse('a/bar')),
            matchesFontAsset(assetUri: Uri.parse('a/bar'), weight: 400, style: 'italic'),
          ],
        ),
      ]),
    );
  });

  testWithoutContext('FlutterManifest has no font family when declaration is '
      'missing the "asset" option', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  fonts:
    - family: foo
      fonts:
        - weight: 400
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.fontsDescriptor, isEmpty);
    expect(flutterManifest.fonts, isEmpty);
  });

  testWithoutContext('FlutterManifest allows a blank flutter section', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.isEmpty, false);
    expect(flutterManifest.isModule, false);
    expect(flutterManifest.isPlugin, false);
    expect(flutterManifest.androidPackage, null);
    expect(flutterManifest.usesAndroidX, false);
  });

  testWithoutContext('FlutterManifest allows a module declaration', () {
    const String manifest = '''
name: test
flutter:
  module:
    androidPackage: com.example
    androidX: true
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.isModule, true);
    expect(flutterManifest.androidPackage, 'com.example');
    expect(flutterManifest.usesAndroidX, true);
  });

  testWithoutContext('FlutterManifest allows a legacy plugin declaration', () {
    const String manifest = '''
name: test
flutter:
  plugin:
    androidPackage: com.example
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.isPlugin, true);
    expect(flutterManifest.androidPackage, 'com.example');
  });

  testWithoutContext('FlutterManifest allows a multi-plat plugin declaration '
      'with android only', () {
    const String manifest = '''
name: test
flutter:
    plugin:
      platforms:
        android:
          package: com.example
          pluginClass: TestPlugin
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.isPlugin, true);
    expect(flutterManifest.androidPackage, 'com.example');
  });

  testWithoutContext('FlutterManifest allows a multi-plat plugin declaration '
      'with ios only', () {
    const String manifest = '''
name: test
flutter:
    plugin:
      platforms:
        ios:
          pluginClass: HelloPlugin
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.isPlugin, true);
    expect(flutterManifest.androidPackage, isNull);
  });

  testWithoutContext('FlutterManifest handles an invalid plugin declaration', () {
    const String manifest = '''
name: test
flutter:
    plugin:
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(logger.errorText, contains('Expected "plugin" to be an object, but got null'));
  });

  testWithoutContext('FlutterManifest parses major.minor.patch+build version clause 1', () {
    const String manifest = '''
name: test
version: 1.0.0+2
dependencies:
  flutter:
    sdk: flutter
flutter:
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(
      flutterManifest,
      matchesManifest(appVersion: '1.0.0+2', buildName: '1.0.0', buildNumber: '2'),
    );
  });

  testWithoutContext('FlutterManifest parses major.minor.patch with no build version', () {
    const String manifest = '''
name: test
version: 0.0.1
dependencies:
  flutter:
    sdk: flutter
flutter:
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, matchesManifest(appVersion: '0.0.1', buildName: '0.0.1'));
  });

  testWithoutContext('FlutterManifest parses major.minor.patch+build version clause 2', () {
    const String manifest = '''
name: test
version: 1.0.0-beta+exp.sha.5114f85
dependencies:
  flutter:
    sdk: flutter
flutter:
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(
      flutterManifest,
      matchesManifest(
        appVersion: '1.0.0-beta+exp.sha.5114f85',
        buildName: '1.0.0-beta',
        buildNumber: 'exp.sha.5114f85',
      ),
    );
  });

  testWithoutContext('FlutterManifest parses major.minor+build version clause', () {
    const String manifest = '''
name: test
version: 1.0.0+2
dependencies:
  flutter:
    sdk: flutter
flutter:
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(
      flutterManifest,
      matchesManifest(appVersion: '1.0.0+2', buildName: '1.0.0', buildNumber: '2'),
    );
  });

  testWithoutContext('FlutterManifest parses empty version clause', () {
    const String manifest = '''
name: test
version:
dependencies:
  flutter:
    sdk: flutter
flutter:
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, matchesManifest());
  });

  testWithoutContext('FlutterManifest parses no version clause', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, matchesManifest());
  });

  // Regression test for https://github.com/flutter/flutter/issues/31764
  testWithoutContext('FlutterManifest returns proper error when font detail is malformed', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  fonts:
    - family: foo
      fonts:
        -asset: a/bar
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(logger.errorText, contains('Expected "fonts" to either be null or a list.'));
  });

  testWithoutContext('FlutterManifest ignores empty list of fonts', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  fonts: []
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, isNotNull);
    expect(flutterManifest!.fonts.length, 0);
  });

  testWithoutContext('FlutterManifest returns proper error when font detail is '
      'not a list of maps', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  fonts:
    - family: foo
      fonts:
        - asset
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(logger.errorText, contains('Expected "fonts" to be a list of maps.'));
  });

  testWithoutContext('FlutterManifest returns proper error when font is a map '
      'instead of a list', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  fonts:
    family: foo
    fonts:
      -asset: a/bar
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(logger.errorText, contains('Expected "fonts" to be a list'));
  });

  testWithoutContext('FlutterManifest returns proper error when second font '
      'family is invalid', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  fonts:
    - family: foo
      fonts:
        - asset: a/bar
    - string
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(logger.errorText, contains('Expected a map.'));
  });

  testWithoutContext('FlutterManifest returns proper error when flutter is a '
      'list instead of a map', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  - uses-material-design: true
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(
      logger.errorText,
      contains(
        'Expected "flutter" section to be an object or null, but got '
        '[{uses-material-design: true}].',
      ),
    );
  });

  testWithoutContext('FlutterManifest can parse manifest on posix filesystem', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
''';
    final FileSystem fileSystem = MemoryFileSystem.test();
    fileSystem.file('pubspec.yaml').writeAsStringSync(manifest);

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromPath('pubspec.yaml', fileSystem: fileSystem, logger: logger)!;

    expect(flutterManifest.isEmpty, false);
  });

  testWithoutContext('FlutterManifest can parse manifest on windows filesystem', () {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
''';

    final FileSystem fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
    fileSystem.file('pubspec.yaml').writeAsStringSync(manifest);

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromPath('pubspec.yaml', fileSystem: fileSystem, logger: logger)!;

    expect(flutterManifest.isEmpty, false);
  });

  testWithoutContext(
    'FlutterManifest getSupportedPlatforms return null if runs on legacy format',
    () {
      const String manifest = '''
name: test
flutter:
  plugin:
    androidPackage: com.example
''';

      final FlutterManifest flutterManifest =
          FlutterManifest.createFromString(manifest, logger: logger)!;

      expect(flutterManifest.isPlugin, true);
      expect(flutterManifest.supportedPlatforms, null);
    },
  );

  testWithoutContext(
    'FlutterManifest validSupportedPlatforms return null if the platform keys are not valid',
    () {
      const String manifest = '''
name: test
flutter:
  plugin:
    platforms:
      some_platform:
        pluginClass: SomeClass
''';

      final FlutterManifest flutterManifest =
          FlutterManifest.createFromString(manifest, logger: logger)!;

      expect(flutterManifest.isPlugin, true);
      expect(flutterManifest.validSupportedPlatforms, null);
    },
  );

  testWithoutContext('FlutterManifest validSupportedPlatforms only returns valid platforms', () {
    const String manifest = '''
name: test
flutter:
  plugin:
    platforms:
      some_platform:
        pluginClass: SomeClass
      ios:
        pluginClass: SomeClass
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.isPlugin, true);
    final Map<String, dynamic> validSupportedPlatforms = flutterManifest.validSupportedPlatforms!;
    expect(validSupportedPlatforms['ios'], <String, dynamic>{'pluginClass': 'SomeClass'});
    expect(validSupportedPlatforms['some_platform'], isNull);
  });

  testWithoutContext('FlutterManifest getSupportedPlatforms returns valid platforms.', () {
    const String manifest = '''
name: test
flutter:
  plugin:
    platforms:
      android:
        package: com.example
        pluginClass: SomeClass
      ios:
        pluginClass: SomeClass
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.isPlugin, true);
    final Map<String, dynamic> validSupportedPlatforms = flutterManifest.validSupportedPlatforms!;
    expect(validSupportedPlatforms['ios'], <String, dynamic>{'pluginClass': 'SomeClass'});
    expect(validSupportedPlatforms['android'], <String, dynamic>{
      'pluginClass': 'SomeClass',
      'package': 'com.example',
    });
  });

  testWithoutContext('FlutterManifest validates a platform section that is a list '
      'instead of a map', () {
    const String manifest = '''
name: test
flutter:
    plugin:
      platforms:
        - android
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(
      logger.errorText,
      contains('flutter.plugin.platforms should be a map with the platform name as the key'),
    );
  });

  testWithoutContext('FlutterManifest validates plugin format not support.', () {
    const String manifest = '''
name: test
flutter:
  plugin:
    android:
      package: com.example
      pluginClass: SomeClass
    ios:
      pluginClass: SomeClass
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(
      logger.errorText,
      contains('Cannot find the `flutter.plugin.platforms` key in the `pubspec.yaml` file. '),
    );
  });

  testWithoutContext('FlutterManifest handles empty licenses list', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  licenses: []
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, isNotNull);
    expect(flutterManifest!.additionalLicenses.length, 0);
  });

  testWithoutContext('FlutterManifest can specify additional LICENSE files', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  licenses:
    - foo.txt
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.additionalLicenses, <String>['foo.txt']);
  });

  testWithoutContext('FlutterManifest can validate incorrect licenses key', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  licenses: foo.txt
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(
      logger.errorText,
      'Expected "licenses" to be a list of files, but got foo.txt (String).\n',
    );
  });

  testWithoutContext('FlutterManifest validates individual list items', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  licenses:
    - foo.txt
    - bar: fizz
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(
      logger.errorText,
      'Expected "licenses" to be a list of files, but '
      'element at index 1 was a YamlMap.\n',
    );
  });

  testWithoutContext('FlutterManifest parses single deferred components', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  deferred-components:
    - name: component1
      libraries:
        - lib1
      assets:
        - path/to/asset.jpg
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest, isNotNull);
    final List<DeferredComponent> deferredComponents = flutterManifest.deferredComponents!;
    expect(deferredComponents.length, 1);
    expect(deferredComponents[0].name, 'component1');
    expect(deferredComponents[0].libraries.length, 1);
    expect(deferredComponents[0].libraries[0], 'lib1');
    expect(deferredComponents[0].assets.length, 1);
    expect(deferredComponents[0].assets[0].uri.path, 'path/to/asset.jpg');
  });

  testWithoutContext('FlutterManifest parses multiple deferred components', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  deferred-components:
    - name: component1
      libraries:
        - lib1
      assets:
        - path/to/asset.jpg
    - name: component2
      libraries:
        - lib2
        - lib3
      assets:
        - path/to/asset2.jpg
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest, isNotNull);
    final List<DeferredComponent> deferredComponents = flutterManifest.deferredComponents!;
    expect(deferredComponents.length, 2);
    expect(deferredComponents[0].name, 'component1');
    expect(deferredComponents[0].libraries.length, 1);
    expect(deferredComponents[0].libraries[0], 'lib1');
    expect(deferredComponents[0].assets.length, 1);
    expect(deferredComponents[0].assets[0].uri.path, 'path/to/asset.jpg');

    expect(deferredComponents[1].name, 'component2');
    expect(deferredComponents[1].libraries.length, 2);
    expect(deferredComponents[1].libraries[0], 'lib2');
    expect(deferredComponents[1].libraries[1], 'lib3');
    expect(deferredComponents[1].assets.length, 1);
    expect(deferredComponents[1].assets[0].uri.path, 'path/to/asset2.jpg');
  });

  testWithoutContext('FlutterManifest parses empty deferred components', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  deferred-components:
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest, isNotNull);
    expect(flutterManifest.deferredComponents!.length, 0);
  });

  testWithoutContext('FlutterManifest deferred component requires name', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  deferred-components:
    - libraries:
        - lib1
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(
      logger.errorText,
      'Expected the 0 element in "deferred-components" to have required key "name" of type String\n',
    );
  });

  testWithoutContext('FlutterManifest deferred component is list', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  deferred-components: blah
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(
      logger.errorText,
      'Expected "deferred-components" to be a list, but got blah (String).\n',
    );
  });

  testWithoutContext('FlutterManifest deferred component libraries is list', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  deferred-components:
    - name: blah
      libraries: blah
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(
      logger.errorText,
      'Expected "libraries" key in the element at '
      'index 0 of "deferred-components" to be a list of String, but '
      'got blah (String).\n',
    );
  });

  testWithoutContext('FlutterManifest deferred component libraries is string', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  deferred-components:
    - name: blah
      libraries:
        - not-a-string:
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(
      logger.errorText,
      'Expected "libraries" key in the element at '
      'index 0 of "deferred-components" to be a list of String, but '
      'element at index 0 was a YamlMap.\n',
    );
  });

  testWithoutContext('FlutterManifest deferred component assets is string', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  deferred-components:
    - name: blah
      assets:
        - not-a-string:
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(
      logger.errorText,
      'Asset manifest entry is malformed. Expected asset entry to be either a string or a map containing a "path" entry. Got Null instead.\n',
    );
  });

  testWithoutContext('FlutterManifest deferred component multiple assets is string', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  deferred-components:
    - name: blah
      assets:
        - path/to/file.so
        - also-not-a-string:
          - woo
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(
      logger.errorText,
      'Asset manifest entry is malformed. Expected asset entry to be either a string or a map containing a "path" entry. Got Null instead.\n',
    );
  });

  testWithoutContext('FlutterManifest multiple deferred components assets is string', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  deferred-components:
    - name: blah
      assets:
        - path/to/file.so
    - name: blah2
      assets:
        - path/to/other/file.so
        - not-a-string:
          - woo
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(
      logger.errorText,
      'Asset manifest entry is malformed. Expected asset entry to be either a string or a map containing a "path" entry. Got Null instead.\n',
    );
  });

  testWithoutContext('FlutterManifest deferred component assets is list', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  deferred-components:
    - name: blah
      assets: blah
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(logger.errorText, 'Expected "assets" to be a list, but got blah (String).\n');
  });

  testWithoutContext('FlutterManifest parses asset-only deferred components', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  deferred-components:
    - name: component1
      assets:
        - path/to/asset1.jpg
        - path/to/asset2.jpg
        - path/to/asset3.jpg
''';

    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest, isNotNull);
    final List<DeferredComponent> deferredComponents = flutterManifest.deferredComponents!;
    expect(deferredComponents.length, 1);
    expect(deferredComponents[0].name, 'component1');
    expect(deferredComponents[0].libraries.length, 0);
    expect(deferredComponents[0].assets.length, 3);
    expect(deferredComponents[0].assets[0].uri.path, 'path/to/asset1.jpg');
    expect(deferredComponents[0].assets[1].uri.path, 'path/to/asset2.jpg');
    expect(deferredComponents[0].assets[2].uri.path, 'path/to/asset3.jpg');
  });

  testWithoutContext('FlutterManifest can parse empty dependencies', () async {
    const String manifest = '''
name: test
''';
    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: BufferLogger.test(),
    );

    expect(flutterManifest, isNotNull);
    expect(flutterManifest!.dependencies, isEmpty);
  });

  testWithoutContext('FlutterManifest knows if Swift Package Manager is disabled', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  disable-swift-package-manager: true
''';
    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.disabledSwiftPackageManager, true);
  });

  testWithoutContext('FlutterManifest does not disable Swift Package Manager if missing', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
''';
    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    expect(flutterManifest.disabledSwiftPackageManager, false);
  });

  testWithoutContext('FlutterManifest can parse default flavor', () async {
    const String manifest = '''
name: test
flutter:
    default-flavor: prod
''';
    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: BufferLogger.test(),
    );

    expect(flutterManifest, isNotNull);
    expect(flutterManifest!.defaultFlavor, 'prod');
  });

  testWithoutContext('FlutterManifest fails on invalid default flavor', () async {
    const String manifest = '''
name: test
flutter:
    default-flavor: 3
''';

    final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
      manifest,
      logger: logger,
    );

    expect(flutterManifest, null);
    expect(logger.errorText, 'Expected "default-flavor" to be a string, but got 3 (int).\n');
  });

  testWithoutContext('FlutterManifest.copyWith generates a valid manifest', () async {
    const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
''';
    final FlutterManifest flutterManifest =
        FlutterManifest.createFromString(manifest, logger: logger)!;

    final FlutterManifest updatedManifest = flutterManifest.copyWith(
      logger: logger,
      assets: <AssetsEntry>[
        AssetsEntry(
          uri: Uri(path: 'foo'),
          flavors: const <String>{'flavor'},
          transformers: const <AssetTransformerEntry>[
            AssetTransformerEntry(package: 'package:foo', args: <String>['arg']),
          ],
        ),
      ],
      fonts: <Font>[
        Font('fontFamily', <FontAsset>[
          FontAsset(Uri(path: 'assetUri'), weight: 100, style: 'normal'),
        ]),
      ],
      models: <Uri>[Uri(path: 'modelUri')],
      shaders: <Uri>[Uri(path: 'shaderUri')],
      deferredComponents: <DeferredComponent>[
        DeferredComponent(
          name: 'deferredComponent',
          libraries: const <String>['deferredComponentLibrary'],
          assets: <AssetsEntry>[
            AssetsEntry(
              uri: Uri(path: 'deferredComponentUri'),
              flavors: const <String>{'deferredComponentFlavor'},
              transformers: const <AssetTransformerEntry>[
                AssetTransformerEntry(
                  package: 'package:deferredComponent',
                  args: <String>['deferredComponentArg'],
                ),
              ],
            ),
          ],
        ),
      ],
    );

    final YamlEditor editor = YamlEditor('');
    editor.update(const <String>[], updatedManifest.toYaml());
    expect(editor.toString(), '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  assets:
    - path: foo
      flavors:
        - flavor
      transformers:
        - package: package:foo
          args:
            - arg
  fonts:
    - family: fontFamily
      fonts:
        - weight: 100
          style: normal
          asset: assetUri
  shaders:
    - shaderUri
  models:
    - modelUri
  deferred-components:
    - name: deferredComponent
      libraries:
        - deferredComponentLibrary
      assets:
        - path: deferredComponentUri
          flavors:
            - deferredComponentFlavor
          transformers:
            - package: package:deferredComponent
              args:
                - deferredComponentArg''');
  });
}

Matcher matchesManifest({String? appVersion, String? buildName, String? buildNumber}) {
  return isA<FlutterManifest>()
      .having((FlutterManifest manifest) => manifest.appVersion, 'appVersion', appVersion)
      .having((FlutterManifest manifest) => manifest.buildName, 'buildName', buildName)
      .having((FlutterManifest manifest) => manifest.buildNumber, 'buildNumber', buildNumber);
}

Matcher matchesFontAsset({required Uri assetUri, int? weight, String? style}) {
  return isA<FontAsset>()
      .having((FontAsset fontAsset) => fontAsset.assetUri, 'assetUri', assetUri)
      .having((FontAsset fontAsset) => fontAsset.weight, 'weight', weight)
      .having((FontAsset fontAsset) => fontAsset.style, 'style', style);
}

Matcher matchesFont({
  required Map<String, Object> descriptor,
  required String familyName,
  required List<Matcher> fontAssets,
}) {
  return isA<Font>()
      .having((Font font) => font.descriptor, 'descriptor', descriptor)
      .having((Font font) => font.familyName, 'familyName', familyName)
      .having((Font font) => font.fontAssets, 'fontAssets', containsAll(fontAssets));
}
