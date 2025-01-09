// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';

import '../src/common.dart';

void main() {
  group('parsing of assets section in flutter manifests', () {
    testWithoutContext('ignores empty list of assets', () {
      final BufferLogger logger = BufferLogger.test();

      const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  assets: []
''';

      final FlutterManifest? flutterManifest = FlutterManifest.createFromString(
        manifest,
        logger: logger,
      );

      expect(flutterManifest, isNotNull);
      expect(flutterManifest!.assets, isEmpty);
    });

    testWithoutContext('parses two simple asset declarations', () async {
      final BufferLogger logger = BufferLogger.test();
      const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  assets:
    - a/foo
    - a/bar
''';

      final FlutterManifest flutterManifest =
          FlutterManifest.createFromString(manifest, logger: logger)!;

      expect(flutterManifest.assets, <AssetsEntry>[
        AssetsEntry(uri: Uri.parse('a/foo')),
        AssetsEntry(uri: Uri.parse('a/bar')),
      ]);
    });

    testWithoutContext('does not crash on empty entry', () {
      final BufferLogger logger = BufferLogger.test();
      const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  assets:
    - lib/gallery/example_code.dart
    -
''';

      FlutterManifest.createFromString(manifest, logger: logger);

      expect(logger.errorText, contains('Asset manifest contains a null or empty uri.'));
    });

    testWithoutContext('handles special characters in asset URIs', () {
      final BufferLogger logger = BufferLogger.test();

      const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  assets:
    - lib/gallery/abc#xyz
    - lib/gallery/abc?xyz
    - lib/gallery/aaa bbb
''';

      final FlutterManifest flutterManifest =
          FlutterManifest.createFromString(manifest, logger: logger)!;
      final List<AssetsEntry> assets = flutterManifest.assets;

      expect(assets, <AssetsEntry>[
        AssetsEntry(uri: Uri.parse('lib/gallery/abc%23xyz')),
        AssetsEntry(uri: Uri.parse('lib/gallery/abc%3Fxyz')),
        AssetsEntry(uri: Uri.parse('lib/gallery/aaa%20bbb')),
      ]);
    });

    testWithoutContext('parses an asset with flavors', () async {
      final BufferLogger logger = BufferLogger.test();
      const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  assets:
    - path: a/foo
      flavors:
        - apple
        - strawberry
''';

      final FlutterManifest flutterManifest =
          FlutterManifest.createFromString(manifest, logger: logger)!;

      expect(flutterManifest.assets, <AssetsEntry>[
        AssetsEntry(uri: Uri.parse('a/foo'), flavors: const <String>{'apple', 'strawberry'}),
      ]);
    });

    testWithoutContext("prints an error when an asset entry's flavor is not a string", () async {
      final BufferLogger logger = BufferLogger.test();

      const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  assets:
    - assets/folder/
    - path: assets/vanilla/
      flavors:
        - key1: value1
          key2: value2
''';
      FlutterManifest.createFromString(manifest, logger: logger);
      expect(
        logger.errorText,
        contains(
          'Unable to parse assets section.\n'
          'In flavors section of asset "assets/vanilla/": Expected flavors '
          'to be a list of String, but element at index 0 was a YamlMap.\n',
        ),
      );
    });
  });
}
