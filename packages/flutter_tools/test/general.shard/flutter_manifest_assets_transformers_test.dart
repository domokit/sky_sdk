// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';

import '../src/common.dart';

void main() {
  group('parsing of assets section in flutter manifests with asset transformers', () {
    testWithoutContext('parses an asset with a simple transformation', () async {
      final BufferLogger logger = BufferLogger.test();
      const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  assets:
    - path: asset/hello.txt
      transformers:
        - package: my_package
  ''';
      final FlutterManifest? parsedManifest = FlutterManifest.createFromString(
        manifest,
        logger: logger,
      );

      expect(parsedManifest!.assets, <AssetsEntry>[
        AssetsEntry(
          uri: Uri.parse('asset/hello.txt'),
          transformers: const <AssetTransformerEntry>[
            AssetTransformerEntry(package: 'my_package', args: <String>[]),
          ],
        ),
      ]);

      expect(logger.errorText, isEmpty);
    });

    testWithoutContext('parses an asset with a transformation that has args', () async {
      final BufferLogger logger = BufferLogger.test();
      const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  assets:
    - path: asset/hello.txt
      transformers:
        - package: my_package
          args: ["-e", "--color", "purple"]
''';
      final FlutterManifest? parsedManifest = FlutterManifest.createFromString(
        manifest,
        logger: logger,
      );

      expect(parsedManifest!.assets, <AssetsEntry>[
        AssetsEntry(
          uri: Uri.parse('asset/hello.txt'),
          transformers: const <AssetTransformerEntry>[
            AssetTransformerEntry(package: 'my_package', args: <String>['-e', '--color', 'purple']),
          ],
        ),
      ]);
      expect(logger.errorText, isEmpty);
    });

    testWithoutContext('fails when a transformers section is not a list', () async {
      final BufferLogger logger = BufferLogger.test();
      const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  assets:
    - path: asset/hello.txt
      transformers:
        - my_transformer
  ''';
      FlutterManifest.createFromString(manifest, logger: logger);
      expect(
        logger.errorText,
        'Unable to parse assets section.\n'
        'In transformers section of asset "asset/hello.txt": Expected '
        'transformers list to be a list of Map, but element at index 0 was a String.\n',
      );
    });
    testWithoutContext('fails when a transformers section package is not a string', () async {
      final BufferLogger logger = BufferLogger.test();

      const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  assets:
    - path: asset/hello.txt
      transformers:
        - package:
            i am a key: i am a value
  ''';
      FlutterManifest.createFromString(manifest, logger: logger);
      expect(
        logger.errorText,
        'Unable to parse assets section.\n'
        'In transformers section of asset "asset/hello.txt": '
        'Expected "package" to be a String. Found YamlMap instead.\n',
      );
    });

    testWithoutContext('fails when a transformer is missing the package field', () async {
      final BufferLogger logger = BufferLogger.test();
      const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  assets:
    - path: asset/hello.txt
      transformers:
        - args: ["-e"]
    ''';
      FlutterManifest.createFromString(manifest, logger: logger);
      expect(
        logger.errorText,
        'Unable to parse assets section.\n'
        'In transformers section of asset "asset/hello.txt": Expected "package" to be a '
        'String. Found Null instead.\n',
      );
    });

    testWithoutContext(
      'fails when a transformer has args field that is not a list of strings',
      () async {
        final BufferLogger logger = BufferLogger.test();
        const String manifest = '''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  uses-material-design: true
  assets:
    - path: asset/hello.txt
      transformers:
        - package: my_transformer
          args: hello
    ''';
        FlutterManifest.createFromString(manifest, logger: logger);
        expect(
          logger.errorText,
          'Unable to parse assets section.\n'
          'In transformers section of asset "asset/hello.txt": In args section '
          'of transformer using package "my_transformer": Expected args to be a '
          'list of String, but got hello (String).\n',
        );
      },
    );
  });
}
