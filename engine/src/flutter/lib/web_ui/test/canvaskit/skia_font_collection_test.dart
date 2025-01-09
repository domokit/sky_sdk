// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine.dart';

import '../common/fake_asset_manager.dart';
import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('$SkiaFontCollection', () {
    setUpUnitTests();

    final List<String> warnings = <String>[];
    late void Function(String) oldPrintWarning;
    late FakeAssetScope testAssetScope;

    setUpAll(() async {
      oldPrintWarning = printWarning;
      printWarning = (String warning) {
        warnings.add(warning);
      };
    });

    tearDownAll(() {
      printWarning = oldPrintWarning;
    });

    setUp(() {
      testAssetScope = fakeAssetManager.pushAssetScope();
      mockHttpFetchResponseFactory = null;
      warnings.clear();
    });

    tearDown(() {
      fakeAssetManager.popAssetScope(testAssetScope);
      mockHttpFetchResponseFactory = null;
    });

    test('logs no warnings with the default mock asset manager', () async {
      final SkiaFontCollection fontCollection = SkiaFontCollection();
      await fontCollection.loadAssetFonts(await fetchFontManifest(fakeAssetManager));

      expect(warnings, isEmpty);
    });

    test('logs a warning if one of the registered fonts is invalid', () async {
      mockHttpFetchResponseFactory = (String url) async {
        final ByteBuffer bogusData =
            Uint8List.fromList('this is not valid font data'.codeUnits).buffer;
        return MockHttpFetchResponse(
          status: 200,
          url: url,
          contentLength: bogusData.lengthInBytes,
          payload: MockHttpFetchPayload(byteBuffer: bogusData),
        );
      };
      final SkiaFontCollection fontCollection = SkiaFontCollection();
      testAssetScope.setAsset(
        'FontManifest.json',
        stringAsUtf8Data('''
[
   {
      "family":"Roboto",
      "fonts":[{"asset":"/fonts/Roboto-Regular.ttf"}]
   },
   {
      "family": "BrokenFont",
      "fonts":[{"asset":"packages/bogus/BrokenFont.ttf"}]
   }
  ]
      '''),
      );
      // It should complete without error, but emit a warning about BrokenFont.
      await fontCollection.loadAssetFonts(await fetchFontManifest(fakeAssetManager));
      expect(
        warnings,
        containsAllInOrder(<String>[
          'Failed to load font BrokenFont at packages/bogus/BrokenFont.ttf',
          'Verify that packages/bogus/BrokenFont.ttf contains a valid font.',
        ]),
      );
    });

    test(
      'logs an HTTP warning if one of the registered fonts is missing (404 file not found)',
      () async {
        final SkiaFontCollection fontCollection = SkiaFontCollection();
        testAssetScope.setAsset(
          'FontManifest.json',
          stringAsUtf8Data('''
[
   {
      "family":"Roboto",
      "fonts":[{"asset":"/fonts/Roboto-Regular.ttf"}]
   },
   {
      "family": "ThisFontDoesNotExist",
      "fonts":[{"asset":"packages/bogus/ThisFontDoesNotExist.ttf"}]
   }
  ]
      '''),
        );

        // It should complete without error, but emit a warning about ThisFontDoesNotExist.
        await fontCollection.loadAssetFonts(await fetchFontManifest(fakeAssetManager));
        expect(
          warnings,
          containsAllInOrder(<String>[
            'Font family ThisFontDoesNotExist not found (404) at packages/bogus/ThisFontDoesNotExist.ttf',
          ]),
        );
      },
    );

    test('prioritizes Ahem loaded via FontManifest.json', () async {
      final SkiaFontCollection fontCollection = SkiaFontCollection();
      testAssetScope.setAsset(
        'FontManifest.json',
        stringAsUtf8Data(
          '''
        [
          {
            "family":"Roboto",
            "fonts":[{"asset":"/assets/fonts/Roboto-Regular.ttf"}]
          },
          {
            "family":"Ahem",
            "fonts":[{"asset":"/assets/fonts/Roboto-Regular.ttf"}]
          }
        ]
      '''.trim(),
        ),
      );

      final ByteBuffer robotoData = await httpFetchByteBuffer('/assets/fonts/Roboto-Regular.ttf');

      await fontCollection.loadAssetFonts(await fetchFontManifest(fakeAssetManager));
      expect(warnings, isEmpty);

      // Use `singleWhere` to make sure only one version of 'Ahem' is loaded.
      final RegisteredFont ahem = fontCollection.debugRegisteredFonts!.singleWhere(
        (RegisteredFont font) => font.family == 'Ahem',
      );

      // Check that the contents of 'Ahem' is actually Roboto, because that's
      // what's specified in the manifest, and the manifest takes precedence.
      expect(ahem.bytes.length, robotoData.lengthInBytes);
    });

    test('falls back to default Ahem URL', () async {
      final SkiaFontCollection fontCollection = renderer.fontCollection as SkiaFontCollection;

      final ByteBuffer ahemData = await httpFetchByteBuffer('/assets/fonts/ahem.ttf');

      // Use `singleWhere` to make sure only one version of 'Ahem' is loaded.
      final RegisteredFont ahem = fontCollection.debugRegisteredFonts!.singleWhere(
        (RegisteredFont font) => font.family == 'Ahem',
      );

      // Check that the contents of 'Ahem' is actually Roboto, because that's
      // what's specified in the manifest, and the manifest takes precedence.
      expect(ahem.bytes.length, ahemData.lengthInBytes);
    });

    test('FlutterTest is the default test font', () async {
      final SkiaFontCollection fontCollection = renderer.fontCollection as SkiaFontCollection;

      expect(fontCollection.debugRegisteredFonts, isNotEmpty);
      expect(fontCollection.debugRegisteredFonts!.first.family, 'FlutterTest');
    });
  });
}
