// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

import '../common/fake_asset_manager.dart';
import '../common/test_initialization.dart';
import 'utils.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests();

  late FakeAssetScope testScope;
  setUp(() {
    mockHttpFetchResponseFactory = null;
    testScope = fakeAssetManager.pushAssetScope();
  });

  tearDown(() {
    fakeAssetManager.popAssetScope(testScope);
  });

  test(
    'Loading valid font from data succeeds without family name (except in HTML renderer)',
    () async {
      final FlutterFontCollection collection = renderer.fontCollection;
      final ByteBuffer ahemData = await httpFetchByteBuffer('/assets/fonts/ahem.ttf');
      expect(
        await collection.loadFontFromList(ahemData.asUint8List()),
        !isHtml, // HtmlFontCollection requires family name
      );
    },
  );

  test('Loading valid font from data succeeds with family name', () async {
    final FlutterFontCollection collection = renderer.fontCollection;
    final ByteBuffer ahemData = await httpFetchByteBuffer('/assets/fonts/ahem.ttf');
    expect(
      await collection.loadFontFromList(ahemData.asUint8List(), fontFamily: 'FamilyName'),
      true,
    );
  });

  test('Loading invalid font from data returns false', () async {
    final FlutterFontCollection collection = renderer.fontCollection;
    final List<int> invalidFontData = utf8.encode('This is not valid font data');
    expect(
      await collection.loadFontFromList(
        Uint8List.fromList(invalidFontData),
        fontFamily: 'FamilyName',
      ),
      false,
    );
  });

  test('Loading valid asset fonts succeds', () async {
    testScope.setAssetPassthrough(robotoVariableFontUrl);
    testScope.setAssetPassthrough(robotoTestFontUrl);
    testScope.setAssetPassthrough(ahemFontUrl);

    final FlutterFontCollection collection = renderer.fontCollection;
    final AssetFontsResult result = await collection.loadAssetFonts(
      FontManifest(<FontFamily>[
        FontFamily(robotoFontFamily, <FontAsset>[
          FontAsset(robotoVariableFontUrl, <String, String>{}),
          FontAsset(robotoTestFontUrl, <String, String>{'weight': 'bold'}),
        ]),
        FontFamily(ahemFontFamily, <FontAsset>[FontAsset(ahemFontUrl, <String, String>{})]),
      ]),
    );
    expect(result.loadedFonts, <String>[robotoVariableFontUrl, robotoTestFontUrl, ahemFontUrl]);
    expect(result.fontFailures, isEmpty);
  });

  test('Loading asset fonts reports when font not found', () async {
    testScope.setAssetPassthrough(robotoVariableFontUrl);
    testScope.setAssetPassthrough(robotoTestFontUrl);

    const String invalidFontUrl = 'assets/invalid_font_url.ttf';

    final FlutterFontCollection collection = renderer.fontCollection;
    final AssetFontsResult result = await collection.loadAssetFonts(
      FontManifest(<FontFamily>[
        FontFamily(robotoFontFamily, <FontAsset>[
          FontAsset(robotoVariableFontUrl, <String, String>{}),
          FontAsset(robotoTestFontUrl, <String, String>{'weight': 'bold'}),
        ]),
        FontFamily(ahemFontFamily, <FontAsset>[FontAsset(invalidFontUrl, <String, String>{})]),
      ]),
    );
    expect(result.loadedFonts, <String>[robotoVariableFontUrl, robotoTestFontUrl]);
    expect(result.fontFailures, hasLength(1));
    if (isHtml) {
      // The HTML renderer doesn't have a way to differentiate 404's from other
      // download errors.
      expect(result.fontFailures[invalidFontUrl], isA<FontDownloadError>());
    } else {
      expect(result.fontFailures[invalidFontUrl], isA<FontNotFoundError>());
    }
  });

  test('Loading asset fonts reports when a font has invalid data', () async {
    const String invalidFontUrl = 'assets/invalid_font_data.ttf';

    testScope.setAssetPassthrough(robotoVariableFontUrl);
    testScope.setAssetPassthrough(robotoTestFontUrl);
    testScope.setAssetPassthrough(invalidFontUrl);

    mockHttpFetchResponseFactory = (String url) async {
      if (url == invalidFontUrl) {
        return MockHttpFetchResponse(
          url: url,
          status: 200,
          payload: MockHttpFetchPayload(
            byteBuffer: stringAsUtf8Data('this is invalid data').buffer,
          ),
        );
      }
      return null;
    };

    final FlutterFontCollection collection = renderer.fontCollection;
    final AssetFontsResult result = await collection.loadAssetFonts(
      FontManifest(<FontFamily>[
        FontFamily(robotoFontFamily, <FontAsset>[
          FontAsset(robotoVariableFontUrl, <String, String>{}),
          FontAsset(robotoTestFontUrl, <String, String>{'weight': 'bold'}),
        ]),
        FontFamily(ahemFontFamily, <FontAsset>[FontAsset(invalidFontUrl, <String, String>{})]),
      ]),
    );
    expect(result.loadedFonts, <String>[robotoVariableFontUrl, robotoTestFontUrl]);
    expect(result.fontFailures, hasLength(1));
    if (isHtml) {
      // The HTML renderer doesn't have a way to differentiate invalid data
      // from other download errors.
      expect(result.fontFailures[invalidFontUrl], isA<FontDownloadError>());
    } else {
      expect(result.fontFailures[invalidFontUrl], isA<FontInvalidDataError>());
    }
  });

  test('Font manifest with numeric and string descriptor values parses correctly', () async {
    testScope.setAsset(
      'FontManifest.json',
      stringAsUtf8Data(r'''
[
  {
    "family": "FakeFont",
    "fonts": [
      {
        "asset": "fonts/FakeFont.ttf",
        "style": "italic",
        "weight": 400
      }
    ]
  }
]
'''),
    );
    final FontManifest manifest = await fetchFontManifest(fakeAssetManager);
    expect(manifest.families.length, 1);

    final FontFamily family = manifest.families.single;
    expect(family.name, 'FakeFont');
    expect(family.fontAssets.length, 1);

    final FontAsset fontAsset = family.fontAssets.single;
    expect(fontAsset.asset, 'fonts/FakeFont.ttf');
    expect(fontAsset.descriptors.length, 2);
    expect(fontAsset.descriptors['style'], 'italic');
    expect(fontAsset.descriptors['weight'], '400');
  });
}
