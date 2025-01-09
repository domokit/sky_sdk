// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../../common/fake_asset_manager.dart';
import '../../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('$HtmlFontCollection', () {
    setUpUnitTests();

    const String testFontUrl = '/assets/fonts/ahem.ttf';

    late FakeAssetScope testScope;
    setUp(() {
      testScope = fakeAssetManager.pushAssetScope();
      testScope.setAssetPassthrough(testFontUrl);

      // Clear the fonts before the test begins to wipe out the fonts from the
      // test initialization.
      domDocument.fonts!.clear();
    });

    tearDown(() {
      fakeAssetManager.popAssetScope(testScope);
    });

    group('regular special characters', () {
      test('Register Asset with no special characters', () async {
        const String testFontFamily = 'Ahem';
        final List<String> fontFamilyList = <String>[];
        final HtmlFontCollection collection = HtmlFontCollection();
        await collection.loadAssetFonts(
          FontManifest(<FontFamily>[
            FontFamily(testFontFamily, <FontAsset>[FontAsset(testFontUrl, <String, String>{})]),
          ]),
        );
        domDocument.fonts!.forEach((DomFontFace f, DomFontFace f2, DomFontFaceSet s) {
          fontFamilyList.add(f.family!);
        });

        expect(fontFamilyList.length, equals(1));
        expect(fontFamilyList.first, 'Ahem');
      });

      test(
        'Register Asset with white space in the family name',
        () async {
          const String testFontFamily = 'Ahem ahem ahem';
          final List<String> fontFamilyList = <String>[];

          final HtmlFontCollection collection = HtmlFontCollection();
          await collection.loadAssetFonts(
            FontManifest(<FontFamily>[
              FontFamily(testFontFamily, <FontAsset>[FontAsset(testFontUrl, <String, String>{})]),
            ]),
          );
          domDocument.fonts!.forEach((DomFontFace f, DomFontFace f2, DomFontFaceSet s) {
            fontFamilyList.add(f.family!);
          });

          expect(fontFamilyList.length, equals(1));
          expect(fontFamilyList.first, 'Ahem ahem ahem');
        },
        // TODO(mdebbar): https://github.com/flutter/flutter/issues/51142
        skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit,
      );

      test('Register Asset with capital case letters', () async {
        const String testFontFamily = 'AhEm';
        final List<String> fontFamilyList = <String>[];

        final HtmlFontCollection collection = HtmlFontCollection();
        await collection.loadAssetFonts(
          FontManifest(<FontFamily>[
            FontFamily(testFontFamily, <FontAsset>[FontAsset(testFontUrl, <String, String>{})]),
          ]),
        );
        domDocument.fonts!.forEach((DomFontFace f, DomFontFace f2, DomFontFaceSet s) {
          fontFamilyList.add(f.family!);
        });

        expect(fontFamilyList.length, equals(1));
        expect(fontFamilyList.first, 'AhEm');
      });

      test('Register Asset with descriptor', () async {
        const String testFontFamily = 'Ahem';
        final List<String> fontFamilyList = <String>[];
        final HtmlFontCollection collection = HtmlFontCollection();
        await collection.loadAssetFonts(
          FontManifest(<FontFamily>[
            FontFamily(testFontFamily, <FontAsset>[
              FontAsset(testFontUrl, <String, String>{'weight': 'bold'}),
            ]),
          ]),
        );

        domDocument.fonts!.forEach((DomFontFace f, DomFontFace f2, DomFontFaceSet s) {
          expect(f.weight, 'bold');
          expect(f2.weight, 'bold');
          fontFamilyList.add(f.family!);
        });

        expect(fontFamilyList.length, equals(1));
        expect(fontFamilyList.first, 'Ahem');
      });
    });

    group('fonts with special characters', () {
      test(
        'Register Asset twice with special character slash',
        () async {
          const String testFontFamily = '/Ahem';
          final List<String> fontFamilyList = <String>[];

          final HtmlFontCollection collection = HtmlFontCollection();
          await collection.loadAssetFonts(
            FontManifest(<FontFamily>[
              FontFamily(testFontFamily, <FontAsset>[FontAsset(testFontUrl, <String, String>{})]),
            ]),
          );
          domDocument.fonts!.forEach((DomFontFace f, DomFontFace f2, DomFontFaceSet s) {
            fontFamilyList.add(f.family!);
          });

          if (ui_web.browser.browserEngine != ui_web.BrowserEngine.firefox) {
            expect(fontFamilyList.length, equals(2));
            expect(fontFamilyList, contains("'/Ahem'"));
            expect(fontFamilyList, contains('/Ahem'));
          } else {
            expect(fontFamilyList.length, equals(1));
            expect(fontFamilyList.first, '"/Ahem"');
          }
        },
        // TODO(mdebbar): https://github.com/flutter/flutter/issues/51142
        skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit,
      );

      test(
        'Register Asset twice with exclamation mark',
        () async {
          const String testFontFamily = 'Ahem!!ahem';
          final List<String> fontFamilyList = <String>[];

          final HtmlFontCollection collection = HtmlFontCollection();
          await collection.loadAssetFonts(
            FontManifest(<FontFamily>[
              FontFamily(testFontFamily, <FontAsset>[FontAsset(testFontUrl, <String, String>{})]),
            ]),
          );
          domDocument.fonts!.forEach((DomFontFace f, DomFontFace f2, DomFontFaceSet s) {
            fontFamilyList.add(f.family!);
          });

          if (ui_web.browser.browserEngine != ui_web.BrowserEngine.firefox) {
            expect(fontFamilyList.length, equals(2));
            expect(fontFamilyList, contains("'Ahem!!ahem'"));
            expect(fontFamilyList, contains('Ahem!!ahem'));
          } else {
            expect(fontFamilyList.length, equals(1));
            expect(fontFamilyList.first, '"Ahem!!ahem"');
          }
        },
        // TODO(mdebbar): https://github.com/flutter/flutter/issues/51142
        skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit,
      );

      test(
        'Register Asset twice with comma',
        () async {
          const String testFontFamily = 'Ahem ,ahem';
          final List<String> fontFamilyList = <String>[];

          final HtmlFontCollection collection = HtmlFontCollection();
          await collection.loadAssetFonts(
            FontManifest(<FontFamily>[
              FontFamily(testFontFamily, <FontAsset>[FontAsset(testFontUrl, <String, String>{})]),
            ]),
          );
          domDocument.fonts!.forEach((DomFontFace f, DomFontFace f2, DomFontFaceSet s) {
            fontFamilyList.add(f.family!);
          });

          if (ui_web.browser.browserEngine != ui_web.BrowserEngine.firefox) {
            expect(fontFamilyList.length, equals(2));
            expect(fontFamilyList, contains("'Ahem ,ahem'"));
            expect(fontFamilyList, contains('Ahem ,ahem'));
          } else {
            expect(fontFamilyList.length, equals(1));
            expect(fontFamilyList.first, '"Ahem ,ahem"');
          }
        },
        // TODO(mdebbar): https://github.com/flutter/flutter/issues/51142
        skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit,
      );

      test(
        'Register Asset twice with a digit at the start of a token',
        () async {
          const String testFontFamily = 'Ahem 1998';
          final List<String> fontFamilyList = <String>[];

          final HtmlFontCollection collection = HtmlFontCollection();
          await collection.loadAssetFonts(
            FontManifest(<FontFamily>[
              FontFamily(testFontFamily, <FontAsset>[FontAsset(testFontUrl, <String, String>{})]),
            ]),
          );
          domDocument.fonts!.forEach((DomFontFace f, DomFontFace f2, DomFontFaceSet s) {
            fontFamilyList.add(f.family!);
          });

          if (ui_web.browser.browserEngine != ui_web.BrowserEngine.firefox) {
            expect(fontFamilyList.length, equals(2));
            expect(fontFamilyList, contains('Ahem 1998'));
            expect(fontFamilyList, contains("'Ahem 1998'"));
          } else {
            expect(fontFamilyList.length, equals(1));
            expect(fontFamilyList.first, '"Ahem 1998"');
          }
        },
        // TODO(mdebbar): https://github.com/flutter/flutter/issues/51142
        skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit,
      );
    });
  });
}
