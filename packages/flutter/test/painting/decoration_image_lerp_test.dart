// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines because it contains golden tests; see:
// https://github.com/flutter/flutter/blob/main/docs/contributing/testing/Writing-a-golden-file-test-for-package-flutter.md
@Tags(<String>['reduced-test-set'])
library;

import 'dart:async';
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets(
    'ImageDecoration.lerp 1',
    (WidgetTester tester) async {
      final MemoryImage green = MemoryImage(
        Uint8List.fromList(<int>[
          0x89,
          0x50,
          0x4e,
          0x47,
          0x0d,
          0x0a,
          0x1a,
          0x0a,
          0x00,
          0x00,
          0x00,
          0x0d,
          0x49,
          0x48,
          0x44,
          0x52,
          0x00,
          0x00,
          0x00,
          0x01,
          0x00,
          0x00,
          0x00,
          0x01,
          0x01,
          0x03,
          0x00,
          0x00,
          0x00,
          0x25,
          0xdb,
          0x56,
          0xca,
          0x00,
          0x00,
          0x00,
          0x03,
          0x50,
          0x4c,
          0x54,
          0x45,
          0x00,
          0xff,
          0x00,
          0x34,
          0x5e,
          0xc0,
          0xa8,
          0x00,
          0x00,
          0x00,
          0x0a,
          0x49,
          0x44,
          0x41,
          0x54,
          0x08,
          0xd7,
          0x63,
          0x60,
          0x00,
          0x00,
          0x00,
          0x02,
          0x00,
          0x01,
          0xe2,
          0x21,
          0xbc,
          0x33,
          0x00,
          0x00,
          0x00,
          0x00,
          0x49,
          0x45,
          0x4e,
          0x44,
          0xae,
          0x42,
          0x60,
          0x82,
        ]),
      );
      final MemoryImage red = MemoryImage(
        Uint8List.fromList(<int>[
          0x89,
          0x50,
          0x4e,
          0x47,
          0x0d,
          0x0a,
          0x1a,
          0x0a,
          0x00,
          0x00,
          0x00,
          0x0d,
          0x49,
          0x48,
          0x44,
          0x52,
          0x00,
          0x00,
          0x00,
          0x01,
          0x00,
          0x00,
          0x00,
          0x01,
          0x01,
          0x03,
          0x00,
          0x00,
          0x00,
          0x25,
          0xdb,
          0x56,
          0xca,
          0x00,
          0x00,
          0x00,
          0x03,
          0x50,
          0x4c,
          0x54,
          0x45,
          0xff,
          0x00,
          0x00,
          0x19,
          0xe2,
          0x09,
          0x37,
          0x00,
          0x00,
          0x00,
          0x0a,
          0x49,
          0x44,
          0x41,
          0x54,
          0x08,
          0xd7,
          0x63,
          0x60,
          0x00,
          0x00,
          0x00,
          0x02,
          0x00,
          0x01,
          0xe2,
          0x21,
          0xbc,
          0x33,
          0x00,
          0x00,
          0x00,
          0x00,
          0x49,
          0x45,
          0x4e,
          0x44,
          0xae,
          0x42,
          0x60,
          0x82,
        ]),
      );

      late final _ImageLoader greenLoader = _ImageLoader(green);
      late final _ImageLoader redLoader = _ImageLoader(red);

      await tester.runAsync(() async {
        await greenLoader.load();
        await redLoader.load();
      });

      await tester.pumpWidget(
        ColoredBox(
          color: Colors.white,
          child: Align(
            alignment: Alignment.topLeft,
            child: RepaintBoundary(
              child: Wrap(
                textDirection: TextDirection.ltr,
                children: <Widget>[
                  _TestImage(DecorationImage(image: green, repeat: ImageRepeat.repeat)),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: green, repeat: ImageRepeat.repeat),
                      DecorationImage(image: red, repeat: ImageRepeat.repeat),
                      0.0,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: green, repeat: ImageRepeat.repeat),
                      DecorationImage(image: red, repeat: ImageRepeat.repeat),
                      0.1,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: green, repeat: ImageRepeat.repeat),
                      DecorationImage(image: red, repeat: ImageRepeat.repeat),
                      0.2,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: green, repeat: ImageRepeat.repeat),
                      DecorationImage(image: red, repeat: ImageRepeat.repeat),
                      0.5,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: green, repeat: ImageRepeat.repeat),
                      DecorationImage(image: red, repeat: ImageRepeat.repeat),
                      0.8,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: green, repeat: ImageRepeat.repeat),
                      DecorationImage(image: red, repeat: ImageRepeat.repeat),
                      0.9,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: green, repeat: ImageRepeat.repeat),
                      DecorationImage(image: red, repeat: ImageRepeat.repeat),
                      1.0,
                    ),
                  ),
                  _TestImage(DecorationImage(image: red, repeat: ImageRepeat.repeat)),
                  for (double t = 0.0; t < 1.0; t += 0.125)
                    _TestImage(
                      DecorationImage.lerp(
                        DecorationImage.lerp(
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          t,
                        ),
                        DecorationImage.lerp(
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          t,
                        ),
                        t,
                      ),
                    ),
                  for (double t = 0.0; t < 1.0; t += 0.125)
                    _TestImage(
                      DecorationImage.lerp(
                        DecorationImage.lerp(
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          1.0 - t,
                        ),
                        DecorationImage.lerp(
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          t,
                        ),
                        t,
                      ),
                    ),
                  for (double t = 0.0; t < 1.0; t += 0.125)
                    _TestImage(
                      DecorationImage.lerp(
                        DecorationImage.lerp(
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          t,
                        ),
                        DecorationImage.lerp(
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          1.0 - t,
                        ),
                        t,
                      ),
                    ),
                  for (double t = 0.0; t < 1.0; t += 0.125)
                    _TestImage(
                      DecorationImage.lerp(
                        DecorationImage.lerp(
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          1.0 - t,
                        ),
                        DecorationImage.lerp(
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          DecorationImage(image: green, repeat: ImageRepeat.repeat),
                          1.0 - t,
                        ),
                        t,
                      ),
                    ),
                ],
              ),
            ),
          ),
        ),
      );

      await expectLater(find.byType(Wrap), matchesGoldenFile('decoration_image.lerp.0.png'));

      if (!kIsWeb) {
        // TODO(ianh): https://github.com/flutter/flutter/issues/130610
        final ui.Image image =
            (await tester.binding.runAsync<ui.Image>(
              () => captureImage(find.byType(Wrap).evaluate().single),
            ))!;
        addTearDown(() => image.dispose());
        final Uint8List bytes =
            (await tester.binding.runAsync<ByteData?>(
              () => image.toByteData(format: ui.ImageByteFormat.rawStraightRgba),
            ))!.buffer.asUint8List();
        expect(image.width, 792);
        expect(image.height, 48);
        expect(bytes, hasLength(image.width * image.height * 4));
        Color getPixel(int x, int y) {
          final int offset = (x + y * image.width) * 4;
          return Color.fromARGB(0xFF, bytes[offset], bytes[offset + 1], bytes[offset + 2]);
        }

        Color getBlockPixel(int index) {
          int x = 12 + index * 24;
          final int y = 12 + (x ~/ image.width) * 24;
          x %= image.width;
          return getPixel(x, y);
        }

        const Color lime = Color(0xFF00FF00);
        expect(getBlockPixel(0), isSameColorAs(lime)); // pure green
        expect(getBlockPixel(1), isSameColorAs(lime)); // 100% green 0% red
        expect(getBlockPixel(2), isSameColorAs(const Color(0xFF19E600)));
        expect(getBlockPixel(3), isSameColorAs(const Color(0xFF33CC00)));
        expect(getBlockPixel(4), isSameColorAs(const Color(0xFF808000))); // 50-50 mix green/red
        expect(getBlockPixel(5), isSameColorAs(const Color(0xFFCD3200)));
        expect(getBlockPixel(6), isSameColorAs(const Color(0xFFE61900)));
        expect(getBlockPixel(7), isSameColorAs(const Color(0xFFFF0000))); // 0% green 100% red
        expect(getBlockPixel(8), isSameColorAs(const Color(0xFFFF0000))); // pure red
        for (int index = 9; index < 40; index += 1) {
          expect(getBlockPixel(index), isSameColorAs(lime));
        }
      }

      greenLoader.dispose();
      redLoader.dispose();
      imageCache.clear();
    },
    // TODO(ianh): https://github.com/flutter/flutter/issues/130612, https://github.com/flutter/flutter/issues/130609
    skip: kIsWeb,
  );

  testWidgets(
    'ImageDecoration.lerp 2',
    (WidgetTester tester) async {
      final MemoryImage cmyk = MemoryImage(
        Uint8List.fromList(<int>[
          0x89,
          0x50,
          0x4e,
          0x47,
          0x0d,
          0x0a,
          0x1a,
          0x0a,
          0x00,
          0x00,
          0x00,
          0x0d,
          0x49,
          0x48,
          0x44,
          0x52,
          0x00,
          0x00,
          0x00,
          0x04,
          0x00,
          0x00,
          0x00,
          0x04,
          0x02,
          0x03,
          0x00,
          0x00,
          0x00,
          0xd4,
          0x9f,
          0x76,
          0xed,
          0x00,
          0x00,
          0x00,
          0x0c,
          0x50,
          0x4c,
          0x54,
          0x45,
          0x00,
          0xff,
          0xff,
          0xff,
          0x00,
          0xff,
          0xff,
          0xff,
          0x00,
          0x00,
          0x00,
          0x00,
          0x3b,
          0x4c,
          0x59,
          0x13,
          0x00,
          0x00,
          0x00,
          0x0e,
          0x49,
          0x44,
          0x41,
          0x54,
          0x08,
          0xd7,
          0x63,
          0x60,
          0x05,
          0xc2,
          0xf5,
          0x0c,
          0xeb,
          0x01,
          0x03,
          0x00,
          0x01,
          0x69,
          0x19,
          0xea,
          0x34,
          0x7b,
          0x00,
          0x00,
          0x00,
          0x00,
          0x49,
          0x45,
          0x4e,
          0x44,
          0xae,
          0x42,
          0x60,
          0x82,
        ]),
      );
      final MemoryImage wrgb = MemoryImage(
        Uint8List.fromList(<int>[
          0x89,
          0x50,
          0x4e,
          0x47,
          0x0d,
          0x0a,
          0x1a,
          0x0a,
          0x00,
          0x00,
          0x00,
          0x0d,
          0x49,
          0x48,
          0x44,
          0x52,
          0x00,
          0x00,
          0x00,
          0x04,
          0x00,
          0x00,
          0x00,
          0x04,
          0x02,
          0x03,
          0x00,
          0x00,
          0x00,
          0xd4,
          0x9f,
          0x76,
          0xed,
          0x00,
          0x00,
          0x00,
          0x0c,
          0x50,
          0x4c,
          0x54,
          0x45,
          0xff,
          0xff,
          0xff,
          0x00,
          0x00,
          0xff,
          0x00,
          0xff,
          0x00,
          0xff,
          0x00,
          0x00,
          0x1e,
          0x46,
          0xbb,
          0x1c,
          0x00,
          0x00,
          0x00,
          0x0e,
          0x49,
          0x44,
          0x41,
          0x54,
          0x08,
          0xd7,
          0x63,
          0xe0,
          0x07,
          0xc2,
          0xa5,
          0x0c,
          0x4b,
          0x01,
          0x03,
          0x50,
          0x01,
          0x69,
          0x4a,
          0x78,
          0x1d,
          0x41,
          0x00,
          0x00,
          0x00,
          0x00,
          0x49,
          0x45,
          0x4e,
          0x44,
          0xae,
          0x42,
          0x60,
          0x82,
        ]),
      );

      late final _ImageLoader cmykLoader = _ImageLoader(cmyk);
      late final _ImageLoader wrgbLoader = _ImageLoader(wrgb);

      await tester.runAsync(() async {
        await cmykLoader.load();
        await wrgbLoader.load();
      });

      await tester.pumpWidget(
        ColoredBox(
          color: Colors.white,
          child: Align(
            alignment: Alignment.topLeft,
            child: RepaintBoundary(
              child: Wrap(
                textDirection: TextDirection.ltr,
                children: <Widget>[
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, fit: BoxFit.contain),
                      DecorationImage(image: cmyk, fit: BoxFit.contain),
                      0.0,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, fit: BoxFit.contain),
                      DecorationImage(image: cmyk, fit: BoxFit.contain),
                      0.1,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, fit: BoxFit.contain),
                      DecorationImage(image: cmyk, fit: BoxFit.contain),
                      0.2,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, fit: BoxFit.contain),
                      DecorationImage(image: cmyk, fit: BoxFit.contain),
                      0.5,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, fit: BoxFit.contain),
                      DecorationImage(image: cmyk, fit: BoxFit.contain),
                      0.8,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, fit: BoxFit.contain),
                      DecorationImage(image: cmyk, fit: BoxFit.contain),
                      0.9,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, fit: BoxFit.contain),
                      DecorationImage(image: cmyk, fit: BoxFit.contain),
                      1.0,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, fit: BoxFit.cover),
                      DecorationImage(image: cmyk, repeat: ImageRepeat.repeat),
                      0.5,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, repeat: ImageRepeat.repeat),
                      DecorationImage(image: cmyk, repeat: ImageRepeat.repeatY),
                      0.5,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, repeat: ImageRepeat.repeatX),
                      DecorationImage(image: cmyk, repeat: ImageRepeat.repeat),
                      0.5,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, repeat: ImageRepeat.repeat, opacity: 0.2),
                      DecorationImage(image: cmyk, repeat: ImageRepeat.repeat, opacity: 0.2),
                      0.25,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, repeat: ImageRepeat.repeat, opacity: 0.2),
                      DecorationImage(image: cmyk, repeat: ImageRepeat.repeat, opacity: 0.2),
                      0.5,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, repeat: ImageRepeat.repeat, opacity: 0.2),
                      DecorationImage(image: cmyk, repeat: ImageRepeat.repeat, opacity: 0.2),
                      0.75,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(image: wrgb, scale: 0.5, repeat: ImageRepeat.repeatX),
                      DecorationImage(image: cmyk, scale: 0.25, repeat: ImageRepeat.repeatY),
                      0.5,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      0.0,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      0.25,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      0.5,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      0.75,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      1.0,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(0.0, 0.0, 1.0, 1.0),
                      ),
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      0.0,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(0.0, 0.0, 1.0, 1.0),
                      ),
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      0.25,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(0.0, 0.0, 1.0, 1.0),
                      ),
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      0.5,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(0.0, 0.0, 1.0, 1.0),
                      ),
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      0.75,
                    ),
                  ),
                  _TestImage(
                    DecorationImage.lerp(
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(0.0, 0.0, 1.0, 1.0),
                      ),
                      DecorationImage(
                        image: cmyk,
                        centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                      ),
                      1.0,
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      await expectLater(find.byType(Wrap), matchesGoldenFile('decoration_image.lerp.1.png'));

      if (!kIsWeb) {
        // TODO(ianh): https://github.com/flutter/flutter/issues/130610
        final ui.Image image =
            (await tester.binding.runAsync<ui.Image>(
              () => captureImage(find.byType(Wrap).evaluate().single),
            ))!;
        addTearDown(() => image.dispose());
        final Uint8List bytes =
            (await tester.binding.runAsync<ByteData?>(
              () => image.toByteData(format: ui.ImageByteFormat.rawStraightRgba),
            ))!.buffer.asUint8List();
        expect(image.width, 24 * 24);
        expect(image.height, 1 * 24);
        expect(bytes, hasLength(image.width * image.height * 4));
        Color getPixel(int x, int y) {
          final int offset = (x + y * image.width) * 4;
          return Color.fromARGB(0xFF, bytes[offset], bytes[offset + 1], bytes[offset + 2]);
        }

        Color getPixelFromBlock(int index, int dx, int dy) {
          const int padding = 2;
          int x = index * 24 + dx + padding;
          final int y = (x ~/ image.width) * 24 + dy + padding;
          x %= image.width;
          return getPixel(x, y);
        }

        // wrgb image
        expect(getPixelFromBlock(0, 5, 5), const Color(0xFFFFFFFF));
        expect(getPixelFromBlock(0, 15, 5), const Color(0xFFFF0000));
        expect(getPixelFromBlock(0, 5, 15), const Color(0xFF00FF00));
        expect(getPixelFromBlock(0, 15, 15), const Color(0xFF0000FF));
        // wrgb/cmyk 50/50 blended image
        expect(getPixelFromBlock(3, 5, 5), const Color(0xFF80FFFF));
        expect(getPixelFromBlock(3, 15, 5), const Color(0xFFFF0080));
        expect(getPixelFromBlock(3, 5, 15), const Color(0xFF80FF00));
        expect(getPixelFromBlock(3, 15, 15), const Color(0xFF000080));
        // cmyk image
        expect(getPixelFromBlock(6, 5, 5), const Color(0xFF00FFFF));
        expect(getPixelFromBlock(6, 15, 5), const Color(0xFFFF00FF));
        expect(getPixelFromBlock(6, 5, 15), const Color(0xFFFFFF00));
        expect(getPixelFromBlock(6, 15, 15), const Color(0xFF000000));
        // top left corner control
        expect(getPixelFromBlock(14, 0, 0), const Color(0xFF00FFFF));
        expect(getPixelFromBlock(14, 1, 1), const Color(0xFF00FFFF));
        expect(getPixelFromBlock(14, 2, 0), const Color(0xFFFF00FF));
        expect(getPixelFromBlock(14, 19, 0), const Color(0xFFFF00FF));
        expect(getPixelFromBlock(14, 0, 2), const Color(0xFFFFFF00));
        expect(getPixelFromBlock(14, 0, 19), const Color(0xFFFFFF00));
        expect(getPixelFromBlock(14, 2, 2), const Color(0xFF000000));
        expect(getPixelFromBlock(14, 19, 19), const Color(0xFF000000));
        // bottom right corner control
        expect(getPixelFromBlock(19, 0, 0), const Color(0xFF00FFFF));
        expect(getPixelFromBlock(19, 17, 17), const Color(0xFF00FFFF));
        expect(getPixelFromBlock(19, 19, 0), const Color(0xFFFF00FF));
        expect(getPixelFromBlock(19, 19, 17), const Color(0xFFFF00FF));
        expect(getPixelFromBlock(19, 0, 19), const Color(0xFFFFFF00));
        expect(getPixelFromBlock(19, 17, 19), const Color(0xFFFFFF00));
        expect(getPixelFromBlock(19, 18, 18), const Color(0xFF000000));
        expect(getPixelFromBlock(19, 19, 19), const Color(0xFF000000));
      }

      cmykLoader.dispose();
      wrgbLoader.dispose();
      imageCache.clear();
    },
    // TODO(ianh): https://github.com/flutter/flutter/issues/130612, https://github.com/flutter/flutter/issues/130609
    skip: kIsWeb,
  );

  testWidgets(
    'ImageDecoration.lerp with colored background',
    (WidgetTester tester) async {
      final MemoryImage cmyk = MemoryImage(
        Uint8List.fromList(<int>[
          0x89,
          0x50,
          0x4e,
          0x47,
          0x0d,
          0x0a,
          0x1a,
          0x0a,
          0x00,
          0x00,
          0x00,
          0x0d,
          0x49,
          0x48,
          0x44,
          0x52,
          0x00,
          0x00,
          0x00,
          0x04,
          0x00,
          0x00,
          0x00,
          0x04,
          0x02,
          0x03,
          0x00,
          0x00,
          0x00,
          0xd4,
          0x9f,
          0x76,
          0xed,
          0x00,
          0x00,
          0x00,
          0x0c,
          0x50,
          0x4c,
          0x54,
          0x45,
          0x00,
          0xff,
          0xff,
          0xff,
          0x00,
          0xff,
          0xff,
          0xff,
          0x00,
          0x00,
          0x00,
          0x00,
          0x3b,
          0x4c,
          0x59,
          0x13,
          0x00,
          0x00,
          0x00,
          0x0e,
          0x49,
          0x44,
          0x41,
          0x54,
          0x08,
          0xd7,
          0x63,
          0x60,
          0x05,
          0xc2,
          0xf5,
          0x0c,
          0xeb,
          0x01,
          0x03,
          0x00,
          0x01,
          0x69,
          0x19,
          0xea,
          0x34,
          0x7b,
          0x00,
          0x00,
          0x00,
          0x00,
          0x49,
          0x45,
          0x4e,
          0x44,
          0xae,
          0x42,
          0x60,
          0x82,
        ]),
      );
      final MemoryImage wrgb = MemoryImage(
        Uint8List.fromList(<int>[
          0x89,
          0x50,
          0x4e,
          0x47,
          0x0d,
          0x0a,
          0x1a,
          0x0a,
          0x00,
          0x00,
          0x00,
          0x0d,
          0x49,
          0x48,
          0x44,
          0x52,
          0x00,
          0x00,
          0x00,
          0x04,
          0x00,
          0x00,
          0x00,
          0x04,
          0x02,
          0x03,
          0x00,
          0x00,
          0x00,
          0xd4,
          0x9f,
          0x76,
          0xed,
          0x00,
          0x00,
          0x00,
          0x0c,
          0x50,
          0x4c,
          0x54,
          0x45,
          0xff,
          0xff,
          0xff,
          0x00,
          0x00,
          0xff,
          0x00,
          0xff,
          0x00,
          0xff,
          0x00,
          0x00,
          0x1e,
          0x46,
          0xbb,
          0x1c,
          0x00,
          0x00,
          0x00,
          0x0e,
          0x49,
          0x44,
          0x41,
          0x54,
          0x08,
          0xd7,
          0x63,
          0xe0,
          0x07,
          0xc2,
          0xa5,
          0x0c,
          0x4b,
          0x01,
          0x03,
          0x50,
          0x01,
          0x69,
          0x4a,
          0x78,
          0x1d,
          0x41,
          0x00,
          0x00,
          0x00,
          0x00,
          0x49,
          0x45,
          0x4e,
          0x44,
          0xae,
          0x42,
          0x60,
          0x82,
        ]),
      );

      late final _ImageLoader cmykLoader = _ImageLoader(cmyk);
      late final _ImageLoader wrgbLoader = _ImageLoader(wrgb);

      await tester.runAsync(() async {
        await cmykLoader.load();
        await wrgbLoader.load();
      });

      await tester.pumpWidget(
        ColoredBox(
          color: Colors.pink,
          child: Align(
            alignment: Alignment.topLeft,
            child: Wrap(
              textDirection: TextDirection.ltr,
              children: <Widget>[
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, fit: BoxFit.contain),
                    DecorationImage(image: cmyk, fit: BoxFit.contain),
                    0.0,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, fit: BoxFit.contain),
                    DecorationImage(image: cmyk, fit: BoxFit.contain),
                    0.1,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, fit: BoxFit.contain),
                    DecorationImage(image: cmyk, fit: BoxFit.contain),
                    0.2,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, fit: BoxFit.contain),
                    DecorationImage(image: cmyk, fit: BoxFit.contain),
                    0.5,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, fit: BoxFit.contain),
                    DecorationImage(image: cmyk, fit: BoxFit.contain),
                    0.8,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, fit: BoxFit.contain),
                    DecorationImage(image: cmyk, fit: BoxFit.contain),
                    0.9,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, fit: BoxFit.contain),
                    DecorationImage(image: cmyk, fit: BoxFit.contain),
                    1.0,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, fit: BoxFit.cover),
                    DecorationImage(image: cmyk, repeat: ImageRepeat.repeat),
                    0.5,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, repeat: ImageRepeat.repeat),
                    DecorationImage(image: cmyk, repeat: ImageRepeat.repeatY),
                    0.5,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, repeat: ImageRepeat.repeatX),
                    DecorationImage(image: cmyk, repeat: ImageRepeat.repeat),
                    0.5,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, repeat: ImageRepeat.repeat, opacity: 0.2),
                    DecorationImage(image: cmyk, repeat: ImageRepeat.repeat, opacity: 0.2),
                    0.25,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, repeat: ImageRepeat.repeat, opacity: 0.2),
                    DecorationImage(image: cmyk, repeat: ImageRepeat.repeat, opacity: 0.2),
                    0.5,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, repeat: ImageRepeat.repeat, opacity: 0.2),
                    DecorationImage(image: cmyk, repeat: ImageRepeat.repeat, opacity: 0.2),
                    0.75,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(image: wrgb, scale: 0.5, repeat: ImageRepeat.repeatX),
                    DecorationImage(image: cmyk, scale: 0.25, repeat: ImageRepeat.repeatY),
                    0.5,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    0.0,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    0.25,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    0.5,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    0.75,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    1.0,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(0.0, 0.0, 1.0, 1.0),
                    ),
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    0.0,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(0.0, 0.0, 1.0, 1.0),
                    ),
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    0.25,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(0.0, 0.0, 1.0, 1.0),
                    ),
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    0.5,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(0.0, 0.0, 1.0, 1.0),
                    ),
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    0.75,
                  ),
                ),
                _TestImage(
                  DecorationImage.lerp(
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(0.0, 0.0, 1.0, 1.0),
                    ),
                    DecorationImage(
                      image: cmyk,
                      centerSlice: const Rect.fromLTWH(2.0, 2.0, 1.0, 1.0),
                    ),
                    1.0,
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      await expectLater(find.byType(Wrap), matchesGoldenFile('decoration_image.lerp.2.png'));

      cmykLoader.dispose();
      wrgbLoader.dispose();
      imageCache.clear();
    },
    // TODO(ianh): https://github.com/flutter/flutter/issues/130612, https://github.com/flutter/flutter/issues/130609
    skip: kIsWeb,
  );
}

class _ImageLoader {
  _ImageLoader(this.image);

  late final MemoryImage image;
  late final ImageStream stream;
  final Completer<ImageInfo> _completer = Completer<ImageInfo>();
  late final ImageStreamListener wrappedListener;

  void _listener(ImageInfo image, bool syncCall) {
    _completer.complete(image);
    addTearDown(image.dispose);
  }

  Future<void> load() {
    stream = image.resolve(ImageConfiguration.empty);
    wrappedListener = ImageStreamListener(_listener);
    stream.addListener(wrappedListener);
    return _completer.future;
  }

  void dispose() {
    stream.removeListener(wrappedListener);
  }
}

class _TestImage extends StatelessWidget {
  const _TestImage(this.image);

  final DecorationImage? image;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(2.0),
      child: SizedBox(
        width: 20,
        height: 20,
        child: DecoratedBox(decoration: BoxDecoration(image: image)),
      ),
    );
  }
}
