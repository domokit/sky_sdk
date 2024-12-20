// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:fake_async/fake_async.dart';
///
/// @docImport 'test_compat.dart';
/// @docImport 'widget_tester.dart';
library;

import 'dart:async';
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'test_async_utils.dart';

final Map<int, ui.Image> _cache = <int, ui.Image>{};

/// Creates an arbitrarily sized image for testing.
///
/// If the [cache] parameter is set to true, the image will be cached for the
/// rest of this suite. This is normally desirable, assuming a test suite uses
/// images with the same dimensions in most tests, as it will save on memory
/// usage and CPU time over the course of the suite. However, it should be
/// avoided for images that are used only once in a test suite, especially if
/// the image is large, as it will require holding on to the memory for that
/// image for the duration of the suite.
///
/// This method requires real async work, and will not work properly in the
/// [FakeAsync] zones set up by [testWidgets]. Typically, it should be invoked
/// as a setup step before [testWidgets] are run, such as [setUp] or [setUpAll].
/// If needed, it can be invoked using [WidgetTester.runAsync].
Future<ui.Image> createTestImage({int width = 1, int height = 1, bool cache = true}) =>
    TestAsyncUtils.guard(() async {
      assert(width > 0);
      assert(height > 0);

      final int cacheKey = Object.hash(width, height);
      if (cache && _cache.containsKey(cacheKey)) {
        return _cache[cacheKey]!.clone();
      }

      final ui.Image image = await _createImage(width, height);
      if (cache) {
        _cache[cacheKey] = image.clone();
      }
      return image;
    });

Future<ui.Image> _createImage(int width, int height) async {
  final Completer<ui.Image> completer = Completer<ui.Image>();
  ui.decodeImageFromPixels(
    Uint8List.fromList(List<int>.filled(width * height * 4, 0)),
    width,
    height,
    ui.PixelFormat.rgba8888,
    (ui.Image image) {
      completer.complete(image);
    },
  );
  return completer.future;
}
