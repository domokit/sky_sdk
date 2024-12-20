// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:js_interop';

import 'package:test/test.dart';
// ignore: implementation_imports
import 'package:ui/src/engine.dart' show renderer;
// ignore: implementation_imports
import 'package:ui/src/engine/dom.dart';
import 'package:ui/ui.dart';

Future<dynamic> _callScreenshotServer(dynamic requestData) async {
  // This is test code, but because the file name doesn't end with "_test.dart"
  // the analyzer doesn't know it, so have to ignore the lint explicitly.
  // ignore: invalid_use_of_visible_for_testing_member
  final HttpFetchResponse response = await testOnlyHttpPost('screenshot', json.encode(requestData));
  return json.decode(await response.text());
}

/// How to compare pixels within the image.
///
/// Keep this enum in sync with the one defined in `goldens.dart`.
enum PixelComparison {
  /// Allows minor blur and anti-aliasing differences by comparing a 3x3 grid
  /// surrounding the pixel rather than direct 1:1 comparison.
  fuzzy,

  /// Compares one pixel at a time.
  ///
  /// Anti-aliasing or blur will result in higher diff rate.
  precise,
}

/// Attempts to match the current browser state with the screenshot [filename].
///
/// If [region] is not null, the golden will only include the part contained by
/// the rectangle.
///
/// [maxDiffRate] specifies the tolerance to the number of non-matching pixels
/// before the test is considered as failing. If [maxDiffRate] is null, applies
/// a default value defined in `test_platform.dart`.
///
/// [pixelComparison] determines the algorithm used to compare pixels. Uses
/// fuzzy comparison by default.
Future<void> matchGoldenFile(String filename, {Rect? region}) async {
  // It is difficult to deterministically tell when rendered content is actually
  // visible to the user, so we pump 15 frames to make sure that the content is
  // has reached the screen. This is at the recommendation of the Chrome team,
  // and they use this same thing in their screenshot unit tests.
  for (int i = 0; i < 15; i++) {
    await awaitNextFrame();
  }

  if (!filename.endsWith('.png')) {
    throw ArgumentError('Filename must end in .png or SkiaGold will ignore it.');
  }
  final Map<String, dynamic> serverParams = <String, dynamic>{
    'filename': filename,
    'region':
        region == null
            ? null
            : <String, dynamic>{
              'x': region.left,
              'y': region.top,
              'width': region.width,
              'height': region.height,
            },
    // We use the renderer tag here rather than `renderer is CanvasKitRenderer`
    // because these unit tests operate on the post-transformed (sdk_rewriter)
    // sdk where the internal classes like `CanvasKitRenderer` are no longer
    // visible.
    'isCanvaskitTest': renderer.rendererTag == 'canvaskit',
  };

  final String response = await _callScreenshotServer(serverParams) as String;
  if (response == 'OK') {
    // Pass
    return;
  }
  fail(response);
}

/// Waits for one frame to complete rendering
Future<void> awaitNextFrame() {
  final Completer<void> completer = Completer<void>();
  domWindow.requestAnimationFrame((JSNumber time) => completer.complete());
  return completer.future;
}
