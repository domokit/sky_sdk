// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';
import 'dart:typed_data';

import 'package:test/test.dart';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:web_engine_tester/golden_tester.dart';

import '../common/rendering.dart';
import '../common/test_initialization.dart';

export '../common/rendering.dart' show renderScene;

const MethodCodec codec = StandardMethodCodec();

/// Common test setup for all CanvasKit unit-tests.
void setUpCanvasKitTest({bool withImplicitView = false}) {
  setUpUnitTests(
    withImplicitView: withImplicitView,
    emulateTesterEnvironment: false,
    setUpTestViewDimensions: false,
  );

  setUp(
    () => debugOverrideJsConfiguration(
      <String, Object?>{'fontFallbackBaseUrl': 'assets/fallback_fonts/'}.jsify()
          as JsFlutterConfiguration?,
    ),
  );
}

/// Convenience getter for the implicit view.
EngineFlutterWindow get implicitView => EnginePlatformDispatcher.instance.implicitView!;

/// Utility function for CanvasKit tests to draw pictures without
/// the [CkPictureRecorder] boilerplate.
CkPicture paintPicture(ui.Rect cullRect, void Function(CkCanvas canvas) painter) {
  final CkPictureRecorder recorder = CkPictureRecorder();
  final CkCanvas canvas = recorder.beginRecording(cullRect);
  painter(canvas);
  return recorder.endRecording();
}

Future<void> matchSceneGolden(String goldenFile, ui.Scene scene, {required ui.Rect region}) async {
  await renderScene(scene);
  await matchGoldenFile(goldenFile, region: region);
}

/// Checks that a [picture] matches the [goldenFile].
///
/// The picture is drawn onto the UI at [ui.Offset.zero] with no additional
/// layers.
Future<void> matchPictureGolden(
  String goldenFile,
  CkPicture picture, {
  required ui.Rect region,
}) async {
  final LayerSceneBuilder sb = LayerSceneBuilder();
  sb.pushOffset(0, 0);
  sb.addPicture(ui.Offset.zero, picture);
  await renderScene(sb.build());
  await matchGoldenFile(goldenFile, region: region);
}

Future<bool> matchImage(ui.Image left, ui.Image right) async {
  if (left.width != right.width || left.height != right.height) {
    return false;
  }
  int getPixel(ByteData data, int x, int y) => data.getUint32((x + y * left.width) * 4);
  final ByteData leftData = (await left.toByteData())!;
  final ByteData rightData = (await right.toByteData())!;
  for (int y = 0; y < left.height; y++) {
    for (int x = 0; x < left.width; x++) {
      if (getPixel(leftData, x, y) != getPixel(rightData, x, y)) {
        return false;
      }
    }
  }
  return true;
}

/// Sends a platform message to create a Platform View with the given id and viewType.
Future<void> createPlatformView(int id, String viewType) {
  final Completer<void> completer = Completer<void>();
  ui.PlatformDispatcher.instance.sendPlatformMessage(
    'flutter/platform_views',
    codec.encodeMethodCall(MethodCall('create', <String, dynamic>{'id': id, 'viewType': viewType})),
    (dynamic _) => completer.complete(),
  );
  return completer.future;
}

/// Disposes of the platform view with the given [id].
Future<void> disposePlatformView(int id) {
  final Completer<void> completer = Completer<void>();
  ui.PlatformDispatcher.instance.sendPlatformMessage(
    'flutter/platform_views',
    codec.encodeMethodCall(MethodCall('dispose', id)),
    (dynamic _) => completer.complete(),
  );
  return completer.future;
}

/// Creates a pre-laid out one-line paragraph of text.
///
/// Useful in tests that need a simple label to annotate goldens.
CkParagraph makeSimpleText(
  String text, {
  String? fontFamily,
  double? fontSize,
  ui.FontStyle? fontStyle,
  ui.FontWeight? fontWeight,
  ui.Color? color,
}) {
  final CkParagraphBuilder builder = CkParagraphBuilder(
    CkParagraphStyle(
      fontFamily: fontFamily ?? 'Roboto',
      fontSize: fontSize ?? 14,
      fontStyle: fontStyle ?? ui.FontStyle.normal,
      fontWeight: fontWeight ?? ui.FontWeight.normal,
    ),
  );
  builder.pushStyle(CkTextStyle(color: color ?? const ui.Color(0xFF000000)));
  builder.addText(text);
  builder.pop();
  final CkParagraph paragraph = builder.build();
  paragraph.layout(const ui.ParagraphConstraints(width: 10000));
  return paragraph;
}
