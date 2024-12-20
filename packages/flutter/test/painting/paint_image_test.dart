// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

class TestCanvas implements Canvas {
  final List<Invocation> invocations = <Invocation>[];

  @override
  void noSuchMethod(Invocation invocation) {
    invocations.add(invocation);
  }
}

void main() {
  late ui.Image image300x300;
  late ui.Image image300x200;

  setUpAll(() async {
    image300x300 = await createTestImage(width: 300, height: 300, cache: false);
    image300x200 = await createTestImage(width: 300, height: 200, cache: false);
  });

  setUp(() {
    debugFlushLastFrameImageSizeInfo();
  });

  test('Cover and align', () async {
    final TestCanvas canvas = TestCanvas();
    paintImage(
      canvas: canvas,
      rect: const Rect.fromLTWH(50.0, 75.0, 200.0, 100.0),
      image: image300x300,
      fit: BoxFit.cover,
      alignment: Alignment.centerLeft,
    );

    final Invocation command = canvas.invocations.firstWhere((Invocation invocation) {
      return invocation.memberName == #drawImageRect;
    });

    expect(command, isNotNull);
    expect(command.positionalArguments[0], equals(image300x300));
    expect(command.positionalArguments[1], equals(const Rect.fromLTWH(0.0, 75.0, 300.0, 150.0)));
    expect(command.positionalArguments[2], equals(const Rect.fromLTWH(50.0, 75.0, 200.0, 100.0)));
  });

  test('debugInvertOversizedImages', () async {
    debugInvertOversizedImages = true;
    expect(
      PaintingBinding.instance.platformDispatcher.views.any(
        (ui.FlutterView view) => view.devicePixelRatio > 1.0,
      ),
      isTrue,
    );
    final FlutterExceptionHandler? oldFlutterError = FlutterError.onError;

    final List<String> messages = <String>[];
    FlutterError.onError = (FlutterErrorDetails details) {
      messages.add(details.exceptionAsString());
    };

    final TestCanvas canvas = TestCanvas();
    const Rect rect = Rect.fromLTWH(50.0, 50.0, 100.0, 50.0);

    paintImage(
      canvas: canvas,
      rect: rect,
      image: image300x300,
      debugImageLabel: 'TestImage',
      fit: BoxFit.fill,
    );

    final List<Invocation> commands =
        canvas.invocations
            .skipWhile((Invocation invocation) => invocation.memberName != #saveLayer)
            .take(4)
            .toList();

    expect(commands[0].positionalArguments[0], rect);
    final Paint paint = commands[0].positionalArguments[1] as Paint;
    expect(
      paint.colorFilter,
      const ColorFilter.matrix(<double>[
        -1,
        0,
        0,
        0,
        255,
        0,
        -1,
        0,
        0,
        255,
        0,
        0,
        -1,
        0,
        255,
        0,
        0,
        0,
        1,
        0,
      ]),
    );
    expect(commands[1].memberName, #translate);
    expect(commands[1].positionalArguments[0], 0.0);
    expect(commands[1].positionalArguments[1], 75.0);

    expect(commands[2].memberName, #scale);
    expect(commands[2].positionalArguments[0], 1.0);
    expect(commands[2].positionalArguments[1], -1.0);

    expect(commands[3].memberName, #translate);
    expect(commands[3].positionalArguments[0], 0.0);
    expect(commands[3].positionalArguments[1], -75.0);

    expect(
      messages.single,
      'Image TestImage has a display size of 300×150 but a decode size of 300×300, which uses an additional 234KB (assuming a device pixel ratio of ${3.0}).\n\n'
      'Consider resizing the asset ahead of time, supplying a cacheWidth parameter of 300, a cacheHeight parameter of 150, or using a ResizeImage.',
    );

    debugInvertOversizedImages = false;
    FlutterError.onError = oldFlutterError;
  });

  test('debugInvertOversizedImages smaller than overhead allowance', () async {
    debugInvertOversizedImages = true;
    final FlutterExceptionHandler? oldFlutterError = FlutterError.onError;

    final List<String> messages = <String>[];
    FlutterError.onError = (FlutterErrorDetails details) {
      messages.add(details.exceptionAsString());
    };

    try {
      // Create a 290x290 sized image, which is ~24kb less than the allocated size,
      // and below the default debugImageOverheadAllowance size of 128kb.
      const Rect rect = Rect.fromLTWH(50.0, 50.0, 290.0, 290.0);
      final TestCanvas canvas = TestCanvas();

      paintImage(
        canvas: canvas,
        rect: rect,
        image: image300x300,
        debugImageLabel: 'TestImage',
        fit: BoxFit.fill,
      );

      expect(messages, isEmpty);
    } finally {
      debugInvertOversizedImages = false;
      FlutterError.onError = oldFlutterError;
    }
  });

  test('centerSlice with scale ≠ 1', () async {
    final TestCanvas canvas = TestCanvas();
    paintImage(
      canvas: canvas,
      rect: const Rect.fromLTRB(10, 20, 430, 420),
      image: image300x300,
      scale: 2.0,
      centerSlice: const Rect.fromLTRB(50, 40, 250, 260),
    );

    final Invocation command = canvas.invocations.firstWhere((Invocation invocation) {
      return invocation.memberName == #drawImageNine;
    });

    expect(command, isNotNull);
    expect(command.positionalArguments[0], equals(image300x300));
    expect(command.positionalArguments[1], equals(const Rect.fromLTRB(100.0, 80.0, 500.0, 520.0)));
    expect(command.positionalArguments[2], equals(const Rect.fromLTRB(20.0, 40.0, 860.0, 840.0)));
  });

  testWidgets('Reports Image painting', (WidgetTester tester) async {
    late ImageSizeInfo imageSizeInfo;
    int count = 0;
    debugOnPaintImage = (ImageSizeInfo info) {
      count += 1;
      imageSizeInfo = info;
    };

    final TestCanvas canvas = TestCanvas();
    paintImage(
      canvas: canvas,
      rect: const Rect.fromLTWH(50.0, 75.0, 200.0, 100.0),
      image: image300x300,
      debugImageLabel: 'test.png',
    );

    expect(count, 1);
    expect(imageSizeInfo, isNotNull);
    expect(imageSizeInfo.source, 'test.png');
    expect(imageSizeInfo.imageSize, const Size(300, 300));
    expect(imageSizeInfo.displaySize, const Size(200, 100) * tester.view.devicePixelRatio);

    // Make sure that we don't report an identical image size info if we
    // redraw in the next frame.
    tester.binding.scheduleForcedFrame();
    await tester.pump();

    paintImage(
      canvas: canvas,
      rect: const Rect.fromLTWH(50.0, 75.0, 200.0, 100.0),
      image: image300x300,
      debugImageLabel: 'test.png',
    );

    expect(count, 1);

    debugOnPaintImage = null;
  });

  testWidgets('Reports Image painting - change per frame', (WidgetTester tester) async {
    late ImageSizeInfo imageSizeInfo;
    int count = 0;
    debugOnPaintImage = (ImageSizeInfo info) {
      count += 1;
      imageSizeInfo = info;
    };

    final TestCanvas canvas = TestCanvas();
    paintImage(
      canvas: canvas,
      rect: const Rect.fromLTWH(50.0, 75.0, 200.0, 100.0),
      image: image300x300,
      debugImageLabel: 'test.png',
    );

    expect(count, 1);
    expect(imageSizeInfo, isNotNull);
    expect(imageSizeInfo.source, 'test.png');
    expect(imageSizeInfo.imageSize, const Size(300, 300));
    expect(imageSizeInfo.displaySize, const Size(200, 100) * tester.view.devicePixelRatio);

    // Make sure that we don't report an identical image size info if we
    // redraw in the next frame.
    tester.binding.scheduleForcedFrame();
    await tester.pump();

    paintImage(
      canvas: canvas,
      rect: const Rect.fromLTWH(50.0, 75.0, 200.0, 150.0),
      image: image300x300,
      debugImageLabel: 'test.png',
    );

    expect(count, 2);
    expect(imageSizeInfo, isNotNull);
    expect(imageSizeInfo.source, 'test.png');
    expect(imageSizeInfo.imageSize, const Size(300, 300));
    expect(imageSizeInfo.displaySize, const Size(200, 150) * tester.view.devicePixelRatio);

    debugOnPaintImage = null;
  });

  testWidgets('Reports Image painting - no debug label', (WidgetTester tester) async {
    late ImageSizeInfo imageSizeInfo;
    int count = 0;
    debugOnPaintImage = (ImageSizeInfo info) {
      count += 1;
      imageSizeInfo = info;
    };

    final TestCanvas canvas = TestCanvas();
    paintImage(
      canvas: canvas,
      rect: const Rect.fromLTWH(50.0, 75.0, 200.0, 100.0),
      image: image300x200,
    );

    expect(count, 1);
    expect(imageSizeInfo, isNotNull);
    expect(imageSizeInfo.source, '<Unknown Image(300×200)>');
    expect(imageSizeInfo.imageSize, const Size(300, 200));
    expect(imageSizeInfo.displaySize, const Size(200, 100) * tester.view.devicePixelRatio);

    debugOnPaintImage = null;
  });

  // See also the DecorationImage tests in: decoration_test.dart
}
