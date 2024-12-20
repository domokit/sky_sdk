// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui' as ui show ColorFilter, Image;

import 'package:fake_async/fake_async.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

import '../image_data.dart';
import '../painting/mocks_for_image_cache.dart';
import '../rendering/rendering_tester.dart';

class TestCanvas implements Canvas {
  final List<Invocation> invocations = <Invocation>[];

  @override
  void noSuchMethod(Invocation invocation) {
    invocations.add(invocation);
  }
}

class SynchronousTestImageProvider extends ImageProvider<int> {
  const SynchronousTestImageProvider(this.image);

  final ui.Image image;

  @override
  Future<int> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<int>(1);
  }

  @override
  ImageStreamCompleter loadImage(int key, ImageDecoderCallback decode) {
    return OneFrameImageStreamCompleter(
      SynchronousFuture<ImageInfo>(TestImageInfo(key, image: image)),
    );
  }
}

class SynchronousErrorTestImageProvider extends ImageProvider<int> {
  const SynchronousErrorTestImageProvider(this.throwable);

  final Object throwable;

  @override
  Future<int> obtainKey(ImageConfiguration configuration) {
    throw throwable;
  }

  @override
  ImageStreamCompleter loadImage(int key, ImageDecoderCallback decode) {
    throw throwable;
  }
}

class AsyncTestImageProvider extends ImageProvider<int> {
  AsyncTestImageProvider(this.image);

  final ui.Image image;

  @override
  Future<int> obtainKey(ImageConfiguration configuration) {
    return Future<int>.value(2);
  }

  @override
  ImageStreamCompleter loadImage(int key, ImageDecoderCallback decode) {
    return OneFrameImageStreamCompleter(Future<ImageInfo>.value(TestImageInfo(key, image: image)));
  }
}

class DelayedImageProvider extends ImageProvider<DelayedImageProvider> {
  DelayedImageProvider(this.image);

  final ui.Image image;

  final Completer<ImageInfo> _completer = Completer<ImageInfo>();

  @override
  Future<DelayedImageProvider> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<DelayedImageProvider>(this);
  }

  @override
  ImageStreamCompleter loadImage(DelayedImageProvider key, ImageDecoderCallback decode) {
    return OneFrameImageStreamCompleter(_completer.future);
  }

  Future<void> complete() async {
    _completer.complete(ImageInfo(image: image));
  }

  @override
  String toString() => '${describeIdentity(this)}()';
}

class MultiFrameImageProvider extends ImageProvider<MultiFrameImageProvider> {
  MultiFrameImageProvider(this.completer);

  final MultiImageCompleter completer;

  @override
  Future<MultiFrameImageProvider> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<MultiFrameImageProvider>(this);
  }

  @override
  ImageStreamCompleter loadImage(MultiFrameImageProvider key, ImageDecoderCallback decode) {
    return completer;
  }

  @override
  String toString() => '${describeIdentity(this)}()';
}

class MultiImageCompleter extends ImageStreamCompleter {
  void testSetImage(ImageInfo info) {
    setImage(info);
  }
}

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('Decoration.lerp()', () {
    const BoxDecoration a = BoxDecoration(color: Color(0xFFFFFFFF));
    const BoxDecoration b = BoxDecoration(color: Color(0x00000000));

    BoxDecoration c = Decoration.lerp(a, b, 0.0)! as BoxDecoration;
    expect(c.color, equals(a.color));

    c = Decoration.lerp(a, b, 0.25)! as BoxDecoration;
    expect(c.color, equals(Color.lerp(const Color(0xFFFFFFFF), const Color(0x00000000), 0.25)));

    c = Decoration.lerp(a, b, 1.0)! as BoxDecoration;
    expect(c.color, equals(b.color));
  });

  test('Decoration.lerp identical a,b', () {
    expect(Decoration.lerp(null, null, 0), null);
    const Decoration decoration = BoxDecoration();
    expect(identical(Decoration.lerp(decoration, decoration, 0.5), decoration), true);
  });

  test('Decoration equality', () {
    const BoxDecoration a = BoxDecoration(
      color: Color(0xFFFFFFFF),
      boxShadow: <BoxShadow>[BoxShadow()],
    );

    const BoxDecoration b = BoxDecoration(
      color: Color(0xFFFFFFFF),
      boxShadow: <BoxShadow>[BoxShadow()],
    );

    expect(a.hashCode, equals(b.hashCode));
    expect(a, equals(b));
  });

  test('BoxDecorationImageListenerSync', () async {
    final ui.Image image = await createTestImage(width: 100, height: 100);
    final ImageProvider imageProvider = SynchronousTestImageProvider(image);
    final DecorationImage backgroundImage = DecorationImage(image: imageProvider);

    final BoxDecoration boxDecoration = BoxDecoration(image: backgroundImage);
    bool onChangedCalled = false;
    final BoxPainter boxPainter = boxDecoration.createBoxPainter(() {
      onChangedCalled = true;
    });

    final TestCanvas canvas = TestCanvas();
    const ImageConfiguration imageConfiguration = ImageConfiguration(size: Size.zero);
    boxPainter.paint(canvas, Offset.zero, imageConfiguration);

    // The onChanged callback should not be invoked during the call to boxPainter.paint
    expect(onChangedCalled, equals(false));
  });

  test('BoxDecorationImageListenerAsync', () async {
    final ui.Image image = await createTestImage(width: 10, height: 10);
    FakeAsync().run((FakeAsync async) {
      final ImageProvider imageProvider = AsyncTestImageProvider(image);
      final DecorationImage backgroundImage = DecorationImage(image: imageProvider);

      final BoxDecoration boxDecoration = BoxDecoration(image: backgroundImage);
      bool onChangedCalled = false;
      final BoxPainter boxPainter = boxDecoration.createBoxPainter(() {
        onChangedCalled = true;
      });

      final TestCanvas canvas = TestCanvas();
      const ImageConfiguration imageConfiguration = ImageConfiguration(size: Size.zero);
      boxPainter.paint(canvas, Offset.zero, imageConfiguration);

      // The onChanged callback should be invoked asynchronously.
      expect(onChangedCalled, equals(false));
      async.flushMicrotasks();
      expect(onChangedCalled, equals(true));
    });
  });

  test('BoxDecorationImageListener does not change when image is clone', () async {
    final ui.Image image1 = await createTestImage(width: 10, height: 10, cache: false);
    final ui.Image image2 = await createTestImage(width: 10, height: 10, cache: false);
    final MultiImageCompleter completer = MultiImageCompleter();
    final MultiFrameImageProvider imageProvider = MultiFrameImageProvider(completer);
    final DecorationImage backgroundImage = DecorationImage(image: imageProvider);

    final BoxDecoration boxDecoration = BoxDecoration(image: backgroundImage);
    bool onChangedCalled = false;
    final BoxPainter boxPainter = boxDecoration.createBoxPainter(() {
      onChangedCalled = true;
    });

    final TestCanvas canvas = TestCanvas();
    const ImageConfiguration imageConfiguration = ImageConfiguration(size: Size.zero);
    boxPainter.paint(canvas, Offset.zero, imageConfiguration);

    // The onChanged callback should be invoked asynchronously.
    expect(onChangedCalled, equals(false));

    completer.testSetImage(ImageInfo(image: image1.clone()));
    await null;

    expect(onChangedCalled, equals(true));
    onChangedCalled = false;
    completer.testSetImage(ImageInfo(image: image1.clone()));
    await null;

    expect(onChangedCalled, equals(false));

    completer.testSetImage(ImageInfo(image: image2.clone()));
    await null;

    expect(onChangedCalled, equals(true));
  });

  // Regression test for https://github.com/flutter/flutter/issues/7289.
  // A reference test would be better.
  test('BoxDecoration backgroundImage clip', () async {
    final ui.Image image = await createTestImage(width: 100, height: 100);
    void testDecoration({
      BoxShape shape = BoxShape.rectangle,
      BorderRadius? borderRadius,
      required bool expectClip,
    }) {
      FakeAsync().run((FakeAsync async) async {
        final DelayedImageProvider imageProvider = DelayedImageProvider(image);
        final DecorationImage backgroundImage = DecorationImage(image: imageProvider);

        final BoxDecoration boxDecoration = BoxDecoration(
          shape: shape,
          borderRadius: borderRadius,
          image: backgroundImage,
        );

        final TestCanvas canvas = TestCanvas();
        const ImageConfiguration imageConfiguration = ImageConfiguration(size: Size(100.0, 100.0));
        bool onChangedCalled = false;
        final BoxPainter boxPainter = boxDecoration.createBoxPainter(() {
          onChangedCalled = true;
        });

        // _BoxDecorationPainter._paintDecorationImage() resolves the background
        // image and adds a listener to the resolved image stream.
        boxPainter.paint(canvas, Offset.zero, imageConfiguration);
        await imageProvider.complete();

        // Run the listener which calls onChanged() which saves an internal
        // reference to the TestImage.
        async.flushMicrotasks();
        expect(onChangedCalled, isTrue);
        boxPainter.paint(canvas, Offset.zero, imageConfiguration);

        // We expect a clip to precede the drawImageRect call.
        final List<Invocation> commands =
            canvas.invocations.where((Invocation invocation) {
              return invocation.memberName == #clipPath || invocation.memberName == #drawImageRect;
            }).toList();
        if (expectClip) {
          // We expect a clip to precede the drawImageRect call.
          expect(commands.length, 2);
          expect(commands[0].memberName, equals(#clipPath));
          expect(commands[1].memberName, equals(#drawImageRect));
        } else {
          expect(commands.length, 1);
          expect(commands[0].memberName, equals(#drawImageRect));
        }
      });
    }

    testDecoration(shape: BoxShape.circle, expectClip: true);
    testDecoration(borderRadius: const BorderRadius.all(Radius.circular(16.0)), expectClip: true);
    testDecoration(expectClip: false);
  });

  test('DecorationImage test', () async {
    const ColorFilter colorFilter = ui.ColorFilter.mode(Color(0xFF00FF00), BlendMode.src);
    final ui.Image image = await createTestImage(width: 100, height: 100);
    final DecorationImage backgroundImage = DecorationImage(
      image: SynchronousTestImageProvider(image),
      colorFilter: colorFilter,
      fit: BoxFit.contain,
      alignment: Alignment.bottomLeft,
      centerSlice: const Rect.fromLTWH(10.0, 20.0, 30.0, 40.0),
      repeat: ImageRepeat.repeatY,
      opacity: 0.5,
      filterQuality: FilterQuality.high,
      invertColors: true,
      isAntiAlias: true,
    );

    final BoxDecoration boxDecoration = BoxDecoration(image: backgroundImage);
    final BoxPainter boxPainter = boxDecoration.createBoxPainter(() {
      assert(false);
    });
    final TestCanvas canvas = TestCanvas();
    boxPainter.paint(canvas, Offset.zero, const ImageConfiguration(size: Size(100.0, 100.0)));

    final Invocation call = canvas.invocations.singleWhere(
      (Invocation call) => call.memberName == #drawImageNine,
    );
    expect(call.isMethod, isTrue);
    expect(call.positionalArguments, hasLength(4));
    expect(call.positionalArguments[0], isA<ui.Image>());
    expect(call.positionalArguments[1], const Rect.fromLTRB(10.0, 20.0, 40.0, 60.0));
    expect(call.positionalArguments[2], const Rect.fromLTRB(0.0, 0.0, 100.0, 100.0));
    expect(call.positionalArguments[3], isA<Paint>());
    final Paint paint = call.positionalArguments[3] as Paint;
    expect(paint.colorFilter, colorFilter);
    expect(paint.color, isSameColorAs(const Color(0x7F000000))); // 0.5 opacity
    expect(paint.filterQuality, FilterQuality.high);
    expect(paint.isAntiAlias, true);
    expect(paint.invertColors, isTrue);
  });

  test('DecorationImage.toString', () async {
    expect(
      DecorationImage(
        image: SynchronousTestImageProvider(await createTestImage(width: 100, height: 100)),
        opacity: 0.99,
        scale: 2.01,
      ).toString(),
      'DecorationImage(SynchronousTestImageProvider(), Alignment.center, scale 2.0, opacity 1.0, FilterQuality.medium)',
    );
  });

  test('DecorationImage with null textDirection configuration should throw Error', () async {
    const ColorFilter colorFilter = ui.ColorFilter.mode(Color(0xFF00FF00), BlendMode.src);
    final ui.Image image = await createTestImage(width: 100, height: 100);
    final DecorationImage backgroundImage = DecorationImage(
      image: SynchronousTestImageProvider(image),
      colorFilter: colorFilter,
      fit: BoxFit.contain,
      centerSlice: const Rect.fromLTWH(10.0, 20.0, 30.0, 40.0),
      repeat: ImageRepeat.repeatY,
      matchTextDirection: true,
      scale: 0.5,
      opacity: 0.5,
      invertColors: true,
      isAntiAlias: true,
    );
    final BoxDecoration boxDecoration = BoxDecoration(image: backgroundImage);
    final BoxPainter boxPainter = boxDecoration.createBoxPainter(() {
      assert(false);
    });
    final TestCanvas canvas = TestCanvas();
    late FlutterError error;
    try {
      boxPainter.paint(canvas, Offset.zero, const ImageConfiguration(size: Size(100.0, 100.0)));
    } on FlutterError catch (e) {
      error = e;
    }
    expect(error, isNotNull);
    expect(error.diagnostics.length, 4);
    expect(error.diagnostics[2], isA<DiagnosticsProperty<DecorationImage>>());
    expect(error.diagnostics[3], isA<DiagnosticsProperty<ImageConfiguration>>());
    expect(
      error.toStringDeep(wrapWidth: 640),
      'FlutterError\n'
      '   DecorationImage.matchTextDirection can only be used when a '
      'TextDirection is available.\n'
      '   When DecorationImagePainter.paint() was called, there was no text '
      'direction provided in the ImageConfiguration object to match.\n'
      '   The DecorationImage was:\n'
      '     DecorationImage(SynchronousTestImageProvider(), '
      'ColorFilter.mode(${const Color(0xff00ff00)}, BlendMode.src), '
      'BoxFit.contain, Alignment.center, '
      'centerSlice: Rect.fromLTRB(10.0, 20.0, 40.0, 60.0), '
      'ImageRepeat.repeatY, '
      'match text direction, '
      'scale 0.5, '
      'opacity 0.5, '
      'FilterQuality.medium, '
      'invert colors, '
      'use anti-aliasing)\n'
      '   The ImageConfiguration was:\n'
      '     ImageConfiguration(size: Size(100.0, 100.0))\n',
    );
  }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/87364

  test('DecorationImage - error listener', () async {
    late String exception;
    final DecorationImage backgroundImage = DecorationImage(
      image: const SynchronousErrorTestImageProvider('threw'),
      onError: (dynamic error, StackTrace? stackTrace) {
        exception = error as String;
      },
    );

    backgroundImage
        .createPainter(() {})
        .paint(TestCanvas(), Rect.largest, Path(), ImageConfiguration.empty);
    // Yield so that the exception callback gets called before we check it.
    await null;
    expect(exception, 'threw');
  });

  test('BoxDecoration.lerp - shapes', () {
    // We don't lerp the shape, we just switch from one to the other at t=0.5.
    // (Use a ShapeDecoration and ShapeBorder if you want to lerp the shapes...)
    expect(
      BoxDecoration.lerp(const BoxDecoration(), const BoxDecoration(shape: BoxShape.circle), -1.0),
      const BoxDecoration(),
    );
    expect(
      BoxDecoration.lerp(const BoxDecoration(), const BoxDecoration(shape: BoxShape.circle), 0.0),
      const BoxDecoration(),
    );
    expect(
      BoxDecoration.lerp(const BoxDecoration(), const BoxDecoration(shape: BoxShape.circle), 0.25),
      const BoxDecoration(),
    );
    expect(
      BoxDecoration.lerp(const BoxDecoration(), const BoxDecoration(shape: BoxShape.circle), 0.75),
      const BoxDecoration(shape: BoxShape.circle),
    );
    expect(
      BoxDecoration.lerp(const BoxDecoration(), const BoxDecoration(shape: BoxShape.circle), 1.0),
      const BoxDecoration(shape: BoxShape.circle),
    );
    expect(
      BoxDecoration.lerp(const BoxDecoration(), const BoxDecoration(shape: BoxShape.circle), 2.0),
      const BoxDecoration(shape: BoxShape.circle),
    );
  });

  test('BoxDecoration.lerp - gradients', () {
    const Gradient gradient = LinearGradient(colors: <Color>[Color(0x00000000), Color(0xFFFFFFFF)]);
    expect(
      BoxDecoration.lerp(const BoxDecoration(), const BoxDecoration(gradient: gradient), -1.0),
      const BoxDecoration(
        gradient: LinearGradient(colors: <Color>[Color(0x00000000), Color(0x00FFFFFF)]),
      ),
    );
    expect(
      BoxDecoration.lerp(const BoxDecoration(), const BoxDecoration(gradient: gradient), 0.0),
      const BoxDecoration(),
    );
    expect(
      BoxDecoration.lerp(const BoxDecoration(), const BoxDecoration(gradient: gradient), 0.2),
      const BoxDecoration(
        gradient: LinearGradient(colors: <Color>[Color(0x00000000), Color(0x33FFFFFF)]),
      ),
    );
    expect(
      BoxDecoration.lerp(const BoxDecoration(), const BoxDecoration(gradient: gradient), 1 / 3),
      const BoxDecoration(
        gradient: LinearGradient(colors: <Color>[Color(0x00000000), Color(0x55FFFFFF)]),
      ),
    );
    expect(
      BoxDecoration.lerp(const BoxDecoration(), const BoxDecoration(gradient: gradient), 1.0),
      const BoxDecoration(gradient: gradient),
    );
    expect(
      BoxDecoration.lerp(const BoxDecoration(), const BoxDecoration(gradient: gradient), 2.0),
      const BoxDecoration(gradient: gradient),
    );
  });

  test('Decoration.lerp with unrelated decorations', () {
    expect(
      Decoration.lerp(const FlutterLogoDecoration(), const BoxDecoration(), 0.0),
      isA<FlutterLogoDecoration>(),
    );
    expect(
      Decoration.lerp(const FlutterLogoDecoration(), const BoxDecoration(), 0.25),
      isA<FlutterLogoDecoration>(),
    );
    expect(
      Decoration.lerp(const FlutterLogoDecoration(), const BoxDecoration(), 0.75),
      isA<BoxDecoration>(),
    );
    expect(
      Decoration.lerp(const FlutterLogoDecoration(), const BoxDecoration(), 1.0),
      isA<BoxDecoration>(),
    );
  });

  test('paintImage BoxFit.none scale test', () async {
    for (double scale = 1.0; scale <= 4.0; scale += 1.0) {
      final TestCanvas canvas = TestCanvas();

      const Rect outputRect = Rect.fromLTWH(30.0, 30.0, 250.0, 250.0);
      final ui.Image image = await createTestImage(width: 100, height: 100);

      paintImage(
        canvas: canvas,
        rect: outputRect,
        image: image,
        scale: scale,
        alignment: Alignment.bottomRight,
        fit: BoxFit.none,
      );

      const Size imageSize = Size(100.0, 100.0);

      final Invocation call = canvas.invocations.firstWhere(
        (Invocation call) => call.memberName == #drawImageRect,
      );

      expect(call.isMethod, isTrue);
      expect(call.positionalArguments, hasLength(4));

      expect(call.positionalArguments[0], isA<ui.Image>());

      // sourceRect should contain all pixels of the source image
      expect(call.positionalArguments[1], Offset.zero & imageSize);

      // Image should be scaled down (divided by scale)
      // and be positioned in the bottom right of the outputRect
      final Size expectedTileSize = imageSize / scale;
      final Rect expectedTileRect = Rect.fromPoints(
        outputRect.bottomRight.translate(-expectedTileSize.width, -expectedTileSize.height),
        outputRect.bottomRight,
      );
      expect(call.positionalArguments[2], expectedTileRect);

      expect(call.positionalArguments[3], isA<Paint>());
    }
  });

  test('paintImage BoxFit.scaleDown scale test', () async {
    for (double scale = 1.0; scale <= 4.0; scale += 1.0) {
      final TestCanvas canvas = TestCanvas();

      // container size > scaled image size
      const Rect outputRect = Rect.fromLTWH(30.0, 30.0, 250.0, 250.0);
      final ui.Image image = await createTestImage(width: 100, height: 100);

      paintImage(
        canvas: canvas,
        rect: outputRect,
        image: image,
        scale: scale,
        alignment: Alignment.bottomRight,
        fit: BoxFit.scaleDown,
      );

      const Size imageSize = Size(100.0, 100.0);

      final Invocation call = canvas.invocations.firstWhere(
        (Invocation call) => call.memberName == #drawImageRect,
      );

      expect(call.isMethod, isTrue);
      expect(call.positionalArguments, hasLength(4));

      expect(call.positionalArguments[0], isA<ui.Image>());

      // sourceRect should contain all pixels of the source image
      expect(call.positionalArguments[1], Offset.zero & imageSize);

      // Image should be scaled down (divided by scale)
      // and be positioned in the bottom right of the outputRect
      final Size expectedTileSize = imageSize / scale;
      final Rect expectedTileRect = Rect.fromPoints(
        outputRect.bottomRight.translate(-expectedTileSize.width, -expectedTileSize.height),
        outputRect.bottomRight,
      );
      expect(call.positionalArguments[2], expectedTileRect);

      expect(call.positionalArguments[3], isA<Paint>());
    }
  });

  test('paintImage BoxFit.scaleDown test', () async {
    final TestCanvas canvas = TestCanvas();

    // container height (20 px) < scaled image height (50 px)
    const Rect outputRect = Rect.fromLTWH(30.0, 30.0, 250.0, 20.0);
    final ui.Image image = await createTestImage(width: 100, height: 100);

    paintImage(
      canvas: canvas,
      rect: outputRect,
      image: image,
      scale: 2.0,
      alignment: Alignment.bottomRight,
      fit: BoxFit.scaleDown,
    );

    const Size imageSize = Size(100.0, 100.0);

    final Invocation call = canvas.invocations.firstWhere(
      (Invocation call) => call.memberName == #drawImageRect,
    );

    expect(call.isMethod, isTrue);
    expect(call.positionalArguments, hasLength(4));

    expect(call.positionalArguments[0], isA<ui.Image>());

    // sourceRect should contain all pixels of the source image
    expect(call.positionalArguments[1], Offset.zero & imageSize);

    // Image should be scaled down to fit in height
    // and be positioned in the bottom right of the outputRect
    const Size expectedTileSize = Size(20.0, 20.0);
    final Rect expectedTileRect = Rect.fromPoints(
      outputRect.bottomRight.translate(-expectedTileSize.width, -expectedTileSize.height),
      outputRect.bottomRight,
    );
    expect(call.positionalArguments[2], expectedTileRect);

    expect(call.positionalArguments[3], isA<Paint>());
  });

  test('paintImage boxFit, scale and alignment test', () async {
    const List<BoxFit> boxFits = <BoxFit>[
      BoxFit.contain,
      BoxFit.cover,
      BoxFit.fitWidth,
      BoxFit.fitWidth,
      BoxFit.fitHeight,
      BoxFit.none,
      BoxFit.scaleDown,
    ];

    for (final BoxFit boxFit in boxFits) {
      final TestCanvas canvas = TestCanvas();

      const Rect outputRect = Rect.fromLTWH(30.0, 30.0, 250.0, 250.0);
      final ui.Image image = await createTestImage(width: 100, height: 100);

      paintImage(canvas: canvas, rect: outputRect, image: image, scale: 3.0, fit: boxFit);

      final Invocation call = canvas.invocations.firstWhere(
        (Invocation call) => call.memberName == #drawImageRect,
      );

      expect(call.isMethod, isTrue);
      expect(call.positionalArguments, hasLength(4));

      // Image should be positioned in the center of the container
      // ignore: avoid_dynamic_calls
      expect(call.positionalArguments[2].center, outputRect.center);
    }
  });

  test('paintImage with repeatX and fitHeight', () async {
    final TestCanvas canvas = TestCanvas();

    // Paint a square image into an output rect that is twice as wide as it is
    // tall.  Two copies of the image should be painted, one next to the other.
    const Rect outputRect = Rect.fromLTWH(30.0, 30.0, 400.0, 200.0);
    final ui.Image image = await createTestImage(width: 100, height: 100);

    paintImage(
      canvas: canvas,
      rect: outputRect,
      image: image,
      alignment: Alignment.topLeft,
      fit: BoxFit.fitHeight,
      repeat: ImageRepeat.repeatX,
    );

    const Size imageSize = Size(100.0, 100.0);

    final List<Invocation> calls =
        canvas.invocations.where((Invocation call) => call.memberName == #drawImageRect).toList();
    final Set<Rect> tileRects = <Rect>{};

    expect(calls, hasLength(2));
    for (final Invocation call in calls) {
      expect(call.isMethod, isTrue);
      expect(call.positionalArguments, hasLength(4));

      expect(call.positionalArguments[0], isA<ui.Image>());

      // sourceRect should contain all pixels of the source image
      expect(call.positionalArguments[1], Offset.zero & imageSize);

      tileRects.add(call.positionalArguments[2] as Rect);

      expect(call.positionalArguments[3], isA<Paint>());
    }

    expect(tileRects, <Rect>{
      const Rect.fromLTWH(30.0, 30.0, 200.0, 200.0),
      const Rect.fromLTWH(230.0, 30.0, 200.0, 200.0),
    });
  });

  test('paintImage with repeatY and fitWidth', () async {
    final TestCanvas canvas = TestCanvas();

    // Paint a square image into an output rect that is twice as tall as it is
    // wide.  Two copies of the image should be painted, one above the other.
    const Rect outputRect = Rect.fromLTWH(30.0, 30.0, 200.0, 400.0);
    final ui.Image image = await createTestImage(width: 100, height: 100);

    paintImage(
      canvas: canvas,
      rect: outputRect,
      image: image,
      alignment: Alignment.topLeft,
      fit: BoxFit.fitWidth,
      repeat: ImageRepeat.repeatY,
    );

    const Size imageSize = Size(100.0, 100.0);

    final List<Invocation> calls =
        canvas.invocations.where((Invocation call) => call.memberName == #drawImageRect).toList();
    final Set<Rect> tileRects = <Rect>{};

    expect(calls, hasLength(2));
    for (final Invocation call in calls) {
      expect(call.isMethod, isTrue);
      expect(call.positionalArguments, hasLength(4));

      expect(call.positionalArguments[0], isA<ui.Image>());

      // sourceRect should contain all pixels of the source image
      expect(call.positionalArguments[1], Offset.zero & imageSize);

      tileRects.add(call.positionalArguments[2] as Rect);

      expect(call.positionalArguments[3], isA<Paint>());
    }

    expect(tileRects, <Rect>{
      const Rect.fromLTWH(30.0, 30.0, 200.0, 200.0),
      const Rect.fromLTWH(30.0, 230.0, 200.0, 200.0),
    });
  });

  test('DecorationImage scale test', () async {
    final ui.Image image = await createTestImage(width: 100, height: 100);
    final DecorationImage backgroundImage = DecorationImage(
      image: SynchronousTestImageProvider(image),
      scale: 4,
      alignment: Alignment.topLeft,
    );

    final BoxDecoration boxDecoration = BoxDecoration(image: backgroundImage);
    final BoxPainter boxPainter = boxDecoration.createBoxPainter(() {
      assert(false);
    });
    final TestCanvas canvas = TestCanvas();
    boxPainter.paint(canvas, Offset.zero, const ImageConfiguration(size: Size(100.0, 100.0)));

    final Invocation call = canvas.invocations.firstWhere(
      (Invocation call) => call.memberName == #drawImageRect,
    );
    // The image should scale down to Size(25.0, 25.0) from Size(100.0, 100.0)
    // considering DecorationImage scale to be 4.0 and Image scale to be 1.0.
    // ignore: avoid_dynamic_calls
    expect(call.positionalArguments[2].size, const Size(25.0, 25.0));
    expect(call.positionalArguments[2], const Rect.fromLTRB(0.0, 0.0, 25.0, 25.0));
  });

  test('DecorationImagePainter disposes of image when disposed', () async {
    final ImageProvider provider = MemoryImage(Uint8List.fromList(kTransparentImage));

    final ImageStream stream = provider.resolve(ImageConfiguration.empty);

    final Completer<ImageInfo> infoCompleter = Completer<ImageInfo>();
    void listener(ImageInfo image, bool syncCall) {
      assert(!infoCompleter.isCompleted);
      infoCompleter.complete(image);
    }

    stream.addListener(ImageStreamListener(listener));

    final ImageInfo info = await infoCompleter.future;
    final int baselineRefCount = info.image.debugGetOpenHandleStackTraces()!.length;

    final DecorationImagePainter painter = DecorationImage(image: provider).createPainter(() {});
    final Canvas canvas = TestCanvas();
    painter.paint(canvas, Rect.zero, Path(), ImageConfiguration.empty);

    expect(info.image.debugGetOpenHandleStackTraces()!.length, baselineRefCount + 1);
    painter.dispose();
    expect(info.image.debugGetOpenHandleStackTraces()!.length, baselineRefCount);

    info.dispose();
  }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/87442

  test('BoxShadow.copyWith', () {
    expect(const BoxShadow(), isNot(const BoxShadow(color: Color(0xFF112233))));
    expect(
      const BoxShadow().copyWith(color: const Color(0xFF112233)),
      const BoxShadow(color: Color(0xFF112233)),
    );
    expect(const BoxShadow(), isNot(const BoxShadow(offset: Offset(1.0, 2.0))));
    expect(
      const BoxShadow().copyWith(offset: const Offset(1.0, 2.0)),
      const BoxShadow(offset: Offset(1.0, 2.0)),
    );
    expect(const BoxShadow(), isNot(const BoxShadow(blurRadius: 123.0)));
    expect(const BoxShadow().copyWith(blurRadius: 123.0), const BoxShadow(blurRadius: 123.0));
    expect(const BoxShadow(), isNot(const BoxShadow(spreadRadius: 123.0)));
    expect(const BoxShadow().copyWith(spreadRadius: 123.0), const BoxShadow(spreadRadius: 123.0));
    expect(const BoxShadow(), isNot(const BoxShadow(blurStyle: BlurStyle.outer)));
    expect(
      const BoxShadow().copyWith(blurStyle: BlurStyle.outer),
      const BoxShadow(blurStyle: BlurStyle.outer),
    );
  });
}
