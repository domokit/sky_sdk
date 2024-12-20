// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';
import 'dart:ui';

import 'package:test/test.dart';

void main() {
  test('Scene.toImageSync succeeds', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    const Color color = Color(0xFF123456);
    canvas.drawPaint(Paint()..color = color);
    final Picture picture = recorder.endRecording();
    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(10, 10);
    builder.addPicture(const Offset(5, 5), picture);
    final Scene scene = builder.build();

    final Image image = scene.toImageSync(6, 8);
    picture.dispose();
    scene.dispose();

    expect(image.width, 6);
    expect(image.height, 8);

    final ByteData? data = await image.toByteData();

    expect(data, isNotNull);
    expect(data!.lengthInBytes, 6 * 8 * 4);
    expect(data.buffer.asUint8List()[0], 0x12);
    expect(data.buffer.asUint8List()[1], 0x34);
    expect(data.buffer.asUint8List()[2], 0x56);
    expect(data.buffer.asUint8List()[3], 0xFF);
  });

  test('Scene.toImageSync succeeds with texture layer', () async {
    final SceneBuilder builder = SceneBuilder();
    builder.pushOffset(10, 10);
    builder.addTexture(0, width: 10, height: 10);

    final Scene scene = builder.build();
    final Image image = scene.toImageSync(20, 20);
    scene.dispose();

    expect(image.width, 20);
    expect(image.height, 20);

    final ByteData? data = await image.toByteData();

    expect(data, isNotNull);
    expect(data!.lengthInBytes, 20 * 20 * 4);
    expect(data.buffer.asUint8List()[0], 0);
    expect(data.buffer.asUint8List()[1], 0);
    expect(data.buffer.asUint8List()[2], 0);
    expect(data.buffer.asUint8List()[3], 0);
  });

  test('addPicture with disposed picture does not crash', () {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.drawPaint(Paint());
    final Picture picture = recorder.endRecording();
    picture.dispose();

    expect(picture.debugDisposed, isTrue);

    final SceneBuilder builder = SceneBuilder();
    expect(
      () => builder.addPicture(Offset.zero, picture),
      throwsA(const isInstanceOf<AssertionError>()),
    );

    final Scene scene = builder.build();
    scene.dispose();
  });

  test('pushTransform validates the matrix', () {
    final SceneBuilder builder = SceneBuilder();
    final Float64List matrix4 = Float64List.fromList(<double>[
      1,
      0,
      0,
      0,
      0,
      1,
      0,
      0,
      0,
      0,
      1,
      0,
      0,
      0,
      0,
      1,
    ]);
    expect(builder.pushTransform(matrix4), isNotNull);

    final Float64List matrix4WrongLength = Float64List.fromList(<double>[
      1,
      0,
      0,
      0,
      0,
      1,
      0,
      0,
      0,
      1,
      0,
      0,
      0,
      0,
    ]);
    expect(() => builder.pushTransform(matrix4WrongLength), throwsA(isA<AssertionError>()));

    final Float64List matrix4NaN = Float64List.fromList(<double>[
      1,
      0,
      0,
      0,
      0,
      1,
      0,
      0,
      0,
      0,
      1,
      0,
      0,
      0,
      0,
      double.nan,
    ]);
    expect(() => builder.pushTransform(matrix4NaN), throwsA(isA<AssertionError>()));

    final Float64List matrix4Infinity = Float64List.fromList(<double>[
      1,
      0,
      0,
      0,
      0,
      1,
      0,
      0,
      0,
      0,
      1,
      0,
      0,
      0,
      0,
      double.infinity,
    ]);
    expect(() => builder.pushTransform(matrix4Infinity), throwsA(isA<AssertionError>()));
  });

  test('SceneBuilder accepts typed layers', () {
    final SceneBuilder builder1 = SceneBuilder();
    final OpacityEngineLayer opacity1 = builder1.pushOpacity(100);
    expect(opacity1, isNotNull);
    builder1.pop();
    builder1.build();

    final SceneBuilder builder2 = SceneBuilder();
    final OpacityEngineLayer opacity2 = builder2.pushOpacity(200, oldLayer: opacity1);
    expect(opacity2, isNotNull);
    builder2.pop();
    builder2.build();
  });

  // Attempts to use the same layer first as `oldLayer` then in `addRetained`.
  void testPushThenIllegalRetain(_TestNoSharingFunction pushFunction) {
    final SceneBuilder builder1 = SceneBuilder();
    final EngineLayer layer = pushFunction(builder1, null);
    builder1.pop();
    builder1.build();

    final SceneBuilder builder2 = SceneBuilder();
    pushFunction(builder2, layer);
    builder2.pop();

    expect(
      () {
        builder2.addRetained(layer);
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.toString(),
          'toString',
          contains('The layer is already being used'),
        ),
      ),
    );
    builder2.build();
  }

  // Attempts to use the same layer first in `addRetained` then as `oldLayer`.
  void testAddRetainedThenIllegalPush(_TestNoSharingFunction pushFunction) {
    final SceneBuilder builder1 = SceneBuilder();
    final EngineLayer layer = pushFunction(builder1, null);
    builder1.pop();
    builder1.build();

    final SceneBuilder builder2 = SceneBuilder();
    builder2.addRetained(layer);

    expect(
      () {
        pushFunction(builder2, layer);
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.toString(),
          'toString',
          contains('The layer is already being used'),
        ),
      ),
    );
    builder2.build();
  }

  // Attempts to retain the same layer twice in the same scene.
  void testDoubleAddRetained(_TestNoSharingFunction pushFunction) {
    final SceneBuilder builder1 = SceneBuilder();
    final EngineLayer layer = pushFunction(builder1, null);
    builder1.pop();
    builder1.build();

    final SceneBuilder builder2 = SceneBuilder();
    builder2.addRetained(layer);
    expect(
      () {
        builder2.addRetained(layer);
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.toString(),
          'toString',
          contains('The layer is already being used'),
        ),
      ),
    );
    builder2.build();
  }

  // Attempts to use the same layer as `oldLayer` twice in the same scene.
  void testPushOldLayerTwice(_TestNoSharingFunction pushFunction) {
    final SceneBuilder builder1 = SceneBuilder();
    final EngineLayer layer = pushFunction(builder1, null);
    builder1.pop();
    builder1.build();

    final SceneBuilder builder2 = SceneBuilder();
    pushFunction(builder2, layer);
    expect(
      () {
        pushFunction(builder2, layer);
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.toString(),
          'toString',
          contains('was previously used as oldLayer'),
        ),
      ),
    );
    builder2.build();
  }

  // Attempts to use a child of a retained layer as an `oldLayer`.
  void testPushChildLayerOfRetainedLayer(_TestNoSharingFunction pushFunction) {
    final SceneBuilder builder1 = SceneBuilder();
    final EngineLayer layer = pushFunction(builder1, null);
    final OpacityEngineLayer childLayer = builder1.pushOpacity(123);
    builder1.pop();
    builder1.pop();
    builder1.build();

    final SceneBuilder builder2 = SceneBuilder();
    builder2.addRetained(layer);
    expect(
      () {
        builder2.pushOpacity(321, oldLayer: childLayer);
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.toString(),
          'toString',
          contains('The layer is already being used'),
        ),
      ),
    );
    builder2.build();
  }

  // Attempts to retain a layer whose child is already used as `oldLayer` elsewhere in the scene.
  void testRetainParentLayerOfPushedChild(_TestNoSharingFunction pushFunction) {
    final SceneBuilder builder1 = SceneBuilder();
    final EngineLayer layer = pushFunction(builder1, null);
    final OpacityEngineLayer childLayer = builder1.pushOpacity(123);
    builder1.pop();
    builder1.pop();
    builder1.build();

    final SceneBuilder builder2 = SceneBuilder();
    builder2.pushOpacity(234, oldLayer: childLayer);
    builder2.pop();
    expect(
      () {
        builder2.addRetained(layer);
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.toString(),
          'toString',
          contains('The layer is already being used'),
        ),
      ),
    );
    builder2.build();
  }

  // Attempts to retain a layer that has been used as `oldLayer` in a previous frame.
  void testRetainOldLayer(_TestNoSharingFunction pushFunction) {
    final SceneBuilder builder1 = SceneBuilder();
    final EngineLayer layer = pushFunction(builder1, null);
    builder1.pop();
    builder1.build();

    final SceneBuilder builder2 = SceneBuilder();
    pushFunction(builder2, layer);
    builder2.pop();
    expect(
      () {
        final SceneBuilder builder3 = SceneBuilder();
        builder3.addRetained(layer);
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.toString(),
          'toString',
          contains('was previously used as oldLayer'),
        ),
      ),
    );
    builder2.build();
  }

  // Attempts to pass layer as `oldLayer` that has been used as `oldLayer` in a previous frame.
  void testPushOldLayer(_TestNoSharingFunction pushFunction) {
    final SceneBuilder builder1 = SceneBuilder();
    final EngineLayer layer = pushFunction(builder1, null);
    builder1.pop();
    builder1.build();

    final SceneBuilder builder2 = SceneBuilder();
    pushFunction(builder2, layer);
    builder2.pop();
    expect(
      () {
        final SceneBuilder builder3 = SceneBuilder();
        pushFunction(builder3, layer);
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.toString(),
          'toString',
          contains('was previously used as oldLayer'),
        ),
      ),
    );
    builder2.build();
  }

  // Attempts to retain a parent of a layer used as `oldLayer` in a previous frame.
  void testRetainsParentOfOldLayer(_TestNoSharingFunction pushFunction) {
    final SceneBuilder builder1 = SceneBuilder();
    final EngineLayer parentLayer = pushFunction(builder1, null);
    final OpacityEngineLayer childLayer = builder1.pushOpacity(123);
    builder1.pop();
    builder1.pop();
    builder1.build();

    final SceneBuilder builder2 = SceneBuilder();
    builder2.pushOpacity(321, oldLayer: childLayer);
    builder2.pop();
    expect(
      () {
        final SceneBuilder builder3 = SceneBuilder();
        builder3.addRetained(parentLayer);
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.toString(),
          'toString',
          contains('was previously used as oldLayer'),
        ),
      ),
    );
    builder2.build();
  }

  void testNoSharing(_TestNoSharingFunction pushFunction) {
    testPushThenIllegalRetain(pushFunction);
    testAddRetainedThenIllegalPush(pushFunction);
    testDoubleAddRetained(pushFunction);
    testPushOldLayerTwice(pushFunction);
    testPushChildLayerOfRetainedLayer(pushFunction);
    testRetainParentLayerOfPushedChild(pushFunction);
    testRetainOldLayer(pushFunction);
    testPushOldLayer(pushFunction);
    testRetainsParentOfOldLayer(pushFunction);
  }

  test('SceneBuilder does not share a layer between addRetained and push*', () {
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushOffset(0, 0, oldLayer: oldLayer as OffsetEngineLayer?);
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushTransform(Float64List(16), oldLayer: oldLayer as TransformEngineLayer?);
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushClipRect(Rect.zero, oldLayer: oldLayer as ClipRectEngineLayer?);
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushClipRRect(RRect.zero, oldLayer: oldLayer as ClipRRectEngineLayer?);
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushClipPath(Path(), oldLayer: oldLayer as ClipPathEngineLayer?);
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushOpacity(100, oldLayer: oldLayer as OpacityEngineLayer?);
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushBackdropFilter(
        ImageFilter.blur(sigmaX: 1.0),
        oldLayer: oldLayer as BackdropFilterEngineLayer?,
      );
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushShaderMask(
        Gradient.radial(Offset.zero, 10, const <Color>[
          Color.fromARGB(0, 0, 0, 0),
          Color.fromARGB(0, 255, 255, 255),
        ]),
        Rect.zero,
        BlendMode.color,
        oldLayer: oldLayer as ShaderMaskEngineLayer?,
      );
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushColorFilter(
        const ColorFilter.mode(Color.fromARGB(0, 0, 0, 0), BlendMode.color),
        oldLayer: oldLayer as ColorFilterEngineLayer?,
      );
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushColorFilter(
        const ColorFilter.matrix(<double>[
          1,
          0,
          0,
          0,
          0,
          0,
          1,
          0,
          0,
          0,
          0,
          0,
          1,
          0,
          0,
          0,
          0,
          0,
          1,
          0,
        ]),
        oldLayer: oldLayer as ColorFilterEngineLayer?,
      );
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushColorFilter(
        const ColorFilter.linearToSrgbGamma(),
        oldLayer: oldLayer as ColorFilterEngineLayer?,
      );
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushColorFilter(
        const ColorFilter.srgbToLinearGamma(),
        oldLayer: oldLayer as ColorFilterEngineLayer?,
      );
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushImageFilter(
        ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0),
        oldLayer: oldLayer as ImageFilterEngineLayer?,
      );
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushImageFilter(
        ImageFilter.dilate(radiusX: 10.0, radiusY: 10.0),
        oldLayer: oldLayer as ImageFilterEngineLayer?,
      );
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushImageFilter(
        ImageFilter.erode(radiusX: 10.0, radiusY: 10.0),
        oldLayer: oldLayer as ImageFilterEngineLayer?,
      );
    });
    testNoSharing((SceneBuilder builder, EngineLayer? oldLayer) {
      return builder.pushImageFilter(
        ImageFilter.matrix(
          Float64List.fromList(<double>[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]),
        ),
        oldLayer: oldLayer as ImageFilterEngineLayer?,
      );
    });
  });
}

typedef _TestNoSharingFunction = EngineLayer Function(SceneBuilder builder, EngineLayer? oldLayer);
