// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('!chrome')
library;

import 'dart:convert';
import 'dart:ui' as ui show Image;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import '../image_data.dart';

ByteData testByteData(double scale) => ByteData(8)..setFloat64(0, scale);
double scaleOf(ByteData data) => data.getFloat64(0);

final Map<Object?, Object?> testManifest = <Object?, Object?>{
  'assets/image.png': <Map<String, Object>>[
    <String, String>{'asset': 'assets/image.png'},
    <String, Object>{'asset': 'assets/1.5x/image.png', 'dpr': 1.5},
    <String, Object>{'asset': 'assets/2.0x/image.png', 'dpr': 2.0},
    <String, Object>{'asset': 'assets/3.0x/image.png', 'dpr': 3.0},
    <String, Object>{'asset': 'assets/4.0x/image.png', 'dpr': 4.0},
  ],
};

class TestAssetBundle extends CachingAssetBundle {
  TestAssetBundle({required Map<Object?, Object?> manifest}) {
    this.manifest = const StandardMessageCodec().encodeMessage(manifest)!;
  }

  late final ByteData manifest;

  @override
  Future<ByteData> load(String key) {
    final ByteData data = switch (key) {
      'AssetManifest.bin' => manifest,
      'assets/image.png' => testByteData(1.0),
      'assets/1.0x/image.png' => testByteData(10.0), // see "...with a main asset and a 1.0x asset"
      'assets/1.5x/image.png' => testByteData(1.5),
      'assets/2.0x/image.png' => testByteData(2.0),
      'assets/3.0x/image.png' => testByteData(3.0),
      'assets/4.0x/image.png' => testByteData(4.0),
      _ => throw ArgumentError('Unexpected key: $key'),
    };
    return SynchronousFuture<ByteData>(data);
  }

  @override
  String toString() => '${describeIdentity(this)}()';
}

class FakeImageStreamCompleter extends ImageStreamCompleter {
  FakeImageStreamCompleter(Future<ImageInfo> image) {
    image.then<void>(setImage);
  }
}

class TestAssetImage extends AssetImage {
  const TestAssetImage(super.assetName, this.images);

  final Map<double, ui.Image> images;

  @override
  ImageStreamCompleter loadImage(AssetBundleImageKey key, ImageDecoderCallback decode) {
    late ImageInfo imageInfo;
    key.bundle.load(key.name).then<void>((ByteData data) {
      final ui.Image image = images[scaleOf(data)]!;
      imageInfo = ImageInfo(image: image, scale: key.scale);
    });
    return FakeImageStreamCompleter(SynchronousFuture<ImageInfo>(imageInfo));
  }
}

Widget buildImageAtRatio(
  String imageName,
  Key key,
  double ratio,
  bool inferSize,
  Map<double, ui.Image> images, [
  AssetBundle? bundle,
]) {
  const double windowSize = 500.0; // 500 logical pixels
  const double imageSize = 200.0; // 200 logical pixels

  return MediaQuery(
    data: MediaQueryData(size: const Size(windowSize, windowSize), devicePixelRatio: ratio),
    child: DefaultAssetBundle(
      bundle: bundle ?? TestAssetBundle(manifest: testManifest),
      child: Center(
        child:
            inferSize
                ? Image(
                  key: key,
                  excludeFromSemantics: true,
                  image: TestAssetImage(imageName, images),
                )
                : Image(
                  key: key,
                  excludeFromSemantics: true,
                  image: TestAssetImage(imageName, images),
                  height: imageSize,
                  width: imageSize,
                  fit: BoxFit.fill,
                ),
      ),
    ),
  );
}

Widget buildImageCacheResized(
  String name,
  Key key,
  int width,
  int height,
  int cacheWidth,
  int cacheHeight,
) {
  return Center(
    child: RepaintBoundary(
      child: SizedBox(
        width: 250,
        height: 250,
        child: Center(
          child: Image.memory(
            Uint8List.fromList(kTransparentImage),
            key: key,
            excludeFromSemantics: true,
            color: const Color(0xFF00FFFF),
            colorBlendMode: BlendMode.plus,
            width: width.toDouble(),
            height: height.toDouble(),
            cacheWidth: cacheWidth,
            cacheHeight: cacheHeight,
          ),
        ),
      ),
    ),
  );
}

RenderImage getRenderImage(WidgetTester tester, Key key) {
  return tester.renderObject<RenderImage>(find.byKey(key));
}

Future<void> pumpTreeToLayout(WidgetTester tester, Widget widget) {
  return tester.pumpWidget(widget, duration: Duration.zero, phase: EnginePhase.layout);
}

void main() {
  const String image = 'assets/image.png';

  final Map<double, ui.Image> images = <double, ui.Image>{};

  setUpAll(() async {
    for (final double scale in const <double>[0.5, 1.0, 1.5, 2.0, 4.0, 10.0]) {
      final int dimension = (48 * scale).floor();
      images[scale] = await createTestImage(width: dimension, height: dimension);
    }
  });

  tearDownAll(() {
    for (final ui.Image image in images.values) {
      image.dispose();
    }
  });

  testWidgets('Image for device pixel ratio 1.0', (WidgetTester tester) async {
    const double ratio = 1.0;
    Key key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, false, images));
    expect(getRenderImage(tester, key).size, const Size(200.0, 200.0));
    expect(getRenderImage(tester, key).scale, 1.0);
    key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, true, images));
    expect(getRenderImage(tester, key).size, const Size(48.0, 48.0));
    expect(getRenderImage(tester, key).scale, 1.0);
  });

  testWidgets('Image for device pixel ratio 0.5', (WidgetTester tester) async {
    const double ratio = 0.5;
    Key key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, false, images));
    expect(getRenderImage(tester, key).size, const Size(200.0, 200.0));
    expect(getRenderImage(tester, key).scale, 1.0);
    key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, true, images));
    expect(getRenderImage(tester, key).size, const Size(48.0, 48.0));
    expect(getRenderImage(tester, key).scale, 1.0);
  });

  testWidgets('Image for device pixel ratio 1.5', (WidgetTester tester) async {
    const double ratio = 1.5;
    Key key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, false, images));
    expect(getRenderImage(tester, key).size, const Size(200.0, 200.0));
    expect(getRenderImage(tester, key).scale, 1.5);
    key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, true, images));
    expect(getRenderImage(tester, key).size, const Size(48.0, 48.0));
    expect(getRenderImage(tester, key).scale, 1.5);
  });

  // A 1.75 DPR screen is typically a low-resolution screen, such that physical
  // pixels are visible to the user. For such screens we prefer to pick the
  // higher resolution image, if available.
  testWidgets('Image for device pixel ratio 1.75', (WidgetTester tester) async {
    const double ratio = 1.75;
    Key key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, false, images));
    expect(getRenderImage(tester, key).size, const Size(200.0, 200.0));
    expect(getRenderImage(tester, key).scale, 2.0);
    key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, true, images));
    expect(getRenderImage(tester, key).size, const Size(48.0, 48.0));
    expect(getRenderImage(tester, key).scale, 2.0);
  });

  testWidgets('Image for device pixel ratio 2.3', (WidgetTester tester) async {
    const double ratio = 2.3;
    Key key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, false, images));
    expect(getRenderImage(tester, key).size, const Size(200.0, 200.0));
    expect(getRenderImage(tester, key).scale, 2.0);
    key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, true, images));
    expect(getRenderImage(tester, key).size, const Size(48.0, 48.0));
    expect(getRenderImage(tester, key).scale, 2.0);
  });

  testWidgets('Image for device pixel ratio 3.7', (WidgetTester tester) async {
    const double ratio = 3.7;
    Key key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, false, images));
    expect(getRenderImage(tester, key).size, const Size(200.0, 200.0));
    expect(getRenderImage(tester, key).scale, 4.0);
    key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, true, images));
    expect(getRenderImage(tester, key).size, const Size(48.0, 48.0));
    expect(getRenderImage(tester, key).scale, 4.0);
  });

  testWidgets('Image for device pixel ratio 5.1', (WidgetTester tester) async {
    const double ratio = 5.1;
    Key key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, false, images));
    expect(getRenderImage(tester, key).size, const Size(200.0, 200.0));
    expect(getRenderImage(tester, key).scale, 4.0);
    key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, true, images));
    expect(getRenderImage(tester, key).size, const Size(48.0, 48.0));
    expect(getRenderImage(tester, key).scale, 4.0);
  });

  testWidgets('Image for device pixel ratio 1.0, with a main asset and a 1.0x asset', (
    WidgetTester tester,
  ) async {
    // If both a main asset and a 1.0x asset are specified, then prefer
    // the 1.0x asset.

    final Map<Object?, Object?> manifest =
        json.decode('''
    {
      "assets/image.png" : [
        {"asset": "assets/1.0x/image.png", "dpr": 1.0},
        {"asset": "assets/1.5x/image.png", "dpr": 1.5},
        {"asset": "assets/2.0x/image.png", "dpr": 2.0},
        {"asset": "assets/3.0x/image.png", "dpr": 3.0},
        {"asset": "assets/4.0x/image.png", "dpr": 4.0}
      ]
    }
    ''')
            as Map<Object?, Object?>;
    final AssetBundle bundle = TestAssetBundle(manifest: manifest);

    const double ratio = 1.0;
    Key key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, false, images, bundle));
    expect(getRenderImage(tester, key).size, const Size(200.0, 200.0));
    // Verify we got the 10x scaled image, since the test ByteData said it should be 10x.
    expect(getRenderImage(tester, key).image!.height, 480);
    key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, true, images, bundle));
    expect(getRenderImage(tester, key).size, const Size(480.0, 480.0));
    // Verify we got the 10x scaled image, since the test ByteData said it should be 10x.
    expect(getRenderImage(tester, key).image!.height, 480);
  });

  testWidgets('Image cache resize upscale display 5', (WidgetTester tester) async {
    final Key key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageCacheResized(image, key, 5, 5, 20, 20));
    expect(getRenderImage(tester, key).size, const Size(5.0, 5.0));
  });

  testWidgets('Image cache resize upscale display 50', (WidgetTester tester) async {
    final Key key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageCacheResized(image, key, 50, 50, 20, 20));
    expect(getRenderImage(tester, key).size, const Size(50.0, 50.0));
  });

  testWidgets('Image cache resize downscale display 5', (WidgetTester tester) async {
    final Key key = GlobalKey();
    await pumpTreeToLayout(tester, buildImageCacheResized(image, key, 5, 5, 1, 1));
    expect(getRenderImage(tester, key).size, const Size(5.0, 5.0));
  });

  // For low-resolution screens we prefer higher-resolution images due to
  // visible physical pixel size (see the test for 1.75 DPR above). However,
  // if higher resolution assets are not available we will pick the best
  // available.
  testWidgets('Low-resolution assets', (WidgetTester tester) async {
    const Map<Object?, Object?> manifest = <Object?, Object?>{
      'assets/image.png': <Map<String, Object>>[
        <String, Object>{'asset': 'assets/image.png'},
        <String, Object>{'asset': 'assets/1.5x/image.png', 'dpr': 1.5},
      ],
    };
    final AssetBundle bundle = TestAssetBundle(manifest: manifest);

    Future<void> testRatio({required double ratio, required double expectedScale}) async {
      Key key = GlobalKey();
      await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, false, images, bundle));
      expect(getRenderImage(tester, key).size, const Size(200.0, 200.0));
      expect(getRenderImage(tester, key).scale, expectedScale);
      key = GlobalKey();
      await pumpTreeToLayout(tester, buildImageAtRatio(image, key, ratio, true, images, bundle));
      expect(getRenderImage(tester, key).size, const Size(48.0, 48.0));
      expect(getRenderImage(tester, key).scale, expectedScale);
    }

    // Choose higher resolution image as it's the lowest available.
    await testRatio(ratio: 0.25, expectedScale: 1.0);
    await testRatio(ratio: 0.5, expectedScale: 1.0);
    await testRatio(ratio: 0.75, expectedScale: 1.0);
    await testRatio(ratio: 1.0, expectedScale: 1.0);

    // Choose higher resolution image even though a lower resolution
    // image is closer.
    await testRatio(ratio: 1.20, expectedScale: 1.5);

    // Choose higher resolution image because it's closer.
    await testRatio(ratio: 1.25, expectedScale: 1.5);
    await testRatio(ratio: 1.5, expectedScale: 1.5);

    // Choose lower resolution image because no higher resolution assets
    // are not available.
    await testRatio(ratio: 1.75, expectedScale: 1.5);
    await testRatio(ratio: 2.0, expectedScale: 1.5);
    await testRatio(ratio: 2.25, expectedScale: 1.5);
    await testRatio(ratio: 10.0, expectedScale: 1.5);
  });
}
