// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

Future<void> main() async {
  final ui.Image image = await createTestImage();

  testWidgets('didHaveMemoryPressure clears imageCache', (WidgetTester tester) async {
    imageCache.putIfAbsent(
      1,
      () => OneFrameImageStreamCompleter(Future<ImageInfo>.value(ImageInfo(image: image))),
    );

    await tester.idle();
    expect(imageCache.currentSize, 1);
    final ByteData message =
        const JSONMessageCodec().encodeMessage(<String, dynamic>{'type': 'memoryPressure'})!;
    await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/system',
      message,
      (_) {},
    );
    expect(imageCache.currentSize, 0);
  });

  test('evict clears live references', () async {
    final TestPaintingBinding binding = TestPaintingBinding();
    expect(binding.imageCache.clearCount, 0);
    expect(binding.imageCache.liveClearCount, 0);

    binding.evict('/path/to/asset.png');
    expect(binding.imageCache.clearCount, 1);
    expect(binding.imageCache.liveClearCount, 1);
  });

  test('ShaderWarmUp is null by default', () {
    expect(PaintingBinding.shaderWarmUp, null);
  });
}

class TestBindingBase implements BindingBase {
  @override
  void initInstances() {}

  @override
  bool debugCheckZone(String entryPoint) {
    return true;
  }

  @override
  void initServiceExtensions() {}

  @override
  Future<void> lockEvents(Future<void> Function() callback) async {}

  @override
  bool get locked => throw UnimplementedError();

  @override
  Future<void> performReassemble() {
    throw UnimplementedError();
  }

  @override
  void postEvent(String eventKind, Map<String, dynamic> eventData) {}

  @override
  Future<void> reassembleApplication() {
    throw UnimplementedError();
  }

  @override
  void registerBoolServiceExtension({
    required String name,
    required AsyncValueGetter<bool> getter,
    required AsyncValueSetter<bool> setter,
  }) {}

  @override
  void registerNumericServiceExtension({
    required String name,
    required AsyncValueGetter<double> getter,
    required AsyncValueSetter<double> setter,
  }) {}

  @override
  void registerServiceExtension({
    required String name,
    required ServiceExtensionCallback callback,
  }) {}

  @override
  void registerSignalServiceExtension({required String name, required AsyncCallback callback}) {}

  @override
  void registerStringServiceExtension({
    required String name,
    required AsyncValueGetter<String> getter,
    required AsyncValueSetter<String> setter,
  }) {}

  @override
  void unlocked() {}

  @override
  ui.SingletonFlutterWindow get window => throw UnimplementedError();

  @override
  ui.PlatformDispatcher get platformDispatcher => throw UnimplementedError();
}

class TestPaintingBinding extends TestBindingBase
    with SchedulerBinding, ServicesBinding, PaintingBinding {
  @override
  final FakeImageCache imageCache = FakeImageCache();

  @override
  ImageCache createImageCache() => imageCache;
}

class FakeImageCache extends ImageCache {
  int clearCount = 0;
  int liveClearCount = 0;

  @override
  void clear() {
    clearCount += 1;
    super.clear();
  }

  @override
  void clearLiveImages() {
    liveClearCount += 1;
    super.clearLiveImages();
  }
}
