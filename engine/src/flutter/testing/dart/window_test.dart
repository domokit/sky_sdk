// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';
import 'dart:ui';

import 'package:test/test.dart';

void main() {
  test('window.sendPlatformMessage preserves callback zone', () {
    runZoned(() {
      final Zone innerZone = Zone.current;
      PlatformDispatcher.instance.sendPlatformMessage(
        'test',
        ByteData.view(Uint8List(0).buffer),
        expectAsync1((ByteData? data) {
          final Zone runZone = Zone.current;
          expect(runZone, isNotNull);
          expect(runZone, same(innerZone));
        }),
      );
    });
  });

  test('FrameTiming.toString has the correct format', () {
    final FrameTiming timing = FrameTiming(
      vsyncStart: 500,
      buildStart: 1000,
      buildFinish: 8000,
      rasterStart: 9000,
      rasterFinish: 19500,
      rasterFinishWallTime: 19501,
      frameNumber: 23,
    );
    expect(
      timing.toString(),
      'FrameTiming(buildDuration: 7.0ms, '
      'rasterDuration: 10.5ms, '
      'vsyncOverhead: 0.5ms, '
      'totalSpan: 19.0ms, '
      'layerCacheCount: 0, '
      'layerCacheBytes: 0, '
      'pictureCacheCount: 0, '
      'pictureCacheBytes: 0, '
      'frameNumber: 23)',
    );
  });

  test('FrameTiming.toString with cache statistics has the correct format', () {
    final FrameTiming timing = FrameTiming(
      vsyncStart: 500,
      buildStart: 1000,
      buildFinish: 8000,
      rasterStart: 9000,
      rasterFinish: 19500,
      rasterFinishWallTime: 19501,
      layerCacheCount: 5,
      layerCacheBytes: 200000,
      pictureCacheCount: 3,
      pictureCacheBytes: 300000,
      frameNumber: 29,
    );
    expect(
      timing.toString(),
      'FrameTiming(buildDuration: 7.0ms, '
      'rasterDuration: 10.5ms, '
      'vsyncOverhead: 0.5ms, '
      'totalSpan: 19.0ms, '
      'layerCacheCount: 5, '
      'layerCacheBytes: 200000, '
      'pictureCacheCount: 3, '
      'pictureCacheBytes: 300000, '
      'frameNumber: 29)',
    );
  });

  test('computePlatformResolvedLocale basic', () {
    final List<Locale> supportedLocales = <Locale>[
      const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'CN'),
      const Locale.fromSubtags(languageCode: 'fr', countryCode: 'FR'),
      const Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
      const Locale.fromSubtags(languageCode: 'en'),
    ];
    // The default implementation returns null due to lack of a real platform.
    final Locale? result = PlatformDispatcher.instance.computePlatformResolvedLocale(
      supportedLocales,
    );
    expect(result, null);
  });

  test('Display is configured for the implicitView', () {
    final FlutterView implicitView = PlatformDispatcher.instance.implicitView!;
    final Display display = implicitView.display;

    expect(display.id, 0);
    expect(display.devicePixelRatio, implicitView.devicePixelRatio);
    expect(display.refreshRate, 60);
    expect(display.size, implicitView.physicalSize);
  });

  test('FlutterView.toString contains the viewId', () {
    final FlutterView flutterView = PlatformDispatcher.instance.implicitView!;
    expect(flutterView.viewId, 0);
    expect(flutterView.toString(), 'FlutterView(id: 0)');
  });

  test('scaleFontSize is the identity function by default when textScaleFactor = 1', () {
    expect(PlatformDispatcher.instance.scaleFontSize(0), 0.0);
    expect(PlatformDispatcher.instance.scaleFontSize(1), 1.0);
    expect(PlatformDispatcher.instance.scaleFontSize(2), 2.0);
    expect(PlatformDispatcher.instance.scaleFontSize(3), 3.0);
    expect(PlatformDispatcher.instance.scaleFontSize(3.4), 3.4);
  });
}
