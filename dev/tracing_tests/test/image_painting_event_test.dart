// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert' show jsonEncode;
import 'dart:developer' as developer;
import 'dart:ui' as ui;

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vm_service/vm_service.dart';
import 'package:vm_service/vm_service_io.dart';

void main() {
  LiveTestWidgetsFlutterBinding.ensureInitialized();

  late VmService vmService;
  late LiveTestWidgetsFlutterBinding binding;
  setUpAll(() async {
    final developer.ServiceProtocolInfo info = await developer.Service.getInfo();

    if (info.serverUri == null) {
      fail('This test _must_ be run with --enable-vmservice.');
    }

    vmService = await vmServiceConnectUri(
      'ws://localhost:${info.serverUri!.port}${info.serverUri!.path}ws',
    );
    await vmService.streamListen(EventStreams.kExtension);

    // Initialize bindings
    binding = LiveTestWidgetsFlutterBinding.instance;
    binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.fullyLive;
    binding.attachRootWidget(const SizedBox.expand());
    expect(binding.framesEnabled, true);
    // Pump two frames to make sure we clear out any inter-frame comparisons.
    await binding.endOfFrame;
    await binding.endOfFrame;
  });

  tearDownAll(() async {
    await vmService.dispose();
  });

  test('Image painting events - deduplicates across frames', () async {
    final Completer<Event> completer = Completer<Event>();
    vmService.onExtensionEvent
        .firstWhere((Event event) => event.extensionKind == 'Flutter.ImageSizesForFrame')
        .then(completer.complete);

    final ui.Image image = await createTestImage(width: 300, height: 300);
    final TestCanvas canvas = TestCanvas();
    paintImage(
      canvas: canvas,
      rect: const Rect.fromLTWH(50.0, 75.0, 200.0, 100.0),
      image: image,
      debugImageLabel: 'test.png',
    );

    // Make sure that we don't report an identical image size info if we
    // redraw in the next frame.
    await binding.endOfFrame;

    paintImage(
      canvas: canvas,
      rect: const Rect.fromLTWH(50.0, 75.0, 200.0, 100.0),
      image: image,
      debugImageLabel: 'test.png',
    );
    await binding.endOfFrame;

    final Event event = await completer.future;
    expect(event.extensionKind, 'Flutter.ImageSizesForFrame');
    expect(
      jsonEncode(event.extensionData!.data),
      contains(
        '"test.png":{"source":"test.png","displaySize":{"width":600.0,"height":300.0},"imageSize":{"width":300.0,"height":300.0},"displaySizeInBytes":960000,"decodedSizeInBytes":480000}',
      ),
    );
  }, skip: isBrowser); // [intended] uses dart:isolate and io.

  test('Image painting events - deduplicates across frames', () async {
    final Completer<Event> completer = Completer<Event>();
    vmService.onExtensionEvent
        .firstWhere((Event event) => event.extensionKind == 'Flutter.ImageSizesForFrame')
        .then(completer.complete);

    final ui.Image image = await createTestImage(width: 300, height: 300);
    final TestCanvas canvas = TestCanvas();
    paintImage(
      canvas: canvas,
      rect: const Rect.fromLTWH(50.0, 75.0, 200.0, 100.0),
      image: image,
      debugImageLabel: 'test.png',
    );

    paintImage(
      canvas: canvas,
      rect: const Rect.fromLTWH(50.0, 75.0, 300.0, 300.0),
      image: image,
      debugImageLabel: 'test.png',
    );
    await binding.endOfFrame;

    final Event event = await completer.future;
    expect(event.extensionKind, 'Flutter.ImageSizesForFrame');
    expect(
      jsonEncode(event.extensionData!.data),
      contains(
        '"test.png":{"source":"test.png","displaySize":{"width":900.0,"height":900.0},"imageSize":{"width":300.0,"height":300.0},"displaySizeInBytes":4320000,"decodedSizeInBytes":480000}',
      ),
    );
  }, skip: isBrowser); // [intended] uses dart:isolate and io.
}

class TestCanvas implements Canvas {
  @override
  void noSuchMethod(Invocation invocation) {}
}
