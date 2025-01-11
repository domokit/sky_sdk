// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';
import 'dart:ui';

import 'channel_util.dart';

import 'scenario.dart';

/// A blank page with a button that pops the page when tapped.
class PoppableScreenScenario extends Scenario {
  /// Creates the PoppableScreenScenario.
  PoppableScreenScenario(super.view) {
    channelBuffers.setListener('flutter/platform', _onHandlePlatformMessage);
  }

  // Rect for the pop button. Only defined once onMetricsChanged is called.
  Rect? _buttonRect;

  @override
  void onBeginFrame(Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);

    canvas.drawPaint(Paint()..color = const Color.fromARGB(255, 255, 255, 255));

    if (_buttonRect != null) {
      canvas.drawRect(_buttonRect!, Paint()..color = const Color.fromARGB(255, 255, 0, 0));
    }
    final Picture picture = recorder.endRecording();

    builder.pushOffset(0, 0);
    builder.addPicture(Offset.zero, picture);
    final Scene scene = builder.build();
    view.render(scene);
    scene.dispose();
  }

  @override
  void onDrawFrame() {
    // Just draw once since the content never changes.
  }

  @override
  void onMetricsChanged() {
    _buttonRect = Rect.fromLTRB(
      view.physicalSize.width / 4,
      view.physicalSize.height * 2 / 5,
      view.physicalSize.width * 3 / 4,
      view.physicalSize.height * 3 / 5,
    );
    view.platformDispatcher.scheduleFrame();
  }

  @override
  void onPointerDataPacket(PointerDataPacket packet) {
    for (final PointerData data in packet.data) {
      if (data.change == PointerChange.up &&
          (_buttonRect?.contains(Offset(data.physicalX, data.physicalY)) ?? false)) {
        _pop();
      }
    }
  }

  void _pop() {
    sendJsonMethodCall(
      dispatcher: view.platformDispatcher,
      // 'flutter/platform' is the hardcoded name of the 'platform'
      // `SystemChannel` from the `SystemNavigator` API.
      // https://github.com/flutter/flutter/blob/master/packages/flutter/lib/src/services/system_navigator.dart.
      channel: 'flutter/platform',
      method: 'SystemNavigator.pop',
      // Don't care about the response. If it doesn't go through, the test
      // will fail.
    );
  }

  void _onHandlePlatformMessage(ByteData? data, PlatformMessageResponseCallback callback) {
    view.platformDispatcher.sendPlatformMessage('flutter/platform', data, null);
  }

  @override
  void unmount() {
    channelBuffers.clearListener('flutter/platform');
    super.unmount();
  }
}
