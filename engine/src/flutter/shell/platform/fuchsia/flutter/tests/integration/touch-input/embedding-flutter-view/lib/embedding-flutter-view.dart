// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'dart:io';
import 'dart:ui';
import 'dart:zircon';

import 'package:args/args.dart';
import 'package:vector_math/vector_math_64.dart' as vector_math_64;

final _argsCsvFilePath = '/config/data/args.csv';

void main(List<String> args) async {
  print('Launching embedding-flutter-view');

  args = args + _GetArgsFromConfigFile();
  final parser =
      ArgParser()
        ..addFlag('showOverlay', defaultsTo: false)
        ..addFlag('focusable', defaultsTo: true);

  final arguments = parser.parse(args);
  for (final option in arguments.options) {
    print('embedding-flutter-view args: $option: ${arguments[option]}');
  }

  TestApp app = TestApp(
    ChildView(await _launchChildView()),
    showOverlay: arguments['showOverlay'],
    focusable: arguments['focusable'],
  );

  app.run();
}

class TestApp {
  static const _black = Color.fromARGB(255, 0, 0, 0);
  static const _blue = Color.fromARGB(255, 0, 0, 255);

  final ChildView childView;
  final bool showOverlay;
  final bool focusable;

  Color _backgroundColor = _blue;

  TestApp(this.childView, {this.showOverlay = false, this.focusable = true}) {}

  void run() {
    childView.create(focusable, (ByteData? reply) {
      // Set up window callbacks.
      window.onPointerDataPacket = (PointerDataPacket packet) {
        this.pointerDataPacket(packet);
      };
      window.onMetricsChanged = () {
        window.scheduleFrame();
      };
      window.onBeginFrame = (Duration duration) {
        this.beginFrame(duration);
      };

      // The child view should be attached to Scenic now.
      // Ready to build the scene.
      window.scheduleFrame();
    });
  }

  void beginFrame(Duration duration) {
    // Convert physical screen size of device to values
    final pixelRatio = window.devicePixelRatio;
    final size = window.physicalSize / pixelRatio;
    final physicalBounds = Offset.zero & window.physicalSize;
    final windowBounds = Offset.zero & size;
    // Set up a Canvas that uses the screen size
    final recorder = PictureRecorder();
    final canvas = Canvas(recorder, physicalBounds);
    canvas.scale(pixelRatio);
    // Draw something
    final paint = Paint()..color = this._backgroundColor;
    canvas.drawRect(windowBounds, paint);
    final picture = recorder.endRecording();
    // Build the scene
    final sceneBuilder =
        SceneBuilder()
          ..pushClipRect(physicalBounds)
          ..addPicture(Offset.zero, picture);

    final childPhysicalSize = window.physicalSize * 0.25;
    // Alignment.center
    final windowCenter = size.center(Offset.zero);
    final windowPhysicalCenter = window.physicalSize.center(Offset.zero);
    final childPhysicalOffset = windowPhysicalCenter - childPhysicalSize.center(Offset.zero);

    sceneBuilder
      ..pushTransform(
        vector_math_64.Matrix4.translationValues(
          childPhysicalOffset.dx,
          childPhysicalOffset.dy,
          0.0,
        ).storage,
      )
      ..addPlatformView(
        childView.viewId,
        width: childPhysicalSize.width,
        height: childPhysicalSize.height,
      )
      ..pop();

    if (showOverlay) {
      final containerSize = size * 0.5;
      // Alignment.center
      final containerOffset = windowCenter - containerSize.center(Offset.zero);

      final overlaySize = containerSize * 0.5;
      // Alignment.topRight
      final overlayOffset = Offset(
        containerOffset.dx + containerSize.width - overlaySize.width,
        containerOffset.dy,
      );
      final overlayPhysicalSize = overlaySize * pixelRatio;
      final overlayPhysicalOffset = overlayOffset * pixelRatio;
      final overlayPhysicalBounds = overlayPhysicalOffset & overlayPhysicalSize;

      final recorder = PictureRecorder();
      final overlayCullRect = Offset.zero & overlayPhysicalSize; // in canvas physical coordinates
      final canvas = Canvas(recorder, overlayCullRect);
      canvas.scale(pixelRatio);

      final paint = Paint()..color = Color.fromARGB(255, 0, 255, 0);
      canvas.drawRect(Offset.zero & overlaySize, paint);

      final overlayPicture = recorder.endRecording();
      sceneBuilder
        ..pushClipRect(overlayPhysicalBounds) // in window physical coordinates
        ..addPicture(overlayPhysicalOffset, overlayPicture)
        ..pop();
    }

    sceneBuilder.pop();
    window.render(sceneBuilder.build());
  }

  void pointerDataPacket(PointerDataPacket packet) async {
    int nowNanos = System.clockGetMonotonic();

    for (PointerData data in packet.data) {
      print('embedding-flutter-view received tap: ${data.toStringFull()}');

      if (data.change == PointerChange.down) {
        this._backgroundColor = _black;
      }

      if (data.change == PointerChange.down || data.change == PointerChange.move) {
        _reportTouchInput(localX: data.physicalX, localY: data.physicalY, timeReceived: nowNanos);
      }
    }

    window.scheduleFrame();
  }

  void _reportTouchInput({
    required double localX,
    required double localY,
    required int timeReceived,
  }) {
    print('embedding-flutter-view reporting touch input to TouchInputListener');
    final message =
        utf8
            .encode(
              json.encode({
                'method': 'TouchInputListener.ReportTouchInput',
                'local_x': localX,
                'local_y': localY,
                'time_received': timeReceived,
                'component_name': 'embedding-flutter-view',
              }),
            )
            .buffer
            .asByteData();
    PlatformDispatcher.instance.sendPlatformMessage('fuchsia/input_test', message, null);
  }
}

class ChildView {
  final int viewId;

  ChildView(this.viewId);

  void create(bool focusable, PlatformMessageResponseCallback callback) {
    // Construct the dart:ui platform message to create the view, and when the
    // return callback is invoked, build the scene. At that point, it is safe
    // to embed the child view in the scene.
    final viewOcclusionHint = Rect.zero;
    final Map<String, dynamic> args = <String, dynamic>{
      'viewId': viewId,
      // Flatland doesn't support disabling hit testing.
      'hitTestable': true,
      'focusable': focusable,
      'viewOcclusionHintLTRB': <double>[
        viewOcclusionHint.left,
        viewOcclusionHint.top,
        viewOcclusionHint.right,
        viewOcclusionHint.bottom,
      ],
    };

    final ByteData createViewMessage =
        utf8
            .encode(json.encode(<String, Object>{'method': 'View.create', 'args': args}))
            .buffer
            .asByteData();

    final platformViewsChannel = 'flutter/platform_views';

    PlatformDispatcher.instance.sendPlatformMessage(
      platformViewsChannel,
      createViewMessage,
      callback,
    );
  }
}

Future<int> _launchChildView() async {
  final message = Int8List.fromList([0x31]);
  final completer = new Completer<ByteData>();
  PlatformDispatcher.instance.sendPlatformMessage(
    'fuchsia/child_view',
    ByteData.sublistView(message),
    (ByteData? reply) {
      completer.complete(reply!);
    },
  );

  return int.parse(ascii.decode(((await completer.future).buffer.asUint8List())));
}

List<String> _GetArgsFromConfigFile() {
  List<String> args;
  final f = File(_argsCsvFilePath);
  if (!f.existsSync()) {
    return List.empty();
  }
  final fileContentCsv = f.readAsStringSync();
  args = fileContentCsv.split('\n');
  return args;
}
