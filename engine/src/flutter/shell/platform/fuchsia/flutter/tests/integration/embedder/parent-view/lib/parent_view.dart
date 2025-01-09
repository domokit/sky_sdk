// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'dart:io';
import 'dart:ui';

import 'package:args/args.dart';
import 'package:vector_math/vector_math_64.dart' as vector_math_64;

final _argsCsvFilePath = '/config/data/args.csv';

void main(List<String> args) async {
  print('parent-view: starting');

  args = args + _GetArgsFromConfigFile();
  final parser =
      ArgParser()
        ..addFlag('showOverlay', defaultsTo: false)
        ..addFlag('focusable', defaultsTo: true);
  final arguments = parser.parse(args);
  for (final option in arguments.options) {
    print('parent-view: $option: ${arguments[option]}');
  }

  TestApp app;
  app = TestApp(
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
      // Set up window allbacks.
      window.onPointerDataPacket = (PointerDataPacket packet) {
        for (final data in packet.data) {
          if (data.change == PointerChange.down) {
            this._backgroundColor = _black;
          }
        }
        window.scheduleFrame();
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
    final windowPhysicalBounds = Offset.zero & window.physicalSize;
    final pixelRatio = window.devicePixelRatio;
    final windowSize = window.physicalSize / pixelRatio;
    final windowBounds = Offset.zero & windowSize;

    final recorder = PictureRecorder();
    final canvas = Canvas(recorder, windowPhysicalBounds);
    canvas.scale(pixelRatio);
    final paint = Paint()..color = this._backgroundColor;
    canvas.drawRect(windowBounds, paint);
    final picture = recorder.endRecording();

    final sceneBuilder =
        SceneBuilder()
          ..pushClipRect(windowPhysicalBounds)
          ..addPicture(Offset.zero, picture);

    final childPhysicalSize = window.physicalSize * 0.33;
    // Alignment.center
    final windowCenter = windowSize.center(Offset.zero);
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
      final containerSize = windowSize * .66;
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
}

class ChildView {
  final int viewId;

  ChildView(this.viewId);

  void create(bool focusable, PlatformMessageResponseCallback callback) {
    // Construct the dart:ui platform message to create the view, and when the
    // return callback is invoked, build the scene. At that point, it is safe
    // to embed the child-view2 in the scene.
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

    final ByteData createViewMessage = ByteData.sublistView(
      utf8.encode(json.encode(<String, Object>{'method': 'View.create', 'args': args})),
    );

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
