// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:web_engine_tester/golden_tester.dart';

import '../common/test_initialization.dart';
import 'utils.dart';

const ui.Color black = ui.Color(0xFF000000);
const ui.Color red = ui.Color(0xFFFF0000);
const ui.Color green = ui.Color(0xFF00FF00);
const ui.Color blue = ui.Color(0xFF0000FF);

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests();

  setUp(() {
    EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(1.0);
  });

  test('can render into multiple views', () async {
    const ui.Rect rect = ui.Rect.fromLTRB(0, 0, 180, 120);

    final host1 = createHostElement(rect);
    final view1 = EngineFlutterView(EnginePlatformDispatcher.instance, host1);
    EnginePlatformDispatcher.instance.viewManager.registerView(view1);

    final host2 = createHostElement(rect);
    final view2 = EngineFlutterView(EnginePlatformDispatcher.instance, host2);
    EnginePlatformDispatcher.instance.viewManager.registerView(view2);

    final host3 = createHostElement(rect);
    final view3 = EngineFlutterView(EnginePlatformDispatcher.instance, host3);
    EnginePlatformDispatcher.instance.viewManager.registerView(view3);

    await Future.wait([
      renderer.renderScene(paintRect(rect, red), view1),
      renderer.renderScene(paintRect(rect, green), view2),
      renderer.renderScene(paintRect(rect, blue), view3),
    ]);

    await matchGoldenFile(
      'ui_multiview_rects.png',
      region: ui.Rect.fromLTRB(0, 0, rect.width, rect.height * 3),
    );

    EnginePlatformDispatcher.instance.viewManager.dispose();
    host1.remove();
    host2.remove();
    host3.remove();
  }, skip: isHtml); // HTML renderer doesn't support multi-view.
}

DomElement createHostElement(ui.Rect rect) {
  final host = createDomElement('div');
  host.style
    ..width = '${rect.width}px'
    ..height = '${rect.height}px';
  domDocument.body!.append(host);
  return host;
}

ui.Scene paintRect(ui.Rect rect, ui.Color color) {
  final recorder = ui.PictureRecorder();
  final canvas = ui.Canvas(recorder, rect);

  // Leave some padding.
  rect = rect.deflate(5.0);

  // Draw a black border of 5px thickness.
  canvas.drawRect(
    rect,
    ui.Paint()
      ..color = black
      ..style = ui.PaintingStyle.fill,
  );
  rect = rect.deflate(5.0);

  // Fill the inner rect with the given color.
  canvas.drawRect(
    rect,
    ui.Paint()
      ..color = color
      ..style = ui.PaintingStyle.fill,
  );

  final picture = recorder.endRecording();
  final sb = ui.SceneBuilder();
  sb.pushOffset(0, 0);
  sb.addPicture(ui.Offset.zero, picture);
  return sb.build();
}
