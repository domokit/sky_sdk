// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide window;

import '../../common/test_initialization.dart';
import 'helper.dart';

const Rect bounds = Rect.fromLTWH(0, 0, 800, 600);

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(
    withImplicitView: true,
    emulateTesterEnvironment: false,
    setUpTestViewDimensions: false,
  );

  test('paints multiple shadows', () {
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    final CanvasParagraph paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto'), (
      CanvasParagraphBuilder builder,
    ) {
      builder.pushStyle(
        EngineTextStyle.only(
          fontSize: 32.0,
          color: blue,
          shadows: <Shadow>[
            const Shadow(color: red, blurRadius: 2.0, offset: Offset(4.0, 2.0)),
            const Shadow(color: green, blurRadius: 3.0),
          ],
        ),
      );
      builder.addText('Lorem ');
      builder.pushStyle(
        EngineTextStyle.only(
          color: green,
          background: Paint()..color = yellow,
          shadows: <Shadow>[const Shadow(blurRadius: 10.0)],
        ),
      );
      builder.addText('ipsum');
      builder.pop();
      builder.addText('dolor.');
    })..layout(constrain(double.infinity));
    canvas.drawParagraph(paragraph, Offset.zero);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_shadows');
  });
}
