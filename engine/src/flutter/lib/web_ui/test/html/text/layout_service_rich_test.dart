// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import '../../common/test_initialization.dart';
import '../paragraph/helper.dart';
import 'layout_service_helper.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(withImplicitView: true);

  test('does not crash on empty spans', () {
    final CanvasParagraph paragraph = rich(ahemStyle, (CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('');

      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('Lorem ipsum');

      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('');
    });

    expect(() => paragraph.layout(constrain(double.infinity)), returnsNormally);
  });

  test('measures spans in the same line correctly', () {
    final CanvasParagraph paragraph = rich(ahemStyle, (CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(fontSize: 12.0));
      // 12.0 * 6 = 72.0 (with spaces)
      // 12.0 * 5 = 60.0 (without spaces)
      builder.addText('Lorem ');

      builder.pushStyle(EngineTextStyle.only(fontSize: 13.0));
      // 13.0 * 6 = 78.0 (with spaces)
      // 13.0 * 5 = 65.0 (without spaces)
      builder.addText('ipsum ');

      builder.pushStyle(EngineTextStyle.only(fontSize: 11.0));
      // 11.0 * 5 = 55.0
      builder.addText('dolor');
    })..layout(constrain(double.infinity));

    expect(paragraph.maxIntrinsicWidth, 205.0);
    expect(paragraph.minIntrinsicWidth, 65.0); // "ipsum"
    expect(paragraph.width, double.infinity);
    expectLines(paragraph, <TestLine>[
      l('Lorem ipsum dolor', 0, 17, hardBreak: true, width: 205.0, left: 0.0),
    ]);
  });

  test('breaks lines correctly at the end of spans', () {
    final CanvasParagraph paragraph = rich(ahemStyle, (CanvasParagraphBuilder builder) {
      builder.addText('Lorem ');
      builder.pushStyle(EngineTextStyle.only(fontSize: 15.0));
      builder.addText('sit ');
      builder.pop();
      builder.addText('.');
    })..layout(constrain(60.0));

    expect(paragraph.maxIntrinsicWidth, 130.0);
    expect(paragraph.minIntrinsicWidth, 50.0); // "Lorem"
    expect(paragraph.width, 60.0);
    expectLines(paragraph, <TestLine>[
      l('Lorem ', 0, 6, hardBreak: false, width: 50.0, left: 0.0),
      l('sit ', 6, 10, hardBreak: false, width: 45.0, left: 0.0),
      l('.', 10, 11, hardBreak: true, width: 10.0, left: 0.0),
    ]);
  });

  test('breaks lines correctly in the middle of spans', () {
    final CanvasParagraph paragraph = rich(ahemStyle, (CanvasParagraphBuilder builder) {
      builder.addText('Lorem ipsum ');
      builder.pushStyle(EngineTextStyle.only(fontSize: 11.0));
      builder.addText('sit dolor');
    })..layout(constrain(100.0));

    expect(paragraph.maxIntrinsicWidth, 219.0);
    expect(paragraph.minIntrinsicWidth, 55.0); // "dolor"
    expect(paragraph.width, 100.0);
    expectLines(paragraph, <TestLine>[
      l('Lorem ', 0, 6, hardBreak: false, width: 50.0, left: 0.0),
      l('ipsum sit ', 6, 16, hardBreak: false, width: 93.0, left: 0.0),
      l('dolor', 16, 21, hardBreak: true, width: 55.0, left: 0.0),
    ]);
  });

  test('handles space-only spans', () {
    final CanvasParagraph paragraph = rich(ahemStyle, (CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('Lorem ');
      builder.pop();
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('   ');
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('  ');
      builder.pushStyle(EngineTextStyle.only(color: black));
      builder.addText('ipsum');
    });
    paragraph.layout(constrain(80.0));

    expect(paragraph.maxIntrinsicWidth, 160.0);
    expect(paragraph.minIntrinsicWidth, 50.0); // "Lorem" or "ipsum"
    expect(paragraph.width, 80.0);
    expectLines(paragraph, <TestLine>[
      l(
        'Lorem      ',
        0,
        11,
        hardBreak: false,
        width: 50.0,
        widthWithTrailingSpaces: 110.0,
        left: 0.0,
      ),
      l('ipsum', 11, 16, hardBreak: true, width: 50.0, left: 0.0),
    ]);
  });

  test('should not break at span end if it is not a line break', () {
    final CanvasParagraph paragraph = rich(ahemStyle, (CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('Lorem');
      builder.pop();
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText(' ');
      builder.pushStyle(EngineTextStyle.only(color: black));
      builder.addText('ip');
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('su');
      builder.pushStyle(EngineTextStyle.only(color: white));
      builder.addText('m');
    })..layout(constrain(50.0));

    expect(paragraph.maxIntrinsicWidth, 110.0);
    expect(paragraph.minIntrinsicWidth, 50.0); // "Lorem" or "ipsum"
    expect(paragraph.width, 50.0);
    expectLines(paragraph, <TestLine>[
      l('Lorem ', 0, 6, hardBreak: false, width: 50.0, left: 0.0),
      l('ipsum', 6, 11, hardBreak: true, width: 50.0, left: 0.0),
    ]);
  });

  test('should handle placeholder-only paragraphs', () {
    final EngineParagraphStyle paragraphStyle = EngineParagraphStyle(
      fontFamily: 'Ahem',
      fontSize: 10,
      textAlign: ui.TextAlign.center,
    );
    final CanvasParagraph paragraph = rich(paragraphStyle, (CanvasParagraphBuilder builder) {
      builder.addPlaceholder(
        300.0,
        50.0,
        ui.PlaceholderAlignment.baseline,
        baseline: ui.TextBaseline.alphabetic,
      );
    })..layout(constrain(500.0));

    expect(paragraph.maxIntrinsicWidth, 300.0);
    expect(paragraph.minIntrinsicWidth, 300.0);
    expect(paragraph.height, 50.0);
    expectLines(paragraph, <TestLine>[
      l(placeholderChar, 0, 1, hardBreak: true, width: 300.0, left: 100.0),
    ]);
  });

  test('correct maxIntrinsicWidth when paragraph ends with placeholder', () {
    final EngineParagraphStyle paragraphStyle = EngineParagraphStyle(
      fontFamily: 'Ahem',
      fontSize: 10,
      textAlign: ui.TextAlign.center,
    );
    final CanvasParagraph paragraph = rich(paragraphStyle, (CanvasParagraphBuilder builder) {
      builder.addText('abcd');
      builder.addPlaceholder(300.0, 50.0, ui.PlaceholderAlignment.bottom);
    })..layout(constrain(400.0));

    expect(paragraph.maxIntrinsicWidth, 340.0);
    expect(paragraph.minIntrinsicWidth, 300.0);
    expect(paragraph.height, 50.0);
    expectLines(paragraph, <TestLine>[
      l('abcd$placeholderChar', 0, 5, hardBreak: true, width: 340.0, left: 30.0),
    ]);
  });

  test('handles new line followed by a placeholder', () {
    final EngineParagraphStyle paragraphStyle = EngineParagraphStyle(
      fontFamily: 'Ahem',
      fontSize: 10,
      textAlign: ui.TextAlign.center,
    );
    final CanvasParagraph paragraph = rich(paragraphStyle, (CanvasParagraphBuilder builder) {
      builder.addText('Lorem\n');
      builder.addPlaceholder(300.0, 40.0, ui.PlaceholderAlignment.bottom);
      builder.addText('ipsum');
    })..layout(constrain(300.0));

    // The placeholder's width + "ipsum"
    expect(paragraph.maxIntrinsicWidth, 300.0 + 50.0);
    expect(paragraph.minIntrinsicWidth, 300.0);
    expect(paragraph.height, 10.0 + 40.0 + 10.0);
    expectLines(paragraph, <TestLine>[
      l('Lorem', 0, 6, hardBreak: true, width: 50.0, height: 10.0, left: 125.0),
      l(placeholderChar, 6, 7, hardBreak: false, width: 300.0, height: 40.0, left: 0.0),
      l('ipsum', 7, 12, hardBreak: true, width: 50.0, height: 10.0, left: 125.0),
    ]);
  });

  test('correctly force-breaks consecutive non-breakable spans', () {
    final CanvasParagraph paragraph = rich(ahemStyle, (ui.ParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(fontSize: 1));
      builder.addText('A');
      builder.pop(); // Back to fontSize: 10
      builder.addText('A' * 20);
      builder.pushStyle(EngineTextStyle.only(fontSize: 1));
      builder.addText('A');
    });
    paragraph.layout(constrain(200));

    expect(paragraph.maxIntrinsicWidth, 200.0 + 2.0);
    expect(paragraph.height, 20.0);
    expectLines(paragraph, <TestLine>[
      // 1x small "A" + 19x big "A"
      l('A' * 20, 0, 20, hardBreak: false, width: 1.0 + 190.0, height: 10.0),
      // 1x big "A" + 1x small "A"
      l('AA', 20, 22, hardBreak: true, width: 10.0 + 1.0, height: 10.0),
    ]);
  });

  test('does not make prohibited line breaks', () {
    final CanvasParagraph paragraph = rich(ahemStyle, (ui.ParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('AAA B');
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('BB ');
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('CC');
    });
    paragraph.layout(constrain(60));

    expect(paragraph.maxIntrinsicWidth, 100.0);
    expectLines(paragraph, <TestLine>[
      l('AAA ', 0, 4, hardBreak: false, width: 30.0),
      l('BBB CC', 4, 10, hardBreak: true, width: 60.0),
    ]);
  });
}
