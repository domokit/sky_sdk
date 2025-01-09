// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui show BoxHeightStyle, BoxWidthStyle, Paragraph, TextBox;

import 'package:flutter/foundation.dart' show isSkiaWeb, kIsWeb;
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

const String _kText =
    "I polished up that handle so carefullee\nThat now I am the Ruler of the Queen's Navee!";

void _applyParentData(List<RenderBox> inlineRenderBoxes, InlineSpan span) {
  int index = 0;
  RenderBox? previousBox;
  span.visitChildren((InlineSpan span) {
    if (span is! WidgetSpan) {
      return true;
    }

    final RenderBox box = inlineRenderBoxes[index];
    box.parentData =
        TextParentData()
          ..span = span
          ..previousSibling = previousBox;
    (previousBox?.parentData as TextParentData?)?.nextSibling = box;
    index += 1;
    previousBox = box;
    return true;
  });
}

// A subclass of RenderParagraph that returns an empty list in getBoxesForSelection
// for a given TextSelection.
// This is intended to simulate SkParagraph's implementation of Paragraph.getBoxesForRange,
// which may return an empty list in some situations where Libtxt would return a list
// containing an empty box.
class RenderParagraphWithEmptySelectionBoxList extends RenderParagraph {
  RenderParagraphWithEmptySelectionBoxList(
    super.text, {
    required super.textDirection,
    required this.emptyListSelection,
  });

  TextSelection emptyListSelection;

  @override
  List<ui.TextBox> getBoxesForSelection(
    TextSelection selection, {
    ui.BoxHeightStyle boxHeightStyle = ui.BoxHeightStyle.tight,
    ui.BoxWidthStyle boxWidthStyle = ui.BoxWidthStyle.tight,
  }) {
    if (selection == emptyListSelection) {
      return <ui.TextBox>[];
    }
    return super.getBoxesForSelection(
      selection,
      boxHeightStyle: boxHeightStyle,
      boxWidthStyle: boxWidthStyle,
    );
  }
}

// A subclass of RenderParagraph that returns an empty list in getBoxesForSelection
// for a selection representing a WidgetSpan.
// This is intended to simulate how SkParagraph's implementation of Paragraph.getBoxesForRange
// can return an empty list for a WidgetSpan with empty dimensions.
class RenderParagraphWithEmptyBoxListForWidgetSpan extends RenderParagraph {
  RenderParagraphWithEmptyBoxListForWidgetSpan(
    super.text, {
    required List<RenderBox> super.children,
    required super.textDirection,
  });

  @override
  List<ui.TextBox> getBoxesForSelection(
    TextSelection selection, {
    ui.BoxHeightStyle boxHeightStyle = ui.BoxHeightStyle.tight,
    ui.BoxWidthStyle boxWidthStyle = ui.BoxWidthStyle.tight,
  }) {
    if (text.getSpanForPosition(selection.base) is WidgetSpan) {
      return <ui.TextBox>[];
    }
    return super.getBoxesForSelection(
      selection,
      boxHeightStyle: boxHeightStyle,
      boxWidthStyle: boxWidthStyle,
    );
  }
}

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('getOffsetForCaret control test', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(text: _kText),
      textDirection: TextDirection.ltr,
    );
    layout(paragraph);

    const Rect caret = Rect.fromLTWH(0.0, 0.0, 2.0, 20.0);

    final Offset offset5 = paragraph.getOffsetForCaret(const TextPosition(offset: 5), caret);
    expect(offset5.dx, greaterThan(0.0));

    final Offset offset25 = paragraph.getOffsetForCaret(const TextPosition(offset: 25), caret);
    expect(offset25.dx, greaterThan(offset5.dx));

    final Offset offset50 = paragraph.getOffsetForCaret(const TextPosition(offset: 50), caret);
    expect(offset50.dy, greaterThan(offset5.dy));
  });

  test('getFullHeightForCaret control test', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(text: _kText, style: TextStyle(fontSize: 10.0)),
      textDirection: TextDirection.ltr,
    );
    layout(paragraph);

    final double height5 = paragraph.getFullHeightForCaret(const TextPosition(offset: 5));
    expect(height5, equals(10.0));
  });

  test('getPositionForOffset control test', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(text: _kText),
      textDirection: TextDirection.ltr,
    );
    layout(paragraph);

    final TextPosition position20 = paragraph.getPositionForOffset(const Offset(20.0, 5.0));
    expect(position20.offset, greaterThan(0.0));

    final TextPosition position40 = paragraph.getPositionForOffset(const Offset(40.0, 5.0));
    expect(position40.offset, greaterThan(position20.offset));

    final TextPosition positionBelow = paragraph.getPositionForOffset(const Offset(5.0, 20.0));
    expect(positionBelow.offset, greaterThan(position40.offset));
  });

  test('getBoxesForSelection control test', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(text: _kText, style: TextStyle(fontSize: 10.0)),
      textDirection: TextDirection.ltr,
    );
    layout(paragraph);

    List<ui.TextBox> boxes = paragraph.getBoxesForSelection(
      const TextSelection(baseOffset: 5, extentOffset: 25),
    );

    expect(boxes.length, equals(1));

    boxes = paragraph.getBoxesForSelection(const TextSelection(baseOffset: 25, extentOffset: 50));

    expect(boxes.any((ui.TextBox box) => box.left == 250 && box.top == 0), isTrue);
    expect(boxes.any((ui.TextBox box) => box.right == 100 && box.top == 10), isTrue);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/61016

  test('getBoxesForSelection test with multiple TextSpans and lines', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(
        text: 'First ',
        style: TextStyle(fontSize: 10.0),
        children: <InlineSpan>[
          TextSpan(text: 'smallsecond ', style: TextStyle(fontSize: 5.0)),
          TextSpan(text: 'third fourth fifth'),
        ],
      ),
      textDirection: TextDirection.ltr,
    );
    // Do layout with width chosen so that this splits as
    // First smallsecond |
    // third fourth |
    // fifth|
    // The corresponding line widths come out to be:
    // 1st line: 120px wide: 6 chars * 10px plus 12 chars * 5px.
    // 2nd line: 130px wide: 13 chars * 10px.
    // 3rd line: 50px wide.
    layout(paragraph, constraints: const BoxConstraints(maxWidth: 140.0));

    final List<ui.TextBox> boxes = paragraph.getBoxesForSelection(
      const TextSelection(baseOffset: 0, extentOffset: 36),
    );

    expect(boxes.length, equals(4));

    // The widths of the boxes should match the calculations above.
    // The heights should all be 10, except for the box for 'smallsecond ',
    // which should have height 5, and be alphabetic baseline-aligned with
    // 'First '. The test font specifies alphabetic baselines at 0.25em above
    // the bottom extent, and 0.75em below the top, so the difference in top
    // alignment becomes (10px * 0.75 - 5px * 0.75) = 3.75px.

    // 'First ':
    expect(boxes[0], const TextBox.fromLTRBD(0.0, 0.0, 60.0, 10.0, TextDirection.ltr));
    // 'smallsecond ' in size 5:
    expect(boxes[1], const TextBox.fromLTRBD(60.0, 3.75, 120.0, 8.75, TextDirection.ltr));
    // 'third fourth ':
    expect(boxes[2], const TextBox.fromLTRBD(0.0, 10.0, 130.0, 20.0, TextDirection.ltr));
    // 'fifth':
    expect(boxes[3], const TextBox.fromLTRBD(0.0, 20.0, 50.0, 30.0, TextDirection.ltr));
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/61016

  test('getBoxesForSelection test with boxHeightStyle and boxWidthStyle set to max', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(
        text: 'First ',
        style: TextStyle(fontFamily: 'FlutterTest', fontSize: 10.0),
        children: <InlineSpan>[
          TextSpan(text: 'smallsecond ', style: TextStyle(fontSize: 8.0)),
          TextSpan(text: 'third fourth fifth'),
        ],
      ),
      textDirection: TextDirection.ltr,
    );
    // Do layout with width chosen so that this splits as
    // First smallsecond |
    // third fourth |
    // fifth|
    // The corresponding line widths come out to be:
    // 1st line: 156px wide: 6 chars * 10px plus 12 chars * 8px.
    // 2nd line: 130px wide: 13 chars * 10px.
    // 3rd line: 50px wide.
    layout(paragraph, constraints: const BoxConstraints(maxWidth: 160.0));

    final List<ui.TextBox> boxes = paragraph.getBoxesForSelection(
      const TextSelection(baseOffset: 0, extentOffset: 36),
      boxHeightStyle: ui.BoxHeightStyle.max,
      boxWidthStyle: ui.BoxWidthStyle.max,
    );

    expect(boxes.length, equals(5));

    // 'First ':
    expect(boxes[0], const TextBox.fromLTRBD(0.0, 0.0, 60.0, 10.0, TextDirection.ltr));
    // 'smallsecond ' in size 8, but on same line as previous box, so height remains 10:
    expect(boxes[1], const TextBox.fromLTRBD(60.0, 0.0, 156.0, 10.0, TextDirection.ltr));
    // 'third fourth ':
    expect(boxes[2], const TextBox.fromLTRBD(0.0, 10.0, 130.0, 20.0, TextDirection.ltr));
    // extra box added to extend width, as per definition of ui.BoxWidthStyle.max:
    expect(boxes[3], const TextBox.fromLTRBD(130.0, 10.0, 156.0, 20.0, TextDirection.ltr));
    // 'fifth':
    expect(boxes[4], const TextBox.fromLTRBD(0.0, 20.0, 50.0, 30.0, TextDirection.ltr));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/61016

  test('getWordBoundary control test', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(text: _kText),
      textDirection: TextDirection.ltr,
    );
    layout(paragraph);

    final TextRange range5 = paragraph.getWordBoundary(const TextPosition(offset: 5));
    expect(range5.textInside(_kText), equals('polished'));

    final TextRange range50 = paragraph.getWordBoundary(const TextPosition(offset: 50));
    expect(range50.textInside(_kText), equals(' '));

    final TextRange range85 = paragraph.getWordBoundary(const TextPosition(offset: 75));
    expect(range85.textInside(_kText), equals("Queen's"));
  });

  test('overflow test', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(
        text:
            'This\n' // 4 characters * 10px font size = 40px width on the first line
            'is a wrapping test. It should wrap at manual newlines, and if softWrap is true, also at spaces.',
        style: TextStyle(fontSize: 10.0),
      ),
      textDirection: TextDirection.ltr,
      maxLines: 1,
    );

    void relayoutWith({int? maxLines, required bool softWrap, required TextOverflow overflow}) {
      paragraph
        ..maxLines = maxLines
        ..softWrap = softWrap
        ..overflow = overflow;
      pumpFrame();
    }

    // Lay out in a narrow box to force wrapping.
    layout(
      paragraph,
      constraints: const BoxConstraints(maxWidth: 50.0),
    ); // enough to fit "This" but not "This is"
    final double lineHeight = paragraph.size.height;

    relayoutWith(maxLines: 3, softWrap: true, overflow: TextOverflow.clip);
    expect(paragraph.size.height, equals(3 * lineHeight));

    relayoutWith(softWrap: true, overflow: TextOverflow.clip);
    expect(paragraph.size.height, greaterThan(5 * lineHeight));

    // Try again with ellipsis overflow. We can't test that the ellipsis are
    // drawn, but we can test the sizing.
    relayoutWith(maxLines: 1, softWrap: true, overflow: TextOverflow.ellipsis);
    expect(paragraph.size.height, equals(lineHeight));

    relayoutWith(maxLines: 3, softWrap: true, overflow: TextOverflow.ellipsis);
    expect(paragraph.size.height, equals(3 * lineHeight));

    // This is the one weird case. If maxLines is null, we would expect to allow
    // infinite wrapping. However, if we did, we'd never know when to append an
    // ellipsis, so this really means "append ellipsis as soon as we exceed the
    // width".
    relayoutWith(softWrap: true, overflow: TextOverflow.ellipsis);
    expect(paragraph.size.height, equals(2 * lineHeight));

    // Now with no soft wrapping.
    relayoutWith(maxLines: 1, softWrap: false, overflow: TextOverflow.clip);
    expect(paragraph.size.height, equals(lineHeight));

    relayoutWith(maxLines: 3, softWrap: false, overflow: TextOverflow.clip);
    expect(paragraph.size.height, equals(2 * lineHeight));

    relayoutWith(softWrap: false, overflow: TextOverflow.clip);
    expect(paragraph.size.height, equals(2 * lineHeight));

    relayoutWith(maxLines: 1, softWrap: false, overflow: TextOverflow.ellipsis);
    expect(paragraph.size.height, equals(lineHeight));

    relayoutWith(maxLines: 3, softWrap: false, overflow: TextOverflow.ellipsis);
    expect(paragraph.size.height, equals(3 * lineHeight));

    relayoutWith(softWrap: false, overflow: TextOverflow.ellipsis);
    expect(paragraph.size.height, equals(2 * lineHeight));

    // Test presence of the fade effect.
    relayoutWith(maxLines: 3, softWrap: true, overflow: TextOverflow.fade);
    expect(paragraph.debugHasOverflowShader, isTrue);

    // Change back to ellipsis and check that the fade shader is cleared.
    relayoutWith(maxLines: 3, softWrap: true, overflow: TextOverflow.ellipsis);
    expect(paragraph.debugHasOverflowShader, isFalse);

    relayoutWith(maxLines: 100, softWrap: true, overflow: TextOverflow.fade);
    expect(paragraph.debugHasOverflowShader, isFalse);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/61018

  test('maxLines', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(
        text:
            "How do you write like you're running out of time? Write day and night like you're running out of time?",
        // 0123456789 0123456789 012 345 0123456 012345 01234 012345678 012345678 0123 012 345 0123456 012345 01234
        // 0          1          2       3       4      5     6         7         8    9       10      11     12
        style: TextStyle(fontSize: 10.0),
      ),
      textDirection: TextDirection.ltr,
    );
    layout(paragraph, constraints: const BoxConstraints(maxWidth: 100.0));
    void layoutAt(int? maxLines) {
      paragraph.maxLines = maxLines;
      pumpFrame();
    }

    layoutAt(null);
    expect(paragraph.size.height, 130.0);

    layoutAt(1);
    expect(paragraph.size.height, 10.0);

    layoutAt(2);
    expect(paragraph.size.height, 20.0);

    layoutAt(3);
    expect(paragraph.size.height, 30.0);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/61018

  test('textAlign triggers TextPainter relayout in the paint method', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(text: 'A', style: TextStyle(fontSize: 10.0)),
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.left,
    );

    Rect getRectForA() =>
        paragraph
            .getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 1))
            .single
            .toRect();

    layout(paragraph, constraints: const BoxConstraints.tightFor(width: 100.0));

    expect(getRectForA(), const Rect.fromLTWH(0, 0, 10, 10));

    paragraph.textAlign = TextAlign.right;
    expect(paragraph.debugNeedsLayout, isFalse);
    expect(paragraph.debugNeedsPaint, isTrue);

    paragraph.paint(MockPaintingContext(), Offset.zero);
    expect(getRectForA(), const Rect.fromLTWH(90, 0, 10, 10));
  });

  group('didExceedMaxLines', () {
    RenderParagraph createRenderParagraph({
      int? maxLines,
      TextOverflow overflow = TextOverflow.clip,
    }) {
      return RenderParagraph(
        const TextSpan(
          text: 'Here is a long text, maybe exceed maxlines',
          style: TextStyle(fontSize: 10.0),
        ),
        textDirection: TextDirection.ltr,
        overflow: overflow,
        maxLines: maxLines,
      );
    }

    test('none limited', () {
      final RenderParagraph paragraph = createRenderParagraph();
      layout(paragraph, constraints: const BoxConstraints(maxWidth: 100.0));
      expect(paragraph.didExceedMaxLines, false);
    });

    test('limited by maxLines', () {
      final RenderParagraph paragraph = createRenderParagraph(maxLines: 1);
      layout(paragraph, constraints: const BoxConstraints(maxWidth: 100.0));
      expect(paragraph.didExceedMaxLines, true);
    });

    test('limited by ellipsis', () {
      final RenderParagraph paragraph = createRenderParagraph(overflow: TextOverflow.ellipsis);
      layout(paragraph, constraints: const BoxConstraints(maxWidth: 100.0));
      expect(paragraph.didExceedMaxLines, true);
    });
  });

  test('changing color does not do layout', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(text: 'Hello', style: TextStyle(color: Color(0xFF000000))),
      textDirection: TextDirection.ltr,
    );
    layout(paragraph, constraints: const BoxConstraints(maxWidth: 100.0), phase: EnginePhase.paint);
    expect(paragraph.debugNeedsLayout, isFalse);
    expect(paragraph.debugNeedsPaint, isFalse);
    paragraph.text = const TextSpan(
      text: 'Hello World',
      style: TextStyle(color: Color(0xFF000000)),
    );
    expect(paragraph.debugNeedsLayout, isTrue);
    expect(paragraph.debugNeedsPaint, isFalse);
    pumpFrame(phase: EnginePhase.paint);
    expect(paragraph.debugNeedsLayout, isFalse);
    expect(paragraph.debugNeedsPaint, isFalse);
    paragraph.text = const TextSpan(
      text: 'Hello World',
      style: TextStyle(color: Color(0xFFFFFFFF)),
    );
    expect(paragraph.debugNeedsLayout, isFalse);
    expect(paragraph.debugNeedsPaint, isTrue);
    pumpFrame(phase: EnginePhase.paint);
    expect(paragraph.debugNeedsLayout, isFalse);
    expect(paragraph.debugNeedsPaint, isFalse);
  });

  test('nested TextSpans in paragraph handle linear textScaler correctly.', () {
    const TextSpan testSpan = TextSpan(
      text: 'a',
      style: TextStyle(fontSize: 10.0),
      children: <TextSpan>[
        TextSpan(
          text: 'b',
          children: <TextSpan>[TextSpan(text: 'c')],
          style: TextStyle(fontSize: 20.0),
        ),
        TextSpan(text: 'd'),
      ],
    );
    final RenderParagraph paragraph = RenderParagraph(
      testSpan,
      textDirection: TextDirection.ltr,
      textScaler: const TextScaler.linear(1.3),
    );
    paragraph.layout(const BoxConstraints());
    expect(paragraph.size.width, 78.0);
    expect(paragraph.size.height, 26.0);

    final int length = testSpan.toPlainText().length;
    // Test the sizes of nested spans.
    final List<ui.TextBox> boxes = <ui.TextBox>[
      for (int i = 0; i < length; ++i)
        ...paragraph.getBoxesForSelection(TextSelection(baseOffset: i, extentOffset: i + 1)),
    ];
    expect(boxes, hasLength(4));

    expect(boxes[0].toRect().width, 13.0);
    expect(boxes[0].toRect().height, 13.0);
    expect(boxes[1].toRect().width, 26.0);
    expect(boxes[1].toRect().height, 26.0);
    expect(boxes[2].toRect().width, 26.0);
    expect(boxes[2].toRect().height, 26.0);
    expect(boxes[3].toRect().width, 13.0);
    expect(boxes[3].toRect().height, 13.0);
  });

  test('toStringDeep', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(text: _kText),
      textDirection: TextDirection.ltr,
      locale: const Locale('ja', 'JP'),
    );
    expect(paragraph, hasAGoodToStringDeep);
    expect(
      paragraph.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderParagraph#00000 NEEDS-LAYOUT NEEDS-PAINT DETACHED\n'
        ' │ parentData: MISSING\n'
        ' │ constraints: MISSING\n'
        ' │ size: MISSING\n'
        ' │ textAlign: start\n'
        ' │ textDirection: ltr\n'
        ' │ softWrap: wrapping at box width\n'
        ' │ overflow: clip\n'
        ' │ locale: ja_JP\n'
        ' │ maxLines: unlimited\n'
        ' ╘═╦══ text ═══\n'
        '   ║ TextSpan:\n'
        '   ║   "I polished up that handle so carefullee\n'
        '   ║   That now I am the Ruler of the Queen\'s Navee!"\n'
        '   ╚═══════════\n',
      ),
    );
  });

  test('locale setter', () {
    // Regression test for https://github.com/flutter/flutter/issues/18175

    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(text: _kText),
      locale: const Locale('zh', 'HK'),
      textDirection: TextDirection.ltr,
    );
    expect(paragraph.locale, const Locale('zh', 'HK'));

    paragraph.locale = const Locale('ja', 'JP');
    expect(paragraph.locale, const Locale('ja', 'JP'));
  });

  test('inline widgets test', () {
    const TextSpan text = TextSpan(
      text: 'a',
      style: TextStyle(fontSize: 10.0),
      children: <InlineSpan>[
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
        TextSpan(text: 'a'),
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
      ],
    );
    // Fake the render boxes that correspond to the WidgetSpans. We use
    // RenderParagraph to reduce dependencies this test has.
    final List<RenderBox> renderBoxes = <RenderBox>[
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
    ];

    final RenderParagraph paragraph = RenderParagraph(
      text,
      textDirection: TextDirection.ltr,
      children: renderBoxes,
    );
    _applyParentData(renderBoxes, text);
    layout(paragraph, constraints: const BoxConstraints(maxWidth: 100.0));

    final List<ui.TextBox> boxes = paragraph.getBoxesForSelection(
      const TextSelection(baseOffset: 0, extentOffset: 8),
    );

    expect(boxes.length, equals(5));
    expect(boxes[0], const TextBox.fromLTRBD(0.0, 4.0, 10.0, 14.0, TextDirection.ltr));
    expect(boxes[1], const TextBox.fromLTRBD(10.0, 0.0, 24.0, 14.0, TextDirection.ltr));
    expect(boxes[2], const TextBox.fromLTRBD(24.0, 0.0, 38.0, 14.0, TextDirection.ltr));
    expect(boxes[3], const TextBox.fromLTRBD(38.0, 4.0, 48.0, 14.0, TextDirection.ltr));
    expect(boxes[4], const TextBox.fromLTRBD(48.0, 0.0, 62.0, 14.0, TextDirection.ltr));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/61020

  test('getBoxesForSelection with boxHeightStyle for inline widgets', () {
    const TextSpan text = TextSpan(
      text: 'a',
      style: TextStyle(fontSize: 10.0),
      children: <InlineSpan>[
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
        TextSpan(text: 'a'),
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
      ],
    );
    // Fake the render boxes that correspond to the WidgetSpans. We use
    // RenderParagraph to reduce the dependencies this test has. The dimensions
    // of these get used in place of the widths and heights specified in the
    // SizedBoxes above: each comes out as (w,h) = (14,14).
    final List<RenderBox> renderBoxes = <RenderBox>[
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
    ];

    final RenderParagraph paragraph = RenderParagraph(
      text,
      textDirection: TextDirection.ltr,
      children: renderBoxes,
    );
    _applyParentData(renderBoxes, text);
    layout(paragraph, constraints: const BoxConstraints(maxWidth: 100.0));

    final List<ui.TextBox> boxes = paragraph.getBoxesForSelection(
      const TextSelection(baseOffset: 0, extentOffset: 8),
      boxHeightStyle: ui.BoxHeightStyle.max,
    );

    expect(boxes.length, equals(5));
    expect(boxes[0], const TextBox.fromLTRBD(0.0, 0.0, 10.0, 14.0, TextDirection.ltr));
    expect(boxes[1], const TextBox.fromLTRBD(10.0, 0.0, 24.0, 14.0, TextDirection.ltr));
    expect(boxes[2], const TextBox.fromLTRBD(24.0, 0.0, 38.0, 14.0, TextDirection.ltr));
    expect(boxes[3], const TextBox.fromLTRBD(38.0, 0.0, 48.0, 14.0, TextDirection.ltr));
    expect(boxes[4], const TextBox.fromLTRBD(48.0, 0.0, 62.0, 14.0, TextDirection.ltr));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/61020

  test('inline widgets multiline test', () {
    const TextSpan text = TextSpan(
      text: 'a',
      style: TextStyle(fontSize: 10.0),
      children: <InlineSpan>[
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
        TextSpan(text: 'a'),
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
        WidgetSpan(child: SizedBox(width: 21, height: 21)),
      ],
    );
    // Fake the render boxes that correspond to the WidgetSpans. We use
    // RenderParagraph to reduce dependencies this test has.
    final List<RenderBox> renderBoxes = <RenderBox>[
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
      RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
    ];

    final RenderParagraph paragraph = RenderParagraph(
      text,
      textDirection: TextDirection.ltr,
      children: renderBoxes,
    );
    _applyParentData(renderBoxes, text);
    layout(paragraph, constraints: const BoxConstraints(maxWidth: 50.0));

    final List<ui.TextBox> boxes = paragraph.getBoxesForSelection(
      const TextSelection(baseOffset: 0, extentOffset: 12),
    );

    expect(boxes.length, equals(9));
    expect(boxes[0], const TextBox.fromLTRBD(0.0, 4.0, 10.0, 14.0, TextDirection.ltr));
    expect(boxes[1], const TextBox.fromLTRBD(10.0, 0.0, 24.0, 14.0, TextDirection.ltr));
    expect(boxes[2], const TextBox.fromLTRBD(24.0, 0.0, 38.0, 14.0, TextDirection.ltr));
    expect(boxes[3], const TextBox.fromLTRBD(38.0, 4.0, 48.0, 14.0, TextDirection.ltr));
    // Wraps
    expect(boxes[4], const TextBox.fromLTRBD(0.0, 14.0, 14.0, 28.0, TextDirection.ltr));
    expect(boxes[5], const TextBox.fromLTRBD(14.0, 14.0, 28.0, 28.0, TextDirection.ltr));
    expect(boxes[6], const TextBox.fromLTRBD(28.0, 14.0, 42.0, 28.0, TextDirection.ltr));
    // Wraps
    expect(boxes[7], const TextBox.fromLTRBD(0.0, 28.0, 14.0, 42.0, TextDirection.ltr));
    expect(boxes[8], const TextBox.fromLTRBD(14.0, 28.0, 28.0, 42.0, TextDirection.ltr));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/61020

  test('Does not include the semantics node of truncated rendering children', () {
    // Regression test for https://github.com/flutter/flutter/issues/88180
    const double screenWidth = 100;
    const String sentence = 'truncated';
    final List<RenderBox> renderBoxes = <RenderBox>[
      RenderParagraph(const TextSpan(text: sentence), textDirection: TextDirection.ltr),
    ];
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(
        text: 'a long line to be truncated.',
        children: <InlineSpan>[WidgetSpan(child: Text(sentence))],
      ),
      overflow: TextOverflow.ellipsis,
      children: renderBoxes,
      textDirection: TextDirection.ltr,
    );
    _applyParentData(renderBoxes, paragraph.text);
    layout(paragraph, constraints: const BoxConstraints(maxWidth: screenWidth));
    final SemanticsNode result = SemanticsNode();
    final SemanticsNode truncatedChild = SemanticsNode();
    truncatedChild.tags = <SemanticsTag>{const PlaceholderSpanIndexSemanticsTag(0)};
    paragraph.assembleSemanticsNode(result, SemanticsConfiguration(), <SemanticsNode>[
      truncatedChild,
    ]);
    // It should only contain the semantics node of the TextSpan.
    expect(result.childrenCount, 1);
    result.visitChildren((SemanticsNode node) {
      expect(node != truncatedChild, isTrue);
      return true;
    });
  });

  test('Supports gesture recognizer semantics', () {
    final RenderParagraph paragraph = RenderParagraph(
      TextSpan(
        text: _kText,
        children: <InlineSpan>[
          TextSpan(text: 'one', recognizer: TapGestureRecognizer()..onTap = () {}),
          TextSpan(text: 'two', recognizer: LongPressGestureRecognizer()..onLongPress = () {}),
          TextSpan(text: 'three', recognizer: DoubleTapGestureRecognizer()..onDoubleTap = () {}),
        ],
      ),
      textDirection: TextDirection.rtl,
    );
    layout(paragraph);

    final SemanticsNode node = SemanticsNode();
    paragraph.assembleSemanticsNode(node, SemanticsConfiguration(), <SemanticsNode>[]);
    final List<SemanticsNode> children = <SemanticsNode>[];
    node.visitChildren((SemanticsNode child) {
      children.add(child);
      return true;
    });
    expect(children.length, 4);
    expect(children[0].getSemanticsData().actions, 0);
    expect(children[1].getSemanticsData().hasAction(SemanticsAction.tap), true);
    expect(children[2].getSemanticsData().hasAction(SemanticsAction.longPress), true);
    expect(children[3].getSemanticsData().hasAction(SemanticsAction.tap), true);
  });

  test('Supports empty text span with spell out', () {
    final RenderParagraph paragraph = RenderParagraph(
      const TextSpan(text: '', spellOut: true),
      textDirection: TextDirection.rtl,
    );
    layout(paragraph);
    final SemanticsNode node = SemanticsNode();
    paragraph.assembleSemanticsNode(node, SemanticsConfiguration(), <SemanticsNode>[]);
    expect(node.attributedLabel.string, '');
    expect(node.attributedLabel.attributes.length, 0);
  });

  test('Asserts on unsupported gesture recognizer', () {
    final RenderParagraph paragraph = RenderParagraph(
      TextSpan(
        text: _kText,
        children: <InlineSpan>[
          TextSpan(text: 'three', recognizer: MultiTapGestureRecognizer()..onTap = (int id) {}),
        ],
      ),
      textDirection: TextDirection.rtl,
    );
    layout(paragraph);

    bool failed = false;
    try {
      paragraph.assembleSemanticsNode(SemanticsNode(), SemanticsConfiguration(), <SemanticsNode>[]);
    } on AssertionError catch (e) {
      failed = true;
      expect(e.message, 'MultiTapGestureRecognizer is not supported.');
    }
    expect(failed, true);
  });

  test('assembleSemanticsNode handles text spans that do not yield selection boxes', () {
    final RenderParagraph paragraph = RenderParagraphWithEmptySelectionBoxList(
      TextSpan(
        text: '',
        children: <InlineSpan>[
          TextSpan(text: 'A', recognizer: TapGestureRecognizer()..onTap = () {}),
          TextSpan(text: 'B', recognizer: TapGestureRecognizer()..onTap = () {}),
          TextSpan(text: 'C', recognizer: TapGestureRecognizer()..onTap = () {}),
        ],
      ),
      textDirection: TextDirection.rtl,
      emptyListSelection: const TextSelection(baseOffset: 0, extentOffset: 1),
    );
    layout(paragraph);

    final SemanticsNode node = SemanticsNode();
    paragraph.assembleSemanticsNode(node, SemanticsConfiguration(), <SemanticsNode>[]);
    expect(node.childrenCount, 2);
  });

  test(
    'assembleSemanticsNode handles empty WidgetSpans that do not yield selection boxes',
    () {
      final TextSpan text = TextSpan(
        text: '',
        children: <InlineSpan>[
          TextSpan(text: 'A', recognizer: TapGestureRecognizer()..onTap = () {}),
          const WidgetSpan(child: SizedBox.shrink()),
          TextSpan(text: 'C', recognizer: TapGestureRecognizer()..onTap = () {}),
        ],
      );
      final List<RenderBox> renderBoxes = <RenderBox>[
        RenderParagraph(const TextSpan(text: 'b'), textDirection: TextDirection.ltr),
      ];
      final RenderParagraph paragraph = RenderParagraphWithEmptyBoxListForWidgetSpan(
        text,
        children: renderBoxes,
        textDirection: TextDirection.ltr,
      );
      _applyParentData(renderBoxes, paragraph.text);
      layout(paragraph);

      final SemanticsNode node = SemanticsNode();
      paragraph.assembleSemanticsNode(node, SemanticsConfiguration(), <SemanticsNode>[]);
      expect(node.childrenCount, 2);
    },
    skip: isBrowser, // https://github.com/flutter/flutter/issues/61020
  );

  test('Basic TextSpan Hit testing', () {
    final TextSpan textSpanA = TextSpan(text: 'A' * 10);
    const TextSpan textSpanBC = TextSpan(text: 'BC', style: TextStyle(letterSpacing: 26.0));

    final TextSpan text = TextSpan(
      style: const TextStyle(fontSize: 10.0),
      children: <InlineSpan>[textSpanA, textSpanBC],
    );

    final RenderParagraph paragraph = RenderParagraph(text, textDirection: TextDirection.ltr);
    layout(paragraph, constraints: const BoxConstraints.tightFor(width: 100.0));

    BoxHitTestResult result;

    // Hit-testing the first line
    // First A
    expect(
      paragraph.hitTest(result = BoxHitTestResult(), position: const Offset(5.0, 5.0)),
      isTrue,
    );
    expect(
      result.path.map((HitTestEntry<HitTestTarget> entry) => entry.target).whereType<TextSpan>(),
      <TextSpan>[textSpanA],
    );
    // The last A.
    expect(
      paragraph.hitTest(result = BoxHitTestResult(), position: const Offset(95.0, 5.0)),
      isTrue,
    );
    expect(
      result.path.map((HitTestEntry<HitTestTarget> entry) => entry.target).whereType<TextSpan>(),
      <TextSpan>[textSpanA],
    );
    // Far away from the line.
    expect(
      paragraph.hitTest(result = BoxHitTestResult(), position: const Offset(200.0, 5.0)),
      isFalse,
    );
    expect(
      result.path.map((HitTestEntry<HitTestTarget> entry) => entry.target).whereType<TextSpan>(),
      <TextSpan>[],
    );

    // Hit-testing the second line
    // Tapping on B (startX = letter-spacing / 2 = 13.0).
    expect(
      paragraph.hitTest(result = BoxHitTestResult(), position: const Offset(18.0, 15.0)),
      isTrue,
    );
    expect(
      result.path.map((HitTestEntry<HitTestTarget> entry) => entry.target).whereType<TextSpan>(),
      <TextSpan>[textSpanBC],
    );

    // Between B and C, with large letter-spacing.
    expect(
      paragraph.hitTest(result = BoxHitTestResult(), position: const Offset(31.0, 15.0)),
      isTrue,
    );
    expect(
      result.path.map((HitTestEntry<HitTestTarget> entry) => entry.target).whereType<TextSpan>(),
      <TextSpan>[textSpanBC],
    );

    // On C.
    expect(
      paragraph.hitTest(result = BoxHitTestResult(), position: const Offset(54.0, 15.0)),
      isTrue,
    );
    expect(
      result.path.map((HitTestEntry<HitTestTarget> entry) => entry.target).whereType<TextSpan>(),
      <TextSpan>[textSpanBC],
    );

    // After C.
    expect(
      paragraph.hitTest(result = BoxHitTestResult(), position: const Offset(100.0, 15.0)),
      isFalse,
    );
    expect(
      result.path.map((HitTestEntry<HitTestTarget> entry) => entry.target).whereType<TextSpan>(),
      <TextSpan>[],
    );

    // Not even remotely close.
    expect(
      paragraph.hitTest(result = BoxHitTestResult(), position: const Offset(9999.0, 9999.0)),
      isFalse,
    );
    expect(
      result.path.map((HitTestEntry<HitTestTarget> entry) => entry.target).whereType<TextSpan>(),
      <TextSpan>[],
    );
  });

  test('TextSpan Hit testing with text justification', () {
    const TextSpan textSpanA = TextSpan(text: 'A '); // The space is a word break.
    const TextSpan textSpanB = TextSpan(
      text: 'B\u200B',
    ); // The zero-width space is used as a line break.
    final TextSpan textSpanC = TextSpan(
      text: 'C' * 10,
    ); // The third span starts a new line since it's too long for the first line.

    // The text should look like:
    // A        B
    // CCCCCCCCCC
    final TextSpan text = TextSpan(
      text: '',
      style: const TextStyle(fontSize: 10.0),
      children: <InlineSpan>[textSpanA, textSpanB, textSpanC],
    );

    final RenderParagraph paragraph = RenderParagraph(
      text,
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.justify,
    );
    layout(paragraph, constraints: const BoxConstraints.tightFor(width: 100.0));
    BoxHitTestResult result;

    // Tapping on A.
    expect(
      paragraph.hitTest(result = BoxHitTestResult(), position: const Offset(5.0, 5.0)),
      isTrue,
    );
    expect(
      result.path.map((HitTestEntry<HitTestTarget> entry) => entry.target).whereType<TextSpan>(),
      <TextSpan>[textSpanA],
    );

    // Between A and B.
    expect(
      paragraph.hitTest(result = BoxHitTestResult(), position: const Offset(50.0, 5.0)),
      isTrue,
    );
    expect(
      result.path.map((HitTestEntry<HitTestTarget> entry) => entry.target).whereType<TextSpan>(),
      <TextSpan>[textSpanA],
    );

    // On B.
    expect(
      paragraph.hitTest(result = BoxHitTestResult(), position: const Offset(95.0, 5.0)),
      isTrue,
    );
    expect(
      result.path.map((HitTestEntry<HitTestTarget> entry) => entry.target).whereType<TextSpan>(),
      <TextSpan>[textSpanB],
    );
  });

  group('Selection', () {
    void selectionParagraph(RenderParagraph paragraph, TextPosition start, TextPosition end) {
      for (final Selectable selectable
          in (paragraph.registrar! as TestSelectionRegistrar).selectables) {
        selectable.dispatchSelectionEvent(
          SelectionEdgeUpdateEvent.forStart(
            globalPosition: paragraph.getOffsetForCaret(start, Rect.zero) + const Offset(0, 5),
          ),
        );
        selectable.dispatchSelectionEvent(
          SelectionEdgeUpdateEvent.forEnd(
            globalPosition: paragraph.getOffsetForCaret(end, Rect.zero) + const Offset(0, 5),
          ),
        );
      }
    }

    test('subscribe to SelectionRegistrar', () {
      final TestSelectionRegistrar registrar = TestSelectionRegistrar();
      final RenderParagraph paragraph = RenderParagraph(
        const TextSpan(text: '1234567'),
        textDirection: TextDirection.ltr,
        registrar: registrar,
      );
      expect(registrar.selectables.length, 1);

      paragraph.text = const TextSpan(text: '');
      expect(registrar.selectables.length, 0);
    });

    test('paints selection highlight', () async {
      final TestSelectionRegistrar registrar = TestSelectionRegistrar();
      const Color selectionColor = Color(0xAF6694e8);
      final RenderParagraph paragraph = RenderParagraph(
        const TextSpan(text: '1234567'),
        textDirection: TextDirection.ltr,
        registrar: registrar,
        selectionColor: selectionColor,
      );
      layout(paragraph);
      final MockPaintingContext paintingContext = MockPaintingContext();
      paragraph.paint(paintingContext, Offset.zero);
      expect(paintingContext.canvas.drawnRect, isNull);
      expect(paintingContext.canvas.drawnRectPaint, isNull);
      selectionParagraph(paragraph, const TextPosition(offset: 1), const TextPosition(offset: 5));

      paintingContext.canvas.clear();
      paragraph.paint(paintingContext, Offset.zero);
      expect(paintingContext.canvas.drawnRect, const Rect.fromLTWH(14.0, 0.0, 56.0, 14.0));
      expect(paintingContext.canvas.drawnRectPaint!.style, PaintingStyle.fill);
      expect(paintingContext.canvas.drawnRectPaint!.color, isSameColorAs(selectionColor));
      // Selection highlight is painted before text.
      expect(paintingContext.canvas.drawnItemTypes, <Type>[Rect, ui.Paragraph]);

      selectionParagraph(paragraph, const TextPosition(offset: 2), const TextPosition(offset: 4));
      paragraph.paint(paintingContext, Offset.zero);
      expect(paintingContext.canvas.drawnRect, const Rect.fromLTWH(28.0, 0.0, 28.0, 14.0));
      expect(paintingContext.canvas.drawnRectPaint!.style, PaintingStyle.fill);
      expect(paintingContext.canvas.drawnRectPaint!.color, isSameColorAs(selectionColor));
    });

    // Regression test for https://github.com/flutter/flutter/issues/126652.
    test('paints selection when tap at chinese character', () async {
      final TestSelectionRegistrar registrar = TestSelectionRegistrar();
      const Color selectionColor = Color(0xAF6694e8);
      final RenderParagraph paragraph = RenderParagraph(
        const TextSpan(text: '你好'),
        textDirection: TextDirection.ltr,
        registrar: registrar,
        selectionColor: selectionColor,
      );
      layout(paragraph);
      final MockPaintingContext paintingContext = MockPaintingContext();
      paragraph.paint(paintingContext, Offset.zero);
      expect(paintingContext.canvas.drawnRect, isNull);
      expect(paintingContext.canvas.drawnRectPaint, isNull);

      for (final Selectable selectable
          in (paragraph.registrar! as TestSelectionRegistrar).selectables) {
        selectable.dispatchSelectionEvent(
          const SelectWordSelectionEvent(globalPosition: Offset(7, 0)),
        );
      }

      paintingContext.canvas.clear();
      paragraph.paint(paintingContext, Offset.zero);
      expect(paintingContext.canvas.drawnRect!.isEmpty, false);
      expect(paintingContext.canvas.drawnRectPaint!.style, PaintingStyle.fill);
      expect(paintingContext.canvas.drawnRectPaint!.color, isSameColorAs(selectionColor));
    }, skip: isBrowser); // https://github.com/flutter/flutter/issues/61016

    test('getPositionForOffset works', () async {
      final RenderParagraph paragraph = RenderParagraph(
        const TextSpan(text: '1234567'),
        textDirection: TextDirection.ltr,
      );
      layout(paragraph);
      expect(
        paragraph.getPositionForOffset(const Offset(42.0, 14.0)),
        const TextPosition(offset: 3),
      );
    });

    test('can handle select all when contains widget span', () async {
      final TestSelectionRegistrar registrar = TestSelectionRegistrar();
      final List<RenderBox> renderBoxes = <RenderBox>[
        RenderParagraph(const TextSpan(text: 'widget'), textDirection: TextDirection.ltr),
      ];
      final RenderParagraph paragraph = RenderParagraph(
        const TextSpan(
          children: <InlineSpan>[
            TextSpan(text: 'before the span'),
            WidgetSpan(child: Text('widget')),
            TextSpan(text: 'after the span'),
          ],
        ),
        textDirection: TextDirection.ltr,
        registrar: registrar,
        children: renderBoxes,
      );
      _applyParentData(renderBoxes, paragraph.text);
      layout(paragraph);
      // The widget span will register to the selection container without going
      // through the render paragraph.
      expect(registrar.selectables.length, 2);
      final Selectable segment1 = registrar.selectables[0];
      segment1.dispatchSelectionEvent(const SelectAllSelectionEvent());
      final SelectionGeometry geometry1 = segment1.value;
      expect(geometry1.hasContent, true);
      expect(geometry1.status, SelectionStatus.uncollapsed);

      final Selectable segment2 = registrar.selectables[1];
      segment2.dispatchSelectionEvent(const SelectAllSelectionEvent());
      final SelectionGeometry geometry2 = segment2.value;
      expect(geometry2.hasContent, true);
      expect(geometry2.status, SelectionStatus.uncollapsed);
    });

    test('can granularly extend selection - character', () async {
      final TestSelectionRegistrar registrar = TestSelectionRegistrar();
      final List<RenderBox> renderBoxes = <RenderBox>[];
      final RenderParagraph paragraph = RenderParagraph(
        const TextSpan(children: <InlineSpan>[TextSpan(text: 'how are you\nI am fine\nThank you')]),
        textDirection: TextDirection.ltr,
        registrar: registrar,
        children: renderBoxes,
      );
      layout(paragraph);

      expect(registrar.selectables.length, 1);
      selectionParagraph(paragraph, const TextPosition(offset: 4), const TextPosition(offset: 5));
      expect(paragraph.selections.length, 1);
      TextSelection selection = paragraph.selections[0];
      expect(selection.start, 4); // how [a]re you
      expect(selection.end, 5);

      // Equivalent to sending shift + arrow-right
      registrar.selectables[0].dispatchSelectionEvent(
        const GranularlyExtendSelectionEvent(
          forward: true,
          isEnd: true,
          granularity: TextGranularity.character,
        ),
      );
      selection = paragraph.selections[0];
      expect(selection.start, 4); // how [ar]e you
      expect(selection.end, 6);

      // Equivalent to sending shift + arrow-left
      registrar.selectables[0].dispatchSelectionEvent(
        const GranularlyExtendSelectionEvent(
          forward: false,
          isEnd: true,
          granularity: TextGranularity.character,
        ),
      );
      selection = paragraph.selections[0];
      expect(selection.start, 4); // how [a]re you
      expect(selection.end, 5);
    });

    test('can granularly extend selection - word', () async {
      final TestSelectionRegistrar registrar = TestSelectionRegistrar();
      final List<RenderBox> renderBoxes = <RenderBox>[];
      final RenderParagraph paragraph = RenderParagraph(
        const TextSpan(children: <InlineSpan>[TextSpan(text: 'how are you\nI am fine\nThank you')]),
        textDirection: TextDirection.ltr,
        registrar: registrar,
        children: renderBoxes,
      );
      layout(paragraph);

      expect(registrar.selectables.length, 1);
      selectionParagraph(paragraph, const TextPosition(offset: 4), const TextPosition(offset: 5));
      expect(paragraph.selections.length, 1);
      TextSelection selection = paragraph.selections[0];
      expect(selection.start, 4); // how [a]re you
      expect(selection.end, 5);

      // Equivalent to sending shift + alt + arrow-right.
      registrar.selectables[0].dispatchSelectionEvent(
        const GranularlyExtendSelectionEvent(
          forward: true,
          isEnd: true,
          granularity: TextGranularity.word,
        ),
      );
      selection = paragraph.selections[0];
      expect(selection.start, 4); // how [are] you
      expect(selection.end, 7);

      // Equivalent to sending shift + alt + arrow-left.
      registrar.selectables[0].dispatchSelectionEvent(
        const GranularlyExtendSelectionEvent(
          forward: false,
          isEnd: true,
          granularity: TextGranularity.word,
        ),
      );
      expect(paragraph.selections.length, 1); // how []are you
      expect(paragraph.selections[0], const TextSelection.collapsed(offset: 4));

      // Equivalent to sending shift + alt + arrow-left.
      registrar.selectables[0].dispatchSelectionEvent(
        const GranularlyExtendSelectionEvent(
          forward: false,
          isEnd: true,
          granularity: TextGranularity.word,
        ),
      );
      selection = paragraph.selections[0];
      expect(selection.start, 0); // [how ]are you
      expect(selection.end, 4);
    });

    test('can granularly extend selection - line', () async {
      final TestSelectionRegistrar registrar = TestSelectionRegistrar();
      final List<RenderBox> renderBoxes = <RenderBox>[];
      final RenderParagraph paragraph = RenderParagraph(
        const TextSpan(children: <InlineSpan>[TextSpan(text: 'how are you\nI am fine\nThank you')]),
        textDirection: TextDirection.ltr,
        registrar: registrar,
        children: renderBoxes,
      );
      layout(paragraph);

      expect(registrar.selectables.length, 1);
      selectionParagraph(paragraph, const TextPosition(offset: 4), const TextPosition(offset: 5));
      expect(paragraph.selections.length, 1);
      TextSelection selection = paragraph.selections[0];
      expect(selection.start, 4); // how [a]re you
      expect(selection.end, 5);

      // Equivalent to sending shift + meta + arrow-right.
      registrar.selectables[0].dispatchSelectionEvent(
        const GranularlyExtendSelectionEvent(
          forward: true,
          isEnd: true,
          granularity: TextGranularity.line,
        ),
      );
      selection = paragraph.selections[0];
      // how [are you]
      expect(selection, const TextRange(start: 4, end: 11));

      // Equivalent to sending shift + meta + arrow-left.
      registrar.selectables[0].dispatchSelectionEvent(
        const GranularlyExtendSelectionEvent(
          forward: false,
          isEnd: true,
          granularity: TextGranularity.line,
        ),
      );
      selection = paragraph.selections[0];
      // [how ]are you
      expect(selection, const TextRange(start: 0, end: 4));
    });

    test('can granularly extend selection - document', () async {
      final TestSelectionRegistrar registrar = TestSelectionRegistrar();
      final List<RenderBox> renderBoxes = <RenderBox>[];
      final RenderParagraph paragraph = RenderParagraph(
        const TextSpan(children: <InlineSpan>[TextSpan(text: 'how are you\nI am fine\nThank you')]),
        textDirection: TextDirection.ltr,
        registrar: registrar,
        children: renderBoxes,
      );
      layout(paragraph);

      expect(registrar.selectables.length, 1);
      selectionParagraph(paragraph, const TextPosition(offset: 14), const TextPosition(offset: 15));
      expect(paragraph.selections.length, 1);
      TextSelection selection = paragraph.selections[0];
      // how are you
      // I [a]m fine
      expect(selection.start, 14);
      expect(selection.end, 15);

      // Equivalent to sending shift + meta + arrow-down.
      registrar.selectables[0].dispatchSelectionEvent(
        const GranularlyExtendSelectionEvent(
          forward: true,
          isEnd: true,
          granularity: TextGranularity.document,
        ),
      );
      selection = paragraph.selections[0];
      // how are you
      // I [am fine
      // Thank you]
      expect(selection.start, 14);
      expect(selection.end, 31);

      // Equivalent to sending shift + meta + arrow-up.
      registrar.selectables[0].dispatchSelectionEvent(
        const GranularlyExtendSelectionEvent(
          forward: false,
          isEnd: true,
          granularity: TextGranularity.document,
        ),
      );
      selection = paragraph.selections[0];
      // [how are you
      // I ]am fine
      // Thank you
      expect(selection.start, 0);
      expect(selection.end, 14);
    });

    test('can granularly extend selection when no active selection', () async {
      final TestSelectionRegistrar registrar = TestSelectionRegistrar();
      final List<RenderBox> renderBoxes = <RenderBox>[];
      final RenderParagraph paragraph = RenderParagraph(
        const TextSpan(children: <InlineSpan>[TextSpan(text: 'how are you\nI am fine\nThank you')]),
        textDirection: TextDirection.ltr,
        registrar: registrar,
        children: renderBoxes,
      );
      layout(paragraph);

      expect(registrar.selectables.length, 1);
      expect(paragraph.selections.length, 0);

      // Equivalent to sending shift + alt + right.
      registrar.selectables[0].dispatchSelectionEvent(
        const GranularlyExtendSelectionEvent(
          forward: true,
          isEnd: true,
          granularity: TextGranularity.word,
        ),
      );
      TextSelection selection = paragraph.selections[0];
      // [how] are you
      // I am fine
      // Thank you
      expect(selection.start, 0);
      expect(selection.end, 3);

      // Remove selection
      registrar.selectables[0].dispatchSelectionEvent(const ClearSelectionEvent());
      expect(paragraph.selections.length, 0);

      // Equivalent to sending shift + alt + left.
      registrar.selectables[0].dispatchSelectionEvent(
        const GranularlyExtendSelectionEvent(
          forward: false,
          isEnd: true,
          granularity: TextGranularity.word,
        ),
      );
      selection = paragraph.selections[0];
      // how are you
      // I am fine
      // Thank [you]
      expect(selection.start, 28);
      expect(selection.end, 31);
    });

    test('can directionally extend selection', () async {
      final TestSelectionRegistrar registrar = TestSelectionRegistrar();
      final List<RenderBox> renderBoxes = <RenderBox>[];
      final RenderParagraph paragraph = RenderParagraph(
        const TextSpan(children: <InlineSpan>[TextSpan(text: 'how are you\nI am fine\nThank you')]),
        textDirection: TextDirection.ltr,
        registrar: registrar,
        children: renderBoxes,
      );
      layout(paragraph);

      expect(registrar.selectables.length, 1);
      selectionParagraph(paragraph, const TextPosition(offset: 14), const TextPosition(offset: 15));
      expect(paragraph.selections.length, 1);
      TextSelection selection = paragraph.selections[0];
      // how are you
      // I [a]m fine
      expect(selection.start, 14);
      expect(selection.end, 15);

      final Matrix4 transform = registrar.selectables[0].getTransformTo(null);
      final double baseline =
          MatrixUtils.transformPoint(
            transform,
            registrar.selectables[0].value.endSelectionPoint!.localPosition,
          ).dx;

      // Equivalent to sending shift + arrow-down.
      registrar.selectables[0].dispatchSelectionEvent(
        DirectionallyExtendSelectionEvent(
          isEnd: true,
          dx: baseline,
          direction: SelectionExtendDirection.nextLine,
        ),
      );
      selection = paragraph.selections[0];
      // how are you
      // I [am fine
      // Tha]nk you
      expect(selection.start, 14);
      expect(selection.end, 25);

      // Equivalent to sending shift + arrow-up.
      registrar.selectables[0].dispatchSelectionEvent(
        DirectionallyExtendSelectionEvent(
          isEnd: true,
          dx: baseline,
          direction: SelectionExtendDirection.previousLine,
        ),
      );
      selection = paragraph.selections[0];
      // how are you
      // I [a]m fine
      // Thank you
      expect(selection.start, 14);
      expect(selection.end, 15);
    });

    test('can directionally extend selection when no selection', () async {
      final TestSelectionRegistrar registrar = TestSelectionRegistrar();
      final List<RenderBox> renderBoxes = <RenderBox>[];
      final RenderParagraph paragraph = RenderParagraph(
        const TextSpan(children: <InlineSpan>[TextSpan(text: 'how are you\nI am fine\nThank you')]),
        textDirection: TextDirection.ltr,
        registrar: registrar,
        children: renderBoxes,
      );
      layout(paragraph);

      expect(registrar.selectables.length, 1);
      expect(paragraph.selections.length, 0);

      final Matrix4 transform = registrar.selectables[0].getTransformTo(null);
      final double baseline =
          MatrixUtils.transformPoint(
            transform,
            Offset(registrar.selectables[0].size.width / 2, 0),
          ).dx;

      // Equivalent to sending shift + arrow-down.
      registrar.selectables[0].dispatchSelectionEvent(
        DirectionallyExtendSelectionEvent(
          isEnd: true,
          dx: baseline,
          direction: SelectionExtendDirection.forward,
        ),
      );
      TextSelection selection = paragraph.selections[0];
      // [how ar]e you
      // I am fine
      // Thank you
      expect(selection.start, 0);
      expect(selection.end, 6);

      registrar.selectables[0].dispatchSelectionEvent(const ClearSelectionEvent());
      expect(paragraph.selections.length, 0);

      // Equivalent to sending shift + arrow-up.
      registrar.selectables[0].dispatchSelectionEvent(
        DirectionallyExtendSelectionEvent(
          isEnd: true,
          dx: baseline,
          direction: SelectionExtendDirection.backward,
        ),
      );
      selection = paragraph.selections[0];
      // how are you
      // I am fine
      // Thank [you]
      expect(selection.start, 28);
      expect(selection.end, 31);
    });
  });

  test('can just update the gesture recognizer', () async {
    final TapGestureRecognizer recognizerBefore = TapGestureRecognizer()..onTap = () {};
    final RenderParagraph paragraph = RenderParagraph(
      TextSpan(text: 'How are you \n', recognizer: recognizerBefore),
      textDirection: TextDirection.ltr,
    );

    int semanticsUpdateCount = 0;
    final SemanticsHandle semanticsHandle = TestRenderingFlutterBinding.instance.ensureSemantics();
    TestRenderingFlutterBinding.instance.pipelineOwner.semanticsOwner!.addListener(() {
      ++semanticsUpdateCount;
    });

    layout(paragraph);

    expect((paragraph.text as TextSpan).recognizer, same(recognizerBefore));
    final SemanticsNode nodeBefore = SemanticsNode();
    paragraph.assembleSemanticsNode(nodeBefore, SemanticsConfiguration(), <SemanticsNode>[]);
    expect(semanticsUpdateCount, 0);
    List<SemanticsNode> children = <SemanticsNode>[];
    nodeBefore.visitChildren((SemanticsNode child) {
      children.add(child);
      return true;
    });
    SemanticsData data = children.single.getSemanticsData();
    expect(data.hasAction(SemanticsAction.longPress), false);
    expect(data.hasAction(SemanticsAction.tap), true);

    final LongPressGestureRecognizer recognizerAfter =
        LongPressGestureRecognizer()..onLongPress = () {};
    paragraph.text = TextSpan(text: 'How are you \n', recognizer: recognizerAfter);

    pumpFrame(phase: EnginePhase.flushSemantics);

    expect((paragraph.text as TextSpan).recognizer, same(recognizerAfter));
    final SemanticsNode nodeAfter = SemanticsNode();
    paragraph.assembleSemanticsNode(nodeAfter, SemanticsConfiguration(), <SemanticsNode>[]);
    expect(semanticsUpdateCount, 1);
    children = <SemanticsNode>[];
    nodeAfter.visitChildren((SemanticsNode child) {
      children.add(child);
      return true;
    });
    data = children.single.getSemanticsData();
    expect(data.hasAction(SemanticsAction.longPress), true);
    expect(data.hasAction(SemanticsAction.tap), false);

    semanticsHandle.dispose();
  });
}

class MockCanvas extends Fake implements Canvas {
  Rect? drawnRect;
  Paint? drawnRectPaint;
  List<Type> drawnItemTypes = <Type>[];

  @override
  void drawRect(Rect rect, Paint paint) {
    drawnRect = rect;
    drawnRectPaint = paint;
    drawnItemTypes.add(Rect);
  }

  @override
  void drawParagraph(ui.Paragraph paragraph, Offset offset) {
    drawnItemTypes.add(ui.Paragraph);
  }

  void clear() {
    drawnRect = null;
    drawnRectPaint = null;
    drawnItemTypes.clear();
  }
}

class MockPaintingContext extends Fake implements PaintingContext {
  @override
  final MockCanvas canvas = MockCanvas();
}

class TestSelectionRegistrar extends SelectionRegistrar {
  final List<Selectable> selectables = <Selectable>[];
  @override
  void add(Selectable selectable) {
    selectables.add(selectable);
  }

  @override
  void remove(Selectable selectable) {
    expect(selectables.remove(selectable), isTrue);
  }
}
