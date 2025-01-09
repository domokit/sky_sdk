// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../../common/test_initialization.dart';
import '../paragraph/helper.dart';
import 'layout_service_helper.dart';

const bool skipWordSpacing = true;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(withImplicitView: true);

  test('no text', () {
    final CanvasParagraph paragraph = CanvasParagraphBuilder(ahemStyle).build();
    paragraph.layout(constrain(double.infinity));

    expect(paragraph.maxIntrinsicWidth, 0);
    expect(paragraph.minIntrinsicWidth, 0);
    expect(paragraph.height, 10);
    expectLines(paragraph, <TestLine>[l('', 0, 0, width: 0.0, height: 10.0, baseline: 8.0)]);
  });

  test('preserves whitespace when measuring', () {
    CanvasParagraph paragraph;

    // leading whitespaces
    paragraph = plain(ahemStyle, '   abc')..layout(constrain(double.infinity));
    expect(paragraph.maxIntrinsicWidth, 60);
    expect(paragraph.minIntrinsicWidth, 30);
    expect(paragraph.height, 10);
    expectLines(paragraph, <TestLine>[l('   abc', 0, 6, hardBreak: true, width: 60.0)]);

    // trailing whitespaces
    paragraph = plain(ahemStyle, 'abc   ')..layout(constrain(double.infinity));
    expect(paragraph.maxIntrinsicWidth, 60);
    expect(paragraph.minIntrinsicWidth, 30);
    expect(paragraph.height, 10);
    expectLines(paragraph, <TestLine>[l('abc   ', 0, 6, hardBreak: true, width: 30.0)]);

    // mixed whitespaces
    paragraph = plain(ahemStyle, '  ab   c  ')..layout(constrain(double.infinity));
    expect(paragraph.maxIntrinsicWidth, 100);
    expect(paragraph.minIntrinsicWidth, 20);
    expect(paragraph.height, 10);
    expectLines(paragraph, <TestLine>[
      l('  ab   c  ', 0, 10, hardBreak: true, width: 80.0, left: 0.0),
    ]);

    // single whitespace
    paragraph = plain(ahemStyle, ' ')..layout(constrain(double.infinity));
    expect(paragraph.maxIntrinsicWidth, 10);
    expect(paragraph.minIntrinsicWidth, 0);
    expect(paragraph.height, 10);
    expectLines(paragraph, <TestLine>[l(' ', 0, 1, hardBreak: true, width: 0.0, left: 0.0)]);

    // whitespace only
    paragraph = plain(ahemStyle, '     ')..layout(constrain(double.infinity));
    expect(paragraph.maxIntrinsicWidth, 50);
    expect(paragraph.minIntrinsicWidth, 0);
    expect(paragraph.height, 10);
    expectLines(paragraph, <TestLine>[l('     ', 0, 5, hardBreak: true, width: 0.0, left: 0.0)]);
  });

  test('uses single-line when text can fit without wrapping', () {
    final CanvasParagraph paragraph = plain(ahemStyle, '12345')..layout(constrain(50.0));

    // Should fit on a single line.
    expect(paragraph.alphabeticBaseline, 8);
    expect(paragraph.maxIntrinsicWidth, 50);
    expect(paragraph.minIntrinsicWidth, 50);
    expect(paragraph.width, 50);
    expect(paragraph.height, 10);
    expectLines(paragraph, <TestLine>[
      l('12345', 0, 5, hardBreak: true, width: 50.0, left: 0.0, height: 10.0, baseline: 8.0),
    ]);
  });

  test('simple multi-line text', () {
    final CanvasParagraph paragraph = plain(ahemStyle, 'foo bar baz')..layout(constrain(70.0));
    expect(paragraph.alphabeticBaseline, 8);
    expect(paragraph.maxIntrinsicWidth, 110);
    expect(paragraph.minIntrinsicWidth, 30);
    expect(paragraph.width, 70);
    expect(paragraph.height, 20);
    expectLines(paragraph, <TestLine>[
      l('foo bar ', 0, 8, hardBreak: false, width: 70.0, left: 0.0, height: 10.0, baseline: 8.0),
      l('baz', 8, 11, hardBreak: true, width: 30.0, left: 0.0, height: 10.0, baseline: 18.0),
    ]);
  });

  test('uses multi-line for long text', () {
    CanvasParagraph paragraph;

    // The long text doesn't fit in 50px of width, so it needs to wrap.
    paragraph = plain(ahemStyle, '1234567890')..layout(constrain(50.0));
    expect(paragraph.alphabeticBaseline, 8);
    expect(paragraph.maxIntrinsicWidth, 100);
    expect(paragraph.minIntrinsicWidth, 100);
    expect(paragraph.width, 50);
    expect(paragraph.height, 20);
    expectLines(paragraph, <TestLine>[
      l('12345', 0, 5, hardBreak: false, width: 50.0, left: 0.0, height: 10.0, baseline: 8.0),
      l('67890', 5, 10, hardBreak: true, width: 50.0, left: 0.0, height: 10.0, baseline: 18.0),
    ]);

    // The first word is force-broken twice.
    paragraph = plain(ahemStyle, 'abcdefghijk lm')..layout(constrain(50.0));
    expect(paragraph.alphabeticBaseline, 8);
    expect(paragraph.maxIntrinsicWidth, 140);
    expect(paragraph.minIntrinsicWidth, 110);
    expect(paragraph.width, 50);
    expect(paragraph.height, 30);
    expectLines(paragraph, <TestLine>[
      l('abcde', 0, 5, hardBreak: false, width: 50.0, left: 0.0, height: 10.0, baseline: 8.0),
      l('fghij', 5, 10, hardBreak: false, width: 50.0, left: 0.0, height: 10.0, baseline: 18.0),
      l('k lm', 10, 14, hardBreak: true, width: 40.0, left: 0.0, height: 10.0, baseline: 28.0),
    ]);

    // Constraints enough only for "abcdef" but not for the trailing space.
    paragraph = plain(ahemStyle, 'abcdef gh')..layout(constrain(60.0));
    expect(paragraph.alphabeticBaseline, 8);
    expect(paragraph.maxIntrinsicWidth, 90);
    expect(paragraph.minIntrinsicWidth, 60);
    expect(paragraph.width, 60);
    expect(paragraph.height, 20);
    expectLines(paragraph, <TestLine>[
      l('abcdef ', 0, 7, hardBreak: false, width: 60.0, left: 0.0, height: 10.0, baseline: 8.0),
      l('gh', 7, 9, hardBreak: true, width: 20.0, left: 0.0, height: 10.0, baseline: 18.0),
    ]);

    // Constraints aren't enough even for a single character. In this case,
    // we show a minimum of one character per line.
    paragraph = plain(ahemStyle, 'AA')..layout(constrain(8.0));
    expect(paragraph.alphabeticBaseline, 8);
    expect(paragraph.maxIntrinsicWidth, 20);
    expect(paragraph.minIntrinsicWidth, 20);
    expect(paragraph.width, 8);
    expect(paragraph.height, 20);
    expectLines(paragraph, <TestLine>[
      l('A', 0, 1, hardBreak: false, width: 10.0, left: 0.0, height: 10.0, baseline: 8.0),
      l('A', 1, 2, hardBreak: true, width: 10.0, left: 0.0, height: 10.0, baseline: 18.0),
    ]);

    // Extremely narrow constraints with new line in the middle.
    paragraph = plain(ahemStyle, 'AA\nA')..layout(constrain(8.0));
    expect(paragraph.alphabeticBaseline, 8);
    expect(paragraph.maxIntrinsicWidth, 20);
    expect(paragraph.minIntrinsicWidth, 20);
    expect(paragraph.width, 8);
    expect(paragraph.height, 30);
    expectLines(paragraph, <TestLine>[
      l('A', 0, 1, hardBreak: false, width: 10.0, left: 0.0, height: 10.0, baseline: 8.0),
      l('A', 1, 3, hardBreak: true, width: 10.0, left: 0.0, height: 10.0, baseline: 18.0),
      l('A', 3, 4, hardBreak: true, width: 10.0, left: 0.0, height: 10.0, baseline: 28.0),
    ]);

    // Extremely narrow constraints with new line in the end.
    paragraph = plain(ahemStyle, 'AAA\n')..layout(constrain(8.0));
    expect(paragraph.alphabeticBaseline, 8);
    expect(paragraph.maxIntrinsicWidth, 30);
    expect(paragraph.minIntrinsicWidth, 30);
    expect(paragraph.width, 8);
    expect(paragraph.height, 40);
    expectLines(paragraph, <TestLine>[
      l('A', 0, 1, hardBreak: false, width: 10.0, left: 0.0, height: 10.0, baseline: 8.0),
      l('A', 1, 2, hardBreak: false, width: 10.0, left: 0.0, height: 10.0, baseline: 18.0),
      l('A', 2, 4, hardBreak: true, width: 10.0, left: 0.0, height: 10.0, baseline: 28.0),
      l('', 4, 4, hardBreak: true, width: 0.0, left: 0.0, height: 10.0, baseline: 38.0),
    ]);
  });

  test('uses multi-line for text that contains new-line', () {
    final CanvasParagraph paragraph = plain(ahemStyle, '12\n34')..layout(constrain(50.0));

    // Text containing newlines should always be drawn in multi-line mode.
    expect(paragraph.maxIntrinsicWidth, 20);
    expect(paragraph.minIntrinsicWidth, 20);
    expect(paragraph.width, 50);
    expect(paragraph.height, 20);
    expectLines(paragraph, <TestLine>[
      l('12', 0, 3, hardBreak: true, width: 20.0, left: 0.0),
      l('34', 3, 5, hardBreak: true, width: 20.0, left: 0.0),
    ]);
  });

  test('empty lines', () {
    CanvasParagraph paragraph;

    // Empty lines in the beginning.
    paragraph = plain(ahemStyle, '\n\n1234')..layout(constrain(50.0));
    expect(paragraph.maxIntrinsicWidth, 40);
    expect(paragraph.minIntrinsicWidth, 40);
    expect(paragraph.height, 30);
    expectLines(paragraph, <TestLine>[
      l('', 0, 1, hardBreak: true, width: 0.0, left: 0.0),
      l('', 1, 2, hardBreak: true, width: 0.0, left: 0.0),
      l('1234', 2, 6, hardBreak: true, width: 40.0, left: 0.0),
    ]);

    // Empty lines in the middle.
    paragraph = plain(ahemStyle, '12\n\n345')..layout(constrain(50.0));
    expect(paragraph.maxIntrinsicWidth, 30);
    expect(paragraph.minIntrinsicWidth, 30);
    expect(paragraph.height, 30);
    expectLines(paragraph, <TestLine>[
      l('12', 0, 3, hardBreak: true, width: 20.0, left: 0.0),
      l('', 3, 4, hardBreak: true, width: 0.0, left: 0.0),
      l('345', 4, 7, hardBreak: true, width: 30.0, left: 0.0),
    ]);

    // Empty lines in the end.
    paragraph = plain(ahemStyle, '1234\n\n')..layout(constrain(50.0));
    expect(paragraph.maxIntrinsicWidth, 40);
    expect(paragraph.minIntrinsicWidth, 40);
    expect(paragraph.height, 30);
    expectLines(paragraph, <TestLine>[
      l('1234', 0, 5, hardBreak: true, width: 40.0, left: 0.0),
      l('', 5, 6, hardBreak: true, width: 0.0, left: 0.0),
      l('', 6, 6, hardBreak: true, width: 0.0, left: 0.0),
    ]);
  });

  test('wraps multi-line text correctly when constraint width is infinite', () {
    final CanvasParagraph paragraph = plain(ahemStyle, '123\n456 789')
      ..layout(constrain(double.infinity));

    expect(paragraph.maxIntrinsicWidth, 70);
    expect(paragraph.minIntrinsicWidth, 30);
    expect(paragraph.width, double.infinity);
    expect(paragraph.height, 20);
    expectLines(paragraph, <TestLine>[
      l('123', 0, 4, hardBreak: true, width: 30.0, left: 0.0),
      l('456 789', 4, 11, hardBreak: true, width: 70.0, left: 0.0),
    ]);
  });

  test('takes letter spacing into account', () {
    final EngineTextStyle spacedTextStyle = EngineTextStyle.only(letterSpacing: 3);
    final CanvasParagraph spacedText = plain(ahemStyle, 'abc', textStyle: spacedTextStyle)
      ..layout(constrain(100.0));

    expect(spacedText.minIntrinsicWidth, 39);
    expect(spacedText.maxIntrinsicWidth, 39);
  });

  test('takes word spacing into account', () {
    final CanvasParagraph normalText = plain(ahemStyle, 'a b c')..layout(constrain(100.0));
    final CanvasParagraph spacedText = plain(
      ahemStyle,
      'a b c',
      textStyle: EngineTextStyle.only(wordSpacing: 1.5),
    )..layout(constrain(100.0));

    expect(normalText.maxIntrinsicWidth < spacedText.maxIntrinsicWidth, isTrue);
  }, skip: skipWordSpacing);

  test('minIntrinsicWidth', () {
    CanvasParagraph paragraph;

    // Simple case.
    paragraph = plain(ahemStyle, 'abc de fghi')..layout(constrain(50.0));
    expect(paragraph.minIntrinsicWidth, 40);
    expectLines(paragraph, <TestLine>[
      l('abc ', 0, 4, hardBreak: false, width: 30.0, left: 0.0),
      l('de ', 4, 7, hardBreak: false, width: 20.0, left: 0.0),
      l('fghi', 7, 11, hardBreak: true, width: 40.0, left: 0.0),
    ]);

    // With new lines.
    paragraph = plain(ahemStyle, 'abcd\nef\nghi')..layout(constrain(50.0));
    expect(paragraph.minIntrinsicWidth, 40);
    expectLines(paragraph, <TestLine>[
      l('abcd', 0, 5, hardBreak: true, width: 40.0, left: 0.0),
      l('ef', 5, 8, hardBreak: true, width: 20.0, left: 0.0),
      l('ghi', 8, 11, hardBreak: true, width: 30.0, left: 0.0),
    ]);

    // With trailing whitespace.
    paragraph = plain(ahemStyle, 'abcd      efg')..layout(constrain(50.0));
    expect(paragraph.minIntrinsicWidth, 40);
    expectLines(paragraph, <TestLine>[
      l('abcd      ', 0, 10, hardBreak: false, width: 40.0, left: 0.0),
      l('efg', 10, 13, hardBreak: true, width: 30.0, left: 0.0),
    ]);

    // With trailing whitespace and new lines.
    paragraph = plain(ahemStyle, 'abc    \ndefg')..layout(constrain(50.0));
    expect(paragraph.minIntrinsicWidth, 40);
    expectLines(paragraph, <TestLine>[
      l('abc    ', 0, 8, hardBreak: true, width: 30.0, left: 0.0),
      l('defg', 8, 12, hardBreak: true, width: 40.0, left: 0.0),
    ]);

    // Very long text.
    paragraph = plain(ahemStyle, 'AAAAAAAAAAAA')..layout(constrain(50.0));
    expect(paragraph.minIntrinsicWidth, 120);
    expectLines(paragraph, <TestLine>[
      l('AAAAA', 0, 5, hardBreak: false, width: 50.0, left: 0.0),
      l('AAAAA', 5, 10, hardBreak: false, width: 50.0, left: 0.0),
      l('AA', 10, 12, hardBreak: true, width: 20.0, left: 0.0),
    ]);
  });

  test('maxIntrinsicWidth', () {
    CanvasParagraph paragraph;

    // Simple case.
    paragraph = plain(ahemStyle, 'abc de fghi')..layout(constrain(50.0));
    expect(paragraph.maxIntrinsicWidth, 110);
    expectLines(paragraph, <TestLine>[
      l('abc ', 0, 4, hardBreak: false, width: 30.0, left: 0.0),
      l('de ', 4, 7, hardBreak: false, width: 20.0, left: 0.0),
      l('fghi', 7, 11, hardBreak: true, width: 40.0, left: 0.0),
    ]);

    // With new lines.
    paragraph = plain(ahemStyle, 'abcd\nef\nghi')..layout(constrain(50.0));
    expect(paragraph.maxIntrinsicWidth, 40);
    expectLines(paragraph, <TestLine>[
      l('abcd', 0, 5, hardBreak: true, width: 40.0, left: 0.0),
      l('ef', 5, 8, hardBreak: true, width: 20.0, left: 0.0),
      l('ghi', 8, 11, hardBreak: true, width: 30.0, left: 0.0),
    ]);

    // With long whitespace.
    paragraph = plain(ahemStyle, 'abcd   efg')..layout(constrain(50.0));
    expect(paragraph.maxIntrinsicWidth, 100);
    expectLines(paragraph, <TestLine>[
      l('abcd   ', 0, 7, hardBreak: false, width: 40.0, left: 0.0),
      l('efg', 7, 10, hardBreak: true, width: 30.0, left: 0.0),
    ]);

    // With trailing whitespace.
    paragraph = plain(ahemStyle, 'abc def   ')..layout(constrain(50.0));
    expect(paragraph.maxIntrinsicWidth, 100);
    expectLines(paragraph, <TestLine>[
      l('abc ', 0, 4, hardBreak: false, width: 30.0, left: 0.0),
      l('def   ', 4, 10, hardBreak: true, width: 30.0, left: 0.0),
    ]);

    // With trailing whitespace and new lines.
    paragraph = plain(ahemStyle, 'abc \ndef   ')..layout(constrain(50.0));
    expect(paragraph.maxIntrinsicWidth, 60);
    expectLines(paragraph, <TestLine>[
      l('abc ', 0, 5, hardBreak: true, width: 30.0, left: 0.0),
      l('def   ', 5, 11, hardBreak: true, width: 30.0, left: 0.0),
    ]);

    // Very long text.
    paragraph = plain(ahemStyle, 'AAAAAAAAAAAA')..layout(constrain(50.0));
    expect(paragraph.maxIntrinsicWidth, 120);
    expectLines(paragraph, <TestLine>[
      l('AAAAA', 0, 5, hardBreak: false, width: 50.0, left: 0.0),
      l('AAAAA', 5, 10, hardBreak: false, width: 50.0, left: 0.0),
      l('AA', 10, 12, hardBreak: true, width: 20.0, left: 0.0),
    ]);
  });

  test('respects text overflow', () {
    final EngineParagraphStyle overflowStyle = EngineParagraphStyle(
      fontFamily: 'Ahem',
      fontSize: 10,
      ellipsis: '...',
    );

    // The text shouldn't be broken into multiple lines, so the height should
    // be equal to a height of a single line.
    final CanvasParagraph longText = plain(
      overflowStyle,
      'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA',
    )..layout(constrain(50.0));
    expect(longText.minIntrinsicWidth, 480);
    expect(longText.maxIntrinsicWidth, 480);
    expect(longText.height, 10);
    expectLines(longText, <TestLine>[l('AA...', 0, 2, hardBreak: true, width: 50.0, left: 0.0)]);

    // The short prefix should make the text break into two lines, but the
    // second line should remain unbroken.
    final CanvasParagraph longTextShortPrefix = plain(
      overflowStyle,
      'AAA\nAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA',
    )..layout(constrain(50.0));
    expect(longTextShortPrefix.minIntrinsicWidth, 450);
    expect(longTextShortPrefix.maxIntrinsicWidth, 450);
    expect(longTextShortPrefix.height, 20);
    expectLines(longTextShortPrefix, <TestLine>[
      l('AAA', 0, 4, hardBreak: true, width: 30.0, left: 0.0),
      l('AA...', 4, 6, hardBreak: true, width: 50.0, left: 0.0),
    ]);

    // Constraints only enough to fit "AA" with the ellipsis, but not the
    // trailing white space.
    final CanvasParagraph trailingSpace = plain(overflowStyle, 'AA AAA')..layout(constrain(50.0));
    expect(trailingSpace.minIntrinsicWidth, 30);
    expect(trailingSpace.maxIntrinsicWidth, 60);
    expect(trailingSpace.height, 10);
    expectLines(trailingSpace, <TestLine>[
      l('AA...', 0, 2, hardBreak: true, width: 50.0, left: 0.0),
    ]);

    // Tiny constraints.
    final CanvasParagraph paragraph = plain(overflowStyle, 'AAAA')..layout(constrain(30.0));
    expect(paragraph.minIntrinsicWidth, 40);
    expect(paragraph.maxIntrinsicWidth, 40);
    expect(paragraph.height, 10);
    expectLines(paragraph, <TestLine>[l('...', 0, 0, hardBreak: true, width: 30.0, left: 0.0)]);

    // Tinier constraints (not enough for the ellipsis).
    paragraph.layout(constrain(10.0));
    expect(paragraph.minIntrinsicWidth, 40);
    expect(paragraph.maxIntrinsicWidth, 40);
    expect(paragraph.height, 10);

    // TODO(mdebbar): https://github.com/flutter/flutter/issues/34346
    // expectLines(paragraph, <TestLine>[
    //   l('.', 0, 0, hardBreak: false, width: 10.0, left: 0.0),
    // ]);
    expectLines(paragraph, <TestLine>[l('...', 0, 0, hardBreak: true, width: 30.0, left: 0.0)]);
  });

  test('respects max lines', () {
    final EngineParagraphStyle maxlinesStyle = EngineParagraphStyle(
      fontFamily: 'Ahem',
      fontSize: 10,
      maxLines: 2,
    );

    // The height should be that of a single line.
    final CanvasParagraph oneline = plain(maxlinesStyle, 'One line')
      ..layout(constrain(double.infinity));
    expect(oneline.height, 10);
    expectLines(oneline, <TestLine>[l('One line', 0, 8, hardBreak: true, width: 80.0, left: 0.0)]);

    // The height should respect max lines and be limited to two lines here.
    final CanvasParagraph threelines = plain(maxlinesStyle, 'First\nSecond\nThird')
      ..layout(constrain(double.infinity));
    expect(threelines.height, 20);
    expectLines(threelines, <TestLine>[
      l('First', 0, 6, hardBreak: true, width: 50.0, left: 0.0),
      l('Second', 6, 13, hardBreak: true, width: 60.0, left: 0.0),
    ]);

    // The height should respect max lines and be limited to two lines here.
    final CanvasParagraph veryLong = plain(
      maxlinesStyle,
      'Lorem ipsum dolor sit amet, consectetur adipiscing elit.',
    )..layout(constrain(50.0));
    expect(veryLong.height, 20);
    expectLines(veryLong, <TestLine>[
      l('Lorem ', 0, 6, hardBreak: false, width: 50.0, left: 0.0),
      l('ipsum ', 6, 12, hardBreak: false, width: 50.0, left: 0.0),
    ]);

    // Case when last line is a long unbreakable word.
    final CanvasParagraph veryLongLastLine = plain(maxlinesStyle, 'AAA AAAAAAAAAAAAAAAAAAA')
      ..layout(constrain(50.0));
    expect(veryLongLastLine.height, 20);
    expectLines(veryLongLastLine, <TestLine>[
      l('AAA ', 0, 4, hardBreak: false, width: 30.0, left: 0.0),
      l('AAAAA', 4, 9, hardBreak: false, width: 50.0, left: 0.0),
    ]);
  });

  test('respects text overflow and max lines combined', () {
    final EngineParagraphStyle onelineStyle = EngineParagraphStyle(
      fontFamily: 'Ahem',
      fontSize: 10,
      maxLines: 1,
      ellipsis: '...',
    );
    final EngineParagraphStyle multilineStyle = EngineParagraphStyle(
      fontFamily: 'Ahem',
      fontSize: 10,
      maxLines: 2,
      ellipsis: '...',
    );

    CanvasParagraph paragraph;

    // Simple no overflow case.
    paragraph = plain(onelineStyle, 'abcdef')..layout(constrain(60.0));
    expect(paragraph.height, 10);
    expectLines(paragraph, <TestLine>[l('abcdef', 0, 6, hardBreak: true, width: 60.0, left: 0.0)]);

    // Simple overflow case.
    paragraph = plain(onelineStyle, 'abcd efg')..layout(constrain(60.0));
    expect(paragraph.height, 10);
    expectLines(paragraph, <TestLine>[l('abc...', 0, 3, hardBreak: true, width: 60.0, left: 0.0)]);

    // Another simple overflow case.
    paragraph = plain(onelineStyle, 'a bcde fgh')..layout(constrain(60.0));
    expect(paragraph.height, 10);
    expectLines(paragraph, <TestLine>[l('a b...', 0, 3, hardBreak: true, width: 60.0, left: 0.0)]);

    // The ellipsis is supposed to go on the second line, but because the
    // 2nd line doesn't overflow, no ellipsis is shown.
    paragraph = plain(multilineStyle, 'abcdef ghijkl')..layout(constrain(60.0));
    expect(paragraph.height, 20);
    expectLines(paragraph, <TestLine>[
      l('abcdef ', 0, 7, hardBreak: false, width: 60.0, left: 0.0),
      l('ghijkl', 7, 13, hardBreak: true, width: 60.0, left: 0.0),
    ]);

    // But when the 2nd line is long enough, the ellipsis is shown.
    paragraph = plain(multilineStyle, 'abcd efghijkl')..layout(constrain(60.0));
    expect(paragraph.height, 20);
    expectLines(paragraph, <TestLine>[
      l('abcd ', 0, 5, hardBreak: false, width: 40.0, left: 0.0),
      l('efg...', 5, 8, hardBreak: true, width: 60.0, left: 0.0),
    ]);

    // Even if the second line can be broken, we don't break it, we just
    // insert the ellipsis.
    paragraph = plain(multilineStyle, 'abcde f gh ijk')..layout(constrain(60.0));
    expect(paragraph.height, 20);
    expectLines(paragraph, <TestLine>[
      l('abcde ', 0, 6, hardBreak: false, width: 50.0, left: 0.0),
      l('f g...', 6, 9, hardBreak: true, width: 60.0, left: 0.0),
    ]);

    // First line overflows but second line doesn't.
    paragraph = plain(multilineStyle, 'abcdefg hijk')..layout(constrain(60.0));
    expect(paragraph.height, 20);
    expectLines(paragraph, <TestLine>[
      l('abcdef', 0, 6, hardBreak: false, width: 60.0, left: 0.0),
      l('g hijk', 6, 12, hardBreak: true, width: 60.0, left: 0.0),
    ]);

    // Both first and second lines overflow.
    paragraph = plain(multilineStyle, 'abcdefg hijklmnop')..layout(constrain(60.0));
    expect(paragraph.height, 20);
    expectLines(paragraph, <TestLine>[
      l('abcdef', 0, 6, hardBreak: false, width: 60.0, left: 0.0),
      l('g h...', 6, 9, hardBreak: true, width: 60.0, left: 0.0),
    ]);
  });

  test('handles textAlign', () {
    CanvasParagraph paragraph;

    EngineParagraphStyle createStyle(ui.TextAlign textAlign) {
      return EngineParagraphStyle(
        fontFamily: 'Ahem',
        fontSize: 10,
        textAlign: textAlign,
        textDirection: ui.TextDirection.ltr,
      );
    }

    paragraph = plain(createStyle(ui.TextAlign.start), 'abc\ndefghi')..layout(constrain(50.0));
    expectLines(paragraph, <TestLine>[
      l('abc', 0, 4, hardBreak: true, width: 30.0, left: 0.0),
      l('defgh', 4, 9, hardBreak: false, width: 50.0, left: 0.0),
      l('i', 9, 10, hardBreak: true, width: 10.0, left: 0.0),
    ]);

    paragraph = plain(createStyle(ui.TextAlign.end), 'abc\ndefghi')..layout(constrain(50.0));
    expectLines(paragraph, <TestLine>[
      l('abc', 0, 4, hardBreak: true, width: 30.0, left: 20.0),
      l('defgh', 4, 9, hardBreak: false, width: 50.0, left: 0.0),
      l('i', 9, 10, hardBreak: true, width: 10.0, left: 40.0),
    ]);

    paragraph = plain(createStyle(ui.TextAlign.center), 'abc\ndefghi')..layout(constrain(50.0));
    expectLines(paragraph, <TestLine>[
      l('abc', 0, 4, hardBreak: true, width: 30.0, left: 10.0),
      l('defgh', 4, 9, hardBreak: false, width: 50.0, left: 0.0),
      l('i', 9, 10, hardBreak: true, width: 10.0, left: 20.0),
    ]);

    paragraph = plain(createStyle(ui.TextAlign.left), 'abc\ndefghi')..layout(constrain(50.0));
    expectLines(paragraph, <TestLine>[
      l('abc', 0, 4, hardBreak: true, width: 30.0, left: 0.0),
      l('defgh', 4, 9, hardBreak: false, width: 50.0, left: 0.0),
      l('i', 9, 10, hardBreak: true, width: 10.0, left: 0.0),
    ]);

    paragraph = plain(createStyle(ui.TextAlign.right), 'abc\ndefghi')..layout(constrain(50.0));
    expectLines(paragraph, <TestLine>[
      l('abc', 0, 4, hardBreak: true, width: 30.0, left: 20.0),
      l('defgh', 4, 9, hardBreak: false, width: 50.0, left: 0.0),
      l('i', 9, 10, hardBreak: true, width: 10.0, left: 40.0),
    ]);
  });

  test('handles rtl with textAlign', () {
    CanvasParagraph paragraph;

    EngineParagraphStyle createStyle(ui.TextAlign textAlign) {
      return EngineParagraphStyle(
        fontFamily: 'Ahem',
        fontSize: 10,
        textAlign: textAlign,
        textDirection: ui.TextDirection.rtl,
      );
    }

    paragraph = plain(createStyle(ui.TextAlign.start), 'abc\ndefghi')..layout(constrain(50.0));
    expectLines(paragraph, <TestLine>[
      l('abc', 0, 4, hardBreak: true, width: 30.0, left: 20.0),
      l('defgh', 4, 9, hardBreak: false, width: 50.0, left: 0.0),
      l('i', 9, 10, hardBreak: true, width: 10.0, left: 40.0),
    ]);

    paragraph = plain(createStyle(ui.TextAlign.end), 'abc\ndefghi')..layout(constrain(50.0));
    expectLines(paragraph, <TestLine>[
      l('abc', 0, 4, hardBreak: true, width: 30.0, left: 0.0),
      l('defgh', 4, 9, hardBreak: false, width: 50.0, left: 0.0),
      l('i', 9, 10, hardBreak: true, width: 10.0, left: 0.0),
    ]);

    paragraph = plain(createStyle(ui.TextAlign.center), 'abc\ndefghi')..layout(constrain(50.0));
    expectLines(paragraph, <TestLine>[
      l('abc', 0, 4, hardBreak: true, width: 30.0, left: 10.0),
      l('defgh', 4, 9, hardBreak: false, width: 50.0, left: 0.0),
      l('i', 9, 10, hardBreak: true, width: 10.0, left: 20.0),
    ]);

    paragraph = plain(createStyle(ui.TextAlign.left), 'abc\ndefghi')..layout(constrain(50.0));
    expectLines(paragraph, <TestLine>[
      l('abc', 0, 4, hardBreak: true, width: 30.0, left: 0.0),
      l('defgh', 4, 9, hardBreak: false, width: 50.0, left: 0.0),
      l('i', 9, 10, hardBreak: true, width: 10.0, left: 0.0),
    ]);

    paragraph = plain(createStyle(ui.TextAlign.right), 'abc\ndefghi')..layout(constrain(50.0));
    expectLines(paragraph, <TestLine>[
      l('abc', 0, 4, hardBreak: true, width: 30.0, left: 20.0),
      l('defgh', 4, 9, hardBreak: false, width: 50.0, left: 0.0),
      l('i', 9, 10, hardBreak: true, width: 10.0, left: 40.0),
    ]);
  });

  test('uses a single minimal canvas', () {
    debugResetCanvasCount();

    plain(ahemStyle, 'Lorem').layout(constrain(double.infinity));
    plain(ahemStyle, 'ipsum dolor').layout(constrain(150.0));
    // Try different styles too.
    plain(
      EngineParagraphStyle(fontWeight: ui.FontWeight.bold),
      'sit amet',
    ).layout(constrain(300.0));

    expect(textContext.canvas!.width, isZero);
    expect(textContext.canvas!.height, isZero);
    // This number is 0 instead of 1 because the canvas is created at the top
    // level as a global variable. So by the time this test runs, the canvas
    // would have been created already.
    //
    // So we just make sure that no new canvas is created after the above layout
    // calls.
    expect(debugCanvasCount, 0);
  });

  test('does not leak styles across spanometers', () {
    // This prevents the Ahem font from being forced in all paragraphs.
    ui_web.debugEmulateFlutterTesterEnvironment = false;

    final CanvasParagraph p1 = plain(
      EngineParagraphStyle(fontSize: 20.0, fontFamily: 'FontFamily1'),
      'Lorem',
    )..layout(constrain(double.infinity));
    // After the layout, the canvas should have the above style applied.
    expect(textContext.font, contains('20px'));
    expect(textContext.font, contains('FontFamily1'));

    final CanvasParagraph p2 = plain(
      EngineParagraphStyle(fontSize: 40.0, fontFamily: 'FontFamily2'),
      'ipsum dolor',
    )..layout(constrain(double.infinity));
    // After the layout, the canvas should have the above style applied.
    expect(textContext.font, contains('40px'));
    expect(textContext.font, contains('FontFamily2'));

    p1.getBoxesForRange(0, 2);
    // getBoxesForRange performs some text measurements. Let's make sure that it
    // applied the correct style.
    expect(textContext.font, contains('20px'));
    expect(textContext.font, contains('FontFamily1'));

    p2.getBoxesForRange(0, 4);
    // getBoxesForRange performs some text measurements. Let's make sure that it
    // applied the correct style.
    expect(textContext.font, contains('40px'));
    expect(textContext.font, contains('FontFamily2'));

    ui_web.debugEmulateFlutterTesterEnvironment = true;
  });
}
