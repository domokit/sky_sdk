// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('RichText with recognizers without handlers does not throw', (
    WidgetTester tester,
  ) async {
    final TapGestureRecognizer recognizer1 = TapGestureRecognizer();
    addTearDown(recognizer1.dispose);
    final LongPressGestureRecognizer recognizer2 = LongPressGestureRecognizer();
    addTearDown(recognizer2.dispose);
    final DoubleTapGestureRecognizer recognizer3 = DoubleTapGestureRecognizer();
    addTearDown(recognizer3.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: RichText(
          text: TextSpan(
            text: 'root',
            children: <InlineSpan>[
              TextSpan(text: 'one', recognizer: recognizer1),
              TextSpan(text: 'two', recognizer: recognizer2),
              TextSpan(text: 'three', recognizer: recognizer3),
            ],
          ),
        ),
      ),
    );

    expect(
      tester.getSemantics(find.byType(RichText)),
      matchesSemantics(
        children: <Matcher>[
          matchesSemantics(label: 'root'),
          matchesSemantics(label: 'one'),
          matchesSemantics(label: 'two'),
          matchesSemantics(label: 'three'),
        ],
      ),
    );
  });

  testWidgets('TextSpan Locale works', (WidgetTester tester) async {
    final TapGestureRecognizer recognizer1 = TapGestureRecognizer();
    addTearDown(recognizer1.dispose);
    final DoubleTapGestureRecognizer recognizer2 = DoubleTapGestureRecognizer();
    addTearDown(recognizer2.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: RichText(
          text: TextSpan(
            text: 'root',
            locale: const Locale('es', 'MX'),
            children: <InlineSpan>[
              TextSpan(text: 'one', recognizer: recognizer1),
              const WidgetSpan(child: SizedBox()),
              TextSpan(text: 'three', recognizer: recognizer2),
            ],
          ),
        ),
      ),
    );
    expect(
      tester.getSemantics(find.byType(RichText)),
      matchesSemantics(
        children: <Matcher>[
          matchesSemantics(
            attributedLabel: AttributedString(
              'root',
              attributes: <StringAttribute>[
                LocaleStringAttribute(
                  range: const TextRange(start: 0, end: 4),
                  locale: const Locale('es', 'MX'),
                ),
              ],
            ),
          ),
          matchesSemantics(
            attributedLabel: AttributedString(
              'one',
              attributes: <StringAttribute>[
                LocaleStringAttribute(
                  range: const TextRange(start: 0, end: 3),
                  locale: const Locale('es', 'MX'),
                ),
              ],
            ),
          ),
          matchesSemantics(
            attributedLabel: AttributedString(
              'three',
              attributes: <StringAttribute>[
                LocaleStringAttribute(
                  range: const TextRange(start: 0, end: 5),
                  locale: const Locale('es', 'MX'),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  });

  testWidgets('TextSpan spellOut works', (WidgetTester tester) async {
    final TapGestureRecognizer recognizer1 = TapGestureRecognizer();
    addTearDown(recognizer1.dispose);
    final DoubleTapGestureRecognizer recognizer2 = DoubleTapGestureRecognizer();
    addTearDown(recognizer2.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: RichText(
          text: TextSpan(
            text: 'root',
            spellOut: true,
            children: <InlineSpan>[
              TextSpan(text: 'one', recognizer: recognizer1),
              const WidgetSpan(child: SizedBox()),
              TextSpan(text: 'three', recognizer: recognizer2),
            ],
          ),
        ),
      ),
    );
    expect(
      tester.getSemantics(find.byType(RichText)),
      matchesSemantics(
        children: <Matcher>[
          matchesSemantics(
            attributedLabel: AttributedString(
              'root',
              attributes: <StringAttribute>[
                SpellOutStringAttribute(range: const TextRange(start: 0, end: 4)),
              ],
            ),
          ),
          matchesSemantics(
            attributedLabel: AttributedString(
              'one',
              attributes: <StringAttribute>[
                SpellOutStringAttribute(range: const TextRange(start: 0, end: 3)),
              ],
            ),
          ),
          matchesSemantics(
            attributedLabel: AttributedString(
              'three',
              attributes: <StringAttribute>[
                SpellOutStringAttribute(range: const TextRange(start: 0, end: 5)),
              ],
            ),
          ),
        ],
      ),
    );
  });

  testWidgets('WidgetSpan calculate correct intrinsic heights', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: ColoredBox(
            color: Colors.green,
            child: IntrinsicHeight(
              child: RichText(
                text: const TextSpan(
                  children: <InlineSpan>[
                    TextSpan(text: 'Start\n', style: TextStyle(height: 1.0, fontSize: 16)),
                    WidgetSpan(child: Row(children: <Widget>[SizedBox(height: 16, width: 16)])),
                    TextSpan(text: 'End', style: TextStyle(height: 1.0, fontSize: 16)),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byType(IntrinsicHeight)).height, 3 * 16);
  });

  testWidgets('RichText implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    RichText(
      text: const TextSpan(text: 'rich text'),
      textAlign: TextAlign.center,
      textDirection: TextDirection.rtl,
      softWrap: false,
      overflow: TextOverflow.ellipsis,
      textScaleFactor: 1.3,
      maxLines: 1,
      locale: const Locale('zh', 'HK'),
      strutStyle: const StrutStyle(fontSize: 16),
      textWidthBasis: TextWidthBasis.longestLine,
      textHeightBehavior: const TextHeightBehavior(applyHeightToFirstAscent: false),
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(
      description,
      unorderedMatches(<Matcher>[
        contains('textAlign: center'),
        contains('textDirection: rtl'),
        contains('softWrap: no wrapping except at line break characters'),
        contains('overflow: ellipsis'),
        contains('textScaler: linear (1.3x)'),
        contains('maxLines: 1'),
        contains('textWidthBasis: longestLine'),
        contains('text: "rich text"'),
        contains('locale: zh_HK'),
        allOf(startsWith('strutStyle: StrutStyle('), contains('size: 16.0')),
        allOf(
          startsWith('textHeightBehavior: TextHeightBehavior('),
          contains('applyHeightToFirstAscent: false'),
          contains('applyHeightToLastDescent: true'),
        ),
      ]),
    );
  });
}
