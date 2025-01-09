// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Text widget parameter takes precedence over DefaultTextHeightBehavior', (
    WidgetTester tester,
  ) async {
    const TextHeightBehavior behavior1 = TextHeightBehavior(
      applyHeightToLastDescent: false,
      applyHeightToFirstAscent: false,
    );
    const TextHeightBehavior behavior2 = TextHeightBehavior(applyHeightToFirstAscent: false);

    await tester.pumpWidget(
      const DefaultTextHeightBehavior(
        textHeightBehavior: behavior2,
        child: Text('Hello', textDirection: TextDirection.ltr, textHeightBehavior: behavior1),
      ),
    );

    final RichText text = tester.firstWidget(find.byType(RichText));
    expect(text, isNotNull);
    expect(text.textHeightBehavior, behavior1);
  });

  testWidgets(
    'DefaultTextStyle.textHeightBehavior takes precedence over DefaultTextHeightBehavior ',
    (WidgetTester tester) async {
      const TextHeightBehavior behavior1 = TextHeightBehavior(
        applyHeightToLastDescent: false,
        applyHeightToFirstAscent: false,
      );
      const TextHeightBehavior behavior2 = TextHeightBehavior(applyHeightToFirstAscent: false);

      await tester.pumpWidget(
        const DefaultTextStyle(
          style: TextStyle(),
          textHeightBehavior: behavior1,
          child: DefaultTextHeightBehavior(
            textHeightBehavior: behavior2,
            child: Text('Hello', textDirection: TextDirection.ltr),
          ),
        ),
      );

      RichText text = tester.firstWidget(find.byType(RichText));
      expect(text, isNotNull);
      expect(text.textHeightBehavior, behavior1);

      await tester.pumpWidget(
        const DefaultTextHeightBehavior(
          textHeightBehavior: behavior2,
          child: DefaultTextStyle(
            style: TextStyle(),
            textHeightBehavior: behavior1,
            child: Text('Hello', textDirection: TextDirection.ltr),
          ),
        ),
      );

      text = tester.firstWidget(find.byType(RichText));
      expect(text, isNotNull);
      expect(text.textHeightBehavior, behavior1);
    },
  );

  testWidgets('DefaultTextHeightBehavior changes propagate to Text', (WidgetTester tester) async {
    const Text textWidget = Text('Hello', textDirection: TextDirection.ltr);
    const TextHeightBehavior behavior1 = TextHeightBehavior(
      applyHeightToLastDescent: false,
      applyHeightToFirstAscent: false,
    );
    const TextHeightBehavior behavior2 = TextHeightBehavior(
      applyHeightToLastDescent: false,
      applyHeightToFirstAscent: false,
    );

    await tester.pumpWidget(
      const DefaultTextHeightBehavior(textHeightBehavior: behavior1, child: textWidget),
    );

    RichText text = tester.firstWidget(find.byType(RichText));
    expect(text, isNotNull);
    expect(text.textHeightBehavior, behavior1);

    await tester.pumpWidget(
      const DefaultTextHeightBehavior(textHeightBehavior: behavior2, child: textWidget),
    );

    text = tester.firstWidget(find.byType(RichText));
    expect(text, isNotNull);
    expect(text.textHeightBehavior, behavior2);
  });

  testWidgets('DefaultTextHeightBehavior.of(context) returns null if no '
      'DefaultTextHeightBehavior widget in tree', (WidgetTester tester) async {
    const Text textWidget = Text('Hello', textDirection: TextDirection.ltr);
    TextHeightBehavior? textHeightBehavior;

    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          textHeightBehavior = DefaultTextHeightBehavior.maybeOf(context);
          return textWidget;
        },
      ),
    );

    expect(textHeightBehavior, isNull);
    final RichText text = tester.firstWidget(find.byType(RichText));
    expect(text, isNotNull);
    expect(text.textHeightBehavior, isNull);
  });
}
