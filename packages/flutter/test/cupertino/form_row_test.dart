// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows prefix', (WidgetTester tester) async {
    const Widget prefix = Text('Enter Value');

    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoFormRow(prefix: prefix, child: CupertinoTextField())),
      ),
    );

    expect(prefix, tester.widget(find.byType(Text)));
  });

  testWidgets('Shows child', (WidgetTester tester) async {
    const Widget child = CupertinoTextField();

    await tester.pumpWidget(
      const CupertinoApp(home: Center(child: CupertinoFormRow(child: child))),
    );

    expect(child, tester.widget(find.byType(CupertinoTextField)));
  });

  testWidgets('RTL puts prefix after child', (WidgetTester tester) async {
    const Widget prefix = Text('Enter Value');
    const Widget child = CupertinoTextField();

    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
          child: Directionality(
            textDirection: TextDirection.rtl,
            child: CupertinoFormRow(prefix: prefix, child: child),
          ),
        ),
      ),
    );

    expect(
      tester.getTopLeft(find.byType(Text)).dx >
          tester.getTopLeft(find.byType(CupertinoTextField)).dx,
      true,
    );
  });

  testWidgets('LTR puts child after prefix', (WidgetTester tester) async {
    const Widget prefix = Text('Enter Value');
    const Widget child = CupertinoTextField();

    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: CupertinoFormRow(prefix: prefix, child: child),
          ),
        ),
      ),
    );

    expect(
      tester.getTopLeft(find.byType(Text)).dx >
          tester.getTopLeft(find.byType(CupertinoTextField)).dx,
      false,
    );
  });

  testWidgets('Shows error widget', (WidgetTester tester) async {
    const Widget error = Text('Error');

    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoFormRow(error: error, child: CupertinoTextField())),
      ),
    );

    expect(error, tester.widget(find.byType(Text)));
  });

  testWidgets('Shows helper widget', (WidgetTester tester) async {
    const Widget helper = Text('Helper');

    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoFormRow(helper: helper, child: CupertinoTextField())),
      ),
    );

    expect(helper, tester.widget(find.byType(Text)));
  });

  testWidgets('Shows helper text above error text', (WidgetTester tester) async {
    const Widget helper = Text('Helper');
    const Widget error = CupertinoActivityIndicator();

    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
          child: CupertinoFormRow(helper: helper, error: error, child: CupertinoTextField()),
        ),
      ),
    );

    expect(
      tester.getTopLeft(find.byType(CupertinoActivityIndicator)).dy >
          tester.getTopLeft(find.byType(Text)).dy,
      true,
    );
  });

  testWidgets('Shows helper in label color and error text in red color', (
    WidgetTester tester,
  ) async {
    const Widget helper = Text('Helper');
    const Widget error = Text('Error');

    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
          child: CupertinoFormRow(helper: helper, error: error, child: CupertinoTextField()),
        ),
      ),
    );

    final DefaultTextStyle helperTextStyle = tester.widget(find.byType(DefaultTextStyle).first);

    expect(helperTextStyle.style.color, CupertinoColors.label);

    final DefaultTextStyle errorTextStyle = tester.widget(find.byType(DefaultTextStyle).last);

    expect(errorTextStyle.style.color, CupertinoColors.destructiveRed);
  });

  testWidgets('CupertinoFormRow adapts to MaterialApp dark mode', (WidgetTester tester) async {
    const Widget prefix = Text('Prefix');
    const Widget helper = Text('Helper');

    Widget buildFormRow(Brightness brightness) {
      return MaterialApp(
        theme: ThemeData(brightness: brightness),
        home: const Center(
          child: CupertinoFormRow(prefix: prefix, helper: helper, child: CupertinoTextField()),
        ),
      );
    }

    // CupertinoFormRow with light theme.
    await tester.pumpWidget(buildFormRow(Brightness.light));
    RenderParagraph helperParagraph = tester.renderObject(find.text('Helper'));
    expect(helperParagraph.text.style!.color, CupertinoColors.label);
    // Text style should not return unresolved color.
    expect(helperParagraph.text.style!.color.toString().contains('UNRESOLVED'), isFalse);
    RenderParagraph prefixParagraph = tester.renderObject(find.text('Prefix'));
    expect(prefixParagraph.text.style!.color, CupertinoColors.label);
    // Text style should not return unresolved color.
    expect(prefixParagraph.text.style!.color.toString().contains('UNRESOLVED'), isFalse);

    // CupertinoFormRow with light theme.
    await tester.pumpWidget(buildFormRow(Brightness.dark));
    helperParagraph = tester.renderObject(find.text('Helper'));
    expect(helperParagraph.text.style!.color, CupertinoColors.label);
    // Text style should not return unresolved color.
    expect(helperParagraph.text.style!.color.toString().contains('UNRESOLVED'), isFalse);
    prefixParagraph = tester.renderObject(find.text('Prefix'));
    expect(prefixParagraph.text.style!.color, CupertinoColors.label);
    // Text style should not return unresolved color.
    expect(prefixParagraph.text.style!.color.toString().contains('UNRESOLVED'), isFalse);
  });
}
