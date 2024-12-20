// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('has correct backdrop filters', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDesktopTextSelectionToolbar(
            anchor: Offset.zero,
            children: <Widget>[
              CupertinoDesktopTextSelectionToolbarButton(
                child: const Text('Tap me'),
                onPressed: () {},
              ),
            ],
          ),
        ),
      ),
    );

    final BackdropFilter toolbarFilter = tester.firstWidget<BackdropFilter>(
      find.descendant(
        of: find.byType(CupertinoDesktopTextSelectionToolbar),
        matching: find.byType(BackdropFilter),
      ),
    );

    expect(
      toolbarFilter.filter.runtimeType,
      // _ComposeImageFilter is internal so we can't test if its filters are
      // for blur and saturation, but checking if it's a _ComposeImageFilter
      // should be enough. Outer and inner parameters don't matter, we just need
      // a new _ComposeImageFilter to get its runtimeType.
      //
      // As web doesn't support ImageFilter.compose, we use just blur when
      // kIsWeb.
      kIsWeb
          ? ImageFilter.blur().runtimeType
          : ImageFilter.compose(outer: ImageFilter.blur(), inner: ImageFilter.blur()).runtimeType,
    );
  });

  testWidgets('has shadow', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDesktopTextSelectionToolbar(
            anchor: Offset.zero,
            children: <Widget>[
              CupertinoDesktopTextSelectionToolbarButton(
                child: const Text('Tap me'),
                onPressed: () {},
              ),
            ],
          ),
        ),
      ),
    );

    final DecoratedBox decoratedBox = tester.firstWidget<DecoratedBox>(
      find.descendant(
        of: find.byType(CupertinoDesktopTextSelectionToolbar),
        matching: find.byType(DecoratedBox),
      ),
    );

    expect((decoratedBox.decoration as BoxDecoration).boxShadow, isNotNull);
  });

  testWidgets('is translucent', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDesktopTextSelectionToolbar(
            anchor: Offset.zero,
            children: <Widget>[
              CupertinoDesktopTextSelectionToolbarButton(
                child: const Text('Tap me'),
                onPressed: () {},
              ),
            ],
          ),
        ),
      ),
    );

    final DecoratedBox decoratedBox = tester
        .widgetList<DecoratedBox>(
          find.descendant(
            of: find.byType(CupertinoDesktopTextSelectionToolbar),
            matching: find.byType(DecoratedBox),
          ),
        )
        // The second DecoratedBox should be the one with color.
        .elementAt(1);

    expect((decoratedBox.decoration as BoxDecoration).color!.opacity, lessThan(1.0));
  });

  testWidgets('positions itself at the anchor', (WidgetTester tester) async {
    // An arbitrary point on the screen to position at.
    const Offset anchor = Offset(30.0, 40.0);

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDesktopTextSelectionToolbar(
            anchor: anchor,
            children: <Widget>[
              CupertinoDesktopTextSelectionToolbarButton(
                child: const Text('Tap me'),
                onPressed: () {},
              ),
            ],
          ),
        ),
      ),
    );

    expect(
      tester.getTopLeft(find.byType(CupertinoDesktopTextSelectionToolbarButton)),
      // Greater than due to padding internal to the toolbar.
      greaterThan(anchor),
    );
  });
}
