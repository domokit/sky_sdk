// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Large Badge defaults', (WidgetTester tester) async {
    late final ThemeData theme;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light(useMaterial3: true),
        home: Align(
          alignment: Alignment.topLeft,
          child: Builder(
            builder: (BuildContext context) {
              // theme.textTheme is updated when the MaterialApp is built.
              theme = Theme.of(context);
              return const Badge(label: Text('0'), child: Icon(Icons.add));
            },
          ),
        ),
      ),
    );

    expect(
      tester.renderObject<RenderParagraph>(find.text('0')).text.style,
      theme.textTheme.labelSmall!.copyWith(color: theme.colorScheme.onError),
    );

    // default badge alignment = AlignmentDirection.topEnd
    // default offset for LTR = Offset(4, -4)
    // default padding = EdgeInsets.symmetric(horizontal: 4)
    // default largeSize = 16
    // '0'.width = 12
    // icon.width = 24

    expect(tester.getSize(find.byType(Badge)), const Size(24, 24)); // default Icon size
    expect(tester.getTopLeft(find.byType(Badge)), Offset.zero);

    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(tester.getTopLeft(find.text('0')), const Offset(16, -4));
    }

    final RenderBox box = tester.renderObject(find.byType(Badge));
    final RRect rrect = RRect.fromLTRBR(12, -4, 31.5, 12, const Radius.circular(8));
    expect(box, paints..rrect(rrect: rrect, color: theme.colorScheme.error));
  });

  testWidgets('Large Badge defaults with RTL', (WidgetTester tester) async {
    late final ThemeData theme;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light(useMaterial3: true),
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Align(
            alignment: Alignment.topLeft,
            child: Builder(
              builder: (BuildContext context) {
                // theme.textTheme is updated when the MaterialApp is built.
                theme = Theme.of(context);
                return const Badge(label: Text('0'), child: Icon(Icons.add));
              },
            ),
          ),
        ),
      ),
    );

    expect(
      tester.renderObject<RenderParagraph>(find.text('0')).text.style,
      theme.textTheme.labelSmall!.copyWith(color: theme.colorScheme.onError),
    );

    expect(tester.getSize(find.byType(Badge)), const Size(24, 24)); // default Icon size
    expect(tester.getTopLeft(find.byType(Badge)), Offset.zero);

    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(tester.getTopLeft(find.text('0')), const Offset(0, -4));
    }

    final RenderBox box = tester.renderObject(find.byType(Badge));
    final RRect rrect = RRect.fromLTRBR(-4, -4, 15.5, 12, const Radius.circular(8));
    expect(box, paints..rrect(rrect: rrect, color: theme.colorScheme.error));
  });

  // Essentially the same as 'Large Badge defaults'
  testWidgets('Badge.count', (WidgetTester tester) async {
    late final ThemeData theme;

    Widget buildFrame(int count) {
      return MaterialApp(
        theme: ThemeData.light(useMaterial3: true),
        home: Align(
          alignment: Alignment.topLeft,
          child: Builder(
            builder: (BuildContext context) {
              // theme.textTheme is updated when the MaterialApp is built.
              if (count == 0) {
                theme = Theme.of(context);
              }
              return Badge.count(count: count, child: const Icon(Icons.add));
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(0));

    expect(
      tester.renderObject<RenderParagraph>(find.text('0')).text.style,
      theme.textTheme.labelSmall!.copyWith(color: theme.colorScheme.onError),
    );

    // default badge alignment = AlignmentDirectional(12, -4)
    // default padding = EdgeInsets.symmetric(horizontal: 4)
    // default largeSize = 16
    // '0'.width = 12
    // icon.width = 24

    expect(tester.getSize(find.byType(Badge)), const Size(24, 24)); // default Icon size
    expect(tester.getTopLeft(find.byType(Badge)), Offset.zero);

    // x = alignment.start + padding.left
    // y = alignment.top
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(tester.getTopLeft(find.text('0')), const Offset(16, -4));
    }

    final RenderBox box = tester.renderObject(find.byType(Badge));
    // '0'.width = 12
    // L = alignment.start
    // T = alignment.top
    // R = L + '0'.width + padding.width
    // B = T + largeSize, R = largeSize/2
    final RRect rrect = RRect.fromLTRBR(12, -4, 31.5, 12, const Radius.circular(8));
    expect(box, paints..rrect(rrect: rrect, color: theme.colorScheme.error));

    await tester.pumpWidget(buildFrame(1000));
    expect(find.text('999+'), findsOneWidget);
  });

  testWidgets('Small Badge defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData.light(useMaterial3: true);

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Align(alignment: Alignment.topLeft, child: Badge(child: Icon(Icons.add))),
      ),
    );

    // default badge location is end=0, top=0
    // default padding = EdgeInsets.symmetric(horizontal: 4)
    // default smallSize = 6
    // icon.width = 24

    expect(tester.getSize(find.byType(Badge)), const Size(24, 24)); // default Icon size
    expect(tester.getTopLeft(find.byType(Badge)), Offset.zero);

    final RenderBox box = tester.renderObject(find.byType(Badge));
    // L = icon.size.width - smallSize
    // T = 0
    // R = icon.size.width
    // B = smallSize
    expect(
      box,
      paints..rrect(
        rrect: RRect.fromLTRBR(18, 0, 24, 6, const Radius.circular(3)),
        color: theme.colorScheme.error,
      ),
    );
  });

  testWidgets('Small Badge RTL defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData.light(useMaterial3: true);

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Directionality(
          textDirection: TextDirection.rtl,
          child: Align(alignment: Alignment.topLeft, child: Badge(child: Icon(Icons.add))),
        ),
      ),
    );

    // default badge location is end=0, top=0
    // default smallSize = 6
    // icon.width = 24

    expect(tester.getSize(find.byType(Badge)), const Size(24, 24)); // default Icon size
    expect(tester.getTopLeft(find.byType(Badge)), Offset.zero);

    final RenderBox box = tester.renderObject(find.byType(Badge));
    // L = 0
    // T = 0
    // R = smallSize
    // B = smallSize
    expect(
      box,
      paints..rrect(
        rrect: RRect.fromLTRBR(0, 0, 6, 6, const Radius.circular(3)),
        color: theme.colorScheme.error,
      ),
    );
  });

  testWidgets('Large Badge textStyle and colors', (WidgetTester tester) async {
    final ThemeData theme = ThemeData.light(useMaterial3: true);
    const Color green = Color(0xff00ff00);
    const Color black = Color(0xff000000);

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Align(
          alignment: Alignment.topLeft,
          child: Badge(
            textColor: green,
            backgroundColor: black,
            textStyle: TextStyle(fontSize: 10),
            label: Text('0'),
            child: Icon(Icons.add),
          ),
        ),
      ),
    );

    final TextStyle textStyle = tester.renderObject<RenderParagraph>(find.text('0')).text.style!;
    expect(textStyle.fontSize, 10);
    expect(textStyle.color, green);
    expect(tester.renderObject(find.byType(Badge)), paints..rrect(color: black));
  });

  testWidgets('isLabelVisible', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light(useMaterial3: true),
        home: const Align(
          alignment: Alignment.topLeft,
          child: Badge(label: Text('0'), isLabelVisible: false, child: Icon(Icons.add)),
        ),
      ),
    );

    expect(find.text('0'), findsNothing);
    expect(find.byType(Icon), findsOneWidget);

    expect(tester.getSize(find.byType(Badge)), const Size(24, 24)); // default Icon size
    expect(tester.getTopLeft(find.byType(Badge)), Offset.zero);
    final RenderBox box = tester.renderObject(find.byType(Badge));
    expect(box, isNot(paints..rrect()));
  });

  testWidgets('Large Badge alignment', (WidgetTester tester) async {
    const Radius badgeRadius = Radius.circular(8);

    Widget buildFrame(Alignment alignment, [Offset offset = Offset.zero]) {
      return MaterialApp(
        theme: ThemeData.light(useMaterial3: true),
        home: Align(
          alignment: Alignment.topLeft,
          child: Badge(
            // Default largeSize = 16, badge with label is "large".
            label: Container(width: 8, height: 8, color: Colors.blue),
            alignment: alignment,
            offset: offset,
            child: Container(color: const Color(0xFF00FF00), width: 200, height: 200),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(Alignment.topLeft));
    final RenderBox box = tester.renderObject(find.byType(Badge));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(0, 0, 16, 16, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.topCenter));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(100 - 8, 0, 100 + 8, 16, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.topRight));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(200 - 16, 0, 200, 16, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.centerLeft));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(0, 100, 16, 100 + 16, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.centerRight));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(200 - 16, 100, 200, 100 + 16, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.bottomLeft));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(0, 200, 16, 200 + 16, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.bottomCenter));
    expect(
      box,
      paints..rrect(rrect: RRect.fromLTRBR(100 - 8, 200, 100 + 8, 200 + 16, badgeRadius)),
    );

    await tester.pumpWidget(buildFrame(Alignment.bottomRight));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(200 - 16, 200, 200, 200 + 16, badgeRadius)));

    const Offset offset = Offset(5, 10);

    await tester.pumpWidget(buildFrame(Alignment.topLeft, offset));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(0, 0, 16, 16, badgeRadius).shift(offset)));

    await tester.pumpWidget(buildFrame(Alignment.topCenter, offset));
    expect(
      box,
      paints..rrect(rrect: RRect.fromLTRBR(100 - 8, 0, 100 + 8, 16, badgeRadius).shift(offset)),
    );

    await tester.pumpWidget(buildFrame(Alignment.topRight, offset));
    expect(
      box,
      paints..rrect(rrect: RRect.fromLTRBR(200 - 16, 0, 200, 16, badgeRadius).shift(offset)),
    );

    await tester.pumpWidget(buildFrame(Alignment.centerLeft, offset));
    expect(
      box,
      paints..rrect(rrect: RRect.fromLTRBR(0, 100, 16, 100 + 16, badgeRadius).shift(offset)),
    );

    await tester.pumpWidget(buildFrame(Alignment.centerRight, offset));
    expect(
      box,
      paints
        ..rrect(rrect: RRect.fromLTRBR(200 - 16, 100, 200, 100 + 16, badgeRadius).shift(offset)),
    );

    await tester.pumpWidget(buildFrame(Alignment.bottomLeft, offset));
    expect(
      box,
      paints..rrect(rrect: RRect.fromLTRBR(0, 200, 16, 200 + 16, badgeRadius).shift(offset)),
    );

    await tester.pumpWidget(buildFrame(Alignment.bottomCenter, offset));
    expect(
      box,
      paints
        ..rrect(rrect: RRect.fromLTRBR(100 - 8, 200, 100 + 8, 200 + 16, badgeRadius).shift(offset)),
    );

    await tester.pumpWidget(buildFrame(Alignment.bottomRight, offset));
    expect(
      box,
      paints
        ..rrect(rrect: RRect.fromLTRBR(200 - 16, 200, 200, 200 + 16, badgeRadius).shift(offset)),
    );
  });

  testWidgets('Small Badge alignment', (WidgetTester tester) async {
    const Radius badgeRadius = Radius.circular(3);

    Widget buildFrame(Alignment alignment, [Offset offset = Offset.zero]) {
      return MaterialApp(
        theme: ThemeData.light(useMaterial3: true),
        home: Align(
          alignment: Alignment.topLeft,
          child: Badge(
            // Default smallSize = 6, badge without label is "small".
            alignment: alignment,
            offset: offset, // Not used for smallSize badges.
            child: Container(color: const Color(0xFF00FF00), width: 200, height: 200),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(Alignment.topLeft));
    final RenderBox box = tester.renderObject(find.byType(Badge));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(0, 0, 6, 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.topCenter));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(100 - 3, 0, 100 + 3, 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.topRight));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(200 - 6, 0, 200, 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.centerLeft));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(0, 100, 6, 100 + 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.centerRight));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(200 - 6, 100, 200, 100 + 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.bottomLeft));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(0, 200, 6, 200 + 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.bottomCenter));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(100 - 3, 200, 100 + 3, 200 + 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.bottomRight));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(200 - 6, 200, 200, 200 + 6, badgeRadius)));

    const Offset offset = Offset(5, 10); // Not used for smallSize Badges.

    await tester.pumpWidget(buildFrame(Alignment.topLeft, offset));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(0, 0, 6, 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.topCenter, offset));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(100 - 3, 0, 100 + 3, 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.topRight, offset));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(200 - 6, 0, 200, 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.centerLeft, offset));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(0, 100, 6, 100 + 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.centerRight, offset));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(200 - 6, 100, 200, 100 + 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.bottomLeft, offset));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(0, 200, 6, 200 + 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.bottomCenter, offset));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(100 - 3, 200, 100 + 3, 200 + 6, badgeRadius)));

    await tester.pumpWidget(buildFrame(Alignment.bottomRight, offset));
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(200 - 6, 200, 200, 200 + 6, badgeRadius)));
  });

  testWidgets('Badge Larger than large size', (WidgetTester tester) async {
    const Radius badgeRadius = Radius.circular(15);

    Widget buildFrame(Alignment alignment, [Offset offset = Offset.zero]) {
      return MaterialApp(
        theme: ThemeData.light(useMaterial3: true),
        home: Align(
          alignment: Alignment.topLeft,
          child: Badge(
            // LargeSize = 16, make content of badge bigger than the default.
            label: Container(width: 30, height: 30, color: Colors.blue),
            alignment: alignment,
            offset: offset,
            child: Container(color: const Color(0xFF00FF00), width: 200, height: 200),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(Alignment.topLeft));
    final RenderBox box = tester.renderObject(find.byType(Badge));
    // Badge should scale with content
    expect(box, paints..rrect(rrect: RRect.fromLTRBR(0, -7, 30 + 8, 23, badgeRadius)));
  });
}
