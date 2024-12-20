// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SliverFillViewport control test', (WidgetTester tester) async {
    final List<Widget> children = List<Widget>.generate(20, (int i) {
      return ColoredBox(color: Colors.green, child: Text('$i', textDirection: TextDirection.ltr));
    });

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            SliverFillViewport(
              delegate: SliverChildListDelegate(
                children,
                addAutomaticKeepAlives: false,
                addSemanticIndexes: false,
              ),
            ),
          ],
        ),
      ),
    );

    final RenderBox box = tester.renderObject<RenderBox>(find.byType(ColoredBox).first);
    expect(box.size.height, equals(600.0));

    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);

    await tester.drag(find.byType(Scrollable), const Offset(0.0, -700.0));
    await tester.pump();

    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsNothing);
    expect(find.text('4'), findsNothing);

    await tester.drag(find.byType(Scrollable), const Offset(0.0, 200.0));
    await tester.pump();

    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsNothing);

    await tester.drag(find.byType(Scrollable), const Offset(0.0, 700.0));
    await tester.pump();

    final RenderBox box2 = tester.renderObject<RenderBox>(find.byType(ColoredBox).first);
    expect(box2.size.height, equals(600.0));

    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsNothing);
    expect(find.text('2'), findsNothing);
    expect(find.text('3'), findsNothing);

    final RenderObject viewport = tester.renderObject<RenderObject>(
      find.byType(SliverFillViewport).first,
    );
    expect(viewport, hasAGoodToStringDeep);
    expect(
      viewport.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        '_RenderSliverFractionalPadding#00000 relayoutBoundary=up1\n'
        ' │ needs compositing\n'
        ' │ parentData: paintOffset=Offset(0.0, 0.0) (can use size)\n'
        ' │ constraints: SliverConstraints(AxisDirection.down,\n'
        ' │   GrowthDirection.forward, ScrollDirection.idle, scrollOffset:\n'
        ' │   0.0, precedingScrollExtent: 0.0, remainingPaintExtent: 600.0,\n'
        ' │   crossAxisExtent: 800.0, crossAxisDirection:\n'
        ' │   AxisDirection.right, viewportMainAxisExtent: 600.0,\n'
        ' │   remainingCacheExtent: 850.0, cacheOrigin: 0.0)\n'
        ' │ geometry: SliverGeometry(scrollExtent: 12000.0, paintExtent:\n'
        ' │   600.0, maxPaintExtent: 12000.0, hasVisualOverflow: true,\n'
        ' │   cacheExtent: 850.0)\n'
        ' │\n'
        ' └─child: RenderSliverFillViewport#00000 relayoutBoundary=up2\n'
        '   │ needs compositing\n'
        '   │ parentData: paintOffset=Offset(0.0, 0.0) (can use size)\n'
        '   │ constraints: SliverConstraints(AxisDirection.down,\n'
        '   │   GrowthDirection.forward, ScrollDirection.idle, scrollOffset:\n'
        '   │   0.0, precedingScrollExtent: 0.0, remainingPaintExtent: 600.0,\n'
        '   │   crossAxisExtent: 800.0, crossAxisDirection:\n'
        '   │   AxisDirection.right, viewportMainAxisExtent: 600.0,\n'
        '   │   remainingCacheExtent: 850.0, cacheOrigin: 0.0)\n'
        '   │ geometry: SliverGeometry(scrollExtent: 12000.0, paintExtent:\n'
        '   │   600.0, maxPaintExtent: 12000.0, hasVisualOverflow: true,\n'
        '   │   cacheExtent: 850.0)\n'
        '   │ currently live children: 0 to 1\n'
        '   │\n'
        '   ├─child with index 0: RenderRepaintBoundary#00000\n'
        '   │ │ needs compositing\n'
        '   │ │ parentData: index=0; layoutOffset=0.0\n'
        '   │ │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '   │ │ layer: OffsetLayer#00000\n'
        '   │ │ size: Size(800.0, 600.0)\n'
        '   │ │ metrics: 66.7% useful (1 bad vs 2 good)\n'
        '   │ │ diagnosis: insufficient data to draw conclusion (less than five\n'
        '   │ │   repaints)\n'
        '   │ │\n'
        '   │ └─child: _RenderColoredBox#00000\n'
        '   │   │ parentData: <none> (can use size)\n'
        '   │   │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '   │   │ size: Size(800.0, 600.0)\n'
        '   │   │ behavior: opaque\n'
        '   │   │\n'
        '   │   └─child: RenderParagraph#00000\n'
        '   │     │ parentData: <none> (can use size)\n'
        '   │     │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '   │     │ semantics node: SemanticsNode#2\n'
        '   │     │ size: Size(800.0, 600.0)\n'
        '   │     │ textAlign: start\n'
        '   │     │ textDirection: ltr\n'
        '   │     │ softWrap: wrapping at box width\n'
        '   │     │ overflow: clip\n'
        '   │     │ maxLines: unlimited\n'
        '   │     ╘═╦══ text ═══\n'
        '   │       ║ TextSpan:\n'
        '   │       ║   <all styles inherited>\n'
        '   │       ║   "0"\n'
        '   │       ╚═══════════\n'
        '   └─child with index 1: RenderRepaintBoundary#00000\n'
        '     │ needs compositing\n'
        '     │ parentData: index=1; layoutOffset=600.0\n'
        '     │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '     │ layer: OffsetLayer#00000 DETACHED\n'
        '     │ size: Size(800.0, 600.0)\n'
        '     │ metrics: 50.0% useful (1 bad vs 1 good)\n'
        '     │ diagnosis: insufficient data to draw conclusion (less than five\n'
        '     │   repaints)\n'
        '     │\n'
        '     └─child: _RenderColoredBox#00000\n'
        '       │ parentData: <none> (can use size)\n'
        '       │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '       │ size: Size(800.0, 600.0)\n'
        '       │ behavior: opaque\n'
        '       │\n'
        '       └─child: RenderParagraph#00000\n'
        '         │ parentData: <none> (can use size)\n'
        '         │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '         │ semantics node: SemanticsNode#3\n'
        '         │ size: Size(800.0, 600.0)\n'
        '         │ textAlign: start\n'
        '         │ textDirection: ltr\n'
        '         │ softWrap: wrapping at box width\n'
        '         │ overflow: clip\n'
        '         │ maxLines: unlimited\n'
        '         ╘═╦══ text ═══\n'
        '           ║ TextSpan:\n'
        '           ║   <all styles inherited>\n'
        '           ║   "1"\n'
        '           ╚═══════════\n',
      ),
    );
  });

  testWidgets('SliverFillViewport padding test', (WidgetTester tester) async {
    final SliverChildListDelegate delegate = SliverChildListDelegate(
      <Widget>[const Text('0')],
      addAutomaticKeepAlives: false,
      addSemanticIndexes: false,
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[SliverFillViewport(viewportFraction: 0.5, delegate: delegate)],
        ),
      ),
    );

    final RenderSliver boxWithPadding = tester.renderObject<RenderSliver>(
      find.byType(SliverFillViewport),
    );
    expect(boxWithPadding.geometry!.paintExtent, equals(600.0));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            SliverFillViewport(padEnds: false, viewportFraction: 0.5, delegate: delegate),
          ],
        ),
      ),
    );

    final RenderSliver boxWithoutPadding = tester.renderObject<RenderSliver>(
      find.byType(SliverFillViewport),
    );
    expect(boxWithoutPadding.geometry!.paintExtent, equals(300.0));
  });
}
