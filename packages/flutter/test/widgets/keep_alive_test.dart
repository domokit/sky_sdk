// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' show Platform;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class Leaf extends StatefulWidget {
  const Leaf({super.key, required this.child});
  final Widget child;
  @override
  State<Leaf> createState() => _LeafState();
}

class _LeafState extends State<Leaf> {
  bool _keepAlive = false;

  void setKeepAlive(bool value) {
    setState(() {
      _keepAlive = value;
    });
  }

  @override
  Widget build(BuildContext context) {
    return KeepAlive(keepAlive: _keepAlive, child: widget.child);
  }
}

List<Widget> generateList(Widget child) {
  return List<Widget>.generate(
    100,
    (int index) => Leaf(key: GlobalObjectKey<_LeafState>(index), child: child),
    growable: false,
  );
}

void main() {
  test('KeepAlive debugTypicalAncestorWidgetClass', () {
    final KeepAlive keepAlive = KeepAlive(keepAlive: false, child: Container());
    expect(
      keepAlive.debugTypicalAncestorWidgetDescription,
      'SliverWithKeepAliveWidget or TwoDimensionalViewport',
    );
  });

  testWidgets('KeepAlive with ListView with itemExtent', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          cacheExtent: 0.0,
          addAutomaticKeepAlives: false,
          addRepaintBoundaries: false,
          addSemanticIndexes: false,
          itemExtent: 12.3, // about 50 widgets visible
          children: generateList(const Placeholder()),
        ),
      ),
    );
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    await tester.drag(find.byType(ListView), const Offset(0.0, -300.0)); // about 25 widgets' worth
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(60).currentState!.setKeepAlive(true);
    await tester.drag(find.byType(ListView), const Offset(0.0, 300.0)); // back to top
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(60).currentState!.setKeepAlive(false);
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
  });

  testWidgets('KeepAlive with ListView without itemExtent', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          cacheExtent: 0.0,
          addAutomaticKeepAlives: false,
          addRepaintBoundaries: false,
          addSemanticIndexes: false,
          children: generateList(
            const SizedBox(height: 12.3, child: Placeholder()),
          ), // about 50 widgets visible
        ),
      ),
    );
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    await tester.drag(find.byType(ListView), const Offset(0.0, -300.0)); // about 25 widgets' worth
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(60).currentState!.setKeepAlive(true);
    await tester.drag(find.byType(ListView), const Offset(0.0, 300.0)); // back to top
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(60).currentState!.setKeepAlive(false);
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
  });

  testWidgets('KeepAlive with GridView', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.count(
          cacheExtent: 0.0,
          addAutomaticKeepAlives: false,
          addRepaintBoundaries: false,
          addSemanticIndexes: false,
          crossAxisCount: 2,
          childAspectRatio: 400.0 / 24.6, // about 50 widgets visible
          children: generateList(const Placeholder()),
        ),
      ),
    );
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    await tester.drag(find.byType(GridView), const Offset(0.0, -300.0)); // about 25 widgets' worth
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(60).currentState!.setKeepAlive(true);
    await tester.drag(find.byType(GridView), const Offset(0.0, 300.0)); // back to top
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60)), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
    const GlobalObjectKey<_LeafState>(60).currentState!.setKeepAlive(false);
    await tester.pump();
    expect(find.byKey(const GlobalObjectKey<_LeafState>(3)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(30)), findsOneWidget);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(59), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(60), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(61), skipOffstage: false), findsNothing);
    expect(find.byKey(const GlobalObjectKey<_LeafState>(90), skipOffstage: false), findsNothing);
  });

  testWidgets('KeepAlive render tree description', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          addAutomaticKeepAlives: false,
          addRepaintBoundaries: false,
          addSemanticIndexes: false,
          itemExtent: 400.0, // 2 visible children
          children: generateList(const Placeholder()),
        ),
      ),
    );
    // The important lines below are the ones marked with "<----"
    expect(
      tester.binding.renderView.toStringDeep(minLevel: DiagnosticLevel.info, wrapWidth: 600),
      equalsIgnoringHashCodes(
        '_ReusableRenderView#00000\n'
        ' │ debug mode enabled - ${Platform.operatingSystem}\n'
        ' │ view size: Size(2400.0, 1800.0) (in physical pixels)\n'
        ' │ device pixel ratio: 3.0 (physical pixels per logical pixel)\n'
        ' │ configuration: BoxConstraints(w=800.0, h=600.0) at 3.0x (in logical pixels)\n'
        ' │\n'
        ' └─child: RenderRepaintBoundary#00000\n'
        '   │ needs compositing\n'
        '   │ parentData: <none>\n'
        '   │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '   │ layer: OffsetLayer#00000\n'
        '   │ size: Size(800.0, 600.0)\n'
        '   │ metrics: 0.0% useful (1 bad vs 0 good)\n'
        '   │ diagnosis: insufficient data to draw conclusion (less than five repaints)\n'
        '   │\n'
        '   └─child: RenderCustomPaint#00000\n'
        '     │ needs compositing\n'
        '     │ parentData: <none> (can use size)\n'
        '     │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '     │ size: Size(800.0, 600.0)\n'
        '     │ painter: null\n'
        '     │ foregroundPainter: _GlowingOverscrollIndicatorPainter(_GlowController(color: ${const Color(0xffffffff)}, axis: vertical), _GlowController(color: ${const Color(0xffffffff)}, axis: vertical))\n'
        '     │\n'
        '     └─child: RenderRepaintBoundary#00000\n'
        '       │ needs compositing\n'
        '       │ parentData: <none> (can use size)\n'
        '       │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '       │ layer: OffsetLayer#00000\n'
        '       │ size: Size(800.0, 600.0)\n'
        '       │ metrics: 0.0% useful (1 bad vs 0 good)\n'
        '       │ diagnosis: insufficient data to draw conclusion (less than five repaints)\n'
        '       │\n'
        '       └─child: _RenderScrollSemantics#00000\n'
        '         │ needs compositing\n'
        '         │ parentData: <none> (can use size)\n'
        '         │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '         │ semantics node: SemanticsNode#1\n'
        '         │ semantic boundary\n'
        '         │ size: Size(800.0, 600.0)\n'
        '         │\n'
        '         └─child: RenderPointerListener#00000\n'
        '           │ needs compositing\n'
        '           │ parentData: <none> (can use size)\n'
        '           │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '           │ size: Size(800.0, 600.0)\n'
        '           │ behavior: deferToChild\n'
        '           │ listeners: signal\n'
        '           │\n'
        '           └─child: RenderSemanticsGestureHandler#00000\n'
        '             │ needs compositing\n'
        '             │ parentData: <none> (can use size)\n'
        '             │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '             │ size: Size(800.0, 600.0)\n'
        '             │ behavior: opaque\n'
        '             │ gestures: vertical scroll\n'
        '             │\n'
        '             └─child: RenderPointerListener#00000\n'
        '               │ needs compositing\n'
        '               │ parentData: <none> (can use size)\n'
        '               │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '               │ size: Size(800.0, 600.0)\n'
        '               │ behavior: opaque\n'
        '               │ listeners: down, panZoomStart\n'
        '               │\n'
        '               └─child: RenderSemanticsAnnotations#00000\n'
        '                 │ needs compositing\n'
        '                 │ parentData: <none> (can use size)\n'
        '                 │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '                 │ size: Size(800.0, 600.0)\n'
        '                 │\n'
        '                 └─child: RenderIgnorePointer#00000\n'
        '                   │ needs compositing\n'
        '                   │ parentData: <none> (can use size)\n'
        '                   │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '                   │ size: Size(800.0, 600.0)\n'
        '                   │ ignoring: false\n'
        '                   │ ignoringSemantics: null\n'
        '                   │\n'
        '                   └─child: RenderViewport#00000\n'
        '                     │ needs compositing\n'
        '                     │ parentData: <none> (can use size)\n'
        '                     │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '                     │ layer: OffsetLayer#00000\n'
        '                     │ size: Size(800.0, 600.0)\n'
        '                     │ axisDirection: down\n'
        '                     │ crossAxisDirection: right\n'
        '                     │ offset: ScrollPositionWithSingleContext#00000(offset: 0.0, range: 0.0..39400.0, viewport: 600.0, ScrollableState, AlwaysScrollableScrollPhysics -> ClampingScrollPhysics -> RangeMaintainingScrollPhysics, IdleScrollActivity#00000, ScrollDirection.idle)\n'
        '                     │ anchor: 0.0\n'
        '                     │\n'
        '                     └─center child: RenderSliverPadding#00000 relayoutBoundary=up1\n'
        '                       │ parentData: paintOffset=Offset(0.0, 0.0) (can use size)\n'
        '                       │ constraints: SliverConstraints(AxisDirection.down, GrowthDirection.forward, ScrollDirection.idle, scrollOffset: 0.0, precedingScrollExtent: 0.0, remainingPaintExtent: 600.0, crossAxisExtent: 800.0, crossAxisDirection: AxisDirection.right, viewportMainAxisExtent: 600.0, remainingCacheExtent: 850.0, cacheOrigin: 0.0)\n'
        '                       │ geometry: SliverGeometry(scrollExtent: 40000.0, paintExtent: 600.0, maxPaintExtent: 40000.0, hasVisualOverflow: true, cacheExtent: 850.0)\n'
        '                       │ padding: EdgeInsets.zero\n'
        '                       │ textDirection: ltr\n'
        '                       │\n'
        '                       └─child: RenderSliverFixedExtentList#00000 relayoutBoundary=up2\n'
        '                         │ parentData: paintOffset=Offset(0.0, 0.0) (can use size)\n'
        '                         │ constraints: SliverConstraints(AxisDirection.down, GrowthDirection.forward, ScrollDirection.idle, scrollOffset: 0.0, precedingScrollExtent: 0.0, remainingPaintExtent: 600.0, crossAxisExtent: 800.0, crossAxisDirection: AxisDirection.right, viewportMainAxisExtent: 600.0, remainingCacheExtent: 850.0, cacheOrigin: 0.0)\n'
        '                         │ geometry: SliverGeometry(scrollExtent: 40000.0, paintExtent: 600.0, maxPaintExtent: 40000.0, hasVisualOverflow: true, cacheExtent: 850.0)\n'
        '                         │ currently live children: 0 to 2\n'
        '                         │\n'
        '                         ├─child with index 0: RenderLimitedBox#00000\n'
        '                         │ │ parentData: index=0; layoutOffset=0.0\n'
        '                         │ │ constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         │ │ size: Size(800.0, 400.0)\n'
        '                         │ │ maxWidth: 400.0\n'
        '                         │ │ maxHeight: 400.0\n'
        '                         │ │\n'
        '                         │ └─child: RenderCustomPaint#00000\n'
        '                         │     parentData: <none> (can use size)\n'
        '                         │     constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         │     size: Size(800.0, 400.0)\n'
        '                         │     painter: _PlaceholderPainter#00000()\n'
        '                         │     preferredSize: Size(Infinity, Infinity)\n'
        '                         │\n'
        '                         ├─child with index 1: RenderLimitedBox#00000\n' // <----- no dashed line starts here
        '                         │ │ parentData: index=1; layoutOffset=400.0\n'
        '                         │ │ constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         │ │ size: Size(800.0, 400.0)\n'
        '                         │ │ maxWidth: 400.0\n'
        '                         │ │ maxHeight: 400.0\n'
        '                         │ │\n'
        '                         │ └─child: RenderCustomPaint#00000\n'
        '                         │     parentData: <none> (can use size)\n'
        '                         │     constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         │     size: Size(800.0, 400.0)\n'
        '                         │     painter: _PlaceholderPainter#00000()\n'
        '                         │     preferredSize: Size(Infinity, Infinity)\n'
        '                         │\n'
        '                         └─child with index 2: RenderLimitedBox#00000 NEEDS-PAINT\n'
        '                           │ parentData: index=2; layoutOffset=800.0\n'
        '                           │ constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                           │ size: Size(800.0, 400.0)\n'
        '                           │ maxWidth: 400.0\n'
        '                           │ maxHeight: 400.0\n'
        '                           │\n'
        '                           └─child: RenderCustomPaint#00000 NEEDS-PAINT\n'
        '                               parentData: <none> (can use size)\n'
        '                               constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                               size: Size(800.0, 400.0)\n'
        '                               painter: _PlaceholderPainter#00000()\n'
        '                               preferredSize: Size(Infinity, Infinity)\n',
      ),
    );
    const GlobalObjectKey<_LeafState>(0).currentState!.setKeepAlive(true);
    await tester.drag(find.byType(ListView), const Offset(0.0, -1000.0));
    await tester.pump();
    const GlobalObjectKey<_LeafState>(3).currentState!.setKeepAlive(true);
    await tester.drag(find.byType(ListView), const Offset(0.0, -1000.0));
    await tester.pump();
    expect(
      tester.binding.renderView.toStringDeep(minLevel: DiagnosticLevel.info, wrapWidth: 600),
      equalsIgnoringHashCodes(
        '_ReusableRenderView#00000\n'
        ' │ debug mode enabled - ${Platform.operatingSystem}\n'
        ' │ view size: Size(2400.0, 1800.0) (in physical pixels)\n'
        ' │ device pixel ratio: 3.0 (physical pixels per logical pixel)\n'
        ' │ configuration: BoxConstraints(w=800.0, h=600.0) at 3.0x (in logical pixels)\n'
        ' │\n'
        ' └─child: RenderRepaintBoundary#00000\n'
        '   │ needs compositing\n'
        '   │ parentData: <none>\n'
        '   │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '   │ layer: OffsetLayer#00000\n'
        '   │ size: Size(800.0, 600.0)\n'
        '   │ metrics: 0.0% useful (1 bad vs 0 good)\n'
        '   │ diagnosis: insufficient data to draw conclusion (less than five repaints)\n'
        '   │\n'
        '   └─child: RenderCustomPaint#00000\n'
        '     │ needs compositing\n'
        '     │ parentData: <none> (can use size)\n'
        '     │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '     │ size: Size(800.0, 600.0)\n'
        '     │ painter: null\n'
        '     │ foregroundPainter: _GlowingOverscrollIndicatorPainter(_GlowController(color: ${const Color(0xffffffff)}, axis: vertical), _GlowController(color: ${const Color(0xffffffff)}, axis: vertical))\n'
        '     │\n'
        '     └─child: RenderRepaintBoundary#00000\n'
        '       │ needs compositing\n'
        '       │ parentData: <none> (can use size)\n'
        '       │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '       │ layer: OffsetLayer#00000\n'
        '       │ size: Size(800.0, 600.0)\n'
        '       │ metrics: 0.0% useful (1 bad vs 0 good)\n'
        '       │ diagnosis: insufficient data to draw conclusion (less than five repaints)\n'
        '       │\n'
        '       └─child: _RenderScrollSemantics#00000\n'
        '         │ needs compositing\n'
        '         │ parentData: <none> (can use size)\n'
        '         │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '         │ semantics node: SemanticsNode#1\n'
        '         │ semantic boundary\n'
        '         │ size: Size(800.0, 600.0)\n'
        '         │\n'
        '         └─child: RenderPointerListener#00000\n'
        '           │ needs compositing\n'
        '           │ parentData: <none> (can use size)\n'
        '           │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '           │ size: Size(800.0, 600.0)\n'
        '           │ behavior: deferToChild\n'
        '           │ listeners: signal\n'
        '           │\n'
        '           └─child: RenderSemanticsGestureHandler#00000\n'
        '             │ needs compositing\n'
        '             │ parentData: <none> (can use size)\n'
        '             │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '             │ size: Size(800.0, 600.0)\n'
        '             │ behavior: opaque\n'
        '             │ gestures: vertical scroll\n'
        '             │\n'
        '             └─child: RenderPointerListener#00000\n'
        '               │ needs compositing\n'
        '               │ parentData: <none> (can use size)\n'
        '               │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '               │ size: Size(800.0, 600.0)\n'
        '               │ behavior: opaque\n'
        '               │ listeners: down, panZoomStart\n'
        '               │\n'
        '               └─child: RenderSemanticsAnnotations#00000\n'
        '                 │ needs compositing\n'
        '                 │ parentData: <none> (can use size)\n'
        '                 │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '                 │ size: Size(800.0, 600.0)\n'
        '                 │\n'
        '                 └─child: RenderIgnorePointer#00000\n'
        '                   │ needs compositing\n'
        '                   │ parentData: <none> (can use size)\n'
        '                   │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '                   │ size: Size(800.0, 600.0)\n'
        '                   │ ignoring: false\n'
        '                   │ ignoringSemantics: null\n'
        '                   │\n'
        '                   └─child: RenderViewport#00000\n'
        '                     │ needs compositing\n'
        '                     │ parentData: <none> (can use size)\n'
        '                     │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '                     │ layer: OffsetLayer#00000\n'
        '                     │ size: Size(800.0, 600.0)\n'
        '                     │ axisDirection: down\n'
        '                     │ crossAxisDirection: right\n'
        '                     │ offset: ScrollPositionWithSingleContext#00000(offset: 2000.0, range: 0.0..39400.0, viewport: 600.0, ScrollableState, AlwaysScrollableScrollPhysics -> ClampingScrollPhysics -> RangeMaintainingScrollPhysics, IdleScrollActivity#00000, ScrollDirection.idle)\n'
        '                     │ anchor: 0.0\n'
        '                     │\n'
        '                     └─center child: RenderSliverPadding#00000 relayoutBoundary=up1\n'
        '                       │ parentData: paintOffset=Offset(0.0, 0.0) (can use size)\n'
        '                       │ constraints: SliverConstraints(AxisDirection.down, GrowthDirection.forward, ScrollDirection.idle, scrollOffset: 2000.0, precedingScrollExtent: 0.0, remainingPaintExtent: 600.0, crossAxisExtent: 800.0, crossAxisDirection: AxisDirection.right, viewportMainAxisExtent: 600.0, remainingCacheExtent: 1100.0, cacheOrigin: -250.0)\n'
        '                       │ geometry: SliverGeometry(scrollExtent: 40000.0, paintExtent: 600.0, maxPaintExtent: 40000.0, hasVisualOverflow: true, cacheExtent: 1100.0)\n'
        '                       │ padding: EdgeInsets.zero\n'
        '                       │ textDirection: ltr\n'
        '                       │\n'
        '                       └─child: RenderSliverFixedExtentList#00000 relayoutBoundary=up2\n'
        '                         │ parentData: paintOffset=Offset(0.0, 0.0) (can use size)\n'
        '                         │ constraints: SliverConstraints(AxisDirection.down, GrowthDirection.forward, ScrollDirection.idle, scrollOffset: 2000.0, precedingScrollExtent: 0.0, remainingPaintExtent: 600.0, crossAxisExtent: 800.0, crossAxisDirection: AxisDirection.right, viewportMainAxisExtent: 600.0, remainingCacheExtent: 1100.0, cacheOrigin: -250.0)\n'
        '                         │ geometry: SliverGeometry(scrollExtent: 40000.0, paintExtent: 600.0, maxPaintExtent: 40000.0, hasVisualOverflow: true, cacheExtent: 1100.0)\n'
        '                         │ currently live children: 4 to 7\n'
        '                         │\n'
        '                         ├─child with index 4: RenderLimitedBox#00000 NEEDS-PAINT\n'
        '                         │ │ parentData: index=4; layoutOffset=1600.0\n'
        '                         │ │ constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         │ │ size: Size(800.0, 400.0)\n'
        '                         │ │ maxWidth: 400.0\n'
        '                         │ │ maxHeight: 400.0\n'
        '                         │ │\n'
        '                         │ └─child: RenderCustomPaint#00000 NEEDS-PAINT\n'
        '                         │     parentData: <none> (can use size)\n'
        '                         │     constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         │     size: Size(800.0, 400.0)\n'
        '                         │     painter: _PlaceholderPainter#00000()\n'
        '                         │     preferredSize: Size(Infinity, Infinity)\n'
        '                         │\n'
        '                         ├─child with index 5: RenderLimitedBox#00000\n' // <----- this is index 5, not 0
        '                         │ │ parentData: index=5; layoutOffset=2000.0\n'
        '                         │ │ constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         │ │ size: Size(800.0, 400.0)\n'
        '                         │ │ maxWidth: 400.0\n'
        '                         │ │ maxHeight: 400.0\n'
        '                         │ │\n'
        '                         │ └─child: RenderCustomPaint#00000\n'
        '                         │     parentData: <none> (can use size)\n'
        '                         │     constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         │     size: Size(800.0, 400.0)\n'
        '                         │     painter: _PlaceholderPainter#00000()\n'
        '                         │     preferredSize: Size(Infinity, Infinity)\n'
        '                         │\n'
        '                         ├─child with index 6: RenderLimitedBox#00000\n'
        '                         │ │ parentData: index=6; layoutOffset=2400.0\n'
        '                         │ │ constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         │ │ size: Size(800.0, 400.0)\n'
        '                         │ │ maxWidth: 400.0\n'
        '                         │ │ maxHeight: 400.0\n'
        '                         │ │\n'
        '                         │ └─child: RenderCustomPaint#00000\n'
        '                         │     parentData: <none> (can use size)\n'
        '                         │     constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         │     size: Size(800.0, 400.0)\n'
        '                         │     painter: _PlaceholderPainter#00000()\n'
        '                         │     preferredSize: Size(Infinity, Infinity)\n'
        '                         │\n'
        '                         ├─child with index 7: RenderLimitedBox#00000 NEEDS-PAINT\n'
        '                         ╎ │ parentData: index=7; layoutOffset=2800.0\n'
        '                         ╎ │ constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         ╎ │ size: Size(800.0, 400.0)\n'
        '                         ╎ │ maxWidth: 400.0\n'
        '                         ╎ │ maxHeight: 400.0\n'
        '                         ╎ │\n'
        '                         ╎ └─child: RenderCustomPaint#00000 NEEDS-PAINT\n'
        '                         ╎     parentData: <none> (can use size)\n'
        '                         ╎     constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         ╎     size: Size(800.0, 400.0)\n'
        '                         ╎     painter: _PlaceholderPainter#00000()\n'
        '                         ╎     preferredSize: Size(Infinity, Infinity)\n'
        '                         ╎\n'
        '                         ╎╌child with index 0 (kept alive but not laid out): RenderLimitedBox#00000\n' // <----- this one is index 0 and is marked as being kept alive but not laid out
        '                         ╎ │ parentData: index=0; keepAlive; layoutOffset=0.0\n'
        '                         ╎ │ constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         ╎ │ size: Size(800.0, 400.0)\n'
        '                         ╎ │ maxWidth: 400.0\n'
        '                         ╎ │ maxHeight: 400.0\n'
        '                         ╎ │\n'
        '                         ╎ └─child: RenderCustomPaint#00000\n'
        '                         ╎     parentData: <none> (can use size)\n'
        '                         ╎     constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                         ╎     size: Size(800.0, 400.0)\n'
        '                         ╎     painter: _PlaceholderPainter#00000()\n'
        '                         ╎     preferredSize: Size(Infinity, Infinity)\n'
        '                         ╎\n'
        '                         └╌child with index 3 (kept alive but not laid out): RenderLimitedBox#00000\n'
        '                           │ parentData: index=3; keepAlive; layoutOffset=1200.0\n'
        '                           │ constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                           │ size: Size(800.0, 400.0)\n'
        '                           │ maxWidth: 400.0\n'
        '                           │ maxHeight: 400.0\n'
        '                           │\n'
        '                           └─child: RenderCustomPaint#00000\n'
        '                               parentData: <none> (can use size)\n'
        '                               constraints: BoxConstraints(w=800.0, h=400.0)\n'
        '                               size: Size(800.0, 400.0)\n'
        '                               painter: _PlaceholderPainter#00000()\n'
        '                               preferredSize: Size(Infinity, Infinity)\n',
      ),
    );
  }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/87876
}
