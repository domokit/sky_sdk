// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

// This returns render paragraph of the Tab label text.
RenderParagraph getTabText(WidgetTester tester, String text) {
  return tester.renderObject<RenderParagraph>(
    find.descendant(
      of: find.byWidgetPredicate((Widget widget) => widget.runtimeType.toString() == '_TabStyle'),
      matching: find.text(text),
    ),
  );
}

// This creates and returns a TabController.
TabController createTabController({
  required int length,
  required TickerProvider vsync,
  int initialIndex = 0,
  Duration? animationDuration,
}) {
  final TabController result = TabController(
    length: length,
    vsync: vsync,
    initialIndex: initialIndex,
    animationDuration: animationDuration,
  );
  addTearDown(result.dispose);
  return result;
}

// This widget is used to test widget state in the tabs_test.dart file.
class TabStateMarker extends StatefulWidget {
  const TabStateMarker({super.key, this.child});

  final Widget? child;

  @override
  TabStateMarkerState createState() => TabStateMarkerState();
}

class TabStateMarkerState extends State<TabStateMarker> {
  String? marker;

  @override
  Widget build(BuildContext context) {
    return widget.child ?? Container();
  }
}

// Tab controller builder for TabControllerFrame widget.
typedef TabControllerFrameBuilder = Widget Function(BuildContext context, TabController controller);

// This widget creates a TabController and passes it to the builder.
class TabControllerFrame extends StatefulWidget {
  const TabControllerFrame({
    super.key,
    required this.length,
    this.initialIndex = 0,
    required this.builder,
  });

  final int length;
  final int initialIndex;
  final TabControllerFrameBuilder builder;

  @override
  TabControllerFrameState createState() => TabControllerFrameState();
}

class TabControllerFrameState extends State<TabControllerFrame>
    with SingleTickerProviderStateMixin {
  late TabController _controller;

  @override
  void initState() {
    super.initState();
    _controller = TabController(
      vsync: this,
      length: widget.length,
      initialIndex: widget.initialIndex,
    );
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return widget.builder(context, _controller);
  }
}

// Test utility class to test tab indicator drawing.
class TabIndicatorRecordingCanvas extends TestRecordingCanvas {
  TabIndicatorRecordingCanvas(this.indicatorColor);

  final Color indicatorColor;
  late Rect indicatorRect;

  @override
  void drawLine(Offset p1, Offset p2, Paint paint) {
    // Assuming that the indicatorWeight is 2.0, the default.
    const double indicatorWeight = 2.0;
    if (paint.color == indicatorColor) {
      indicatorRect = Rect.fromPoints(p1, p2).inflate(indicatorWeight / 2.0);
    }
  }
}

// This creates a Fake implementation of ScrollMetrics.
class TabMockScrollMetrics extends Fake implements ScrollMetrics {}

class TabBarTestScrollPhysics extends ScrollPhysics {
  const TabBarTestScrollPhysics({super.parent});

  @override
  TabBarTestScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return TabBarTestScrollPhysics(parent: buildParent(ancestor));
  }

  @override
  double applyPhysicsToUserOffset(ScrollMetrics position, double offset) {
    return offset == 10 ? 20 : offset;
  }

  static final SpringDescription _kDefaultSpring = SpringDescription.withDampingRatio(
    mass: 0.5,
    stiffness: 500.0,
    ratio: 1.1,
  );

  @override
  SpringDescription get spring => _kDefaultSpring;
}

// This widget is used to log the lifecycle of the TabBarView children.
class TabBody extends StatefulWidget {
  const TabBody({super.key, required this.index, required this.log, this.marker = ''});

  final int index;
  final List<String> log;
  final String marker;

  @override
  State<TabBody> createState() => TabBodyState();
}

class TabBodyState extends State<TabBody> {
  @override
  void initState() {
    widget.log.add('init: ${widget.index}');
    super.initState();
  }

  @override
  void didUpdateWidget(TabBody oldWidget) {
    super.didUpdateWidget(oldWidget);
    // To keep the logging straight, widgets must not change their index.
    assert(oldWidget.index == widget.index);
  }

  @override
  void dispose() {
    widget.log.add('dispose: ${widget.index}');
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child:
          widget.marker.isEmpty
              ? Text('${widget.index}')
              : Text('${widget.index}-${widget.marker}'),
    );
  }
}

// This widget is used to test the lifecycle of the TabBarView children with Ink widget.
class TabKeepAliveInk extends StatefulWidget {
  const TabKeepAliveInk({super.key, required this.title});

  final String title;

  @override
  State<StatefulWidget> createState() => _TabKeepAliveInkState();
}

class _TabKeepAliveInkState extends State<TabKeepAliveInk> with AutomaticKeepAliveClientMixin {
  @override
  bool get wantKeepAlive => true;

  @override
  Widget build(BuildContext context) {
    super.build(context);
    return Ink(child: Text(widget.title));
  }
}

// This widget is used to test the lifecycle of the TabBarView children.
class TabAlwaysKeepAliveWidget extends StatefulWidget {
  const TabAlwaysKeepAliveWidget({super.key});

  static String text = 'AlwaysKeepAlive';

  @override
  State<TabAlwaysKeepAliveWidget> createState() => _TabAlwaysKeepAliveWidgetState();
}

class _TabAlwaysKeepAliveWidgetState extends State<TabAlwaysKeepAliveWidget>
    with AutomaticKeepAliveClientMixin {
  @override
  bool get wantKeepAlive => true;

  @override
  Widget build(BuildContext context) {
    super.build(context);
    return Text(TabAlwaysKeepAliveWidget.text);
  }
}

// This decoration is used to test the indicator decoration image configuration.
class TestIndicatorDecoration extends Decoration {
  final List<TestIndicatorBoxPainter> painters = <TestIndicatorBoxPainter>[];

  @override
  BoxPainter createBoxPainter([VoidCallback? onChanged]) {
    final TestIndicatorBoxPainter painter = TestIndicatorBoxPainter();
    painters.add(painter);
    return painter;
  }
}

class TestIndicatorBoxPainter extends BoxPainter {
  ImageConfiguration? lastConfiguration;

  @override
  void paint(Canvas canvas, Offset offset, ImageConfiguration configuration) {
    lastConfiguration = configuration;
  }
}
