// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' show FlutterView;

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

class ScheduledFrameTrackingPlatformDispatcher extends TestPlatformDispatcher {
  ScheduledFrameTrackingPlatformDispatcher({required super.platformDispatcher});

  int _scheduledFrameCount = 0;
  int get scheduledFrameCount => _scheduledFrameCount;

  void resetScheduledFrameCount() {
    _scheduledFrameCount = 0;
  }

  @override
  void scheduleFrame() {
    _scheduledFrameCount++;
    super.scheduleFrame();
  }
}

class ScheduledFrameTrackingBindings extends AutomatedTestWidgetsFlutterBinding {
  late final ScheduledFrameTrackingPlatformDispatcher _platformDispatcher =
      ScheduledFrameTrackingPlatformDispatcher(platformDispatcher: super.platformDispatcher);

  @override
  ScheduledFrameTrackingPlatformDispatcher get platformDispatcher => _platformDispatcher;
}

class OffscreenRenderView extends RenderView {
  OffscreenRenderView({required super.view})
    : super(configuration: TestViewConfiguration.fromView(view: view));

  @override
  void compositeFrame() {
    // Don't draw to ui.window
  }
}

class OffscreenWidgetTree {
  OffscreenWidgetTree(this.view) {
    renderView.attach(pipelineOwner);
    renderView.prepareInitialFrame();
    pipelineOwner.requestVisualUpdate();
  }

  final FlutterView view;
  late final RenderView renderView = OffscreenRenderView(view: view);
  final BuildOwner buildOwner = BuildOwner(focusManager: FocusManager());
  final PipelineOwner pipelineOwner = PipelineOwner();
  RenderObjectToWidgetElement<RenderBox>? root;

  void pumpWidget(Widget? app) {
    root = RenderObjectToWidgetAdapter<RenderBox>(
      container: renderView,
      debugShortDescription: '[root]',
      child: app,
    ).attachToRenderTree(buildOwner, root);
    pumpFrame();
  }

  void pumpFrame() {
    buildOwner.buildScope(root!);
    pipelineOwner.flushLayout();
    pipelineOwner.flushCompositingBits();
    pipelineOwner.flushPaint();
    renderView.compositeFrame();
    pipelineOwner.flushSemantics();
    buildOwner.finalizeTree();
  }
}

class Counter {
  int count = 0;
}

class Trigger {
  VoidCallback? callback;
  void fire() {
    callback?.call();
  }
}

class TriggerableWidget extends StatefulWidget {
  const TriggerableWidget({super.key, required this.trigger, required this.counter});

  final Trigger trigger;
  final Counter counter;

  @override
  TriggerableState createState() => TriggerableState();
}

class TriggerableState extends State<TriggerableWidget> {
  @override
  void initState() {
    super.initState();
    widget.trigger.callback = fire;
  }

  @override
  void didUpdateWidget(TriggerableWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    widget.trigger.callback = fire;
  }

  int _count = 0;
  void fire() {
    setState(() {
      _count++;
    });
  }

  @override
  Widget build(BuildContext context) {
    widget.counter.count++;
    return Text('Bang $_count!', textDirection: TextDirection.ltr);
  }
}

class TestFocusable extends StatefulWidget {
  const TestFocusable({super.key, required this.focusNode, this.autofocus = true});

  final bool autofocus;
  final FocusNode focusNode;

  @override
  TestFocusableState createState() => TestFocusableState();
}

class TestFocusableState extends State<TestFocusable> {
  bool _didAutofocus = false;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    if (!_didAutofocus && widget.autofocus) {
      _didAutofocus = true;
      FocusScope.of(context).autofocus(widget.focusNode);
    }
  }

  @override
  Widget build(BuildContext context) {
    return const Text('Test focus node', textDirection: TextDirection.ltr);
  }
}

void main() {
  // Override the bindings for this test suite so that we can track the number
  // of times a frame has been scheduled.
  ScheduledFrameTrackingBindings();

  testWidgets('RenderObjectToWidgetAdapter.attachToRenderTree does not schedule frame', (
    WidgetTester tester,
  ) async {
    expect(WidgetsBinding.instance, isA<ScheduledFrameTrackingBindings>());
    final ScheduledFrameTrackingPlatformDispatcher platformDispatcher =
        tester.platformDispatcher as ScheduledFrameTrackingPlatformDispatcher;
    platformDispatcher.resetScheduledFrameCount();
    expect(platformDispatcher.scheduledFrameCount, isZero);
    final OffscreenWidgetTree tree = OffscreenWidgetTree(tester.view);
    tree.pumpWidget(const SizedBox.shrink());
    expect(platformDispatcher.scheduledFrameCount, isZero);
  });

  testWidgets('no crosstalk between widget build owners', (WidgetTester tester) async {
    final Trigger trigger1 = Trigger();
    final Counter counter1 = Counter();
    final Trigger trigger2 = Trigger();
    final Counter counter2 = Counter();
    final OffscreenWidgetTree tree = OffscreenWidgetTree(tester.view);
    // Both counts should start at zero
    expect(counter1.count, equals(0));
    expect(counter2.count, equals(0));
    // Lay out the "onscreen" in the default test binding
    await tester.pumpWidget(TriggerableWidget(trigger: trigger1, counter: counter1));
    // Only the "onscreen" widget should have built
    expect(counter1.count, equals(1));
    expect(counter2.count, equals(0));
    // Lay out the "offscreen" in a separate tree
    tree.pumpWidget(TriggerableWidget(trigger: trigger2, counter: counter2));
    // Now both widgets should have built
    expect(counter1.count, equals(1));
    expect(counter2.count, equals(1));
    // Mark both as needing layout
    trigger1.fire();
    trigger2.fire();
    // Marking as needing layout shouldn't immediately build anything
    expect(counter1.count, equals(1));
    expect(counter2.count, equals(1));
    // Pump the "onscreen" layout
    await tester.pump();
    // Only the "onscreen" widget should have rebuilt
    expect(counter1.count, equals(2));
    expect(counter2.count, equals(1));
    // Pump the "offscreen" layout
    tree.pumpFrame();
    // Now both widgets should have rebuilt
    expect(counter1.count, equals(2));
    expect(counter2.count, equals(2));
    // Mark both as needing layout, again
    trigger1.fire();
    trigger2.fire();
    // Now pump the "offscreen" layout first
    tree.pumpFrame();
    // Only the "offscreen" widget should have rebuilt
    expect(counter1.count, equals(2));
    expect(counter2.count, equals(3));
    // Pump the "onscreen" layout
    await tester.pump();
    // Now both widgets should have rebuilt
    expect(counter1.count, equals(3));
    expect(counter2.count, equals(3));
  });

  testWidgets('no crosstalk between focus nodes', (WidgetTester tester) async {
    final OffscreenWidgetTree tree = OffscreenWidgetTree(tester.view);
    final FocusNode onscreenFocus = FocusNode();
    addTearDown(onscreenFocus.dispose);
    final FocusNode offscreenFocus = FocusNode();
    addTearDown(offscreenFocus.dispose);

    await tester.pumpWidget(TestFocusable(focusNode: onscreenFocus));
    tree.pumpWidget(TestFocusable(focusNode: offscreenFocus));

    // Autofocus is delayed one frame.
    await tester.pump();
    tree.pumpFrame();

    expect(onscreenFocus.hasFocus, isTrue);
    expect(offscreenFocus.hasFocus, isTrue);
  });

  testWidgets('able to tear down offscreen tree', (WidgetTester tester) async {
    final OffscreenWidgetTree tree = OffscreenWidgetTree(tester.view);
    final List<WidgetState> states = <WidgetState>[];
    tree.pumpWidget(SizedBox(child: TestStates(states: states)));
    expect(states, <WidgetState>[WidgetState.initialized]);
    expect(tree.renderView.child, isNotNull);
    tree.pumpWidget(null); // The root node should be allowed to have no child.
    expect(states, <WidgetState>[WidgetState.initialized, WidgetState.disposed]);
    expect(tree.renderView.child, isNull);
  });
}

enum WidgetState { initialized, disposed }

class TestStates extends StatefulWidget {
  const TestStates({super.key, required this.states});

  final List<WidgetState> states;

  @override
  TestStatesState createState() => TestStatesState();
}

class TestStatesState extends State<TestStates> {
  @override
  void initState() {
    super.initState();
    widget.states.add(WidgetState.initialized);
  }

  @override
  void dispose() {
    widget.states.add(WidgetState.disposed);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) => Container();
}
