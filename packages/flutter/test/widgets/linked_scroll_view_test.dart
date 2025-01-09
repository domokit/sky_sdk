// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file contains a wacky demonstration of creating a custom ScrollPosition
// setup. It's testing that we don't regress the factoring of the
// ScrollPosition/ScrollActivity logic into a state where you can no longer
// implement this, e.g. by oversimplifying it or overfitting it to the features
// built into the framework itself.

import 'dart:collection';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

class LinkedScrollController extends ScrollController {
  LinkedScrollController({this.before, this.after});

  LinkedScrollController? before;
  LinkedScrollController? after;

  ScrollController? _parent;

  void setParent(ScrollController? newParent) {
    if (_parent != null) {
      positions.forEach(_parent!.detach);
    }
    _parent = newParent;
    if (_parent != null) {
      positions.forEach(_parent!.attach);
    }
  }

  @override
  void attach(ScrollPosition position) {
    assert(
      position is LinkedScrollPosition,
      'A LinkedScrollController must only be used with LinkedScrollPositions.',
    );
    final LinkedScrollPosition linkedPosition = position as LinkedScrollPosition;
    assert(
      linkedPosition.owner == this,
      'A LinkedScrollPosition cannot change controllers once created.',
    );
    super.attach(position);
    _parent?.attach(position);
  }

  @override
  void detach(ScrollPosition position) {
    super.detach(position);
    _parent?.detach(position);
  }

  @override
  void dispose() {
    if (_parent != null) {
      positions.forEach(_parent!.detach);
    }
    super.dispose();
  }

  @override
  LinkedScrollPosition createScrollPosition(
    ScrollPhysics physics,
    ScrollContext context,
    ScrollPosition? oldPosition,
  ) {
    return LinkedScrollPosition(
      this,
      physics: physics,
      context: context,
      initialPixels: initialScrollOffset,
      oldPosition: oldPosition,
    );
  }

  bool get canLinkWithBefore => before != null && before!.hasClients;

  bool get canLinkWithAfter => after != null && after!.hasClients;

  Iterable<LinkedScrollActivity> linkWithBefore(LinkedScrollPosition driver) {
    assert(canLinkWithBefore);
    return before!.link(driver);
  }

  Iterable<LinkedScrollActivity> linkWithAfter(LinkedScrollPosition driver) {
    assert(canLinkWithAfter);
    return after!.link(driver);
  }

  Iterable<LinkedScrollActivity> link(LinkedScrollPosition driver) sync* {
    assert(hasClients);
    for (final LinkedScrollPosition position in positions.cast<LinkedScrollPosition>()) {
      yield position.link(driver);
    }
  }

  @override
  void debugFillDescription(List<String> description) {
    super.debugFillDescription(description);
    final String linkSymbol = switch ((before, after)) {
      (null, null) => 'none',
      (null, _) => '➡',
      (_, null) => '⬅',
      (_, _) => '⬌',
    };
    description.add('links: $linkSymbol');
  }
}

class LinkedScrollPosition extends ScrollPositionWithSingleContext {
  LinkedScrollPosition(
    this.owner, {
    required super.physics,
    required super.context,
    required double super.initialPixels,
    super.oldPosition,
  });

  final LinkedScrollController owner;

  Set<LinkedScrollActivity>? _beforeActivities;
  Set<LinkedScrollActivity>? _afterActivities;

  @override
  void beginActivity(ScrollActivity? newActivity) {
    if (newActivity == null) {
      return;
    }
    if (_beforeActivities != null) {
      for (final LinkedScrollActivity activity in _beforeActivities!) {
        activity.unlink(this);
      }
      _beforeActivities!.clear();
    }
    if (_afterActivities != null) {
      for (final LinkedScrollActivity activity in _afterActivities!) {
        activity.unlink(this);
      }
      _afterActivities!.clear();
    }
    super.beginActivity(newActivity);
  }

  @override
  void applyUserOffset(double delta) {
    updateUserScrollDirection(delta > 0.0 ? ScrollDirection.forward : ScrollDirection.reverse);
    final double value = pixels - physics.applyPhysicsToUserOffset(this, delta);

    if (value == pixels) {
      return;
    }

    double beforeOverscroll = 0.0;
    if (owner.canLinkWithBefore && (value < minScrollExtent)) {
      final double delta = value - minScrollExtent;
      _beforeActivities ??= HashSet<LinkedScrollActivity>();
      _beforeActivities!.addAll(owner.linkWithBefore(this));
      for (final LinkedScrollActivity activity in _beforeActivities!) {
        beforeOverscroll = math.min(activity.moveBy(delta), beforeOverscroll);
      }
      assert(beforeOverscroll <= 0.0);
    }

    double afterOverscroll = 0.0;
    if (owner.canLinkWithAfter && (value > maxScrollExtent)) {
      final double delta = value - maxScrollExtent;
      _afterActivities ??= HashSet<LinkedScrollActivity>();
      _afterActivities!.addAll(owner.linkWithAfter(this));
      for (final LinkedScrollActivity activity in _afterActivities!) {
        afterOverscroll = math.max(activity.moveBy(delta), afterOverscroll);
      }
      assert(afterOverscroll >= 0.0);
    }

    assert(beforeOverscroll == 0.0 || afterOverscroll == 0.0);

    final double localOverscroll = setPixels(
      value.clamp(
        owner.canLinkWithBefore ? minScrollExtent : -double.infinity,
        owner.canLinkWithAfter ? maxScrollExtent : double.infinity,
      ),
    );

    assert(localOverscroll == 0.0 || (beforeOverscroll == 0.0 && afterOverscroll == 0.0));
  }

  void _userMoved(ScrollDirection direction) {
    updateUserScrollDirection(direction);
  }

  LinkedScrollActivity link(LinkedScrollPosition driver) {
    if (this.activity is! LinkedScrollActivity) {
      beginActivity(LinkedScrollActivity(this));
    }
    final LinkedScrollActivity? activity = this.activity as LinkedScrollActivity?;
    activity!.link(driver);
    return activity;
  }

  void unlink(LinkedScrollActivity activity) {
    _beforeActivities?.remove(activity);
    _afterActivities?.remove(activity);
  }

  @override
  void debugFillDescription(List<String> description) {
    super.debugFillDescription(description);
    description.add('owner: $owner');
  }
}

class LinkedScrollActivity extends ScrollActivity {
  LinkedScrollActivity(LinkedScrollPosition super.delegate);

  @override
  LinkedScrollPosition get delegate => super.delegate as LinkedScrollPosition;

  final Set<LinkedScrollPosition> drivers = HashSet<LinkedScrollPosition>();

  void link(LinkedScrollPosition driver) {
    drivers.add(driver);
  }

  void unlink(LinkedScrollPosition driver) {
    drivers.remove(driver);
    if (drivers.isEmpty) {
      delegate.goIdle();
    }
  }

  @override
  bool get shouldIgnorePointer => true;

  @override
  bool get isScrolling => true;

  // LinkedScrollActivity is not self-driven but moved by calls to the [moveBy]
  // method.
  @override
  double get velocity => 0.0;

  double moveBy(double delta) {
    assert(drivers.isNotEmpty);
    ScrollDirection? commonDirection;
    for (final LinkedScrollPosition driver in drivers) {
      commonDirection ??= driver.userScrollDirection;
      if (driver.userScrollDirection != commonDirection) {
        commonDirection = ScrollDirection.idle;
      }
    }

    if (commonDirection != null) {
      delegate._userMoved(commonDirection);
    }
    return delegate.setPixels(delegate.pixels + delta);
  }

  @override
  void dispose() {
    for (final LinkedScrollPosition driver in drivers) {
      driver.unlink(this);
    }
    super.dispose();
  }
}

class Test extends StatefulWidget {
  const Test({super.key});
  @override
  State<Test> createState() => _TestState();
}

class _TestState extends State<Test> {
  late LinkedScrollController _beforeController;
  late LinkedScrollController _afterController;

  @override
  void initState() {
    super.initState();
    _beforeController = LinkedScrollController();
    _afterController = LinkedScrollController(before: _beforeController);
    _beforeController.after = _afterController;
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _beforeController.setParent(PrimaryScrollController.maybeOf(context));
    _afterController.setParent(PrimaryScrollController.maybeOf(context));
  }

  @override
  void dispose() {
    _beforeController.dispose();
    _afterController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Directionality(
      textDirection: TextDirection.ltr,
      child: Column(
        children: <Widget>[
          Expanded(
            child: ListView(
              controller: _beforeController,
              children: <Widget>[
                Container(
                  margin: const EdgeInsets.all(8.0),
                  padding: const EdgeInsets.all(8.0),
                  height: 250.0,
                  color: const Color(0xFF90F090),
                  child: const Center(child: Text('Hello A')),
                ),
                Container(
                  margin: const EdgeInsets.all(8.0),
                  padding: const EdgeInsets.all(8.0),
                  height: 250.0,
                  color: const Color(0xFF90F090),
                  child: const Center(child: Text('Hello B')),
                ),
                Container(
                  margin: const EdgeInsets.all(8.0),
                  padding: const EdgeInsets.all(8.0),
                  height: 250.0,
                  color: const Color(0xFF90F090),
                  child: const Center(child: Text('Hello C')),
                ),
                Container(
                  margin: const EdgeInsets.all(8.0),
                  padding: const EdgeInsets.all(8.0),
                  height: 250.0,
                  color: const Color(0xFF90F090),
                  child: const Center(child: Text('Hello D')),
                ),
              ],
            ),
          ),
          const Divider(),
          Expanded(
            child: ListView(
              controller: _afterController,
              children: <Widget>[
                Container(
                  margin: const EdgeInsets.all(8.0),
                  padding: const EdgeInsets.all(8.0),
                  height: 250.0,
                  color: const Color(0xFF9090F0),
                  child: const Center(child: Text('Hello 1')),
                ),
                Container(
                  margin: const EdgeInsets.all(8.0),
                  padding: const EdgeInsets.all(8.0),
                  height: 250.0,
                  color: const Color(0xFF9090F0),
                  child: const Center(child: Text('Hello 2')),
                ),
                Container(
                  margin: const EdgeInsets.all(8.0),
                  padding: const EdgeInsets.all(8.0),
                  height: 250.0,
                  color: const Color(0xFF9090F0),
                  child: const Center(child: Text('Hello 3')),
                ),
                Container(
                  margin: const EdgeInsets.all(8.0),
                  padding: const EdgeInsets.all(8.0),
                  height: 250.0,
                  color: const Color(0xFF9090F0),
                  child: const Center(child: Text('Hello 4')),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

void main() {
  testWidgets('LinkedScrollController - 1', (WidgetTester tester) async {
    await tester.pumpWidget(const Test());
    expect(find.text('Hello A'), findsOneWidget);
    expect(find.text('Hello 1'), findsOneWidget);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 4'), findsNothing);
    await tester.pump(const Duration(seconds: 2));
    await tester.fling(find.text('Hello A'), const Offset(0.0, -50.0), 10000.0);
    await tester.pumpAndSettle();
    await tester.pump(const Duration(seconds: 2));
    expect(find.text('Hello A'), findsNothing);
    expect(find.text('Hello 1'), findsOneWidget);
    expect(find.text('Hello D'), findsOneWidget);
    expect(find.text('Hello 4'), findsNothing);
    await tester.pump(const Duration(seconds: 2));
    await tester.drag(find.text('Hello D'), const Offset(0.0, -10000.0));
    await tester.pump(const Duration(seconds: 2));
    expect(find.text('Hello A'), findsNothing);
    expect(find.text('Hello 1'), findsNothing);
    expect(find.text('Hello D'), findsOneWidget);
    expect(find.text('Hello 4'), findsOneWidget);
    await tester.pump(const Duration(seconds: 2));
    await tester.drag(find.text('Hello D'), const Offset(0.0, -10000.0));
    await tester.pump(const Duration(seconds: 2));
    expect(find.text('Hello A'), findsNothing);
    expect(find.text('Hello 1'), findsNothing);
    expect(find.text('Hello D'), findsOneWidget);
    expect(find.text('Hello 4'), findsOneWidget);
    await tester.pump(const Duration(seconds: 2));
    await tester.drag(find.text('Hello 4'), const Offset(0.0, -10000.0));
    await tester.pump(const Duration(seconds: 2));
    expect(find.text('Hello A'), findsNothing);
    expect(find.text('Hello 1'), findsNothing);
    expect(find.text('Hello D'), findsOneWidget);
    expect(find.text('Hello 4'), findsOneWidget);
    await tester.pump(const Duration(seconds: 2));
    await tester.drag(find.text('Hello D'), const Offset(0.0, 10000.0));
    await tester.pump(const Duration(seconds: 2));
    expect(find.text('Hello A'), findsOneWidget);
    expect(find.text('Hello 1'), findsNothing);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 4'), findsOneWidget);
    await tester.pump(const Duration(seconds: 2));
    await tester.drag(find.text('Hello A'), const Offset(0.0, 10000.0));
    await tester.pump(const Duration(seconds: 2));
    expect(find.text('Hello A'), findsOneWidget);
    expect(find.text('Hello 1'), findsNothing);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 4'), findsOneWidget);
    await tester.pump(const Duration(seconds: 2));
    await tester.drag(find.text('Hello A'), const Offset(0.0, -10000.0));
    await tester.pump(const Duration(seconds: 2));
    expect(find.text('Hello A'), findsNothing);
    expect(find.text('Hello 1'), findsNothing);
    expect(find.text('Hello D'), findsOneWidget);
    expect(find.text('Hello 4'), findsOneWidget);
    await tester.pump(const Duration(seconds: 2));
    await tester.drag(find.text('Hello 4'), const Offset(0.0, 10000.0));
    await tester.pump(const Duration(seconds: 2));
    expect(find.text('Hello A'), findsOneWidget);
    expect(find.text('Hello 1'), findsOneWidget);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 4'), findsNothing);
    await tester.pump(const Duration(seconds: 2));
    await tester.drag(find.text('Hello 1'), const Offset(0.0, 10000.0));
    await tester.pump(const Duration(seconds: 2));
    expect(find.text('Hello A'), findsOneWidget);
    expect(find.text('Hello 1'), findsOneWidget);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 4'), findsNothing);
    await tester.pump(const Duration(seconds: 2));
    await tester.drag(find.text('Hello 1'), const Offset(0.0, -10000.0));
    await tester.pump(const Duration(seconds: 2));
    expect(find.text('Hello A'), findsOneWidget);
    expect(find.text('Hello 1'), findsNothing);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 4'), findsOneWidget);
  });
  testWidgets('LinkedScrollController - 2', (WidgetTester tester) async {
    await tester.pumpWidget(const Test());
    expect(find.text('Hello A'), findsOneWidget);
    expect(find.text('Hello B'), findsOneWidget);
    expect(find.text('Hello C'), findsNothing);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 1'), findsOneWidget);
    expect(find.text('Hello 2'), findsOneWidget);
    expect(find.text('Hello 3'), findsNothing);
    expect(find.text('Hello 4'), findsNothing);
    final TestGesture gestureTop = await tester.startGesture(const Offset(200.0, 150.0));
    final TestGesture gestureBottom = await tester.startGesture(const Offset(600.0, 450.0));
    await tester.pump(const Duration(seconds: 1));
    await gestureTop.moveBy(const Offset(0.0, -270.0));
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('Hello A'), findsNothing);
    expect(find.text('Hello B'), findsOneWidget);
    expect(find.text('Hello C'), findsOneWidget);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 1'), findsOneWidget);
    expect(find.text('Hello 2'), findsOneWidget);
    expect(find.text('Hello 3'), findsNothing);
    expect(find.text('Hello 4'), findsNothing);
    await gestureBottom.moveBy(const Offset(0.0, -270.0));
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('Hello A'), findsNothing);
    expect(find.text('Hello B'), findsOneWidget);
    expect(find.text('Hello C'), findsOneWidget);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 1'), findsNothing);
    expect(find.text('Hello 2'), findsOneWidget);
    expect(find.text('Hello 3'), findsOneWidget);
    expect(find.text('Hello 4'), findsNothing);
    await gestureTop.moveBy(const Offset(0.0, -270.0));
    await gestureBottom.moveBy(const Offset(0.0, -270.0));
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('Hello A'), findsNothing);
    expect(find.text('Hello B'), findsNothing);
    expect(find.text('Hello C'), findsOneWidget);
    expect(find.text('Hello D'), findsOneWidget);
    expect(find.text('Hello 1'), findsNothing);
    expect(find.text('Hello 2'), findsNothing);
    expect(find.text('Hello 3'), findsOneWidget);
    expect(find.text('Hello 4'), findsOneWidget);
    await gestureTop.moveBy(const Offset(0.0, 270.0));
    await gestureBottom.moveBy(const Offset(0.0, 270.0));
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('Hello A'), findsNothing);
    expect(find.text('Hello B'), findsOneWidget);
    expect(find.text('Hello C'), findsOneWidget);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 1'), findsNothing);
    expect(find.text('Hello 2'), findsOneWidget);
    expect(find.text('Hello 3'), findsOneWidget);
    expect(find.text('Hello 4'), findsNothing);
    await gestureBottom.moveBy(const Offset(0.0, 270.0));
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('Hello A'), findsNothing);
    expect(find.text('Hello B'), findsOneWidget);
    expect(find.text('Hello C'), findsOneWidget);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 1'), findsOneWidget);
    expect(find.text('Hello 2'), findsOneWidget);
    expect(find.text('Hello 3'), findsNothing);
    expect(find.text('Hello 4'), findsNothing);
    await gestureBottom.moveBy(const Offset(0.0, 50.0));
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('Hello A'), findsOneWidget);
    expect(find.text('Hello B'), findsOneWidget);
    expect(find.text('Hello C'), findsNothing);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 1'), findsOneWidget);
    expect(find.text('Hello 2'), findsOneWidget);
    expect(find.text('Hello 3'), findsNothing);
    expect(find.text('Hello 4'), findsNothing);
    await gestureBottom.moveBy(const Offset(0.0, 50.0));
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('Hello A'), findsOneWidget);
    expect(find.text('Hello B'), findsOneWidget);
    expect(find.text('Hello C'), findsNothing);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 1'), findsOneWidget);
    expect(find.text('Hello 2'), findsOneWidget);
    expect(find.text('Hello 3'), findsNothing);
    expect(find.text('Hello 4'), findsNothing);
    await gestureBottom.moveBy(const Offset(0.0, 50.0));
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('Hello A'), findsOneWidget);
    expect(find.text('Hello B'), findsOneWidget);
    expect(find.text('Hello C'), findsNothing);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 1'), findsOneWidget);
    expect(find.text('Hello 2'), findsOneWidget);
    expect(find.text('Hello 3'), findsNothing);
    expect(find.text('Hello 4'), findsNothing);
    await gestureTop.moveBy(const Offset(0.0, -270.0));
    expect(find.text('Hello A'), findsOneWidget);
    expect(find.text('Hello B'), findsOneWidget);
    expect(find.text('Hello C'), findsNothing);
    expect(find.text('Hello D'), findsNothing);
    expect(find.text('Hello 1'), findsOneWidget);
    expect(find.text('Hello 2'), findsOneWidget);
    expect(find.text('Hello 3'), findsNothing);
    expect(find.text('Hello 4'), findsNothing);
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 60));
  });
}
