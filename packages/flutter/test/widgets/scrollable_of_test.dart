// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class ScrollPositionListener extends StatefulWidget {
  const ScrollPositionListener({super.key, required this.child, required this.log});

  final Widget child;
  final ValueChanged<String> log;

  @override
  State<ScrollPositionListener> createState() => _ScrollPositionListenerState();
}

class _ScrollPositionListenerState extends State<ScrollPositionListener> {
  ScrollPosition? _position;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _position?.removeListener(listener);
    _position = Scrollable.maybeOf(context)?.position;
    _position?.addListener(listener);
    widget.log('didChangeDependencies ${_position?.pixels.toStringAsFixed(1)}');
  }

  @override
  void dispose() {
    _position?.removeListener(listener);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) => widget.child;

  void listener() {
    widget.log('listener ${_position?.pixels.toStringAsFixed(1)}');
  }
}

class TestScrollController extends ScrollController {
  TestScrollController({required this.deferLoading});

  final bool deferLoading;

  @override
  ScrollPosition createScrollPosition(
    ScrollPhysics physics,
    ScrollContext context,
    ScrollPosition? oldPosition,
  ) {
    return TestScrollPosition(
      physics: physics,
      context: context,
      oldPosition: oldPosition,
      deferLoading: deferLoading,
    );
  }
}

class TestScrollPosition extends ScrollPositionWithSingleContext {
  TestScrollPosition({
    required super.physics,
    required super.context,
    super.oldPosition,
    required this.deferLoading,
  });

  final bool deferLoading;

  @override
  bool recommendDeferredLoading(BuildContext context) => deferLoading;
}

class TestScrollable extends StatefulWidget {
  const TestScrollable({super.key, required this.child});

  final Widget child;

  @override
  State<StatefulWidget> createState() => TestScrollableState();
}

class TestScrollableState extends State<TestScrollable> {
  int dependenciesChanged = 0;

  @override
  void didChangeDependencies() {
    dependenciesChanged += 1;
    super.didChangeDependencies();
  }

  @override
  Widget build(BuildContext context) {
    return widget.child;
  }
}

class TestChild extends StatefulWidget {
  const TestChild({super.key});

  @override
  State<TestChild> createState() => TestChildState();
}

class TestChildState extends State<TestChild> {
  int dependenciesChanged = 0;
  late ScrollableState scrollable;

  @override
  void didChangeDependencies() {
    dependenciesChanged += 1;
    scrollable = Scrollable.of(context, axis: Axis.horizontal);
    super.didChangeDependencies();
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox.square(dimension: 1000, child: Text(scrollable.axisDirection.toString()));
  }
}

void main() {
  testWidgets('Scrollable.of() dependent rebuilds when Scrollable position changes', (
    WidgetTester tester,
  ) async {
    late String logValue;
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    // Changing the SingleChildScrollView's physics causes the
    // ScrollController's ScrollPosition to be rebuilt.

    Widget buildFrame(ScrollPhysics? physics) {
      return SingleChildScrollView(
        controller: controller,
        physics: physics,
        child: ScrollPositionListener(
          log: (String s) {
            logValue = s;
          },
          child: const SizedBox(height: 400.0),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(null));
    expect(logValue, 'didChangeDependencies 0.0');

    controller.jumpTo(100.0);
    expect(logValue, 'listener 100.0');

    await tester.pumpWidget(buildFrame(const ClampingScrollPhysics()));
    expect(logValue, 'didChangeDependencies 100.0');

    controller.jumpTo(200.0);
    expect(logValue, 'listener 200.0');

    controller.jumpTo(300.0);
    expect(logValue, 'listener 300.0');

    await tester.pumpWidget(buildFrame(const BouncingScrollPhysics()));
    expect(logValue, 'didChangeDependencies 300.0');

    controller.jumpTo(400.0);
    expect(logValue, 'listener 400.0');
  });

  testWidgets('Scrollable.of() is possible using ScrollNotification context', (
    WidgetTester tester,
  ) async {
    late ScrollNotification notification;

    await tester.pumpWidget(
      NotificationListener<ScrollNotification>(
        onNotification: (ScrollNotification value) {
          notification = value;
          return false;
        },
        child: const SingleChildScrollView(child: SizedBox(height: 1200.0)),
      ),
    );

    final TestGesture gesture = await tester.startGesture(const Offset(100.0, 100.0));
    await tester.pump(const Duration(seconds: 1));

    final StatefulElement scrollableElement =
        find.byType(Scrollable).evaluate().first as StatefulElement;
    expect(Scrollable.of(notification.context!), equals(scrollableElement.state));

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Static Scrollable methods can target a specific axis', (WidgetTester tester) async {
    final TestScrollController horizontalController = TestScrollController(deferLoading: true);
    addTearDown(horizontalController.dispose);
    final TestScrollController verticalController = TestScrollController(deferLoading: false);
    addTearDown(verticalController.dispose);
    late final AxisDirection foundAxisDirection;
    late final bool foundRecommendation;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SingleChildScrollView(
          scrollDirection: Axis.horizontal,
          controller: horizontalController,
          child: SingleChildScrollView(
            controller: verticalController,
            child: Builder(
              builder: (BuildContext context) {
                foundAxisDirection = Scrollable.of(context, axis: Axis.horizontal).axisDirection;
                foundRecommendation = Scrollable.recommendDeferredLoadingForContext(
                  context,
                  axis: Axis.horizontal,
                );
                return const SizedBox(height: 1200.0, width: 1200.0);
              },
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(foundAxisDirection, AxisDirection.right);
    expect(foundRecommendation, isTrue);
  });

  testWidgets('Axis targeting scrollables establishes the correct dependencies', (
    WidgetTester tester,
  ) async {
    final GlobalKey<TestScrollableState> verticalKey = GlobalKey<TestScrollableState>();
    final GlobalKey<TestChildState> childKey = GlobalKey<TestChildState>();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SingleChildScrollView(
          scrollDirection: Axis.horizontal,
          child: TestScrollable(key: verticalKey, child: TestChild(key: childKey)),
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(verticalKey.currentState!.dependenciesChanged, 1);
    expect(childKey.currentState!.dependenciesChanged, 1);

    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    // Change the horizontal ScrollView, adding a controller
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SingleChildScrollView(
          scrollDirection: Axis.horizontal,
          controller: controller,
          child: TestScrollable(key: verticalKey, child: TestChild(key: childKey)),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(verticalKey.currentState!.dependenciesChanged, 1);
    expect(childKey.currentState!.dependenciesChanged, 2);
  });
}
