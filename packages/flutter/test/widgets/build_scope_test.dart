// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'test_widgets.dart';

class ProbeWidget extends StatefulWidget {
  const ProbeWidget({super.key});
  @override
  ProbeWidgetState createState() => ProbeWidgetState();
}

class ProbeWidgetState extends State<ProbeWidget> {
  static int buildCount = 0;

  @override
  void initState() {
    super.initState();
    setState(() {});
  }

  @override
  void didUpdateWidget(ProbeWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    setState(() {});
    buildCount++;
    return Container();
  }
}

class BadWidget extends StatelessWidget {
  const BadWidget(this.parentState, {super.key});

  final BadWidgetParentState parentState;

  @override
  Widget build(BuildContext context) {
    parentState._markNeedsBuild();
    return Container();
  }
}

class BadWidgetParent extends StatefulWidget {
  const BadWidgetParent({super.key});
  @override
  BadWidgetParentState createState() => BadWidgetParentState();
}

class BadWidgetParentState extends State<BadWidgetParent> {
  void _markNeedsBuild() {
    setState(() {
      // Our state didn't really change, but we're doing something pathological
      // here to trigger an interesting scenario to test.
    });
  }

  @override
  Widget build(BuildContext context) {
    return BadWidget(this);
  }
}

class BadDisposeWidget extends StatefulWidget {
  const BadDisposeWidget({super.key});
  @override
  BadDisposeWidgetState createState() => BadDisposeWidgetState();
}

class BadDisposeWidgetState extends State<BadDisposeWidget> {
  @override
  Widget build(BuildContext context) {
    return Container();
  }

  @override
  void dispose() {
    setState(() {
      /* This is invalid behavior. */
    });
    super.dispose();
  }
}

class StatefulWrapper extends StatefulWidget {
  const StatefulWrapper({super.key, required this.child});

  final Widget child;

  @override
  StatefulWrapperState createState() => StatefulWrapperState();
}

class StatefulWrapperState extends State<StatefulWrapper> {
  void trigger() {
    setState(() {
      built = null;
    });
  }

  int? built;
  late int oldBuilt;

  static int buildId = 0;

  @override
  Widget build(BuildContext context) {
    buildId += 1;
    built = buildId;
    return widget.child;
  }
}

class Wrapper extends StatelessWidget {
  const Wrapper({super.key, required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) {
    return child;
  }
}

void main() {
  testWidgets('Legal times for setState', (WidgetTester tester) async {
    final GlobalKey flipKey = GlobalKey();
    expect(ProbeWidgetState.buildCount, equals(0));
    await tester.pumpWidget(const ProbeWidget(key: Key('a')));
    expect(ProbeWidgetState.buildCount, equals(1));
    await tester.pumpWidget(const ProbeWidget(key: Key('b')));
    expect(ProbeWidgetState.buildCount, equals(2));
    await tester.pumpWidget(
      FlipWidget(key: flipKey, left: Container(), right: const ProbeWidget(key: Key('c'))),
    );
    expect(ProbeWidgetState.buildCount, equals(2));
    final FlipWidgetState flipState1 = flipKey.currentState! as FlipWidgetState;
    flipState1.flip();
    await tester.pump();
    expect(ProbeWidgetState.buildCount, equals(3));
    final FlipWidgetState flipState2 = flipKey.currentState! as FlipWidgetState;
    flipState2.flip();
    await tester.pump();
    expect(ProbeWidgetState.buildCount, equals(3));
    await tester.pumpWidget(Container());
    expect(ProbeWidgetState.buildCount, equals(3));
  });

  testWidgets('Setting parent state during build is forbidden', (WidgetTester tester) async {
    await tester.pumpWidget(const BadWidgetParent());
    expect(tester.takeException(), isFlutterError);
    await tester.pumpWidget(Container());
  });

  testWidgets(
    'Setting state during dispose is forbidden',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      await tester.pumpWidget(const BadDisposeWidget());
      expect(tester.takeException(), isNull);
      await tester.pumpWidget(Container());
      expect(tester.takeException(), isNotNull);
    },
  );

  testWidgets('Dirty element list sort order', (WidgetTester tester) async {
    final GlobalKey key1 = GlobalKey(debugLabel: 'key1');
    final GlobalKey key2 = GlobalKey(debugLabel: 'key2');

    bool didMiddle = false;
    late Widget middle;
    final List<StateSetter> setStates = <StateSetter>[];
    Widget builder(BuildContext context, StateSetter setState) {
      setStates.add(setState);
      final bool returnMiddle = !didMiddle;
      didMiddle = true;
      return Wrapper(
        child: Wrapper(child: StatefulWrapper(child: returnMiddle ? middle : Container())),
      );
    }

    final Widget part1 = Wrapper(
      child: KeyedSubtree(key: key1, child: StatefulBuilder(builder: builder)),
    );
    final Widget part2 = Wrapper(
      child: KeyedSubtree(key: key2, child: StatefulBuilder(builder: builder)),
    );

    middle = part2;
    await tester.pumpWidget(part1);

    for (final StatefulWrapperState state in tester.stateList<StatefulWrapperState>(
      find.byType(StatefulWrapper),
    )) {
      expect(state.built, isNotNull);
      state.oldBuilt = state.built!;
      state.trigger();
    }
    for (final StateSetter setState in setStates) {
      setState(() {});
    }

    StatefulWrapperState.buildId = 0;
    middle = part1;
    didMiddle = false;
    await tester.pumpWidget(part2);

    for (final StatefulWrapperState state in tester.stateList<StatefulWrapperState>(
      find.byType(StatefulWrapper),
    )) {
      expect(state.built, isNotNull);
      expect(state.built, isNot(equals(state.oldBuilt)));
    }
  });
}
