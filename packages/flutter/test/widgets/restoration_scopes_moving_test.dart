// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('widget moves scopes during restore', (WidgetTester tester) async {
    await tester.pumpWidget(
      const RootRestorationScope(
        restorationId: 'root',
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: TestWidgetWithCounterChild(),
        ),
      ),
    );

    expect(
      tester
          .state<TestWidgetWithCounterChildState>(find.byType(TestWidgetWithCounterChild))
          .restoreChild,
      true,
    );
    expect(find.text('Counter: 0'), findsOneWidget);
    await tester.tap(find.text('Counter: 0'));
    await tester.pump();
    expect(find.text('Counter: 1'), findsOneWidget);

    final TestRestorationData dataWithChild = await tester.getRestorationData();

    tester
        .state<TestWidgetWithCounterChildState>(find.byType(TestWidgetWithCounterChild))
        .restoreChild = false;
    await tester.pump();
    expect(
      tester
          .state<TestWidgetWithCounterChildState>(find.byType(TestWidgetWithCounterChild))
          .restoreChild,
      false,
    );

    await tester.tap(find.text('Counter: 1'));
    await tester.pump();
    expect(find.text('Counter: 2'), findsOneWidget);

    final TestRestorationData dataWithoutChild = await tester.getRestorationData();

    // Child moves from outside to inside scope.
    await tester.restoreFrom(dataWithChild);
    expect(find.text('Counter: 1'), findsOneWidget);

    await tester.tap(find.text('Counter: 1'));
    await tester.pump();
    expect(find.text('Counter: 2'), findsOneWidget);

    // Child stays inside scope.
    await tester.restoreFrom(dataWithChild);
    expect(find.text('Counter: 1'), findsOneWidget);

    await tester.tap(find.text('Counter: 1'));
    await tester.tap(find.text('Counter: 1'));
    await tester.tap(find.text('Counter: 1'));
    await tester.tap(find.text('Counter: 1'));
    await tester.tap(find.text('Counter: 1'));
    await tester.pump();
    expect(find.text('Counter: 6'), findsOneWidget);

    // Child moves from inside to outside scope.
    await tester.restoreFrom(dataWithoutChild);
    expect(find.text('Counter: 6'), findsOneWidget);

    await tester.tap(find.text('Counter: 6'));
    await tester.pump();
    expect(find.text('Counter: 7'), findsOneWidget);

    // Child stays outside scope.
    await tester.restoreFrom(dataWithoutChild);
    expect(find.text('Counter: 7'), findsOneWidget);

    expect(
      tester
          .state<TestWidgetWithCounterChildState>(find.byType(TestWidgetWithCounterChild))
          .toggleCount,
      0,
    );
  });

  testWidgets('restoration is turned on later', (WidgetTester tester) async {
    tester.binding.restorationManager.disableRestoration();
    await tester.pumpWidget(
      const RootRestorationScope(
        restorationId: 'root-child',
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: TestWidget(restorationId: 'foo'),
        ),
      ),
    );

    final TestWidgetState state = tester.state<TestWidgetState>(find.byType(TestWidget));
    expect(find.text('hello'), findsOneWidget);
    expect(state.buckets.single, isNull);
    expect(state.flags.single, isTrue);
    expect(state.bucket, isNull);

    state.buckets.clear();
    state.flags.clear();

    await tester.restoreFrom(TestRestorationData.empty);

    await tester.pumpWidget(
      const RootRestorationScope(
        restorationId: 'root-child',
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: TestWidget(restorationId: 'foo'),
        ),
      ),
    );

    expect(find.text('hello'), findsOneWidget);
    expect(state.buckets.single, isNull);
    expect(state.flags.single, isFalse);
    expect(state.bucket, isNotNull);

    expect(state.toggleCount, 0);
  });
}

class TestWidgetWithCounterChild extends StatefulWidget {
  const TestWidgetWithCounterChild({super.key});

  @override
  State<TestWidgetWithCounterChild> createState() => TestWidgetWithCounterChildState();
}

class TestWidgetWithCounterChildState extends State<TestWidgetWithCounterChild>
    with RestorationMixin {
  final RestorableBool childRestorationEnabled = RestorableBool(true);

  int toggleCount = 0;

  @override
  void didToggleBucket(RestorationBucket? oldBucket) {
    super.didToggleBucket(oldBucket);
    toggleCount++;
  }

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(childRestorationEnabled, 'childRestorationEnabled');
  }

  bool get restoreChild => childRestorationEnabled.value;
  set restoreChild(bool value) {
    if (value == childRestorationEnabled.value) {
      return;
    }
    setState(() {
      childRestorationEnabled.value = value;
    });
  }

  @override
  void dispose() {
    childRestorationEnabled.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Counter(restorationId: restoreChild ? 'counter' : null);
  }

  @override
  String get restorationId => 'foo';
}

class Counter extends StatefulWidget {
  const Counter({super.key, this.restorationId});

  final String? restorationId;

  @override
  State<Counter> createState() => CounterState();
}

class CounterState extends State<Counter> with RestorationMixin {
  final RestorableInt count = RestorableInt(0);

  @override
  String? get restorationId => widget.restorationId;

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(count, 'counter');
  }

  @override
  void dispose() {
    count.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return OutlinedButton(
      onPressed: () {
        setState(() {
          count.value++;
        });
      },
      child: Text('Counter: ${count.value}'),
    );
  }
}

class TestWidget extends StatefulWidget {
  const TestWidget({super.key, required this.restorationId});

  final String? restorationId;

  @override
  State<TestWidget> createState() => TestWidgetState();
}

class TestWidgetState extends State<TestWidget> with RestorationMixin {
  List<RestorationBucket?> buckets = <RestorationBucket?>[];
  List<bool> flags = <bool>[];

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    buckets.add(oldBucket);
    flags.add(initialRestore);
  }

  int toggleCount = 0;

  @override
  void didToggleBucket(RestorationBucket? oldBucket) {
    super.didToggleBucket(oldBucket);
    toggleCount++;
  }

  @override
  String? get restorationId => widget.restorationId;

  @override
  Widget build(BuildContext context) {
    return const Text('hello');
  }
}
