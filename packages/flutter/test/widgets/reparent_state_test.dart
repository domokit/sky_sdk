// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

class StateMarker extends StatefulWidget {
  const StateMarker({super.key, this.child});

  final Widget? child;

  @override
  StateMarkerState createState() => StateMarkerState();
}

class StateMarkerState extends State<StateMarker> {
  String? marker;

  @override
  Widget build(BuildContext context) {
    return widget.child ?? Container();
  }
}

class DeactivateLogger extends StatefulWidget {
  const DeactivateLogger({required Key super.key, required this.log});

  final List<String> log;

  @override
  DeactivateLoggerState createState() => DeactivateLoggerState();
}

class DeactivateLoggerState extends State<DeactivateLogger> {
  @override
  void deactivate() {
    widget.log.add('deactivate');
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    widget.log.add('build');
    return Container();
  }
}

void main() {
  testWidgets('can reparent state', (WidgetTester tester) async {
    final GlobalKey left = GlobalKey();
    final GlobalKey right = GlobalKey();

    const StateMarker grandchild = StateMarker();
    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          ColoredBox(color: Colors.green, child: StateMarker(key: left)),
          ColoredBox(color: Colors.green, child: StateMarker(key: right, child: grandchild)),
        ],
      ),
    );

    final StateMarkerState leftState = left.currentState! as StateMarkerState;
    leftState.marker = 'left';
    final StateMarkerState rightState = right.currentState! as StateMarkerState;
    rightState.marker = 'right';

    final StateMarkerState grandchildState = tester.state(find.byWidget(grandchild));
    expect(grandchildState, isNotNull);
    grandchildState.marker = 'grandchild';

    const StateMarker newGrandchild = StateMarker();
    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          ColoredBox(color: Colors.green, child: StateMarker(key: right, child: newGrandchild)),
          ColoredBox(color: Colors.green, child: StateMarker(key: left)),
        ],
      ),
    );

    expect(left.currentState, equals(leftState));
    expect(leftState.marker, equals('left'));
    expect(right.currentState, equals(rightState));
    expect(rightState.marker, equals('right'));

    final StateMarkerState newGrandchildState = tester.state(find.byWidget(newGrandchild));
    expect(newGrandchildState, isNotNull);
    expect(newGrandchildState, equals(grandchildState));
    expect(newGrandchildState.marker, equals('grandchild'));

    await tester.pumpWidget(
      Center(
        child: ColoredBox(color: Colors.green, child: StateMarker(key: left, child: Container())),
      ),
    );

    expect(left.currentState, equals(leftState));
    expect(leftState.marker, equals('left'));
    expect(right.currentState, isNull);
  });

  testWidgets('can reparent state with multichild widgets', (WidgetTester tester) async {
    final GlobalKey left = GlobalKey();
    final GlobalKey right = GlobalKey();

    const StateMarker grandchild = StateMarker();
    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[StateMarker(key: left), StateMarker(key: right, child: grandchild)],
      ),
    );

    final StateMarkerState leftState = left.currentState! as StateMarkerState;
    leftState.marker = 'left';
    final StateMarkerState rightState = right.currentState! as StateMarkerState;
    rightState.marker = 'right';

    final StateMarkerState grandchildState = tester.state(find.byWidget(grandchild));
    expect(grandchildState, isNotNull);
    grandchildState.marker = 'grandchild';

    const StateMarker newGrandchild = StateMarker();
    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[StateMarker(key: right, child: newGrandchild), StateMarker(key: left)],
      ),
    );

    expect(left.currentState, equals(leftState));
    expect(leftState.marker, equals('left'));
    expect(right.currentState, equals(rightState));
    expect(rightState.marker, equals('right'));

    final StateMarkerState newGrandchildState = tester.state(find.byWidget(newGrandchild));
    expect(newGrandchildState, isNotNull);
    expect(newGrandchildState, equals(grandchildState));
    expect(newGrandchildState.marker, equals('grandchild'));

    await tester.pumpWidget(
      Center(
        child: ColoredBox(color: Colors.green, child: StateMarker(key: left, child: Container())),
      ),
    );

    expect(left.currentState, equals(leftState));
    expect(leftState.marker, equals('left'));
    expect(right.currentState, isNull);
  });

  testWidgets('can with scrollable list', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();

    await tester.pumpWidget(StateMarker(key: key));

    final StateMarkerState keyState = key.currentState! as StateMarkerState;
    keyState.marker = 'marked';

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          itemExtent: 100.0,
          children: <Widget>[
            SizedBox(key: const Key('container'), height: 100.0, child: StateMarker(key: key)),
          ],
        ),
      ),
    );

    expect(key.currentState, equals(keyState));
    expect(keyState.marker, equals('marked'));

    await tester.pumpWidget(StateMarker(key: key));

    expect(key.currentState, equals(keyState));
    expect(keyState.marker, equals('marked'));
  });

  testWidgets('Reparent during update children', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[StateMarker(key: key), const SizedBox(width: 100.0, height: 100.0)],
      ),
    );

    final StateMarkerState keyState = key.currentState! as StateMarkerState;
    keyState.marker = 'marked';

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[const SizedBox(width: 100.0, height: 100.0), StateMarker(key: key)],
      ),
    );

    expect(key.currentState, equals(keyState));
    expect(keyState.marker, equals('marked'));

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[StateMarker(key: key), const SizedBox(width: 100.0, height: 100.0)],
      ),
    );

    expect(key.currentState, equals(keyState));
    expect(keyState.marker, equals('marked'));
  });

  testWidgets('Reparent to child during update children', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          const SizedBox(width: 100.0, height: 100.0),
          StateMarker(key: key),
          const SizedBox(width: 100.0, height: 100.0),
        ],
      ),
    );

    final StateMarkerState keyState = key.currentState! as StateMarkerState;
    keyState.marker = 'marked';

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          SizedBox(width: 100.0, height: 100.0, child: StateMarker(key: key)),
          const SizedBox(width: 100.0, height: 100.0),
        ],
      ),
    );

    expect(key.currentState, equals(keyState));
    expect(keyState.marker, equals('marked'));

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          const SizedBox(width: 100.0, height: 100.0),
          StateMarker(key: key),
          const SizedBox(width: 100.0, height: 100.0),
        ],
      ),
    );

    expect(key.currentState, equals(keyState));
    expect(keyState.marker, equals('marked'));

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          const SizedBox(width: 100.0, height: 100.0),
          SizedBox(width: 100.0, height: 100.0, child: StateMarker(key: key)),
        ],
      ),
    );

    expect(key.currentState, equals(keyState));
    expect(keyState.marker, equals('marked'));

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          const SizedBox(width: 100.0, height: 100.0),
          StateMarker(key: key),
          const SizedBox(width: 100.0, height: 100.0),
        ],
      ),
    );

    expect(key.currentState, equals(keyState));
    expect(keyState.marker, equals('marked'));
  });

  testWidgets('Deactivate implies build', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    final List<String> log = <String>[];
    final DeactivateLogger logger = DeactivateLogger(key: key, log: log);

    await tester.pumpWidget(Container(key: UniqueKey(), child: logger));

    expect(log, equals(<String>['build']));

    await tester.pumpWidget(Container(key: UniqueKey(), child: logger));

    expect(log, equals(<String>['build', 'deactivate', 'build']));
    log.clear();

    await tester.pump();
    expect(log, isEmpty);
  });

  testWidgets('Reparenting with multiple moves', (WidgetTester tester) async {
    final GlobalKey key1 = GlobalKey();
    final GlobalKey key2 = GlobalKey();
    final GlobalKey key3 = GlobalKey();

    await tester.pumpWidget(
      Row(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          StateMarker(
            key: key1,
            child: StateMarker(
              key: key2,
              child: StateMarker(key: key3, child: StateMarker(child: Container(width: 100.0))),
            ),
          ),
        ],
      ),
    );

    await tester.pumpWidget(
      Row(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          StateMarker(key: key2, child: StateMarker(child: Container(width: 100.0))),
          StateMarker(
            key: key1,
            child: StateMarker(key: key3, child: StateMarker(child: Container(width: 100.0))),
          ),
        ],
      ),
    );
  });
}
