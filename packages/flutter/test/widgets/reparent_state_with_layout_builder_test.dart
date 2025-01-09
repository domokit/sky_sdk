// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

// This is a regression test for https://github.com/flutter/flutter/issues/5840.

class Bar extends StatefulWidget {
  const Bar({super.key});
  @override
  BarState createState() => BarState();
}

class BarState extends State<Bar> {
  final GlobalKey _fooKey = GlobalKey();

  bool _mode = false;

  void trigger() {
    setState(() {
      _mode = !_mode;
    });
  }

  @override
  Widget build(BuildContext context) {
    if (_mode) {
      return SizedBox(
        child: LayoutBuilder(
          builder: (BuildContext context, BoxConstraints constraints) {
            return StatefulCreationCounter(key: _fooKey);
          },
        ),
      );
    } else {
      return LayoutBuilder(
        builder: (BuildContext context, BoxConstraints constraints) {
          return StatefulCreationCounter(key: _fooKey);
        },
      );
    }
  }
}

class StatefulCreationCounter extends StatefulWidget {
  const StatefulCreationCounter({super.key});

  @override
  StatefulCreationCounterState createState() => StatefulCreationCounterState();
}

class StatefulCreationCounterState extends State<StatefulCreationCounter> {
  static int creationCount = 0;

  @override
  void initState() {
    super.initState();
    creationCount += 1;
  }

  @override
  Widget build(BuildContext context) => Container();
}

void main() {
  testWidgets('reparent state with layout builder', (WidgetTester tester) async {
    expect(StatefulCreationCounterState.creationCount, 0);
    await tester.pumpWidget(const Bar());
    expect(StatefulCreationCounterState.creationCount, 1);
    final BarState s = tester.state<BarState>(find.byType(Bar));
    s.trigger();
    await tester.pump();
    expect(StatefulCreationCounterState.creationCount, 1);
  });

  testWidgets('Clean then reparent with dependencies', (WidgetTester tester) async {
    int layoutBuilderBuildCount = 0;

    late StateSetter keyedSetState;
    late StateSetter layoutBuilderSetState;
    late StateSetter childSetState;

    final GlobalKey key = GlobalKey();
    final Widget keyedWidget = StatefulBuilder(
      key: key,
      builder: (BuildContext context, StateSetter setState) {
        keyedSetState = setState;
        MediaQuery.of(context);
        return Container();
      },
    );

    Widget layoutBuilderChild = keyedWidget;
    Widget deepChild = Container();

    await tester.pumpWidget(
      MediaQuery(
        data: MediaQueryData.fromView(tester.view),
        child: Column(
          children: <Widget>[
            StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                layoutBuilderSetState = setState;
                return LayoutBuilder(
                  builder: (BuildContext context, BoxConstraints constraints) {
                    layoutBuilderBuildCount += 1;
                    return layoutBuilderChild; // initially keyedWidget above, but then a new Container
                  },
                );
              },
            ),
            ColoredBox(
              color: Colors.green,
              child: ColoredBox(
                color: Colors.green,
                child: ColoredBox(
                  color: Colors.green,
                  child: ColoredBox(
                    color: Colors.green,
                    child: ColoredBox(
                      color: Colors.green,
                      child: ColoredBox(
                        color: Colors.green,
                        child: StatefulBuilder(
                          builder: (BuildContext context, StateSetter setState) {
                            childSetState = setState;
                            return deepChild; // initially a Container, but then the keyedWidget above
                          },
                        ),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
    expect(layoutBuilderBuildCount, 1);

    keyedSetState(() {
      /* Change nothing but add the element to the dirty list. */
    });

    childSetState(() {
      // The deep child builds in the initial build phase. It takes the child
      // from the LayoutBuilder before the LayoutBuilder has a chance to build.
      deepChild = keyedWidget;
    });

    layoutBuilderSetState(() {
      // The layout builder will build in a separate build scope. This delays
      // the removal of the keyed child until this build scope.
      layoutBuilderChild = Container();
    });

    // The essential part of this test is that this call to pump doesn't throw.
    await tester.pump();
    expect(layoutBuilderBuildCount, 2);
  });
}
