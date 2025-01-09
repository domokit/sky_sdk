// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('restartAndRestore', (WidgetTester tester) async {
    await tester.pumpWidget(
      const RootRestorationScope(
        restorationId: 'root-child',
        child: _RestorableWidget(restorationId: 'restorable-widget'),
      ),
    );

    final _RestorableWidgetState state = tester.state(find.byType(_RestorableWidget));
    expect(find.text('Hello World 100'), findsOneWidget);
    expect(state.doubleValue, 1.0);

    state.setValues('Guten Morgen', 200, 33.4);
    await tester.pump();

    expect(find.text('Guten Morgen 200'), findsOneWidget);
    expect(state.doubleValue, 33.4);

    await tester.restartAndRestore();

    expect(find.text('Guten Morgen 200'), findsOneWidget);
    expect(find.text('Hello World 100'), findsNothing);
    final _RestorableWidgetState restoredState = tester.state(find.byType(_RestorableWidget));
    expect(restoredState, isNot(same(state)));
    expect(restoredState.doubleValue, 1.0);
  });

  testWidgets('restore from previous restoration data', (WidgetTester tester) async {
    await tester.pumpWidget(
      const RootRestorationScope(
        restorationId: 'root-child',
        child: _RestorableWidget(restorationId: 'restorable-widget'),
      ),
    );

    final _RestorableWidgetState state = tester.state(find.byType(_RestorableWidget));
    expect(find.text('Hello World 100'), findsOneWidget);
    expect(state.doubleValue, 1.0);

    state.setValues('Guten Morgen', 200, 33.4);
    await tester.pump();

    expect(find.text('Guten Morgen 200'), findsOneWidget);
    expect(state.doubleValue, 33.4);

    final TestRestorationData data = await tester.getRestorationData();

    state.setValues('See you later!', 400, 123.5);
    await tester.pump();

    expect(find.text('See you later! 400'), findsOneWidget);
    expect(state.doubleValue, 123.5);

    await tester.restoreFrom(data);

    expect(tester.state(find.byType(_RestorableWidget)), same(state));
    expect(find.text('Guten Morgen 200'), findsOneWidget);
    expect(state.doubleValue, 123.5);
  });
}

class _RestorableWidget extends StatefulWidget {
  const _RestorableWidget({this.restorationId});

  final String? restorationId;

  @override
  State<_RestorableWidget> createState() => _RestorableWidgetState();
}

class _RestorableWidgetState extends State<_RestorableWidget> with RestorationMixin {
  final RestorableString stringValue = RestorableString('Hello World');
  final RestorableInt intValue = RestorableInt(100);

  double doubleValue = 1.0; // Not restorable.

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(stringValue, 'string');
    registerForRestoration(intValue, 'int');
  }

  void setValues(String s, int i, double d) {
    setState(() {
      stringValue.value = s;
      intValue.value = i;
      doubleValue = d;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Text('${stringValue.value} ${intValue.value}', textDirection: TextDirection.ltr);
  }

  @override
  String? get restorationId => widget.restorationId;
}
