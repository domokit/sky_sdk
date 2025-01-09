// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can call setState from didUpdateWidget', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(textDirection: TextDirection.ltr, child: WidgetUnderTest(text: 'hello')),
    );

    expect(find.text('hello'), findsOneWidget);
    expect(find.text('world'), findsNothing);
    final _WidgetUnderTestState state = tester.state<_WidgetUnderTestState>(
      find.byType(WidgetUnderTest),
    );
    expect(state.setStateCalled, 0);
    expect(state.didUpdateWidgetCalled, 0);

    await tester.pumpWidget(
      const Directionality(textDirection: TextDirection.ltr, child: WidgetUnderTest(text: 'world')),
    );

    expect(find.text('world'), findsOneWidget);
    expect(find.text('hello'), findsNothing);
    expect(state.setStateCalled, 1);
    expect(state.didUpdateWidgetCalled, 1);
  });
}

class WidgetUnderTest extends StatefulWidget {
  const WidgetUnderTest({super.key, required this.text});

  final String text;

  @override
  State<WidgetUnderTest> createState() => _WidgetUnderTestState();
}

class _WidgetUnderTestState extends State<WidgetUnderTest> {
  late String text = widget.text;

  int setStateCalled = 0;
  int didUpdateWidgetCalled = 0;

  @override
  void didUpdateWidget(WidgetUnderTest oldWidget) {
    super.didUpdateWidget(oldWidget);
    didUpdateWidgetCalled += 1;
    if (oldWidget.text != widget.text) {
      // This setState is load bearing for the test.
      setState(() {
        text = widget.text;
      });
    }
  }

  @override
  void setState(VoidCallback fn) {
    super.setState(fn);
    setStateCalled += 1;
  }

  @override
  Widget build(BuildContext context) {
    return Text(text);
  }
}
