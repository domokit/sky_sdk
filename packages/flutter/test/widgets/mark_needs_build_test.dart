// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets(
    'setState can be called from build, initState, didChangeDependencies, and didUpdateWidget',
    (WidgetTester tester) async {
      // Initial build.
      await tester.pumpWidget(
        const Directionality(textDirection: TextDirection.ltr, child: TestWidget(value: 1)),
      );
      final _TestWidgetState state = tester.state(find.byType(TestWidget));
      expect(state.calledDuringBuild, 1);
      expect(state.calledDuringInitState, 1);
      expect(state.calledDuringDidChangeDependencies, 1);
      expect(state.calledDuringDidUpdateWidget, 0);

      // Update Widget.
      late Widget child;
      await tester.pumpWidget(
        Directionality(textDirection: TextDirection.ltr, child: child = const TestWidget(value: 2)),
      );
      expect(state.calledDuringBuild, 2); // Increased.
      expect(state.calledDuringInitState, 1);
      expect(state.calledDuringDidChangeDependencies, 1);
      expect(state.calledDuringDidUpdateWidget, 1); // Increased.

      // Build after state is dirty.
      state.markNeedsBuild();
      await tester.pump();
      expect(state.calledDuringBuild, 3); // Increased.
      expect(state.calledDuringInitState, 1);
      expect(state.calledDuringDidChangeDependencies, 1);
      expect(state.calledDuringDidUpdateWidget, 1);

      // Change dependency.
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.rtl, // Changed.
          child: child,
        ),
      );
      expect(state.calledDuringBuild, 4); // Increased.
      expect(state.calledDuringInitState, 1);
      expect(state.calledDuringDidChangeDependencies, 2); // Increased.
      expect(state.calledDuringDidUpdateWidget, 1);
    },
  );
}

class TestWidget extends StatefulWidget {
  const TestWidget({super.key, required this.value});

  final int value;

  @override
  State<TestWidget> createState() => _TestWidgetState();
}

class _TestWidgetState extends State<TestWidget> {
  int calledDuringBuild = 0;
  int calledDuringInitState = 0;
  int calledDuringDidChangeDependencies = 0;
  int calledDuringDidUpdateWidget = 0;

  void markNeedsBuild() {
    setState(() {
      // Intentionally left empty.
    });
  }

  @override
  void initState() {
    super.initState();
    setState(() {
      calledDuringInitState++;
    });
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    setState(() {
      calledDuringDidChangeDependencies++;
    });
  }

  @override
  void didUpdateWidget(TestWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    setState(() {
      calledDuringDidUpdateWidget++;
    });
  }

  @override
  Widget build(BuildContext context) {
    setState(() {
      calledDuringBuild++;
    });
    return SizedBox.expand(child: Text('${widget.value}: ${Directionality.of(context)}'));
  }
}
