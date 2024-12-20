// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class Inside extends StatefulWidget {
  const Inside({super.key});
  @override
  InsideState createState() => InsideState();
}

class InsideState extends State<Inside> {
  @override
  Widget build(BuildContext context) {
    return Listener(
      onPointerDown: _handlePointerDown,
      child: const Text('INSIDE', textDirection: TextDirection.ltr),
    );
  }

  void _handlePointerDown(PointerDownEvent event) {
    setState(() {});
  }
}

class Middle extends StatefulWidget {
  const Middle({super.key, this.child});

  final Inside? child;

  @override
  MiddleState createState() => MiddleState();
}

class MiddleState extends State<Middle> {
  @override
  Widget build(BuildContext context) {
    return Listener(onPointerDown: _handlePointerDown, child: widget.child);
  }

  void _handlePointerDown(PointerDownEvent event) {
    setState(() {});
  }
}

class Outside extends StatefulWidget {
  const Outside({super.key});
  @override
  OutsideState createState() => OutsideState();
}

class OutsideState extends State<Outside> {
  @override
  Widget build(BuildContext context) {
    return const Middle(child: Inside());
  }
}

void main() {
  testWidgets('setState() smoke test', (WidgetTester tester) async {
    await tester.pumpWidget(const Outside());
    final Offset location = tester.getCenter(find.text('INSIDE'));
    final TestGesture gesture = await tester.startGesture(location);
    await tester.pump();
    await gesture.up();
    await tester.pump();
  });
}
