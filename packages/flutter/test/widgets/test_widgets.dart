// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

const BoxDecoration kBoxDecorationA = BoxDecoration(color: Color(0xFFFF0000));

const BoxDecoration kBoxDecorationB = BoxDecoration(color: Color(0xFF00FF00));

const BoxDecoration kBoxDecorationC = BoxDecoration(color: Color(0xFF0000FF));

class TestBuildCounter extends StatelessWidget {
  const TestBuildCounter({super.key});

  static int buildCount = 0;

  @override
  Widget build(BuildContext context) {
    buildCount += 1;
    return const DecoratedBox(decoration: kBoxDecorationA);
  }
}

class FlipWidget extends StatefulWidget {
  const FlipWidget({super.key, required this.left, required this.right});

  final Widget left;
  final Widget right;

  @override
  FlipWidgetState createState() => FlipWidgetState();
}

class FlipWidgetState extends State<FlipWidget> {
  bool _showLeft = true;

  void flip() {
    setState(() {
      _showLeft = !_showLeft;
    });
  }

  @override
  Widget build(BuildContext context) {
    return _showLeft ? widget.left : widget.right;
  }
}

void flipStatefulWidget(WidgetTester tester, {bool skipOffstage = true}) {
  tester.state<FlipWidgetState>(find.byType(FlipWidget, skipOffstage: skipOffstage)).flip();
}
