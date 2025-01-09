// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

late ChangerState changer;

class Changer extends StatefulWidget {
  const Changer(this.child, {super.key});

  final Widget child;

  @override
  ChangerState createState() => ChangerState();
}

class ChangerState extends State<Changer> {
  bool _state = false;

  @override
  void initState() {
    super.initState();
    changer = this;
  }

  void test() {
    setState(() {
      _state = true;
    });
  }

  @override
  Widget build(BuildContext context) => _state ? Wrapper(widget.child) : widget.child;
}

class Wrapper extends StatelessWidget {
  const Wrapper(this.child, {super.key});

  final Widget child;

  @override
  Widget build(BuildContext context) => child;
}

class Leaf extends StatefulWidget {
  const Leaf({super.key});
  @override
  LeafState createState() => LeafState();
}

class LeafState extends State<Leaf> {
  @override
  Widget build(BuildContext context) => const Text('leaf', textDirection: TextDirection.ltr);
}

void main() {
  testWidgets('three-way setState() smoke test', (WidgetTester tester) async {
    await tester.pumpWidget(const Changer(Wrapper(Leaf())));
    await tester.pumpWidget(const Changer(Wrapper(Leaf())));
    changer.test();
    await tester.pump();
  });
}
