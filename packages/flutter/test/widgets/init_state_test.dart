// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

List<String> ancestors = <String>[];

class TestWidget extends StatefulWidget {
  const TestWidget({super.key});
  @override
  TestWidgetState createState() => TestWidgetState();
}

class TestWidgetState extends State<TestWidget> {
  @override
  void initState() {
    super.initState();
    context.visitAncestorElements((Element element) {
      ancestors.add(element.widget.runtimeType.toString());
      return true;
    });
  }

  @override
  Widget build(BuildContext context) => Container();
}

void main() {
  testWidgets('initState() is called when we are in the tree', (WidgetTester tester) async {
    await tester.pumpWidget(const Parent(child: TestWidget()));
    expect(ancestors, containsAllInOrder(<String>['Parent', 'View', 'RootWidget']));
  });
}

class Parent extends StatelessWidget {
  const Parent({super.key, required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) => child;
}
