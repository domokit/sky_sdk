// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

typedef TestCallback = void Function(BuildContext context);

class TestWidget extends StatefulWidget {
  const TestWidget(this.callback, {super.key});

  final TestCallback callback;

  @override
  TestWidgetState createState() => TestWidgetState();
}

class TestWidgetState extends State<TestWidget> {
  @override
  void dispose() {
    widget.callback(context);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) => const Text('test', textDirection: TextDirection.ltr);
}

void main() {
  testWidgets(
    'dependOnInheritedWidgetOfExactType() called from dispose() throws error',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      bool disposeCalled = false;
      await tester.pumpWidget(
        TestWidget((BuildContext context) {
          disposeCalled = true;
          context.dependOnInheritedWidgetOfExactType<InheritedWidget>();
        }),
      );
      await tester.pumpWidget(Container());
      expect(disposeCalled, isTrue);
      expect(tester.takeException(), isFlutterError);
    },
  );

  testWidgets(
    'getElementForInheritedWidgetOfExactType() called from dispose() throws error',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      bool disposeCalled = false;
      await tester.pumpWidget(
        TestWidget((BuildContext context) {
          disposeCalled = true;
          context.getElementForInheritedWidgetOfExactType<InheritedWidget>();
        }),
      );
      await tester.pumpWidget(Container());
      expect(disposeCalled, isTrue);
      expect(tester.takeException(), isFlutterError);
    },
  );

  testWidgets(
    'findAncestorWidgetOfExactType() called from dispose() throws error',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      bool disposeCalled = false;
      await tester.pumpWidget(
        TestWidget((BuildContext context) {
          disposeCalled = true;
          context.findAncestorWidgetOfExactType<Container>();
        }),
      );
      await tester.pumpWidget(Container());
      expect(disposeCalled, isTrue);
      expect(tester.takeException(), isFlutterError);
    },
  );

  testWidgets(
    'findAncestorStateOfType() called from dispose() throws error',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      bool disposeCalled = false;
      await tester.pumpWidget(
        TestWidget((BuildContext context) {
          disposeCalled = true;
          context.findAncestorStateOfType<State>();
        }),
      );
      await tester.pumpWidget(Container());
      expect(disposeCalled, isTrue);
      expect(tester.takeException(), isFlutterError);
    },
  );

  testWidgets(
    'findAncestorRenderObjectOfType() called from dispose() throws error',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      bool disposeCalled = false;
      await tester.pumpWidget(
        TestWidget((BuildContext context) {
          disposeCalled = true;
          context.findAncestorRenderObjectOfType<RenderObject>();
        }),
      );
      await tester.pumpWidget(Container());
      expect(disposeCalled, isTrue);
      expect(tester.takeException(), isFlutterError);
    },
  );

  testWidgets(
    'visitAncestorElements() called from dispose() throws error',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      bool disposeCalled = false;
      await tester.pumpWidget(
        TestWidget((BuildContext context) {
          disposeCalled = true;
          context.visitAncestorElements((Element element) => true);
        }),
      );
      await tester.pumpWidget(Container());
      expect(disposeCalled, isTrue);
      expect(tester.takeException(), isFlutterError);
    },
  );

  testWidgets('dispose() method does not unconditionally throw an error', (
    WidgetTester tester,
  ) async {
    bool disposeCalled = false;
    await tester.pumpWidget(
      TestWidget((BuildContext context) {
        disposeCalled = true;
      }),
    );
    await tester.pumpWidget(Container());
    expect(disposeCalled, isTrue);
  });
}
