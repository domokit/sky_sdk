// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

const String text = 'Hello World! How are you? Life is good!';
const String alternativeText = 'Everything is awesome!!';

void main() {
  testWidgets('TextField restoration', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(restorationScopeId: 'app', home: TestWidget()));

    await restoreAndVerify(tester);
  });

  testWidgets('TextField restoration with external controller', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(restorationScopeId: 'root', home: TestWidget(useExternal: true)),
    );

    await restoreAndVerify(tester);
  });
}

Future<void> restoreAndVerify(WidgetTester tester) async {
  expect(find.text(text), findsNothing);
  expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, 0);

  await tester.enterText(find.byType(TextField), text);
  await skipPastScrollingAnimation(tester);
  expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, 0);

  await tester.drag(find.byType(Scrollable), const Offset(0, -80));
  await skipPastScrollingAnimation(tester);

  expect(find.text(text), findsOneWidget);
  expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, 60);

  await tester.restartAndRestore();

  expect(find.text(text), findsOneWidget);
  expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, 60);

  final TestRestorationData data = await tester.getRestorationData();

  await tester.enterText(find.byType(TextField), alternativeText);
  await skipPastScrollingAnimation(tester);
  await tester.drag(find.byType(Scrollable), const Offset(0, 80));
  await skipPastScrollingAnimation(tester);

  expect(find.text(text), findsNothing);
  expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, isNot(60));

  await tester.restoreFrom(data);

  expect(find.text(text), findsOneWidget);
  expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, 60);
}

class TestWidget extends StatefulWidget {
  const TestWidget({super.key, this.useExternal = false});

  final bool useExternal;

  @override
  TestWidgetState createState() => TestWidgetState();
}

class TestWidgetState extends State<TestWidget> with RestorationMixin {
  final RestorableTextEditingController controller = RestorableTextEditingController();

  @override
  String get restorationId => 'widget';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(controller, 'controller');
  }

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Material(
      child: Align(
        child: SizedBox(
          width: 50,
          child: TextField(
            restorationId: 'text',
            maxLines: 3,
            controller: widget.useExternal ? controller.value : null,
          ),
        ),
      ),
    );
  }
}

Future<void> skipPastScrollingAnimation(WidgetTester tester) async {
  await tester.pump();
  await tester.pump(const Duration(milliseconds: 200));
}
