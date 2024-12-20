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

  testWidgets('State restoration (No Form ancestor) - onUserInteraction error text validation', (
    WidgetTester tester,
  ) async {
    String? errorText(String? value) => '$value/error';
    late GlobalKey<FormFieldState<String>> formState;

    Widget builder() {
      return MaterialApp(
        restorationScopeId: 'app',
        home: MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter state) {
                  formState = GlobalKey<FormFieldState<String>>();
                  return Material(
                    child: TextFormField(
                      key: formState,
                      autovalidateMode: AutovalidateMode.onUserInteraction,
                      restorationId: 'text_form_field',
                      initialValue: 'foo',
                      validator: errorText,
                    ),
                  );
                },
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(builder());

    // No error text is visible yet.
    expect(find.text(errorText('foo')!), findsNothing);

    await tester.enterText(find.byType(TextFormField), 'bar');
    await tester.pumpAndSettle();
    expect(find.text(errorText('bar')!), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();
    await tester.restartAndRestore();
    // Error text should be present after restart and restore.
    expect(find.text(errorText('bar')!), findsOneWidget);

    // Resetting the form state should remove the error text.
    formState.currentState!.reset();
    await tester.pumpAndSettle();
    expect(find.text(errorText('bar')!), findsNothing);
    await tester.restartAndRestore();
    // Error text should still be removed after restart and restore.
    expect(find.text(errorText('bar')!), findsNothing);

    await tester.restoreFrom(data);
    expect(find.text(errorText('bar')!), findsOneWidget);
  });

  testWidgets(
    'State Restoration (No Form ancestor) - validator sets the error text only when validate is called',
    (WidgetTester tester) async {
      String? errorText(String? value) => '$value/error';
      late GlobalKey<FormFieldState<String>> formState;

      Widget builder(AutovalidateMode mode) {
        return MaterialApp(
          restorationScopeId: 'app',
          home: MediaQuery(
            data: const MediaQueryData(),
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: Center(
                child: StatefulBuilder(
                  builder: (BuildContext context, StateSetter state) {
                    formState = GlobalKey<FormFieldState<String>>();
                    return Material(
                      child: TextFormField(
                        key: formState,
                        restorationId: 'form_field',
                        autovalidateMode: mode,
                        initialValue: 'foo',
                        validator: errorText,
                      ),
                    );
                  },
                ),
              ),
            ),
          ),
        );
      }

      // Start off not autovalidating.
      await tester.pumpWidget(builder(AutovalidateMode.disabled));

      Future<void> checkErrorText(String testValue) async {
        formState.currentState!.reset();
        await tester.pumpWidget(builder(AutovalidateMode.disabled));
        await tester.enterText(find.byType(TextFormField), testValue);
        await tester.pump();

        // We have to manually validate if we're not autovalidating.
        expect(find.text(errorText(testValue)!), findsNothing);
        formState.currentState!.validate();
        await tester.pump();
        expect(find.text(errorText(testValue)!), findsOneWidget);
        final TestRestorationData data = await tester.getRestorationData();
        await tester.restartAndRestore();
        // Error text should be present after restart and restore.
        expect(find.text(errorText(testValue)!), findsOneWidget);

        formState.currentState!.reset();
        await tester.pumpAndSettle();
        expect(find.text(errorText(testValue)!), findsNothing);

        await tester.restoreFrom(data);
        expect(find.text(errorText(testValue)!), findsOneWidget);

        // Try again with autovalidation. Should validate immediately.
        formState.currentState!.reset();
        await tester.pumpWidget(builder(AutovalidateMode.always));
        await tester.enterText(find.byType(TextFormField), testValue);
        await tester.pump();

        expect(find.text(errorText(testValue)!), findsOneWidget);
        await tester.restartAndRestore();
        // Error text should be present after restart and restore.
        expect(find.text(errorText(testValue)!), findsOneWidget);
      }

      await checkErrorText('Test');
      await checkErrorText('');
    },
  );
}

Future<void> restoreAndVerify(WidgetTester tester) async {
  expect(find.text(text), findsNothing);
  expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, 0);

  await tester.enterText(find.byType(TextFormField), text);
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

  await tester.enterText(find.byType(TextFormField), alternativeText);
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
          child: TextFormField(
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
