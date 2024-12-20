// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

const String text = 'Hello World! How are you? Life is good!';
const String alternativeText = 'Everything is awesome!!';

void main() {
  testWidgets('CupertinoTextFormFieldRow restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(restorationScopeId: 'app', home: RestorableTestWidget()),
    );

    await restoreAndVerify(tester);
  });

  testWidgets('CupertinoTextFormFieldRow restoration with external controller', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const CupertinoApp(
        restorationScopeId: 'root',
        home: RestorableTestWidget(useExternalController: true),
      ),
    );

    await restoreAndVerify(tester);
  });

  testWidgets('State restoration (No Form ancestor) - onUserInteraction error text validation', (
    WidgetTester tester,
  ) async {
    String? errorText(String? value) => '$value/error';
    late GlobalKey<FormFieldState<String>> formState;

    Widget builder() {
      return CupertinoApp(
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
                    child: CupertinoTextFormFieldRow(
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

    await tester.enterText(find.byType(CupertinoTextFormFieldRow), 'bar');
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
        return CupertinoApp(
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
                      child: CupertinoTextFormFieldRow(
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
        await tester.enterText(find.byType(CupertinoTextFormFieldRow), testValue);
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
        await tester.enterText(find.byType(CupertinoTextFormFieldRow), testValue);
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

  await tester.enterText(find.byType(CupertinoTextFormFieldRow), text);
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

  await tester.enterText(find.byType(CupertinoTextFormFieldRow), alternativeText);
  await skipPastScrollingAnimation(tester);
  await tester.drag(find.byType(Scrollable), const Offset(0, 80));
  await skipPastScrollingAnimation(tester);

  expect(find.text(text), findsNothing);
  expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, isNot(60));

  await tester.restoreFrom(data);

  expect(find.text(text), findsOneWidget);
  expect(tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels, 60);
}

class RestorableTestWidget extends StatefulWidget {
  const RestorableTestWidget({super.key, this.useExternalController = false});

  final bool useExternalController;

  @override
  RestorableTestWidgetState createState() => RestorableTestWidgetState();
}

class RestorableTestWidgetState extends State<RestorableTestWidget> with RestorationMixin {
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
          child: CupertinoTextFormFieldRow(
            restorationId: 'text',
            maxLines: 3,
            controller: widget.useExternalController ? controller.value : null,
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
