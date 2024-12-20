// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/src/services/spell_check.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Passes textAlign to underlying CupertinoTextField', (WidgetTester tester) async {
    const TextAlign alignment = TextAlign.center;

    await tester.pumpWidget(
      CupertinoApp(home: Center(child: CupertinoTextFormFieldRow(textAlign: alignment))),
    );

    final Finder textFieldFinder = find.byType(CupertinoTextField);
    expect(textFieldFinder, findsOneWidget);

    final CupertinoTextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.textAlign, alignment);
  });

  testWidgets('Passes spellCheckConfiguration to underlying CupertinoTextField', (
    WidgetTester tester,
  ) async {
    final SpellCheckConfiguration spellCheckConfig = SpellCheckConfiguration(
      spellCheckService: DefaultSpellCheckService(),
      misspelledSelectionColor: const Color.fromARGB(255, 255, 255, 0),
    );

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoTextFormFieldRow(spellCheckConfiguration: spellCheckConfig)),
      ),
    );

    final Finder textFieldFinder = find.byType(CupertinoTextField);
    expect(textFieldFinder, findsOneWidget);

    final CupertinoTextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.spellCheckConfiguration, spellCheckConfig);
  });

  testWidgets('Passes scrollPhysics to underlying TextField', (WidgetTester tester) async {
    const ScrollPhysics scrollPhysics = ScrollPhysics();

    await tester.pumpWidget(
      CupertinoApp(home: Center(child: CupertinoTextFormFieldRow(scrollPhysics: scrollPhysics))),
    );

    final Finder textFieldFinder = find.byType(CupertinoTextField);
    expect(textFieldFinder, findsOneWidget);

    final CupertinoTextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.scrollPhysics, scrollPhysics);
  });

  testWidgets('Passes textAlignVertical to underlying CupertinoTextField', (
    WidgetTester tester,
  ) async {
    const TextAlignVertical textAlignVertical = TextAlignVertical.bottom;

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoTextFormFieldRow(textAlignVertical: textAlignVertical)),
      ),
    );

    final Finder textFieldFinder = find.byType(CupertinoTextField);
    expect(textFieldFinder, findsOneWidget);

    final CupertinoTextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.textAlignVertical, textAlignVertical);
  });

  testWidgets('Passes textInputAction to underlying CupertinoTextField', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoTextFormFieldRow(textInputAction: TextInputAction.next)),
      ),
    );

    final Finder textFieldFinder = find.byType(CupertinoTextField);
    expect(textFieldFinder, findsOneWidget);

    final CupertinoTextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.textInputAction, TextInputAction.next);
  });

  testWidgets('Passes onEditingComplete to underlying CupertinoTextField', (
    WidgetTester tester,
  ) async {
    void onEditingComplete() {}

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoTextFormFieldRow(onEditingComplete: onEditingComplete)),
      ),
    );

    final Finder textFieldFinder = find.byType(CupertinoTextField);
    expect(textFieldFinder, findsOneWidget);

    final CupertinoTextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.onEditingComplete, onEditingComplete);
  });

  testWidgets('Passes cursor attributes to underlying CupertinoTextField', (
    WidgetTester tester,
  ) async {
    const double cursorWidth = 3.14;
    const double cursorHeight = 6.28;
    const Radius cursorRadius = Radius.circular(2);
    const Color cursorColor = CupertinoColors.systemPurple;

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTextFormFieldRow(
            cursorWidth: cursorWidth,
            cursorHeight: cursorHeight,
            cursorColor: cursorColor,
          ),
        ),
      ),
    );

    final Finder textFieldFinder = find.byType(CupertinoTextField);
    expect(textFieldFinder, findsOneWidget);

    final CupertinoTextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.cursorWidth, cursorWidth);
    expect(textFieldWidget.cursorHeight, cursorHeight);
    expect(textFieldWidget.cursorRadius, cursorRadius);
    expect(textFieldWidget.cursorColor, cursorColor);
  });

  testWidgets('onFieldSubmit callbacks are called', (WidgetTester tester) async {
    bool called = false;

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTextFormFieldRow(
            onFieldSubmitted: (String value) {
              called = true;
            },
          ),
        ),
      ),
    );

    await tester.showKeyboard(find.byType(CupertinoTextField));
    await tester.testTextInput.receiveAction(TextInputAction.done);
    await tester.pump();
    expect(called, true);
  });

  testWidgets('onChanged callbacks are called', (WidgetTester tester) async {
    late String value;

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTextFormFieldRow(
            onChanged: (String v) {
              value = v;
            },
          ),
        ),
      ),
    );

    await tester.enterText(find.byType(CupertinoTextField), 'Soup');
    await tester.pump();
    expect(value, 'Soup');
  });

  testWidgets('autovalidateMode is passed to super', (WidgetTester tester) async {
    int validateCalled = 0;

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTextFormFieldRow(
            autovalidateMode: AutovalidateMode.always,
            validator: (String? value) {
              validateCalled++;
              return null;
            },
          ),
        ),
      ),
    );

    expect(validateCalled, 1);
    await tester.enterText(find.byType(CupertinoTextField), 'a');
    await tester.pump();
    expect(validateCalled, 2);
  });

  testWidgets('validate is called if widget is enabled', (WidgetTester tester) async {
    int validateCalled = 0;

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTextFormFieldRow(
            enabled: true,
            autovalidateMode: AutovalidateMode.always,
            validator: (String? value) {
              validateCalled += 1;
              return null;
            },
          ),
        ),
      ),
    );

    expect(validateCalled, 1);
    await tester.enterText(find.byType(CupertinoTextField), 'a');
    await tester.pump();
    expect(validateCalled, 2);
  });

  testWidgets('readonly text form field will hide cursor by default', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoTextFormFieldRow(initialValue: 'readonly', readOnly: true)),
      ),
    );

    await tester.showKeyboard(find.byType(CupertinoTextFormFieldRow));
    expect(tester.testTextInput.hasAnyClients, false);

    await tester.tap(find.byType(CupertinoTextField));
    await tester.pump();
    expect(tester.testTextInput.hasAnyClients, false);

    await tester.longPress(find.text('readonly'));
    await tester.pump();

    // Context menu should not have paste.
    expect(find.byType(CupertinoTextSelectionToolbar), findsOneWidget);
    expect(find.text('Paste'), findsNothing);

    final EditableTextState editableTextState = tester.firstState(find.byType(EditableText));
    final RenderEditable renderEditable = editableTextState.renderEditable;

    // Make sure it does not paint caret for a period of time.
    await tester.pump(const Duration(milliseconds: 200));
    expect(renderEditable, paintsExactlyCountTimes(#drawRect, 0));

    await tester.pump(const Duration(milliseconds: 200));
    expect(renderEditable, paintsExactlyCountTimes(#drawRect, 0));

    await tester.pump(const Duration(milliseconds: 200));
    expect(renderEditable, paintsExactlyCountTimes(#drawRect, 0));
  }, skip: isBrowser); // [intended] We do not use Flutter-rendered context menu on the Web.

  testWidgets('onTap is called upon tap', (WidgetTester tester) async {
    int tapCount = 0;
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTextFormFieldRow(
            onTap: () {
              tapCount += 1;
            },
          ),
        ),
      ),
    );

    expect(tapCount, 0);
    await tester.tap(find.byType(CupertinoTextField));
    // Wait a bit so they're all single taps and not double taps.
    await tester.pump(const Duration(milliseconds: 300));
    await tester.tap(find.byType(CupertinoTextField));
    await tester.pump(const Duration(milliseconds: 300));
    await tester.tap(find.byType(CupertinoTextField));
    await tester.pump(const Duration(milliseconds: 300));
    expect(tapCount, 3);
  });

  // Regression test for https://github.com/flutter/flutter/issues/54472.
  testWidgets('reset resets the text fields value to the initialValue', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(home: Center(child: CupertinoTextFormFieldRow(initialValue: 'initialValue'))),
    );

    await tester.enterText(find.byType(CupertinoTextFormFieldRow), 'changedValue');

    final FormFieldState<String> state = tester.state<FormFieldState<String>>(
      find.byType(CupertinoTextFormFieldRow),
    );
    state.reset();

    expect(find.text('changedValue'), findsNothing);
    expect(find.text('initialValue'), findsOneWidget);
  });

  // Regression test for https://github.com/flutter/flutter/issues/54472.
  testWidgets('didChange changes text fields value', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(home: Center(child: CupertinoTextFormFieldRow(initialValue: 'initialValue'))),
    );

    expect(find.text('initialValue'), findsOneWidget);

    final FormFieldState<String> state = tester.state<FormFieldState<String>>(
      find.byType(CupertinoTextFormFieldRow),
    );
    state.didChange('changedValue');

    expect(find.text('initialValue'), findsNothing);
    expect(find.text('changedValue'), findsOneWidget);
  });

  testWidgets('onChanged callbacks value and FormFieldState.value are sync', (
    WidgetTester tester,
  ) async {
    bool called = false;

    late FormFieldState<String> state;

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTextFormFieldRow(
            onChanged: (String value) {
              called = true;
              expect(value, state.value);
            },
          ),
        ),
      ),
    );

    state = tester.state<FormFieldState<String>>(find.byType(CupertinoTextFormFieldRow));

    await tester.enterText(find.byType(CupertinoTextField), 'Soup');

    expect(called, true);
  });

  testWidgets('autofillHints is passed to super', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTextFormFieldRow(
            autofillHints: const <String>[AutofillHints.countryName],
          ),
        ),
      ),
    );

    final CupertinoTextField widget = tester.widget(find.byType(CupertinoTextField));
    expect(widget.autofillHints, equals(const <String>[AutofillHints.countryName]));
  });

  testWidgets('autovalidateMode is passed to super', (WidgetTester tester) async {
    int validateCalled = 0;

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          child: CupertinoTextFormFieldRow(
            autovalidateMode: AutovalidateMode.onUserInteraction,
            validator: (String? value) {
              validateCalled++;
              return null;
            },
          ),
        ),
      ),
    );

    expect(validateCalled, 0);
    await tester.enterText(find.byType(CupertinoTextField), 'a');
    await tester.pump();
    expect(validateCalled, 1);
  });

  testWidgets('AutovalidateMode.always mode shows error from the start', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTextFormFieldRow(
            initialValue: 'Value',
            autovalidateMode: AutovalidateMode.always,
            validator: (String? value) => 'Error',
          ),
        ),
      ),
    );

    final Finder errorTextFinder = find.byType(Text);
    expect(errorTextFinder, findsOneWidget);

    final Text errorText = tester.widget(errorTextFinder);
    expect(errorText.data, 'Error');
  });

  testWidgets('Shows error text upon invalid input', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController(text: '');
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTextFormFieldRow(
            controller: controller,
            autovalidateMode: AutovalidateMode.onUserInteraction,
            validator: (String? value) => 'Error',
          ),
        ),
      ),
    );

    expect(find.byType(Text), findsNothing);

    controller.text = 'Value';

    await tester.pumpAndSettle();

    final Finder errorTextFinder = find.byType(Text);
    expect(errorTextFinder, findsOneWidget);

    final Text errorText = tester.widget(errorTextFinder);
    expect(errorText.data, 'Error');
  });

  testWidgets('Shows prefix', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoTextFormFieldRow(prefix: const Text('Enter Value'))),
      ),
    );

    final Finder errorTextFinder = find.byType(Text);
    expect(errorTextFinder, findsOneWidget);

    final Text errorText = tester.widget(errorTextFinder);
    expect(errorText.data, 'Enter Value');
  });

  testWidgets('Passes textDirection to underlying CupertinoTextField', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoTextFormFieldRow(textDirection: TextDirection.ltr)),
      ),
    );

    final Finder ltrTextFieldFinder = find.byType(CupertinoTextField);
    expect(ltrTextFieldFinder, findsOneWidget);

    final CupertinoTextField ltrTextFieldWidget = tester.widget(ltrTextFieldFinder);
    expect(ltrTextFieldWidget.textDirection, TextDirection.ltr);

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoTextFormFieldRow(textDirection: TextDirection.rtl)),
      ),
    );

    final Finder rtlTextFieldFinder = find.byType(CupertinoTextField);
    expect(rtlTextFieldFinder, findsOneWidget);

    final CupertinoTextField rtlTextFieldWidget = tester.widget(rtlTextFieldFinder);
    expect(rtlTextFieldWidget.textDirection, TextDirection.rtl);
  });

  testWidgets('CupertinoTextFormFieldRow onChanged is called when the form is reset', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/123009.
    final GlobalKey<FormFieldState<String>> stateKey = GlobalKey<FormFieldState<String>>();
    final GlobalKey<FormState> formKey = GlobalKey<FormState>();
    String value = 'initialValue';

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: Form(
            key: formKey,
            child: CupertinoTextFormFieldRow(
              key: stateKey,
              initialValue: value,
              onChanged: (String newValue) {
                value = newValue;
              },
            ),
          ),
        ),
      ),
    );

    // Initial value is 'initialValue'.
    expect(stateKey.currentState!.value, 'initialValue');
    expect(value, 'initialValue');

    // Change value to 'changedValue'.
    await tester.enterText(find.byType(CupertinoTextField), 'changedValue');
    expect(stateKey.currentState!.value, 'changedValue');
    expect(value, 'changedValue');

    // Should be back to 'initialValue' when the form is reset.
    formKey.currentState!.reset();
    await tester.pump();
    expect(stateKey.currentState!.value, 'initialValue');
    expect(value, 'initialValue');
  });
}
