// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/clipboard_utils.dart';

class TestMaterialLocalizations extends DefaultMaterialLocalizations {
  @override
  String formatCompactDate(DateTime date) {
    return '${date.month}/${date.day}/${date.year}';
  }
}

class TestMaterialLocalizationsDelegate extends LocalizationsDelegate<MaterialLocalizations> {
  @override
  bool isSupported(Locale locale) => true;

  @override
  Future<MaterialLocalizations> load(Locale locale) {
    return SynchronousFuture<MaterialLocalizations>(TestMaterialLocalizations());
  }

  @override
  bool shouldReload(TestMaterialLocalizationsDelegate old) => false;
}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  final MockClipboard mockClipboard = MockClipboard();

  Widget inputDatePickerField({
    Key? key,
    DateTime? initialDate,
    DateTime? firstDate,
    DateTime? lastDate,
    ValueChanged<DateTime>? onDateSubmitted,
    ValueChanged<DateTime>? onDateSaved,
    SelectableDayPredicate? selectableDayPredicate,
    String? errorFormatText,
    String? errorInvalidText,
    String? fieldHintText,
    String? fieldLabelText,
    bool autofocus = false,
    Key? formKey,
    ThemeData? theme,
    Iterable<LocalizationsDelegate<dynamic>>? localizationsDelegates,
    bool acceptEmptyDate = false,
    FocusNode? focusNode,
  }) {
    return MaterialApp(
      theme: theme ?? ThemeData.from(colorScheme: const ColorScheme.light()),
      localizationsDelegates: localizationsDelegates,
      home: Material(
        child: Form(
          key: formKey,
          child: InputDatePickerFormField(
            key: key,
            initialDate: initialDate ?? DateTime(2016, DateTime.january, 15),
            firstDate: firstDate ?? DateTime(2001),
            lastDate: lastDate ?? DateTime(2031, DateTime.december, 31),
            onDateSubmitted: onDateSubmitted,
            onDateSaved: onDateSaved,
            selectableDayPredicate: selectableDayPredicate,
            errorFormatText: errorFormatText,
            errorInvalidText: errorInvalidText,
            fieldHintText: fieldHintText,
            fieldLabelText: fieldLabelText,
            autofocus: autofocus,
            acceptEmptyDate: acceptEmptyDate,
            focusNode: focusNode,
          ),
        ),
      ),
    );
  }

  TextField textField(WidgetTester tester) {
    return tester.widget<TextField>(find.byType(TextField));
  }

  TextEditingController textFieldController(WidgetTester tester) {
    return textField(tester).controller!;
  }

  double textOpacity(WidgetTester tester, String textValue) {
    final FadeTransition opacityWidget = tester.widget<FadeTransition>(
      find.ancestor(of: find.text(textValue), matching: find.byType(FadeTransition)).first,
    );
    return opacityWidget.opacity.value;
  }

  group('InputDatePickerFormField', () {
    testWidgets('Initial date is the default', (WidgetTester tester) async {
      final GlobalKey<FormState> formKey = GlobalKey<FormState>();
      final DateTime initialDate = DateTime(2016, DateTime.february, 21);
      DateTime? inputDate;
      await tester.pumpWidget(
        inputDatePickerField(
          initialDate: initialDate,
          onDateSaved: (DateTime date) => inputDate = date,
          formKey: formKey,
        ),
      );
      expect(textFieldController(tester).value.text, equals('02/21/2016'));
      formKey.currentState!.save();
      expect(inputDate, equals(initialDate));
    });

    testWidgets('Changing initial date is reflected in text value', (WidgetTester tester) async {
      final DateTime initialDate = DateTime(2016, DateTime.february, 21);
      final DateTime updatedInitialDate = DateTime(2016, DateTime.february, 23);
      await tester.pumpWidget(inputDatePickerField(initialDate: initialDate));
      expect(textFieldController(tester).value.text, equals('02/21/2016'));

      await tester.pumpWidget(inputDatePickerField(initialDate: updatedInitialDate));
      await tester.pumpAndSettle();
      expect(textFieldController(tester).value.text, equals('02/23/2016'));
    });

    testWidgets('Valid date entry', (WidgetTester tester) async {
      final GlobalKey<FormState> formKey = GlobalKey<FormState>();
      DateTime? inputDate;
      await tester.pumpWidget(
        inputDatePickerField(onDateSaved: (DateTime date) => inputDate = date, formKey: formKey),
      );

      textFieldController(tester).text = '02/21/2016';
      formKey.currentState!.save();
      expect(inputDate, equals(DateTime(2016, DateTime.february, 21)));
    });

    testWidgets('Invalid text entry shows errorFormat text', (WidgetTester tester) async {
      final GlobalKey<FormState> formKey = GlobalKey<FormState>();
      DateTime? inputDate;
      await tester.pumpWidget(
        inputDatePickerField(onDateSaved: (DateTime date) => inputDate = date, formKey: formKey),
      );
      // Default errorFormat text
      expect(find.text('Invalid format.'), findsNothing);
      await tester.enterText(find.byType(TextField), 'foobar');
      expect(formKey.currentState!.validate(), isFalse);
      await tester.pumpAndSettle();
      expect(inputDate, isNull);
      expect(find.text('Invalid format.'), findsOneWidget);

      // Change to a custom errorFormat text
      await tester.pumpWidget(
        inputDatePickerField(
          onDateSaved: (DateTime date) => inputDate = date,
          errorFormatText: 'That is not a date.',
          formKey: formKey,
        ),
      );
      expect(formKey.currentState!.validate(), isFalse);
      await tester.pumpAndSettle();
      expect(find.text('Invalid format.'), findsNothing);
      expect(find.text('That is not a date.'), findsOneWidget);
    });

    testWidgets(
      'Valid text entry, but date outside first or last date shows bounds shows errorInvalid text',
      (WidgetTester tester) async {
        final GlobalKey<FormState> formKey = GlobalKey<FormState>();
        DateTime? inputDate;
        await tester.pumpWidget(
          inputDatePickerField(
            firstDate: DateTime(1966, DateTime.february, 21),
            lastDate: DateTime(2040, DateTime.february, 23),
            onDateSaved: (DateTime date) => inputDate = date,
            formKey: formKey,
          ),
        );
        // Default errorInvalid text
        expect(find.text('Out of range.'), findsNothing);
        // Before first date
        await tester.enterText(find.byType(TextField), '02/21/1950');
        expect(formKey.currentState!.validate(), isFalse);
        await tester.pumpAndSettle();
        expect(inputDate, isNull);
        expect(find.text('Out of range.'), findsOneWidget);
        // After last date
        await tester.enterText(find.byType(TextField), '02/23/2050');
        expect(formKey.currentState!.validate(), isFalse);
        await tester.pumpAndSettle();
        expect(inputDate, isNull);
        expect(find.text('Out of range.'), findsOneWidget);

        await tester.pumpWidget(
          inputDatePickerField(
            onDateSaved: (DateTime date) => inputDate = date,
            errorInvalidText: 'Not in given range.',
            formKey: formKey,
          ),
        );
        expect(formKey.currentState!.validate(), isFalse);
        await tester.pumpAndSettle();
        expect(find.text('Out of range.'), findsNothing);
        expect(find.text('Not in given range.'), findsOneWidget);
      },
    );

    testWidgets(
      'selectableDatePredicate will be used to show errorInvalid if date is not selectable',
      (WidgetTester tester) async {
        final GlobalKey<FormState> formKey = GlobalKey<FormState>();
        DateTime? inputDate;
        await tester.pumpWidget(
          inputDatePickerField(
            initialDate: DateTime(2016, DateTime.january, 16),
            onDateSaved: (DateTime date) => inputDate = date,
            selectableDayPredicate: (DateTime date) => date.day.isEven,
            formKey: formKey,
          ),
        );
        // Default errorInvalid text
        expect(find.text('Out of range.'), findsNothing);
        // Odd day shouldn't be valid
        await tester.enterText(find.byType(TextField), '02/21/1966');
        expect(formKey.currentState!.validate(), isFalse);
        await tester.pumpAndSettle();
        expect(inputDate, isNull);
        expect(find.text('Out of range.'), findsOneWidget);
        // Even day is valid
        await tester.enterText(find.byType(TextField), '02/24/2030');
        expect(formKey.currentState!.validate(), isTrue);
        formKey.currentState!.save();
        await tester.pumpAndSettle();
        expect(inputDate, equals(DateTime(2030, DateTime.february, 24)));
        expect(find.text('Out of range.'), findsNothing);
      },
    );

    testWidgets('Empty field shows hint text when focused', (WidgetTester tester) async {
      await tester.pumpWidget(inputDatePickerField());
      // Focus on it
      await tester.tap(find.byType(TextField));
      await tester.pumpAndSettle();

      // Hint text should be invisible
      expect(textOpacity(tester, 'mm/dd/yyyy'), equals(0.0));
      textFieldController(tester).clear();
      await tester.pumpAndSettle();
      // Hint text should be visible
      expect(textOpacity(tester, 'mm/dd/yyyy'), equals(1.0));

      // Change to a different hint text
      await tester.pumpWidget(inputDatePickerField(fieldHintText: 'Enter some date'));
      await tester.pumpAndSettle();
      expect(find.text('mm/dd/yyyy'), findsNothing);
      expect(textOpacity(tester, 'Enter some date'), equals(1.0));
      await tester.enterText(find.byType(TextField), 'foobar');
      await tester.pumpAndSettle();
      expect(textOpacity(tester, 'Enter some date'), equals(0.0));
    });

    testWidgets('Label text', (WidgetTester tester) async {
      await tester.pumpWidget(inputDatePickerField());
      // Default label
      expect(find.text('Enter Date'), findsOneWidget);

      await tester.pumpWidget(inputDatePickerField(fieldLabelText: 'Give me a date!'));
      expect(find.text('Enter Date'), findsNothing);
      expect(find.text('Give me a date!'), findsOneWidget);
    });

    testWidgets('Semantics', (WidgetTester tester) async {
      final SemanticsHandle semantics = tester.ensureSemantics();

      // Fill the clipboard so that the Paste option is available in the text
      // selection menu.
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        mockClipboard.handleMethodCall,
      );
      await Clipboard.setData(const ClipboardData(text: 'Clipboard data'));
      addTearDown(
        () => tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.platform,
          null,
        ),
      );

      await tester.pumpWidget(inputDatePickerField(autofocus: true));
      await tester.pumpAndSettle();

      expect(
        tester.getSemantics(find.byType(EditableText)),
        matchesSemantics(
          label: 'Enter Date',
          isTextField: true,
          hasEnabledState: true,
          isEnabled: true,
          isFocused: true,
          value: '01/15/2016',
          hasTapAction: true,
          hasFocusAction: true,
          hasSetTextAction: true,
          hasSetSelectionAction: true,
          hasCopyAction: true,
          hasCutAction: true,
          hasPasteAction: true,
          hasMoveCursorBackwardByCharacterAction: true,
          hasMoveCursorBackwardByWordAction: true,
        ),
      );
      semantics.dispose();
    });

    testWidgets('InputDecorationTheme is honored', (WidgetTester tester) async {
      const InputBorder border = InputBorder.none;
      await tester.pumpWidget(
        inputDatePickerField(
          theme: ThemeData.from(
            colorScheme: const ColorScheme.light(),
          ).copyWith(inputDecorationTheme: const InputDecorationTheme(border: border)),
        ),
      );
      await tester.pumpAndSettle();

      // Get the border and container color from the painter of the _BorderContainer
      // (this was cribbed from input_decorator_test.dart).
      final CustomPaint customPaint = tester.widget(
        find.descendant(
          of: find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_BorderContainer'),
          matching: find.byWidgetPredicate((Widget w) => w is CustomPaint),
        ),
      );
      final dynamic /*_InputBorderPainter*/ inputBorderPainter = customPaint.foregroundPainter;
      // ignore: avoid_dynamic_calls
      final dynamic /*_InputBorderTween*/ inputBorderTween = inputBorderPainter.border;
      // ignore: avoid_dynamic_calls
      final Animation<double> animation = inputBorderPainter.borderAnimation as Animation<double>;
      // ignore: avoid_dynamic_calls
      final InputBorder actualBorder = inputBorderTween.evaluate(animation) as InputBorder;
      // ignore: avoid_dynamic_calls
      final Color containerColor = inputBorderPainter.blendedColor as Color;

      // Border should match
      expect(actualBorder, equals(border));

      // It shouldn't be filled, so the color should be transparent
      expect(containerColor, equals(Colors.transparent));
    });

    testWidgets('Date text localization', (WidgetTester tester) async {
      final Iterable<LocalizationsDelegate<dynamic>> delegates = <LocalizationsDelegate<dynamic>>[
        TestMaterialLocalizationsDelegate(),
        DefaultWidgetsLocalizations.delegate,
      ];
      await tester.pumpWidget(inputDatePickerField(localizationsDelegates: delegates));
      await tester.enterText(find.byType(TextField), '01/01/2022');
      await tester.pumpAndSettle();

      // Verify that the widget can be updated to a new value after the
      // entered text was transformed by the localization formatter.
      await tester.pumpWidget(
        inputDatePickerField(initialDate: DateTime(2017), localizationsDelegates: delegates),
      );
    });

    testWidgets(
      'when an empty date is entered and acceptEmptyDate is true, then errorFormatText is not shown',
      (WidgetTester tester) async {
        final GlobalKey<FormState> formKey = GlobalKey<FormState>();
        const String errorFormatText = 'That is not a date.';
        await tester.pumpWidget(
          inputDatePickerField(
            errorFormatText: errorFormatText,
            formKey: formKey,
            acceptEmptyDate: true,
          ),
        );
        await tester.enterText(find.byType(TextField), '');
        await tester.pumpAndSettle();
        formKey.currentState!.validate();
        await tester.pumpAndSettle();
        expect(find.text(errorFormatText), findsNothing);
      },
    );

    testWidgets(
      'when an empty date is entered and acceptEmptyDate is false, then errorFormatText is shown',
      (WidgetTester tester) async {
        final GlobalKey<FormState> formKey = GlobalKey<FormState>();
        const String errorFormatText = 'That is not a date.';
        await tester.pumpWidget(
          inputDatePickerField(errorFormatText: errorFormatText, formKey: formKey),
        );
        await tester.enterText(find.byType(TextField), '');
        await tester.pumpAndSettle();
        formKey.currentState!.validate();
        await tester.pumpAndSettle();
        expect(find.text(errorFormatText), findsOneWidget);
      },
    );
  });

  testWidgets('FocusNode can request focus', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    await tester.pumpWidget(inputDatePickerField(focusNode: focusNode));
    expect((tester.widget(find.byType(TextField)) as TextField).focusNode, focusNode);
    expect(focusNode.hasFocus, isFalse);
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, isTrue);
    focusNode.unfocus();
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, isFalse);
  });
}
