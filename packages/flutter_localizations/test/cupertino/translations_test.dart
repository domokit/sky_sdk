// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:flutter/cupertino.dart';
import 'package:flutter/services.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:path/path.dart' as path;

import '../test_utils.dart';

final String rootDirectoryPath = Directory.current.path;

void main() {
  for (final String language in kCupertinoSupportedLanguages) {
    testWidgets('translations exist for $language', (WidgetTester tester) async {
      final Locale locale = Locale(language);

      expect(GlobalCupertinoLocalizations.delegate.isSupported(locale), isTrue);

      final CupertinoLocalizations localizations = await GlobalCupertinoLocalizations.delegate.load(
        locale,
      );

      expect(localizations.datePickerYear(0), isNotNull);
      expect(localizations.datePickerYear(1), isNotNull);
      expect(localizations.datePickerYear(2), isNotNull);
      expect(localizations.datePickerYear(10), isNotNull);

      expect(localizations.datePickerMonth(1), isNotNull);
      expect(localizations.datePickerMonth(2), isNotNull);
      expect(localizations.datePickerMonth(11), isNotNull);
      expect(localizations.datePickerMonth(12), isNotNull);

      expect(localizations.datePickerStandaloneMonth(1), isNotNull);
      expect(localizations.datePickerStandaloneMonth(2), isNotNull);
      expect(localizations.datePickerStandaloneMonth(11), isNotNull);
      expect(localizations.datePickerStandaloneMonth(12), isNotNull);

      expect(localizations.datePickerDayOfMonth(0), isNotNull);
      expect(localizations.datePickerDayOfMonth(1), isNotNull);
      expect(localizations.datePickerDayOfMonth(2), isNotNull);
      expect(localizations.datePickerDayOfMonth(10), isNotNull);

      expect(localizations.datePickerDayOfMonth(0, 1), isNotNull);
      expect(localizations.datePickerDayOfMonth(1, 2), isNotNull);
      expect(localizations.datePickerDayOfMonth(2, 3), isNotNull);
      expect(localizations.datePickerDayOfMonth(10, 4), isNotNull);

      expect(localizations.datePickerMediumDate(DateTime(2019, 3, 25)), isNotNull);

      expect(localizations.datePickerHour(0), isNotNull);
      expect(localizations.datePickerHour(1), isNotNull);
      expect(localizations.datePickerHour(2), isNotNull);
      expect(localizations.datePickerHour(10), isNotNull);

      expect(localizations.datePickerHourSemanticsLabel(0), isNotNull);
      expect(localizations.datePickerHourSemanticsLabel(1), isNotNull);
      expect(localizations.datePickerHourSemanticsLabel(2), isNotNull);
      expect(localizations.datePickerHourSemanticsLabel(10), isNotNull);
      expect(localizations.datePickerHourSemanticsLabel(0), isNot(contains(r'$hour')));
      expect(localizations.datePickerHourSemanticsLabel(1), isNot(contains(r'$hour')));
      expect(localizations.datePickerHourSemanticsLabel(2), isNot(contains(r'$hour')));
      expect(localizations.datePickerHourSemanticsLabel(10), isNot(contains(r'$hour')));

      expect(localizations.datePickerDateOrder, isNotNull);
      expect(localizations.datePickerDateTimeOrder, isNotNull);
      expect(localizations.anteMeridiemAbbreviation, isNotNull);
      expect(localizations.postMeridiemAbbreviation, isNotNull);
      expect(localizations.alertDialogLabel, isNotNull);

      expect(localizations.timerPickerHour(0), isNotNull);
      expect(localizations.timerPickerHour(1), isNotNull);
      expect(localizations.timerPickerHour(2), isNotNull);
      expect(localizations.timerPickerHour(10), isNotNull);

      expect(localizations.timerPickerMinute(0), isNotNull);
      expect(localizations.timerPickerMinute(1), isNotNull);
      expect(localizations.timerPickerMinute(2), isNotNull);
      expect(localizations.timerPickerMinute(10), isNotNull);

      expect(localizations.timerPickerSecond(0), isNotNull);
      expect(localizations.timerPickerSecond(1), isNotNull);
      expect(localizations.timerPickerSecond(2), isNotNull);
      expect(localizations.timerPickerSecond(10), isNotNull);

      expect(localizations.timerPickerHourLabel(0), isNotNull);
      expect(localizations.timerPickerHourLabel(1), isNotNull);
      expect(localizations.timerPickerHourLabel(2), isNotNull);
      expect(localizations.timerPickerHourLabel(10), isNotNull);

      expect(localizations.timerPickerMinuteLabel(0), isNotNull);
      expect(localizations.timerPickerMinuteLabel(1), isNotNull);
      expect(localizations.timerPickerMinuteLabel(2), isNotNull);
      expect(localizations.timerPickerMinuteLabel(10), isNotNull);

      expect(localizations.timerPickerSecondLabel(0), isNotNull);
      expect(localizations.timerPickerSecondLabel(1), isNotNull);
      expect(localizations.timerPickerSecondLabel(2), isNotNull);
      expect(localizations.timerPickerSecondLabel(10), isNotNull);

      expect(localizations.cutButtonLabel, isNotNull);
      expect(localizations.copyButtonLabel, isNotNull);
      expect(localizations.pasteButtonLabel, isNotNull);
      expect(localizations.selectAllButtonLabel, isNotNull);

      expect(localizations.tabSemanticsLabel(tabIndex: 2, tabCount: 5), isNotNull);
      expect(
        localizations.tabSemanticsLabel(tabIndex: 2, tabCount: 5),
        isNot(contains(r'$tabIndex')),
      );
      expect(
        localizations.tabSemanticsLabel(tabIndex: 2, tabCount: 5),
        isNot(contains(r'$tabCount')),
      );
      expect(() => localizations.tabSemanticsLabel(tabIndex: 0, tabCount: 5), throwsAssertionError);
      expect(() => localizations.tabSemanticsLabel(tabIndex: 2, tabCount: 0), throwsAssertionError);
    });
  }

  testWidgets('Spot check French', (WidgetTester tester) async {
    const Locale locale = Locale('fr');
    expect(GlobalCupertinoLocalizations.delegate.isSupported(locale), isTrue);
    final CupertinoLocalizations localizations = await GlobalCupertinoLocalizations.delegate.load(
      locale,
    );
    expect(localizations, isA<CupertinoLocalizationFr>());
    expect(localizations.alertDialogLabel, 'Alerte');
    expect(localizations.datePickerHourSemanticsLabel(1), '1 heure');
    expect(localizations.datePickerHourSemanticsLabel(12), '12 heures');
    expect(localizations.pasteButtonLabel, 'Coller');
    expect(localizations.datePickerDateOrder, DatePickerDateOrder.dmy);
    expect(localizations.timerPickerSecondLabel(20), 's');
    expect(localizations.selectAllButtonLabel, 'Tout sélectionner');
    expect(localizations.timerPickerMinute(10), '10');
  });

  testWidgets('Spot check Chinese', (WidgetTester tester) async {
    const Locale locale = Locale('zh');
    expect(GlobalCupertinoLocalizations.delegate.isSupported(locale), isTrue);
    final CupertinoLocalizations localizations = await GlobalCupertinoLocalizations.delegate.load(
      locale,
    );
    expect(localizations, isA<CupertinoLocalizationZh>());
    expect(localizations.alertDialogLabel, '提醒');
    expect(localizations.datePickerHourSemanticsLabel(1), '1 点');
    expect(localizations.datePickerHourSemanticsLabel(12), '12 点');
    expect(localizations.pasteButtonLabel, '粘贴');
    expect(localizations.datePickerDateOrder, DatePickerDateOrder.ymd);
    expect(localizations.timerPickerSecondLabel(20), '秒');
    expect(localizations.selectAllButtonLabel, '全选');
    expect(localizations.timerPickerMinute(10), '10');
  });

  // Regression test for https://github.com/flutter/flutter/issues/53036.
  testWidgets('`nb` uses `no` as its synonym when `nb` arb file is not present', (
    WidgetTester tester,
  ) async {
    final File nbCupertinoArbFile = File(
      path.join(rootDirectoryPath, 'lib', 'src', 'l10n', 'cupertino_nb.arb'),
    );
    final File noCupertinoArbFile = File(
      path.join(rootDirectoryPath, 'lib', 'src', 'l10n', 'cupertino_no.arb'),
    );

    if (noCupertinoArbFile.existsSync() && !nbCupertinoArbFile.existsSync()) {
      Locale locale = const Locale.fromSubtags(languageCode: 'no');
      expect(GlobalCupertinoLocalizations.delegate.isSupported(locale), isTrue);
      CupertinoLocalizations localizations = await GlobalCupertinoLocalizations.delegate.load(
        locale,
      );
      expect(localizations, isA<CupertinoLocalizationNo>());

      final String pasteButtonLabelNo = localizations.pasteButtonLabel;
      final String copyButtonLabelNo = localizations.copyButtonLabel;
      final String cutButtonLabelNo = localizations.cutButtonLabel;

      locale = const Locale.fromSubtags(languageCode: 'nb');
      expect(GlobalCupertinoLocalizations.delegate.isSupported(locale), isTrue);
      localizations = await GlobalCupertinoLocalizations.delegate.load(locale);
      expect(localizations, isA<CupertinoLocalizationNb>());
      expect(localizations.pasteButtonLabel, pasteButtonLabelNo);
      expect(localizations.copyButtonLabel, copyButtonLabelNo);
      expect(localizations.cutButtonLabel, cutButtonLabelNo);
    }
  });

  // Regression test for https://github.com/flutter/flutter/issues/36704.
  testWidgets('kn arb file should be properly Unicode escaped', (WidgetTester tester) async {
    final File file = File(path.join(rootDirectoryPath, 'lib', 'src', 'l10n', 'cupertino_kn.arb'));

    final Map<String, dynamic> bundle =
        json.decode(file.readAsStringSync()) as Map<String, dynamic>;

    // Encodes the arb resource values if they have not already been
    // encoded.
    encodeBundleTranslations(bundle);

    // Generates the encoded arb output file in as a string.
    final String encodedArbFile = generateArbString(bundle);

    // After encoding the bundles, the generated string should match
    // the existing material_kn.arb.
    if (Platform.isWindows) {
      // On Windows, the character '\n' can output the two-character sequence
      // '\r\n' (and when reading the file back, '\r\n' is translated back
      // into a single '\n' character).
      expect(file.readAsStringSync().replaceAll('\r\n', '\n'), encodedArbFile);
    } else {
      expect(file.readAsStringSync(), encodedArbFile);
    }
  });

  // Regression test for https://github.com/flutter/flutter/issues/110451.
  testWidgets('Finnish translation for tab label', (WidgetTester tester) async {
    const Locale locale = Locale('fi');
    expect(GlobalCupertinoLocalizations.delegate.isSupported(locale), isTrue);
    final CupertinoLocalizations localizations = await GlobalCupertinoLocalizations.delegate.load(
      locale,
    );
    expect(localizations, isA<CupertinoLocalizationFi>());
    expect(localizations.tabSemanticsLabel(tabIndex: 1, tabCount: 2), 'Välilehti 1 kautta 2');
  });

  // Regression test for https://github.com/flutter/flutter/issues/130874.
  testWidgets(
    'buildButtonItems builds a localized "No Replacements found" button when no suggestions',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          locale: const Locale('ru'),
          localizationsDelegates: GlobalCupertinoLocalizations.delegates,
          supportedLocales: const <Locale>[Locale('en'), Locale('ru')],
          home: _FakeEditableText(),
        ),
      );
      final _FakeEditableTextState editableTextState = tester.state(find.byType(_FakeEditableText));
      final List<ContextMenuButtonItem>? buttonItems =
          CupertinoSpellCheckSuggestionsToolbar.buildButtonItems(editableTextState);

      expect(buttonItems, isNotNull);
      expect(buttonItems, hasLength(1));
      expect(buttonItems!.first.label, 'Варианты замены не найдены');
      expect(buttonItems.first.onPressed, isNull);
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/141764
  testWidgets('zh-CN translation for look up label', (WidgetTester tester) async {
    const Locale locale = Locale('zh');
    expect(GlobalCupertinoLocalizations.delegate.isSupported(locale), isTrue);
    final CupertinoLocalizations localizations = await GlobalCupertinoLocalizations.delegate.load(
      locale,
    );
    expect(localizations, isA<CupertinoLocalizationZh>());
    expect(localizations.lookUpButtonLabel, '查询');
  });

  // Regression test for https://github.com/flutter/flutter/pull/151364
  testWidgets('ko-KR translation for cut, copy, paste label in ButtonLabel', (
    WidgetTester tester,
  ) async {
    const Locale locale = Locale('ko');
    expect(GlobalCupertinoLocalizations.delegate.isSupported(locale), isTrue);
    final CupertinoLocalizations localizations = await GlobalCupertinoLocalizations.delegate.load(
      locale,
    );
    expect(localizations, isA<CupertinoLocalizationKo>());
    expect(localizations.cutButtonLabel, '잘라내기');
    expect(localizations.copyButtonLabel, '복사');
    expect(localizations.pasteButtonLabel, '붙여넣기');
  });

  testWidgets('localizations.datePickerDayOfMonth uses the current locale for weekdays', (
    WidgetTester tester,
  ) async {
    const Locale locale = Locale('zh');
    expect(GlobalCupertinoLocalizations.delegate.isSupported(locale), isTrue);
    final CupertinoLocalizations localizations = await GlobalCupertinoLocalizations.delegate.load(
      locale,
    );
    expect(localizations, isA<CupertinoLocalizationZh>());
    expect(localizations.datePickerDayOfMonth(1), '1日');
    expect(localizations.datePickerDayOfMonth(1, 2), '周二 1日');
  });
}

class _FakeEditableText extends EditableText {
  _FakeEditableText()
    : super(
        controller: TextEditingController(),
        focusNode: FocusNode(),
        backgroundCursorColor: CupertinoColors.white,
        cursorColor: CupertinoColors.white,
        style: const TextStyle(),
      );

  @override
  _FakeEditableTextState createState() => _FakeEditableTextState();
}

class _FakeEditableTextState extends EditableTextState {
  _FakeEditableTextState();

  @override
  TextEditingValue get currentTextEditingValue => TextEditingValue.empty;

  @override
  SuggestionSpan? findSuggestionSpanAtCursorIndex(int cursorIndex) {
    return const SuggestionSpan(TextRange(start: 0, end: 0), <String>[]);
  }
}
