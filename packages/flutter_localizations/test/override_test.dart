// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:intl/intl.dart' as intl;

class FooMaterialLocalizations extends MaterialLocalizationEn {
  FooMaterialLocalizations(Locale localeName, this.backButtonTooltip)
    : super(
        localeName: localeName.toString(),
        fullYearFormat: intl.DateFormat.y(),
        compactDateFormat: intl.DateFormat.yMd(),
        shortDateFormat: intl.DateFormat.yMMMd(),
        mediumDateFormat: intl.DateFormat('E, MMM\u00a0d'),
        longDateFormat: intl.DateFormat.yMMMMEEEEd(),
        yearMonthFormat: intl.DateFormat.yMMMM(),
        shortMonthDayFormat: intl.DateFormat.MMMd(),
        decimalFormat: intl.NumberFormat.decimalPattern(),
        twoDigitZeroPaddedFormat: intl.NumberFormat('00'),
      );

  @override
  final String backButtonTooltip;
}

class FooMaterialLocalizationsDelegate extends LocalizationsDelegate<MaterialLocalizations> {
  const FooMaterialLocalizationsDelegate({
    this.supportedLanguage = 'en',
    this.backButtonTooltip = 'foo',
  });

  final String supportedLanguage;
  final String backButtonTooltip;

  @override
  bool isSupported(Locale locale) {
    return supportedLanguage == 'allLanguages' || locale.languageCode == supportedLanguage;
  }

  @override
  Future<FooMaterialLocalizations> load(Locale locale) {
    return SynchronousFuture<FooMaterialLocalizations>(
      FooMaterialLocalizations(locale, backButtonTooltip),
    );
  }

  @override
  bool shouldReload(FooMaterialLocalizationsDelegate old) => false;
}

Widget buildFrame({
  Locale? locale,
  Iterable<LocalizationsDelegate<dynamic>> delegates = GlobalMaterialLocalizations.delegates,
  required WidgetBuilder buildContent,
  LocaleResolutionCallback? localeResolutionCallback,
  Iterable<Locale> supportedLocales = const <Locale>[Locale('en', 'US'), Locale('es', 'ES')],
}) {
  return MaterialApp(
    color: const Color(0xFFFFFFFF),
    locale: locale,
    supportedLocales: supportedLocales,
    localizationsDelegates: delegates,
    localeResolutionCallback: localeResolutionCallback,
    onGenerateRoute: (RouteSettings settings) {
      return MaterialPageRoute<void>(
        builder: (BuildContext context) {
          return buildContent(context);
        },
      );
    },
  );
}

void main() {
  testWidgets('Locale fallbacks', (WidgetTester tester) async {
    final Key textKey = UniqueKey();

    await tester.pumpWidget(
      buildFrame(
        buildContent: (BuildContext context) {
          return Text(MaterialLocalizations.of(context).backButtonTooltip, key: textKey);
        },
      ),
    );

    expect(tester.widget<Text>(find.byKey(textKey)).data, 'Back');

    // Unrecognized locale falls back to 'en'
    await tester.binding.setLocale('foo', 'BAR');
    await tester.pump();
    expect(tester.widget<Text>(find.byKey(textKey)).data, 'Back');

    // Spanish Bolivia locale, falls back to just 'es'
    await tester.binding.setLocale('es', 'BO');
    await tester.pump();
    expect(tester.widget<Text>(find.byKey(textKey)).data, 'Atrás');
  });

  testWidgets("Localizations.override widget tracks parent's locale", (WidgetTester tester) async {
    Widget buildLocaleFrame(Locale locale) {
      return buildFrame(
        locale: locale,
        supportedLocales: <Locale>[locale],
        buildContent: (BuildContext context) {
          return Localizations.override(
            context: context,
            child: Builder(
              builder: (BuildContext context) {
                // No MaterialLocalizations are defined for the first Localizations
                // ancestor, so we should get the values from the default one, i.e.
                // the one created by WidgetsApp via the LocalizationsDelegate
                // provided by MaterialApp.
                return Text(MaterialLocalizations.of(context).backButtonTooltip);
              },
            ),
          );
        },
      );
    }

    await tester.pumpWidget(buildLocaleFrame(const Locale('en', 'US')));
    expect(find.text('Back'), findsOneWidget);

    await tester.pumpWidget(buildLocaleFrame(const Locale('de', 'DE')));
    expect(find.text('Zurück'), findsOneWidget);

    await tester.pumpWidget(buildLocaleFrame(const Locale('zh', 'CN')));
    expect(find.text('返回'), findsOneWidget);
  });

  testWidgets('Localizations.override widget with hardwired locale', (WidgetTester tester) async {
    Widget buildLocaleFrame(Locale locale) {
      return buildFrame(
        locale: locale,
        buildContent: (BuildContext context) {
          return Localizations.override(
            context: context,
            locale: const Locale('en', 'US'),
            child: Builder(
              builder: (BuildContext context) {
                // No MaterialLocalizations are defined for the Localizations.override
                // ancestor, so we should get all values from the default one, i.e.
                // the one created by WidgetsApp via the LocalizationsDelegate
                // provided by MaterialApp.
                return Text(MaterialLocalizations.of(context).backButtonTooltip);
              },
            ),
          );
        },
      );
    }

    await tester.pumpWidget(buildLocaleFrame(const Locale('en', 'US')));
    expect(find.text('Back'), findsOneWidget);

    await tester.pumpWidget(buildLocaleFrame(const Locale('de', 'DE')));
    expect(find.text('Back'), findsOneWidget);

    await tester.pumpWidget(buildLocaleFrame(const Locale('zh', 'CN')));
    expect(find.text('Back'), findsOneWidget);
  });

  testWidgets('MaterialApp adds MaterialLocalizations for additional languages', (
    WidgetTester tester,
  ) async {
    final Key textKey = UniqueKey();

    await tester.pumpWidget(
      buildFrame(
        delegates: <LocalizationsDelegate<dynamic>>[
          const FooMaterialLocalizationsDelegate(supportedLanguage: 'fr', backButtonTooltip: 'FR'),
          const FooMaterialLocalizationsDelegate(supportedLanguage: 'de', backButtonTooltip: 'DE'),
          GlobalCupertinoLocalizations.delegate,
        ],
        supportedLocales: const <Locale>[Locale('en'), Locale('fr'), Locale('de')],
        buildContent: (BuildContext context) {
          return Text(MaterialLocalizations.of(context).backButtonTooltip, key: textKey);
        },
      ),
    );

    expect(tester.widget<Text>(find.byKey(textKey)).data, 'Back');

    await tester.binding.setLocale('fr', 'CA');
    await tester.pump();
    expect(find.text('FR'), findsOneWidget);

    await tester.binding.setLocale('de', 'DE');
    await tester.pump();
    expect(find.text('DE'), findsOneWidget);
  });

  testWidgets('MaterialApp overrides MaterialLocalizations for all locales', (
    WidgetTester tester,
  ) async {
    final Key textKey = UniqueKey();

    await tester.pumpWidget(
      buildFrame(
        // Accept whatever locale we're given
        localeResolutionCallback: (Locale? locale, Iterable<Locale> supportedLocales) => locale,
        delegates: <LocalizationsDelegate<dynamic>>[
          const FooMaterialLocalizationsDelegate(supportedLanguage: 'allLanguages'),
          GlobalCupertinoLocalizations.delegate,
        ],
        buildContent: (BuildContext context) {
          // Should always be 'foo', no matter what the locale is
          return Text(MaterialLocalizations.of(context).backButtonTooltip, key: textKey);
        },
      ),
    );

    expect(tester.widget<Text>(find.byKey(textKey)).data, 'foo');

    await tester.binding.setLocale('zh', 'CN');
    await tester.pump();
    expect(find.text('foo'), findsOneWidget);

    await tester.binding.setLocale('de', 'DE');
    await tester.pump();
    expect(find.text('foo'), findsOneWidget);
  });

  testWidgets('MaterialApp overrides MaterialLocalizations for default locale', (
    WidgetTester tester,
  ) async {
    final Key textKey = UniqueKey();

    await tester.pumpWidget(
      buildFrame(
        delegates: <LocalizationsDelegate<dynamic>>[
          const FooMaterialLocalizationsDelegate(),
          GlobalCupertinoLocalizations.delegate,
        ],
        // supportedLocales not specified, so all locales resolve to 'en'
        buildContent: (BuildContext context) {
          return Text(MaterialLocalizations.of(context).backButtonTooltip, key: textKey);
        },
      ),
    );

    // Unsupported locale '_' (the widget tester's default) resolves to 'en'.
    expect(tester.widget<Text>(find.byKey(textKey)).data, 'foo');

    // Unsupported locale 'zh' resolves to 'en'.
    await tester.binding.setLocale('zh', 'CN');
    await tester.pump();
    expect(find.text('foo'), findsOneWidget);

    // Unsupported locale 'de' resolves to 'en'.
    await tester.binding.setLocale('de', 'DE');
    await tester.pump();
    expect(find.text('foo'), findsOneWidget);
  });

  testWidgets('deprecated Android/Java locales are modernized', (WidgetTester tester) async {
    final Key textKey = UniqueKey();

    await tester.pumpWidget(
      buildFrame(
        supportedLocales: <Locale>[
          const Locale('en', 'US'),
          const Locale('he', 'IL'),
          const Locale('yi', 'IL'),
          const Locale('id', 'JV'),
        ],
        buildContent: (BuildContext context) {
          return Text('${Localizations.localeOf(context)}', key: textKey);
        },
      ),
    );

    expect(tester.widget<Text>(find.byKey(textKey)).data, 'en_US');

    // Hebrew was iw (ISO-639) is he (ISO-639-1)
    await tester.binding.setLocale('iw', 'IL');
    await tester.pump();
    expect(tester.widget<Text>(find.byKey(textKey)).data, 'he_IL');

    // Yiddish was ji (ISO-639) is yi (ISO-639-1)
    await tester.binding.setLocale('ji', 'IL');
    await tester.pump();
    expect(
      tester.takeException(),
      "Warning: This application's locale, yi_IL, is not supported by all of its localization delegates.",
    );
    expect(tester.widget<Text>(find.byKey(textKey)).data, 'yi_IL');

    // Indonesian was in (ISO-639) is id (ISO-639-1)
    await tester.binding.setLocale('in', 'JV');
    await tester.pump();
    expect(tester.widget<Text>(find.byKey(textKey)).data, 'id_JV');
  });
}
