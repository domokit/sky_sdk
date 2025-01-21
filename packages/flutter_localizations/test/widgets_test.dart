// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';

class TestLocalizations {
  TestLocalizations(this.locale, this.prefix);

  final Locale locale;
  final String? prefix;

  static Future<TestLocalizations> loadSync(Locale locale, String? prefix) {
    return SynchronousFuture<TestLocalizations>(TestLocalizations(locale, prefix));
  }

  static Future<TestLocalizations> loadAsync(Locale locale, String? prefix) {
    return Future<TestLocalizations>.delayed(
      const Duration(milliseconds: 100),
      () => TestLocalizations(locale, prefix),
    );
  }

  static TestLocalizations of(BuildContext context) {
    return Localizations.of<TestLocalizations>(context, TestLocalizations)!;
  }

  String get message => '${prefix ?? ""}$locale';
}

class SyncTestLocalizationsDelegate extends LocalizationsDelegate<TestLocalizations> {
  SyncTestLocalizationsDelegate([this.prefix]);

  final String? prefix; // Changing this value triggers a rebuild
  final List<bool> shouldReloadValues = <bool>[];

  @override
  bool isSupported(Locale locale) => true;

  @override
  Future<TestLocalizations> load(Locale locale) => TestLocalizations.loadSync(locale, prefix);

  @override
  bool shouldReload(SyncTestLocalizationsDelegate old) {
    shouldReloadValues.add(prefix != old.prefix);
    return prefix != old.prefix;
  }

  @override
  String toString() => '${objectRuntimeType(this, 'SyncTestLocalizationsDelegate')}($prefix)';
}

class AsyncTestLocalizationsDelegate extends LocalizationsDelegate<TestLocalizations> {
  AsyncTestLocalizationsDelegate([this.prefix]);

  final String? prefix; // Changing this value triggers a rebuild
  final List<bool> shouldReloadValues = <bool>[];

  @override
  bool isSupported(Locale locale) => true;

  @override
  Future<TestLocalizations> load(Locale locale) => TestLocalizations.loadAsync(locale, prefix);

  @override
  bool shouldReload(AsyncTestLocalizationsDelegate old) {
    shouldReloadValues.add(prefix != old.prefix);
    return prefix != old.prefix;
  }

  @override
  String toString() => '${objectRuntimeType(this, 'AsyncTestLocalizationsDelegate')}($prefix)';
}

class MoreLocalizations {
  MoreLocalizations(this.locale);

  final Locale locale;

  static Future<MoreLocalizations> loadSync(Locale locale) {
    return SynchronousFuture<MoreLocalizations>(MoreLocalizations(locale));
  }

  static Future<MoreLocalizations> loadAsync(Locale locale) {
    return Future<MoreLocalizations>.delayed(
      const Duration(milliseconds: 100),
      () => MoreLocalizations(locale),
    );
  }

  static MoreLocalizations of(BuildContext context) {
    return Localizations.of<MoreLocalizations>(context, MoreLocalizations)!;
  }

  String get message => '$locale';
}

class SyncMoreLocalizationsDelegate extends LocalizationsDelegate<MoreLocalizations> {
  @override
  Future<MoreLocalizations> load(Locale locale) => MoreLocalizations.loadSync(locale);

  @override
  bool isSupported(Locale locale) => true;

  @override
  bool shouldReload(SyncMoreLocalizationsDelegate old) => false;
}

class AsyncMoreLocalizationsDelegate extends LocalizationsDelegate<MoreLocalizations> {
  @override
  Future<MoreLocalizations> load(Locale locale) => MoreLocalizations.loadAsync(locale);

  @override
  bool isSupported(Locale locale) => true;

  @override
  bool shouldReload(AsyncMoreLocalizationsDelegate old) => false;
}

class OnlyRTLDefaultWidgetsLocalizations extends DefaultWidgetsLocalizations {
  @override
  TextDirection get textDirection => TextDirection.rtl;
}

class OnlyRTLDefaultWidgetsLocalizationsDelegate
    extends LocalizationsDelegate<WidgetsLocalizations> {
  const OnlyRTLDefaultWidgetsLocalizationsDelegate();

  @override
  bool isSupported(Locale locale) => true;

  @override
  Future<WidgetsLocalizations> load(Locale locale) {
    return SynchronousFuture<WidgetsLocalizations>(OnlyRTLDefaultWidgetsLocalizations());
  }

  @override
  bool shouldReload(OnlyRTLDefaultWidgetsLocalizationsDelegate old) => false;
}

Widget buildFrame({
  Locale? locale,
  Iterable<LocalizationsDelegate<dynamic>>? delegates,
  required WidgetBuilder buildContent,
  LocaleResolutionCallback? localeResolutionCallback,
  List<Locale> supportedLocales = const <Locale>[Locale('en', 'US'), Locale('en', 'GB')],
}) {
  return WidgetsApp(
    color: const Color(0xFFFFFFFF),
    locale: locale,
    localizationsDelegates: delegates,
    localeResolutionCallback: localeResolutionCallback,
    supportedLocales: supportedLocales,
    onGenerateRoute: (RouteSettings settings) {
      return PageRouteBuilder<void>(
        pageBuilder: (BuildContext context, Animation<double> _, Animation<double> _) {
          return buildContent(context);
        },
      );
    },
  );
}

class SyncLoadTest extends StatefulWidget {
  const SyncLoadTest({super.key});

  @override
  SyncLoadTestState createState() => SyncLoadTestState();
}

class SyncLoadTestState extends State<SyncLoadTest> {
  @override
  Widget build(BuildContext context) {
    return Text(TestLocalizations.of(context).message, textDirection: TextDirection.rtl);
  }
}

void main() {
  testWidgets('Localizations.localeFor in a WidgetsApp with system locale', (
    WidgetTester tester,
  ) async {
    late BuildContext pageContext;

    await tester.pumpWidget(
      buildFrame(
        buildContent: (BuildContext context) {
          pageContext = context;
          return const Text('Hello World', textDirection: TextDirection.ltr);
        },
      ),
    );

    await tester.binding.setLocale('en', 'GB');
    await tester.pump();
    expect(Localizations.localeOf(pageContext), const Locale('en', 'GB'));

    await tester.binding.setLocale('en', 'US');
    await tester.pump();
    expect(Localizations.localeOf(pageContext), const Locale('en', 'US'));
  });

  testWidgets('Localizations.localeFor in a WidgetsApp with an explicit locale', (
    WidgetTester tester,
  ) async {
    const Locale locale = Locale('en', 'US');
    late BuildContext pageContext;

    await tester.pumpWidget(
      buildFrame(
        locale: locale,
        buildContent: (BuildContext context) {
          pageContext = context;
          return const Text('Hello World');
        },
      ),
    );

    expect(Localizations.localeOf(pageContext), locale);

    await tester.binding.setLocale('en', 'GB');
    await tester.pump();

    // The WidgetApp's explicit locale overrides the system's locale.
    expect(Localizations.localeOf(pageContext), locale);
  });

  testWidgets('Synchronously loaded localizations in a WidgetsApp', (WidgetTester tester) async {
    final List<LocalizationsDelegate<dynamic>> delegates = <LocalizationsDelegate<dynamic>>[
      SyncTestLocalizationsDelegate(),
      DefaultWidgetsLocalizations.delegate,
    ];

    Future<void> pumpTest(Locale locale) async {
      await tester.pumpWidget(
        Localizations(locale: locale, delegates: delegates, child: const SyncLoadTest()),
      );
    }

    await pumpTest(const Locale('en', 'US'));
    expect(find.text('en_US'), findsOneWidget);

    await pumpTest(const Locale('en', 'GB'));
    await tester.pump();
    expect(find.text('en_GB'), findsOneWidget);

    await pumpTest(const Locale('en', 'US'));
    await tester.pump();
    expect(find.text('en_US'), findsOneWidget);
  });

  testWidgets('Asynchronously loaded localizations in a WidgetsApp', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        delegates: <LocalizationsDelegate<dynamic>>[AsyncTestLocalizationsDelegate()],
        buildContent: (BuildContext context) {
          return Text(TestLocalizations.of(context).message);
        },
      ),
    );
    await tester.pump(
      const Duration(milliseconds: 50),
    ); // TestLocalizations.loadAsync() takes 100ms
    expect(find.text('en_US'), findsNothing); // TestLocalizations hasn't been loaded yet

    await tester.pump(const Duration(milliseconds: 50)); // TestLocalizations.loadAsync() completes
    await tester.pumpAndSettle();
    expect(find.text('en_US'), findsOneWidget); // default test locale is US english

    await tester.binding.setLocale('en', 'GB');
    await tester.pump(const Duration(milliseconds: 100));
    await tester.pumpAndSettle();
    expect(find.text('en_GB'), findsOneWidget);

    await tester.binding.setLocale('en', 'US');
    await tester.pump(const Duration(milliseconds: 50));
    // TestLocalizations.loadAsync() hasn't completed yet so the old text
    // localization is still displayed
    expect(find.text('en_GB'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 50)); // finish the async load
    await tester.pumpAndSettle();
    expect(find.text('en_US'), findsOneWidget);
  });

  testWidgets('Localizations with multiple sync delegates', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        delegates: <LocalizationsDelegate<dynamic>>[
          SyncTestLocalizationsDelegate(),
          SyncMoreLocalizationsDelegate(),
        ],
        locale: const Locale('en', 'US'),
        buildContent: (BuildContext context) {
          return Column(
            children: <Widget>[
              Text('A: ${TestLocalizations.of(context).message}'),
              Text('B: ${MoreLocalizations.of(context).message}'),
            ],
          );
        },
      ),
    );

    // All localizations were loaded synchronously
    expect(find.text('A: en_US'), findsOneWidget);
    expect(find.text('B: en_US'), findsOneWidget);
  });

  testWidgets('Localizations with multiple delegates', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        delegates: <LocalizationsDelegate<dynamic>>[
          SyncTestLocalizationsDelegate(),
          AsyncMoreLocalizationsDelegate(), // No resources until this completes
        ],
        locale: const Locale('en', 'US'),
        buildContent: (BuildContext context) {
          return Column(
            children: <Widget>[
              Text('A: ${TestLocalizations.of(context).message}'),
              Text('B: ${MoreLocalizations.of(context).message}'),
            ],
          );
        },
      ),
    );

    await tester.pump(const Duration(milliseconds: 50));
    expect(find.text('A: en_US'), findsNothing); // MoreLocalizations.load() hasn't completed yet
    expect(find.text('B: en_US'), findsNothing);

    await tester.pump(const Duration(milliseconds: 50));
    await tester.pumpAndSettle();

    expect(find.text('A: en_US'), findsOneWidget);
    expect(find.text('B: en_US'), findsOneWidget);
  });

  testWidgets('Multiple Localizations', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        delegates: <LocalizationsDelegate<dynamic>>[SyncTestLocalizationsDelegate()],
        locale: const Locale('en', 'US'),
        buildContent: (BuildContext context) {
          return Column(
            children: <Widget>[
              Text('A: ${TestLocalizations.of(context).message}'),
              Localizations(
                locale: const Locale('en', 'GB'),
                delegates: <LocalizationsDelegate<dynamic>>[
                  SyncTestLocalizationsDelegate(),
                  DefaultWidgetsLocalizations.delegate,
                ],
                // Create a new context within the en_GB Localization
                child: Builder(
                  builder: (BuildContext context) {
                    return Text('B: ${TestLocalizations.of(context).message}');
                  },
                ),
              ),
            ],
          );
        },
      ),
    );

    expect(find.text('A: en_US'), findsOneWidget);
    expect(find.text('B: en_GB'), findsOneWidget);
  });

  // If both the locale and the length and type of a Localizations delegate list
  // stays the same BUT one of its delegate.shouldReload() methods returns true,
  // then the dependent widgets should rebuild.
  testWidgets('Localizations sync delegate shouldReload returns true', (WidgetTester tester) async {
    final SyncTestLocalizationsDelegate originalDelegate = SyncTestLocalizationsDelegate();
    await tester.pumpWidget(
      buildFrame(
        delegates: <LocalizationsDelegate<dynamic>>[
          originalDelegate,
          SyncMoreLocalizationsDelegate(),
        ],
        locale: const Locale('en', 'US'),
        buildContent: (BuildContext context) {
          return Column(
            children: <Widget>[
              Text('A: ${TestLocalizations.of(context).message}'),
              Text('B: ${MoreLocalizations.of(context).message}'),
            ],
          );
        },
      ),
    );

    await tester.pumpAndSettle();
    expect(find.text('A: en_US'), findsOneWidget);
    expect(find.text('B: en_US'), findsOneWidget);
    expect(originalDelegate.shouldReloadValues, <bool>[]);

    final SyncTestLocalizationsDelegate modifiedDelegate = SyncTestLocalizationsDelegate('---');
    await tester.pumpWidget(
      buildFrame(
        delegates: <LocalizationsDelegate<dynamic>>[
          modifiedDelegate,
          SyncMoreLocalizationsDelegate(),
        ],
        locale: const Locale('en', 'US'),
        buildContent: (BuildContext context) {
          return Column(
            children: <Widget>[
              Text('A: ${TestLocalizations.of(context).message}'),
              Text('B: ${MoreLocalizations.of(context).message}'),
            ],
          );
        },
      ),
    );

    await tester.pumpAndSettle();
    expect(find.text('A: ---en_US'), findsOneWidget);
    expect(find.text('B: en_US'), findsOneWidget);
    expect(modifiedDelegate.shouldReloadValues, <bool>[true]);
    expect(originalDelegate.shouldReloadValues, <bool>[]);
  });

  testWidgets('Localizations async delegate shouldReload returns true', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      buildFrame(
        delegates: <LocalizationsDelegate<dynamic>>[
          AsyncTestLocalizationsDelegate(),
          AsyncMoreLocalizationsDelegate(),
        ],
        locale: const Locale('en', 'US'),
        buildContent: (BuildContext context) {
          return Column(
            children: <Widget>[
              Text('A: ${TestLocalizations.of(context).message}'),
              Text('B: ${MoreLocalizations.of(context).message}'),
            ],
          );
        },
      ),
    );

    await tester.pumpAndSettle();
    expect(find.text('A: en_US'), findsOneWidget);
    expect(find.text('B: en_US'), findsOneWidget);

    final AsyncTestLocalizationsDelegate modifiedDelegate = AsyncTestLocalizationsDelegate('---');
    await tester.pumpWidget(
      buildFrame(
        delegates: <LocalizationsDelegate<dynamic>>[
          modifiedDelegate,
          AsyncMoreLocalizationsDelegate(),
        ],
        locale: const Locale('en', 'US'),
        buildContent: (BuildContext context) {
          return Column(
            children: <Widget>[
              Text('A: ${TestLocalizations.of(context).message}'),
              Text('B: ${MoreLocalizations.of(context).message}'),
            ],
          );
        },
      ),
    );

    await tester.pumpAndSettle();
    expect(find.text('A: ---en_US'), findsOneWidget);
    expect(find.text('B: en_US'), findsOneWidget);
    expect(modifiedDelegate.shouldReloadValues, <bool>[true]);
  });

  testWidgets('Directionality tracks system locale', (WidgetTester tester) async {
    late BuildContext pageContext;

    await tester.pumpWidget(
      buildFrame(
        delegates: const <LocalizationsDelegate<dynamic>>[GlobalWidgetsLocalizations.delegate],
        supportedLocales: const <Locale>[Locale('en', 'GB'), Locale('ar', 'EG')],
        buildContent: (BuildContext context) {
          pageContext = context;
          return const Text('Hello World');
        },
      ),
    );

    await tester.binding.setLocale('en', 'GB');
    await tester.pump();
    expect(Directionality.of(pageContext), TextDirection.ltr);

    await tester.binding.setLocale('ar', 'EG');
    await tester.pump();
    expect(Directionality.of(pageContext), TextDirection.rtl);
  });

  testWidgets('localeResolutionCallback override', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        localeResolutionCallback: (Locale? newLocale, Iterable<Locale> supportedLocales) {
          return const Locale('foo', 'BAR');
        },
        buildContent: (BuildContext context) {
          return Text(Localizations.localeOf(context).toString());
        },
      ),
    );

    await tester.pumpAndSettle();
    expect(find.text('foo_BAR'), findsOneWidget);

    await tester.binding.setLocale('en', 'GB');
    await tester.pumpAndSettle();
    expect(find.text('foo_BAR'), findsOneWidget);
  });

  testWidgets('supportedLocales and defaultLocaleChangeHandler', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[
          Locale('zh', 'CN'),
          Locale('en', 'GB'),
          Locale('en', 'CA'),
        ],
        buildContent: (BuildContext context) {
          return Text(Localizations.localeOf(context).toString());
        },
      ),
    );

    await tester.pumpAndSettle();
    expect(find.text('en_GB'), findsOneWidget);

    // defaultLocaleChangedHandler prefers exact supported locale match
    await tester.binding.setLocale('en', 'CA');
    await tester.pumpAndSettle();
    expect(find.text('en_CA'), findsOneWidget);

    // defaultLocaleChangedHandler chooses 1st matching supported locale.languageCode
    await tester.binding.setLocale('en', 'US');
    await tester.pumpAndSettle();
    expect(find.text('en_GB'), findsOneWidget);

    // defaultLocaleChangedHandler: no matching supported locale, so use the 1st one
    await tester.binding.setLocale('da', 'DA');
    await tester.pumpAndSettle();
    expect(find.text('zh_CN'), findsOneWidget);
  });

  testWidgets("Localizations.override widget tracks parent's locale and delegates", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      buildFrame(
        // Accept whatever locale we're given
        localeResolutionCallback: (Locale? locale, Iterable<Locale> supportedLocales) => locale,
        delegates: const <LocalizationsDelegate<dynamic>>[GlobalWidgetsLocalizations.delegate],
        buildContent: (BuildContext context) {
          return Localizations.override(
            context: context,
            child: Builder(
              builder: (BuildContext context) {
                final Locale locale = Localizations.localeOf(context);
                final TextDirection direction = WidgetsLocalizations.of(context).textDirection;
                return Text('$locale $direction');
              },
            ),
          );
        },
      ),
    );

    // Initial WidgetTester locale is `en_US`.
    await tester.pumpAndSettle();
    expect(find.text('en_US TextDirection.ltr'), findsOneWidget);

    await tester.binding.setLocale('en', 'CA');
    await tester.pumpAndSettle();
    expect(find.text('en_CA TextDirection.ltr'), findsOneWidget);

    await tester.binding.setLocale('ar', 'EG');
    await tester.pumpAndSettle();
    expect(find.text('ar_EG TextDirection.rtl'), findsOneWidget);

    await tester.binding.setLocale('da', 'DA');
    await tester.pumpAndSettle();
    expect(find.text('da_DA TextDirection.ltr'), findsOneWidget);
  });

  testWidgets("Localizations.override widget overrides parent's DefaultWidgetLocalizations", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      buildFrame(
        // Accept whatever locale we're given
        localeResolutionCallback: (Locale? locale, Iterable<Locale> supportedLocales) => locale,
        buildContent: (BuildContext context) {
          return Localizations.override(
            context: context,
            delegates: const <OnlyRTLDefaultWidgetsLocalizationsDelegate>[
              // Override: no matter what the locale, textDirection is always RTL.
              OnlyRTLDefaultWidgetsLocalizationsDelegate(),
            ],
            child: Builder(
              builder: (BuildContext context) {
                final Locale locale = Localizations.localeOf(context);
                final TextDirection direction = WidgetsLocalizations.of(context).textDirection;
                return Text('$locale $direction');
              },
            ),
          );
        },
      ),
    );

    // Initial WidgetTester locale is `en_US`.
    await tester.pumpAndSettle();
    expect(find.text('en_US TextDirection.rtl'), findsOneWidget);

    await tester.binding.setLocale('en', 'CA');
    await tester.pumpAndSettle();
    expect(find.text('en_CA TextDirection.rtl'), findsOneWidget);

    await tester.binding.setLocale('ar', 'EG');
    await tester.pumpAndSettle();
    expect(find.text('ar_EG TextDirection.rtl'), findsOneWidget);

    await tester.binding.setLocale('da', 'DA');
    await tester.pumpAndSettle();
    expect(find.text('da_DA TextDirection.rtl'), findsOneWidget);
  });

  testWidgets('WidgetsApp overrides DefaultWidgetLocalizations', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        // Accept whatever locale we're given
        localeResolutionCallback: (Locale? locale, Iterable<Locale> supportedLocales) => locale,
        delegates: <OnlyRTLDefaultWidgetsLocalizationsDelegate>[
          const OnlyRTLDefaultWidgetsLocalizationsDelegate(),
        ],
        buildContent: (BuildContext context) {
          final Locale locale = Localizations.localeOf(context);
          final TextDirection direction = WidgetsLocalizations.of(context).textDirection;
          return Text('$locale $direction');
        },
      ),
    );

    // Initial WidgetTester locale is `en_US`.
    await tester.pumpAndSettle();
    expect(find.text('en_US TextDirection.rtl'), findsOneWidget);

    await tester.binding.setLocale('en', 'CA');
    await tester.pumpAndSettle();
    expect(find.text('en_CA TextDirection.rtl'), findsOneWidget);

    await tester.binding.setLocale('ar', 'EG');
    await tester.pumpAndSettle();
    expect(find.text('ar_EG TextDirection.rtl'), findsOneWidget);

    await tester.binding.setLocale('da', 'DA');
    await tester.pumpAndSettle();
    expect(find.text('da_DA TextDirection.rtl'), findsOneWidget);
  });

  // We provide <Locale>[Locale('en', 'US'), Locale('zh', 'CN')] as ui.window.locales
  // for flutter tester so that the behavior of tests match that of production
  // environments. Here, we test the default locales.
  testWidgets('WidgetsApp DefaultWidgetLocalizations', (WidgetTester tester) async {
    await tester.pumpAndSettle();
    await tester.pumpWidget(
      buildFrame(
        // Accept whatever locale we're given
        localeResolutionCallback: (Locale? locale, Iterable<Locale> supportedLocales) => locale,
        delegates: <OnlyRTLDefaultWidgetsLocalizationsDelegate>[
          const OnlyRTLDefaultWidgetsLocalizationsDelegate(),
        ],
        buildContent: (BuildContext context) {
          final Locale locale1 = WidgetsBinding.instance.platformDispatcher.locales.first;
          final Locale locale2 = WidgetsBinding.instance.platformDispatcher.locales[1];
          return Text('$locale1 $locale2');
        },
      ),
    );
    // Initial WidgetTester default locales is `en_US` and `zh_CN`.
    await tester.pumpAndSettle();
    expect(find.text('en_US zh_CN'), findsOneWidget);
  });

  testWidgets('WidgetsApp.locale is resolved against supportedLocales', (
    WidgetTester tester,
  ) async {
    // app locale matches a supportedLocale
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[Locale('zh', 'CN'), Locale('en', 'US')],
        locale: const Locale('en', 'US'),
        buildContent: (BuildContext context) {
          return Text(Localizations.localeOf(context).toString());
        },
      ),
    );
    await tester.pumpAndSettle();
    expect(find.text('en_US'), findsOneWidget);

    // app locale matches a supportedLocale's language
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[Locale('zh', 'CN'), Locale('en', 'GB')],
        locale: const Locale('en', 'US'),
        buildContent: (BuildContext context) {
          return Text(Localizations.localeOf(context).toString());
        },
      ),
    );
    await tester.pumpAndSettle();
    expect(find.text('en_GB'), findsOneWidget);

    // app locale matches no supportedLocale
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[Locale('zh', 'CN'), Locale('en', 'US')],
        locale: const Locale('ab', 'CD'),
        buildContent: (BuildContext context) {
          return Text(Localizations.localeOf(context).toString());
        },
      ),
    );
    await tester.pumpAndSettle();
    expect(find.text('zh_CN'), findsOneWidget);
  });

  // Example from http://unicode.org/reports/tr35/#LanguageMatching
  testWidgets('WidgetsApp Unicode tr35 1', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[Locale('de'), Locale('fr'), Locale('ja')],
        buildContent: (BuildContext context) {
          final Locale locale = Localizations.localeOf(context);
          return Text('$locale');
        },
      ),
    );
    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'de', countryCode: 'AT'),
      Locale.fromSubtags(languageCode: 'fr'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('de'), findsOneWidget);
  });

  // Examples from http://unicode.org/reports/tr35/#LanguageMatching
  testWidgets('WidgetsApp Unicode tr35 2', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[Locale('ja', 'JP'), Locale('de'), Locale('zh', 'TW')],
        buildContent: (BuildContext context) {
          final Locale locale = Localizations.localeOf(context);
          return Text('$locale');
        },
      ),
    );
    await tester.binding.setLocales(const <Locale>[Locale.fromSubtags(languageCode: 'zh')]);
    await tester.pumpAndSettle();
    expect(find.text('zh_TW'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'de'),
      Locale.fromSubtags(languageCode: 'fr'),
      Locale.fromSubtags(languageCode: 'de', countryCode: 'SW'),
      Locale.fromSubtags(languageCode: 'it'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('de'), findsOneWidget);
  });

  testWidgets('WidgetsApp EdgeCase Chinese', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[
          Locale.fromSubtags(languageCode: 'zh'),
          Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'CN'),
          Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'TW'),
          Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'HK'),
        ],
        buildContent: (BuildContext context) {
          final Locale locale = Localizations.localeOf(context);
          return Text('$locale');
        },
      ),
    );

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'de'),
      Locale.fromSubtags(languageCode: 'fr'),
      Locale.fromSubtags(languageCode: 'de', countryCode: 'SW'),
      Locale.fromSubtags(languageCode: 'zh'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'US'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_TW'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_TW'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'TW'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_TW'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'TW'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_TW'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'HK'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_HK'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'HK'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hans_CN'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'CN'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hans_CN'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh'), findsOneWidget);

    // This behavior is up to the implementer to decide if a perfect scriptCode match
    // is better than a countryCode match.
    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'CN'),
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'CN'),
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'CN'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_TW'), findsOneWidget);

    // languageCode only match is not enough to prevent resolving a perfect match
    // further down the preferredLocales list.
    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'CN'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hans_CN'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'JP'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hans_CN'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'CN'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hans_CN'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'CN'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hans_CN'), findsOneWidget);

    // When no language match, we try for country only, since it is likely users are
    // at least familiar with their country's language. This is a possible case only
    // on iOS, where countryCode can be selected independently from language and script.
    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', scriptCode: 'Hans', countryCode: 'TW'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_TW'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'TW'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_TW'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'HK'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_HK'), findsOneWidget);
  });

  // Same as 'WidgetsApp EdgeCase Chinese' test except the supportedLocales order is
  // reversed.
  testWidgets('WidgetsApp EdgeCase ReverseChinese', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[
          Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'HK'),
          Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'TW'),
          Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'CN'),
          Locale.fromSubtags(languageCode: 'zh'),
        ],
        buildContent: (BuildContext context) {
          final Locale locale = Localizations.localeOf(context);
          return Text('$locale');
        },
      ),
    );

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'de'),
      Locale.fromSubtags(languageCode: 'fr'),
      Locale.fromSubtags(languageCode: 'de', countryCode: 'SW'),
      Locale.fromSubtags(languageCode: 'zh'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'US'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_HK'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_HK'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'TW'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_TW'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'TW'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_TW'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'HK'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_HK'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'HK'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hans_CN'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'CN'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hans_CN'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_HK'), findsOneWidget);

    // This behavior is up to the implementer to decide if a perfect scriptCode match
    // is better than a countryCode match.
    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'CN'),
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'CN'),
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'CN'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_HK'), findsOneWidget);

    // languageCode only match is not enough to prevent resolving a perfect match
    // further down the preferredLocales list.
    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'CN'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hans_CN'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'JP'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hans_CN'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'zh', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'CN'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hans_CN'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
      Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'CN'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hans_CN'), findsOneWidget);

    // When no language match, we try for country only, since it is likely users are
    // at least familiar with their country's language. This is a possible case only
    // on iOS, where countryCode can be selected independently from language and script.
    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', scriptCode: 'Hans', countryCode: 'TW'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_TW'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'TW'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_TW'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'HK'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('zh_Hant_HK'), findsOneWidget);
  });

  // Examples from https://developer.android.com/guide/topics/resources/multilingual-support
  testWidgets('WidgetsApp Android', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[
          Locale('en'),
          Locale('de', 'DE'),
          Locale('es', 'ES'),
          Locale('fr', 'FR'),
          Locale('it', 'IT'),
        ],
        buildContent: (BuildContext context) {
          final Locale locale = Localizations.localeOf(context);
          return Text('$locale');
        },
      ),
    );
    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'fr', countryCode: 'CH'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('fr_FR'), findsOneWidget);
  });

  // Examples from https://developer.android.com/guide/topics/resources/multilingual-support
  testWidgets('WidgetsApp Android', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[
          Locale('en'),
          Locale('de', 'DE'),
          Locale('es', 'ES'),
          Locale('it', 'IT'),
        ],
        buildContent: (BuildContext context) {
          final Locale locale = Localizations.localeOf(context);
          return Text('$locale');
        },
      ),
    );
    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'fr', countryCode: 'CH'),
      Locale.fromSubtags(languageCode: 'it', countryCode: 'CH'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('it_IT'), findsOneWidget);
  });

  testWidgets('WidgetsApp Country-only fallback', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[
          Locale('en', 'US'),
          Locale('de', 'DE'),
          Locale('de', 'AU'),
          Locale('de', 'LU'),
          Locale('de', 'CH'),
          Locale('es', 'ES'),
          Locale('es', 'US'),
          Locale('it', 'IT'),
          Locale('zh', 'CN'),
          Locale('zh', 'TW'),
          Locale('fr', 'FR'),
          Locale('br', 'FR'),
          Locale('pt', 'BR'),
          Locale('pt', 'PT'),
        ],
        buildContent: (BuildContext context) {
          final Locale locale = Localizations.localeOf(context);
          return Text('$locale');
        },
      ),
    );
    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'ar', countryCode: 'CH'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('de_CH'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'ar', countryCode: 'FR'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('fr_FR'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'ar', countryCode: 'US'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_US'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'es', countryCode: 'US'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('es_US'), findsOneWidget);

    // Strongly prefer matching first locale even if next one is perfect.
    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'pt'),
      Locale.fromSubtags(languageCode: 'pt', countryCode: 'PT'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('pt_PT'), findsOneWidget);

    // Don't country match with any other available match. This behavior is
    // up for reconsideration.
    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'ar', countryCode: 'BR'),
      Locale.fromSubtags(languageCode: 'pt'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('pt_BR'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'ar', countryCode: 'BR'),
      Locale.fromSubtags(languageCode: 'pt', countryCode: 'PT'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('pt_PT'), findsOneWidget);
  });

  // Simulates a Chinese-default app that supports english in Canada but not
  // French. French-Canadian users should get 'en_CA' instead of Chinese.
  testWidgets('WidgetsApp Multilingual country', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[
          Locale('zh', 'CN'),
          Locale('en', 'CA'),
          Locale('en', 'US'),
          Locale('en', 'AU'),
          Locale('de', 'DE'),
        ],
        buildContent: (BuildContext context) {
          final Locale locale = Localizations.localeOf(context);
          return Text('$locale');
        },
      ),
    );

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'fr', countryCode: 'CA'),
      Locale.fromSubtags(languageCode: 'fr'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_CA'), findsOneWidget);
  });

  testWidgets('WidgetsApp Common cases', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        // Decently well localized app.
        supportedLocales: const <Locale>[
          Locale('en', 'US'),
          Locale('en', 'GB'),
          Locale('en', 'AU'),
          Locale('en', 'CA'),
          Locale('zh', 'CN'),
          Locale('zh', 'TW'),
          Locale('de', 'DE'),
          Locale('de', 'CH'),
          Locale('es', 'MX'),
          Locale('es', 'ES'),
          Locale('es', 'AR'),
          Locale('es', 'CO'),
          Locale('ru', 'RU'),
          Locale('fr', 'FR'),
          Locale('fr', 'CA'),
          Locale('ar', 'SA'),
          Locale('ar', 'EG'),
          Locale('ar', 'IQ'),
          Locale('ar', 'MA'),
          Locale('af'),
          Locale('bg'),
          Locale('nl', 'NL'),
          Locale('pl'),
          Locale('cs'),
          Locale('fa'),
          Locale('el'),
          Locale('he'),
          Locale('hi'),
          Locale('pa'),
          Locale('ta'),
          Locale('id'),
          Locale('it', 'IT'),
          Locale('ja'),
          Locale('ko'),
          Locale('ms'),
          Locale('mn'),
          Locale('pt', 'BR'),
          Locale('pt', 'PT'),
          Locale('sv', 'SE'),
          Locale('th'),
          Locale('tr'),
          Locale('vi'),
        ],
        buildContent: (BuildContext context) {
          final Locale locale = Localizations.localeOf(context);
          return Text('$locale');
        },
      ),
    );

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_US'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[Locale.fromSubtags(languageCode: 'en')]);
    await tester.pumpAndSettle();
    expect(find.text('en_US'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'CA'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_CA'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'AU'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_AU'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'ar', countryCode: 'CH'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('ar_SA'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[Locale.fromSubtags(languageCode: 'ar')]);
    await tester.pumpAndSettle();
    expect(find.text('ar_SA'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'ar', countryCode: 'IQ'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('ar_IQ'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'es', countryCode: 'ES'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('es_ES'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[Locale.fromSubtags(languageCode: 'es')]);
    await tester.pumpAndSettle();
    expect(find.text('es_MX'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'pa', countryCode: 'US'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('pa'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'hi', countryCode: 'IN'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('hi'), findsOneWidget);

    // Multiple preferred locales:

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'NZ'),
      Locale.fromSubtags(languageCode: 'en', countryCode: 'AU'),
      Locale.fromSubtags(languageCode: 'en', countryCode: 'GB'),
      Locale.fromSubtags(languageCode: 'en'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_AU'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'ab'),
      Locale.fromSubtags(languageCode: 'en', countryCode: 'NZ'),
      Locale.fromSubtags(languageCode: 'en', countryCode: 'AU'),
      Locale.fromSubtags(languageCode: 'en', countryCode: 'GB'),
      Locale.fromSubtags(languageCode: 'en'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_AU'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'NZ'),
      Locale.fromSubtags(languageCode: 'en', countryCode: 'PH'),
      Locale.fromSubtags(languageCode: 'en', countryCode: 'ZA'),
      Locale.fromSubtags(languageCode: 'en', countryCode: 'CB'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_US'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'CA'),
      Locale.fromSubtags(languageCode: 'en', countryCode: 'AU'),
      Locale.fromSubtags(languageCode: 'en', countryCode: 'GB'),
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_CA'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'da'),
      Locale.fromSubtags(languageCode: 'en'),
      Locale.fromSubtags(languageCode: 'en', countryCode: 'CA'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_CA'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'da'),
      Locale.fromSubtags(languageCode: 'fo'),
      Locale.fromSubtags(languageCode: 'hr'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_US'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'da'),
      Locale.fromSubtags(languageCode: 'fo'),
      Locale.fromSubtags(languageCode: 'hr', countryCode: 'CA'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_CA'), findsOneWidget);
  });

  testWidgets('WidgetsApp invalid preferredLocales', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFrame(
        supportedLocales: const <Locale>[
          Locale('zh', 'CN'),
          Locale('en', 'CA'),
          Locale('en', 'US'),
          Locale('en', 'AU'),
          Locale('de', 'DE'),
        ],
        localeResolutionCallback: (Locale? locale, Iterable<Locale> supportedLocales) {
          if (locale == null) {
            return const Locale('und', 'US');
          }
          return const Locale('en', 'US');
        },
        buildContent: (BuildContext context) {
          final Locale locale = Localizations.localeOf(context);
          return Text('$locale');
        },
      ),
    );

    await tester.binding.setLocales(const <Locale>[
      Locale.fromSubtags(languageCode: 'en', countryCode: 'US'),
    ]);
    await tester.pumpAndSettle();
    expect(find.text('en_US'), findsOneWidget);

    await tester.binding.setLocales(const <Locale>[]);
    await tester.pumpAndSettle();
    expect(find.text('und_US'), findsOneWidget);
  });
}
