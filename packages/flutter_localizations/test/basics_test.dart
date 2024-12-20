// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Material3 - Nested Localizations', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        // Creates the outer Localizations widget.
        theme: ThemeData(useMaterial3: true),
        home: ListView(
          children: <Widget>[
            const LocalizationTracker(key: ValueKey<String>('outer')),
            Localizations(
              locale: const Locale('zh', 'CN'),
              delegates: GlobalMaterialLocalizations.delegates,
              child: const LocalizationTracker(key: ValueKey<String>('inner')),
            ),
          ],
        ),
      ),
    );
    // Most localized aspects of the TextTheme text styles are the same for the default US local and
    // for Chinese for Material3. The baselines for all text styles differ.
    final LocalizationTrackerState outerTracker = tester.state(
      find.byKey(const ValueKey<String>('outer'), skipOffstage: false),
    );
    expect(outerTracker.textBaseline, TextBaseline.alphabetic);
    final LocalizationTrackerState innerTracker = tester.state(
      find.byKey(const ValueKey<String>('inner'), skipOffstage: false),
    );
    expect(innerTracker.textBaseline, TextBaseline.ideographic);
  });

  testWidgets('Material2 - Nested Localizations', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        // Creates the outer Localizations widget.
        theme: ThemeData(useMaterial3: false),
        home: ListView(
          children: <Widget>[
            const LocalizationTracker(key: ValueKey<String>('outer')),
            Localizations(
              locale: const Locale('zh', 'CN'),
              delegates: GlobalMaterialLocalizations.delegates,
              child: const LocalizationTracker(key: ValueKey<String>('inner')),
            ),
          ],
        ),
      ),
    );
    // Most localized aspects of the TextTheme text styles are the same for the default US local and
    // for Chinese for Material2. The baselines for all text styles differ.
    final LocalizationTrackerState outerTracker = tester.state(
      find.byKey(const ValueKey<String>('outer'), skipOffstage: false),
    );
    expect(outerTracker.textBaseline, TextBaseline.alphabetic);
    final LocalizationTrackerState innerTracker = tester.state(
      find.byKey(const ValueKey<String>('inner'), skipOffstage: false),
    );
    expect(innerTracker.textBaseline, TextBaseline.ideographic);
  });

  testWidgets(
    'Localizations is compatible with ChangeNotifier.dispose() called during didChangeDependencies',
    (WidgetTester tester) async {
      // PageView calls ScrollPosition.dispose() during didChangeDependencies.
      await tester.pumpWidget(
        MaterialApp(
          supportedLocales: const <Locale>[Locale('en', 'US'), Locale('es', 'ES')],
          localizationsDelegates: <LocalizationsDelegate<dynamic>>[
            _DummyLocalizationsDelegate(),
            ...GlobalMaterialLocalizations.delegates,
          ],
          home: PageView(),
        ),
      );

      await tester.binding.setLocale('es', 'US');
      await tester.pump();
      await tester.pumpWidget(Container());
    },
  );

  testWidgets('Locale without countryCode', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/pull/16782
    await tester.pumpWidget(
      MaterialApp(
        localizationsDelegates: GlobalMaterialLocalizations.delegates,
        supportedLocales: const <Locale>[Locale('es', 'ES'), Locale('zh')],
        home: Container(),
      ),
    );

    await tester.binding.setLocale('zh', '');
    await tester.pump();
    await tester.binding.setLocale('es', 'US');
    await tester.pump();
  });
}

/// A localizations delegate that does not contain any useful data, and is only
/// used to trigger didChangeDependencies upon locale change.
class _DummyLocalizationsDelegate extends LocalizationsDelegate<DummyLocalizations> {
  @override
  Future<DummyLocalizations> load(Locale locale) async => DummyLocalizations();

  @override
  bool isSupported(Locale locale) => true;

  @override
  bool shouldReload(_DummyLocalizationsDelegate old) => true;
}

class DummyLocalizations {}

class LocalizationTracker extends StatefulWidget {
  const LocalizationTracker({super.key});

  @override
  State<StatefulWidget> createState() => LocalizationTrackerState();
}

class LocalizationTrackerState extends State<LocalizationTracker> {
  late TextBaseline textBaseline;

  @override
  Widget build(BuildContext context) {
    textBaseline = Theme.of(context).textTheme.bodySmall!.textBaseline!;
    return Container();
  }
}
