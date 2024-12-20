// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  tearDown(() {
    LicenseRegistry.reset();
  });

  testWidgets('Material3 has sentence case labels', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        builder: (BuildContext context, Widget? child) {
          return MediaQuery(
            // Display has a vertical hinge down the middle
            data: const MediaQueryData(
              size: Size(800, 600),
              displayFeatures: <DisplayFeature>[
                DisplayFeature(
                  bounds: Rect.fromLTRB(390, 0, 410, 600),
                  type: DisplayFeatureType.hinge,
                  state: DisplayFeatureState.unknown,
                ),
              ],
            ),
            child: child!,
          );
        },
        home: Builder(
          builder:
              (BuildContext context) => ElevatedButton(
                onPressed: () {
                  showAboutDialog(context: context, useRootNavigator: false, applicationName: 'A');
                },
                child: const Text('Show About Dialog'),
              ),
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pumpAndSettle();
    expect(find.text('Close'), findsOneWidget);
    expect(find.text('View licenses'), findsOneWidget);
  });

  testWidgets('Material2 - AboutListTile control test', (WidgetTester tester) async {
    const FlutterLogo logo = FlutterLogo();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        title: 'Pirate app',
        home: Scaffold(
          appBar: AppBar(title: const Text('Home')),
          drawer: Drawer(
            child: ListView(
              children: const <Widget>[
                AboutListTile(
                  applicationVersion: '0.1.2',
                  applicationIcon: logo,
                  applicationLegalese: 'I am the very model of a modern major general.',
                  aboutBoxChildren: <Widget>[Text('About box')],
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.text('About Pirate app'), findsNothing);
    expect(find.text('0.1.2'), findsNothing);
    expect(find.byWidget(logo), findsNothing);
    expect(find.text('I am the very model of a modern major general.'), findsNothing);
    expect(find.text('About box'), findsNothing);

    await tester.tap(find.byType(IconButton));
    await tester.pumpAndSettle();

    expect(find.text('About Pirate app'), findsOneWidget);
    expect(find.text('0.1.2'), findsNothing);
    expect(find.byWidget(logo), findsNothing);
    expect(find.text('I am the very model of a modern major general.'), findsNothing);
    expect(find.text('About box'), findsNothing);

    await tester.tap(find.text('About Pirate app'));
    await tester.pumpAndSettle();

    expect(find.text('About Pirate app'), findsOneWidget);
    expect(find.text('0.1.2'), findsOneWidget);
    expect(find.byWidget(logo), findsOneWidget);
    expect(find.text('I am the very model of a modern major general.'), findsOneWidget);
    expect(find.text('About box'), findsOneWidget);

    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['Pirate package '], 'Pirate license'),
      ]);
    });

    await tester.tap(find.text('VIEW LICENSES'));
    await tester.pumpAndSettle();

    expect(find.text('Pirate app'), findsOneWidget);
    expect(find.text('0.1.2'), findsOneWidget);
    expect(find.byWidget(logo), findsOneWidget);
    expect(find.text('I am the very model of a modern major general.'), findsOneWidget);
    await tester.tap(find.text('Pirate package '));
    await tester.pumpAndSettle();
    expect(find.text('Pirate license'), findsOneWidget);
  });

  testWidgets('Material3 - AboutListTile control test', (WidgetTester tester) async {
    const FlutterLogo logo = FlutterLogo();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        title: 'Pirate app',
        home: Scaffold(
          appBar: AppBar(title: const Text('Home')),
          drawer: Drawer(
            child: ListView(
              children: const <Widget>[
                AboutListTile(
                  applicationVersion: '0.1.2',
                  applicationIcon: logo,
                  applicationLegalese: 'I am the very model of a modern major general.',
                  aboutBoxChildren: <Widget>[Text('About box')],
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.text('About Pirate app'), findsNothing);
    expect(find.text('0.1.2'), findsNothing);
    expect(find.byWidget(logo), findsNothing);
    expect(find.text('I am the very model of a modern major general.'), findsNothing);
    expect(find.text('About box'), findsNothing);

    await tester.tap(find.byType(IconButton));
    await tester.pumpAndSettle();

    expect(find.text('About Pirate app'), findsOneWidget);
    expect(find.text('0.1.2'), findsNothing);
    expect(find.byWidget(logo), findsNothing);
    expect(find.text('I am the very model of a modern major general.'), findsNothing);
    expect(find.text('About box'), findsNothing);

    await tester.tap(find.text('About Pirate app'));
    await tester.pumpAndSettle();

    expect(find.text('About Pirate app'), findsOneWidget);
    expect(find.text('0.1.2'), findsOneWidget);
    expect(find.byWidget(logo), findsOneWidget);
    expect(find.text('I am the very model of a modern major general.'), findsOneWidget);
    expect(find.text('About box'), findsOneWidget);

    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['Pirate package '], 'Pirate license'),
      ]);
    });

    await tester.tap(find.text('View licenses'));
    await tester.pumpAndSettle();

    expect(find.text('Pirate app'), findsOneWidget);
    expect(find.text('0.1.2'), findsOneWidget);
    expect(find.byWidget(logo), findsOneWidget);
    expect(find.text('I am the very model of a modern major general.'), findsOneWidget);
    await tester.tap(find.text('Pirate package '));
    await tester.pumpAndSettle();
    expect(find.text('Pirate license'), findsOneWidget);
  });

  testWidgets('About box logic defaults to executable name for app name', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(title: 'flutter_tester', home: Material(child: AboutListTile())),
    );
    expect(find.text('About flutter_tester'), findsOneWidget);
  });

  testWidgets('LicensePage control test', (WidgetTester tester) async {
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['AAA'], 'BBB'),
      ]);
    });

    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['Another package'], 'Another license'),
      ]);
    });

    await tester.pumpWidget(const MaterialApp(home: Center(child: LicensePage())));

    expect(find.text('AAA'), findsNothing);
    expect(find.text('BBB'), findsNothing);
    expect(find.text('Another package'), findsNothing);
    expect(find.text('Another license'), findsNothing);

    await tester.pumpAndSettle();

    // Check for packages.
    expect(find.text('AAA'), findsOneWidget);
    expect(find.text('Another package'), findsOneWidget);

    // Check license is displayed after entering into license page for 'AAA'.
    await tester.tap(find.text('AAA'));
    await tester.pumpAndSettle();
    expect(find.text('BBB'), findsOneWidget);

    /// Go back to list of packages.
    await tester.pageBack();
    await tester.pumpAndSettle();

    /// Check license is displayed after entering into license page for
    /// 'Another package'.
    await tester.tap(find.text('Another package'));
    await tester.pumpAndSettle();
    expect(find.text('Another license'), findsOneWidget);
  });

  testWidgets('LicensePage control test with all properties', (WidgetTester tester) async {
    const FlutterLogo logo = FlutterLogo();

    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['AAA'], 'BBB'),
      ]);
    });

    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['Another package'], 'Another license'),
      ]);
    });

    await tester.pumpWidget(
      const MaterialApp(
        title: 'Pirate app',
        home: Center(
          child: LicensePage(
            applicationName: 'LicensePage test app',
            applicationVersion: '0.1.2',
            applicationIcon: logo,
            applicationLegalese: 'I am the very model of a modern major general.',
          ),
        ),
      ),
    );

    expect(find.text('Pirate app'), findsNothing);
    expect(find.text('LicensePage test app'), findsOneWidget);
    expect(find.text('0.1.2'), findsOneWidget);
    expect(find.byWidget(logo), findsOneWidget);
    expect(find.text('I am the very model of a modern major general.'), findsOneWidget);
    expect(find.text('AAA'), findsNothing);
    expect(find.text('BBB'), findsNothing);
    expect(find.text('Another package'), findsNothing);
    expect(find.text('Another license'), findsNothing);

    await tester.pumpAndSettle();

    expect(find.text('Pirate app'), findsNothing);
    expect(find.text('LicensePage test app'), findsOneWidget);
    expect(find.text('0.1.2'), findsOneWidget);
    expect(find.byWidget(logo), findsOneWidget);
    expect(find.text('I am the very model of a modern major general.'), findsOneWidget);

    // Check for packages.
    expect(find.text('AAA'), findsOneWidget);
    expect(find.text('Another package'), findsOneWidget);

    // Check license is displayed after entering into license page for 'AAA'.
    await tester.tap(find.text('AAA'));
    await tester.pumpAndSettle();
    expect(find.text('BBB'), findsOneWidget);

    /// Go back to list of packages.
    await tester.pageBack();
    await tester.pumpAndSettle();

    /// Check license is displayed after entering into license page for
    /// 'Another package'.
    await tester.tap(find.text('Another package'));
    await tester.pumpAndSettle();
    expect(find.text('Another license'), findsOneWidget);
  });

  testWidgets('Material2 - _PackageLicensePage title style without AppBarTheme', (
    WidgetTester tester,
  ) async {
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['AAA'], 'BBB'),
      ]);
    });

    const TextStyle titleTextStyle = TextStyle(fontSize: 20, color: Colors.black, inherit: false);
    const TextStyle subtitleTextStyle = TextStyle(fontSize: 15, color: Colors.red, inherit: false);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          useMaterial3: false,
          primaryTextTheme: const TextTheme(
            titleLarge: titleTextStyle,
            titleSmall: subtitleTextStyle,
          ),
        ),
        home: const Center(child: LicensePage()),
      ),
    );
    await tester.pumpAndSettle();

    // Check for packages.
    expect(find.text('AAA'), findsOneWidget);

    // Check license is displayed after entering into license page for 'AAA'.
    await tester.tap(find.text('AAA'));
    await tester.pumpAndSettle();

    // Check for titles style.
    final Text title = tester.widget(find.text('AAA'));
    expect(title.style, titleTextStyle);
    final Text subtitle = tester.widget(find.text('1 license.'));
    expect(subtitle.style, subtitleTextStyle);
  });

  testWidgets('Material3 - _PackageLicensePage title style without AppBarTheme', (
    WidgetTester tester,
  ) async {
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['AAA'], 'BBB'),
      ]);
    });

    const TextStyle titleTextStyle = TextStyle(fontSize: 20, color: Colors.black, inherit: false);
    const TextStyle subtitleTextStyle = TextStyle(fontSize: 15, color: Colors.red, inherit: false);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          useMaterial3: true,
          textTheme: const TextTheme(titleLarge: titleTextStyle, titleSmall: subtitleTextStyle),
        ),
        home: const Center(child: LicensePage()),
      ),
    );
    await tester.pumpAndSettle();

    // Check for packages.
    expect(find.text('AAA'), findsOneWidget);

    // Check license is displayed after entering into license page for 'AAA'.
    await tester.tap(find.text('AAA'));
    await tester.pumpAndSettle();

    // Check for titles style.
    final Text title = tester.widget(find.text('AAA'));
    expect(title.style, titleTextStyle);
    final Text subtitle = tester.widget(find.text('1 license.'));
    expect(subtitle.style, subtitleTextStyle);
  });

  testWidgets('_PackageLicensePage title style with AppBarTheme', (WidgetTester tester) async {
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['AAA'], 'BBB'),
      ]);
    });

    const TextStyle titleTextStyle = TextStyle(fontSize: 20, color: Colors.indigo);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          // Not used because appBarTheme is prioritized.
          primaryTextTheme: const TextTheme(
            titleLarge: TextStyle(fontSize: 12, color: Colors.grey),
            titleSmall: TextStyle(fontSize: 10, color: Colors.grey),
          ),
          appBarTheme: const AppBarTheme(
            titleTextStyle: titleTextStyle,
            foregroundColor: Colors.indigo,
          ),
        ),
        home: const Center(child: LicensePage()),
      ),
    );
    await tester.pumpAndSettle();

    // Check for packages.
    expect(find.text('AAA'), findsOneWidget);

    // Check license is displayed after entering into license page for 'AAA'.
    await tester.tap(find.text('AAA'));
    await tester.pumpAndSettle();

    // Check for titles style.
    final Text title = tester.widget(find.text('AAA'));
    expect(title.style, titleTextStyle);
  });

  testWidgets('Material2 - LicensePage respects the notch', (WidgetTester tester) async {
    const double safeareaPadding = 27.0;

    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['ABC'], 'DEF'),
      ]);
    });

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const MediaQuery(
          data: MediaQueryData(padding: EdgeInsets.all(safeareaPadding)),
          child: LicensePage(),
        ),
      ),
    );

    await tester.pumpAndSettle();

    // The position of the top left of app bar title should indicate whether
    // the safe area is sufficiently respected.
    expect(
      tester.getTopLeft(find.text('Licenses')),
      const Offset(16.0 + safeareaPadding, 18.0 + safeareaPadding),
    );
  });

  testWidgets('Material3 - LicensePage respects the notch', (WidgetTester tester) async {
    const double safeareaPadding = 27.0;

    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['ABC'], 'DEF'),
      ]);
    });

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: const MediaQuery(
          data: MediaQueryData(padding: EdgeInsets.all(safeareaPadding)),
          child: LicensePage(),
        ),
      ),
    );

    await tester.pumpAndSettle();

    // The position of the top left of app bar title should indicate whether
    // the safe area is sufficiently respected.
    expect(
      tester.getTopLeft(find.text('Licenses')),
      const Offset(16.0 + safeareaPadding, 14.0 + safeareaPadding),
    );
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('LicensePage returns early if unmounted', (WidgetTester tester) async {
    final Completer<LicenseEntry> licenseCompleter = Completer<LicenseEntry>();
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromFuture(licenseCompleter.future);
    });

    await tester.pumpWidget(const MaterialApp(home: LicensePage()));
    await tester.pump();

    await tester.pumpWidget(const MaterialApp(home: Placeholder()));

    await tester.pumpAndSettle();
    final FakeLicenseEntry licenseEntry = FakeLicenseEntry();
    licenseCompleter.complete(licenseEntry);
    expect(licenseEntry.packagesCalled, false);
  });

  testWidgets('LicensePage returns late if unmounted', (WidgetTester tester) async {
    final Completer<LicenseEntry> licenseCompleter = Completer<LicenseEntry>();
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromFuture(licenseCompleter.future);
    });

    await tester.pumpWidget(const MaterialApp(home: LicensePage()));
    await tester.pump();
    final FakeLicenseEntry licenseEntry = FakeLicenseEntry();
    licenseCompleter.complete(licenseEntry);

    await tester.pumpWidget(const MaterialApp(home: Placeholder()));

    await tester.pumpAndSettle();
    expect(licenseEntry.packagesCalled, true);
  });

  testWidgets('LicensePage logic defaults to executable name for app name', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(title: 'flutter_tester', home: Material(child: LicensePage())),
    );
    expect(find.text('flutter_tester'), findsOneWidget);
  });

  testWidgets('AboutListTile dense property is applied', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: Center(child: AboutListTile()))),
    );
    Rect tileRect = tester.getRect(find.byType(AboutListTile));
    expect(tileRect.height, 56.0);

    await tester.pumpWidget(
      const MaterialApp(home: Material(child: Center(child: AboutListTile(dense: false)))),
    );
    tileRect = tester.getRect(find.byType(AboutListTile));
    expect(tileRect.height, 56.0);

    await tester.pumpWidget(
      const MaterialApp(home: Material(child: Center(child: AboutListTile(dense: true)))),
    );
    tileRect = tester.getRect(find.byType(AboutListTile));
    expect(tileRect.height, 48.0);
  });

  testWidgets('showLicensePage uses nested navigator by default', (WidgetTester tester) async {
    final LicensePageObserver rootObserver = LicensePageObserver();
    final LicensePageObserver nestedObserver = LicensePageObserver();

    await tester.pumpWidget(
      MaterialApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        initialRoute: '/',
        onGenerateRoute: (_) {
          return PageRouteBuilder<dynamic>(
            pageBuilder:
                (_, __, ___) => Navigator(
                  observers: <NavigatorObserver>[nestedObserver],
                  onGenerateRoute: (RouteSettings settings) {
                    return PageRouteBuilder<dynamic>(
                      pageBuilder: (BuildContext context, _, __) {
                        return ElevatedButton(
                          onPressed: () {
                            showLicensePage(context: context, applicationName: 'A');
                          },
                          child: const Text('Show License Page'),
                        );
                      },
                    );
                  },
                ),
          );
        },
      ),
    );

    // Open the dialog.
    await tester.tap(find.byType(ElevatedButton));

    expect(rootObserver.licensePageCount, 0);
    expect(nestedObserver.licensePageCount, 1);
  });

  testWidgets('showLicensePage uses root navigator if useRootNavigator is true', (
    WidgetTester tester,
  ) async {
    final LicensePageObserver rootObserver = LicensePageObserver();
    final LicensePageObserver nestedObserver = LicensePageObserver();

    await tester.pumpWidget(
      MaterialApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        initialRoute: '/',
        onGenerateRoute: (_) {
          return PageRouteBuilder<dynamic>(
            pageBuilder:
                (_, __, ___) => Navigator(
                  observers: <NavigatorObserver>[nestedObserver],
                  onGenerateRoute: (RouteSettings settings) {
                    return PageRouteBuilder<dynamic>(
                      pageBuilder: (BuildContext context, _, __) {
                        return ElevatedButton(
                          onPressed: () {
                            showLicensePage(
                              context: context,
                              useRootNavigator: true,
                              applicationName: 'A',
                            );
                          },
                          child: const Text('Show License Page'),
                        );
                      },
                    );
                  },
                ),
          );
        },
      ),
    );

    // Open the dialog.
    await tester.tap(find.byType(ElevatedButton));

    expect(rootObserver.licensePageCount, 1);
    expect(nestedObserver.licensePageCount, 0);
  });

  group('Barrier dismissible', () {
    late AboutDialogObserver rootObserver;

    setUp(() {
      rootObserver = AboutDialogObserver();
    });

    testWidgets('Barrier is dismissible with default parameter', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          navigatorObservers: <NavigatorObserver>[rootObserver],
          home: Material(
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return ElevatedButton(
                    child: const Text('X'),
                    onPressed: () => showAboutDialog(context: context),
                  );
                },
              ),
            ),
          ),
        ),
      );

      // Open the dialog.
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();
      expect(rootObserver.dialogCount, 1);

      // Tap on the barrier.
      await tester.tapAt(const Offset(10.0, 10.0));
      await tester.pumpAndSettle();
      expect(rootObserver.dialogCount, 0);
    });

    testWidgets('Barrier is not dismissible with barrierDismissible is false', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          navigatorObservers: <NavigatorObserver>[rootObserver],
          home: Material(
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return ElevatedButton(
                    child: const Text('X'),
                    onPressed: () => showAboutDialog(context: context, barrierDismissible: false),
                  );
                },
              ),
            ),
          ),
        ),
      );

      // Open the dialog.
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();
      expect(rootObserver.dialogCount, 1);

      // Tap on the barrier, which shouldn't do anything this time.
      await tester.tapAt(const Offset(10.0, 10.0));
      await tester.pumpAndSettle();
      expect(rootObserver.dialogCount, 1);
    });
  });

  testWidgets('Barrier color', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  child: const Text('X'),
                  onPressed: () => showAboutDialog(context: context),
                );
              },
            ),
          ),
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pumpAndSettle();
    expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color, Colors.black54);

    // Dismiss the dialog.
    await tester.tapAt(const Offset(10.0, 10.0));

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  child: const Text('X'),
                  onPressed: () => showAboutDialog(context: context, barrierColor: Colors.pink),
                );
              },
            ),
          ),
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pumpAndSettle();
    expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color, Colors.pink);
  });

  testWidgets('Barrier Label', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  child: const Text('X'),
                  onPressed: () => showAboutDialog(context: context, barrierLabel: 'Custom Label'),
                );
              },
            ),
          ),
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pumpAndSettle();
    expect(
      tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).semanticsLabel,
      'Custom Label',
    );
  });

  testWidgets('showAboutDialog uses root navigator by default', (WidgetTester tester) async {
    final AboutDialogObserver rootObserver = AboutDialogObserver();
    final AboutDialogObserver nestedObserver = AboutDialogObserver();

    await tester.pumpWidget(
      MaterialApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        home: Navigator(
          observers: <NavigatorObserver>[nestedObserver],
          onGenerateRoute: (RouteSettings settings) {
            return MaterialPageRoute<dynamic>(
              builder: (BuildContext context) {
                return ElevatedButton(
                  onPressed: () {
                    showAboutDialog(context: context, applicationName: 'A');
                  },
                  child: const Text('Show About Dialog'),
                );
              },
            );
          },
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.byType(ElevatedButton));

    expect(rootObserver.dialogCount, 1);
    expect(nestedObserver.dialogCount, 0);
  });

  testWidgets('showAboutDialog uses nested navigator if useRootNavigator is false', (
    WidgetTester tester,
  ) async {
    final AboutDialogObserver rootObserver = AboutDialogObserver();
    final AboutDialogObserver nestedObserver = AboutDialogObserver();

    await tester.pumpWidget(
      MaterialApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        home: Navigator(
          observers: <NavigatorObserver>[nestedObserver],
          onGenerateRoute: (RouteSettings settings) {
            return MaterialPageRoute<dynamic>(
              builder: (BuildContext context) {
                return ElevatedButton(
                  onPressed: () {
                    showAboutDialog(
                      context: context,
                      useRootNavigator: false,
                      applicationName: 'A',
                    );
                  },
                  child: const Text('Show About Dialog'),
                );
              },
            );
          },
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.byType(ElevatedButton));

    expect(rootObserver.dialogCount, 0);
    expect(nestedObserver.dialogCount, 1);
  });

  group('showAboutDialog avoids overlapping display features', () {
    testWidgets('default positioning', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: child!,
            );
          },
          home: Builder(
            builder:
                (BuildContext context) => ElevatedButton(
                  onPressed: () {
                    showAboutDialog(
                      context: context,
                      useRootNavigator: false,
                      applicationName: 'A',
                    );
                  },
                  child: const Text('Show About Dialog'),
                ),
          ),
        ),
      );

      // Open the dialog.
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();

      // By default it should place the dialog on the left screen
      expect(tester.getTopLeft(find.byType(AboutDialog)), Offset.zero);
      expect(tester.getBottomRight(find.byType(AboutDialog)), const Offset(390.0, 600.0));
    });

    testWidgets('positioning using anchorPoint', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: child!,
            );
          },
          home: Builder(
            builder:
                (BuildContext context) => ElevatedButton(
                  onPressed: () {
                    showAboutDialog(
                      context: context,
                      useRootNavigator: false,
                      applicationName: 'A',
                      anchorPoint: const Offset(1000, 0),
                    );
                  },
                  child: const Text('Show About Dialog'),
                ),
          ),
        ),
      );

      // Open the dialog.
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();

      // The anchorPoint hits the right side of the display
      expect(tester.getTopLeft(find.byType(AboutDialog)), const Offset(410.0, 0.0));
      expect(tester.getBottomRight(find.byType(AboutDialog)), const Offset(800.0, 600.0));
    });

    testWidgets('positioning using Directionality', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: Directionality(textDirection: TextDirection.rtl, child: child!),
            );
          },
          home: Builder(
            builder:
                (BuildContext context) => ElevatedButton(
                  onPressed: () {
                    showAboutDialog(
                      context: context,
                      useRootNavigator: false,
                      applicationName: 'A',
                    );
                  },
                  child: const Text('Show About Dialog'),
                ),
          ),
        ),
      );

      // Open the dialog.
      await tester.tap(find.byType(ElevatedButton));
      await tester.pumpAndSettle();

      // Since this is rtl, the first screen is the on the right
      expect(tester.getTopLeft(find.byType(AboutDialog)), const Offset(410.0, 0.0));
      expect(tester.getBottomRight(find.byType(AboutDialog)), const Offset(800.0, 600.0));
    });
  });

  testWidgets("AboutListTile's child should not be offset when the icon is not specified.", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(home: Scaffold(body: AboutListTile(child: Text('About')))),
    );

    expect(
      find.descendant(of: find.byType(AboutListTile), matching: find.byType(Icon)),
      findsNothing,
    );
  });

  testWidgets("AboutDialog's contents are scrollable", (WidgetTester tester) async {
    final Key contentKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Navigator(
          onGenerateRoute: (RouteSettings settings) {
            return MaterialPageRoute<dynamic>(
              builder: (BuildContext context) {
                return ElevatedButton(
                  onPressed: () {
                    showAboutDialog(
                      context: context,
                      useRootNavigator: false,
                      applicationName: 'A',
                      children: <Widget>[
                        Container(key: contentKey, color: Colors.orange, height: 500),
                      ],
                    );
                  },
                  child: const Text('Show About Dialog'),
                );
              },
            );
          },
        ),
      ),
    );

    await tester.tap(find.text('Show About Dialog'));
    await tester.pumpAndSettle();

    // Try dragging by the [AboutDialog]'s title.
    RenderBox box = tester.renderObject(find.text('A'));
    Offset originalOffset = box.localToGlobal(Offset.zero);
    await tester.drag(find.byKey(contentKey), const Offset(0.0, -20.0));

    expect(box.localToGlobal(Offset.zero), equals(originalOffset.translate(0.0, -20.0)));

    // Try dragging by the additional children in contents.
    box = tester.renderObject(find.byKey(contentKey));
    originalOffset = box.localToGlobal(Offset.zero);
    await tester.drag(find.byKey(contentKey), const Offset(0.0, -20.0));

    expect(box.localToGlobal(Offset.zero), equals(originalOffset.translate(0.0, -20.0)));
  });

  testWidgets("Material2 - LicensePage's color must be same whether loading or done", (
    WidgetTester tester,
  ) async {
    const Color scaffoldColor = Color(0xFF123456);
    const Color cardColor = Color(0xFF654321);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light(
          useMaterial3: false,
        ).copyWith(scaffoldBackgroundColor: scaffoldColor, cardColor: cardColor),
        home: Scaffold(
          body: Center(
            child: Builder(
              builder:
                  (BuildContext context) => GestureDetector(
                    child: const Text('Show licenses'),
                    onTap: () {
                      showLicensePage(
                        context: context,
                        applicationName: 'MyApp',
                        applicationVersion: '1.0.0',
                      );
                    },
                  ),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('Show licenses'));
    await tester.pump();
    await tester.pump();

    // Check color when loading.
    final List<Material> materialLoadings =
        tester.widgetList<Material>(find.byType(Material)).toList();
    expect(materialLoadings.length, equals(4));
    expect(materialLoadings[1].color, scaffoldColor);
    expect(materialLoadings[2].color, cardColor);

    await tester.pumpAndSettle();

    // Check color when done.
    expect(find.byKey(const ValueKey<ConnectionState>(ConnectionState.done)), findsOneWidget);
    final List<Material> materialDones =
        tester.widgetList<Material>(find.byType(Material)).toList();
    expect(materialDones.length, equals(3));
    expect(materialDones[0].color, scaffoldColor);
    expect(materialDones[1].color, cardColor);
  });

  testWidgets("Material3 - LicensePage's color must be same whether loading or done", (
    WidgetTester tester,
  ) async {
    const Color scaffoldColor = Color(0xFF123456);
    const Color cardColor = Color(0xFF654321);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light(
          useMaterial3: true,
        ).copyWith(scaffoldBackgroundColor: scaffoldColor, cardColor: cardColor),
        home: Scaffold(
          body: Center(
            child: Builder(
              builder:
                  (BuildContext context) => GestureDetector(
                    child: const Text('Show licenses'),
                    onTap: () {
                      showLicensePage(
                        context: context,
                        applicationName: 'MyApp',
                        applicationVersion: '1.0.0',
                      );
                    },
                  ),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('Show licenses'));
    await tester.pump();
    await tester.pump();

    // Check color when loading.
    final List<Material> materialLoadings =
        tester.widgetList<Material>(find.byType(Material)).toList();
    expect(materialLoadings.length, equals(5));
    expect(materialLoadings[1].color, scaffoldColor);
    expect(materialLoadings[2].color, cardColor);

    await tester.pumpAndSettle();

    // Check color when done.
    expect(find.byKey(const ValueKey<ConnectionState>(ConnectionState.done)), findsOneWidget);
    final List<Material> materialDones =
        tester.widgetList<Material>(find.byType(Material)).toList();
    expect(materialDones.length, equals(4));
    expect(materialDones[0].color, scaffoldColor);
    expect(materialDones[1].color, cardColor);
  });

  testWidgets(
    'Conflicting scrollbars are not applied by ScrollBehavior to _PackageLicensePage',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/83819
      LicenseRegistry.addLicense(() {
        return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
          const LicenseEntryWithLineBreaks(<String>['AAA'], 'BBB'),
        ]);
      });

      await tester.pumpWidget(const MaterialApp(home: Center(child: LicensePage())));
      await tester.pumpAndSettle();

      // Check for packages.
      expect(find.text('AAA'), findsOneWidget);
      // Check license is displayed after entering into license page for 'AAA'.
      await tester.tap(find.text('AAA'));
      await tester.pumpAndSettle();

      // The inherited ScrollBehavior should not apply Scrollbars since they are
      // already built in to the widget.
      switch (debugDefaultTargetPlatformOverride) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(find.byType(CupertinoScrollbar), findsNothing);
        case TargetPlatform.iOS:
          expect(find.byType(CupertinoScrollbar), findsOneWidget);
        case null:
          break;
      }
      expect(find.byType(Scrollbar), findsOneWidget);
      expect(find.byType(RawScrollbar), findsNothing);
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets('ListView of license entries is primary', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/120710
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        LicenseEntryWithLineBreaks(
          <String>['AAA'],
          // Add enough content to scroll
          List<String>.generate(500, (int index) => 'BBBB').join('\n'),
        ),
      ]);
    });

    await tester.pumpWidget(
      MaterialApp(
        title: 'Flutter Code Sample',
        home: Scaffold(
          body: Builder(
            builder:
                (BuildContext context) => TextButton(
                  child: const Text('Show License Page'),
                  onPressed: () {
                    showLicensePage(context: context);
                  },
                ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(find.text('Show License Page'), findsOneWidget);
    await tester.tap(find.text('Show License Page'));
    await tester.pumpAndSettle();

    // Check for packages.
    expect(find.text('AAA'), findsOneWidget);
    // Check license is displayed after entering into license page for 'AAA'.
    await tester.tap(find.text('AAA'));
    await tester.pumpAndSettle();

    // The inherited ScrollBehavior should not apply Scrollbars since they are
    // already built in to the widget.
    switch (debugDefaultTargetPlatformOverride) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        expect(find.byType(CupertinoScrollbar), findsNothing);
      case TargetPlatform.iOS:
        expect(find.byType(CupertinoScrollbar), findsOneWidget);
      case null:
        break;
    }
    expect(find.byType(Scrollbar), findsOneWidget);
    expect(find.byType(RawScrollbar), findsNothing);
    await tester.drag(find.byType(ListView), const Offset(0.0, 20.0));
    await tester.pumpAndSettle(); // No exception triggered.
  }, variant: TargetPlatformVariant.all());

  testWidgets('LicensePage padding', (WidgetTester tester) async {
    const FlutterLogo logo = FlutterLogo();

    await tester.pumpWidget(
      const MaterialApp(
        title: 'Pirate app',
        home: Center(
          child: LicensePage(
            applicationName: 'LicensePage test app',
            applicationIcon: logo,
            applicationVersion: '0.1.2',
            applicationLegalese: 'I am the very model of a modern major general.',
          ),
        ),
      ),
    );

    final Finder appName = find.text('LicensePage test app');
    final Finder appIcon = find.byType(FlutterLogo);
    final Finder appVersion = find.text('0.1.2');
    final Finder appLegalese = find.text('I am the very model of a modern major general.');
    final Finder appPowered = find.text('Powered by Flutter');

    expect(appName, findsOneWidget);
    expect(appIcon, findsOneWidget);
    expect(appVersion, findsOneWidget);
    expect(appLegalese, findsOneWidget);
    expect(appPowered, findsOneWidget);

    // Bottom padding is applied to the app version and app legalese text.
    final double appNameBottomPadding =
        tester.getTopLeft(appIcon).dy - tester.getBottomLeft(appName).dy;
    expect(appNameBottomPadding, 0.0);

    final double appIconBottomPadding =
        tester.getTopLeft(appVersion).dy - tester.getBottomLeft(appIcon).dy;
    expect(appIconBottomPadding, 0.0);

    final double appVersionBottomPadding =
        tester.getTopLeft(appLegalese).dy - tester.getBottomLeft(appVersion).dy;
    expect(appVersionBottomPadding, 18.0);

    final double appLegaleseBottomPadding =
        tester.getTopLeft(appPowered).dy - tester.getBottomLeft(appLegalese).dy;
    expect(appLegaleseBottomPadding, 18.0);
  });

  testWidgets('LicensePage has no extra padding between app icon and app powered text', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/99559

    const FlutterLogo logo = FlutterLogo();

    await tester.pumpWidget(
      const MaterialApp(
        title: 'Pirate app',
        home: Center(child: LicensePage(applicationIcon: logo)),
      ),
    );

    final Finder appName = find.text('LicensePage test app');
    final Finder appIcon = find.byType(FlutterLogo);
    final Finder appVersion = find.text('0.1.2');
    final Finder appLegalese = find.text('I am the very model of a modern major general.');
    final Finder appPowered = find.text('Powered by Flutter');

    expect(appName, findsNothing);
    expect(appIcon, findsOneWidget);
    expect(appVersion, findsNothing);
    expect(appLegalese, findsNothing);
    expect(appPowered, findsOneWidget);

    // Padding between app icon and app powered text.
    final double appIconBottomPadding =
        tester.getTopLeft(appPowered).dy - tester.getBottomLeft(appIcon).dy;
    expect(appIconBottomPadding, 18.0);
  });

  testWidgets('Material2 - Error handling test', (WidgetTester tester) async {
    LicenseRegistry.addLicense(() => Stream<LicenseEntry>.error(Exception('Injected failure')));
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const Material(child: AboutListTile()),
      ),
    );
    await tester.tap(find.byType(ListTile));
    await tester.pump();
    await tester.pump(const Duration(seconds: 2));
    await tester.tap(find.text('VIEW LICENSES'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 2));
    final Finder finder = find.byWidgetPredicate(
      (Widget widget) => widget.runtimeType.toString() == '_PackagesView',
    );
    // force the stream to complete (has to be done in a runAsync block since it's areal async process)
    await tester.runAsync(() => (tester.firstState(finder) as dynamic).licenses as Future<dynamic>);
    expect(tester.takeException().toString(), 'Exception: Injected failure');
    await tester.pumpAndSettle();
    expect(tester.takeException().toString(), 'Exception: Injected failure');
    expect(find.text('Exception: Injected failure'), findsOneWidget);
  });

  testWidgets('Material3 - Error handling test', (WidgetTester tester) async {
    LicenseRegistry.addLicense(() => Stream<LicenseEntry>.error(Exception('Injected failure')));
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: const Material(child: AboutListTile()),
      ),
    );
    await tester.tap(find.byType(ListTile));
    await tester.pump();
    await tester.pump(const Duration(seconds: 2));
    await tester.tap(find.text('View licenses'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 2));
    final Finder finder = find.byWidgetPredicate(
      (Widget widget) => widget.runtimeType.toString() == '_PackagesView',
    );
    // force the stream to complete (has to be done in a runAsync block since it's areal async process)
    await tester.runAsync(() => (tester.firstState(finder) as dynamic).licenses as Future<dynamic>);
    expect(tester.takeException().toString(), 'Exception: Injected failure');
    await tester.pumpAndSettle();
    expect(tester.takeException().toString(), 'Exception: Injected failure');
    expect(find.text('Exception: Injected failure'), findsOneWidget);
  });

  testWidgets('Material2 - LicensePage master view layout position - ltr', (
    WidgetTester tester,
  ) async {
    const TextDirection textDirection = TextDirection.ltr;
    const Size defaultSize = Size(800.0, 600.0);
    const Size wideSize = Size(1200.0, 600.0);
    const String title = 'License ABC';
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['ABC'], 'DEF'),
      ]);
    });

    addTearDown(() async {
      await tester.binding.setSurfaceSize(null);
    });

    // Configure to show the default layout.
    await tester.binding.setSurfaceSize(defaultSize);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        title: title,
        home: const Scaffold(
          body: Directionality(textDirection: textDirection, child: LicensePage()),
        ),
      ),
    );

    await tester.pumpAndSettle(); // Finish rendering the page.

    // If the layout width is less than 840.0 pixels, nested layout is
    // used which positions license page title at the top center.
    Offset titleOffset = tester.getCenter(find.text(title));
    expect(titleOffset, Offset(defaultSize.width / 2, 92.0));
    expect(tester.getCenter(find.byType(ListView)), Offset(defaultSize.width / 2, 328.0));

    // Configure a wide window to show the lateral UI.
    await tester.binding.setSurfaceSize(wideSize);

    await tester.pumpWidget(
      const MaterialApp(
        title: title,
        home: Scaffold(body: Directionality(textDirection: textDirection, child: LicensePage())),
      ),
    );

    await tester.pumpAndSettle(); // Finish rendering the page.

    // If the layout width is greater than 840.0 pixels, lateral UI layout
    // is used which positions license page title and packageList
    // at the top left.
    titleOffset = tester.getTopRight(find.text(title));
    expect(titleOffset, const Offset(292.0, 136.0));
    expect(titleOffset.dx, lessThan(wideSize.width - 320)); // Default master view width is 320.0.
    expect(tester.getCenter(find.byType(ListView)), const Offset(160, 356));
  });

  testWidgets('Material3 - LicensePage master view layout position - ltr', (
    WidgetTester tester,
  ) async {
    const TextDirection textDirection = TextDirection.ltr;
    const Size defaultSize = Size(800.0, 600.0);
    const Size wideSize = Size(1200.0, 600.0);
    const String title = 'License ABC';
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['ABC'], 'DEF'),
      ]);
    });

    addTearDown(() async {
      await tester.binding.setSurfaceSize(null);
    });

    // Configure to show the default layout.
    await tester.binding.setSurfaceSize(defaultSize);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        title: title,
        home: const Scaffold(
          body: Directionality(textDirection: textDirection, child: LicensePage()),
        ),
      ),
    );

    await tester.pumpAndSettle(); // Finish rendering the page.

    // If the layout width is less than 840.0 pixels, nested layout is
    // used which positions license page title at the top center.
    Offset titleOffset = tester.getCenter(find.text(title));
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(titleOffset, Offset(defaultSize.width / 2, 96.0));
    }
    expect(tester.getCenter(find.byType(ListView)), Offset(defaultSize.width / 2, 328.0));

    // Configure a wide window to show the lateral UI.
    await tester.binding.setSurfaceSize(wideSize);

    await tester.pumpWidget(
      const MaterialApp(
        title: title,
        home: Scaffold(body: Directionality(textDirection: textDirection, child: LicensePage())),
      ),
    );

    await tester.pumpAndSettle(); // Finish rendering the page.

    // If the layout width is greater than 840.0 pixels, lateral UI layout
    // is used which positions license page title and packageList
    // at the top left.
    titleOffset = tester.getTopRight(find.text(title));
    expect(titleOffset, const Offset(292.0, 136.0));
    expect(titleOffset.dx, lessThan(wideSize.width - 320)); // Default master view width is 320.0.
    expect(tester.getCenter(find.byType(ListView)), const Offset(160, 356));
  });

  testWidgets('Material2 - LicensePage master view layout position - rtl', (
    WidgetTester tester,
  ) async {
    const TextDirection textDirection = TextDirection.rtl;
    const Size defaultSize = Size(800.0, 600.0);
    const Size wideSize = Size(1200.0, 600.0);
    const String title = 'License ABC';
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['ABC'], 'DEF'),
      ]);
    });

    addTearDown(() async {
      await tester.binding.setSurfaceSize(null);
    });

    // Configure to show the default layout.
    await tester.binding.setSurfaceSize(defaultSize);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        title: title,
        home: const Scaffold(
          body: Directionality(textDirection: textDirection, child: LicensePage()),
        ),
      ),
    );

    await tester.pumpAndSettle(); // Finish rendering the page.

    // If the layout width is less than 840.0 pixels, nested layout is
    // used which positions license page title at the top center.
    Offset titleOffset = tester.getCenter(find.text(title));
    expect(titleOffset, Offset(defaultSize.width / 2, 92.0));
    expect(tester.getCenter(find.byType(ListView)), Offset(defaultSize.width / 2, 328.0));

    // Configure a wide window to show the lateral UI.
    await tester.binding.setSurfaceSize(wideSize);

    await tester.pumpWidget(
      const MaterialApp(
        title: title,
        home: Scaffold(body: Directionality(textDirection: textDirection, child: LicensePage())),
      ),
    );

    await tester.pumpAndSettle(); // Finish rendering the page.

    // If the layout width is greater than 840.0 pixels, lateral UI layout
    // is used which positions license page title and packageList
    // at the top right.
    titleOffset = tester.getTopLeft(find.text(title));
    expect(titleOffset, const Offset(908.0, 136.0));
    expect(
      titleOffset.dx,
      greaterThan(wideSize.width - 320),
    ); // Default master view width is 320.0.
    expect(tester.getCenter(find.byType(ListView)), const Offset(1040.0, 356.0));
  });

  testWidgets('Material3 - LicensePage master view layout position - rtl', (
    WidgetTester tester,
  ) async {
    const TextDirection textDirection = TextDirection.rtl;
    const Size defaultSize = Size(800.0, 600.0);
    const Size wideSize = Size(1200.0, 600.0);
    const String title = 'License ABC';
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['ABC'], 'DEF'),
      ]);
    });

    addTearDown(() async {
      await tester.binding.setSurfaceSize(null);
    });

    // Configure to show the default layout.
    await tester.binding.setSurfaceSize(defaultSize);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        title: title,
        home: const Scaffold(
          body: Directionality(textDirection: textDirection, child: LicensePage()),
        ),
      ),
    );

    await tester.pumpAndSettle(); // Finish rendering the page.

    // If the layout width is less than 840.0 pixels, nested layout is
    // used which positions license page title at the top center.
    Offset titleOffset = tester.getCenter(find.text(title));
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(titleOffset, Offset(defaultSize.width / 2, 96.0));
    }
    expect(tester.getCenter(find.byType(ListView)), Offset(defaultSize.width / 2, 328.0));

    // Configure a wide window to show the lateral UI.
    await tester.binding.setSurfaceSize(wideSize);

    await tester.pumpWidget(
      const MaterialApp(
        title: title,
        home: Scaffold(body: Directionality(textDirection: textDirection, child: LicensePage())),
      ),
    );

    await tester.pumpAndSettle(); // Finish rendering the page.

    // If the layout width is greater than 840.0 pixels, lateral UI layout
    // is used which positions license page title and packageList
    // at the top right.
    titleOffset = tester.getTopLeft(find.text(title));
    expect(titleOffset, const Offset(908.0, 136.0));
    expect(
      titleOffset.dx,
      greaterThan(wideSize.width - 320),
    ); // Default master view width is 320.0.
    expect(tester.getCenter(find.byType(ListView)), const Offset(1040.0, 356.0));
  });

  testWidgets('License page title in lateral UI does not use AppBarTheme.foregroundColor', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/108991
    final ThemeData theme = ThemeData(
      appBarTheme: const AppBarTheme(foregroundColor: Color(0xFFFFFFFF)),
      useMaterial3: true,
    );
    const String title = 'License ABC';
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['ABC'], 'DEF'),
      ]);
    });

    addTearDown(() async {
      await tester.binding.setSurfaceSize(null);
    });

    // Configure a wide window to show the lateral UI.
    await tester.binding.setSurfaceSize(const Size(1200.0, 600.0));

    await tester.pumpWidget(
      MaterialApp(title: title, theme: theme, home: const Scaffold(body: LicensePage())),
    );

    await tester.pumpAndSettle(); // Finish rendering the page.

    final RenderParagraph renderParagraph =
        tester.renderObject(find.text('ABC').last) as RenderParagraph;

    // License page title should not use AppBarTheme's foregroundColor.
    expect(renderParagraph.text.style!.color, isNot(theme.appBarTheme.foregroundColor));

    // License page title in the lateral UI uses default text style color.
    expect(renderParagraph.text.style!.color, theme.textTheme.titleLarge!.color);
  });

  testWidgets('License page default title text color in the nested UI', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/108991
    final ThemeData theme = ThemeData(useMaterial3: true);
    const String title = 'License ABC';
    LicenseRegistry.addLicense(() {
      return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
        const LicenseEntryWithLineBreaks(<String>['ABC'], 'DEF'),
      ]);
    });

    await tester.pumpWidget(
      MaterialApp(title: title, theme: theme, home: const Scaffold(body: LicensePage())),
    );

    await tester.pumpAndSettle(); // Finish rendering the page.

    // Currently in the master view.
    expect(find.text('License ABC'), findsOneWidget);

    // Navigate to the license page.
    await tester.tap(find.text('ABC'));
    await tester.pumpAndSettle();

    // Master view is no longer visible.
    expect(find.text('License ABC'), findsNothing);

    final RenderParagraph renderParagraph =
        tester.renderObject(find.text('ABC').first) as RenderParagraph;
    expect(renderParagraph.text.style!.color, theme.textTheme.titleLarge!.color);
  });

  group('Material 2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    testWidgets('License page default title text color in the nested UI', (
      WidgetTester tester,
    ) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/108991
      final ThemeData theme = ThemeData(useMaterial3: false);
      const String title = 'License ABC';
      LicenseRegistry.addLicense(() {
        return Stream<LicenseEntry>.fromIterable(<LicenseEntry>[
          const LicenseEntryWithLineBreaks(<String>['ABC'], 'DEF'),
        ]);
      });

      await tester.pumpWidget(
        MaterialApp(title: title, theme: theme, home: const Scaffold(body: LicensePage())),
      );

      await tester.pumpAndSettle(); // Finish rendering the page.

      // Currently in the master view.
      expect(find.text('License ABC'), findsOneWidget);

      // Navigate to the license page.
      await tester.tap(find.text('ABC'));
      await tester.pumpAndSettle();

      // Master view is no longer visible.
      expect(find.text('License ABC'), findsNothing);

      final RenderParagraph renderParagraph =
          tester.renderObject(find.text('ABC').first) as RenderParagraph;
      expect(renderParagraph.text.style!.color, theme.primaryTextTheme.titleLarge!.color);
    });
  });

  testWidgets('Adaptive AboutDialog shows correct widget on each platform', (
    WidgetTester tester,
  ) async {
    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: platform),
          home: const Material(
            child: Center(child: ElevatedButton(onPressed: null, child: Text('Go'))),
          ),
        ),
      );

      final BuildContext context = tester.element(find.text('Go'));

      showAdaptiveAboutDialog(
        context: context,
        applicationIcon: const Icon(Icons.abc),
        applicationName: 'Test',
        applicationVersion: '1.0.0',
        applicationLegalese: 'Application Legalese',
        children: <Widget>[const Text('Test1')],
      );

      await tester.pumpAndSettle(const Duration(seconds: 1));
      expect(find.byType(CupertinoDialogAction), findsWidgets);
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: platform),
          home: const Material(
            child: Center(child: ElevatedButton(onPressed: null, child: Text('Go'))),
          ),
        ),
      );

      final BuildContext context = tester.element(find.text('Go'));

      showAboutDialog(
        context: context,
        applicationIcon: const Icon(Icons.abc),
        applicationName: 'Test',
        applicationVersion: '1.0.0',
      );

      await tester.pumpAndSettle(const Duration(seconds: 1));
      expect(find.byType(CupertinoDialogAction), findsNothing);
    }
  });

  testWidgets('Adaptive AboutDialog closes correctly on each platform', (
    WidgetTester tester,
  ) async {
    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: platform),
          home: const Material(
            child: Center(child: ElevatedButton(onPressed: null, child: Text('Go'))),
          ),
        ),
      );

      final BuildContext context = tester.element(find.text('Go'));

      showAdaptiveAboutDialog(
        context: context,
        applicationName: 'Test',
        applicationVersion: '1.0.0',
      );

      await tester.pumpAndSettle(const Duration(seconds: 1));
      expect(find.byType(CupertinoDialogAction), findsWidgets);

      await tester.tap(find.text('Close'));
      await tester.pumpAndSettle();
      expect(find.byType(CupertinoAlertDialog), findsNothing);
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: platform),
          home: const Material(
            child: Center(child: ElevatedButton(onPressed: null, child: Text('Go'))),
          ),
        ),
      );

      final BuildContext context = tester.element(find.text('Go'));

      showAdaptiveAboutDialog(
        context: context,
        applicationName: 'Test',
        applicationVersion: '1.0.0',
      );

      await tester.pumpAndSettle(const Duration(seconds: 1));
      expect(find.byType(TextButton), findsWidgets);

      await tester.tap(find.text('Close'));
      await tester.pumpAndSettle();
      expect(find.byType(AlertDialog), findsNothing);
    }
  });
}

class FakeLicenseEntry extends LicenseEntry {
  FakeLicenseEntry();

  bool get packagesCalled => _packagesCalled;
  bool _packagesCalled = false;

  @override
  Iterable<LicenseParagraph> paragraphs = <LicenseParagraph>[];

  @override
  Iterable<String> get packages {
    _packagesCalled = true;
    return <String>[];
  }
}

class LicensePageObserver extends NavigatorObserver {
  int licensePageCount = 0;

  @override
  void didPush(Route<dynamic> route, Route<dynamic>? previousRoute) {
    if (route is MaterialPageRoute<dynamic>) {
      licensePageCount++;
    }
    super.didPush(route, previousRoute);
  }
}

class AboutDialogObserver extends NavigatorObserver {
  int dialogCount = 0;

  @override
  void didPush(Route<dynamic> route, Route<dynamic>? previousRoute) {
    if (route is DialogRoute) {
      dialogCount++;
    }
    super.didPush(route, previousRoute);
  }

  @override
  void didPop(Route<dynamic> route, Route<dynamic>? previousRoute) {
    if (route is DialogRoute) {
      dialogCount--;
    }
    super.didPop(route, previousRoute);
  }
}
