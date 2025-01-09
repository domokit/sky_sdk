// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Material2 - Text baseline with CJK locale', (WidgetTester tester) async {
    // This test in combination with 'Text baseline with EN locale' verify the baselines
    // used to align text with ideographic baselines are reasonable. We are currently
    // using the alphabetic baseline to lay out as the ideographic baseline is not yet
    // properly implemented. When the ideographic baseline is better defined and implemented,
    // the values of this test should change very slightly. See the issue this is based off
    // of: https://github.com/flutter/flutter/issues/25782.
    final Key targetKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        routes: <String, WidgetBuilder>{
          '/next': (BuildContext context) {
            return const Text('Next');
          },
        },
        localizationsDelegates: GlobalMaterialLocalizations.delegates,
        supportedLocales: const <Locale>[
          Locale('en', 'US'),
          Locale('es', 'ES'),
          Locale('zh', 'CN'),
        ],
        locale: const Locale('zh', 'CN'),
        home: Material(
          child: Center(
            child: Builder(
              key: targetKey,
              builder: (BuildContext context) {
                return PopupMenuButton<int>(
                  onSelected: (int value) {
                    Navigator.pushNamed(context, '/next');
                  },
                  itemBuilder: (BuildContext context) {
                    return <PopupMenuItem<int>>[
                      const PopupMenuItem<int>(
                        value: 1,
                        child: Text('hello, world', style: TextStyle(color: Colors.blue)),
                      ),
                      const PopupMenuItem<int>(
                        value: 2,
                        child: Text('你好，世界', style: TextStyle(color: Colors.blue)),
                      ),
                    ];
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(targetKey));
    await tester.pumpAndSettle();

    expect(find.text('hello, world'), findsOneWidget);
    expect(find.text('你好，世界'), findsOneWidget);

    expect(tester.getTopLeft(find.text('hello, world')).dy, 299.5);
    expect(tester.getBottomLeft(find.text('hello, world')).dy, 316.5);

    expect(tester.getTopLeft(find.text('你好，世界')).dy, 347.5);
    expect(tester.getBottomLeft(find.text('你好，世界')).dy, 364.5);
  });

  testWidgets('Material3 - Text baseline with CJK locale', (WidgetTester tester) async {
    // This test in combination with 'Text baseline with EN locale' verify the baselines
    // used to align text with ideographic baselines are reasonable. We are currently
    // using the alphabetic baseline to lay out as the ideographic baseline is not yet
    // properly implemented. When the ideographic baseline is better defined and implemented,
    // the values of this test should change very slightly. See the issue this is based off
    // of: https://github.com/flutter/flutter/issues/25782.
    final Key targetKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        routes: <String, WidgetBuilder>{
          '/next': (BuildContext context) {
            return const Text('Next');
          },
        },
        localizationsDelegates: GlobalMaterialLocalizations.delegates,
        supportedLocales: const <Locale>[
          Locale('en', 'US'),
          Locale('es', 'ES'),
          Locale('zh', 'CN'),
        ],
        locale: const Locale('zh', 'CN'),
        home: Material(
          child: Center(
            child: Builder(
              key: targetKey,
              builder: (BuildContext context) {
                return PopupMenuButton<int>(
                  onSelected: (int value) {
                    Navigator.pushNamed(context, '/next');
                  },
                  itemBuilder: (BuildContext context) {
                    return <PopupMenuItem<int>>[
                      const PopupMenuItem<int>(
                        value: 1,
                        child: Text('hello, world', style: TextStyle(color: Colors.blue)),
                      ),
                      const PopupMenuItem<int>(
                        value: 2,
                        child: Text('你好，世界', style: TextStyle(color: Colors.blue)),
                      ),
                    ];
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(targetKey));
    await tester.pumpAndSettle();

    expect(find.text('hello, world'), findsOneWidget);
    expect(find.text('你好，世界'), findsOneWidget);

    expect(tester.getTopLeft(find.text('hello, world')).dy, 298.0);
    expect(tester.getBottomLeft(find.text('hello, world')).dy, 318.0);

    expect(tester.getTopLeft(find.text('你好，世界')).dy, 346.0);
    expect(tester.getBottomLeft(find.text('你好，世界')).dy, 366.0);
  });

  testWidgets('Material2 - Text baseline with EN locale', (WidgetTester tester) async {
    // This test in combination with 'Text baseline with CJK locale' verify the baselines
    // used to align text with ideographic baselines are reasonable. We are currently
    // using the alphabetic baseline to lay out as the ideographic baseline is not yet
    // properly implemented. When the ideographic baseline is better defined and implemented,
    // the values of this test should change very slightly. See the issue this is based off
    // of: https://github.com/flutter/flutter/issues/25782.
    final Key targetKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        routes: <String, WidgetBuilder>{
          '/next': (BuildContext context) {
            return const Text('Next');
          },
        },
        localizationsDelegates: GlobalMaterialLocalizations.delegates,
        supportedLocales: const <Locale>[
          Locale('en', 'US'),
          Locale('es', 'ES'),
          Locale('zh', 'CN'),
        ],
        locale: const Locale('en', 'US'),
        home: Material(
          child: Center(
            child: Builder(
              key: targetKey,
              builder: (BuildContext context) {
                return PopupMenuButton<int>(
                  onSelected: (int value) {
                    Navigator.pushNamed(context, '/next');
                  },
                  itemBuilder: (BuildContext context) {
                    return <PopupMenuItem<int>>[
                      const PopupMenuItem<int>(
                        value: 1,
                        child: Text('hello, world', style: TextStyle(color: Colors.blue)),
                      ),
                      const PopupMenuItem<int>(
                        value: 2,
                        child: Text('你好，世界', style: TextStyle(color: Colors.blue)),
                      ),
                    ];
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(targetKey));
    await tester.pumpAndSettle();

    expect(find.text('hello, world'), findsOneWidget);
    expect(find.text('你好，世界'), findsOneWidget);

    expect(tester.getTopLeft(find.text('hello, world')).dy, 300.0);
    expect(tester.getBottomLeft(find.text('hello, world')).dy, 316.0);

    expect(tester.getTopLeft(find.text('你好，世界')).dy, 348.0);
    expect(tester.getBottomLeft(find.text('你好，世界')).dy, 364.0);
  });

  testWidgets('Material3 - Text baseline with EN locale', (WidgetTester tester) async {
    // This test in combination with 'Text baseline with CJK locale' verify the baselines
    // used to align text with ideographic baselines are reasonable. We are currently
    // using the alphabetic baseline to lay out as the ideographic baseline is not yet
    // properly implemented. When the ideographic baseline is better defined and implemented,
    // the values of this test should change very slightly. See the issue this is based off
    // of: https://github.com/flutter/flutter/issues/25782.
    final Key targetKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        routes: <String, WidgetBuilder>{
          '/next': (BuildContext context) {
            return const Text('Next');
          },
        },
        localizationsDelegates: GlobalMaterialLocalizations.delegates,
        supportedLocales: const <Locale>[
          Locale('en', 'US'),
          Locale('es', 'ES'),
          Locale('zh', 'CN'),
        ],
        locale: const Locale('en', 'US'),
        home: Material(
          child: Center(
            child: Builder(
              key: targetKey,
              builder: (BuildContext context) {
                return PopupMenuButton<int>(
                  onSelected: (int value) {
                    Navigator.pushNamed(context, '/next');
                  },
                  itemBuilder: (BuildContext context) {
                    return <PopupMenuItem<int>>[
                      const PopupMenuItem<int>(
                        value: 1,
                        child: Text('hello, world', style: TextStyle(color: Colors.blue)),
                      ),
                      const PopupMenuItem<int>(
                        value: 2,
                        child: Text('你好，世界', style: TextStyle(color: Colors.blue)),
                      ),
                    ];
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(targetKey));
    await tester.pumpAndSettle();

    expect(find.text('hello, world'), findsOneWidget);
    expect(find.text('你好，世界'), findsOneWidget);

    expect(tester.getTopLeft(find.text('hello, world')).dy, 298.0);
    expect(tester.getBottomLeft(find.text('hello, world')).dy, 318.0);

    expect(tester.getTopLeft(find.text('你好，世界')).dy, 346.0);
    expect(tester.getBottomLeft(find.text('你好，世界')).dy, 366.0);
  });
}
