// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

const Color kTitleColor = Color(0xFF333333);
const String kTitleString = 'Hello World';

Future<void> pumpApp(WidgetTester tester, {GenerateAppTitle? onGenerateTitle, Color? color}) async {
  await tester.pumpWidget(
    WidgetsApp(
      supportedLocales: const <Locale>[Locale('en', 'US'), Locale('en', 'GB')],
      title: kTitleString,
      color: color ?? kTitleColor,
      onGenerateTitle: onGenerateTitle,
      onGenerateRoute: (RouteSettings settings) {
        return PageRouteBuilder<void>(
          pageBuilder: (
            BuildContext context,
            Animation<double> animation,
            Animation<double> secondaryAnimation,
          ) {
            return Container();
          },
        );
      },
    ),
  );
}

void main() {
  testWidgets('Specified title and color are used to build a Title', (WidgetTester tester) async {
    await pumpApp(tester);
    expect(tester.widget<Title>(find.byType(Title)).title, kTitleString);
    expect(tester.widget<Title>(find.byType(Title)).color, kTitleColor);
  });

  testWidgets('Specified color is made opaque for Title', (WidgetTester tester) async {
    // The Title widget can only handle fully opaque colors, the WidgetApp should
    // ensure it only uses a fully opaque version of its color for the title.
    const Color transparentBlue = Color(0xDD0000ff);
    const Color opaqueBlue = Color(0xFF0000ff);
    await pumpApp(tester, color: transparentBlue);
    expect(tester.widget<Title>(find.byType(Title)).color, opaqueBlue);
  });

  testWidgets('onGenerateTitle handles changing locales', (WidgetTester tester) async {
    String generateTitle(BuildContext context) {
      return Localizations.localeOf(context).toString();
    }

    await pumpApp(tester, onGenerateTitle: generateTitle);
    expect(tester.widget<Title>(find.byType(Title)).title, 'en_US');
    expect(tester.widget<Title>(find.byType(Title)).color, kTitleColor);

    await tester.binding.setLocale('en', 'GB');
    await tester.pump();
    expect(tester.widget<Title>(find.byType(Title)).title, 'en_GB');
    expect(tester.widget<Title>(find.byType(Title)).color, kTitleColor);

    // Not a supported locale, so we switch to supportedLocales[0], en_US
    await tester.binding.setLocale('fr', 'CA');
    await tester.pump();
    expect(tester.widget<Title>(find.byType(Title)).title, 'en_US');
    expect(tester.widget<Title>(find.byType(Title)).color, kTitleColor);
  });
}
