// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';
import 'dart:math';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/color_scheme/dynamic_content_color.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  final List<(ImageProvider<Object>, Brightness)> loadColorSchemeCalls =
      <(ImageProvider<Object>, Brightness)>[];

  Future<ColorScheme> fakeColorSchemeLoader(
    ImageProvider<Object> provider,
    Brightness brightness,
  ) async {
    loadColorSchemeCalls.add((provider, brightness));
    final int index = example.DynamicColorExample.images.indexOf(provider);
    final int seedColor = 0xf * pow(0x10, index).toInt();
    return ColorScheme.fromSeed(seedColor: Color(seedColor), brightness: brightness);
  }

  setUp(() {
    loadColorSchemeCalls.clear();
  });

  // The app being tested loads images via HTTP which the test
  // framework defeats by default.
  setUpAll(() {
    HttpOverrides.global = null;
  });

  testWidgets('The content is visible', (WidgetTester tester) async {
    await tester.pumpWidget(example.DynamicColorExample(loadColorScheme: fakeColorSchemeLoader));
    await tester.pump();

    expect(find.widgetWithText(AppBar, 'Content Based Dynamic Color'), findsOne);
    expect(find.byType(Switch), findsOne);
    expect(find.byIcon(Icons.light_mode), findsOne);

    expect(find.text('Light ColorScheme'), findsOne);
    expect(find.text('Dark ColorScheme'), findsOne);
    expect(find.text('primary'), findsExactly(2));
    expect(find.text('onPrimary'), findsExactly(2));
    expect(find.text('primaryContainer'), findsExactly(2));
    expect(find.text('onPrimaryContainer'), findsExactly(2));
    expect(find.text('secondary'), findsExactly(2));
    expect(find.text('onSecondary'), findsExactly(2));
    expect(find.text('secondaryContainer'), findsExactly(2));
    expect(find.text('onSecondaryContainer'), findsExactly(2));
    expect(find.text('tertiary'), findsExactly(2));
    expect(find.text('onTertiary'), findsExactly(2));
    expect(find.text('tertiaryContainer'), findsExactly(2));
    expect(find.text('onTertiaryContainer'), findsExactly(2));
    expect(find.text('error'), findsExactly(2));
    expect(find.text('onError'), findsExactly(2));
    expect(find.text('errorContainer'), findsExactly(2));
    expect(find.text('onErrorContainer'), findsExactly(2));
    expect(find.text('surface'), findsExactly(2));
    expect(find.text('onSurface'), findsExactly(2));
    expect(find.text('onSurfaceVariant'), findsExactly(2));
    expect(find.text('outline'), findsExactly(2));
    expect(find.text('shadow'), findsExactly(2));
    expect(find.text('inverseSurface'), findsExactly(2));
    expect(find.text('onInverseSurface'), findsExactly(2));
    expect(find.text('inversePrimary'), findsExactly(2));

    expect(loadColorSchemeCalls, hasLength(1));
    expect(
      loadColorSchemeCalls.single.$1,
      isA<NetworkImage>().having(
        (NetworkImage provider) => provider.url,
        'url',
        'https://flutter.github.io/assets-for-api-docs/assets/material/content_based_color_scheme_1.png',
      ),
    );
    expect(loadColorSchemeCalls.single.$2, Brightness.light);

    await tester.pumpAndSettle(); // Clears the timers from image loading.
  });

  testWidgets('The brightness can be changed', (WidgetTester tester) async {
    await tester.pumpWidget(example.DynamicColorExample(loadColorScheme: fakeColorSchemeLoader));
    await tester.pump();

    expect(loadColorSchemeCalls, hasLength(1));
    expect(
      loadColorSchemeCalls.single.$1,
      isA<NetworkImage>().having(
        (NetworkImage provider) => provider.url,
        'url',
        'https://flutter.github.io/assets-for-api-docs/assets/material/content_based_color_scheme_1.png',
      ),
    );
    expect(loadColorSchemeCalls.single.$2, Brightness.light);
    await tester.pump();
    await tester.pump(kThemeChangeDuration);

    ThemeData themeData = Theme.of(tester.element(find.byType(Scaffold)));

    expect(themeData.colorScheme.primary, const Color(0xff565992));
    expect(themeData.colorScheme.secondary, const Color(0xff5c5d72));

    await tester.tap(find.byType(Switch));
    await tester.pump();

    expect(loadColorSchemeCalls, hasLength(2));
    expect(
      loadColorSchemeCalls.last.$1,
      isA<NetworkImage>().having(
        (NetworkImage provider) => provider.url,
        'url',
        'https://flutter.github.io/assets-for-api-docs/assets/material/content_based_color_scheme_1.png',
      ),
    );
    expect(loadColorSchemeCalls.last.$2, Brightness.dark);

    await tester.pump(kThemeChangeDuration);

    themeData = Theme.of(tester.element(find.byType(Scaffold)));

    expect(themeData.colorScheme.primary, const Color(0xffbfc2ff));
    expect(themeData.colorScheme.secondary, const Color(0xffc5c4dd));
  });

  testWidgets('Tapping an image loads a new color scheme', (WidgetTester tester) async {
    await tester.pumpWidget(example.DynamicColorExample(loadColorScheme: fakeColorSchemeLoader));
    await tester.pump();

    await tester.tapAt(tester.getCenter(find.byType(Image).at(3)));
    await tester.pump();

    expect(loadColorSchemeCalls, hasLength(2));
    expect(
      loadColorSchemeCalls.last.$1,
      isA<NetworkImage>().having(
        (NetworkImage provider) => provider.url,
        'url',
        'https://flutter.github.io/assets-for-api-docs/assets/material/content_based_color_scheme_4.png',
      ),
    );
    expect(loadColorSchemeCalls.last.$2, Brightness.light);

    await tester.pump(kThemeChangeDuration);

    final ThemeData themeData = Theme.of(tester.element(find.byType(Scaffold)));

    expect(themeData.colorScheme.primary, const Color(0xff406836));
    expect(themeData.colorScheme.secondary, const Color(0xff54634d));
  });
}
