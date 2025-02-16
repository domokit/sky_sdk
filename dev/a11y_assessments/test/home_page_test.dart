// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:a11y_assessments/main.dart';
import 'package:a11y_assessments/use_cases/action_chip.dart';
import 'package:a11y_assessments/use_cases/auto_complete.dart';
import 'package:a11y_assessments/use_cases/badge.dart';
import 'package:a11y_assessments/use_cases/card.dart';
import 'package:a11y_assessments/use_cases/check_box_list_tile.dart';
import 'package:a11y_assessments/use_cases/date_picker.dart';
import 'package:a11y_assessments/use_cases/dialog.dart';
import 'package:a11y_assessments/use_cases/drawer.dart';
import 'package:a11y_assessments/use_cases/expansion_tile.dart';
import 'package:a11y_assessments/use_cases/material_banner.dart';
import 'package:a11y_assessments/use_cases/navigation_bar.dart';
import 'package:a11y_assessments/use_cases/navigation_drawer.dart';
import 'package:a11y_assessments/use_cases/navigation_rail.dart';
import 'package:a11y_assessments/use_cases/radio_list_tile.dart';
import 'package:a11y_assessments/use_cases/slider.dart';
import 'package:a11y_assessments/use_cases/snack_bar.dart';
import 'package:a11y_assessments/use_cases/switch_list_tile.dart';
import 'package:a11y_assessments/use_cases/text_button.dart';
import 'package:a11y_assessments/use_cases/text_field.dart';
import 'package:a11y_assessments/use_cases/text_field_password.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:material_color_utilities/material_color_utilities.dart';

import 'test_utils.dart';

void main() {
  testWidgets('Has light and dark theme', (WidgetTester tester) async {
    await tester.pumpWidget(const App());
    final MaterialApp app = find.byType(MaterialApp).evaluate().first.widget as MaterialApp;
    expect(app.theme!.brightness, equals(Brightness.light));
    expect(app.darkTheme!.brightness, equals(Brightness.dark));
  });

  testWidgets('App can generate high-contrast color scheme', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MediaQuery(data: MediaQueryData(highContrast: true), child: App()),
    );

    final MaterialApp app = find.byType(MaterialApp).evaluate().first.widget as MaterialApp;

    final DynamicScheme highContrastScheme = SchemeTonalSpot(
      sourceColorHct: Hct.fromInt(const Color(0xff6750a4).value),
      isDark: false,
      contrastLevel: 1.0,
    );
    final ColorScheme appScheme = app.theme!.colorScheme;

    expect(appScheme.primary.value, MaterialDynamicColors.primary.getArgb(highContrastScheme));
    expect(appScheme.onPrimary.value, MaterialDynamicColors.onPrimary.getArgb(highContrastScheme));
    expect(
      appScheme.primaryContainer.value,
      MaterialDynamicColors.primaryContainer.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onPrimaryContainer.value,
      MaterialDynamicColors.onPrimaryContainer.getArgb(highContrastScheme),
    );
    expect(
      appScheme.primaryFixed.value,
      MaterialDynamicColors.primaryFixed.getArgb(highContrastScheme),
    );
    expect(
      appScheme.primaryFixedDim.value,
      MaterialDynamicColors.primaryFixedDim.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onPrimaryFixed.value,
      MaterialDynamicColors.onPrimaryFixed.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onPrimaryFixedVariant.value,
      MaterialDynamicColors.onPrimaryFixedVariant.getArgb(highContrastScheme),
    );
    expect(appScheme.secondary.value, MaterialDynamicColors.secondary.getArgb(highContrastScheme));
    expect(
      appScheme.onSecondary.value,
      MaterialDynamicColors.onSecondary.getArgb(highContrastScheme),
    );
    expect(
      appScheme.secondaryContainer.value,
      MaterialDynamicColors.secondaryContainer.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onSecondaryContainer.value,
      MaterialDynamicColors.onSecondaryContainer.getArgb(highContrastScheme),
    );
    expect(
      appScheme.secondaryFixed.value,
      MaterialDynamicColors.secondaryFixed.getArgb(highContrastScheme),
    );
    expect(
      appScheme.secondaryFixedDim.value,
      MaterialDynamicColors.secondaryFixedDim.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onSecondaryFixed.value,
      MaterialDynamicColors.onSecondaryFixed.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onSecondaryFixedVariant.value,
      MaterialDynamicColors.onSecondaryFixedVariant.getArgb(highContrastScheme),
    );
    expect(appScheme.tertiary.value, MaterialDynamicColors.tertiary.getArgb(highContrastScheme));
    expect(
      appScheme.onTertiary.value,
      MaterialDynamicColors.onTertiary.getArgb(highContrastScheme),
    );
    expect(
      appScheme.tertiaryContainer.value,
      MaterialDynamicColors.tertiaryContainer.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onTertiaryContainer.value,
      MaterialDynamicColors.onTertiaryContainer.getArgb(highContrastScheme),
    );
    expect(
      appScheme.tertiaryFixed.value,
      MaterialDynamicColors.tertiaryFixed.getArgb(highContrastScheme),
    );
    expect(
      appScheme.tertiaryFixedDim.value,
      MaterialDynamicColors.tertiaryFixedDim.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onTertiaryFixed.value,
      MaterialDynamicColors.onTertiaryFixed.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onTertiaryFixedVariant.value,
      MaterialDynamicColors.onTertiaryFixedVariant.getArgb(highContrastScheme),
    );
    expect(appScheme.error.value, MaterialDynamicColors.error.getArgb(highContrastScheme));
    expect(appScheme.onError.value, MaterialDynamicColors.onError.getArgb(highContrastScheme));
    expect(
      appScheme.errorContainer.value,
      MaterialDynamicColors.errorContainer.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onErrorContainer.value,
      MaterialDynamicColors.onErrorContainer.getArgb(highContrastScheme),
    );
    expect(
      appScheme.background.value,
      MaterialDynamicColors.background.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onBackground.value,
      MaterialDynamicColors.onBackground.getArgb(highContrastScheme),
    );
    expect(appScheme.surface.value, MaterialDynamicColors.surface.getArgb(highContrastScheme));
    expect(
      appScheme.surfaceDim.value,
      MaterialDynamicColors.surfaceDim.getArgb(highContrastScheme),
    );
    expect(
      appScheme.surfaceBright.value,
      MaterialDynamicColors.surfaceBright.getArgb(highContrastScheme),
    );
    expect(
      appScheme.surfaceContainerLowest.value,
      MaterialDynamicColors.surfaceContainerLowest.getArgb(highContrastScheme),
    );
    expect(
      appScheme.surfaceContainerLow.value,
      MaterialDynamicColors.surfaceContainerLow.getArgb(highContrastScheme),
    );
    expect(
      appScheme.surfaceContainer.value,
      MaterialDynamicColors.surfaceContainer.getArgb(highContrastScheme),
    );
    expect(
      appScheme.surfaceContainerHigh.value,
      MaterialDynamicColors.surfaceContainerHigh.getArgb(highContrastScheme),
    );
    expect(
      appScheme.surfaceContainerHighest.value,
      MaterialDynamicColors.surfaceContainerHighest.getArgb(highContrastScheme),
    );
    expect(appScheme.onSurface.value, MaterialDynamicColors.onSurface.getArgb(highContrastScheme));
    expect(
      appScheme.surfaceVariant.value,
      MaterialDynamicColors.surfaceVariant.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onSurfaceVariant.value,
      MaterialDynamicColors.onSurfaceVariant.getArgb(highContrastScheme),
    );
    expect(appScheme.outline.value, MaterialDynamicColors.outline.getArgb(highContrastScheme));
    expect(
      appScheme.outlineVariant.value,
      MaterialDynamicColors.outlineVariant.getArgb(highContrastScheme),
    );
    expect(appScheme.shadow.value, MaterialDynamicColors.shadow.getArgb(highContrastScheme));
    expect(appScheme.scrim.value, MaterialDynamicColors.scrim.getArgb(highContrastScheme));
    expect(
      appScheme.inverseSurface.value,
      MaterialDynamicColors.inverseSurface.getArgb(highContrastScheme),
    );
    expect(
      appScheme.onInverseSurface.value,
      MaterialDynamicColors.inverseOnSurface.getArgb(highContrastScheme),
    );
    expect(
      appScheme.inversePrimary.value,
      MaterialDynamicColors.inversePrimary.getArgb(highContrastScheme),
    );
  });

  testWidgets('Each A11y Assessments page has a unique page title.', (WidgetTester tester) async {
    final List<MethodCall> log = <MethodCall>[];

    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
      MethodCall methodCall,
    ) async {
      if (methodCall.method == 'SystemChrome.setApplicationSwitcherDescription') {
        log.add(methodCall);
      }
      return null;
    });

    await tester.pumpWidget(
      Title(color: const Color(0xFF00FF00), title: 'Accessibility Assessments', child: Container()),
    );
    expect(
      log[0],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{
          'label': 'Accessibility Assessments',
          'primaryColor': 4278255360,
        },
      ),
    );

    await pumpsUseCase(tester, AutoCompleteUseCase());
    expect(
      log[2],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'AutoComplete', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, ActionChipUseCase());
    expect(
      log[4],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'ActionChip', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, BadgeUseCase());
    expect(
      log[6],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'Badge', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, CardUseCase());
    expect(
      log[8],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'Card', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, CheckBoxListTile());
    expect(
      log[10],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'CheckBoxListTile', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, DatePickerUseCase());
    expect(
      log[12],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'DatePicker', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, DialogUseCase());
    expect(
      log[14],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'Dialog', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, ExpansionTileUseCase());
    expect(
      log[16],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'ExpansionTile', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, MaterialBannerUseCase());
    expect(
      log[18],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'MaterialBanner', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, NavigationBarUseCase());
    expect(
      log[20],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'NavigationBar', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, RadioListTileUseCase());
    expect(
      log[22],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'RadioListTile', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, SliderUseCase());
    expect(
      log[24],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'Slider', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, SnackBarUseCase());
    expect(
      log[26],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'SnackBar', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, SwitchListTileUseCase());
    expect(
      log[28],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'SwitchListTile', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, TextButtonUseCase());
    expect(
      log[30],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'TextButton', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, TextFieldUseCase());
    expect(
      log[32],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'TextField', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, TextFieldPasswordUseCase());
    expect(
      log[34],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'TextField password', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, NavigationDrawerUseCase());
    expect(
      log[36],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'NavigationDrawer', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, NavigationRailUseCase());
    expect(
      log[38],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'NavigationRail', 'primaryColor': 4284960932},
      ),
    );

    await pumpsUseCase(tester, DrawerUseCase());
    expect(
      log[40],
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'drawer', 'primaryColor': 4284960932},
      ),
    );
  });

  testWidgets('a11y assessments home page has one h1 tag', (WidgetTester tester) async {
    await tester.pumpWidget(const App());
    final Finder findHeadingLevelOnes = find.bySemanticsLabel('Accessibility Assessments');
    await tester.pumpAndSettle();
    expect(findHeadingLevelOnes, findsOne);
  });
}
