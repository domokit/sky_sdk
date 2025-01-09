// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'backdrop.dart';
import 'category_menu_page.dart';
import 'colors.dart';
import 'expanding_bottom_sheet.dart';
import 'home.dart';
import 'login.dart';
import 'supplemental/cut_corners_border.dart';

class ShrineApp extends StatefulWidget {
  const ShrineApp({super.key});

  @override
  State<ShrineApp> createState() => _ShrineAppState();
}

class _ShrineAppState extends State<ShrineApp> with SingleTickerProviderStateMixin {
  // Controller to coordinate both the opening/closing of backdrop and sliding
  // of expanding bottom sheet
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 450),
      value: 1.0,
    );
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      // The automatically applied scrollbars on desktop can cause a crash for
      // demos where many scrollables are all attached to the same
      // PrimaryScrollController. The gallery needs to be migrated before
      // enabling this. https://github.com/flutter/gallery/issues/523
      scrollBehavior: const MaterialScrollBehavior().copyWith(scrollbars: false),
      title: 'Shrine',
      home: HomePage(
        backdrop: Backdrop(
          frontLayer: const ProductPage(),
          backLayer: CategoryMenuPage(onCategoryTap: () => _controller.forward()),
          frontTitle: const Text('SHRINE'),
          backTitle: const Text('MENU'),
          controller: _controller,
        ),
        expandingBottomSheet: ExpandingBottomSheet(hideController: _controller),
      ),
      initialRoute: '/login',
      onGenerateRoute: _getRoute,
      // Copy the platform from the main theme in order to support platform
      // toggling from the Gallery options menu.
      theme: _kShrineTheme.copyWith(platform: Theme.of(context).platform),
    );
  }
}

Route<dynamic>? _getRoute(RouteSettings settings) {
  if (settings.name != '/login') {
    return null;
  }

  return MaterialPageRoute<void>(
    settings: settings,
    builder: (BuildContext context) => const LoginPage(),
    fullscreenDialog: true,
  );
}

final ThemeData _kShrineTheme = _buildShrineTheme();

IconThemeData _customIconTheme(IconThemeData original) {
  return original.copyWith(color: kShrineBrown900);
}

ThemeData _buildShrineTheme() {
  final ThemeData base = ThemeData.light();
  return base.copyWith(
    colorScheme: kShrineColorScheme,
    primaryColor: kShrinePink100,
    scaffoldBackgroundColor: kShrineBackgroundWhite,
    cardColor: kShrineBackgroundWhite,
    primaryIconTheme: _customIconTheme(base.iconTheme),
    inputDecorationTheme: const InputDecorationTheme(border: CutCornersBorder()),
    textTheme: _buildShrineTextTheme(base.textTheme),
    primaryTextTheme: _buildShrineTextTheme(base.primaryTextTheme),
    iconTheme: _customIconTheme(base.iconTheme),
    appBarTheme: const AppBarTheme(backgroundColor: kShrinePink100),
  );
}

TextTheme _buildShrineTextTheme(TextTheme base) {
  return base
      .copyWith(
        headlineSmall: base.headlineSmall!.copyWith(fontWeight: FontWeight.w500),
        titleLarge: base.titleLarge!.copyWith(fontSize: 18.0),
        bodySmall: base.bodySmall!.copyWith(fontWeight: FontWeight.w400, fontSize: 14.0),
        bodyLarge: base.bodyLarge!.copyWith(fontWeight: FontWeight.w500, fontSize: 16.0),
        labelLarge: base.labelLarge!.copyWith(fontWeight: FontWeight.w500, fontSize: 14.0),
      )
      .apply(fontFamily: 'Raleway', displayColor: kShrineBrown900, bodyColor: kShrineBrown900);
}

const ColorScheme kShrineColorScheme = ColorScheme(
  primary: kShrinePink100,
  secondary: kShrinePink50,
  surface: kShrineSurfaceWhite,
  error: kShrineErrorRed,
  onPrimary: kShrineBrown900,
  onSecondary: kShrineBrown900,
  onSurface: kShrineBrown900,
  onError: kShrineSurfaceWhite,
  brightness: Brightness.light,
);
