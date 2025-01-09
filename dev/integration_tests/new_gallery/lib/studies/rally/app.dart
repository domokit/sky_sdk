// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:animations/animations.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:google_fonts/google_fonts.dart';

import '../../data/gallery_options.dart';
import '../../gallery_localizations.dart';
import '../../layout/letter_spacing.dart';
import 'colors.dart';
import 'home.dart';
import 'login.dart';
import 'routes.dart' as routes;

/// The RallyApp is a MaterialApp with a theme and 2 routes.
///
/// The home route is the main page with tabs for sub pages.
/// The login route is the initial route.
class RallyApp extends StatelessWidget {
  const RallyApp({super.key});

  static const String loginRoute = routes.loginRoute;
  static const String homeRoute = routes.homeRoute;

  static const SharedAxisPageTransitionsBuilder sharedZAxisTransitionBuilder =
      SharedAxisPageTransitionsBuilder(
        fillColor: RallyColors.primaryBackground,
        transitionType: SharedAxisTransitionType.scaled,
      );

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      restorationScopeId: 'rally_app',
      title: 'Rally',
      debugShowCheckedModeBanner: false,
      theme: _buildRallyTheme().copyWith(
        platform: GalleryOptions.of(context).platform,
        pageTransitionsTheme: PageTransitionsTheme(
          builders: <TargetPlatform, PageTransitionsBuilder>{
            for (final TargetPlatform type in TargetPlatform.values)
              type: sharedZAxisTransitionBuilder,
          },
        ),
      ),
      localizationsDelegates: GalleryLocalizations.localizationsDelegates,
      supportedLocales: GalleryLocalizations.supportedLocales,
      locale: GalleryOptions.of(context).locale,
      initialRoute: loginRoute,
      routes: <String, WidgetBuilder>{
        homeRoute: (BuildContext context) => const HomePage(),
        loginRoute: (BuildContext context) => const LoginPage(),
      },
    );
  }

  ThemeData _buildRallyTheme() {
    final ThemeData base = ThemeData.dark();
    return ThemeData(
      appBarTheme: const AppBarTheme(
        systemOverlayStyle: SystemUiOverlayStyle.light,
        backgroundColor: RallyColors.primaryBackground,
        elevation: 0,
      ),
      scaffoldBackgroundColor: RallyColors.primaryBackground,
      focusColor: RallyColors.focusColor,
      textTheme: _buildRallyTextTheme(base.textTheme),
      inputDecorationTheme: const InputDecorationTheme(
        labelStyle: TextStyle(color: RallyColors.gray, fontWeight: FontWeight.w500),
        filled: true,
        fillColor: RallyColors.inputBackground,
        focusedBorder: InputBorder.none,
      ),
      visualDensity: VisualDensity.standard,
      colorScheme: base.colorScheme.copyWith(primary: RallyColors.primaryBackground),
    );
  }

  TextTheme _buildRallyTextTheme(TextTheme base) {
    return base
        .copyWith(
          bodyMedium: GoogleFonts.robotoCondensed(
            fontSize: 14,
            fontWeight: FontWeight.w400,
            letterSpacing: letterSpacingOrNone(0.5),
          ),
          bodyLarge: GoogleFonts.eczar(
            fontSize: 40,
            fontWeight: FontWeight.w400,
            letterSpacing: letterSpacingOrNone(1.4),
          ),
          labelLarge: GoogleFonts.robotoCondensed(
            fontWeight: FontWeight.w700,
            letterSpacing: letterSpacingOrNone(2.8),
          ),
          headlineSmall: GoogleFonts.eczar(
            fontSize: 40,
            fontWeight: FontWeight.w600,
            letterSpacing: letterSpacingOrNone(1.4),
          ),
        )
        .apply(displayColor: Colors.white, bodyColor: Colors.white);
  }
}
