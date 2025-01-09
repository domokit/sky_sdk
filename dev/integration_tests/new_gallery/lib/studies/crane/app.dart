// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../data/gallery_options.dart';
import '../../gallery_localizations.dart';
import 'backdrop.dart';
import 'backlayer.dart';
import 'eat_form.dart';
import 'fly_form.dart';
import 'routes.dart' as routes;
import 'sleep_form.dart';
import 'theme.dart';

class CraneApp extends StatelessWidget {
  const CraneApp({super.key});

  static const String defaultRoute = routes.defaultRoute;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      restorationScopeId: 'crane_app',
      title: 'Crane',
      debugShowCheckedModeBanner: false,
      localizationsDelegates: GalleryLocalizations.localizationsDelegates,
      supportedLocales: GalleryLocalizations.supportedLocales,
      locale: GalleryOptions.of(context).locale,
      initialRoute: CraneApp.defaultRoute,
      routes: <String, WidgetBuilder>{
        CraneApp.defaultRoute: (BuildContext context) => const _Home(),
      },
      theme: craneTheme.copyWith(platform: GalleryOptions.of(context).platform),
    );
  }
}

class _Home extends StatelessWidget {
  const _Home();

  @override
  Widget build(BuildContext context) {
    return const ApplyTextOptions(
      child: Backdrop(
        frontLayer: SizedBox(),
        backLayerItems: <BackLayerItem>[FlyForm(), SleepForm(), EatForm()],
        frontTitle: Text('CRANE'),
        backTitle: Text('MENU'),
      ),
    );
  }
}
