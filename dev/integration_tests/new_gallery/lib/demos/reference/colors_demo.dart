// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery_localizations.dart';

// BEGIN colorsDemo

const double kColorItemHeight = 48;

class _Palette {
  _Palette({required this.name, required this.primary, this.accent, this.threshold = 900});

  final String name;
  final MaterialColor primary;
  final MaterialAccentColor? accent;

  // Titles for indices > threshold are white, otherwise black.
  final int threshold;
}

List<_Palette> _allPalettes(BuildContext context) {
  final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
  return <_Palette>[
    _Palette(
      name: localizations.colorsRed,
      primary: Colors.red,
      accent: Colors.redAccent,
      threshold: 300,
    ),
    _Palette(
      name: localizations.colorsPink,
      primary: Colors.pink,
      accent: Colors.pinkAccent,
      threshold: 200,
    ),
    _Palette(
      name: localizations.colorsPurple,
      primary: Colors.purple,
      accent: Colors.purpleAccent,
      threshold: 200,
    ),
    _Palette(
      name: localizations.colorsDeepPurple,
      primary: Colors.deepPurple,
      accent: Colors.deepPurpleAccent,
      threshold: 200,
    ),
    _Palette(
      name: localizations.colorsIndigo,
      primary: Colors.indigo,
      accent: Colors.indigoAccent,
      threshold: 200,
    ),
    _Palette(
      name: localizations.colorsBlue,
      primary: Colors.blue,
      accent: Colors.blueAccent,
      threshold: 400,
    ),
    _Palette(
      name: localizations.colorsLightBlue,
      primary: Colors.lightBlue,
      accent: Colors.lightBlueAccent,
      threshold: 500,
    ),
    _Palette(
      name: localizations.colorsCyan,
      primary: Colors.cyan,
      accent: Colors.cyanAccent,
      threshold: 600,
    ),
    _Palette(
      name: localizations.colorsTeal,
      primary: Colors.teal,
      accent: Colors.tealAccent,
      threshold: 400,
    ),
    _Palette(
      name: localizations.colorsGreen,
      primary: Colors.green,
      accent: Colors.greenAccent,
      threshold: 500,
    ),
    _Palette(
      name: localizations.colorsLightGreen,
      primary: Colors.lightGreen,
      accent: Colors.lightGreenAccent,
      threshold: 600,
    ),
    _Palette(
      name: localizations.colorsLime,
      primary: Colors.lime,
      accent: Colors.limeAccent,
      threshold: 800,
    ),
    _Palette(name: localizations.colorsYellow, primary: Colors.yellow, accent: Colors.yellowAccent),
    _Palette(name: localizations.colorsAmber, primary: Colors.amber, accent: Colors.amberAccent),
    _Palette(
      name: localizations.colorsOrange,
      primary: Colors.orange,
      accent: Colors.orangeAccent,
      threshold: 700,
    ),
    _Palette(
      name: localizations.colorsDeepOrange,
      primary: Colors.deepOrange,
      accent: Colors.deepOrangeAccent,
      threshold: 400,
    ),
    _Palette(name: localizations.colorsBrown, primary: Colors.brown, threshold: 200),
    _Palette(name: localizations.colorsGrey, primary: Colors.grey, threshold: 500),
    _Palette(name: localizations.colorsBlueGrey, primary: Colors.blueGrey, threshold: 500),
  ];
}

class _ColorItem extends StatelessWidget {
  const _ColorItem({required this.index, required this.color, this.prefix = ''});

  final int index;
  final Color color;
  final String prefix;

  String get _colorString => "#${color.value.toRadixString(16).padLeft(8, '0').toUpperCase()}";

  @override
  Widget build(BuildContext context) {
    return Semantics(
      container: true,
      child: Container(
        height: kColorItemHeight,
        padding: const EdgeInsets.symmetric(horizontal: 16),
        color: color,
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: <Widget>[Text('$prefix$index'), Flexible(child: Text(_colorString))],
        ),
      ),
    );
  }
}

class _PaletteTabView extends StatelessWidget {
  const _PaletteTabView({required this.colors});

  final _Palette colors;
  static const List<int> primaryKeys = <int>[50, 100, 200, 300, 400, 500, 600, 700, 800, 900];
  static const List<int> accentKeys = <int>[100, 200, 400, 700];

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    final TextStyle whiteTextStyle = textTheme.bodyMedium!.copyWith(color: Colors.white);
    final TextStyle blackTextStyle = textTheme.bodyMedium!.copyWith(color: Colors.black);
    return Scrollbar(
      child: ListView(
        itemExtent: kColorItemHeight,
        children: <Widget>[
          for (final int key in primaryKeys)
            DefaultTextStyle(
              style: key > colors.threshold ? whiteTextStyle : blackTextStyle,
              child: _ColorItem(index: key, color: colors.primary[key]!),
            ),
          if (colors.accent != null)
            for (final int key in accentKeys)
              DefaultTextStyle(
                style: key > colors.threshold ? whiteTextStyle : blackTextStyle,
                child: _ColorItem(index: key, color: colors.accent![key]!, prefix: 'A'),
              ),
        ],
      ),
    );
  }
}

class ColorsDemo extends StatelessWidget {
  const ColorsDemo({super.key});

  @override
  Widget build(BuildContext context) {
    final List<_Palette> palettes = _allPalettes(context);
    return DefaultTabController(
      length: palettes.length,
      child: Scaffold(
        appBar: AppBar(
          automaticallyImplyLeading: false,
          title: Text(GalleryLocalizations.of(context)!.demoColorsTitle),
          bottom: TabBar(
            isScrollable: true,
            tabs: <Widget>[for (final _Palette palette in palettes) Tab(text: palette.name)],
            labelColor: Theme.of(context).colorScheme.onPrimary,
          ),
        ),
        body: TabBarView(
          children: <Widget>[
            for (final _Palette palette in palettes) _PaletteTabView(colors: palette),
          ],
        ),
      ),
    );
  }
}

// END
