// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import '../../gallery_localizations.dart';

// BEGIN bottomAppBarDemo

class BottomAppBarDemo extends StatefulWidget {
  const BottomAppBarDemo({super.key});

  @override
  State createState() => _BottomAppBarDemoState();
}

class _BottomAppBarDemoState extends State<BottomAppBarDemo> with RestorationMixin {
  final RestorableBool _showFab = RestorableBool(true);
  final RestorableBool _showNotch = RestorableBool(true);
  final RestorableInt _currentFabLocation = RestorableInt(0);

  @override
  String get restorationId => 'bottom_app_bar_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_showFab, 'show_fab');
    registerForRestoration(_showNotch, 'show_notch');
    registerForRestoration(_currentFabLocation, 'fab_location');
  }

  @override
  void dispose() {
    _showFab.dispose();
    _showNotch.dispose();
    _currentFabLocation.dispose();
    super.dispose();
  }

  // Since FloatingActionButtonLocation is not an enum, the index of the
  // selected FloatingActionButtonLocation is used for state restoration.
  static const List<FloatingActionButtonLocation> _fabLocations = <FloatingActionButtonLocation>[
    FloatingActionButtonLocation.endDocked,
    FloatingActionButtonLocation.centerDocked,
    FloatingActionButtonLocation.endFloat,
    FloatingActionButtonLocation.centerFloat,
  ];

  void _onShowNotchChanged(bool value) {
    setState(() {
      _showNotch.value = value;
    });
  }

  void _onShowFabChanged(bool value) {
    setState(() {
      _showFab.value = value;
    });
  }

  void _onFabLocationChanged(int? value) {
    setState(() {
      _currentFabLocation.value = value!;
    });
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Text(localizations.demoBottomAppBarTitle),
      ),
      body: ListView(
        padding: const EdgeInsets.only(bottom: 88),
        children: <Widget>[
          SwitchListTile(
            title: Text(localizations.demoFloatingButtonTitle),
            value: _showFab.value,
            onChanged: _onShowFabChanged,
          ),
          SwitchListTile(
            title: Text(localizations.bottomAppBarNotch),
            value: _showNotch.value,
            onChanged: _onShowNotchChanged,
          ),
          Padding(
            padding: const EdgeInsets.all(16),
            child: Text(localizations.bottomAppBarPosition),
          ),
          RadioListTile<int>(
            title: Text(localizations.bottomAppBarPositionDockedEnd),
            value: 0,
            groupValue: _currentFabLocation.value,
            onChanged: _onFabLocationChanged,
          ),
          RadioListTile<int>(
            title: Text(localizations.bottomAppBarPositionDockedCenter),
            value: 1,
            groupValue: _currentFabLocation.value,
            onChanged: _onFabLocationChanged,
          ),
          RadioListTile<int>(
            title: Text(localizations.bottomAppBarPositionFloatingEnd),
            value: 2,
            groupValue: _currentFabLocation.value,
            onChanged: _onFabLocationChanged,
          ),
          RadioListTile<int>(
            title: Text(localizations.bottomAppBarPositionFloatingCenter),
            value: 3,
            groupValue: _currentFabLocation.value,
            onChanged: _onFabLocationChanged,
          ),
        ],
      ),
      floatingActionButton:
          _showFab.value
              ? Semantics(
                container: true,
                sortKey: const OrdinalSortKey(0),
                child: FloatingActionButton(
                  onPressed: () {},
                  tooltip: localizations.buttonTextCreate,
                  child: const Icon(Icons.add),
                ),
              )
              : null,
      floatingActionButtonLocation: _fabLocations[_currentFabLocation.value],
      bottomNavigationBar: _DemoBottomAppBar(
        fabLocation: _fabLocations[_currentFabLocation.value],
        shape: _showNotch.value ? const CircularNotchedRectangle() : null,
      ),
    );
  }
}

class _DemoBottomAppBar extends StatelessWidget {
  const _DemoBottomAppBar({required this.fabLocation, this.shape});

  final FloatingActionButtonLocation fabLocation;
  final NotchedShape? shape;

  static final List<FloatingActionButtonLocation> centerLocations = <FloatingActionButtonLocation>[
    FloatingActionButtonLocation.centerDocked,
    FloatingActionButtonLocation.centerFloat,
  ];

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Semantics(
      sortKey: const OrdinalSortKey(1),
      container: true,
      label: localizations.bottomAppBar,
      child: BottomAppBar(
        shape: shape,
        child: IconTheme(
          data: IconThemeData(color: Theme.of(context).colorScheme.onPrimary),
          child: Row(
            children: <Widget>[
              IconButton(
                tooltip: MaterialLocalizations.of(context).openAppDrawerTooltip,
                icon: const Icon(Icons.menu),
                onPressed: () {},
              ),
              if (centerLocations.contains(fabLocation)) const Spacer(),
              IconButton(
                tooltip: localizations.starterAppTooltipSearch,
                icon: const Icon(Icons.search),
                onPressed: () {},
              ),
              IconButton(
                tooltip: localizations.starterAppTooltipFavorite,
                icon: const Icon(Icons.favorite),
                onPressed: () {},
              ),
            ],
          ),
        ),
      ),
    );
  }
}

// END
