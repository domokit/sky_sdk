// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:animations/animations.dart';
import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';
import 'material_demo_types.dart';

// BEGIN bottomNavigationDemo

class BottomNavigationDemo extends StatefulWidget {
  const BottomNavigationDemo({super.key, required this.restorationId, required this.type});

  final String restorationId;
  final BottomNavigationDemoType type;

  @override
  State<BottomNavigationDemo> createState() => _BottomNavigationDemoState();
}

class _BottomNavigationDemoState extends State<BottomNavigationDemo> with RestorationMixin {
  final RestorableInt _currentIndex = RestorableInt(0);

  @override
  String get restorationId => widget.restorationId;

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_currentIndex, 'bottom_navigation_tab_index');
  }

  @override
  void dispose() {
    _currentIndex.dispose();
    super.dispose();
  }

  String _title(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    switch (widget.type) {
      case BottomNavigationDemoType.withLabels:
        return localizations.demoBottomNavigationPersistentLabels;
      case BottomNavigationDemoType.withoutLabels:
        return localizations.demoBottomNavigationSelectedLabel;
    }
  }

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    final TextTheme textTheme = Theme.of(context).textTheme;
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    List<BottomNavigationBarItem> bottomNavigationBarItems = <BottomNavigationBarItem>[
      BottomNavigationBarItem(
        icon: const Icon(Icons.add_comment),
        label: localizations.bottomNavigationCommentsTab,
      ),
      BottomNavigationBarItem(
        icon: const Icon(Icons.calendar_today),
        label: localizations.bottomNavigationCalendarTab,
      ),
      BottomNavigationBarItem(
        icon: const Icon(Icons.account_circle),
        label: localizations.bottomNavigationAccountTab,
      ),
      BottomNavigationBarItem(
        icon: const Icon(Icons.alarm_on),
        label: localizations.bottomNavigationAlarmTab,
      ),
      BottomNavigationBarItem(
        icon: const Icon(Icons.camera_enhance),
        label: localizations.bottomNavigationCameraTab,
      ),
    ];

    if (widget.type == BottomNavigationDemoType.withLabels) {
      bottomNavigationBarItems = bottomNavigationBarItems.sublist(
        0,
        bottomNavigationBarItems.length - 2,
      );
      _currentIndex.value = _currentIndex.value.clamp(0, bottomNavigationBarItems.length - 1);
    }

    return Scaffold(
      appBar: AppBar(automaticallyImplyLeading: false, title: Text(_title(context))),
      body: Center(
        child: PageTransitionSwitcher(
          transitionBuilder: (
            Widget child,
            Animation<double> animation,
            Animation<double> secondaryAnimation,
          ) {
            return FadeThroughTransition(
              animation: animation,
              secondaryAnimation: secondaryAnimation,
              child: child,
            );
          },
          child: _NavigationDestinationView(
            // Adding [UniqueKey] to make sure the widget rebuilds when transitioning.
            key: UniqueKey(),
            item: bottomNavigationBarItems[_currentIndex.value],
          ),
        ),
      ),
      bottomNavigationBar: BottomNavigationBar(
        showUnselectedLabels: widget.type == BottomNavigationDemoType.withLabels,
        items: bottomNavigationBarItems,
        currentIndex: _currentIndex.value,
        type: BottomNavigationBarType.fixed,
        selectedFontSize: textTheme.bodySmall!.fontSize!,
        unselectedFontSize: textTheme.bodySmall!.fontSize!,
        onTap: (int index) {
          setState(() {
            _currentIndex.value = index;
          });
        },
        selectedItemColor: colorScheme.onPrimary,
        unselectedItemColor: colorScheme.onPrimary.withOpacity(0.38),
        backgroundColor: colorScheme.primary,
      ),
    );
  }
}

class _NavigationDestinationView extends StatelessWidget {
  const _NavigationDestinationView({super.key, required this.item});

  final BottomNavigationBarItem item;

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: <Widget>[
        ExcludeSemantics(
          child: Center(
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: ClipRRect(
                borderRadius: BorderRadius.circular(8),
                child: Image.asset(
                  'assets/demos/bottom_navigation_background.png',
                  package: 'flutter_gallery_assets',
                ),
              ),
            ),
          ),
        ),
        Center(
          child: IconTheme(
            data: const IconThemeData(color: Colors.white, size: 80),
            child: Semantics(
              label: GalleryLocalizations.of(
                context,
              )!.bottomNavigationContentPlaceholder(item.label!),
              child: item.icon,
            ),
          ),
        ),
      ],
    );
  }
}

// END
