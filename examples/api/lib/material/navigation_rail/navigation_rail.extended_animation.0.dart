// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';

/// Flutter code sample for [NavigationRail.extendedAnimation].

void main() => runApp(const ExtendedAnimationExampleApp());

class ExtendedAnimationExampleApp extends StatelessWidget {
  const ExtendedAnimationExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: Scaffold(body: MyNavigationRail()));
  }
}

class MyNavigationRail extends StatefulWidget {
  const MyNavigationRail({super.key});

  @override
  State<MyNavigationRail> createState() => _MyNavigationRailState();
}

class _MyNavigationRailState extends State<MyNavigationRail> {
  int _selectedIndex = 0;
  bool _extended = false;

  @override
  Widget build(BuildContext context) {
    return Row(
      children: <Widget>[
        NavigationRail(
          selectedIndex: _selectedIndex,
          extended: _extended,
          leading: MyNavigationRailFab(
            onPressed: () {
              setState(() {
                _extended = !_extended;
              });
            },
          ),
          onDestinationSelected: (int index) {
            setState(() {
              _selectedIndex = index;
            });
          },
          labelType: NavigationRailLabelType.none,
          destinations: const <NavigationRailDestination>[
            NavigationRailDestination(
              icon: Icon(Icons.favorite_border),
              selectedIcon: Icon(Icons.favorite),
              label: Text('First'),
            ),
            NavigationRailDestination(
              icon: Icon(Icons.bookmark_border),
              selectedIcon: Icon(Icons.book),
              label: Text('Second'),
            ),
            NavigationRailDestination(
              icon: Icon(Icons.star_border),
              selectedIcon: Icon(Icons.star),
              label: Text('Third'),
            ),
          ],
        ),
        const VerticalDivider(thickness: 1, width: 1),
        // This is the main content.
        Expanded(
          child: Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                const Text('Tap on FloatingActionButton to expand'),
                const SizedBox(height: 20),
                Text('selectedIndex: $_selectedIndex'),
              ],
            ),
          ),
        ),
      ],
    );
  }
}

class MyNavigationRailFab extends StatelessWidget {
  const MyNavigationRailFab({super.key, this.onPressed});

  final VoidCallback? onPressed;

  @override
  Widget build(BuildContext context) {
    final Animation<double> animation = NavigationRail.extendedAnimation(context);
    return AnimatedBuilder(
      animation: animation,
      builder: (BuildContext context, Widget? child) {
        // The extended fab has a shorter height than the regular fab.
        return Container(
          height: 56,
          padding: EdgeInsets.symmetric(vertical: lerpDouble(0, 6, animation.value)!),
          child:
              animation.value == 0
                  ? FloatingActionButton(onPressed: onPressed, child: const Icon(Icons.add))
                  : Align(
                    alignment: AlignmentDirectional.centerStart,
                    widthFactor: animation.value,
                    child: Padding(
                      padding: const EdgeInsetsDirectional.only(start: 8),
                      child: FloatingActionButton.extended(
                        icon: const Icon(Icons.add),
                        label: const Text('CREATE'),
                        onPressed: onPressed,
                      ),
                    ),
                  ),
        );
      },
    );
  }
}
