// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

class NavigationIconView {
  NavigationIconView({
    required Widget icon,
    Widget? activeIcon,
    String? title,
    Color? color,
    required TickerProvider vsync,
  }) : _icon = icon,
       _color = color,
       _title = title,
       item = BottomNavigationBarItem(
         icon: icon,
         activeIcon: activeIcon,
         label: title,
         backgroundColor: color,
       ),
       controller = AnimationController(duration: kThemeAnimationDuration, vsync: vsync) {
    _animation = controller.drive(
      CurveTween(curve: const Interval(0.5, 1.0, curve: Curves.fastOutSlowIn)),
    );
  }

  final Widget _icon;
  final Color? _color;
  final String? _title;
  final BottomNavigationBarItem item;
  final AnimationController controller;
  late Animation<double> _animation;

  FadeTransition transition(BottomNavigationBarType type, BuildContext context) {
    Color? iconColor;
    switch (type) {
      case BottomNavigationBarType.shifting:
        iconColor = _color;
      case BottomNavigationBarType.fixed:
        final ThemeData theme = Theme.of(context);
        iconColor = switch (theme.brightness) {
          Brightness.light => theme.colorScheme.primary,
          Brightness.dark => theme.colorScheme.secondary,
        };
    }

    return FadeTransition(
      opacity: _animation,
      child: SlideTransition(
        position: _animation.drive(
          Tween<Offset>(
            begin: const Offset(0.0, 0.02), // Slightly down.
            end: Offset.zero,
          ),
        ),
        child: IconTheme(
          data: IconThemeData(color: iconColor, size: 120.0),
          child: Semantics(label: 'Placeholder for $_title tab', child: _icon),
        ),
      ),
    );
  }
}

class CustomIcon extends StatelessWidget {
  const CustomIcon({super.key});

  @override
  Widget build(BuildContext context) {
    final IconThemeData iconTheme = IconTheme.of(context);
    return Container(
      margin: const EdgeInsets.all(4.0),
      width: iconTheme.size! - 8.0,
      height: iconTheme.size! - 8.0,
      color: iconTheme.color,
    );
  }
}

class CustomInactiveIcon extends StatelessWidget {
  const CustomInactiveIcon({super.key});

  @override
  Widget build(BuildContext context) {
    final IconThemeData iconTheme = IconTheme.of(context);
    return Container(
      margin: const EdgeInsets.all(4.0),
      width: iconTheme.size! - 8.0,
      height: iconTheme.size! - 8.0,
      decoration: BoxDecoration(border: Border.all(color: iconTheme.color!, width: 2.0)),
    );
  }
}

class BottomNavigationDemo extends StatefulWidget {
  const BottomNavigationDemo({super.key});

  static const String routeName = '/material/bottom_navigation';

  @override
  State<BottomNavigationDemo> createState() => _BottomNavigationDemoState();
}

class _BottomNavigationDemoState extends State<BottomNavigationDemo> with TickerProviderStateMixin {
  int _currentIndex = 0;
  BottomNavigationBarType _type = BottomNavigationBarType.shifting;
  late List<NavigationIconView> _navigationViews;

  @override
  void initState() {
    super.initState();
    _navigationViews = <NavigationIconView>[
      NavigationIconView(
        icon: const Icon(Icons.access_alarm),
        title: 'Alarm',
        color: Colors.deepPurple,
        vsync: this,
      ),
      NavigationIconView(
        activeIcon: const CustomIcon(),
        icon: const CustomInactiveIcon(),
        title: 'Box',
        color: Colors.deepOrange,
        vsync: this,
      ),
      NavigationIconView(
        activeIcon: const Icon(Icons.cloud),
        icon: const Icon(Icons.cloud_queue),
        title: 'Cloud',
        color: Colors.teal,
        vsync: this,
      ),
      NavigationIconView(
        activeIcon: const Icon(Icons.favorite),
        icon: const Icon(Icons.favorite_border),
        title: 'Favorites',
        color: Colors.indigo,
        vsync: this,
      ),
      NavigationIconView(
        icon: const Icon(Icons.event_available),
        title: 'Event',
        color: Colors.pink,
        vsync: this,
      ),
    ];

    _navigationViews[_currentIndex].controller.value = 1.0;
  }

  @override
  void dispose() {
    for (final NavigationIconView view in _navigationViews) {
      view.controller.dispose();
    }
    super.dispose();
  }

  Widget _buildTransitionsStack() {
    final List<FadeTransition> transitions = <FadeTransition>[
      for (final NavigationIconView view in _navigationViews) view.transition(_type, context),
    ];

    // We want to have the newly animating (fading in) views on top.
    transitions.sort((FadeTransition a, FadeTransition b) {
      final Animation<double> aAnimation = a.opacity;
      final Animation<double> bAnimation = b.opacity;
      final double aValue = aAnimation.value;
      final double bValue = bAnimation.value;
      return aValue.compareTo(bValue);
    });

    return Stack(children: transitions);
  }

  @override
  Widget build(BuildContext context) {
    final BottomNavigationBar botNavBar = BottomNavigationBar(
      items:
          _navigationViews
              .map<BottomNavigationBarItem>(
                (NavigationIconView navigationView) => navigationView.item,
              )
              .toList(),
      currentIndex: _currentIndex,
      type: _type,
      onTap: (int index) {
        setState(() {
          _navigationViews[_currentIndex].controller.reverse();
          _currentIndex = index;
          _navigationViews[_currentIndex].controller.forward();
        });
      },
    );

    return Scaffold(
      appBar: AppBar(
        title: const Text('Bottom navigation'),
        actions: <Widget>[
          MaterialDemoDocumentationButton(BottomNavigationDemo.routeName),
          PopupMenuButton<BottomNavigationBarType>(
            onSelected: (BottomNavigationBarType value) {
              setState(() {
                _type = value;
              });
            },
            itemBuilder:
                (BuildContext context) => <PopupMenuItem<BottomNavigationBarType>>[
                  const PopupMenuItem<BottomNavigationBarType>(
                    value: BottomNavigationBarType.fixed,
                    child: Text('Fixed'),
                  ),
                  const PopupMenuItem<BottomNavigationBarType>(
                    value: BottomNavigationBarType.shifting,
                    child: Text('Shifting'),
                  ),
                ],
          ),
        ],
      ),
      body: Center(child: _buildTransitionsStack()),
      bottomNavigationBar: botNavBar,
    );
  }
}
