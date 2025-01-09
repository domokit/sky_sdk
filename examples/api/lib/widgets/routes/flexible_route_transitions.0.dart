// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';

/// This sample demonstrates creating a custom page transition that is able to
/// override the outgoing transition of the route behind it in the navigation
/// stack using [DelegatedTransitionBuilder].

void main() {
  runApp(const FlexibleRouteTransitionsApp());
}

class FlexibleRouteTransitionsApp extends StatelessWidget {
  const FlexibleRouteTransitionsApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Mixing Routes',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple),
        pageTransitionsTheme: const PageTransitionsTheme(
          builders: <TargetPlatform, PageTransitionsBuilder>{
            // By default the zoom builder is used on all platforms but iOS. Normally
            // on iOS the default is the Cupertino sliding transition. Setting
            // it to use zoom on all platforms allows the example to show multiple
            // transitions in one app for all platforms.
            TargetPlatform.iOS: ZoomPageTransitionsBuilder(),
          },
        ),
      ),
      home: const _MyHomePage(title: 'Zoom Page'),
    );
  }
}

class _MyHomePage extends StatelessWidget {
  const _MyHomePage({required this.title});

  final String title;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(title),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            TextButton(
              onPressed: () {
                Navigator.of(context).push(
                  _VerticalTransitionPageRoute<void>(
                    builder: (BuildContext context) {
                      return const _MyHomePage(title: 'Crazy Vertical Page');
                    },
                  ),
                );
              },
              child: const Text('Crazy Vertical Transition'),
            ),
            TextButton(
              onPressed: () {
                Navigator.of(context).push(
                  MaterialPageRoute<void>(
                    builder: (BuildContext context) {
                      return const _MyHomePage(title: 'Zoom Page');
                    },
                  ),
                );
              },
              child: const Text('Zoom Transition'),
            ),
            TextButton(
              onPressed: () {
                final CupertinoPageRoute<void> route = CupertinoPageRoute<void>(
                  builder: (BuildContext context) {
                    return const _MyHomePage(title: 'Cupertino Page');
                  },
                );
                Navigator.of(context).push(route);
              },
              child: const Text('Cupertino Transition'),
            ),
          ],
        ),
      ),
    );
  }
}

// A PageRoute that applies a _VerticalPageTransition.
class _VerticalTransitionPageRoute<T> extends PageRoute<T> {
  _VerticalTransitionPageRoute({required this.builder});

  final WidgetBuilder builder;

  @override
  DelegatedTransitionBuilder? get delegatedTransition =>
      _VerticalPageTransition._delegatedTransitionBuilder;

  @override
  Color? get barrierColor => const Color(0x00000000);

  @override
  bool get barrierDismissible => false;

  @override
  String? get barrierLabel => 'Should be no visible barrier...';

  @override
  bool get maintainState => true;

  @override
  bool get opaque => false;

  @override
  Duration get transitionDuration => const Duration(milliseconds: 2000);

  @override
  Widget buildPage(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
  ) {
    return builder(context);
  }

  @override
  Widget buildTransitions(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return _VerticalPageTransition(
      primaryRouteAnimation: animation,
      secondaryRouteAnimation: secondaryAnimation,
      child: child,
    );
  }
}

// A page transition that slides off the screen vertically, and uses
// delegatedTransition to ensure that the outgoing route slides with it.
class _VerticalPageTransition extends StatelessWidget {
  _VerticalPageTransition({
    required Animation<double> primaryRouteAnimation,
    required this.secondaryRouteAnimation,
    required this.child,
  }) : _primaryPositionAnimation = CurvedAnimation(
         parent: primaryRouteAnimation,
         curve: _curve,
         reverseCurve: _curve,
       ).drive(_kBottomUpTween),
       _secondaryPositionAnimation = CurvedAnimation(
         parent: secondaryRouteAnimation,
         curve: _curve,
         reverseCurve: _curve,
       ).drive(_kTopDownTween);

  final Animation<Offset> _primaryPositionAnimation;

  final Animation<Offset> _secondaryPositionAnimation;

  final Animation<double> secondaryRouteAnimation;

  final Widget child;

  static const Curve _curve = Curves.decelerate;

  static final Animatable<Offset> _kBottomUpTween = Tween<Offset>(
    begin: const Offset(0.0, 1.0),
    end: Offset.zero,
  );

  static final Animatable<Offset> _kTopDownTween = Tween<Offset>(
    begin: Offset.zero,
    end: const Offset(0.0, -1.0),
  );

  // When the _VerticalTransitionPageRoute animates onto or off of the navigation
  // stack, this transition is given to the route below it so that they animate in
  // sync.
  static Widget _delegatedTransitionBuilder(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    bool allowSnapshotting,
    Widget? child,
  ) {
    final Animatable<Offset> tween = Tween<Offset>(
      begin: Offset.zero,
      end: const Offset(0.0, -1.0),
    ).chain(CurveTween(curve: _curve));

    return SlideTransition(position: secondaryAnimation.drive(tween), child: child);
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasDirectionality(context));
    final TextDirection textDirection = Directionality.of(context);
    return SlideTransition(
      position: _secondaryPositionAnimation,
      textDirection: textDirection,
      transformHitTests: false,
      child: SlideTransition(
        position: _primaryPositionAnimation,
        textDirection: textDirection,
        child: ClipRRect(
          borderRadius: const BorderRadius.vertical(top: Radius.circular(12)),
          child: child,
        ),
      ),
    );
  }
}
