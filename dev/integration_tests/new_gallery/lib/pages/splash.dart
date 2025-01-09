// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:flutter/material.dart';

import '../constants.dart';
import '../layout/adaptive.dart';
import 'home.dart';

const double homePeekDesktop = 210.0;
const double homePeekMobile = 60.0;

class SplashPageAnimation extends InheritedWidget {
  const SplashPageAnimation({super.key, required this.isFinished, required super.child});

  final bool isFinished;

  static SplashPageAnimation? of(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType();
  }

  @override
  bool updateShouldNotify(SplashPageAnimation oldWidget) => true;
}

class SplashPage extends StatefulWidget {
  const SplashPage({super.key, required this.child});

  final Widget child;

  @override
  State<SplashPage> createState() => _SplashPageState();
}

class _SplashPageState extends State<SplashPage> with SingleTickerProviderStateMixin {
  late AnimationController _controller;
  late int _effect;
  final Random _random = Random();

  // A map of the effect index to its duration. This duration is used to
  // determine how long to display the splash animation at launch.
  //
  // If a new effect is added, this map should be updated.
  final Map<int, int> _effectDurations = <int, int>{
    1: 5,
    2: 4,
    3: 4,
    4: 5,
    5: 5,
    6: 4,
    7: 4,
    8: 4,
    9: 3,
    10: 6,
  };

  bool get _isSplashVisible {
    return _controller.status == AnimationStatus.completed ||
        _controller.status == AnimationStatus.forward;
  }

  @override
  void initState() {
    super.initState();

    // If the number of included effects changes, this number should be changed.
    _effect = _random.nextInt(_effectDurations.length) + 1;

    _controller = AnimationController(duration: splashPageAnimationDuration, vsync: this)
      ..addListener(() {
        setState(() {});
      });
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  Animation<RelativeRect> _getPanelAnimation(BuildContext context, BoxConstraints constraints) {
    final double height =
        constraints.biggest.height - (isDisplayDesktop(context) ? homePeekDesktop : homePeekMobile);
    return RelativeRectTween(
      begin: RelativeRect.fill,
      end: RelativeRect.fromLTRB(0, height, 0, 0),
    ).animate(CurvedAnimation(parent: _controller, curve: Curves.easeInOut));
  }

  @override
  Widget build(BuildContext context) {
    return NotificationListener<ToggleSplashNotification>(
      onNotification: (_) {
        _controller.forward();
        return true;
      },
      child: SplashPageAnimation(
        isFinished: _controller.isDismissed,
        child: LayoutBuilder(
          builder: (BuildContext context, BoxConstraints constraints) {
            final Animation<RelativeRect> animation = _getPanelAnimation(context, constraints);
            Widget frontLayer = widget.child;
            if (_isSplashVisible) {
              frontLayer = MouseRegion(
                cursor: SystemMouseCursors.click,
                child: GestureDetector(
                  behavior: HitTestBehavior.opaque,
                  onTap: _controller.reverse,
                  onVerticalDragEnd: (DragEndDetails details) {
                    if (details.velocity.pixelsPerSecond.dy < -200) {
                      _controller.reverse();
                    }
                  },
                  child: IgnorePointer(child: frontLayer),
                ),
              );
            }

            if (isDisplayDesktop(context)) {
              frontLayer = Padding(
                padding: const EdgeInsets.only(top: 136),
                child: ClipRRect(
                  borderRadius: const BorderRadius.vertical(top: Radius.circular(40)),
                  child: frontLayer,
                ),
              );
            }

            return Stack(
              children: <Widget>[
                _SplashBackLayer(
                  isSplashCollapsed: !_isSplashVisible,
                  effect: _effect,
                  onTap: _controller.forward,
                ),
                PositionedTransition(rect: animation, child: frontLayer),
              ],
            );
          },
        ),
      ),
    );
  }
}

class _SplashBackLayer extends StatelessWidget {
  const _SplashBackLayer({required this.isSplashCollapsed, required this.effect, this.onTap});

  final bool isSplashCollapsed;
  final int effect;
  final GestureTapCallback? onTap;

  @override
  Widget build(BuildContext context) {
    final String effectAsset = 'splash_effects/splash_effect_$effect.gif';
    final Image flutterLogo = Image.asset(
      'assets/logo/flutter_logo.png',
      package: 'flutter_gallery_assets',
    );

    Widget? child;
    if (isSplashCollapsed) {
      if (isDisplayDesktop(context)) {
        child = Padding(
          padding: const EdgeInsets.only(top: 50),
          child: Align(
            alignment: Alignment.topCenter,
            child: MouseRegion(
              cursor: SystemMouseCursors.click,
              child: GestureDetector(onTap: onTap, child: flutterLogo),
            ),
          ),
        );
      }
    } else {
      child = Stack(
        children: <Widget>[
          Center(child: Image.asset(effectAsset, package: 'flutter_gallery_assets')),
          Center(child: flutterLogo),
        ],
      );
    }

    return ExcludeSemantics(
      child: Material(
        // This is the background color of the gifs.
        color: const Color(0xFF030303),
        child: Padding(
          padding: EdgeInsets.only(
            bottom: isDisplayDesktop(context) ? homePeekDesktop : homePeekMobile,
          ),
          child: child,
        ),
      ),
    );
  }
}
