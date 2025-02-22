// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: public_member_api_docs

import 'dart:ui' as ui;

import 'package:flutter/material.dart';
import 'package:flutter/physics.dart';

/// Flutter code sample for a [MenuControllerDecorator] that animates a
/// [RawMenuAnchor] with a [SpringSimulation].
void main() {
  runApp(const MenuControllerDecoratorApp());
}

class AnimatedMenuController extends MenuControllerDecorator {
  const AnimatedMenuController({required super.menuController, required this.animationController});
  final AnimationController animationController;
  SpringSimulation get forwardSpring => SpringSimulation(
    SpringDescription.withDampingRatio(mass: 1.0, stiffness: 150, ratio: 0.7),
    animationController.value,
    1.0,
    0.0,
  );
  SpringSimulation get reverseSpring => SpringSimulation(
    SpringDescription.withDampingRatio(mass: 1.0, stiffness: 200, ratio: 0.7),
    animationController.value,
    0.0,
    0.0,
  );

  @override
  void handleMenuOpenRequest({ui.Offset? position}) {
    // Call whenComplete() rather than whenCompleteOrCancel() to avoid marking
    // the menu as opened when the [AnimationStatus] moves from forward to
    // reverse.
    animationController.animateWith(forwardSpring).whenComplete(markMenuOpened);
  }

  @override
  void handleMenuCloseRequest() {
    // Call whenComplete() rather than whenCompleteOrCancel() to avoid marking
    // the menu as closed when the [AnimationStatus] moves from reverse to
    // forward.
    animationController.animateBackWith(reverseSpring).whenComplete(markMenuClosed);
  }
}

class MenuControllerDecoratorExample extends StatefulWidget {
  const MenuControllerDecoratorExample({super.key});

  @override
  State<MenuControllerDecoratorExample> createState() => _MenuControllerDecoratorExampleState();
}

class _MenuControllerDecoratorExampleState extends State<MenuControllerDecoratorExample>
    with SingleTickerProviderStateMixin {
  late final AnimationController animationController;
  late final AnimatedMenuController menuController;

  @override
  void initState() {
    super.initState();
    // Use an unbounded animation controller to allow simulations to run
    // indefinitely.
    animationController = AnimationController.unbounded(vsync: this);
    menuController = AnimatedMenuController(
      menuController: MenuController(),
      animationController: animationController,
    );
  }

  @override
  void dispose() {
    animationController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return RawMenuAnchor(
      controller: menuController,
      overlayBuilder: (BuildContext context, RawMenuOverlayInfo info) {
        // Center the menu below the anchor.
        final ui.Offset position = info.anchorRect.bottomCenter.translate(-75, 4);
        final ColorScheme colorScheme = ColorScheme.of(context);
        return Positioned(
          top: position.dy,
          left: position.dx,
          child: Semantics(
            explicitChildNodes: true,
            scopesRoute: true,
            child: ExcludeFocus(
              excluding: MenuController.maybeAnimationStatusOf(context) == AnimationStatus.reverse,
              child: TapRegion(
                groupId: info.tapRegionGroupId,
                onTapOutside: (PointerDownEvent event) {
                  menuController.close();
                },
                child: ScaleTransition(
                  scale: animationController.view,
                  child: FadeTransition(
                    opacity: animationController.drive(
                      Animatable<double>.fromCallback(
                        (double value) => ui.clampDouble(value, 0, 1),
                      ),
                    ),
                    child: Material(
                      elevation: 8,
                      clipBehavior: Clip.antiAlias,
                      borderRadius: BorderRadius.circular(16),
                      color: colorScheme.primary,
                      child: SizeTransition(
                        axisAlignment: -1,
                        sizeFactor: animationController.view,
                        fixedCrossAxisSizeFactor: 1.0,
                        child: SizedBox(
                          height: 200,
                          width: 150,
                          child: Text(
                            'ANIMATION STATUS:\n${animationController.status.name}',
                            textAlign: TextAlign.center,
                            style: TextStyle(color: colorScheme.onPrimary, fontSize: 12),
                          ),
                        ),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ),
        );
      },
      builder: (BuildContext context, MenuController menuController, Widget? child) {
        return FilledButton(
          onPressed: () {
            if (menuController.animationStatus.isForwardOrCompleted) {
              menuController.close();
            } else {
              menuController.open();
            }
          },
          child: Text(menuController.animationStatus.isForwardOrCompleted ? 'Close' : 'Open'),
        );
      },
    );
  }
}

class MenuControllerDecoratorApp extends StatelessWidget {
  const MenuControllerDecoratorApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData.from(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(
          seedColor: Colors.blue,
          dynamicSchemeVariant: DynamicSchemeVariant.vibrant,
        ),
      ),
      home: const Scaffold(body: Center(child: MenuControllerDecoratorExample())),
    );
  }
}
