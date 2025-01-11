// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:ui' as ui;

import 'package:flutter/material.dart';
import 'package:flutter/semantics.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for a [RawMenuAnchor.overlayBuilder] that demonstrates
/// how to create a custom, animated menu overlay.
void main() => runApp(const MenuOverlayBuilderApp());

enum Animals {
  cat('Cat', leading: Text('🦁')),
  kitten('Kitten', leading: Text('🐱')),
  felisCatus('Felis catus', leading: Text('🐈')),
  dog('Dog', leading: Text('🐕'));

  const Animals(this.label, {this.leading});
  final String label;
  final Widget? leading;
}

class MenuOverlayBuilderExample extends StatefulWidget {
  const MenuOverlayBuilderExample({super.key});

  static const double menuItemExtent = 44.0;

  @override
  State<MenuOverlayBuilderExample> createState() => _MenuOverlayBuilderExampleState();
}

class _MenuOverlayBuilderExampleState extends State<MenuOverlayBuilderExample> {
  final MenuController controller = MenuController();
  Animals? _selected;

  void _handlePressed(Animals index) {
    setState(() {
      _selected = index;
    });
    controller.close();
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        Text('Favorite Animal:', style: theme.textTheme.titleMedium),
        const SizedBox(width: 8),
        RawMenuAnchor.overlayBuilder(
          controller: controller,
          overlayBuilder: (BuildContext context, RawMenuAnchorOverlayPosition position) {
            return ShiftingMenuOverlay(
              position: position,
              itemExtent: MenuOverlayBuilderExample.menuItemExtent,
              padding: const EdgeInsets.symmetric(vertical: 6),
              constraints: const BoxConstraints(minWidth: 172),
              selectedIndex: _selected?.index ?? 0,
              children: <Widget>[
                for (final Animals item in Animals.values)
                  MenuItemButton(
                    autofocus: _selected == item,
                    onPressed: () {
                      _handlePressed(item);
                    },
                    leadingIcon: SizedBox(width: 24, child: item.leading),
                    trailingIcon: _selected == item ? const Icon(Icons.check) : null,
                    child: Text(item.label),
                  ),
              ],
            );
          },
          builder: (BuildContext context, MenuController controller, Widget? child) {
            return FilledButton(
              onPressed: () {
                if (controller.isOpen) {
                  controller.close();
                } else {
                  controller.open();
                }
              },
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: <Widget>[
                  Expanded(child: Text(_selected?.label ?? 'Select One')),
                  const Flexible(child: Icon(Icons.arrow_drop_down)),
                ],
              ),
            );
          },
        ),
      ],
    );
  }
}

class ShiftingMenuOverlay extends StatelessWidget {
  const ShiftingMenuOverlay({
    super.key,
    required this.selectedIndex,
    required this.position,
    required this.children,
    required this.itemExtent,
    this.padding = EdgeInsets.zero,
    this.constraints = const BoxConstraints(),
  });

  final int selectedIndex;
  final double itemExtent;
  final RawMenuAnchorOverlayPosition position;
  final List<Widget> children;
  final EdgeInsets padding;
  final BoxConstraints constraints;

  static const Cubic curve = Cubic(0.36, 1.1, 0, 1);

  static const Map<SingleActivator, Intent> shortcuts = <SingleActivator, Intent>{
    SingleActivator(LogicalKeyboardKey.escape): DismissIntent(),
    SingleActivator(LogicalKeyboardKey.arrowDown): DirectionalFocusIntent(TraversalDirection.down),
    SingleActivator(LogicalKeyboardKey.arrowUp): DirectionalFocusIntent(TraversalDirection.up),
  };

  @override
  Widget build(BuildContext context) {
    final int lastDivisibleIndex = math.max(children.length - 1, 1);
    final double bias = (selectedIndex / lastDivisibleIndex) * 2 - 1;
    final Alignment alignment = Alignment(0, bias);
    final Color color = Theme.of(context).colorScheme.surface;
    final ui.Rect(:double height, :ui.Offset topCenter) = position.anchorRect;
    final double verticalOffset =
        selectedIndex * itemExtent + padding.top + (itemExtent - height) / 2;
    return TweenAnimationBuilder<double>(
      tween: Tween<double>(begin: 0, end: 1),
      duration: const Duration(milliseconds: 650),
      curve: curve,
      builder: (BuildContext context, double value, Widget? child) {
        return Positioned(
          top: topCenter.dy - verticalOffset,
          left: topCenter.dx - 84,
          child: Transform.scale(
            alignment: alignment,
            scaleY: value * 0.5 + 0.5,
            transformHitTests: false,
            child: Opacity(opacity: ui.clampDouble(value, 0, 1), child: child),
          ),
        );
      },
      child: PhysicalModel(
        color: color,
        elevation: 8,
        borderRadius: const BorderRadius.all(Radius.circular(6)),
        clipBehavior: Clip.antiAlias,
        child: Semantics.fromProperties(
          explicitChildNodes: true,
          properties: const SemanticsProperties(scopesRoute: true),
          child: TapRegion(
            groupId: position.tapRegionGroupId,
            consumeOutsideTaps: true,
            onTapOutside: (PointerDownEvent event) {
              MenuController.maybeOf(context)?.close();
            },
            child: FocusScope(
              skipTraversal: true,
              descendantsAreFocusable: true,
              child: Shortcuts(
                shortcuts: shortcuts,
                child: Padding(
                  padding: padding,
                  child: ConstrainedBox(
                    constraints: constraints,
                    child: IntrinsicWidth(
                      child: SingleChildScrollView(child: ListBody(children: children)),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}

class MenuOverlayBuilderApp extends StatelessWidget {
  const MenuOverlayBuilderApp({super.key});

  static const ButtonStyle buttonStyle = ButtonStyle(
    splashFactory: InkSparkle.splashFactory,
    visualDensity: VisualDensity.comfortable,
    iconSize: WidgetStatePropertyAll<double>(17),
    overlayColor: WidgetStatePropertyAll<Color>(ui.Color(0x1D82B0FF)),
    textStyle: WidgetStatePropertyAll<TextStyle>(TextStyle(fontSize: 15)),
    padding: WidgetStatePropertyAll<EdgeInsets>(EdgeInsets.symmetric(horizontal: 12)),
    shape: WidgetStatePropertyAll<RoundedRectangleBorder>(RoundedRectangleBorder()),
  );

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData.from(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.blue),
      ).copyWith(
        menuButtonTheme: const MenuButtonThemeData(style: buttonStyle),
        filledButtonTheme: FilledButtonThemeData(
          style: buttonStyle.copyWith(fixedSize: WidgetStateProperty.all(const Size(168, 36))),
        ),
      ),
      home: const Scaffold(
        body: Center(
          child: UnconstrainedBox(clipBehavior: Clip.hardEdge, child: MenuOverlayBuilderExample()),
        ),
      ),
    );
  }
}
