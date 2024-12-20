// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'ink_well.dart';
/// @docImport 'material.dart';
/// @docImport 'menu_button_theme.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'button_style.dart';
import 'material_state.dart';
import 'menu_anchor.dart';
import 'theme.dart';
import 'theme_data.dart';

// Examples can assume:
// late Widget child;
// late BuildContext context;
// late MenuStyle style;
// @immutable
// class MyAppHome extends StatelessWidget {
//   const MyAppHome({super.key});
//   @override
//   Widget build(BuildContext context) => const SizedBox();
// }

/// The visual properties that menus have in common.
///
/// Menus created by [MenuBar] and [MenuAnchor] and their themes have a
/// [MenuStyle] property which defines the visual properties whose default
/// values are to be overridden. The default values are defined by the
/// individual menu widgets and are typically based on overall theme's
/// [ThemeData.colorScheme] and [ThemeData.textTheme].
///
/// All of the [MenuStyle] properties are null by default.
///
/// Many of the [MenuStyle] properties are [WidgetStateProperty] objects which
/// resolve to different values depending on the menu's state. For example the
/// [Color] properties are defined with `WidgetStateProperty<Color>` and can
/// resolve to different colors depending on if the menu is pressed, hovered,
/// focused, disabled, etc.
///
/// These properties can override the default value for just one state or all of
/// them. For example to create a [SubmenuButton] whose background color is the
/// color scheme’s primary color with 50% opacity, but only when the menu is
/// pressed, one could write:
///
/// ```dart
/// SubmenuButton(
///   menuStyle: MenuStyle(
///     backgroundColor: WidgetStateProperty.resolveWith<Color?>(
///       (Set<WidgetState> states) {
///         if (states.contains(WidgetState.focused)) {
///           return Theme.of(context).colorScheme.primary.withOpacity(0.5);
///         }
///         return null; // Use the component's default.
///       },
///     ),
///   ),
///   menuChildren: const <Widget>[ /* ... */ ],
///   child: const Text('Fly me to the moon'),
/// ),
/// ```
///
/// In this case the background color for all other menu states would fall back
/// to the [SubmenuButton]'s default values. To unconditionally set the menu's
/// [backgroundColor] for all states one could write:
///
/// ```dart
/// const SubmenuButton(
///   menuStyle: MenuStyle(
///     backgroundColor: WidgetStatePropertyAll<Color>(Colors.green),
///   ),
///   menuChildren: <Widget>[ /* ... */ ],
///   child: Text('Let me play among the stars'),
/// ),
/// ```
///
/// To configure all of the application's menus in the same way, specify the
/// overall theme's `menuTheme`:
///
/// ```dart
/// MaterialApp(
///   theme: ThemeData(
///     menuTheme: const MenuThemeData(
///       style: MenuStyle(backgroundColor: WidgetStatePropertyAll<Color>(Colors.red)),
///     ),
///   ),
///   home: const MyAppHome(),
/// ),
/// ```
///
/// See also:
///
/// * [MenuAnchor], a widget which hosts cascading menus.
/// * [MenuBar], a widget which defines a menu bar of buttons hosting cascading
///   menus.
/// * [MenuButtonTheme], the theme for [SubmenuButton]s and [MenuItemButton]s.
/// * [ButtonStyle], a similar configuration object for button styles.
@immutable
class MenuStyle with Diagnosticable {
  /// Create a [MenuStyle].
  const MenuStyle({
    this.backgroundColor,
    this.shadowColor,
    this.surfaceTintColor,
    this.elevation,
    this.padding,
    this.minimumSize,
    this.fixedSize,
    this.maximumSize,
    this.side,
    this.shape,
    this.mouseCursor,
    this.visualDensity,
    this.alignment,
  });

  /// The menu's background fill color.
  final MaterialStateProperty<Color?>? backgroundColor;

  /// The shadow color of the menu's [Material].
  ///
  /// The material's elevation shadow can be difficult to see for dark themes,
  /// so by default the menu classes add a semi-transparent overlay to indicate
  /// elevation. See [ThemeData.applyElevationOverlayColor].
  final MaterialStateProperty<Color?>? shadowColor;

  /// The surface tint color of the menu's [Material].
  ///
  /// See [Material.surfaceTintColor] for more details.
  final MaterialStateProperty<Color?>? surfaceTintColor;

  /// The elevation of the menu's [Material].
  final MaterialStateProperty<double?>? elevation;

  /// The padding between the menu's boundary and its child.
  final MaterialStateProperty<EdgeInsetsGeometry?>? padding;

  /// The minimum size of the menu itself.
  ///
  /// This value must be less than or equal to [maximumSize].
  final MaterialStateProperty<Size?>? minimumSize;

  /// The menu's size.
  ///
  /// This size is still constrained by the style's [minimumSize] and
  /// [maximumSize]. Fixed size dimensions whose value is [double.infinity] are
  /// ignored.
  ///
  /// To specify menus with a fixed width and the default height use `fixedSize:
  /// Size.fromWidth(320)`. Similarly, to specify a fixed height and the default
  /// width use `fixedSize: Size.fromHeight(100)`.
  final MaterialStateProperty<Size?>? fixedSize;

  /// The maximum size of the menu itself.
  ///
  /// A [Size.infinite] or null value for this property means that the menu's
  /// maximum size is not constrained.
  ///
  /// This value must be greater than or equal to [minimumSize].
  final MaterialStateProperty<Size?>? maximumSize;

  /// The color and weight of the menu's outline.
  ///
  /// This value is combined with [shape] to create a shape decorated with an
  /// outline.
  final MaterialStateProperty<BorderSide?>? side;

  /// The shape of the menu's underlying [Material].
  ///
  /// This shape is combined with [side] to create a shape decorated with an
  /// outline.
  final MaterialStateProperty<OutlinedBorder?>? shape;

  /// The cursor for a mouse pointer when it enters or is hovering over this
  /// menu's [InkWell].
  final MaterialStateProperty<MouseCursor?>? mouseCursor;

  /// Defines how compact the menu's layout will be.
  ///
  /// {@macro flutter.material.themedata.visualDensity}
  ///
  /// See also:
  ///
  ///  * [ThemeData.visualDensity], which specifies the [visualDensity] for all
  ///    widgets within a [Theme].
  final VisualDensity? visualDensity;

  /// Determines the desired alignment of the submenu when opened relative to
  /// the button that opens it.
  ///
  /// If there isn't sufficient space to open the menu with the given alignment,
  /// and there's space on the other side of the button, then the alignment is
  /// swapped to it's opposite (1 becomes -1, etc.), and the menu will try to
  /// appear on the other side of the button. If there isn't enough space there
  /// either, then the menu will be pushed as far over as necessary to display
  /// as much of itself as possible, possibly overlapping the parent button.
  final AlignmentGeometry? alignment;

  @override
  int get hashCode {
    final List<Object?> values = <Object?>[
      backgroundColor,
      shadowColor,
      surfaceTintColor,
      elevation,
      padding,
      minimumSize,
      fixedSize,
      maximumSize,
      side,
      shape,
      mouseCursor,
      visualDensity,
      alignment,
    ];
    return Object.hashAll(values);
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is MenuStyle &&
        other.backgroundColor == backgroundColor &&
        other.shadowColor == shadowColor &&
        other.surfaceTintColor == surfaceTintColor &&
        other.elevation == elevation &&
        other.padding == padding &&
        other.minimumSize == minimumSize &&
        other.fixedSize == fixedSize &&
        other.maximumSize == maximumSize &&
        other.side == side &&
        other.shape == shape &&
        other.mouseCursor == mouseCursor &&
        other.visualDensity == visualDensity &&
        other.alignment == alignment;
  }

  /// Returns a copy of this MenuStyle with the given fields replaced with
  /// the new values.
  MenuStyle copyWith({
    MaterialStateProperty<Color?>? backgroundColor,
    MaterialStateProperty<Color?>? shadowColor,
    MaterialStateProperty<Color?>? surfaceTintColor,
    MaterialStateProperty<double?>? elevation,
    MaterialStateProperty<EdgeInsetsGeometry?>? padding,
    MaterialStateProperty<Size?>? minimumSize,
    MaterialStateProperty<Size?>? fixedSize,
    MaterialStateProperty<Size?>? maximumSize,
    MaterialStateProperty<BorderSide?>? side,
    MaterialStateProperty<OutlinedBorder?>? shape,
    MaterialStateProperty<MouseCursor?>? mouseCursor,
    VisualDensity? visualDensity,
    AlignmentGeometry? alignment,
  }) {
    return MenuStyle(
      backgroundColor: backgroundColor ?? this.backgroundColor,
      shadowColor: shadowColor ?? this.shadowColor,
      surfaceTintColor: surfaceTintColor ?? this.surfaceTintColor,
      elevation: elevation ?? this.elevation,
      padding: padding ?? this.padding,
      minimumSize: minimumSize ?? this.minimumSize,
      fixedSize: fixedSize ?? this.fixedSize,
      maximumSize: maximumSize ?? this.maximumSize,
      side: side ?? this.side,
      shape: shape ?? this.shape,
      mouseCursor: mouseCursor ?? this.mouseCursor,
      visualDensity: visualDensity ?? this.visualDensity,
      alignment: alignment ?? this.alignment,
    );
  }

  /// Returns a copy of this MenuStyle where the non-null fields in [style]
  /// have replaced the corresponding null fields in this MenuStyle.
  ///
  /// In other words, [style] is used to fill in unspecified (null) fields
  /// this MenuStyle.
  MenuStyle merge(MenuStyle? style) {
    if (style == null) {
      return this;
    }
    return copyWith(
      backgroundColor: backgroundColor ?? style.backgroundColor,
      shadowColor: shadowColor ?? style.shadowColor,
      surfaceTintColor: surfaceTintColor ?? style.surfaceTintColor,
      elevation: elevation ?? style.elevation,
      padding: padding ?? style.padding,
      minimumSize: minimumSize ?? style.minimumSize,
      fixedSize: fixedSize ?? style.fixedSize,
      maximumSize: maximumSize ?? style.maximumSize,
      side: side ?? style.side,
      shape: shape ?? style.shape,
      mouseCursor: mouseCursor ?? style.mouseCursor,
      visualDensity: visualDensity ?? style.visualDensity,
      alignment: alignment ?? style.alignment,
    );
  }

  /// Linearly interpolate between two [MenuStyle]s.
  static MenuStyle? lerp(MenuStyle? a, MenuStyle? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return MenuStyle(
      backgroundColor: MaterialStateProperty.lerp<Color?>(
        a?.backgroundColor,
        b?.backgroundColor,
        t,
        Color.lerp,
      ),
      shadowColor: MaterialStateProperty.lerp<Color?>(
        a?.shadowColor,
        b?.shadowColor,
        t,
        Color.lerp,
      ),
      surfaceTintColor: MaterialStateProperty.lerp<Color?>(
        a?.surfaceTintColor,
        b?.surfaceTintColor,
        t,
        Color.lerp,
      ),
      elevation: MaterialStateProperty.lerp<double?>(a?.elevation, b?.elevation, t, lerpDouble),
      padding: MaterialStateProperty.lerp<EdgeInsetsGeometry?>(
        a?.padding,
        b?.padding,
        t,
        EdgeInsetsGeometry.lerp,
      ),
      minimumSize: MaterialStateProperty.lerp<Size?>(a?.minimumSize, b?.minimumSize, t, Size.lerp),
      fixedSize: MaterialStateProperty.lerp<Size?>(a?.fixedSize, b?.fixedSize, t, Size.lerp),
      maximumSize: MaterialStateProperty.lerp<Size?>(a?.maximumSize, b?.maximumSize, t, Size.lerp),
      side: MaterialStateBorderSide.lerp(a?.side, b?.side, t),
      shape: MaterialStateProperty.lerp<OutlinedBorder?>(
        a?.shape,
        b?.shape,
        t,
        OutlinedBorder.lerp,
      ),
      mouseCursor: t < 0.5 ? a?.mouseCursor : b?.mouseCursor,
      visualDensity: t < 0.5 ? a?.visualDensity : b?.visualDensity,
      alignment: AlignmentGeometry.lerp(a?.alignment, b?.alignment, t),
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'backgroundColor',
        backgroundColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'shadowColor',
        shadowColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'surfaceTintColor',
        surfaceTintColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<double?>>(
        'elevation',
        elevation,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<EdgeInsetsGeometry?>>(
        'padding',
        padding,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Size?>>(
        'minimumSize',
        minimumSize,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Size?>>('fixedSize', fixedSize, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Size?>>(
        'maximumSize',
        maximumSize,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<BorderSide?>>('side', side, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<OutlinedBorder?>>(
        'shape',
        shape,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<MouseCursor?>>(
        'mouseCursor',
        mouseCursor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<VisualDensity>('visualDensity', visualDensity, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<AlignmentGeometry>('alignment', alignment, defaultValue: null),
    );
  }
}
