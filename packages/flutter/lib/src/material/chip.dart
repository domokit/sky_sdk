// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'action_chip.dart';
/// @docImport 'app.dart';
/// @docImport 'choice_chip.dart';
/// @docImport 'circle_avatar.dart';
/// @docImport 'filter_chip.dart';
/// @docImport 'input_chip.dart';
/// @docImport 'scaffold.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart' show clampDouble;
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'chip_theme.dart';
import 'color_scheme.dart';
import 'colors.dart';
import 'constants.dart';
import 'debug.dart';
import 'icons.dart';
import 'ink_decoration.dart';
import 'ink_well.dart';
import 'material.dart';
import 'material_localizations.dart';
import 'material_state.dart';
import 'material_state_mixin.dart';
import 'text_theme.dart';
import 'theme.dart';
import 'theme_data.dart';
import 'tooltip.dart';

// Some design constants
const double _kChipHeight = 32.0;

const int _kCheckmarkAlpha = 0xde; // 87%
const int _kDisabledAlpha = 0x61; // 38%
const double _kCheckmarkStrokeWidth = 2.0;

const Duration _kSelectDuration = Duration(milliseconds: 195);
const Duration _kCheckmarkDuration = Duration(milliseconds: 150);
const Duration _kCheckmarkReverseDuration = Duration(milliseconds: 50);
const Duration _kDrawerDuration = Duration(milliseconds: 150);
const Duration _kReverseDrawerDuration = Duration(milliseconds: 100);
const Duration _kDisableDuration = Duration(milliseconds: 75);

const Color _kSelectScrimColor = Color(0x60191919);
const Icon _kDefaultDeleteIcon = Icon(Icons.cancel);

/// An interface defining the base attributes for a Material Design chip.
///
/// Chips are compact elements that represent an attribute, text, entity, or
/// action.
///
/// The defaults mentioned in the documentation for each attribute are what
/// the implementing classes typically use for defaults (but this class doesn't
/// provide or enforce them).
///
/// See also:
///
///  * [Chip], a chip that displays information and can be deleted.
///  * [InputChip], a chip that represents a complex piece of information, such
///    as an entity (person, place, or thing) or conversational text, in a
///    compact form.
///  * [ChoiceChip], allows a single selection from a set of options. Choice
///    chips contain related descriptive text or categories.
///  * [FilterChip], uses tags or descriptive words as a way to filter content.
///  * [ActionChip], represents an action related to primary content.
///  * <https://material.io/design/components/chips.html>
abstract interface class ChipAttributes {
  /// The primary content of the chip.
  ///
  /// Typically a [Text] widget.
  Widget get label;

  /// A widget to display prior to the chip's label.
  ///
  /// Typically a [CircleAvatar] widget.
  Widget? get avatar;

  /// The style to be applied to the chip's label.
  ///
  /// If this is null and [ThemeData.useMaterial3] is true, then
  /// [TextTheme.labelLarge] is used. Otherwise, [TextTheme.bodyLarge]
  /// is used.
  //
  /// This only has an effect on widgets that respect the [DefaultTextStyle],
  /// such as [Text].
  ///
  /// If [TextStyle.color] is a [WidgetStateProperty<Color>], [WidgetStateProperty.resolve]
  /// is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.disabled].
  ///  * [WidgetState.selected].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.pressed].
  TextStyle? get labelStyle;

  /// The color and weight of the chip's outline.
  ///
  /// Defaults to the border side in the ambient [ChipThemeData]. If the theme
  /// border side resolves to null and [ThemeData.useMaterial3] is true, then
  /// [BorderSide] with a [ColorScheme.outline] color is used when the chip is
  /// enabled, and [BorderSide] with a [ColorScheme.onSurface] color with an
  /// opacity of 0.12 is used when the chip is disabled. Otherwise, it defaults
  /// to null.
  ///
  /// This value is combined with [shape] to create a shape decorated with an
  /// outline. To omit the outline entirely, pass [BorderSide.none] to [side].
  ///
  /// If it is a [WidgetStateBorderSide], [WidgetStateProperty.resolve] is
  /// used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.disabled].
  ///  * [WidgetState.selected].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.pressed].
  BorderSide? get side;

  /// The [OutlinedBorder] to draw around the chip.
  ///
  /// Defaults to the shape in the ambient [ChipThemeData]. If the theme
  /// shape resolves to null and [ThemeData.useMaterial3] is true, then
  /// [RoundedRectangleBorder] with a circular border radius of 8.0 is used.
  /// Otherwise, [StadiumBorder] is used.
  ///
  /// This shape is combined with [side] to create a shape decorated with an
  /// outline. If [side] is not null or side of [shape] is [BorderSide.none],
  /// side of [shape] is ignored. To omit the outline entirely,
  /// pass [BorderSide.none] to [side].
  ///
  /// If it is a [WidgetStateOutlinedBorder], [WidgetStateProperty.resolve]
  /// is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.disabled].
  ///  * [WidgetState.selected].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.pressed].
  OutlinedBorder? get shape;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  Clip get clipBehavior;

  /// {@macro flutter.widgets.Focus.focusNode}
  FocusNode? get focusNode;

  /// {@macro flutter.widgets.Focus.autofocus}
  bool get autofocus;

  /// The color that fills the chip, in all [WidgetState]s.
  ///
  /// Defaults to null.
  ///
  /// Resolves in the following states:
  ///  * [WidgetState.selected].
  ///  * [WidgetState.disabled].
  MaterialStateProperty<Color?>? get color;

  /// Color to be used for the unselected, enabled chip's background.
  ///
  /// The default is light grey.
  Color? get backgroundColor;

  /// The padding between the contents of the chip and the outside [shape].
  ///
  /// If this is null and [ThemeData.useMaterial3] is true, then
  /// a padding of 8.0 logical pixels on all sides is used. Otherwise,
  /// it defaults to a padding of 4.0 logical pixels on all sides.
  EdgeInsetsGeometry? get padding;

  /// Defines how compact the chip's layout will be.
  ///
  /// Chips are unaffected by horizontal density changes.
  ///
  /// {@macro flutter.material.themedata.visualDensity}
  ///
  /// See also:
  ///
  ///  * [ThemeData.visualDensity], which specifies the [visualDensity] for all
  ///    widgets within a [Theme].
  VisualDensity? get visualDensity;

  /// The padding around the [label] widget.
  ///
  /// By default, this is 4 logical pixels at the beginning and the end of the
  /// label, and zero on top and bottom.
  EdgeInsetsGeometry? get labelPadding;

  /// Configures the minimum size of the tap target.
  ///
  /// Defaults to [ThemeData.materialTapTargetSize].
  ///
  /// See also:
  ///
  ///  * [MaterialTapTargetSize], for a description of how this affects tap targets.
  MaterialTapTargetSize? get materialTapTargetSize;

  /// Elevation to be applied on the chip relative to its parent.
  ///
  /// This controls the size of the shadow below the chip.
  ///
  /// Defaults to 0. The value is always non-negative.
  double? get elevation;

  /// Color of the chip's shadow when the elevation is greater than 0.
  ///
  /// If this is null and [ThemeData.useMaterial3] is true, then
  /// [Colors.transparent] color is used. Otherwise, it defaults to null.
  Color? get shadowColor;

  /// Color of the chip's surface tint overlay when its elevation is
  /// greater than 0.
  ///
  /// This is not recommended for use. [Material 3 spec](https://m3.material.io/styles/color/the-color-system/color-roles)
  /// introduced a set of tone-based surfaces and surface containers in its [ColorScheme],
  /// which provide more flexibility. The intention is to eventually remove surface tint color from
  /// the framework.
  ///
  /// If this is null, defaults to [Colors.transparent].
  Color? get surfaceTintColor;

  /// Theme used for all icons in the chip.
  ///
  /// If this is null and [ThemeData.useMaterial3] is true, then [IconThemeData]
  /// with a [ColorScheme.primary] color and a size of 18.0 is used when
  /// the chip is enabled, and [IconThemeData] with a [ColorScheme.onSurface]
  /// color and a size of 18.0 is used when the chip is disabled. Otherwise,
  /// it defaults to null.
  IconThemeData? get iconTheme;

  /// Optional size constraints for the avatar.
  ///
  /// When unspecified, defaults to a minimum size of chip height or label height
  /// (whichever is greater) and a padding of 8.0 pixels on all sides.
  ///
  /// The default constraints ensure that the avatar is accessible.
  /// Specifying this parameter enables creation of avatar smaller than
  /// the minimum size, but it is not recommended.
  ///
  /// {@tool dartpad}
  /// This sample shows how to use [avatarBoxConstraints] to adjust avatar size constraints
  ///
  /// ** See code in examples/api/lib/material/chip/chip_attributes.avatar_box_constraints.0.dart **
  /// {@end-tool}
  BoxConstraints? get avatarBoxConstraints;

  /// Used to override the default chip animations durations.
  ///
  /// If [ChipAnimationStyle.enableAnimation] with duration or reverse duration is
  /// provided, it will be used to override the chip enable and disable animation durations.
  /// If it is null, then default duration will be 75ms.
  ///
  /// If [ChipAnimationStyle.selectAnimation] with duration or reverse duration is provided,
  /// it will be used to override the chip select and unselect animation durations.
  /// If it is null, then default duration will be 195ms.
  ///
  /// If [ChipAnimationStyle.avatarDrawerAnimation] with duration or reverse duration
  /// is provided, it will be used to override the chip checkmark animation duration.
  /// If it is null, then default duration will be 150ms.
  ///
  /// If [ChipAnimationStyle.deleteDrawerAnimation] with duration or reverse duration
  /// is provided, it will be used to override the chip delete icon animation duration.
  /// If it is null, then default duration will be 150ms.
  ///
  /// {@tool dartpad}
  /// This sample showcases how to override the chip animations durations using
  /// [ChipAnimationStyle].
  ///
  /// ** See code in examples/api/lib/material/chip/chip_attributes.chip_animation_style.0.dart **
  /// {@end-tool}
  ChipAnimationStyle? get chipAnimationStyle;

  /// The cursor for a mouse pointer when it enters or is hovering over the
  /// widget.
  ///
  /// If [mouseCursor] is a [WidgetStateMouseCursor],
  /// [WidgetStateProperty.resolve] is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  ///
  /// If this property is null, [WidgetStateMouseCursor.clickable] will be used.
  MouseCursor? get mouseCursor;
}

/// An interface for Material Design chips that can be deleted.
///
/// The defaults mentioned in the documentation for each attribute are what
/// the implementing classes typically use for defaults (but this class doesn't
/// provide or enforce them).
///
/// See also:
///
///  * [Chip], a chip that displays information and can be deleted.
///  * [InputChip], a chip that represents a complex piece of information, such
///    as an entity (person, place, or thing) or conversational text, in a
///    compact form.
///  * <https://material.io/design/components/chips.html>
abstract interface class DeletableChipAttributes {
  /// The icon displayed when [onDeleted] is set.
  ///
  /// If [deleteIconColor] is provided, it will be used as the color of the
  /// delete icon. If [deleteIconColor] is null, then the icon will use the
  /// color specified in the chip [IconTheme]. If the [IconTheme] is null, then
  /// the icon will use the color specified in the [ThemeData.iconTheme].
  ///
  /// If a size is specified in the chip [IconTheme], then the delete icon will
  /// use that size. Otherwise, defaults to 18 pixels.
  ///
  /// Defaults to an [Icon] widget set to use [Icons.clear].
  /// If [ThemeData.useMaterial3] is false, then defaults to an [Icon] widget
  /// set to use [Icons.cancel].
  Widget? get deleteIcon;

  /// Called when the user taps the [deleteIcon] to delete the chip.
  ///
  /// If null, the delete button will not appear on the chip.
  ///
  /// The chip will not automatically remove itself: this just tells the app
  /// that the user tapped the delete button. In order to delete the chip, you
  /// have to do something similar to the following sample:
  ///
  /// {@tool dartpad}
  /// This sample shows how to use [onDeleted] to remove an entry when the
  /// delete button is tapped.
  ///
  /// ** See code in examples/api/lib/material/chip/deletable_chip_attributes.on_deleted.0.dart **
  /// {@end-tool}
  VoidCallback? get onDeleted;

  /// Used to define the delete icon's color with an [IconTheme] that
  /// contains the icon.
  ///
  /// The default is `Color(0xde000000)`
  /// (slightly transparent black) for light themes, and `Color(0xdeffffff)`
  /// (slightly transparent white) for dark themes.
  ///
  /// The delete icon appears if [DeletableChipAttributes.onDeleted] is
  /// non-null.
  Color? get deleteIconColor;

  /// The message to be used for the chip's delete button tooltip.
  ///
  /// If provided with an empty string, the tooltip of the delete button will be
  /// disabled.
  ///
  /// If null, the default [MaterialLocalizations.deleteButtonTooltip] will be
  /// used.
  ///
  /// If the chip is disabled, the delete button tooltip will not be shown.
  String? get deleteButtonTooltipMessage;

  /// Optional size constraints for the delete icon.
  ///
  /// When unspecified, defaults to a minimum size of chip height or label height
  /// (whichever is greater) and a padding of 8.0 pixels on all sides.
  ///
  /// The default constraints ensure that the delete icon is accessible.
  /// Specifying this parameter enables creation of delete icon smaller than
  /// the minimum size, but it is not recommended.
  ///
  /// {@tool dartpad}
  /// This sample shows how to use [deleteIconBoxConstraints] to adjust delete icon
  /// size constraints.
  ///
  /// ** See code in examples/api/lib/material/chip/deletable_chip_attributes.delete_icon_box_constraints.0.dart **
  /// {@end-tool}
  BoxConstraints? get deleteIconBoxConstraints;
}

/// An interface for Material Design chips that can have check marks.
///
/// The defaults mentioned in the documentation for each attribute are what
/// the implementing classes typically use for defaults (but this class doesn't
/// provide or enforce them).
///
/// See also:
///
///  * [InputChip], a chip that represents a complex piece of information, such
///    as an entity (person, place, or thing) or conversational text, in a
///    compact form.
///  * [ChoiceChip], allows a single selection from a set of options. Choice
///    chips contain related descriptive text or categories.
///  * [FilterChip], uses tags or descriptive words as a way to filter content.
///  * <https://material.io/design/components/chips.html>
abstract interface class CheckmarkableChipAttributes {
  /// Whether or not to show a check mark when
  /// [SelectableChipAttributes.selected] is true.
  ///
  /// Defaults to true.
  bool? get showCheckmark;

  /// [Color] of the chip's check mark when a check mark is visible.
  ///
  /// This will override the color set by the platform's brightness setting.
  ///
  /// If null, it will defer to a color selected by the platform's brightness
  /// setting.
  Color? get checkmarkColor;
}

/// An interface for Material Design chips that can be selected.
///
/// The defaults mentioned in the documentation for each attribute are what
/// the implementing classes typically use for defaults (but this class doesn't
/// provide or enforce them).
///
/// See also:
///
///  * [InputChip], a chip that represents a complex piece of information, such
///    as an entity (person, place, or thing) or conversational text, in a
///    compact form.
///  * [ChoiceChip], allows a single selection from a set of options. Choice
///    chips contain related descriptive text or categories.
///  * [FilterChip], uses tags or descriptive words as a way to filter content.
///  * <https://material.io/design/components/chips.html>
abstract interface class SelectableChipAttributes {
  /// Whether or not this chip is selected.
  ///
  /// If [onSelected] is not null, this value will be used to determine if the
  /// select check mark will be shown or not.
  ///
  /// Defaults to false.
  bool get selected;

  /// Called when the chip should change between selected and de-selected
  /// states.
  ///
  /// When the chip is tapped, then the [onSelected] callback, if set, will be
  /// applied to `!selected` (see [selected]).
  ///
  /// The chip passes the new value to the callback but does not actually
  /// change state until the parent widget rebuilds the chip with the new
  /// value.
  ///
  /// The callback provided to [onSelected] should update the state of the
  /// parent [StatefulWidget] using the [State.setState] method, so that the
  /// parent gets rebuilt.
  ///
  /// The [onSelected] and [TappableChipAttributes.onPressed] callbacks must not
  /// both be specified at the same time.
  ///
  /// {@tool snippet}
  ///
  /// A [StatefulWidget] that illustrates use of onSelected in an [InputChip].
  ///
  /// ```dart
  /// class Wood extends StatefulWidget {
  ///   const Wood({super.key});
  ///
  ///   @override
  ///   State<StatefulWidget> createState() => WoodState();
  /// }
  ///
  /// class WoodState extends State<Wood> {
  ///   bool _useChisel = false;
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return InputChip(
  ///       label: const Text('Use Chisel'),
  ///       selected: _useChisel,
  ///       onSelected: (bool newValue) {
  ///         setState(() {
  ///           _useChisel = newValue;
  ///         });
  ///       },
  ///     );
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  ValueChanged<bool>? get onSelected;

  /// Elevation to be applied on the chip relative to its parent during the
  /// press motion.
  ///
  /// This controls the size of the shadow below the chip.
  ///
  /// Defaults to 8. The value is always non-negative.
  double? get pressElevation;

  /// Color to be used for the chip's background, indicating that it is
  /// selected.
  ///
  /// The chip is selected when [selected] is true.
  Color? get selectedColor;

  /// Color of the chip's shadow when the elevation is greater than 0 and the
  /// chip is selected.
  ///
  /// The default is [Colors.black].
  Color? get selectedShadowColor;

  /// Tooltip string to be used for the body area (where the label and avatar
  /// are) of the chip.
  String? get tooltip;

  /// The shape of the translucent highlight painted over the avatar when the
  /// [selected] property is true.
  ///
  /// Only the outer path of the shape is used.
  ///
  /// Defaults to [CircleBorder].
  ShapeBorder get avatarBorder;
}

/// An interface for Material Design chips that can be enabled and disabled.
///
/// The defaults mentioned in the documentation for each attribute are what
/// the implementing classes typically use for defaults (but this class doesn't
/// provide or enforce them).
///
/// See also:
///
///  * [InputChip], a chip that represents a complex piece of information, such
///    as an entity (person, place, or thing) or conversational text, in a
///    compact form.
///  * [ChoiceChip], allows a single selection from a set of options. Choice
///    chips contain related descriptive text or categories.
///  * [FilterChip], uses tags or descriptive words as a way to filter content.
///  * <https://material.io/design/components/chips.html>
abstract interface class DisabledChipAttributes {
  /// Whether or not this chip is enabled for input.
  ///
  /// If this is true, but all of the user action callbacks are null (i.e.
  /// [SelectableChipAttributes.onSelected], [TappableChipAttributes.onPressed],
  /// and [DeletableChipAttributes.onDeleted]), then the
  /// control will still be shown as disabled.
  ///
  /// This is typically used if you want the chip to be disabled, but also show
  /// a delete button.
  ///
  /// For classes which don't have this as a constructor argument, [isEnabled]
  /// returns true if their user action callback is set.
  ///
  /// Defaults to true.
  bool get isEnabled;

  /// The color used for the chip's background to indicate that it is not
  /// enabled.
  ///
  /// The chip is disabled when [isEnabled] is false, or all three of
  /// [SelectableChipAttributes.onSelected], [TappableChipAttributes.onPressed],
  /// and [DeletableChipAttributes.onDeleted] are null.
  ///
  /// It defaults to [Colors.black38].
  Color? get disabledColor;
}

/// An interface for Material Design chips that can be tapped.
///
/// The defaults mentioned in the documentation for each attribute are what
/// the implementing classes typically use for defaults (but this class doesn't
/// provide or enforce them).
///
/// See also:
///
///  * [InputChip], a chip that represents a complex piece of information, such
///    as an entity (person, place, or thing) or conversational text, in a
///    compact form.
///  * [ChoiceChip], allows a single selection from a set of options. Choice
///    chips contain related descriptive text or categories.
///  * [FilterChip], uses tags or descriptive words as a way to filter content.
///  * [ActionChip], represents an action related to primary content.
///  * <https://material.io/design/components/chips.html>
abstract interface class TappableChipAttributes {
  /// Called when the user taps the chip.
  ///
  /// If [onPressed] is set, then this callback will be called when the user
  /// taps on the label or avatar parts of the chip. If [onPressed] is null,
  /// then the chip will be disabled.
  ///
  /// {@tool snippet}
  ///
  /// ```dart
  /// class Blacksmith extends StatelessWidget {
  ///   const Blacksmith({super.key});
  ///
  ///   void startHammering() {
  ///     print('bang bang bang');
  ///   }
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return InputChip(
  ///       label: const Text('Apply Hammer'),
  ///       onPressed: startHammering,
  ///     );
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  VoidCallback? get onPressed;

  /// Elevation to be applied on the chip relative to its parent during the
  /// press motion.
  ///
  /// This controls the size of the shadow below the chip.
  ///
  /// Defaults to 8. The value is always non-negative.
  double? get pressElevation;

  /// Tooltip string to be used for the body area (where the label and avatar
  /// are) of the chip.
  String? get tooltip;
}

/// A helper class that overrides the default chip animation parameters.
class ChipAnimationStyle {
  /// Creates an instance of Chip Animation Style class.
  ChipAnimationStyle({
    this.enableAnimation,
    this.selectAnimation,
    this.avatarDrawerAnimation,
    this.deleteDrawerAnimation,
  });

  /// If [enableAnimation] with duration or reverse duration is provided,
  /// it will be used to override the chip enable and disable animation durations.
  /// If it is null, then default duration will be 75ms.
  final AnimationStyle? enableAnimation;

  /// If [selectAnimation] with duration or reverse duration is provided,
  /// it will be used to override the chip select and unselect animation durations.
  /// If it is null, then default duration will be 195ms.
  final AnimationStyle? selectAnimation;

  /// If [avatarDrawerAnimation] with duration or reverse duration is provided,
  /// it will be used to override the chip checkmark animation duration. If it
  /// is null, then default duration will be 150ms.
  final AnimationStyle? avatarDrawerAnimation;

  /// If [deleteDrawerAnimation] with duration or reverse duration is provided,
  /// it will be used to override the chip delete icon animation duration. If it
  /// is null, then default duration will be 150ms.
  final AnimationStyle? deleteDrawerAnimation;
}

/// A Material Design chip.
///
/// Chips are compact elements that represent an attribute, text, entity, or
/// action.
///
/// Supplying a non-null [onDeleted] callback will cause the chip to include a
/// button for deleting the chip.
///
/// Its ancestors must include [Material], [MediaQuery], [Directionality], and
/// [MaterialLocalizations]. Typically all of these widgets are provided by
/// [MaterialApp] and [Scaffold]. The [label] and [clipBehavior] arguments must
/// not be null.
///
/// {@tool snippet}
///
/// ```dart
/// Chip(
///   avatar: CircleAvatar(
///     backgroundColor: Colors.grey.shade800,
///     child: const Text('AB'),
///   ),
///   label: const Text('Aaron Burr'),
/// )
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [InputChip], a chip that represents a complex piece of information, such
///    as an entity (person, place, or thing) or conversational text, in a
///    compact form.
///  * [ChoiceChip], allows a single selection from a set of options. Choice
///    chips contain related descriptive text or categories.
///  * [FilterChip], uses tags or descriptive words as a way to filter content.
///  * [ActionChip], represents an action related to primary content.
///  * [CircleAvatar], which shows images or initials of entities.
///  * [Wrap], A widget that displays its children in multiple horizontal or
///    vertical runs.
///  * <https://material.io/design/components/chips.html>
class Chip extends StatelessWidget implements ChipAttributes, DeletableChipAttributes {
  /// Creates a Material Design chip.
  ///
  /// The [elevation] must be null or non-negative.
  const Chip({
    super.key,
    this.avatar,
    required this.label,
    this.labelStyle,
    this.labelPadding,
    this.deleteIcon,
    this.onDeleted,
    this.deleteIconColor,
    this.deleteButtonTooltipMessage,
    this.side,
    this.shape,
    this.clipBehavior = Clip.none,
    this.focusNode,
    this.autofocus = false,
    this.color,
    this.backgroundColor,
    this.padding,
    this.visualDensity,
    this.materialTapTargetSize,
    this.elevation,
    this.shadowColor,
    this.surfaceTintColor,
    this.iconTheme,
    this.avatarBoxConstraints,
    this.deleteIconBoxConstraints,
    this.chipAnimationStyle,
    this.mouseCursor,
  }) : assert(elevation == null || elevation >= 0.0);

  @override
  final Widget? avatar;
  @override
  final Widget label;
  @override
  final TextStyle? labelStyle;
  @override
  final EdgeInsetsGeometry? labelPadding;
  @override
  final BorderSide? side;
  @override
  final OutlinedBorder? shape;
  @override
  final Clip clipBehavior;
  @override
  final FocusNode? focusNode;
  @override
  final bool autofocus;
  @override
  final MaterialStateProperty<Color?>? color;
  @override
  final Color? backgroundColor;
  @override
  final EdgeInsetsGeometry? padding;
  @override
  final VisualDensity? visualDensity;
  @override
  final Widget? deleteIcon;
  @override
  final VoidCallback? onDeleted;
  @override
  final Color? deleteIconColor;
  @override
  final String? deleteButtonTooltipMessage;
  @override
  final MaterialTapTargetSize? materialTapTargetSize;
  @override
  final double? elevation;
  @override
  final Color? shadowColor;
  @override
  final Color? surfaceTintColor;
  @override
  final IconThemeData? iconTheme;
  @override
  final BoxConstraints? avatarBoxConstraints;
  @override
  final BoxConstraints? deleteIconBoxConstraints;
  @override
  final ChipAnimationStyle? chipAnimationStyle;
  @override
  final MouseCursor? mouseCursor;

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterial(context));
    return RawChip(
      avatar: avatar,
      label: label,
      labelStyle: labelStyle,
      labelPadding: labelPadding,
      deleteIcon: deleteIcon,
      onDeleted: onDeleted,
      deleteIconColor: deleteIconColor,
      deleteButtonTooltipMessage: deleteButtonTooltipMessage,
      tapEnabled: false,
      side: side,
      shape: shape,
      clipBehavior: clipBehavior,
      focusNode: focusNode,
      autofocus: autofocus,
      color: color,
      backgroundColor: backgroundColor,
      padding: padding,
      visualDensity: visualDensity,
      materialTapTargetSize: materialTapTargetSize,
      elevation: elevation,
      shadowColor: shadowColor,
      surfaceTintColor: surfaceTintColor,
      iconTheme: iconTheme,
      avatarBoxConstraints: avatarBoxConstraints,
      deleteIconBoxConstraints: deleteIconBoxConstraints,
      chipAnimationStyle: chipAnimationStyle,
      mouseCursor: mouseCursor,
    );
  }
}

/// A raw Material Design chip.
///
/// This serves as the basis for all of the chip widget types to aggregate.
/// It is typically not created directly, one of the other chip types
/// that are appropriate for the use case are used instead:
///
///  * [Chip] a simple chip that can only display information and be deleted.
///  * [InputChip] represents a complex piece of information, such as an entity
///    (person, place, or thing) or conversational text, in a compact form.
///  * [ChoiceChip] allows a single selection from a set of options.
///  * [FilterChip] a chip that uses tags or descriptive words as a way to
///    filter content.
///  * [ActionChip]s display a set of actions related to primary content.
///
/// Raw chips are typically only used if you want to create your own custom chip
/// type.
///
/// Raw chips can be selected by setting [onSelected], deleted by setting
/// [onDeleted], and pushed like a button with [onPressed]. They have a [label],
/// and they can have a leading icon (see [avatar]) and a trailing icon
/// ([deleteIcon]). Colors and padding can be customized.
///
/// Requires one of its ancestors to be a [Material] widget.
///
/// See also:
///
///  * [CircleAvatar], which shows images or initials of people.
///  * [Wrap], A widget that displays its children in multiple horizontal or
///    vertical runs.
///  * <https://material.io/design/components/chips.html>
class RawChip extends StatefulWidget
    implements
        ChipAttributes,
        DeletableChipAttributes,
        SelectableChipAttributes,
        CheckmarkableChipAttributes,
        DisabledChipAttributes,
        TappableChipAttributes {
  /// Creates a RawChip.
  ///
  /// The [onPressed] and [onSelected] callbacks must not both be specified at
  /// the same time.
  ///
  /// The [pressElevation] and [elevation] must be null or non-negative.
  /// Typically, [pressElevation] is greater than [elevation].
  const RawChip({
    super.key,
    this.defaultProperties,
    this.avatar,
    required this.label,
    this.labelStyle,
    this.padding,
    this.visualDensity,
    this.labelPadding,
    Widget? deleteIcon,
    this.onDeleted,
    this.deleteIconColor,
    this.deleteButtonTooltipMessage,
    this.onPressed,
    this.onSelected,
    this.pressElevation,
    this.tapEnabled = true,
    this.selected = false,
    this.isEnabled = true,
    this.disabledColor,
    this.selectedColor,
    this.tooltip,
    this.side,
    this.shape,
    this.clipBehavior = Clip.none,
    this.focusNode,
    this.autofocus = false,
    this.color,
    this.backgroundColor,
    this.materialTapTargetSize,
    this.elevation,
    this.shadowColor,
    this.surfaceTintColor,
    this.iconTheme,
    this.selectedShadowColor,
    this.showCheckmark,
    this.checkmarkColor,
    this.avatarBorder = const CircleBorder(),
    this.avatarBoxConstraints,
    this.deleteIconBoxConstraints,
    this.chipAnimationStyle,
    this.mouseCursor,
  }) : assert(pressElevation == null || pressElevation >= 0.0),
       assert(elevation == null || elevation >= 0.0),
       deleteIcon = deleteIcon ?? _kDefaultDeleteIcon;

  /// Defines the defaults for the chip properties if
  /// they are not specified elsewhere.
  ///
  /// If null then [ChipThemeData.fromDefaults] will be used
  /// for the default properties.
  final ChipThemeData? defaultProperties;

  @override
  final Widget? avatar;
  @override
  final Widget label;
  @override
  final TextStyle? labelStyle;
  @override
  final EdgeInsetsGeometry? labelPadding;
  @override
  final Widget deleteIcon;
  @override
  final VoidCallback? onDeleted;
  @override
  final Color? deleteIconColor;
  @override
  final String? deleteButtonTooltipMessage;
  @override
  final ValueChanged<bool>? onSelected;
  @override
  final VoidCallback? onPressed;
  @override
  final double? pressElevation;
  @override
  final bool selected;
  @override
  final bool isEnabled;
  @override
  final Color? disabledColor;
  @override
  final Color? selectedColor;
  @override
  final String? tooltip;
  @override
  final BorderSide? side;
  @override
  final OutlinedBorder? shape;
  @override
  final Clip clipBehavior;
  @override
  final FocusNode? focusNode;
  @override
  final bool autofocus;
  @override
  final MaterialStateProperty<Color?>? color;
  @override
  final Color? backgroundColor;
  @override
  final EdgeInsetsGeometry? padding;
  @override
  final VisualDensity? visualDensity;
  @override
  final MaterialTapTargetSize? materialTapTargetSize;
  @override
  final double? elevation;
  @override
  final Color? shadowColor;
  @override
  final Color? surfaceTintColor;
  @override
  final IconThemeData? iconTheme;
  @override
  final Color? selectedShadowColor;
  @override
  final bool? showCheckmark;
  @override
  final Color? checkmarkColor;
  @override
  final ShapeBorder avatarBorder;
  @override
  final BoxConstraints? avatarBoxConstraints;
  @override
  final BoxConstraints? deleteIconBoxConstraints;
  @override
  final ChipAnimationStyle? chipAnimationStyle;
  @override
  final MouseCursor? mouseCursor;

  /// If set, this indicates that the chip should be disabled if all of the
  /// tap callbacks ([onSelected], [onPressed]) are null.
  ///
  /// For example, the [Chip] class sets this to false because it can't be
  /// disabled, even if no callbacks are set on it, since it is used for
  /// displaying information only.
  ///
  /// Defaults to true.
  final bool tapEnabled;

  @override
  State<RawChip> createState() => _RawChipState();
}

class _RawChipState extends State<RawChip>
    with MaterialStateMixin, TickerProviderStateMixin<RawChip> {
  static const Duration pressedAnimationDuration = Duration(milliseconds: 75);

  late AnimationController selectController;
  late AnimationController avatarDrawerController;
  late AnimationController deleteDrawerController;
  late AnimationController enableController;
  late CurvedAnimation checkmarkAnimation;
  late CurvedAnimation avatarDrawerAnimation;
  late CurvedAnimation deleteDrawerAnimation;
  late CurvedAnimation enableAnimation;
  late CurvedAnimation selectionFade;

  bool get hasDeleteButton => widget.onDeleted != null;
  bool get hasAvatar => widget.avatar != null;

  bool get canTap {
    return widget.isEnabled &&
        widget.tapEnabled &&
        (widget.onPressed != null || widget.onSelected != null);
  }

  bool _isTapping = false;
  bool get isTapping => canTap && _isTapping;

  @override
  void initState() {
    assert(widget.onSelected == null || widget.onPressed == null);
    super.initState();
    setMaterialState(MaterialState.disabled, !widget.isEnabled);
    setMaterialState(MaterialState.selected, widget.selected);
    selectController = AnimationController(
      duration: widget.chipAnimationStyle?.selectAnimation?.duration ?? _kSelectDuration,
      reverseDuration: widget.chipAnimationStyle?.selectAnimation?.reverseDuration,
      value: widget.selected ? 1.0 : 0.0,
      vsync: this,
    );
    selectionFade = CurvedAnimation(parent: selectController, curve: Curves.fastOutSlowIn);
    avatarDrawerController = AnimationController(
      duration: widget.chipAnimationStyle?.avatarDrawerAnimation?.duration ?? _kDrawerDuration,
      reverseDuration: widget.chipAnimationStyle?.avatarDrawerAnimation?.reverseDuration,
      value: hasAvatar || widget.selected ? 1.0 : 0.0,
      vsync: this,
    );
    deleteDrawerController = AnimationController(
      duration: widget.chipAnimationStyle?.deleteDrawerAnimation?.duration ?? _kDrawerDuration,
      reverseDuration: widget.chipAnimationStyle?.deleteDrawerAnimation?.reverseDuration,
      value: hasDeleteButton ? 1.0 : 0.0,
      vsync: this,
    );
    enableController = AnimationController(
      duration: widget.chipAnimationStyle?.enableAnimation?.duration ?? _kDisableDuration,
      reverseDuration: widget.chipAnimationStyle?.enableAnimation?.reverseDuration,
      value: widget.isEnabled ? 1.0 : 0.0,
      vsync: this,
    );

    // These will delay the start of some animations, and/or reduce their
    // length compared to the overall select animation, using Intervals.
    final double checkmarkPercentage =
        _kCheckmarkDuration.inMilliseconds / _kSelectDuration.inMilliseconds;
    final double checkmarkReversePercentage =
        _kCheckmarkReverseDuration.inMilliseconds / _kSelectDuration.inMilliseconds;
    final double avatarDrawerReversePercentage =
        _kReverseDrawerDuration.inMilliseconds / _kSelectDuration.inMilliseconds;
    checkmarkAnimation = CurvedAnimation(
      parent: selectController,
      curve: Interval(1.0 - checkmarkPercentage, 1.0, curve: Curves.fastOutSlowIn),
      reverseCurve: Interval(1.0 - checkmarkReversePercentage, 1.0, curve: Curves.fastOutSlowIn),
    );
    deleteDrawerAnimation = CurvedAnimation(
      parent: deleteDrawerController,
      curve: Curves.fastOutSlowIn,
    );
    avatarDrawerAnimation = CurvedAnimation(
      parent: avatarDrawerController,
      curve: Curves.fastOutSlowIn,
      reverseCurve: Interval(1.0 - avatarDrawerReversePercentage, 1.0, curve: Curves.fastOutSlowIn),
    );
    enableAnimation = CurvedAnimation(parent: enableController, curve: Curves.fastOutSlowIn);
  }

  @override
  void dispose() {
    selectController.dispose();
    avatarDrawerController.dispose();
    deleteDrawerController.dispose();
    enableController.dispose();
    checkmarkAnimation.dispose();
    avatarDrawerAnimation.dispose();
    deleteDrawerAnimation.dispose();
    enableAnimation.dispose();
    selectionFade.dispose();
    super.dispose();
  }

  void _handleTapDown(TapDownDetails details) {
    if (!canTap) {
      return;
    }
    setMaterialState(MaterialState.pressed, true);
    setState(() {
      _isTapping = true;
    });
  }

  void _handleTapCancel() {
    if (!canTap) {
      return;
    }
    setMaterialState(MaterialState.pressed, false);
    setState(() {
      _isTapping = false;
    });
  }

  void _handleTap() {
    if (!canTap) {
      return;
    }
    setMaterialState(MaterialState.pressed, false);
    setState(() {
      _isTapping = false;
    });
    // Only one of these can be set, so only one will be called.
    widget.onSelected?.call(!widget.selected);
    widget.onPressed?.call();
  }

  OutlinedBorder _getShape(ThemeData theme, ChipThemeData chipTheme, ChipThemeData chipDefaults) {
    final BorderSide? resolvedSide =
        MaterialStateProperty.resolveAs<BorderSide?>(widget.side, materialStates) ??
        MaterialStateProperty.resolveAs<BorderSide?>(chipTheme.side, materialStates);
    final OutlinedBorder resolvedShape =
        MaterialStateProperty.resolveAs<OutlinedBorder?>(widget.shape, materialStates) ??
        MaterialStateProperty.resolveAs<OutlinedBorder?>(chipTheme.shape, materialStates) ??
        MaterialStateProperty.resolveAs<OutlinedBorder?>(chipDefaults.shape, materialStates)
        // TODO(tahatesser): Remove this fallback when Material 2 is deprecated.
        ??
        const StadiumBorder();
    // If the side is provided, shape uses the provided side.
    if (resolvedSide != null) {
      return resolvedShape.copyWith(side: resolvedSide);
    }
    // If the side is not provided and the shape's side is not [BorderSide.none],
    // then the shape's side is used. Otherwise, the default side is used.
    return resolvedShape.side != BorderSide.none
        ? resolvedShape
        : resolvedShape.copyWith(side: chipDefaults.side);
  }

  Color? resolveColor({
    MaterialStateProperty<Color?>? color,
    Color? selectedColor,
    Color? backgroundColor,
    Color? disabledColor,
    MaterialStateProperty<Color?>? defaultColor,
  }) {
    return _IndividualOverrides(
          color: color,
          selectedColor: selectedColor,
          backgroundColor: backgroundColor,
          disabledColor: disabledColor,
        ).resolve(materialStates) ??
        defaultColor?.resolve(materialStates);
  }

  /// Picks between three different colors, depending upon the state of two
  /// different animations.
  Color? _getBackgroundColor(ThemeData theme, ChipThemeData chipTheme, ChipThemeData chipDefaults) {
    if (theme.useMaterial3) {
      final Color? disabledColor = resolveColor(
        color: widget.color ?? chipTheme.color,
        disabledColor: widget.disabledColor ?? chipTheme.disabledColor,
        defaultColor: chipDefaults.color,
      );
      final Color? backgroundColor = resolveColor(
        color: widget.color ?? chipTheme.color,
        backgroundColor: widget.backgroundColor ?? chipTheme.backgroundColor,
        defaultColor: chipDefaults.color,
      );
      final Color? selectedColor = resolveColor(
        color: widget.color ?? chipTheme.color,
        selectedColor: widget.selectedColor ?? chipTheme.selectedColor,
        defaultColor: chipDefaults.color,
      );
      final ColorTween backgroundTween = ColorTween(begin: disabledColor, end: backgroundColor);
      final ColorTween selectTween = ColorTween(
        begin: backgroundTween.evaluate(enableController),
        end: selectedColor,
      );
      return selectTween.evaluate(selectionFade);
    } else {
      final ColorTween backgroundTween = ColorTween(
        begin: widget.disabledColor ?? chipTheme.disabledColor ?? theme.disabledColor,
        end:
            widget.backgroundColor ??
            chipTheme.backgroundColor ??
            theme.chipTheme.backgroundColor ??
            chipDefaults.backgroundColor,
      );
      final ColorTween selectTween = ColorTween(
        begin: backgroundTween.evaluate(enableController),
        end:
            widget.selectedColor ??
            chipTheme.selectedColor ??
            theme.chipTheme.selectedColor ??
            chipDefaults.selectedColor,
      );
      return selectTween.evaluate(selectionFade);
    }
  }

  @override
  void didUpdateWidget(RawChip oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.isEnabled != widget.isEnabled) {
      setState(() {
        setMaterialState(MaterialState.disabled, !widget.isEnabled);
        if (widget.isEnabled) {
          enableController.forward();
        } else {
          enableController.reverse();
        }
      });
    }
    if (oldWidget.avatar != widget.avatar || oldWidget.selected != widget.selected) {
      setState(() {
        if (hasAvatar || widget.selected) {
          avatarDrawerController.forward();
        } else {
          avatarDrawerController.reverse();
        }
      });
    }
    if (oldWidget.selected != widget.selected) {
      setState(() {
        setMaterialState(MaterialState.selected, widget.selected);
        if (widget.selected) {
          selectController.forward();
        } else {
          selectController.reverse();
        }
      });
    }
    if (oldWidget.onDeleted != widget.onDeleted) {
      setState(() {
        if (hasDeleteButton) {
          deleteDrawerController.forward();
        } else {
          deleteDrawerController.reverse();
        }
      });
    }
  }

  Widget? _wrapWithTooltip({String? tooltip, bool enabled = true, Widget? child}) {
    if (child == null || !enabled || tooltip == null) {
      return child;
    }
    return Tooltip(message: tooltip, child: child);
  }

  Widget? _buildDeleteIcon(
    BuildContext context,
    ThemeData theme,
    ChipThemeData chipTheme,
    ChipThemeData chipDefaults,
  ) {
    if (!hasDeleteButton) {
      return null;
    }
    final IconThemeData iconTheme =
        widget.iconTheme ??
        chipTheme.iconTheme ??
        theme.chipTheme.iconTheme ??
        _ChipDefaultsM3(context, widget.isEnabled).iconTheme!;
    final Color? effectiveDeleteIconColor =
        widget.deleteIconColor ??
        chipTheme.deleteIconColor ??
        theme.chipTheme.deleteIconColor ??
        widget.iconTheme?.color ??
        chipTheme.iconTheme?.color ??
        chipDefaults.deleteIconColor;
    final double effectiveIconSize =
        widget.iconTheme?.size ??
        chipTheme.iconTheme?.size ??
        theme.chipTheme.iconTheme?.size ??
        _ChipDefaultsM3(context, widget.isEnabled).iconTheme!.size!;
    return Semantics(
      container: true,
      button: true,
      child: _wrapWithTooltip(
        tooltip:
            widget.deleteButtonTooltipMessage ??
            MaterialLocalizations.of(context).deleteButtonTooltip,
        enabled: widget.isEnabled && widget.onDeleted != null,
        child: InkWell(
          // Radius should be slightly less than the full size of the chip.
          radius: (_kChipHeight + (widget.padding?.vertical ?? 0.0)) * .45,
          // Keeps the splash from being constrained to the icon alone.
          splashFactory: _UnconstrainedInkSplashFactory(Theme.of(context).splashFactory),
          customBorder: const CircleBorder(),
          onTap: widget.isEnabled ? widget.onDeleted : null,
          child: IconTheme(
            data: iconTheme.copyWith(color: effectiveDeleteIconColor, size: effectiveIconSize),
            child: widget.deleteIcon,
          ),
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterial(context));
    assert(debugCheckHasMediaQuery(context));
    assert(debugCheckHasDirectionality(context));
    assert(debugCheckHasMaterialLocalizations(context));

    final ThemeData theme = Theme.of(context);
    final ChipThemeData chipTheme = ChipTheme.of(context);
    final Brightness brightness = chipTheme.brightness ?? theme.brightness;
    final ChipThemeData chipDefaults =
        widget.defaultProperties ??
        (theme.useMaterial3
            ? _ChipDefaultsM3(context, widget.isEnabled)
            : ChipThemeData.fromDefaults(
              brightness: brightness,
              secondaryColor:
                  brightness == Brightness.dark ? Colors.tealAccent[200]! : theme.primaryColor,
              labelStyle: theme.textTheme.bodyLarge!,
            ));
    final TextDirection? textDirection = Directionality.maybeOf(context);
    final OutlinedBorder resolvedShape = _getShape(theme, chipTheme, chipDefaults);

    final double elevation = widget.elevation ?? chipTheme.elevation ?? chipDefaults.elevation ?? 0;
    final double pressElevation =
        widget.pressElevation ?? chipTheme.pressElevation ?? chipDefaults.pressElevation ?? 0;
    final Color? shadowColor =
        widget.shadowColor ?? chipTheme.shadowColor ?? chipDefaults.shadowColor;
    final Color? surfaceTintColor =
        widget.surfaceTintColor ?? chipTheme.surfaceTintColor ?? chipDefaults.surfaceTintColor;
    final Color? selectedShadowColor =
        widget.selectedShadowColor ??
        chipTheme.selectedShadowColor ??
        chipDefaults.selectedShadowColor;
    final Color? checkmarkColor =
        widget.checkmarkColor ?? chipTheme.checkmarkColor ?? chipDefaults.checkmarkColor;
    final bool showCheckmark =
        widget.showCheckmark ?? chipTheme.showCheckmark ?? chipDefaults.showCheckmark!;
    final EdgeInsetsGeometry padding = widget.padding ?? chipTheme.padding ?? chipDefaults.padding!;
    // Widget's label style is merged with this below.
    final TextStyle labelStyle = chipTheme.labelStyle ?? chipDefaults.labelStyle!;
    final IconThemeData? iconTheme =
        widget.iconTheme ?? chipTheme.iconTheme ?? chipDefaults.iconTheme;
    final BoxConstraints? avatarBoxConstraints =
        widget.avatarBoxConstraints ?? chipTheme.avatarBoxConstraints;
    final BoxConstraints? deleteIconBoxConstraints =
        widget.deleteIconBoxConstraints ?? chipTheme.deleteIconBoxConstraints;

    final TextStyle effectiveLabelStyle = labelStyle.merge(widget.labelStyle);
    final Color? resolvedLabelColor = MaterialStateProperty.resolveAs<Color?>(
      effectiveLabelStyle.color,
      materialStates,
    );
    final TextStyle resolvedLabelStyle = effectiveLabelStyle.copyWith(color: resolvedLabelColor);
    final Widget? avatar =
        iconTheme != null && hasAvatar
            ? IconTheme.merge(data: chipDefaults.iconTheme!.merge(iconTheme), child: widget.avatar!)
            : widget.avatar;

    /// The chip at text scale 1 starts with 8px on each side and as text scaling
    /// gets closer to 2 the label padding is linearly interpolated from 8px to 4px.
    /// Once the widget has a text scaling of 2 or higher than the label padding
    /// remains 4px.
    final double defaultFontSize = effectiveLabelStyle.fontSize ?? 14.0;
    final double effectiveTextScale =
        MediaQuery.textScalerOf(context).scale(defaultFontSize) / 14.0;
    final EdgeInsetsGeometry defaultLabelPadding =
        EdgeInsets.lerp(
          const EdgeInsets.symmetric(horizontal: 8.0),
          const EdgeInsets.symmetric(horizontal: 4.0),
          clampDouble(effectiveTextScale - 1.0, 0.0, 1.0),
        )!;

    final EdgeInsetsGeometry labelPadding =
        widget.labelPadding ??
        chipTheme.labelPadding ??
        chipDefaults.labelPadding ??
        defaultLabelPadding;

    Widget result = Material(
      elevation: isTapping ? pressElevation : elevation,
      shadowColor: widget.selected ? selectedShadowColor : shadowColor,
      surfaceTintColor: surfaceTintColor,
      animationDuration: pressedAnimationDuration,
      shape: resolvedShape,
      clipBehavior: widget.clipBehavior,
      child: InkWell(
        onFocusChange: updateMaterialState(MaterialState.focused),
        focusNode: widget.focusNode,
        autofocus: widget.autofocus,
        canRequestFocus: widget.isEnabled,
        onTap: canTap ? _handleTap : null,
        onTapDown: canTap ? _handleTapDown : null,
        onTapCancel: canTap ? _handleTapCancel : null,
        onHover: canTap ? updateMaterialState(MaterialState.hovered) : null,
        mouseCursor: widget.mouseCursor,
        hoverColor: (widget.color ?? chipTheme.color) == null ? null : Colors.transparent,
        customBorder: resolvedShape,
        child: AnimatedBuilder(
          animation: Listenable.merge(<Listenable>[selectController, enableController]),
          builder: (BuildContext context, Widget? child) {
            return Ink(
              decoration: ShapeDecoration(
                shape: resolvedShape,
                color: _getBackgroundColor(theme, chipTheme, chipDefaults),
              ),
              child: child,
            );
          },
          child: _wrapWithTooltip(
            tooltip: widget.tooltip,
            enabled: widget.onPressed != null || widget.onSelected != null,
            child: _ChipRenderWidget(
              theme: _ChipRenderTheme(
                label: DefaultTextStyle(
                  overflow: TextOverflow.fade,
                  textAlign: TextAlign.start,
                  maxLines: 1,
                  softWrap: false,
                  style: resolvedLabelStyle,
                  child: widget.label,
                ),
                avatar: AnimatedSwitcher(
                  duration: _kDrawerDuration,
                  switchInCurve: Curves.fastOutSlowIn,
                  child: avatar,
                ),
                deleteIcon: AnimatedSwitcher(
                  duration: _kDrawerDuration,
                  switchInCurve: Curves.fastOutSlowIn,
                  child: _buildDeleteIcon(context, theme, chipTheme, chipDefaults),
                ),
                brightness: brightness,
                padding: padding.resolve(textDirection),
                visualDensity: widget.visualDensity ?? theme.visualDensity,
                labelPadding: labelPadding.resolve(textDirection),
                showAvatar: hasAvatar,
                showCheckmark: showCheckmark,
                checkmarkColor: checkmarkColor,
                canTapBody: canTap,
              ),
              value: widget.selected,
              checkmarkAnimation: checkmarkAnimation,
              enableAnimation: enableAnimation,
              avatarDrawerAnimation: avatarDrawerAnimation,
              deleteDrawerAnimation: deleteDrawerAnimation,
              isEnabled: widget.isEnabled,
              avatarBorder: widget.avatarBorder,
              avatarBoxConstraints: avatarBoxConstraints,
              deleteIconBoxConstraints: deleteIconBoxConstraints,
            ),
          ),
        ),
      ),
    );

    final BoxConstraints constraints;
    final Offset densityAdjustment =
        (widget.visualDensity ?? theme.visualDensity).baseSizeAdjustment;
    switch (widget.materialTapTargetSize ?? theme.materialTapTargetSize) {
      case MaterialTapTargetSize.padded:
        constraints = BoxConstraints(
          minWidth: kMinInteractiveDimension + densityAdjustment.dx,
          minHeight: kMinInteractiveDimension + densityAdjustment.dy,
        );
      case MaterialTapTargetSize.shrinkWrap:
        constraints = const BoxConstraints();
    }
    result = _ChipRedirectingHitDetectionWidget(
      constraints: constraints,
      child: Center(widthFactor: 1.0, heightFactor: 1.0, child: result),
    );
    return Semantics(
      button: widget.tapEnabled,
      container: true,
      selected: widget.selected,
      enabled: widget.tapEnabled ? canTap : null,
      child: result,
    );
  }
}

class _IndividualOverrides extends MaterialStateProperty<Color?> {
  _IndividualOverrides({this.color, this.backgroundColor, this.selectedColor, this.disabledColor});

  final MaterialStateProperty<Color?>? color;
  final Color? backgroundColor;
  final Color? selectedColor;
  final Color? disabledColor;

  @override
  Color? resolve(Set<MaterialState> states) {
    if (color != null) {
      return color!.resolve(states);
    }
    if (states.contains(MaterialState.selected) && states.contains(MaterialState.disabled)) {
      return selectedColor;
    }
    if (states.contains(MaterialState.disabled)) {
      return disabledColor;
    }
    if (states.contains(MaterialState.selected)) {
      return selectedColor;
    }
    return backgroundColor;
  }
}

/// Redirects the `buttonRect.dy` passed to [RenderBox.hitTest] to the vertical
/// center of the widget.
///
/// The primary purpose of this widget is to allow padding around the [RawChip]
/// to trigger the child ink feature without increasing the size of the material.
class _ChipRedirectingHitDetectionWidget extends SingleChildRenderObjectWidget {
  const _ChipRedirectingHitDetectionWidget({super.child, required this.constraints});

  final BoxConstraints constraints;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _RenderChipRedirectingHitDetection(constraints);
  }

  @override
  void updateRenderObject(
    BuildContext context,
    covariant _RenderChipRedirectingHitDetection renderObject,
  ) {
    renderObject.additionalConstraints = constraints;
  }
}

class _RenderChipRedirectingHitDetection extends RenderConstrainedBox {
  _RenderChipRedirectingHitDetection(BoxConstraints additionalConstraints)
    : super(additionalConstraints: additionalConstraints);

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    if (!size.contains(position)) {
      return false;
    }
    // Only redirects hit detection which occurs above and below the render object.
    // In order to make this assumption true, I have removed the minimum width
    // constraints, since any reasonable chip would be at least that wide.
    final Offset offset = Offset(position.dx, size.height / 2);
    return result.addWithRawTransform(
      transform: MatrixUtils.forceToPoint(offset),
      position: position,
      hitTest: (BoxHitTestResult result, Offset position) {
        assert(position == offset);
        return child!.hitTest(result, position: offset);
      },
    );
  }
}

class _ChipRenderWidget extends SlottedMultiChildRenderObjectWidget<_ChipSlot, RenderBox> {
  const _ChipRenderWidget({
    required this.theme,
    this.value,
    this.isEnabled,
    required this.checkmarkAnimation,
    required this.avatarDrawerAnimation,
    required this.deleteDrawerAnimation,
    required this.enableAnimation,
    this.avatarBorder,
    this.avatarBoxConstraints,
    this.deleteIconBoxConstraints,
  });

  final _ChipRenderTheme theme;
  final bool? value;
  final bool? isEnabled;
  final Animation<double> checkmarkAnimation;
  final Animation<double> avatarDrawerAnimation;
  final Animation<double> deleteDrawerAnimation;
  final Animation<double> enableAnimation;
  final ShapeBorder? avatarBorder;
  final BoxConstraints? avatarBoxConstraints;
  final BoxConstraints? deleteIconBoxConstraints;

  @override
  Iterable<_ChipSlot> get slots => _ChipSlot.values;

  @override
  Widget? childForSlot(_ChipSlot slot) {
    return switch (slot) {
      _ChipSlot.label => theme.label,
      _ChipSlot.avatar => theme.avatar,
      _ChipSlot.deleteIcon => theme.deleteIcon,
    };
  }

  @override
  void updateRenderObject(BuildContext context, _RenderChip renderObject) {
    renderObject
      ..theme = theme
      ..textDirection = Directionality.of(context)
      ..value = value
      ..isEnabled = isEnabled
      ..checkmarkAnimation = checkmarkAnimation
      ..avatarDrawerAnimation = avatarDrawerAnimation
      ..deleteDrawerAnimation = deleteDrawerAnimation
      ..enableAnimation = enableAnimation
      ..avatarBorder = avatarBorder
      ..avatarBoxConstraints = avatarBoxConstraints
      ..deleteIconBoxConstraints = deleteIconBoxConstraints;
  }

  @override
  SlottedContainerRenderObjectMixin<_ChipSlot, RenderBox> createRenderObject(BuildContext context) {
    return _RenderChip(
      theme: theme,
      textDirection: Directionality.of(context),
      value: value,
      isEnabled: isEnabled,
      checkmarkAnimation: checkmarkAnimation,
      avatarDrawerAnimation: avatarDrawerAnimation,
      deleteDrawerAnimation: deleteDrawerAnimation,
      enableAnimation: enableAnimation,
      avatarBorder: avatarBorder,
      avatarBoxConstraints: avatarBoxConstraints,
      deleteIconBoxConstraints: deleteIconBoxConstraints,
    );
  }
}

enum _ChipSlot { label, avatar, deleteIcon }

@immutable
class _ChipRenderTheme {
  const _ChipRenderTheme({
    required this.avatar,
    required this.label,
    required this.deleteIcon,
    required this.brightness,
    required this.padding,
    required this.visualDensity,
    required this.labelPadding,
    required this.showAvatar,
    required this.showCheckmark,
    required this.checkmarkColor,
    required this.canTapBody,
  });

  final Widget avatar;
  final Widget label;
  final Widget deleteIcon;
  final Brightness brightness;
  final EdgeInsets padding;
  final VisualDensity visualDensity;
  final EdgeInsets labelPadding;
  final bool showAvatar;
  final bool showCheckmark;
  final Color? checkmarkColor;
  final bool canTapBody;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _ChipRenderTheme &&
        other.avatar == avatar &&
        other.label == label &&
        other.deleteIcon == deleteIcon &&
        other.brightness == brightness &&
        other.padding == padding &&
        other.labelPadding == labelPadding &&
        other.showAvatar == showAvatar &&
        other.showCheckmark == showCheckmark &&
        other.checkmarkColor == checkmarkColor &&
        other.canTapBody == canTapBody;
  }

  @override
  int get hashCode => Object.hash(
    avatar,
    label,
    deleteIcon,
    brightness,
    padding,
    labelPadding,
    showAvatar,
    showCheckmark,
    checkmarkColor,
    canTapBody,
  );
}

class _RenderChip extends RenderBox with SlottedContainerRenderObjectMixin<_ChipSlot, RenderBox> {
  _RenderChip({
    required _ChipRenderTheme theme,
    required TextDirection textDirection,
    this.value,
    this.isEnabled,
    required this.checkmarkAnimation,
    required this.avatarDrawerAnimation,
    required this.deleteDrawerAnimation,
    required this.enableAnimation,
    this.avatarBorder,
    BoxConstraints? avatarBoxConstraints,
    BoxConstraints? deleteIconBoxConstraints,
  }) : _theme = theme,
       _textDirection = textDirection,
       _avatarBoxConstraints = avatarBoxConstraints,
       _deleteIconBoxConstraints = deleteIconBoxConstraints;

  bool? value;
  bool? isEnabled;
  late Rect _deleteButtonRect;
  late Rect _pressRect;
  Animation<double> checkmarkAnimation;
  Animation<double> avatarDrawerAnimation;
  Animation<double> deleteDrawerAnimation;
  Animation<double> enableAnimation;
  ShapeBorder? avatarBorder;

  RenderBox get avatar => childForSlot(_ChipSlot.avatar)!;
  RenderBox get deleteIcon => childForSlot(_ChipSlot.deleteIcon)!;
  RenderBox get label => childForSlot(_ChipSlot.label)!;

  _ChipRenderTheme get theme => _theme;
  _ChipRenderTheme _theme;
  set theme(_ChipRenderTheme value) {
    if (_theme == value) {
      return;
    }
    _theme = value;
    markNeedsLayout();
  }

  TextDirection get textDirection => _textDirection;
  TextDirection _textDirection;
  set textDirection(TextDirection value) {
    if (_textDirection == value) {
      return;
    }
    _textDirection = value;
    markNeedsLayout();
  }

  BoxConstraints? get avatarBoxConstraints => _avatarBoxConstraints;
  BoxConstraints? _avatarBoxConstraints;
  set avatarBoxConstraints(BoxConstraints? value) {
    if (_avatarBoxConstraints == value) {
      return;
    }
    _avatarBoxConstraints = value;
    markNeedsLayout();
  }

  BoxConstraints? get deleteIconBoxConstraints => _deleteIconBoxConstraints;
  BoxConstraints? _deleteIconBoxConstraints;
  set deleteIconBoxConstraints(BoxConstraints? value) {
    if (_deleteIconBoxConstraints == value) {
      return;
    }
    _deleteIconBoxConstraints = value;
    markNeedsLayout();
  }

  // The returned list is ordered for hit testing.
  @override
  Iterable<RenderBox> get children {
    final RenderBox? avatar = childForSlot(_ChipSlot.avatar);
    final RenderBox? label = childForSlot(_ChipSlot.label);
    final RenderBox? deleteIcon = childForSlot(_ChipSlot.deleteIcon);
    return <RenderBox>[
      if (avatar != null) avatar,
      if (label != null) label,
      if (deleteIcon != null) deleteIcon,
    ];
  }

  bool get isDrawingCheckmark => theme.showCheckmark && !checkmarkAnimation.isDismissed;
  bool get deleteIconShowing => !deleteDrawerAnimation.isDismissed;

  static Rect _boxRect(RenderBox box) => _boxParentData(box).offset & box.size;

  static BoxParentData _boxParentData(RenderBox box) => box.parentData! as BoxParentData;

  @override
  double computeMinIntrinsicWidth(double height) {
    // The overall padding isn't affected by missing avatar or delete icon
    // because we add the padding regardless to give extra padding for the label
    // when they're missing.
    final double overallPadding = theme.padding.horizontal + theme.labelPadding.horizontal;
    return overallPadding +
        avatar.getMinIntrinsicWidth(height) +
        label.getMinIntrinsicWidth(height) +
        deleteIcon.getMinIntrinsicWidth(height);
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    final double overallPadding = theme.padding.horizontal + theme.labelPadding.horizontal;
    return overallPadding +
        avatar.getMaxIntrinsicWidth(height) +
        label.getMaxIntrinsicWidth(height) +
        deleteIcon.getMaxIntrinsicWidth(height);
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return math.max(
      _kChipHeight,
      theme.padding.vertical + theme.labelPadding.vertical + label.getMinIntrinsicHeight(width),
    );
  }

  @override
  double computeMaxIntrinsicHeight(double width) => getMinIntrinsicHeight(width);

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    // The baseline of this widget is the baseline of the label.
    return (BaselineOffset(label.getDistanceToActualBaseline(baseline)) +
            _boxParentData(label).offset.dy)
        .offset;
  }

  BoxConstraints _labelConstraintsFrom(
    BoxConstraints contentConstraints,
    double iconWidth,
    double contentSize,
    Size rawLabelSize,
  ) {
    // Now that we know the label height and the width of the icons, we can
    // determine how much to shrink the width constraints for the "real" layout.
    final double freeSpace =
        contentConstraints.maxWidth -
        iconWidth -
        theme.labelPadding.horizontal -
        theme.padding.horizontal;
    final double maxLabelWidth = math.max(0.0, freeSpace);
    return BoxConstraints(
      minHeight: rawLabelSize.height,
      maxHeight: contentSize,
      maxWidth: maxLabelWidth.isFinite ? maxLabelWidth : rawLabelSize.width,
    );
  }

  Size _layoutAvatar(
    double contentSize, [
    ChildLayouter layoutChild = ChildLayoutHelper.layoutChild,
  ]) {
    final BoxConstraints avatarConstraints =
        avatarBoxConstraints ?? BoxConstraints.tightFor(width: contentSize, height: contentSize);
    final Size avatarBoxSize = layoutChild(avatar, avatarConstraints);
    if (!theme.showCheckmark && !theme.showAvatar) {
      return Size(0.0, contentSize);
    }
    final double avatarFullWidth = theme.showAvatar ? avatarBoxSize.width : contentSize;
    return Size(avatarFullWidth * avatarDrawerAnimation.value, avatarBoxSize.height);
  }

  Size _layoutDeleteIcon(
    double contentSize, [
    ChildLayouter layoutChild = ChildLayoutHelper.layoutChild,
  ]) {
    final BoxConstraints deleteIconConstraints =
        deleteIconBoxConstraints ??
        BoxConstraints.tightFor(width: contentSize, height: contentSize);
    final Size boxSize = layoutChild(deleteIcon, deleteIconConstraints);
    if (!deleteIconShowing) {
      return Size(0.0, contentSize);
    }
    return Size(deleteDrawerAnimation.value * boxSize.width, boxSize.height);
  }

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    if (!size.contains(position)) {
      return false;
    }
    final bool hitIsOnDeleteIcon = _hitIsOnDeleteIcon(
      padding: theme.padding,
      labelPadding: theme.labelPadding,
      tapPosition: position,
      chipSize: size,
      deleteButtonSize: deleteIcon.size,
      textDirection: textDirection,
    );
    final RenderBox hitTestChild = hitIsOnDeleteIcon ? deleteIcon : label;

    final Offset center = hitTestChild.size.center(Offset.zero);
    return result.addWithRawTransform(
      transform: MatrixUtils.forceToPoint(center),
      position: position,
      hitTest: (BoxHitTestResult result, Offset position) {
        assert(position == center);
        return hitTestChild.hitTest(result, position: center);
      },
    );
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return _computeSizes(constraints, ChildLayoutHelper.dryLayoutChild).size;
  }

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    final _ChipSizes sizes = _computeSizes(constraints, ChildLayoutHelper.dryLayoutChild);
    final BaselineOffset labelBaseline =
        BaselineOffset(label.getDryBaseline(sizes.labelConstraints, baseline)) +
        (sizes.content - sizes.label.height + sizes.densityAdjustment.dy) / 2 +
        theme.padding.top +
        theme.labelPadding.top;
    return labelBaseline.offset;
  }

  _ChipSizes _computeSizes(BoxConstraints constraints, ChildLayouter layoutChild) {
    final BoxConstraints contentConstraints = constraints.loosen();
    // Find out the height of the label within the constraints.
    final Size rawLabelSize = label.getDryLayout(contentConstraints);
    final double contentSize = math.max(
      _kChipHeight - theme.padding.vertical + theme.labelPadding.vertical,
      rawLabelSize.height + theme.labelPadding.vertical,
    );
    assert(contentSize >= rawLabelSize.height);
    final Size avatarSize = _layoutAvatar(contentSize, layoutChild);
    final Size deleteIconSize = _layoutDeleteIcon(contentSize, layoutChild);

    final BoxConstraints labelConstraints = _labelConstraintsFrom(
      contentConstraints,
      avatarSize.width + deleteIconSize.width,
      contentSize,
      rawLabelSize,
    );

    final Size labelSize = theme.labelPadding.inflateSize(layoutChild(label, labelConstraints));
    final Offset densityAdjustment = Offset(0.0, theme.visualDensity.baseSizeAdjustment.dy / 2.0);
    // This is the overall size of the content: it doesn't include
    // theme.padding, that is added in at the end.
    final Size overallSize =
        Size(avatarSize.width + labelSize.width + deleteIconSize.width, contentSize) +
        densityAdjustment;
    final Size paddedSize = Size(
      overallSize.width + theme.padding.horizontal,
      overallSize.height + theme.padding.vertical,
    );

    return _ChipSizes(
      size: constraints.constrain(paddedSize),
      overall: overallSize,
      content: contentSize,
      densityAdjustment: densityAdjustment,
      avatar: avatarSize,
      labelConstraints: labelConstraints,
      label: labelSize,
      deleteIcon: deleteIconSize,
    );
  }

  @override
  void performLayout() {
    final _ChipSizes sizes = _computeSizes(constraints, ChildLayoutHelper.layoutChild);

    // Now we have all of the dimensions. Place the children where they belong.

    const double left = 0.0;
    final double right = sizes.overall.width;

    Offset centerLayout(Size boxSize, double x) {
      assert(sizes.content >= boxSize.height);
      switch (textDirection) {
        case TextDirection.rtl:
          x -= boxSize.width;
        case TextDirection.ltr:
          break;
      }
      return Offset(x, (sizes.content - boxSize.height + sizes.densityAdjustment.dy) / 2.0);
    }

    // These are the offsets to the upper left corners of the boxes (including
    // the child's padding) containing the children, for each child, but not
    // including the overall padding.
    Offset avatarOffset = Offset.zero;
    Offset labelOffset = Offset.zero;
    Offset deleteIconOffset = Offset.zero;
    switch (textDirection) {
      case TextDirection.rtl:
        double start = right;
        if (theme.showCheckmark || theme.showAvatar) {
          avatarOffset = centerLayout(sizes.avatar, start);
          start -= sizes.avatar.width;
        }
        labelOffset = centerLayout(sizes.label, start);
        start -= sizes.label.width;
        if (deleteIconShowing) {
          _deleteButtonRect = Rect.fromLTWH(
            0.0,
            0.0,
            sizes.deleteIcon.width + theme.padding.right,
            sizes.overall.height + theme.padding.vertical,
          );
          deleteIconOffset = centerLayout(sizes.deleteIcon, start);
        } else {
          _deleteButtonRect = Rect.zero;
        }
        start -= sizes.deleteIcon.width;
        if (theme.canTapBody) {
          _pressRect = Rect.fromLTWH(
            _deleteButtonRect.width,
            0.0,
            sizes.overall.width - _deleteButtonRect.width + theme.padding.horizontal,
            sizes.overall.height + theme.padding.vertical,
          );
        } else {
          _pressRect = Rect.zero;
        }
      case TextDirection.ltr:
        double start = left;
        if (theme.showCheckmark || theme.showAvatar) {
          avatarOffset = centerLayout(sizes.avatar, start - avatar.size.width + sizes.avatar.width);
          start += sizes.avatar.width;
        }
        labelOffset = centerLayout(sizes.label, start);
        start += sizes.label.width;
        if (theme.canTapBody) {
          _pressRect = Rect.fromLTWH(
            0.0,
            0.0,
            deleteIconShowing
                ? start + theme.padding.left
                : sizes.overall.width + theme.padding.horizontal,
            sizes.overall.height + theme.padding.vertical,
          );
        } else {
          _pressRect = Rect.zero;
        }
        start -= deleteIcon.size.width - sizes.deleteIcon.width;
        if (deleteIconShowing) {
          deleteIconOffset = centerLayout(sizes.deleteIcon, start);
          _deleteButtonRect = Rect.fromLTWH(
            start + theme.padding.left,
            0.0,
            sizes.deleteIcon.width + theme.padding.right,
            sizes.overall.height + theme.padding.vertical,
          );
        } else {
          _deleteButtonRect = Rect.zero;
        }
    }
    // Center the label vertically.
    labelOffset =
        labelOffset +
        Offset(0.0, ((sizes.label.height - theme.labelPadding.vertical) - label.size.height) / 2.0);
    _boxParentData(avatar).offset = theme.padding.topLeft + avatarOffset;
    _boxParentData(label).offset = theme.padding.topLeft + labelOffset + theme.labelPadding.topLeft;
    _boxParentData(deleteIcon).offset = theme.padding.topLeft + deleteIconOffset;
    final Size paddedSize = Size(
      sizes.overall.width + theme.padding.horizontal,
      sizes.overall.height + theme.padding.vertical,
    );
    size = constraints.constrain(paddedSize);
    assert(
      size.height == constraints.constrainHeight(paddedSize.height),
      "Constrained height ${size.height} doesn't match expected height "
      '${constraints.constrainWidth(paddedSize.height)}',
    );
    assert(
      size.width == constraints.constrainWidth(paddedSize.width),
      "Constrained width ${size.width} doesn't match expected width "
      '${constraints.constrainWidth(paddedSize.width)}',
    );
  }

  static final ColorTween selectionScrimTween = ColorTween(
    begin: Colors.transparent,
    end: _kSelectScrimColor,
  );

  Color get _disabledColor {
    if (enableAnimation.isCompleted) {
      return Colors.white;
    }
    final Color color = switch (theme.brightness) {
      Brightness.light => Colors.white,
      Brightness.dark => Colors.black,
    };
    return ColorTween(
      begin: color.withAlpha(_kDisabledAlpha),
      end: color,
    ).evaluate(enableAnimation)!;
  }

  void _paintCheck(Canvas canvas, Offset origin, double size) {
    Color? paintColor =
        theme.checkmarkColor ??
        switch ((theme.brightness, theme.showAvatar)) {
          (Brightness.light, true) => Colors.white,
          (Brightness.light, false) => Colors.black.withAlpha(_kCheckmarkAlpha),
          (Brightness.dark, true) => Colors.black,
          (Brightness.dark, false) => Colors.white.withAlpha(_kCheckmarkAlpha),
        };

    final ColorTween fadeTween = ColorTween(begin: Colors.transparent, end: paintColor);

    paintColor =
        checkmarkAnimation.status == AnimationStatus.reverse
            ? fadeTween.evaluate(checkmarkAnimation)
            : paintColor;

    final Paint paint =
        Paint()
          ..color = paintColor!
          ..style = PaintingStyle.stroke
          ..strokeWidth = _kCheckmarkStrokeWidth * avatar.size.height / 24.0;
    final double t =
        checkmarkAnimation.status == AnimationStatus.reverse ? 1.0 : checkmarkAnimation.value;
    if (t == 0.0) {
      // Nothing to draw.
      return;
    }
    assert(t > 0.0 && t <= 1.0);
    // As t goes from 0.0 to 1.0, animate the two check mark strokes from the
    // short side to the long side.
    final Path path = Path();
    final Offset start = Offset(size * 0.15, size * 0.45);
    final Offset mid = Offset(size * 0.4, size * 0.7);
    final Offset end = Offset(size * 0.85, size * 0.25);
    if (t < 0.5) {
      final double strokeT = t * 2.0;
      final Offset drawMid = Offset.lerp(start, mid, strokeT)!;
      path.moveTo(origin.dx + start.dx, origin.dy + start.dy);
      path.lineTo(origin.dx + drawMid.dx, origin.dy + drawMid.dy);
    } else {
      final double strokeT = (t - 0.5) * 2.0;
      final Offset drawEnd = Offset.lerp(mid, end, strokeT)!;
      path.moveTo(origin.dx + start.dx, origin.dy + start.dy);
      path.lineTo(origin.dx + mid.dx, origin.dy + mid.dy);
      path.lineTo(origin.dx + drawEnd.dx, origin.dy + drawEnd.dy);
    }
    canvas.drawPath(path, paint);
  }

  void _paintSelectionOverlay(PaintingContext context, Offset offset) {
    if (isDrawingCheckmark) {
      if (theme.showAvatar) {
        final Rect avatarRect = _boxRect(avatar).shift(offset);
        final Paint darkenPaint =
            Paint()
              ..color = selectionScrimTween.evaluate(checkmarkAnimation)!
              ..blendMode = BlendMode.srcATop;
        final Path path = avatarBorder!.getOuterPath(avatarRect);
        context.canvas.drawPath(path, darkenPaint);
      }
      // Need to make the check mark be a little smaller than the avatar.
      final double checkSize = avatar.size.height * 0.75;
      final Offset checkOffset =
          _boxParentData(avatar).offset +
          Offset(avatar.size.height * 0.125, avatar.size.height * 0.125);
      _paintCheck(context.canvas, offset + checkOffset, checkSize);
    }
  }

  final LayerHandle<OpacityLayer> _avatarOpacityLayerHandler = LayerHandle<OpacityLayer>();

  void _paintAvatar(PaintingContext context, Offset offset) {
    void paintWithOverlay(PaintingContext context, Offset offset) {
      context.paintChild(avatar, _boxParentData(avatar).offset + offset);
      _paintSelectionOverlay(context, offset);
    }

    if (!theme.showAvatar && avatarDrawerAnimation.isDismissed) {
      _avatarOpacityLayerHandler.layer = null;
      return;
    }
    final Color disabledColor = _disabledColor;
    final int disabledColorAlpha = disabledColor.alpha;
    if (needsCompositing) {
      _avatarOpacityLayerHandler.layer = context.pushOpacity(
        offset,
        disabledColorAlpha,
        paintWithOverlay,
        oldLayer: _avatarOpacityLayerHandler.layer,
      );
    } else {
      _avatarOpacityLayerHandler.layer = null;
      if (disabledColorAlpha != 0xff) {
        context.canvas.saveLayer(
          _boxRect(avatar).shift(offset).inflate(20.0),
          Paint()..color = disabledColor,
        );
      }
      paintWithOverlay(context, offset);
      if (disabledColorAlpha != 0xff) {
        context.canvas.restore();
      }
    }
  }

  final LayerHandle<OpacityLayer> _labelOpacityLayerHandler = LayerHandle<OpacityLayer>();
  final LayerHandle<OpacityLayer> _deleteIconOpacityLayerHandler = LayerHandle<OpacityLayer>();

  void _paintChild(
    PaintingContext context,
    Offset offset,
    RenderBox? child, {
    required bool isDeleteIcon,
  }) {
    if (child == null) {
      _labelOpacityLayerHandler.layer = null;
      _deleteIconOpacityLayerHandler.layer = null;
      return;
    }
    final int disabledColorAlpha = _disabledColor.alpha;
    if (!enableAnimation.isCompleted) {
      if (needsCompositing) {
        _labelOpacityLayerHandler.layer = context.pushOpacity(offset, disabledColorAlpha, (
          PaintingContext context,
          Offset offset,
        ) {
          context.paintChild(child, _boxParentData(child).offset + offset);
        }, oldLayer: _labelOpacityLayerHandler.layer);
        if (isDeleteIcon) {
          _deleteIconOpacityLayerHandler.layer = context.pushOpacity(offset, disabledColorAlpha, (
            PaintingContext context,
            Offset offset,
          ) {
            context.paintChild(child, _boxParentData(child).offset + offset);
          }, oldLayer: _deleteIconOpacityLayerHandler.layer);
        }
      } else {
        _labelOpacityLayerHandler.layer = null;
        _deleteIconOpacityLayerHandler.layer = null;
        final Rect childRect = _boxRect(child).shift(offset);
        context.canvas.saveLayer(childRect.inflate(20.0), Paint()..color = _disabledColor);
        context.paintChild(child, _boxParentData(child).offset + offset);
        context.canvas.restore();
      }
    } else {
      context.paintChild(child, _boxParentData(child).offset + offset);
    }
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    checkmarkAnimation.addListener(markNeedsPaint);
    avatarDrawerAnimation.addListener(markNeedsLayout);
    deleteDrawerAnimation.addListener(markNeedsLayout);
    enableAnimation.addListener(markNeedsPaint);
  }

  @override
  void detach() {
    checkmarkAnimation.removeListener(markNeedsPaint);
    avatarDrawerAnimation.removeListener(markNeedsLayout);
    deleteDrawerAnimation.removeListener(markNeedsLayout);
    enableAnimation.removeListener(markNeedsPaint);
    super.detach();
  }

  @override
  void dispose() {
    _labelOpacityLayerHandler.layer = null;
    _deleteIconOpacityLayerHandler.layer = null;
    _avatarOpacityLayerHandler.layer = null;
    super.dispose();
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    _paintAvatar(context, offset);
    if (deleteIconShowing) {
      _paintChild(context, offset, deleteIcon, isDeleteIcon: true);
    }
    _paintChild(context, offset, label, isDeleteIcon: false);
  }

  // Set this to true to have outlines of the tap targets drawn over
  // the chip. This should never be checked in while set to 'true'.
  static const bool _debugShowTapTargetOutlines = false;

  @override
  void debugPaint(PaintingContext context, Offset offset) {
    assert(
      !_debugShowTapTargetOutlines ||
          () {
            // Draws a rect around the tap targets to help with visualizing where
            // they really are.
            final Paint outlinePaint =
                Paint()
                  ..color = const Color(0xff800000)
                  ..strokeWidth = 1.0
                  ..style = PaintingStyle.stroke;
            if (deleteIconShowing) {
              context.canvas.drawRect(_deleteButtonRect.shift(offset), outlinePaint);
            }
            context.canvas.drawRect(
              _pressRect.shift(offset),
              outlinePaint..color = const Color(0xff008000),
            );
            return true;
          }(),
    );
  }

  @override
  bool hitTestSelf(Offset position) =>
      _deleteButtonRect.contains(position) || _pressRect.contains(position);
}

class _ChipSizes {
  _ChipSizes({
    required this.size,
    required this.overall,
    required this.content,
    required this.avatar,
    required this.labelConstraints,
    required this.label,
    required this.deleteIcon,
    required this.densityAdjustment,
  });
  final Size size;
  final Size overall;
  final double content;
  final Size avatar;
  final BoxConstraints labelConstraints;
  final Size label;
  final Size deleteIcon;
  final Offset densityAdjustment;
}

class _UnconstrainedInkSplashFactory extends InteractiveInkFeatureFactory {
  const _UnconstrainedInkSplashFactory(this.parentFactory);

  final InteractiveInkFeatureFactory parentFactory;

  @override
  InteractiveInkFeature create({
    required MaterialInkController controller,
    required RenderBox referenceBox,
    required Offset position,
    required Color color,
    required TextDirection textDirection,
    bool containedInkWell = false,
    RectCallback? rectCallback,
    BorderRadius? borderRadius,
    ShapeBorder? customBorder,
    double? radius,
    VoidCallback? onRemoved,
  }) {
    return parentFactory.create(
      controller: controller,
      referenceBox: referenceBox,
      position: position,
      color: color,
      rectCallback: rectCallback,
      borderRadius: borderRadius,
      customBorder: customBorder,
      radius: radius,
      onRemoved: onRemoved,
      textDirection: textDirection,
    );
  }
}

bool _hitIsOnDeleteIcon({
  required EdgeInsetsGeometry padding,
  required EdgeInsetsGeometry labelPadding,
  required Offset tapPosition,
  required Size chipSize,
  required Size deleteButtonSize,
  required TextDirection textDirection,
}) {
  // The chipSize includes the padding, so we need to deflate the size and adjust the
  // tap position to account for the padding.
  final EdgeInsets resolvedPadding = padding.resolve(textDirection);
  final Size deflatedSize = resolvedPadding.deflateSize(chipSize);
  final Offset adjustedPosition = tapPosition - Offset(resolvedPadding.left, resolvedPadding.top);
  // The delete button hit area should be at least the width of the delete
  // button and right label padding, but, if there's room, up to 24 pixels
  // from the center of the delete icon (corresponding to part of a 48x48 square
  // that Material would prefer for touch targets), but no more than approximately
  // half of the overall size of the chip when the chip is small.
  //
  // This isn't affected by materialTapTargetSize because it only applies to the
  // width of the tappable region within the chip, not outside of the chip,
  // which is handled elsewhere. Also because delete buttons aren't specified to
  // be used on touch devices, only desktop devices.

  // Max out at not quite half, so that tests that tap on the center of a small
  // chip will still hit the chip, not the delete button.
  final double accessibleDeleteButtonWidth = math.min(
    deflatedSize.width * 0.499,
    math.min(
      labelPadding.resolve(textDirection).right + deleteButtonSize.width,
      24.0 + deleteButtonSize.width / 2.0,
    ),
  );
  return switch (textDirection) {
    TextDirection.ltr => adjustedPosition.dx >= deflatedSize.width - accessibleDeleteButtonWidth,
    TextDirection.rtl => adjustedPosition.dx <= accessibleDeleteButtonWidth,
  };
}

// BEGIN GENERATED TOKEN PROPERTIES - Chip

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _ChipDefaultsM3 extends ChipThemeData {
  _ChipDefaultsM3(this.context, this.isEnabled)
    : super(
        elevation: 0.0,
        shape: const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(8.0))),
        showCheckmark: true,
      );

  final BuildContext context;
  final bool isEnabled;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  TextStyle? get labelStyle => _textTheme.labelLarge?.copyWith(
    color: isEnabled
      ? _colors.onSurfaceVariant
      : _colors.onSurface,
  );

  @override
  MaterialStateProperty<Color?>? get color => null; // Subclasses override this getter

  @override
  Color? get shadowColor => Colors.transparent;

  @override
  Color? get surfaceTintColor => Colors.transparent;

  @override
  Color? get checkmarkColor => null;

  @override
  Color? get deleteIconColor => isEnabled
    ? _colors.onSurfaceVariant
    : _colors.onSurface;

  @override
  BorderSide? get side => isEnabled
    ? BorderSide(color: _colors.outlineVariant)
    : BorderSide(color: _colors.onSurface.withOpacity(0.12));

  @override
  IconThemeData? get iconTheme => IconThemeData(
    color: isEnabled
      ? _colors.primary
      : _colors.onSurface,
    size: 18.0,
  );

  @override
  EdgeInsetsGeometry? get padding => const EdgeInsets.all(8.0);

  /// The label padding of the chip scales with the font size specified in the
  /// [labelStyle], and the system font size settings that scale font sizes
  /// globally.
  ///
  /// The chip at effective font size 14.0 starts with 8px on each side and as
  /// the font size scales up to closer to 28.0, the label padding is linearly
  /// interpolated from 8px to 4px. Once the label has a font size of 2 or
  /// higher, label padding remains 4px.
  @override
  EdgeInsetsGeometry? get labelPadding {
    final double fontSize = labelStyle?.fontSize ?? 14.0;
    final double fontSizeRatio = MediaQuery.textScalerOf(context).scale(fontSize) / 14.0;
    return EdgeInsets.lerp(
      const EdgeInsets.symmetric(horizontal: 8.0),
      const EdgeInsets.symmetric(horizontal: 4.0),
      clampDouble(fontSizeRatio - 1.0, 0.0, 1.0),
    )!;
  }
}
// dart format on

// END GENERATED TOKEN PROPERTIES - Chip
