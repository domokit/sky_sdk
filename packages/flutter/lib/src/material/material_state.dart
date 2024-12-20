// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/scheduler.dart';
///
/// @docImport 'action_chip.dart';
/// @docImport 'button_style.dart';
/// @docImport 'elevated_button.dart';
/// @docImport 'ink_well.dart';
/// @docImport 'input_decorator.dart';
/// @docImport 'list_tile.dart';
/// @docImport 'outlined_button.dart';
/// @docImport 'text_button.dart';
/// @docImport 'text_field.dart';
/// @docImport 'time_picker_theme.dart';
library;

import 'package:flutter/widgets.dart';

import 'input_border.dart';

// Examples can assume:
// late BuildContext context;

/// Interactive states that some of the Material widgets can take on when
/// receiving input from the user.
///
/// States are defined by https://material.io/design/interaction/states.html#usage.
///
/// Some Material widgets track their current state in a `Set<MaterialState>`.
///
/// See also:
///
///  * [WidgetState], a general non-Material version that can be used
///    interchangeably with `MaterialState`. They functionally work the same,
///    except [WidgetState] can be used outside of Material.
///  * [MaterialStateProperty], an interface for objects that "resolve" to
///    different values depending on a widget's material state.
/// {@template flutter.material.MaterialStateProperty.implementations}
///  * [MaterialStateColor], a [Color] that implements `MaterialStateProperty`
///    which is used in APIs that need to accept either a [Color] or a
///    `MaterialStateProperty<Color>`.
///  * [MaterialStateMouseCursor], a [MouseCursor] that implements
///    `MaterialStateProperty` which is used in APIs that need to accept either
///    a [MouseCursor] or a [MaterialStateProperty<MouseCursor>].
///  * [MaterialStateOutlinedBorder], an [OutlinedBorder] that implements
///    `MaterialStateProperty` which is used in APIs that need to accept either
///    an [OutlinedBorder] or a [MaterialStateProperty<OutlinedBorder>].
///  * [MaterialStateOutlineInputBorder], an [OutlineInputBorder] that implements
///    `MaterialStateProperty` which is used in APIs that need to accept either
///    an [OutlineInputBorder] or a [MaterialStateProperty<OutlineInputBorder>].
///  * [MaterialStateUnderlineInputBorder], an [UnderlineInputBorder] that implements
///    `MaterialStateProperty` which is used in APIs that need to accept either
///    an [UnderlineInputBorder] or a [MaterialStateProperty<UnderlineInputBorder>].
///  * [MaterialStateBorderSide], a [BorderSide] that implements
///    `MaterialStateProperty` which is used in APIs that need to accept either
///    a [BorderSide] or a [MaterialStateProperty<BorderSide>].
///  * [MaterialStateTextStyle], a [TextStyle] that implements
///    `MaterialStateProperty` which is used in APIs that need to accept either
///    a [TextStyle] or a [MaterialStateProperty<TextStyle>].
/// {@endtemplate}
@Deprecated(
  'Use WidgetState instead. '
  'Moved to the Widgets layer to make code available outside of Material. '
  'This feature was deprecated after v3.19.0-0.3.pre.',
)
typedef MaterialState = WidgetState;

/// Signature for the function that returns a value of type `T` based on a given
/// set of states.
///
/// See also:
///
///  * [WidgetPropertyResolver], the non-Material form of `MaterialPropertyResolver`
///    that can be used interchangeably with `MaterialPropertyResolver.
@Deprecated(
  'Use WidgetPropertyResolver instead. '
  'Moved to the Widgets layer to make code available outside of Material. '
  'This feature was deprecated after v3.19.0-0.3.pre.',
)
typedef MaterialPropertyResolver<T> = WidgetPropertyResolver<T>;

/// Defines a [Color] that is also a [MaterialStateProperty].
///
/// This class exists to enable widgets with [Color] valued properties
/// to also accept [MaterialStateProperty<Color>] values. A material
/// state color property represents a color which depends on
/// a widget's "interactive state". This state is represented as a
/// [Set] of [MaterialState]s, like [MaterialState.pressed],
/// [MaterialState.focused] and [MaterialState.hovered].
///
/// [MaterialStateColor] should only be used with widgets that document
/// their support, like [TimePickerThemeData.dayPeriodColor].
///
/// To use a [MaterialStateColor], you can either:
///   1. Create a subclass of [MaterialStateColor] and implement the abstract `resolve` method.
///   2. Use [MaterialStateColor.resolveWith] and pass in a callback that
///      will be used to resolve the color in the given states.
///
/// If a [MaterialStateColor] is used for a property or a parameter that doesn't
/// support resolving [MaterialStateProperty<Color>]s, then its default color
/// value will be used for all states.
///
/// To define a `const` [MaterialStateColor], you'll need to extend
/// [MaterialStateColor] and override its [resolve] method. You'll also need
/// to provide a `defaultValue` to the super constructor, so that we can know
/// at compile-time what its default color is.
///
/// {@tool snippet}
///
/// This example defines a [MaterialStateColor] with a const constructor.
///
/// ```dart
/// class MyColor extends MaterialStateColor {
///   const MyColor() : super(_defaultColor);
///
///   static const int _defaultColor = 0xcafefeed;
///   static const int _pressedColor = 0xdeadbeef;
///
///   @override
///   Color resolve(Set<MaterialState> states) {
///     if (states.contains(MaterialState.pressed)) {
///       return const Color(_pressedColor);
///     }
///     return const Color(_defaultColor);
///   }
/// }
/// ```
/// {@end-tool}
///
/// See also
///
///  * [WidgetStateColor], the non-Material version that can be used
///    interchangeably with `MaterialStateColor`.
@Deprecated(
  'Use WidgetStateColor instead. '
  'Moved to the Widgets layer to make code available outside of Material. '
  'This feature was deprecated after v3.19.0-0.3.pre.',
)
typedef MaterialStateColor = WidgetStateColor;

/// Defines a [MouseCursor] whose value depends on a set of [MaterialState]s which
/// represent the interactive state of a component.
///
/// This kind of [MouseCursor] is useful when the set of interactive
/// actions a widget supports varies with its state. For example, a
/// mouse pointer hovering over a disabled [ListTile] should not
/// display [SystemMouseCursors.click], since a disabled list tile
/// doesn't respond to mouse clicks. [ListTile]'s default mouse cursor
/// is a [MaterialStateMouseCursor.clickable], which resolves to
/// [SystemMouseCursors.basic] when the button is disabled.
///
/// To use a [MaterialStateMouseCursor], you should create a subclass of
/// [MaterialStateMouseCursor] and implement the abstract `resolve` method.
///
/// {@tool dartpad}
/// This example defines a mouse cursor that resolves to
/// [SystemMouseCursors.forbidden] when its widget is disabled.
///
/// ** See code in examples/api/lib/material/material_state/material_state_mouse_cursor.0.dart **
/// {@end-tool}
///
/// This class should only be used for parameters which are documented to take
/// [MaterialStateMouseCursor], otherwise only the default state will be used.
///
/// See also:
///
///  * [WidgetStateMouseCursor], the non-Material version that can be used
///    interchangeably with `MaterialStateMouseCursor`.
///  * [MouseCursor] for introduction on the mouse cursor system.
///  * [SystemMouseCursors], which defines cursors that are supported by
///    native platforms.
@Deprecated(
  'Use WidgetStateMouseCursor instead. '
  'Moved to the Widgets layer to make code available outside of Material. '
  'This feature was deprecated after v3.19.0-0.3.pre.',
)
typedef MaterialStateMouseCursor = WidgetStateMouseCursor;

/// Defines a [BorderSide] whose value depends on a set of [MaterialState]s
/// which represent the interactive state of a component.
///
/// To use a [MaterialStateBorderSide], you should create a subclass of a
/// [MaterialStateBorderSide] and override the abstract `resolve` method.
///
/// This class enables existing widget implementations with [BorderSide]
/// properties to be extended to also effectively support `MaterialStateProperty<BorderSide>`
/// property values. [MaterialStateBorderSide] should only be used with widgets that document
/// their support, like [ActionChip.side].
///
/// {@tool dartpad}
/// This example defines a subclass of [MaterialStateBorderSide], that resolves
/// to a red border side when its widget is selected.
///
/// ** See code in examples/api/lib/material/material_state/material_state_border_side.0.dart **
/// {@end-tool}
///
/// This class should only be used for parameters which are documented to take
/// [MaterialStateBorderSide], otherwise only the default state will be used.
///
/// See also:
///
///  * [WidgetStateBorderSide], the non-Material version that can be used
///    interchangeably with `MaterialStateBorderSide`.
@Deprecated(
  'Use WidgetStateBorderSide instead. '
  'Moved to the Widgets layer to make code available outside of Material. '
  'This feature was deprecated after v3.19.0-0.3.pre.',
)
typedef MaterialStateBorderSide = WidgetStateBorderSide;

/// Defines an [OutlinedBorder] whose value depends on a set of [MaterialState]s
/// which represent the interactive state of a component.
///
/// To use a [MaterialStateOutlinedBorder], you should create a subclass of an
/// [OutlinedBorder] and implement [MaterialStateOutlinedBorder]'s abstract
/// `resolve` method.
///
/// {@tool dartpad}
/// This example defines a subclass of [RoundedRectangleBorder] and an
/// implementation of [MaterialStateOutlinedBorder], that resolves to
/// [RoundedRectangleBorder] when its widget is selected.
///
/// ** See code in examples/api/lib/material/material_state/material_state_outlined_border.0.dart **
/// {@end-tool}
///
/// This class should only be used for parameters which are documented to take
/// [MaterialStateOutlinedBorder], otherwise only the default state will be used.
///
/// See also:
///
///  * [WidgetStateOutlinedBorder], the non-Material version that can be used
///    interchangeably with `MaterialStateOutlinedBorder`.
///  * [ShapeBorder] the base class for shape outlines.
@Deprecated(
  'Use WidgetStateOutlinedBorder instead. '
  'Moved to the Widgets layer to make code available outside of Material. '
  'This feature was deprecated after v3.19.0-0.3.pre.',
)
typedef MaterialStateOutlinedBorder = WidgetStateOutlinedBorder;

/// Defines a [TextStyle] that is also a [MaterialStateProperty].
///
/// This class exists to enable widgets with [TextStyle] valued properties
/// to also accept [MaterialStateProperty<TextStyle>] values. A material
/// state text style property represents a text style which depends on
/// a widget's "interactive state". This state is represented as a
/// [Set] of [MaterialState]s, like [MaterialState.pressed],
/// [MaterialState.focused] and [MaterialState.hovered].
///
/// [MaterialStateTextStyle] should only be used with widgets that document
/// their support, like [InputDecoration.labelStyle].
///
/// To use a [MaterialStateTextStyle], you can either:
///   1. Create a subclass of [MaterialStateTextStyle] and implement the abstract `resolve` method.
///   2. Use [MaterialStateTextStyle.resolveWith] and pass in a callback that
///      will be used to resolve the color in the given states.
///
/// If a [MaterialStateTextStyle] is used for a property or a parameter that doesn't
/// support resolving [MaterialStateProperty<TextStyle>]s, then its default color
/// value will be used for all states.
///
/// To define a `const` [MaterialStateTextStyle], you'll need to extend
/// [MaterialStateTextStyle] and override its [resolve] method. You'll also need
/// to provide a `defaultValue` to the super constructor, so that we can know
/// at compile-time what its default color is.
///
/// See also:
///
///  * [WidgetStateTextStyle], the non-Material version that can be used
///    interchangeably with `MaterialStateTextStyle`.
@Deprecated(
  'Use WidgetStateTextStyle instead. '
  'Moved to the Widgets layer to make code available outside of Material. '
  'This feature was deprecated after v3.19.0-0.3.pre.',
)
typedef MaterialStateTextStyle = WidgetStateTextStyle;

/// Defines a [OutlineInputBorder] that is also a [MaterialStateProperty].
///
/// This class exists to enable widgets with [OutlineInputBorder] valued properties
/// to also accept [MaterialStateProperty<OutlineInputBorder>] values. A material
/// state input border property represents a text style which depends on
/// a widget's "interactive state". This state is represented as a
/// [Set] of [MaterialState]s, like [MaterialState.pressed],
/// [MaterialState.focused] and [MaterialState.hovered].
///
/// [MaterialStateOutlineInputBorder] should only be used with widgets that document
/// their support, like [InputDecoration.border].
///
/// To use a [MaterialStateOutlineInputBorder], you can either:
///   1. Create a subclass of [MaterialStateOutlineInputBorder] and implement the abstract `resolve` method.
///   2. Use [MaterialStateOutlineInputBorder.resolveWith] and pass in a callback that
///      will be used to resolve the color in the given states.
///
/// If a [MaterialStateOutlineInputBorder] is used for a property or a parameter that doesn't
/// support resolving [MaterialStateProperty<OutlineInputBorder>]s, then its default color
/// value will be used for all states.
///
/// To define a `const` [MaterialStateOutlineInputBorder], you'll need to extend
/// [MaterialStateOutlineInputBorder] and override its [resolve] method. You'll also need
/// to provide a `defaultValue` to the super constructor, so that we can know
/// at compile-time what its default color is.
@Deprecated(
  'Use WidgetStateInputBorder instead. '
  'Renamed to match other WidgetStateProperty objects. '
  'This feature was deprecated after v3.26.0-0.1.pre.',
)
abstract class MaterialStateOutlineInputBorder extends OutlineInputBorder
    implements MaterialStateProperty<InputBorder> {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  @Deprecated(
    'Use WidgetStateInputBorder instead. '
    'Renamed to match other WidgetStateProperty objects. '
    'This feature was deprecated after v3.26.0-0.1.pre.',
  )
  const MaterialStateOutlineInputBorder();

  /// Creates a [MaterialStateOutlineInputBorder] from a [MaterialPropertyResolver<InputBorder>]
  /// callback function.
  ///
  /// If used as a regular input border, the border resolved in the default state (the
  /// empty set of states) will be used.
  ///
  /// The given callback parameter must return a non-null text style in the default
  /// state.
  @Deprecated(
    'Use WidgetStateInputBorder.resolveWith() instead. '
    'Renamed to match other WidgetStateProperty objects. '
    'This feature was deprecated after v3.26.0-0.1.pre.',
  )
  const factory MaterialStateOutlineInputBorder.resolveWith(
    MaterialPropertyResolver<InputBorder> callback,
  ) = _MaterialStateOutlineInputBorder;

  /// Returns a [InputBorder] that's to be used when a Material component is in the
  /// specified state.
  @override
  InputBorder resolve(Set<MaterialState> states);
}

/// A [MaterialStateOutlineInputBorder] created from a [MaterialPropertyResolver<OutlineInputBorder>]
/// callback alone.
///
/// If used as a regular input border, the border resolved in the default state will
/// be used.
///
/// Used by [MaterialStateTextStyle.resolveWith].
class _MaterialStateOutlineInputBorder extends MaterialStateOutlineInputBorder {
  const _MaterialStateOutlineInputBorder(this._resolve);

  final MaterialPropertyResolver<InputBorder> _resolve;

  @override
  InputBorder resolve(Set<MaterialState> states) => _resolve(states);
}

/// Defines a [UnderlineInputBorder] that is also a [MaterialStateProperty].
///
/// This class exists to enable widgets with [UnderlineInputBorder] valued properties
/// to also accept [MaterialStateProperty<UnderlineInputBorder>] values. A material
/// state input border property represents a text style which depends on
/// a widget's "interactive state". This state is represented as a
/// [Set] of [MaterialState]s, like [MaterialState.pressed],
/// [MaterialState.focused] and [MaterialState.hovered].
///
/// [MaterialStateUnderlineInputBorder] should only be used with widgets that document
/// their support, like [InputDecoration.border].
///
/// To use a [MaterialStateUnderlineInputBorder], you can either:
///   1. Create a subclass of [MaterialStateUnderlineInputBorder] and implement the abstract `resolve` method.
///   2. Use [MaterialStateUnderlineInputBorder.resolveWith] and pass in a callback that
///      will be used to resolve the color in the given states.
///
/// If a [MaterialStateUnderlineInputBorder] is used for a property or a parameter that doesn't
/// support resolving [MaterialStateProperty<UnderlineInputBorder>]s, then its default color
/// value will be used for all states.
///
/// To define a `const` [MaterialStateUnderlineInputBorder], you'll need to extend
/// [MaterialStateUnderlineInputBorder] and override its [resolve] method. You'll also need
/// to provide a `defaultValue` to the super constructor, so that we can know
/// at compile-time what its default color is.
@Deprecated(
  'Use WidgetStateInputBorder instead. '
  'Renamed to match other WidgetStateProperty objects. '
  'This feature was deprecated after v3.26.0-0.1.pre.',
)
abstract class MaterialStateUnderlineInputBorder extends UnderlineInputBorder
    implements MaterialStateProperty<InputBorder> {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  @Deprecated(
    'Use WidgetStateInputBorder instead. '
    'Renamed to match other WidgetStateProperty objects. '
    'This feature was deprecated after v3.26.0-0.1.pre.',
  )
  const MaterialStateUnderlineInputBorder();

  /// Creates a [MaterialStateUnderlineInputBorder] from a [MaterialPropertyResolver<InputBorder>]
  /// callback function.
  ///
  /// If used as a regular input border, the border resolved in the default state (the
  /// empty set of states) will be used.
  ///
  /// The given callback parameter must return a non-null text style in the default
  /// state.
  @Deprecated(
    'Use WidgetStateInputBorder.resolveWith() instead. '
    'Renamed to match other WidgetStateProperty objects. '
    'This feature was deprecated after v3.26.0-0.1.pre.',
  )
  const factory MaterialStateUnderlineInputBorder.resolveWith(
    MaterialPropertyResolver<InputBorder> callback,
  ) = _MaterialStateUnderlineInputBorder;

  /// Returns a [InputBorder] that's to be used when a Material component is in the
  /// specified state.
  @override
  InputBorder resolve(Set<MaterialState> states);
}

/// A [MaterialStateUnderlineInputBorder] created from a [MaterialPropertyResolver<UnderlineInputBorder>]
/// callback alone.
///
/// If used as a regular input border, the border resolved in the default state will
/// be used.
///
/// Used by [MaterialStateTextStyle.resolveWith].
class _MaterialStateUnderlineInputBorder extends MaterialStateUnderlineInputBorder {
  const _MaterialStateUnderlineInputBorder(this._resolve);

  final MaterialPropertyResolver<InputBorder> _resolve;

  @override
  InputBorder resolve(Set<MaterialState> states) => _resolve(states);
}

/// Defines an [InputBorder] that is also a [WidgetStateProperty].
///
/// This class exists to enable widgets with [InputBorder] valued properties
/// to also accept [WidgetStateProperty] objects.
///
/// [WidgetStateInputBorder] should only be used with widgets that document
/// their support, like [InputDecoration.border].
///
/// A [WidgetStateInputBorder] can be created by:
///  1. Creating a class that extends [OutlineInputBorder] or [UnderlineInputBorder]
///     and implements [WidgetStateInputBorder]. The class would also need to
///     override the [resolve] method.
///  2. Using [WidgetStateInputBorder.resolveWith] with a callback that
///     resolves the input border in the given states.
///  3. Using [WidgetStateInputBorder.fromMap] to assign a border with a [WidgetStateMap].
///
/// {@tool dartpad}
/// This example shows how to use [WidgetStateInputBorder] to create
/// a [TextField] with an appearance that responds to user interaction.
///
/// ** See code in examples/api/lib/material/widget_state_input_border/widget_state_input_border.0.dart **
/// {@end-tool}
abstract interface class WidgetStateInputBorder
    implements InputBorder, WidgetStateProperty<InputBorder> {
  /// Creates a [WidgetStateInputBorder] using a [WidgetPropertyResolver]
  /// callback.
  ///
  /// This constructor should only be used for fields that support
  /// [WidgetStateInputBorder], such as [InputDecoration.border]
  /// (if used as a regular [InputBorder], it acts the same as
  /// an empty `OutlineInputBorder()` constructor).
  const factory WidgetStateInputBorder.resolveWith(WidgetPropertyResolver<InputBorder> callback) =
      _WidgetStateInputBorder;

  /// Creates a [WidgetStateOutlinedBorder] from a [WidgetStateMap].
  ///
  /// {@macro flutter.widgets.WidgetStateProperty.fromMap}
  /// It should only be used for fields that support [WidgetStateOutlinedBorder]
  /// objects, such as [InputDecoration.border]
  /// (throws an error if used as a regular [OutlinedBorder]).
  ///
  /// {@macro flutter.widgets.WidgetState.any}
  const factory WidgetStateInputBorder.fromMap(WidgetStateMap<InputBorder> map) =
      _WidgetInputBorderMapper;
}

class _WidgetStateInputBorder extends OutlineInputBorder implements WidgetStateInputBorder {
  const _WidgetStateInputBorder(this._resolve);

  final WidgetPropertyResolver<InputBorder> _resolve;

  @override
  InputBorder resolve(Set<WidgetState> states) => _resolve(states);
}

class _WidgetInputBorderMapper extends WidgetStateMapper<InputBorder>
    implements WidgetStateInputBorder {
  const _WidgetInputBorderMapper(super.map);
}

/// Interface for classes that [resolve] to a value of type `T` based
/// on a widget's interactive "state", which is defined as a set
/// of [MaterialState]s.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=CylXr3AF3uU}
///
/// Material state properties represent values that depend on a widget's material
/// "state". The state is encoded as a set of [MaterialState] values, like
/// [MaterialState.focused], [MaterialState.hovered], [MaterialState.pressed]. For
/// example the [InkWell.overlayColor] defines the color that fills the ink well
/// when it's pressed (the "splash color"), focused, or hovered. The [InkWell]
/// uses the overlay color's [resolve] method to compute the color for the
/// ink well's current state.
///
/// [ButtonStyle], which is used to configure the appearance of
/// buttons like [TextButton], [ElevatedButton], and [OutlinedButton],
/// has many material state properties. The button widgets keep track
/// of their current material state and [resolve] the button style's
/// material state properties when their value is needed.
///
/// {@tool dartpad}
/// This example shows how you can override the default text and icon
/// color (the "foreground color") of a [TextButton] with a
/// [MaterialStateProperty]. In this example, the button's text color
/// will be `Colors.blue` when the button is being pressed, hovered,
/// or focused. Otherwise, the text color will be `Colors.red`.
///
/// ** See code in examples/api/lib/material/material_state/material_state_property.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [WidgetStateProperty], the non-Material version that can be used
///    interchangeably with `MaterialStateProperty`.
/// {@macro flutter.material.MaterialStateProperty.implementations}
@Deprecated(
  'Use WidgetStateProperty instead. '
  'Moved to the Widgets layer to make code available outside of Material. '
  'This feature was deprecated after v3.19.0-0.3.pre.',
)
typedef MaterialStateProperty<T> = WidgetStateProperty<T>;

/// Convenience class for creating a [MaterialStateProperty] that
/// resolves to the given value for all states.
///
/// See also:
///
///  * [WidgetStatePropertyAll], the non-Material version that can be used
///    interchangeably with `MaterialStatePropertyAll`.
@Deprecated(
  'Use WidgetStatePropertyAll instead. '
  'Moved to the Widgets layer to make code available outside of Material. '
  'This feature was deprecated after v3.19.0-0.3.pre.',
)
typedef MaterialStatePropertyAll<T> = WidgetStatePropertyAll<T>;

/// Manages a set of [MaterialState]s and notifies listeners of changes.
///
/// Used by widgets that expose their internal state for the sake of
/// extensions that add support for additional states. See
/// [TextButton] for an example.
///
/// The controller's [value] is its current set of states. Listeners
/// are notified whenever the [value] changes. The [value] should only be
/// changed with [update]; it should not be modified directly.
///
/// The controller's [value] represents the set of states that a
/// widget's visual properties, typically [MaterialStateProperty]
/// values, are resolved against. It is _not_ the intrinsic state of
/// the widget. The widget is responsible for ensuring that the
/// controller's [value] tracks its intrinsic state. For example one
/// cannot request the keyboard focus for a widget by adding
/// [MaterialState.focused] to its controller. When the widget gains the
/// or loses the focus it will [update] its controller's [value] and
/// notify listeners of the change.
///
/// When calling `setState` in a [MaterialStatesController] listener, use the
/// [SchedulerBinding.addPostFrameCallback] to delay the call to `setState` after
/// the frame has been rendered. It's generally prudent to use the
/// [SchedulerBinding.addPostFrameCallback] because some of the widgets that
/// depend on [MaterialStatesController] may call [update] in their build method.
/// In such cases, listener's that call `setState` - during the build phase - will cause
/// an error.
///
/// See also:
///
///  * [WidgetStatesController], the non-Material version that can be used
///    interchangeably with `MaterialStatesController`.
@Deprecated(
  'Use WidgetStatesController instead. '
  'Moved to the Widgets layer to make code available outside of Material. '
  'This feature was deprecated after v3.19.0-0.3.pre.',
)
typedef MaterialStatesController = WidgetStatesController;
