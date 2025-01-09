// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'ink_well.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'material_state.dart';

/// Mixin for [State] classes that require knowledge of changing [MaterialState]
/// values for their child widgets.
///
/// This mixin does nothing by mere application to a [State] class, but is
/// helpful when writing `build` methods that include child [InkWell],
/// [GestureDetector], [MouseRegion], or [Focus] widgets. Instead of manually
/// creating handlers for each type of user interaction, such [State] classes can
/// instead provide a `ValueChanged<bool>` function and allow [MaterialStateMixin]
/// to manage the set of active [MaterialState]s, and the calling of [setState]
/// as necessary.
///
/// {@tool snippet}
/// This example shows how to write a [StatefulWidget] that uses the
/// [MaterialStateMixin] class to watch [MaterialState] values.
///
/// ```dart
/// class MyWidget extends StatefulWidget {
///   const MyWidget({super.key, required this.color, required this.child});
///
///   final MaterialStateColor color;
///   final Widget child;
///
///   @override
///   State<MyWidget> createState() => MyWidgetState();
/// }
///
/// class MyWidgetState extends State<MyWidget> with MaterialStateMixin<MyWidget> {
///   @override
///   Widget build(BuildContext context) {
///     return InkWell(
///       onFocusChange: updateMaterialState(MaterialState.focused),
///       child: ColoredBox(
///         color: widget.color.resolve(materialStates),
///         child: widget.child,
///       ),
///     );
///   }
/// }
/// ```
/// {@end-tool}
@optionalTypeArgs
mixin MaterialStateMixin<T extends StatefulWidget> on State<T> {
  /// Managed set of active [MaterialState] values; designed to be passed to
  /// [MaterialStateProperty.resolve] methods.
  ///
  /// To mutate and have [setState] called automatically for you, use
  /// [setMaterialState], [addMaterialState], or [removeMaterialState]. Directly
  /// mutating the set is possible, and may be necessary if you need to alter its
  /// list without calling [setState] (and thus triggering a re-render).
  ///
  /// To check for a single condition, convenience getters [isPressed], [isHovered],
  /// [isFocused], etc, are available for each [MaterialState] value.
  @protected
  Set<MaterialState> materialStates = <MaterialState>{};

  /// Callback factory which accepts a [MaterialState] value and returns a
  /// closure to mutate [materialStates] and call [setState].
  ///
  /// Accepts an optional second named parameter, `onChanged`, which allows
  /// arbitrary functionality to be wired through the [MaterialStateMixin].
  /// If supplied, the [onChanged] function is only called when child widgets
  /// report events that make changes to the current set of [MaterialState]s.
  ///
  /// {@tool snippet}
  /// This example shows how to use the [updateMaterialState] callback factory
  /// in other widgets, including the optional [onChanged] callback.
  ///
  /// ```dart
  /// class MyWidget extends StatefulWidget {
  ///   const MyWidget({super.key, this.onPressed});
  ///
  ///   /// Something important this widget must do when pressed.
  ///   final VoidCallback? onPressed;
  ///
  ///   @override
  ///   State<MyWidget> createState() => MyWidgetState();
  /// }
  ///
  /// class MyWidgetState extends State<MyWidget> with MaterialStateMixin<MyWidget> {
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return ColoredBox(
  ///       color: isPressed ? Colors.black : Colors.white,
  ///       child: InkWell(
  ///         onHighlightChanged: updateMaterialState(
  ///           MaterialState.pressed,
  ///           onChanged: (bool val) {
  ///             if (val) {
  ///               widget.onPressed?.call();
  ///             }
  ///           },
  ///         ),
  ///       ),
  ///     );
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  @protected
  ValueChanged<bool> updateMaterialState(MaterialState key, {ValueChanged<bool>? onChanged}) {
    return (bool value) {
      if (materialStates.contains(key) == value) {
        return;
      }
      setMaterialState(key, value);
      onChanged?.call(value);
    };
  }

  /// Mutator to mark a [MaterialState] value as either active or inactive.
  @protected
  void setMaterialState(MaterialState state, bool isSet) {
    return isSet ? addMaterialState(state) : removeMaterialState(state);
  }

  /// Mutator to mark a [MaterialState] value as active.
  @protected
  void addMaterialState(MaterialState state) {
    if (materialStates.add(state)) {
      setState(() {});
    }
  }

  /// Mutator to mark a [MaterialState] value as inactive.
  @protected
  void removeMaterialState(MaterialState state) {
    if (materialStates.remove(state)) {
      setState(() {});
    }
  }

  /// Getter for whether this class considers [MaterialState.disabled] to be active.
  bool get isDisabled => materialStates.contains(MaterialState.disabled);

  /// Getter for whether this class considers [MaterialState.dragged] to be active.
  bool get isDragged => materialStates.contains(MaterialState.dragged);

  /// Getter for whether this class considers [MaterialState.error] to be active.
  bool get isErrored => materialStates.contains(MaterialState.error);

  /// Getter for whether this class considers [MaterialState.focused] to be active.
  bool get isFocused => materialStates.contains(MaterialState.focused);

  /// Getter for whether this class considers [MaterialState.hovered] to be active.
  bool get isHovered => materialStates.contains(MaterialState.hovered);

  /// Getter for whether this class considers [MaterialState.pressed] to be active.
  bool get isPressed => materialStates.contains(MaterialState.pressed);

  /// Getter for whether this class considers [MaterialState.scrolledUnder] to be active.
  bool get isScrolledUnder => materialStates.contains(MaterialState.scrolledUnder);

  /// Getter for whether this class considers [MaterialState.selected] to be active.
  bool get isSelected => materialStates.contains(MaterialState.selected);

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<Set<MaterialState>>(
        'materialStates',
        materialStates,
        defaultValue: <MaterialState>{},
      ),
    );
  }
}
