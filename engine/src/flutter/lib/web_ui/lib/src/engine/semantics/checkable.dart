// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// TODO(yjbanov): TalkBack on Android incorrectly reads state changes for radio
//                buttons. When checking a radio button it reads
//                "Checked, not checked". This is likely due to another radio
//                button automatically becoming unchecked. VoiceOver reads it
//                correctly. It is possible we can fix this by using
//                "radiogroup" and "aria-owns". This may require a change in the
//                framework. Currently the framework does not report the
//                grouping of radio buttons.

import 'package:ui/ui.dart' as ui;

import 'label_and_value.dart';
import 'semantics.dart';

/// The specific type of checkable control.
enum _CheckableKind {
  /// A checkbox. An element, which has [ui.SemanticsFlag.hasCheckedState] set
  /// and does not have [ui.SemanticsFlag.isInMutuallyExclusiveGroup] or
  /// [ui.SemanticsFlag.hasToggledState] state, is marked as a checkbox.
  checkbox,

  /// A radio button, defined by [ui.SemanticsFlag.isInMutuallyExclusiveGroup].
  radio,

  /// A switch, defined by [ui.SemanticsFlag.hasToggledState].
  toggle,
}

_CheckableKind _checkableKindFromSemanticsFlag(SemanticsObject semanticsObject) {
  if (semanticsObject.hasFlag(ui.SemanticsFlag.isInMutuallyExclusiveGroup)) {
    return _CheckableKind.radio;
  } else if (semanticsObject.hasFlag(ui.SemanticsFlag.hasToggledState)) {
    return _CheckableKind.toggle;
  } else {
    return _CheckableKind.checkbox;
  }
}

/// Renders semantics objects that have checkable (on/off) states.
///
/// Three objects which are implemented by this class are checkboxes, radio
/// buttons and switches.
///
/// See also [ui.SemanticsFlag.hasCheckedState], [ui.SemanticsFlag.isChecked],
/// [ui.SemanticsFlag.isInMutuallyExclusiveGroup], [ui.SemanticsFlag.isToggled],
/// [ui.SemanticsFlag.hasToggledState].
///
/// See also [Selectable] behavior, which expresses a similar but different
/// boolean state of being "selected".
class SemanticCheckable extends SemanticRole {
  SemanticCheckable(SemanticsObject semanticsObject)
    : _kind = _checkableKindFromSemanticsFlag(semanticsObject),
      super.withBasics(
        SemanticRoleKind.checkable,
        semanticsObject,
        preferredLabelRepresentation: LabelRepresentation.ariaLabel,
      ) {
    addTappable();
  }

  final _CheckableKind _kind;

  @override
  void update() {
    super.update();

    if (semanticsObject.isFlagsDirty) {
      switch (_kind) {
        case _CheckableKind.checkbox:
          setAriaRole('checkbox');
        case _CheckableKind.radio:
          setAriaRole('radio');
        case _CheckableKind.toggle:
          setAriaRole('switch');
      }

      /// Adding disabled and aria-disabled attribute to notify the assistive
      /// technologies of disabled elements.
      _updateDisabledAttribute();

      setAttribute(
        'aria-checked',
        (semanticsObject.hasFlag(ui.SemanticsFlag.isChecked) ||
                semanticsObject.hasFlag(ui.SemanticsFlag.isToggled))
            ? 'true'
            : 'false',
      );
    }
  }

  @override
  void dispose() {
    super.dispose();
    _removeDisabledAttribute();
  }

  void _updateDisabledAttribute() {
    if (semanticsObject.enabledState() == EnabledState.disabled) {
      setAttribute('aria-disabled', 'true');
      setAttribute('disabled', 'true');
    } else {
      _removeDisabledAttribute();
    }
  }

  void _removeDisabledAttribute() {
    removeAttribute('aria-disabled');
    removeAttribute('disabled');
  }

  @override
  bool focusAsRouteDefault() => focusable?.focusAsRouteDefault() ?? false;
}

/// Adds selectability behavior to a semantic node.
///
/// A selectable node would have the `aria-selected` set to "true" if the node
/// is currently selected (i.e. [SemanticsObject.isSelected] is true), and set
/// to "false" if it's not selected (i.e. [SemanticsObject.isSelected] is
/// false). If the node is not selectable (i.e. [SemanticsObject.isSelectable]
/// is false), then `aria-selected` is unset.
///
/// See also [SemanticCheckable], which expresses a similar but different
/// boolean state of being "checked" or "toggled".
class Selectable extends SemanticBehavior {
  Selectable(super.semanticsObject, super.owner);

  @override
  void update() {
    if (semanticsObject.isFlagsDirty) {
      if (semanticsObject.isSelectable) {
        owner.setAttribute('aria-selected', semanticsObject.isSelected);
      } else {
        owner.removeAttribute('aria-selected');
      }
    }
  }
}
