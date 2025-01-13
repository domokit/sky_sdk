// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

/// Sets the "button" ARIA role.
class SemanticButton extends SemanticRole {
  SemanticButton(SemanticsObject semanticsObject)
    : super.withBasics(
        EngineSemanticsRole.button,
        semanticsObject,
        preferredLabelRepresentation: LabelRepresentation.domText,
      ) {
    addTappable();
    setAriaRole('button');
  }

  @override
  bool focusAsRouteDefault() => focusable?.focusAsRouteDefault() ?? false;

  @override
  void update() {
    super.update();

    if (semanticsObject.enabledState() == EnabledState.disabled) {
      setAttribute('aria-disabled', 'true');
    } else {
      removeAttribute('aria-disabled');
    }
  }
}

/// Implements clicking and tapping behavior for a semantics node.
///
/// Listens to HTML DOM "click" events detected by the browser.
///
/// A DOM "click" is different from the click and tap gestures detected by the
/// framework from raw pointer events. When an assistive technology is enabled
/// the browser may not send us pointer events. In that mode we forward HTML
/// click as [ui.SemanticsAction.tap].
///
/// See also [ClickDebouncer].
class Tappable extends SemanticBehavior {
  Tappable(super.semanticsObject, super.owner) {
    _clickListener = createDomEventListener((DomEvent click) {
      PointerBinding.clickDebouncer.onClick(click, viewId, semanticsObject.id, _isListening);
    });
    owner.element.addEventListener('click', _clickListener);
  }

  @override
  bool get acceptsPointerEvents => true;

  DomEventListener? _clickListener;
  bool _isListening = false;

  @override
  void update() {
    final bool wasListening = _isListening;
    _isListening =
        semanticsObject.enabledState() != EnabledState.disabled && semanticsObject.isTappable;
    if (wasListening != _isListening) {
      _updateAttribute();
    }
  }

  void _updateAttribute() {
    // The `flt-tappable` attribute marks the element for the ClickDebouncer to
    // to know that it should debounce click events on this element. The
    // contract is that the element that has this attribute is also the element
    // that receives pointer and "click" events.
    if (_isListening) {
      owner.element.setAttribute('flt-tappable', '');
    } else {
      owner.element.removeAttribute('flt-tappable');
    }
  }

  @override
  void dispose() {
    owner.removeEventListener('click', _clickListener);
    _clickListener = null;
    super.dispose();
  }
}
