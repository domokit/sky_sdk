// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../dom.dart';
import '../platform_dispatcher.dart';
import 'focusable.dart';
import 'label_and_value.dart';
import 'semantics.dart';

/// Adds increment/decrement event handling to a semantics object.
///
/// The implementation uses a hidden `<input type="range">` element with ARIA
/// attributes to cause the browser to render increment/decrement controls to
/// the assistive technology.
///
/// The input element is disabled whenever the gesture mode switches to pointer
/// events. This is to prevent the browser from taking over drag gestures. Drag
/// gestures must be interpreted by the Flutter framework.
class SemanticIncrementable extends SemanticRole {
  SemanticIncrementable(SemanticsObject semanticsObject)
    : _focusManager = AccessibilityFocusManager(semanticsObject.owner),
      super.blank(SemanticRoleKind.incrementable, semanticsObject) {
    // The following generic roles can coexist with incrementables. Generic focus
    // management is not used by this role because the root DOM element is not
    // the one being focused on, but the internal `<input>` element.
    addLiveRegion();
    addRouteName();
    addLabelAndValue(preferredRepresentation: LabelRepresentation.ariaLabel);

    append(_element);
    _element.type = 'range';
    _element.setAttribute('role', 'slider');

    _element.addEventListener(
      'change',
      createDomEventListener((_) {
        if (_element.disabled!) {
          return;
        }
        _pendingResync = true;
        final int newInputValue = int.parse(_element.value!);
        if (newInputValue > _currentSurrogateValue) {
          _currentSurrogateValue += 1;
          EnginePlatformDispatcher.instance.invokeOnSemanticsAction(
            viewId,
            semanticsObject.id,
            ui.SemanticsAction.increase,
            null,
          );
        } else if (newInputValue < _currentSurrogateValue) {
          _currentSurrogateValue -= 1;
          EnginePlatformDispatcher.instance.invokeOnSemanticsAction(
            viewId,
            semanticsObject.id,
            ui.SemanticsAction.decrease,
            null,
          );
        }
      }),
    );

    // Store the callback as a closure because Dart does not guarantee that
    // tear-offs produce the same function object.
    _gestureModeListener = (GestureMode mode) {
      update();
    };
    EngineSemantics.instance.addGestureModeListener(_gestureModeListener);
    _focusManager.manage(semanticsObject.id, _element);
  }

  @override
  bool focusAsRouteDefault() {
    _element.focusWithoutScroll();
    return true;
  }

  /// The HTML element used to render semantics to the browser.
  final DomHTMLInputElement _element = createDomHTMLInputElement();

  final AccessibilityFocusManager _focusManager;

  /// The value used by the input element.
  ///
  /// Flutter values are strings, and are not necessarily numbers. In order to
  /// convey to the browser what the available "range" of values is we
  /// substitute the framework value with a generated `int` surrogate.
  /// "aria-valuetext" attribute is used to cause the browser to announce the
  /// framework value to the user.
  int _currentSurrogateValue = 1;

  /// Disables the input [_element] when the gesture mode switches to
  /// [GestureMode.pointerEvents], and enables it when the mode switches back to
  /// [GestureMode.browserGestures].
  late final GestureModeCallback _gestureModeListener;

  /// Whether we forwarded a semantics action to the framework and awaiting an
  /// update.
  ///
  /// This field is used to determine whether the HTML DOM of the semantics
  /// tree should be updated.
  bool _pendingResync = false;

  @override
  void update() {
    super.update();

    switch (EngineSemantics.instance.gestureMode) {
      case GestureMode.browserGestures:
        _enableBrowserGestureHandling();
        _updateInputValues();
      case GestureMode.pointerEvents:
        _disableBrowserGestureHandling();
    }
    _focusManager.changeFocus(semanticsObject.hasFocus);
  }

  void _enableBrowserGestureHandling() {
    assert(EngineSemantics.instance.gestureMode == GestureMode.browserGestures);
    if (!_element.disabled!) {
      return;
    }
    _element.disabled = false;
  }

  void _updateInputValues() {
    assert(EngineSemantics.instance.gestureMode == GestureMode.browserGestures);

    final bool updateNeeded =
        _pendingResync ||
        semanticsObject.isValueDirty ||
        semanticsObject.isIncreasedValueDirty ||
        semanticsObject.isDecreasedValueDirty;

    if (!updateNeeded) {
      return;
    }

    _pendingResync = false;

    final String surrogateTextValue = '$_currentSurrogateValue';
    _element.value = surrogateTextValue;
    _element.setAttribute('aria-valuenow', surrogateTextValue);
    _element.setAttribute('aria-valuetext', semanticsObject.value!);

    final bool canIncrease = semanticsObject.increasedValue!.isNotEmpty;
    final String surrogateMaxTextValue =
        canIncrease ? '${_currentSurrogateValue + 1}' : surrogateTextValue;
    _element.max = surrogateMaxTextValue;
    _element.setAttribute('aria-valuemax', surrogateMaxTextValue);

    final bool canDecrease = semanticsObject.decreasedValue!.isNotEmpty;
    final String surrogateMinTextValue =
        canDecrease ? '${_currentSurrogateValue - 1}' : surrogateTextValue;
    _element.min = surrogateMinTextValue;
    _element.setAttribute('aria-valuemin', surrogateMinTextValue);
  }

  void _disableBrowserGestureHandling() {
    if (_element.disabled!) {
      return;
    }
    _element.disabled = true;
  }

  @override
  void dispose() {
    super.dispose();
    _focusManager.stopManaging();
    EngineSemantics.instance.removeGestureModeListener(_gestureModeListener);
    _disableBrowserGestureHandling();
    _element.remove();
  }
}
