// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../dom.dart';
import '../platform_dispatcher.dart';
import '../util.dart';
import 'semantics.dart';

/// Supplies generic accessibility focus features to semantics nodes that have
/// [ui.SemanticsFlag.isFocusable] set.
///
/// Assumes that the element being focused on is [SemanticsObject.element].
/// Semantic roles with special needs can implement custom focus management and
/// exclude this behavior.
///
/// `"tab-index=0"` is used because `<flt-semantics>` is not intrinsically
/// focusable. Examples of intrinsically focusable elements include:
///
///   * <button>
///   * <input> (of any type)
///   * <a>
///   * <textarea>
///
/// See also:
///
///   * https://developer.mozilla.org/en-US/docs/Web/Accessibility/Keyboard-navigable_JavaScript_widgets
class Focusable extends SemanticBehavior {
  Focusable(super.semanticsObject, super.owner)
    : _focusManager = AccessibilityFocusManager(semanticsObject.owner);

  final AccessibilityFocusManager _focusManager;

  /// Requests focus as a result of a route (e.g. dialog) deciding that the node
  /// managed by this class should be focused by default when nothing requests
  /// focus explicitly.
  ///
  /// This method of taking focus is different from the regular method of using
  /// the [SemanticsObject.hasFocus] flag, as in this case the framework did not
  /// explicitly request focus. Instead, the DOM element is being focus directly
  /// programmatically, simulating the screen reader choosing a default element
  /// to focus on.
  ///
  /// Returns `true` if the node took the focus. Returns `false` if the node did
  /// not take the focus. The return value can be used to decide whether to stop
  /// searching for a node that should take focus.
  bool focusAsRouteDefault() {
    _focusManager._lastEvent = AccessibilityFocusManagerEvent.requestedFocus;
    owner.element.focusWithoutScroll();
    return true;
  }

  @override
  void update() {
    if (semanticsObject.isFocusable) {
      if (!_focusManager.isManaging) {
        _focusManager.manage(semanticsObject.id, owner.element);
      }
      _focusManager.changeFocus(
        semanticsObject.hasFocus && (!semanticsObject.hasEnabledState || semanticsObject.isEnabled),
      );
    } else {
      _focusManager.stopManaging();
    }
  }

  @override
  void dispose() {
    super.dispose();
    _focusManager.stopManaging();
  }
}

/// Objects associated with the element whose focus is being managed.
typedef _FocusTarget =
    ({
      /// [SemanticsObject.id] of the semantics node being managed.
      int semanticsNodeId,

      /// The element whose focus is being managed.
      DomElement element,

      /// The listener for the "focus" DOM event.
      DomEventListener domFocusListener,

      /// The listener for the "blur" DOM event.
      DomEventListener domBlurListener,
    });

enum AccessibilityFocusManagerEvent {
  /// No event has happend for the target element.
  nothing,

  /// The engine requested focus on the DOM element, possibly because the
  /// framework requested it.
  requestedFocus,

  /// Received the DOM "focus" event.
  receivedDomFocus,

  /// Received the DOM "blur" event.
  receivedDomBlur,
}

/// Implements accessibility focus management for arbitrary elements.
///
/// Unlike [Focusable], which implements focus features on [SemanticsObject]s
/// whose [SemanticsObject.element] is directly focusable, this class can help
/// implementing focus features on custom elements. For example,
/// [SemanticIncrementable] uses a custom `<input>` tag internally while its
/// root-level element is not focusable. However, it can still use this class to
/// manage the focus of the internal element.
class AccessibilityFocusManager {
  /// Creates a focus manager tied to a specific [EngineSemanticsOwner].
  AccessibilityFocusManager(this._owner);

  final EngineSemanticsOwner _owner;

  _FocusTarget? _target;

  AccessibilityFocusManagerEvent _lastEvent = AccessibilityFocusManagerEvent.nothing;

  // The last focus value set by this focus manager, used to prevent requesting
  // focus on the same element repeatedly. Requesting focus on DOM elements is
  // not an idempotent operation. If the element is already focused and focus is
  // requested the browser will scroll to that element. However, scrolling is
  // not this class' concern and so this class should avoid doing anything that
  // would affect scrolling.
  bool? _lastSetValue;

  /// Whether this focus manager is managing a focusable target.
  bool get isManaging => _target != null;

  /// Starts managing the focus of the given [element].
  ///
  /// The "focus" and "blur" DOM events are forwarded to the framework-side
  /// semantics node with ID [semanticsNodeId] as [ui.SemanticsAction]s.
  ///
  /// If this manage was already managing a different element, stops managing
  /// the old element and starts managing the new one.
  ///
  /// Calling this with the same element but a different [semanticsNodeId] will
  /// cause any future focus/blur events to be forwarded to the new ID.
  void manage(int semanticsNodeId, DomElement element) {
    if (identical(element, _target?.element)) {
      final _FocusTarget previousTarget = _target!;
      if (semanticsNodeId == previousTarget.semanticsNodeId) {
        return;
      }

      // No need to hook up new DOM listeners. The existing ones are good enough.
      _target = (
        semanticsNodeId: semanticsNodeId,
        element: previousTarget.element,
        domFocusListener: previousTarget.domFocusListener,
        domBlurListener: previousTarget.domBlurListener,
      );
      return;
    }

    if (_target != null) {
      // The element changed. Clear the old element before initializing the new one.
      stopManaging();
    }

    final _FocusTarget newTarget = (
      semanticsNodeId: semanticsNodeId,
      element: element,
      domFocusListener: createDomEventListener((_) => _didReceiveDomFocus()),
      domBlurListener: createDomEventListener((_) => _didReceiveDomBlur()),
    );
    _target = newTarget;
    _lastEvent = AccessibilityFocusManagerEvent.nothing;

    element.tabIndex = 0;
    element.addEventListener('focus', newTarget.domFocusListener);
    element.addEventListener('blur', newTarget.domBlurListener);
  }

  /// Stops managing the focus of the current element, if any.
  void stopManaging() {
    final _FocusTarget? target = _target;
    _target = null;
    _lastSetValue = null;

    if (target == null) {
      /// Nothing is being managed. Just return.
      return;
    }

    target.element.removeEventListener('focus', target.domFocusListener);
    target.element.removeEventListener('blur', target.domBlurListener);
  }

  void _didReceiveDomFocus() {
    final _FocusTarget? target = _target;

    if (target == null) {
      // DOM events can be asynchronous. By the time the event reaches here, the
      // focus manager may have been disposed of.
      return;
    }

    // Do not notify the framework if DOM focus was acquired as a result of
    // requesting it programmatically. Only notify the framework if the DOM
    // focus was initiated by the browser, e.g. as a result of the screen reader
    // shifting focus.
    if (_lastEvent != AccessibilityFocusManagerEvent.requestedFocus) {
      EnginePlatformDispatcher.instance.invokeOnSemanticsAction(
        _owner.viewId,
        target.semanticsNodeId,
        ui.SemanticsAction.focus,
        null,
      );
    }

    _lastEvent = AccessibilityFocusManagerEvent.receivedDomFocus;
  }

  void _didReceiveDomBlur() {
    _lastEvent = AccessibilityFocusManagerEvent.receivedDomBlur;
  }

  /// Requests focus or blur on the DOM element.
  void changeFocus(bool value) {
    final _FocusTarget? target = _target;

    if (target == null) {
      // If this branch is being executed, there's a bug somewhere already, but
      // it doesn't hurt to clean up old values anyway.
      _lastSetValue = null;

      // Nothing is being managed right now.
      assert(() {
        printWarning(
          'Cannot change focus to $value. No element is being managed by this '
          'AccessibilityFocusManager.',
        );
        return true;
      }());
      return;
    }

    if (value == _lastSetValue) {
      // The focus is being changed to a value that's already been requested in
      // the past. Do nothing.
      return;
    }
    _lastSetValue = value;

    if (value) {
      _owner.willRequestFocus();
    } else {
      // Do not blur elements. Instead let the element be blurred by requesting
      // focus elsewhere. Blurring elements is a very error-prone thing to do,
      // as it is subject to non-local effects. Let's say the framework decides
      // that a semantics node is currently not focused. That would lead to
      // changeFocus(false) to be called. However, what if this node is inside
      // a route, and nothing else in the route is focused? The Flutter
      // framework expects that the screen reader will focus on the first (in
      // traversal order) focusable element inside the route and send a
      // SemanticsAction.focus action. Screen readers on the web do not do
      // that, and so the web engine has to implement this behavior directly. So
      // the route will look for a focusable element and request focus on it,
      // but now there may be a race between this method unsetting the focus and
      // the route requesting focus on the same element.
      return;
    }

    // Delay the focus request until the final DOM structure is established
    // because the element may not yet be attached to the DOM, or it may be
    // reparented and lose focus again.
    _owner.addOneTimePostUpdateCallback(() {
      if (_target != target) {
        // The element may have been swapped or the manager may have been disposed
        // of between the focus change request and the post update callback
        // invocation. So check again that the element is still the same and is
        // not null.
        return;
      }

      _lastEvent = AccessibilityFocusManagerEvent.requestedFocus;
      target.element.focusWithoutScroll();
    });
  }
}
