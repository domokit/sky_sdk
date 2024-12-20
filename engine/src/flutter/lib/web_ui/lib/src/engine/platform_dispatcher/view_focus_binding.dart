// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'package:meta/meta.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

/// Tracks the [FlutterView]s focus changes.
final class ViewFocusBinding {
  ViewFocusBinding(this._viewManager, this._onViewFocusChange);

  /// Whether [FlutterView] focus changes will be reported and performed.
  ///
  /// DO NOT rely on this bit as it will go away soon. You're warned :)!
  @visibleForTesting
  static bool isEnabled = true;

  final FlutterViewManager _viewManager;
  final ui.ViewFocusChangeCallback _onViewFocusChange;

  int? _lastViewId;
  ui.ViewFocusDirection _viewFocusDirection = ui.ViewFocusDirection.forward;

  StreamSubscription<int>? _onViewCreatedListener;

  void init() {
    // We need a global listener here to know if the user was pressing "shift"
    // when the Flutter view receives focus, to move the Flutter focus to the
    // *last* focusable element.
    domDocument.body?.addEventListener(_keyDown, _handleKeyDown);
    domDocument.body?.addEventListener(_keyUp, _handleKeyUp);

    // If so, update `_handleViewCreated` and add a `_handleViewDisposed` to attach
    // and remove the focus/blur listener.
    _onViewCreatedListener = _viewManager.onViewCreated.listen(_handleViewCreated);
  }

  void dispose() {
    domDocument.body?.removeEventListener(_keyDown, _handleKeyDown);
    domDocument.body?.removeEventListener(_keyUp, _handleKeyUp);
    _onViewCreatedListener?.cancel();
  }

  void changeViewFocus(int viewId, ui.ViewFocusState state) {
    if (!isEnabled) {
      return;
    }
    final DomElement? viewElement = _viewManager[viewId]?.dom.rootElement;

    switch (state) {
      case ui.ViewFocusState.focused:
        // Only move the focus to the flutter view if nothing inside it is focused already.
        if (viewId != _viewId(domDocument.activeElement)) {
          viewElement?.focusWithoutScroll();
        }
      case ui.ViewFocusState.unfocused:
        viewElement?.blur();
    }
  }

  late final DomEventListener _handleFocusin = createDomEventListener((DomEvent event) {
    event as DomFocusEvent;
    _handleFocusChange(event.target as DomElement?);
  });

  late final DomEventListener _handleFocusout = createDomEventListener((DomEvent event) {
    // During focusout processing, activeElement typically points to <body />.
    // However, if an element is focused during a blur event, activeElement points to that focused element.
    // We leverage this behavior to ignore focusout events where the document has focus but activeElement is not <body />.
    //
    // Refer to https://github.com/flutter/engine/pull/54965 for more info.
    final bool wasFocusInvoked =
        domDocument.hasFocus() && domDocument.activeElement != domDocument.body;
    if (wasFocusInvoked) {
      return;
    }

    event as DomFocusEvent;
    _handleFocusChange(event.relatedTarget as DomElement?);
  });

  late final DomEventListener _handleKeyDown = createDomEventListener((DomEvent event) {
    // The right event type needs to be checked because Chrome seems to be firing
    // `Event` events instead of `KeyboardEvent` events when autofilling is used.
    // See https://github.com/flutter/flutter/issues/149968 for more info.
    if (event is DomKeyboardEvent && (event.shiftKey ?? false)) {
      _viewFocusDirection = ui.ViewFocusDirection.backward;
    }
  });

  late final DomEventListener _handleKeyUp = createDomEventListener((DomEvent event) {
    _viewFocusDirection = ui.ViewFocusDirection.forward;
  });

  void _handleFocusChange(DomElement? focusedElement) {
    if (!isEnabled) {
      return;
    }

    final int? viewId = _viewId(focusedElement);
    if (viewId == _lastViewId) {
      return;
    }

    final ui.ViewFocusEvent event;
    if (viewId == null) {
      event = ui.ViewFocusEvent(
        viewId: _lastViewId!,
        state: ui.ViewFocusState.unfocused,
        direction: ui.ViewFocusDirection.undefined,
      );
    } else {
      event = ui.ViewFocusEvent(
        viewId: viewId,
        state: ui.ViewFocusState.focused,
        direction: _viewFocusDirection,
      );
    }
    _updateViewKeyboardReachability(_lastViewId, reachable: true);
    _updateViewKeyboardReachability(viewId, reachable: false);
    _lastViewId = viewId;
    _onViewFocusChange(event);
  }

  int? _viewId(DomElement? element) {
    final FlutterViewManager viewManager = EnginePlatformDispatcher.instance.viewManager;
    return viewManager.findViewForElement(element)?.viewId;
  }

  void _handleViewCreated(int viewId) {
    final DomElement? rootElement = _viewManager[viewId]?.dom.rootElement;

    rootElement?.addEventListener(_focusin, _handleFocusin);
    rootElement?.addEventListener(_focusout, _handleFocusout);

    _updateViewKeyboardReachability(viewId, reachable: true);
  }

  // Controls whether the Flutter view identified by [viewId] is reachable by
  // keyboard.
  void _updateViewKeyboardReachability(int? viewId, {required bool reachable}) {
    if (viewId == null) {
      return;
    }

    final DomElement? rootElement = _viewManager[viewId]?.dom.rootElement;
    // A tabindex with value zero means the DOM element can be reached using the
    // keyboard (tab, shift + tab). When its value is -1 it is still focusable
    // but can't be focused as the result of keyboard events. This is specially
    // important when the semantics tree is enabled as it puts DOM nodes inside
    // the flutter view and having it with a zero tabindex messes the focus
    // traversal order when pressing tab or shift tab.
    rootElement?.setAttribute('tabindex', reachable ? 0 : -1);
  }

  static const String _focusin = 'focusin';
  static const String _focusout = 'focusout';
  static const String _keyDown = 'keydown';
  static const String _keyUp = 'keyup';
}
