// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../engine.dart' show registerHotRestartListener;
import 'dom.dart';
import 'keyboard_binding.dart';
import 'platform_dispatcher.dart';
import 'services.dart';

/// Provides keyboard bindings, such as the `flutter/keyevent` channel.
class RawKeyboard {
  RawKeyboard._(this._onMacOs) {
    registerHotRestartListener(() {
      dispose();
    });
  }

  /// Initializes the [RawKeyboard] singleton.
  ///
  /// Use the [instance] getter to get the singleton after calling this method.
  static void initialize({bool onMacOs = false}) {
    _instance ??= RawKeyboard._(onMacOs);
    // KeyboardBinding is responsible for forwarding the keyboard
    // events to the RawKeyboard handler.
    KeyboardBinding.initInstance();
  }

  /// The [RawKeyboard] singleton.
  static RawKeyboard? get instance => _instance;
  static RawKeyboard? _instance;

  /// A mapping of [KeyboardEvent.code] to [Timer].
  ///
  /// The timer is for when to synthesize a keyup for the [KeyboardEvent.code]
  /// if no repeat events were received.
  final Map<String, Timer> _keydownTimers = <String, Timer>{};

  /// Uninitializes the [RawKeyboard] singleton.
  ///
  /// After calling this method this object becomes unusable and [instance]
  /// becomes `null`. Call [initialize] again to initialize a new singleton.
  void dispose() {
    for (final String key in _keydownTimers.keys) {
      _keydownTimers[key]!.cancel();
    }
    _keydownTimers.clear();

    _instance = null;
  }

  static const JSONMessageCodec _messageCodec = JSONMessageCodec();

  /// Contains meta state from the latest event.
  ///
  /// Initializing with `0x0` which means no meta keys are pressed.
  int _lastMetaState = 0x0;

  final bool _onMacOs;

  // When the user enters a browser/system shortcut (e.g. `cmd+alt+i`) on macOS,
  // the browser doesn't send a keyup for it. This puts the framework in a
  // corrupt state because it thinks the key was never released.
  //
  // To avoid this, we rely on the fact that browsers send repeat events
  // while the key is held down by the user. If we don't receive a repeat
  // event within a specific duration ([_kKeydownCancelDurationMac]) we assume
  // the user has released the key and we synthesize a keyup event.
  bool _shouldDoKeyGuard() {
    return _onMacOs;
  }

  bool _shouldIgnore(FlutterHtmlKeyboardEvent event) {
    // During IME composition, Tab fires twice (once for composition and once
    // for regular tabbing behavior), which causes issues. Intercepting the
    // tab keydown event during composition prevents these issues from occurring.
    // https://developer.mozilla.org/en-US/docs/Web/API/Element/keydown_event#ignoring_keydown_during_ime_composition
    return event.type == 'keydown' && event.key == 'Tab' && event.isComposing;
  }

  void handleHtmlEvent(DomEvent domEvent) {
    if (!domInstanceOfString(domEvent, 'KeyboardEvent')) {
      return;
    }

    final FlutterHtmlKeyboardEvent event = FlutterHtmlKeyboardEvent(domEvent as DomKeyboardEvent);
    final String timerKey = event.code!;

    if (_shouldIgnore(event)) {
      return;
    }

    // Don't handle synthesizing a keyup event for modifier keys
    if (!_isModifierKey(event) && _shouldDoKeyGuard()) {
      _keydownTimers[timerKey]?.cancel();

      // Only keys affected by modifiers, require synthesizing
      // because the browser always sends a keyup event otherwise
      if (event.type == 'keydown' && _isAffectedByModifiers(event)) {
        _keydownTimers[timerKey] = Timer(_kKeydownCancelDurationMac, () {
          _keydownTimers.remove(timerKey);
          _synthesizeKeyup(event);
        });
      } else {
        _keydownTimers.remove(timerKey);
      }
    }

    _lastMetaState = _getMetaState(event);
    if (event.type == 'keydown') {
      // For lock modifiers _getMetaState won't report a metaState at keydown.
      // See https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent/getModifierState.
      if (event.key == 'CapsLock') {
        _lastMetaState |= modifierCapsLock;
      } else if (event.code == 'NumLock') {
        _lastMetaState |= modifierNumLock;
      } else if (event.key == 'ScrollLock') {
        _lastMetaState |= modifierScrollLock;
      } else if (event.key == 'Meta' &&
          ui_web.browser.operatingSystem == ui_web.OperatingSystem.linux) {
        // On Chrome Linux, metaState can be wrong when a Meta key is pressed.
        _lastMetaState |= _modifierMeta;
      } else if (event.code == 'MetaLeft' && event.key == 'Process') {
        // When Meta key is pressed, browsers can emit an event whose key is 'Process'.
        // See https://github.com/flutter/flutter/issues/141186.
        _lastMetaState |= _modifierMeta;
      }
    }
    final Map<String, dynamic> eventData = <String, dynamic>{
      'type': event.type,
      'keymap': 'web',
      'code': event.code,
      'key': event.key,
      'location': event.location,
      'metaState': _lastMetaState,
      'keyCode': event.keyCode,
    };

    EnginePlatformDispatcher.instance.invokeOnPlatformMessage(
      'flutter/keyevent',
      _messageCodec.encodeMessage(eventData),
      (ByteData? data) {
        if (data == null) {
          return;
        }
        final Map<String, dynamic> jsonResponse =
            _messageCodec.decodeMessage(data) as Map<String, dynamic>;
        if (jsonResponse['handled'] as bool) {
          // If the framework handled it, then don't propagate it any further.
          event.preventDefault();
          event.stopPropagation();
        }
      },
    );
  }

  void _synthesizeKeyup(FlutterHtmlKeyboardEvent event) {
    final Map<String, dynamic> eventData = <String, dynamic>{
      'type': 'keyup',
      'keymap': 'web',
      'code': event.code,
      'key': event.key,
      'location': event.location,
      'metaState': _lastMetaState,
      'keyCode': event.keyCode,
    };

    EnginePlatformDispatcher.instance.invokeOnPlatformMessage(
      'flutter/keyevent',
      _messageCodec.encodeMessage(eventData),
      _noopCallback,
    );
  }

  /// After a keydown is received, this is the duration we wait for a repeat event
  /// before we decide to synthesize a keyup event.
  ///
  /// This value is only for macOS, where the keyboard repeat delay goes up to
  /// 2000ms.
  static const Duration _kKeydownCancelDurationMac = Duration(milliseconds: 2000);
}

const int _modifierNone = 0x00;
const int _modifierShift = 0x01;
const int _modifierAlt = 0x02;
const int _modifierControl = 0x04;
const int _modifierMeta = 0x08;
const int modifierNumLock = 0x10;
const int modifierCapsLock = 0x20;
const int modifierScrollLock = 0x40;

/// Creates a bitmask representing the meta state of the [event].
int _getMetaState(FlutterHtmlKeyboardEvent event) {
  int metaState = _modifierNone;
  if (event.getModifierState('Shift')) {
    metaState |= _modifierShift;
  }
  if (event.getModifierState('Alt') || event.getModifierState('AltGraph')) {
    metaState |= _modifierAlt;
  }
  if (event.getModifierState('Control')) {
    metaState |= _modifierControl;
  }
  if (event.getModifierState('Meta')) {
    metaState |= _modifierMeta;
  }
  // See https://github.com/flutter/flutter/issues/66601 for why we don't
  // set the ones below based on persistent state.
  // if (event.getModifierState("CapsLock")) {
  //   metaState |= modifierCapsLock;
  // }
  // if (event.getModifierState("NumLock")) {
  //   metaState |= modifierNumLock;
  // }
  // if (event.getModifierState("ScrollLock")) {
  //   metaState |= modifierScrollLock;
  // }
  return metaState;
}

/// Returns true if the [event] was caused by a modifier key.
///
/// Modifier keys are shift, alt, ctrl and meta/cmd/win. These are the keys used
/// to perform keyboard shortcuts (e.g. `cmd+c`, `cmd+l`).
bool _isModifierKey(FlutterHtmlKeyboardEvent event) {
  final String key = event.key!;
  return key == 'Meta' || key == 'Shift' || key == 'Alt' || key == 'Control';
}

/// Returns true if the [event] is been affects by any of the modifiers key
///
/// This is a strong indication that this key is been used for a shortcut
bool _isAffectedByModifiers(FlutterHtmlKeyboardEvent event) {
  return event.ctrlKey || event.shiftKey || event.altKey || event.metaKey;
}

void _noopCallback(ByteData? data) {}
