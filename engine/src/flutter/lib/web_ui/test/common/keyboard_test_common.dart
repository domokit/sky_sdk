// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/src/engine.dart';

typedef VoidCallback = void Function();

class MockKeyboardEvent implements FlutterHtmlKeyboardEvent {
  MockKeyboardEvent({
    required this.type,
    required this.code,
    required this.key,
    this.timeStamp = 0,
    this.repeat = false,
    this.keyCode = 0,
    this.isComposing = false,
    bool altKey = false,
    bool ctrlKey = false,
    bool shiftKey = false,
    bool metaKey = false,
    bool altGrKey = false,
    this.location = 0,
    this.onPreventDefault,
    this.onStopPropagation,
  }) : modifierState = <String>{
         if (altKey) 'Alt',
         if (ctrlKey) 'Control',
         if (shiftKey) 'Shift',
         if (metaKey) 'Meta',
         if (altGrKey) 'AltGraph',
       } {
    _lastEvent = this;
  }

  @override
  String type;

  @override
  String? code;

  @override
  String? key;

  @override
  bool? repeat;

  @override
  int keyCode;

  @override
  num? timeStamp;

  @override
  bool isComposing;

  @override
  bool get altKey => modifierState.contains('Alt');

  @override
  bool get ctrlKey => modifierState.contains('Control');

  @override
  bool get shiftKey => modifierState.contains('Shift');

  @override
  bool get metaKey => modifierState.contains('Meta');

  @override
  int? location;

  @override
  bool getModifierState(String key) => modifierState.contains(key);
  final Set<String> modifierState;

  @override
  void preventDefault() {
    onPreventDefault?.call();
    _defaultPrevented = true;
  }

  VoidCallback? onPreventDefault;

  @override
  bool get defaultPrevented => _defaultPrevented;
  bool _defaultPrevented = false;

  @override
  void stopPropagation() {
    onStopPropagation?.call();
  }

  VoidCallback? onStopPropagation;

  static bool get lastDefaultPrevented => _lastEvent?.defaultPrevented ?? false;
  static MockKeyboardEvent? _lastEvent;
}
