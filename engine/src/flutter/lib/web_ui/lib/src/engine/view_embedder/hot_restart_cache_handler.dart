// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:meta/meta.dart';
import 'package:ui/src/engine.dart';

import '../dom.dart';

/// This is state persistent across hot restarts that indicates what
/// to clear.  Delay removal of old visible state to make the
/// transition appear smooth.
@JS('window.__flutterState')
external JSArray<JSAny?>? get _jsHotRestartStore;

@JS('window.__flutterState')
external set _jsHotRestartStore(JSArray<JSAny?>? nodes);

/// Handles [DomElement]s that need to be removed after a hot-restart.
///
/// This class shouldn't be used directly. It's only made public for testing
/// purposes. Instead, use [registerElementForCleanup].
///
/// Elements are stored in a [JSArray] stored globally at `window.__flutterState`.
///
/// When the app hot-restarts (and a new instance of this class is created),
/// all elements in the global [JSArray] is removed from the DOM.
class HotRestartCacheHandler {
  @visibleForTesting
  HotRestartCacheHandler() {
    _resetHotRestartStore();
  }

  /// Removes every element that was registered prior to the hot-restart from
  /// the DOM.
  void _resetHotRestartStore() {
    final JSArray<JSAny?>? jsStore = _jsHotRestartStore;

    if (jsStore != null) {
      // We are in a post hot-restart world, clear the elements now.
      final List<Object?> store = jsStore.toObjectShallow as List<Object?>;
      for (final Object? element in store) {
        if (element != null) {
          (element as DomElement).remove();
        }
      }
    }
    _jsHotRestartStore = JSArray<JSAny?>();
  }

  /// Registers a [DomElement] to be removed after hot-restart.
  @visibleForTesting
  void registerElement(DomElement element) {
    _jsHotRestartStore!.push(element as JSObject);
  }
}

final HotRestartCacheHandler? _hotRestartCache = () {
  // In release mode, we don't need a hot restart cache, so we leave it null.
  HotRestartCacheHandler? cache;
  assert(() {
    cache = HotRestartCacheHandler();
    return true;
  }());
  return cache;
}();

/// Registers a [DomElement] to be cleaned up after hot restart.
void registerElementForCleanup(DomElement element) {
  _hotRestartCache?.registerElement(element);
}
