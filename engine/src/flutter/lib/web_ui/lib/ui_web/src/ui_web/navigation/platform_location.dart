// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:ui/src/engine.dart';

import 'url_strategy.dart';

/// Function type that handles pop state events.
typedef EventListener = dynamic Function(Object event);

/// Encapsulates all calls to DOM apis, which allows the [UrlStrategy] classes
/// to be platform agnostic and testable.
///
/// For convenience, the [PlatformLocation] class can be used by implementations
/// of [UrlStrategy] to interact with DOM apis like pushState, popState, etc.
abstract interface class PlatformLocation {
  /// Registers an event listener for the `popstate` event.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/WindowEventHandlers/onpopstate
  void addPopStateListener(EventListener fn);

  /// Unregisters the given listener (added by [addPopStateListener]) from the
  /// `popstate` event.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/WindowEventHandlers/onpopstate
  void removePopStateListener(EventListener fn);

  /// The `pathname` part of the URL in the browser address bar.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/Location/pathname
  String get pathname;

  /// The `query` part of the URL in the browser address bar.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/Location/search
  String get search;

  /// The `hash]` part of the URL in the browser address bar.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/Location/hash
  String? get hash;

  /// The `state` in the current history entry.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/History/state
  Object? get state;

  /// Adds a new entry to the browser history stack.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/History/pushState
  void pushState(Object? state, String title, String url);

  /// Replaces the current entry in the browser history stack.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/History/replaceState
  void replaceState(Object? state, String title, String url);

  /// Moves forwards or backwards through the history stack.
  ///
  /// A negative [count] value causes a backward move in the history stack. And
  /// a positive [count] value causs a forward move.
  ///
  /// Examples:
  ///
  /// * `go(-2)` moves back 2 steps in history.
  /// * `go(3)` moves forward 3 steps in hisotry.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/History/go
  void go(int count);

  /// The base href where the Flutter app is being served.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/HTML/Element/base
  String? getBaseHref();
}

final Map<EventListener, DomEventListener> _popStateListenersCache =
    <EventListener, DomEventListener>{};

/// Delegates to real browser APIs to provide platform location functionality.
class BrowserPlatformLocation implements PlatformLocation {
  /// Default constructor for [BrowserPlatformLocation].
  const BrowserPlatformLocation();

  DomLocation get _location => domWindow.location;
  DomHistory get _history => domWindow.history;

  @visibleForTesting
  DomEventListener getOrCreateDomEventListener(EventListener fn) {
    return _popStateListenersCache.putIfAbsent(fn, () => createDomEventListener(fn));
  }

  @override
  void addPopStateListener(EventListener fn) {
    domWindow.addEventListener('popstate', getOrCreateDomEventListener(fn));
  }

  @override
  void removePopStateListener(EventListener fn) {
    assert(
      _popStateListenersCache.containsKey(fn),
      'Removing a listener that was never added or was removed already.',
    );
    domWindow.removeEventListener('popstate', getOrCreateDomEventListener(fn));
    _popStateListenersCache.remove(fn);
  }

  @override
  String get pathname => _location.pathname!;

  @override
  String get search => _location.search!;

  @override
  String get hash => _location.locationHash;

  @override
  Object? get state => _history.state;

  @override
  void pushState(Object? state, String title, String url) {
    _history.pushState(state, title, url);
  }

  @override
  void replaceState(Object? state, String title, String url) {
    _history.replaceState(state, title, url);
  }

  @override
  void go(int count) {
    _history.go(count);
  }

  @override
  String? getBaseHref() => domDocument.baseUri;
}
