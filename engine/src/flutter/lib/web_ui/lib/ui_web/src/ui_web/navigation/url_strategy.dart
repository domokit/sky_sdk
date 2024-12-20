// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import '../testing.dart';
import 'platform_location.dart';

UrlStrategy _realDefaultUrlStrategy =
    debugEmulateFlutterTesterEnvironment
        ? TestUrlStrategy.fromEntry(const TestHistoryEntry('default', null, '/'))
        : const HashUrlStrategy();

UrlStrategy get _defaultUrlStrategy => debugDefaultUrlStrategyOverride ?? _realDefaultUrlStrategy;

/// Overrides the default URL strategy.
///
/// Setting this to null allows the real default URL strategy to be used.
///
/// This is intended to be used for testing and debugging only.
@visibleForTesting
UrlStrategy? debugDefaultUrlStrategyOverride;

/// Whether a custom URL strategy has been set or not.
//
// It is valid to set [_customUrlStrategy] to null, so we can't use a null
// check to determine whether it was set or not. We need an extra boolean.
bool isCustomUrlStrategySet = false;

/// Whether a custom URL strategy can be set or not.
///
/// This is used to prevent setting a custom URL strategy for a second time or
/// after the app has already started running.
bool _customUrlStrategyCanBeSet = true;

/// A custom URL strategy set by the app before running.
UrlStrategy? _customUrlStrategy;

/// Returns the present [UrlStrategy] for handling the browser URL.
///
/// Returns null when the browser integration has been manually disabled.
UrlStrategy? get urlStrategy => isCustomUrlStrategySet ? _customUrlStrategy : _defaultUrlStrategy;

/// Sets a custom URL strategy instead of the default one.
///
/// Passing null disables browser history integration altogether.
///
/// This setter can only be called once. Subsequent calls will throw an error
/// in debug mode.
set urlStrategy(UrlStrategy? strategy) {
  assert(
    _customUrlStrategyCanBeSet,
    'Cannot set URL strategy a second time or after the app has been initialized.',
  );
  preventCustomUrlStrategy();
  isCustomUrlStrategySet = true;
  _customUrlStrategy = strategy;
}

/// From this point on, prevents setting a custom URL strategy.
void preventCustomUrlStrategy() {
  _customUrlStrategyCanBeSet = false;
}

/// Resets everything to do with custom URL strategy.
///
/// This should only be used in tests to reset things back after each test.
void debugResetCustomUrlStrategy() {
  isCustomUrlStrategySet = false;
  _customUrlStrategyCanBeSet = true;
  _customUrlStrategy = null;
}

/// Callback that receives the new state of the browser history entry.
typedef PopStateListener = void Function(Object? state);

/// Represents and reads route state from the browser's URL.
///
/// By default, the [HashUrlStrategy] subclass is used if the app doesn't
/// specify one.
abstract class UrlStrategy {
  /// Adds a listener to the `popstate` event and returns a function that, when
  /// invoked, removes the listener.
  ui.VoidCallback addPopStateListener(PopStateListener fn);

  /// Returns the active path in the browser.
  String getPath();

  /// The state of the current browser history entry.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/History/state
  Object? getState();

  /// Given a path that's internal to the app, create the external url that
  /// will be used in the browser.
  String prepareExternalUrl(String internalUrl);

  /// Push a new history entry.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/History/pushState
  void pushState(Object? state, String title, String url);

  /// Replace the currently active history entry.
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
  Future<void> go(int count);
}

/// This is an implementation of [UrlStrategy] that uses the browser URL's
/// [hash fragments](https://en.wikipedia.org/wiki/Uniform_Resource_Locator#Syntax)
/// to represent its state.
///
/// In order to use this [UrlStrategy] for an app, it needs to be set like this:
///
/// ```dart
/// import 'package:flutter_web_plugins/flutter_web_plugins.dart';
///
/// // Somewhere before calling `runApp()` do:
/// setUrlStrategy(const HashUrlStrategy());
/// ```
class HashUrlStrategy implements UrlStrategy {
  /// Creates an instance of [HashUrlStrategy].
  ///
  /// The [PlatformLocation] parameter is useful for testing to mock out browser
  /// interactions.
  const HashUrlStrategy([this._platformLocation = const BrowserPlatformLocation()]);

  final PlatformLocation _platformLocation;

  @override
  ui.VoidCallback addPopStateListener(PopStateListener fn) {
    void wrappedFn(Object event) {
      // `fn` expects `event.state`, not a `DomEvent`.
      fn((event as DomPopStateEvent).state);
    }

    _platformLocation.addPopStateListener(wrappedFn);
    return () => _platformLocation.removePopStateListener(wrappedFn);
  }

  @override
  String getPath() {
    // the hash value is always prefixed with a `#`
    // and if it is empty then it will stay empty
    final String path = _platformLocation.hash ?? '';
    assert(path.isEmpty || path.startsWith('#'));

    // We don't want to return an empty string as a path. Instead we default to "/".
    if (path.isEmpty || path == '#') {
      return '/';
    }
    // At this point, we know [path] starts with "#" and isn't empty.
    return path.substring(1);
  }

  @override
  Object? getState() => _platformLocation.state;

  @override
  String prepareExternalUrl(String internalUrl) {
    // It's convention that if the hash path is empty, we omit the `#`; however,
    // if the empty URL is pushed it won't replace any existing fragment. So
    // when the hash path is empty, we still return the location's path and
    // query.
    final String hash;
    if (internalUrl.isEmpty || internalUrl == '/') {
      // Let's not add the hash at all when the app is in the home page. That
      // way, the URL of the home page is cleaner.
      //
      // See: https://github.com/flutter/flutter/issues/127608
      hash = '';
    } else {
      hash = '#$internalUrl';
    }
    return '${_platformLocation.pathname}${_platformLocation.search}$hash';
  }

  @override
  void pushState(Object? state, String title, String url) {
    _platformLocation.pushState(state, title, prepareExternalUrl(url));
  }

  @override
  void replaceState(Object? state, String title, String url) {
    _platformLocation.replaceState(state, title, prepareExternalUrl(url));
  }

  @override
  Future<void> go(int count) {
    _platformLocation.go(count);
    return _waitForPopState();
  }

  /// Waits until the next popstate event is fired.
  ///
  /// This is useful, for example, to wait until the browser has handled the
  /// `history.back` transition.
  Future<void> _waitForPopState() {
    final Completer<void> completer = Completer<void>();
    late ui.VoidCallback unsubscribe;
    unsubscribe = addPopStateListener((_) {
      unsubscribe();
      completer.complete();
    });
    return completer.future;
  }
}
