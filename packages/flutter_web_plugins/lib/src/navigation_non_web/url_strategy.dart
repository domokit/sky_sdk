// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui' as ui;

import 'platform_location.dart';

export 'platform_location.dart';

/// Callback that receives the new state of the browser history entry.
typedef PopStateListener = void Function(Object? state);

/// Represents and reads route state from the browser's URL.
///
/// By default, the [HashUrlStrategy] subclass is used if the app doesn't
/// specify one.
abstract class UrlStrategy {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const UrlStrategy();

  /// Adds a listener to the `popstate` event and returns a function that, when
  /// invoked, removes the listener.
  ui.VoidCallback addPopStateListener(PopStateListener fn) {
    // No-op.
    return () {};
  }

  /// Returns the active path in the browser.
  String getPath() => '';

  /// The state of the current browser history entry.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/History/state
  Object? getState() => null;

  /// Given a path that's internal to the app, create the external url that
  /// will be used in the browser.
  String prepareExternalUrl(String internalUrl) => '';

  /// Push a new history entry.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/History/pushState
  void pushState(Object? state, String title, String url) {
    // No-op.
  }

  /// Replace the currently active history entry.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/History/replaceState
  void replaceState(Object? state, String title, String url) {
    // No-op.
  }

  /// Moves forwards or backwards through the history stack.
  ///
  /// A negative [count] value causes a backward move in the history stack. And
  /// a positive [count] value causes a forward move.
  ///
  /// Examples:
  ///
  /// * `go(-2)` moves back 2 steps in history.
  /// * `go(3)` moves forward 3 steps in history.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/API/History/go
  Future<void> go(int count) async {
    // No-op.
  }
}

/// Returns the present [UrlStrategy] for handling the browser URL.
///
/// In case null is returned, the browser integration has been manually
/// disabled by [setUrlStrategy].
UrlStrategy? get urlStrategy => null;

/// Change the strategy to use for handling browser URL.
///
/// Setting this to null disables all integration with the browser history.
void setUrlStrategy(UrlStrategy? strategy) {
  // No-op in non-web platforms.
}

/// Use the [PathUrlStrategy] to handle the browser URL.
void usePathUrlStrategy() {
  // No-op in non-web platforms.
}

/// Uses the browser URL's [hash fragments](https://en.wikipedia.org/wiki/Uniform_Resource_Locator#Syntax)
/// to represent its state.
///
/// By default, this class is used as the URL strategy for the app. However,
/// this class is still useful for apps that want to extend it.
///
/// In order to use [HashUrlStrategy] for an app, it needs to be set like this:
///
/// ```dart
/// import 'package:flutter_web_plugins/flutter_web_plugins.dart';
///
/// void main() {
///   // Somewhere before calling `runApp()` do:
///   setUrlStrategy(const HashUrlStrategy());
/// }
/// ```
class HashUrlStrategy extends UrlStrategy {
  /// Creates an instance of [HashUrlStrategy].
  ///
  /// The [PlatformLocation] parameter is useful for testing to mock out browser
  /// integrations.
  const HashUrlStrategy([PlatformLocation? _]);
}

/// Uses the browser URL's pathname to represent Flutter's route name.
///
/// In order to use [PathUrlStrategy] for an app, it needs to be set like this:
///
/// ```dart
/// import 'package:flutter_web_plugins/flutter_web_plugins.dart';
///
/// void main() {
///   // Somewhere before calling `runApp()` do:
///   setUrlStrategy(PathUrlStrategy());
/// }
/// ```
class PathUrlStrategy extends HashUrlStrategy {
  /// Creates an instance of [PathUrlStrategy].
  ///
  /// The [PlatformLocation] parameter is useful for testing to mock out browser
  /// integrations.
  const PathUrlStrategy([PlatformLocation? _, bool _ = false]);
}
