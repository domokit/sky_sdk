// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui_web' as ui_web;

import 'utils.dart';

export 'dart:ui_web'
    show
        BrowserPlatformLocation,
        EventListener,
        HashUrlStrategy,
        PlatformLocation,
        UrlStrategy,
        urlStrategy;

/// Change the strategy to use for handling browser URL.
///
/// Setting this to null disables all integration with the browser history.
void setUrlStrategy(ui_web.UrlStrategy? strategy) {
  ui_web.urlStrategy = strategy;
}

/// Use the [PathUrlStrategy] to handle the browser URL.
void usePathUrlStrategy() {
  setUrlStrategy(PathUrlStrategy());
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
class PathUrlStrategy extends ui_web.HashUrlStrategy {
  /// Creates an instance of [PathUrlStrategy].
  ///
  /// The [ui_web.PlatformLocation] parameter is useful for testing to mock out browser
  /// interactions.
  PathUrlStrategy([super.platformLocation, this.includeHash = false])
    : _platformLocation = platformLocation,
      _basePath = stripTrailingSlash(
        extractPathname(checkBaseHref(platformLocation.getBaseHref())),
      );

  final ui_web.PlatformLocation _platformLocation;
  final String _basePath;

  /// There were an issue with url #hash which disappears from URL on first start of the web application
  /// This flag allows to preserve that hash and was introduced mainly to preserve backward compatibility
  /// with existing applications that rely on a full match on the path. If someone navigates to
  /// /profile or /profile#foo, they both will work without this flag otherwise /profile#foo won't match
  /// with the /profile route name anymore because the hash became part of the path.
  ///
  /// This flag solves the edge cases when using auth provider which redirects back to the app with
  /// token in redirect URL as /#access_token=bla_bla_bla
  final bool includeHash;

  @override
  String getPath() {
    final String? hash = includeHash ? _platformLocation.hash : null;
    final String path = _platformLocation.pathname + _platformLocation.search + (hash ?? '');
    if (_basePath.isNotEmpty && path.startsWith(_basePath)) {
      return ensureLeadingSlash(path.substring(_basePath.length));
    }
    return ensureLeadingSlash(path);
  }

  @override
  String prepareExternalUrl(String internalUrl) {
    if (internalUrl.isEmpty) {
      internalUrl = '/';
    }
    assert(
      internalUrl.startsWith('/'),
      "When using PathUrlStrategy, all route names must start with '/' because "
      "the browser's pathname always starts with '/'. "
      "Found route name: '$internalUrl'",
    );
    return '$_basePath$internalUrl';
  }
}
