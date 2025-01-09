// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
import 'dart:js_interop';
import 'package:ui/src/engine.dart';

/// Exposes web-only functionality for this app's `FlutterView`s objects.
final FlutterViewManagerProxy views = FlutterViewManagerProxy(
  viewManager: EnginePlatformDispatcher.instance.viewManager,
);

/// This class exposes web-only attributes for the registered views in a multi-view app.
class FlutterViewManagerProxy {
  FlutterViewManagerProxy({required FlutterViewManager viewManager}) : _viewManager = viewManager;
  // The proxied viewManager instance.
  final FlutterViewManager _viewManager;

  /// Returns the host element for [viewId].
  ///
  /// In the full-page mode, the host element is the `<body>` element of the page
  /// and the view is the one and only [PlatformDispatcher.implicitView].
  ///
  /// In the add-to-app mode, the host element is the value of `hostElement`
  /// provided when creating the view.
  ///
  /// This is useful for plugins and apps to have a safe DOM Element where they
  /// can add their own custom HTML elements (for example: file inputs for the
  /// file_selector plugin).
  JSAny? getHostElement(int viewId) {
    return _viewManager.getHostElement(viewId) as JSAny?;
  }

  /// Returns the `initialData` configuration value passed from JS when `viewId` was added.
  ///
  /// Developers can access the initial data from Dart in two ways:
  ///  * Defining their own `staticInterop` class that describes what you're
  ///    passing to your views, to retain type safety on Dart (preferred).
  ///  * Calling [NullableUndefineableJSAnyExtension.dartify] and accessing the
  ///    returned object as if it were a [Map] (not recommended).
  JSAny? getInitialData(int viewId) {
    return _viewManager.getOptions(viewId)?.initialData;
  }
}
