// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/src/engine.dart';

/// A function which takes a unique `id` and some `params` and creates an HTML
/// element.
typedef ParameterizedPlatformViewFactory = Object Function(int viewId, {Object? params});

/// A function which takes a unique `id` and creates an HTML element.
typedef PlatformViewFactory = Object Function(int viewId);

/// The platform view registry for this app.
final PlatformViewRegistry platformViewRegistry = PlatformViewRegistry();

/// A registry for factories that create platform views.
class PlatformViewRegistry {
  /// The view type of the built-in factory that creates visible platform view
  /// DOM elements.
  ///
  /// There's no need to register this view type with [PlatformViewRegistry]
  /// because it is registered by default.
  static const String defaultVisibleViewType = '_default_document_create_element_visible';

  /// The view type of the built-in factory that creates invisible platform view
  /// DOM elements.
  ///
  /// There's no need to register this view type with [PlatformViewRegistry]
  /// because it is registered by default.
  static const String defaultInvisibleViewType = '_default_document_create_element_invisible';

  /// Register [viewType] as being created by the given [viewFactory].
  ///
  /// [viewFactory] can be any function that takes an integer and optional
  /// `params` and returns an `HTMLElement` DOM object.
  bool registerViewFactory(String viewType, Function viewFactory, {bool isVisible = true}) {
    return PlatformViewManager.instance.registerFactory(
      viewType,
      viewFactory,
      isVisible: isVisible,
    );
  }

  /// Returns the view previously created for [viewId].
  ///
  /// Throws if no view has been created for [viewId].
  Object getViewById(int viewId) {
    return PlatformViewManager.instance.getViewById(viewId);
  }
}
