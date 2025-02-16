// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// The widget in this file is an empty mock for non-web platforms. See
// `_platform_selectable_region_context_menu_web.dart` for the web
// implementation.

import 'framework.dart';
import 'selection_container.dart';

/// Function signature for `ui_web.platformViewRegistry.registerViewFactory`.
@visibleForTesting
typedef RegisterViewFactory = void Function(String, Object Function(int viewId), {bool isVisible});

/// A widget that provides native selection context menu for its child subtree.
///
/// This widget currently only supports Flutter web. Using this widget in non-web
/// platforms will throw [UnimplementedError]s.
///
/// In web platform, this widget registers a singleton platform view, i.e. a
/// HTML DOM element. The created platform view will be shared between all
/// [PlatformSelectableRegionContextMenu]s.
///
/// Only one [SelectionContainerDelegate] can attach to the
/// [PlatformSelectableRegionContextMenu] at a time. Use [attach] method to make
/// a [SelectionContainerDelegate] to be the active client.
class PlatformSelectableRegionContextMenu extends StatelessWidget {
  /// Creates a [PlatformSelectableRegionContextMenu]
  // ignore: prefer_const_constructors_in_immutables
  PlatformSelectableRegionContextMenu({
    // ignore: avoid_unused_constructor_parameters
    required Widget child,
    super.key,
  });

  /// Attaches the `client` to be able to open platform-appropriate context menus.
  static void attach(SelectionContainerDelegate client) => throw UnimplementedError();

  /// Detaches the `client` from the platform-appropriate selection context menus.
  static void detach(SelectionContainerDelegate client) => throw UnimplementedError();

  /// Override this to provide a custom implementation of `ui_web.platformViewRegistry.registerViewFactory`.
  ///
  /// This should only be used for testing.
  @visibleForTesting
  static RegisterViewFactory? debugOverrideRegisterViewFactory;

  /// Resets the view factory registration to its initial state.
  @visibleForTesting
  static void debugResetRegistry() {
    throw UnimplementedError();
  }

  @override
  Widget build(BuildContext context) => throw UnimplementedError();
}
