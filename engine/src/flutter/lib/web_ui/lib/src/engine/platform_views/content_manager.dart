// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../dom.dart';
import '../util.dart';
import 'slots.dart';

/// This class handles the lifecycle of Platform Views in the DOM of a Flutter Web App.
///
/// There are three important parts of Platform Views. This class manages two of
/// them:
///
/// * `factories`: The functions used to render the contents of any given Platform
/// View by its `viewType`.
/// * `contents`: The result [DomElement] of calling a `factory` function.
///
/// The third part is `slots`, which are created on demand by the
/// [createPlatformViewSlot] function.
///
/// This class keeps a registry of `factories`, `contents` so the framework can
/// CRUD Platform Views as needed, regardless of the rendering backend.
class PlatformViewManager {
  PlatformViewManager() {
    // Register some default factories.
    registerFactory(ui_web.PlatformViewRegistry.defaultVisibleViewType, _defaultFactory);
    registerFactory(
      ui_web.PlatformViewRegistry.defaultInvisibleViewType,
      _defaultFactory,
      isVisible: false,
    );
  }

  /// The shared instance of PlatformViewManager shared across the engine to handle
  /// rendering of PlatformViews into the web app.
  static PlatformViewManager instance = PlatformViewManager();

  // The factory functions, indexed by the viewType
  final Map<String, Function> _factories = <String, Function>{};

  // The references to content tags, indexed by their framework-given ID.
  final Map<int, DomElement> _contents = <int, DomElement>{};

  final Set<String> _invisibleViews = <String>{};
  final Map<int, String> _viewIdToType = <int, String>{};

  /// Returns `true` if the passed in `viewType` has been registered before.
  ///
  /// See [registerFactory] to understand how factories are registered.
  bool knowsViewType(String viewType) {
    return _factories.containsKey(viewType);
  }

  /// Returns `true` if the passed in `viewId` has been rendered (and not disposed) before.
  ///
  /// See [renderContent] and [createPlatformViewSlot] to understand how platform views are
  /// rendered.
  bool knowsViewId(int viewId) {
    return _contents.containsKey(viewId);
  }

  /// Returns the cached contents of [viewId], to be injected into the DOM.
  ///
  /// This is only used by the active `Renderer` object when a platform view needs
  /// to be injected in the DOM, through `FlutterView.DomManager.injectPlatformView`.
  ///
  /// This may return null, if [renderContent] was not called before this. The
  /// framework seems to allow/need this for some tests, so it is allowed here
  /// as well.
  ///
  /// App programmers should not access this directly, and instead use [getViewById].
  DomElement? getSlottedContent(int viewId) {
    return _contents[viewId];
  }

  /// Returns the HTML element created by a registered factory for [viewId].
  ///
  /// Throws an [AssertionError] if [viewId] hasn't been rendered before.
  DomElement getViewById(int viewId) {
    assert(knowsViewId(viewId), 'No view has been rendered for viewId: $viewId');
    // `_contents[viewId]` is the <flt-platform-view> element created by us. The
    // first (and only) child of that is the element created by the user-supplied
    // factory function.
    return _contents[viewId]!.firstElementChild!;
  }

  /// Registers a `factoryFunction` that knows how to render a Platform View of `viewType`.
  ///
  /// `viewType` is selected by the programmer, but it can't be overridden once
  /// it's been set.
  ///
  /// `factoryFunction` needs to be a [PlatformViewFactory].
  bool registerFactory(String viewType, Function factoryFunction, {bool isVisible = true}) {
    assert(
      factoryFunction is ui_web.PlatformViewFactory ||
          factoryFunction is ui_web.ParameterizedPlatformViewFactory,
      'Factory signature is invalid. Expected either '
      '{${ui_web.PlatformViewFactory}} or {${ui_web.ParameterizedPlatformViewFactory}} '
      'but got: {${factoryFunction.runtimeType}}',
    );

    if (_factories.containsKey(viewType)) {
      return false;
    }
    _factories[viewType] = factoryFunction;
    if (!isVisible) {
      _invisibleViews.add(viewType);
    }
    return true;
  }

  /// Creates the HTML markup for the `contents` of a Platform View.
  ///
  /// The result of this call is cached in the `_contents` Map, so the active
  /// renderer can inject it as needed.
  ///
  /// The resulting DOM for the `contents` of a Platform View looks like this:
  ///
  /// ```html
  /// <flt-platform-view id="flt-pv-VIEW_ID" slot="...">
  ///   <arbitrary-html-elements />
  /// </flt-platform-view-slot>
  /// ```
  ///
  /// The `arbitrary-html-elements` are the result of the call to the user-supplied
  /// `factory` function for this Platform View (see [registerFactory]).
  ///
  /// The outer `flt-platform-view` tag is a simple wrapper that we add to have
  /// a place where to attach the `slot` property, that will tell the browser
  /// what `slot` tag will reveal this `contents`, **without modifying the returned
  /// html from the `factory` function**.
  DomElement renderContent(String viewType, int viewId, Object? params) {
    assert(
      knowsViewType(viewType),
      'Attempted to render contents of unregistered viewType: $viewType',
    );

    final String slotName = getPlatformViewSlotName(viewId);
    _viewIdToType[viewId] = viewType;

    return _contents.putIfAbsent(viewId, () {
      final DomElement wrapper =
          domDocument.createElement('flt-platform-view')
            ..id = getPlatformViewDomId(viewId)
            ..setAttribute('slot', slotName);

      final Function factoryFunction = _factories[viewType]!;
      final DomElement content;

      if (factoryFunction is ui_web.ParameterizedPlatformViewFactory) {
        content = factoryFunction(viewId, params: params) as DomElement;
      } else {
        factoryFunction as ui_web.PlatformViewFactory;
        content = factoryFunction(viewId) as DomElement;
      }

      _ensureContentCorrectlySized(content, viewType);
      wrapper.append(content);

      return wrapper;
    });
  }

  /// Removes a PlatformView by its `viewId` from the manager, and from the DOM.
  ///
  /// Once a view has been cleared, calls to [knowsViewId] will fail, as if it had
  /// never been rendered before.
  void clearPlatformView(int viewId) {
    // Remove from our cache, and then from the DOM...
    _contents.remove(viewId)?.remove();
  }

  /// Attempt to ensure that the contents of the user-supplied DOM element will
  /// fill the space allocated for this platform view by the framework.
  void _ensureContentCorrectlySized(DomElement content, String viewType) {
    // Scrutinize closely any other modifications to `content`.
    // We shouldn't modify users' returned `content` if at all possible.
    // Note there's also no getContent(viewId) function anymore, to prevent
    // from later modifications too.
    if (content.style.height.isEmpty) {
      printWarning(
        'Height of Platform View type: [$viewType] may not be set.'
        ' Defaulting to `height: 100%`.\n'
        'Set `style.height` to any appropriate value to stop this message.',
      );

      content.style.height = '100%';
    }

    if (content.style.width.isEmpty) {
      printWarning(
        'Width of Platform View type: [$viewType] may not be set.'
        ' Defaulting to `width: 100%`.\n'
        'Set `style.width` to any appropriate value to stop this message.',
      );

      content.style.width = '100%';
    }
  }

  /// Returns `true` if the given [viewId] is for an invisible platform view.
  bool isInvisible(int viewId) {
    final String? viewType = _viewIdToType[viewId];
    return viewType != null && _invisibleViews.contains(viewType);
  }

  /// Returns `true` if the given [viewId] is a platform view with a visible
  /// component.
  bool isVisible(int viewId) => !isInvisible(viewId);

  /// Clears the state. Used in tests.
  void debugClear() {
    _contents.keys.toList().forEach(clearPlatformView);
    _factories.clear();
    _contents.clear();
    _invisibleViews.clear();
    _viewIdToType.clear();
  }
}

DomElement _defaultFactory(int viewId, {Object? params}) {
  params!;
  params as Map<Object?, Object?>;
  return domDocument.createElement(params.readString('tagName'))
    ..style.width = '100%'
    ..style.height = '100%';
}
