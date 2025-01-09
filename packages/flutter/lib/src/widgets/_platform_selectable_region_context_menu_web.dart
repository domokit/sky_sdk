// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';
import 'dart:ui_web' as ui_web;

import 'package:flutter/rendering.dart';

import '../web.dart' as web;
import 'basic.dart';
import 'framework.dart';
import 'platform_view.dart';
import 'selection_container.dart';

const String _viewType = 'Browser__WebContextMenuViewType__';
const String _kClassName = 'web-selectable-region-context-menu';
// These css rules hides the dom element with the class name.
const String _kClassSelectionRule = '.$_kClassName::selection { background: transparent; }';
const String _kClassRule = '''
.$_kClassName {
  color: transparent;
  user-select: text;
  -webkit-user-select: text; /* Safari */
  -moz-user-select: text; /* Firefox */
  -ms-user-select: text; /* IE10+ */
}
''';
const int _kRightClickButton = 2;

typedef _WebSelectionCallBack = void Function(web.HTMLElement, web.MouseEvent);

/// Function signature for `ui_web.platformViewRegistry.registerViewFactory`.
@visibleForTesting
typedef RegisterViewFactory = void Function(String, Object Function(int viewId), {bool isVisible});

/// See `_platform_selectable_region_context_menu_io.dart` for full
/// documentation.
class PlatformSelectableRegionContextMenu extends StatelessWidget {
  /// See `_platform_selectable_region_context_menu_io.dart`.
  PlatformSelectableRegionContextMenu({required this.child, super.key}) {
    if (_registeredViewType == null) {
      _register();
    }
  }

  /// See `_platform_selectable_region_context_menu_io.dart`.
  final Widget child;

  /// See `_platform_selectable_region_context_menu_io.dart`.
  // ignore: use_setters_to_change_properties
  static void attach(SelectionContainerDelegate client) {
    _activeClient = client;
  }

  /// See `_platform_selectable_region_context_menu_io.dart`.
  static void detach(SelectionContainerDelegate client) {
    if (_activeClient != client) {
      _activeClient = null;
    }
  }

  static SelectionContainerDelegate? _activeClient;

  // Keeps track if this widget has already registered its view factories or not.
  static String? _registeredViewType;

  static RegisterViewFactory get _registerViewFactory =>
      debugOverrideRegisterViewFactory ?? ui_web.platformViewRegistry.registerViewFactory;

  /// Override this to provide a custom implementation of [ui_web.platformViewRegistry.registerViewFactory].
  ///
  /// This should only be used for testing.
  // See `_platform_selectable_region_context_menu_io.dart`.
  @visibleForTesting
  static RegisterViewFactory? debugOverrideRegisterViewFactory;

  // Registers the view factories for the interceptor widgets.
  static void _register() {
    assert(_registeredViewType == null);
    _registeredViewType = _registerWebSelectionCallback((
      web.HTMLElement element,
      web.MouseEvent event,
    ) {
      final SelectionContainerDelegate? client = _activeClient;
      if (client != null) {
        // Converts the html right click event to flutter coordinate.
        final Offset localOffset = Offset(event.offsetX.toDouble(), event.offsetY.toDouble());
        final Matrix4 transform = client.getTransformTo(null);
        final Offset globalOffset = MatrixUtils.transformPoint(transform, localOffset);
        client.dispatchSelectionEvent(SelectWordSelectionEvent(globalPosition: globalOffset));
        // The innerText must contain the text in order to be selected by
        // the browser.
        element.innerText = client.getSelectedContent()?.plainText ?? '';

        // Programmatically select the dom element in browser.
        final web.Range range = web.document.createRange()..selectNode(element);

        web.window.getSelection()
          ?..removeAllRanges()
          ..addRange(range);
      }
    });
  }

  static String _registerWebSelectionCallback(_WebSelectionCallBack callback) {
    _registerViewFactory(_viewType, (int viewId) {
      final web.HTMLElement htmlElement = web.document.createElement('div') as web.HTMLElement;
      htmlElement
        ..style.width = '100%'
        ..style.height = '100%'
        ..classList.add(_kClassName);

      // Create css style for _kClassName.
      final web.HTMLStyleElement styleElement =
          web.document.createElement('style') as web.HTMLStyleElement;
      web.document.head!.append(styleElement as JSAny);
      final web.CSSStyleSheet sheet = styleElement.sheet!;
      sheet.insertRule(_kClassRule, 0);
      sheet.insertRule(_kClassSelectionRule, 1);

      htmlElement.addEventListener(
        'mousedown',
        (web.Event event) {
          final web.MouseEvent mouseEvent = event as web.MouseEvent;
          if (mouseEvent.button != _kRightClickButton) {
            return;
          }
          callback(htmlElement, mouseEvent);
        }.toJS,
      );
      return htmlElement;
    }, isVisible: false);
    return _viewType;
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: <Widget>[const Positioned.fill(child: HtmlElementView(viewType: _viewType)), child],
    );
  }
}
