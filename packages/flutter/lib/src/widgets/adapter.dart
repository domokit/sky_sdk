// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'binding.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

import 'framework.dart';

/// A bridge from a [RenderObject] to an [Element] tree.
///
/// The given container is the [RenderObject] that the [Element] tree should be
/// inserted into. It must be a [RenderObject] that implements the
/// [RenderObjectWithChildMixin] protocol. The type argument `T` is the kind of
/// [RenderObject] that the container expects as its child.
///
/// The [RenderObjectToWidgetAdapter] is an alternative to [RootWidget] for
/// bootstrapping an element tree. Unlike [RootWidget] it requires the
/// existence of a render tree (the [container]) to attach the element tree to.
class RenderObjectToWidgetAdapter<T extends RenderObject> extends RenderObjectWidget {
  /// Creates a bridge from a [RenderObject] to an [Element] tree.
  RenderObjectToWidgetAdapter({this.child, required this.container, this.debugShortDescription})
    : super(key: GlobalObjectKey(container));

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// The [RenderObject] that is the parent of the [Element] created by this widget.
  final RenderObjectWithChildMixin<T> container;

  /// A short description of this widget used by debugging aids.
  final String? debugShortDescription;

  @override
  RenderObjectToWidgetElement<T> createElement() => RenderObjectToWidgetElement<T>(this);

  @override
  RenderObjectWithChildMixin<T> createRenderObject(BuildContext context) => container;

  @override
  void updateRenderObject(BuildContext context, RenderObject renderObject) {}

  /// Inflate this widget and actually set the resulting [RenderObject] as the
  /// child of [container].
  ///
  /// If `element` is null, this function will create a new element. Otherwise,
  /// the given element will have an update scheduled to switch to this widget.
  RenderObjectToWidgetElement<T> attachToRenderTree(
    BuildOwner owner, [
    RenderObjectToWidgetElement<T>? element,
  ]) {
    if (element == null) {
      owner.lockState(() {
        element = createElement();
        assert(element != null);
        element!.assignOwner(owner);
      });
      owner.buildScope(element!, () {
        element!.mount(null, null);
      });
    } else {
      element._newWidget = this;
      element.markNeedsBuild();
    }
    return element!;
  }

  @override
  String toStringShort() => debugShortDescription ?? super.toStringShort();
}

/// The root of an element tree that is hosted by a [RenderObject].
///
/// This element class is the instantiation of a [RenderObjectToWidgetAdapter]
/// widget. It can be used only as the root of an [Element] tree (it cannot be
/// mounted into another [Element]; it's parent must be null).
///
/// In typical usage, it will be instantiated for a [RenderObjectToWidgetAdapter]
/// whose container is the [RenderView].
class RenderObjectToWidgetElement<T extends RenderObject> extends RenderTreeRootElement
    with RootElementMixin {
  /// Creates an element that is hosted by a [RenderObject].
  ///
  /// The [RenderObject] created by this element is not automatically set as a
  /// child of the hosting [RenderObject]. To actually attach this element to
  /// the render tree, call [RenderObjectToWidgetAdapter.attachToRenderTree].
  RenderObjectToWidgetElement(RenderObjectToWidgetAdapter<T> super.widget);

  Element? _child;

  static const Object _rootChildSlot = Object();

  @override
  void visitChildren(ElementVisitor visitor) {
    if (_child != null) {
      visitor(_child!);
    }
  }

  @override
  void forgetChild(Element child) {
    assert(child == _child);
    _child = null;
    super.forgetChild(child);
  }

  @override
  void mount(Element? parent, Object? newSlot) {
    assert(parent == null);
    super.mount(parent, newSlot);
    _rebuild();
    assert(_child != null);
  }

  @override
  void update(RenderObjectToWidgetAdapter<T> newWidget) {
    super.update(newWidget);
    assert(widget == newWidget);
    _rebuild();
  }

  // When we are assigned a new widget, we store it here
  // until we are ready to update to it.
  Widget? _newWidget;

  @override
  void performRebuild() {
    if (_newWidget != null) {
      // _newWidget can be null if, for instance, we were rebuilt
      // due to a reassemble.
      final Widget newWidget = _newWidget!;
      _newWidget = null;
      update(newWidget as RenderObjectToWidgetAdapter<T>);
    }
    super.performRebuild();
    assert(_newWidget == null);
  }

  @pragma('vm:notify-debugger-on-exception')
  void _rebuild() {
    try {
      _child = updateChild(
        _child,
        (widget as RenderObjectToWidgetAdapter<T>).child,
        _rootChildSlot,
      );
    } catch (exception, stack) {
      final FlutterErrorDetails details = FlutterErrorDetails(
        exception: exception,
        stack: stack,
        library: 'widgets library',
        context: ErrorDescription('attaching to the render tree'),
      );
      FlutterError.reportError(details);
      final Widget error = ErrorWidget.builder(details);
      _child = updateChild(null, error, _rootChildSlot);
    }
  }

  @override
  RenderObjectWithChildMixin<T> get renderObject =>
      super.renderObject as RenderObjectWithChildMixin<T>;

  @override
  void insertRenderObjectChild(RenderObject child, Object? slot) {
    assert(slot == _rootChildSlot);
    assert(renderObject.debugValidateChild(child));
    renderObject.child = child as T;
  }

  @override
  void moveRenderObjectChild(RenderObject child, Object? oldSlot, Object? newSlot) {
    assert(false);
  }

  @override
  void removeRenderObjectChild(RenderObject child, Object? slot) {
    assert(renderObject.child == child);
    renderObject.child = null;
  }
}
