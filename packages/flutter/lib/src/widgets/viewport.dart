// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'page_view.dart';
/// @docImport 'scroll_position.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'scrollable.dart';
/// @docImport 'sliver.dart';
library;

import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'debug.dart';
import 'framework.dart';
import 'scroll_notification.dart';

export 'package:flutter/rendering.dart' show AxisDirection, GrowthDirection;

/// A widget through which a portion of larger content can be viewed, typically
/// in combination with a [Scrollable].
///
/// [Viewport] is the visual workhorse of the scrolling machinery. It displays a
/// subset of its children according to its own dimensions and the given
/// [offset]. As the offset varies, different children are visible through
/// the viewport.
///
/// [Viewport] hosts a bidirectional list of slivers, anchored on a [center]
/// sliver, which is placed at the zero scroll offset. The center widget is
/// displayed in the viewport according to the [anchor] property.
///
/// Slivers that are earlier in the child list than [center] are displayed in
/// reverse order in the reverse [axisDirection] starting from the [center]. For
/// example, if the [axisDirection] is [AxisDirection.down], the first sliver
/// before [center] is placed above the [center]. The slivers that are later in
/// the child list than [center] are placed in order in the [axisDirection]. For
/// example, in the preceding scenario, the first sliver after [center] is
/// placed below the [center].
///
/// [Viewport] cannot contain box children directly. Instead, use a
/// [SliverList], [SliverFixedExtentList], [SliverGrid], or a
/// [SliverToBoxAdapter], for example.
///
/// See also:
///
///  * [ListView], [PageView], [GridView], and [CustomScrollView], which combine
///    [Scrollable] and [Viewport] into widgets that are easier to use.
///  * [SliverToBoxAdapter], which allows a box widget to be placed inside a
///    sliver context (the opposite of this widget).
///  * [ShrinkWrappingViewport], a variant of [Viewport] that shrink-wraps its
///    contents along the main axis.
///  * [ViewportElementMixin], which should be mixed in to the [Element] type used
///    by viewport-like widgets to correctly handle scroll notifications.
class Viewport extends MultiChildRenderObjectWidget {
  /// Creates a widget that is bigger on the inside.
  ///
  /// The viewport listens to the [offset], which means you do not need to
  /// rebuild this widget when the [offset] changes.
  ///
  /// The [cacheExtent] must be specified if the [cacheExtentStyle] is
  /// not [CacheExtentStyle.pixel].
  Viewport({
    super.key,
    this.axisDirection = AxisDirection.down,
    this.crossAxisDirection,
    this.anchor = 0.0,
    required this.offset,
    this.center,
    this.cacheExtent,
    this.cacheExtentStyle = CacheExtentStyle.pixel,
    this.clipBehavior = Clip.hardEdge,
    List<Widget> slivers = const <Widget>[],
  }) : assert(center == null || slivers.where((Widget child) => child.key == center).length == 1),
       assert(cacheExtentStyle != CacheExtentStyle.viewport || cacheExtent != null),
       super(children: slivers);

  /// The direction in which the [offset]'s [ViewportOffset.pixels] increases.
  ///
  /// For example, if the [axisDirection] is [AxisDirection.down], a scroll
  /// offset of zero is at the top of the viewport and increases towards the
  /// bottom of the viewport.
  final AxisDirection axisDirection;

  /// The direction in which child should be laid out in the cross axis.
  ///
  /// If the [axisDirection] is [AxisDirection.down] or [AxisDirection.up], this
  /// property defaults to [AxisDirection.left] if the ambient [Directionality]
  /// is [TextDirection.rtl] and [AxisDirection.right] if the ambient
  /// [Directionality] is [TextDirection.ltr].
  ///
  /// If the [axisDirection] is [AxisDirection.left] or [AxisDirection.right],
  /// this property defaults to [AxisDirection.down].
  final AxisDirection? crossAxisDirection;

  /// The relative position of the zero scroll offset.
  ///
  /// For example, if [anchor] is 0.5 and the [axisDirection] is
  /// [AxisDirection.down] or [AxisDirection.up], then the zero scroll offset is
  /// vertically centered within the viewport. If the [anchor] is 1.0, and the
  /// [axisDirection] is [AxisDirection.right], then the zero scroll offset is
  /// on the left edge of the viewport.
  ///
  /// {@macro flutter.rendering.GrowthDirection.sample}
  final double anchor;

  /// Which part of the content inside the viewport should be visible.
  ///
  /// The [ViewportOffset.pixels] value determines the scroll offset that the
  /// viewport uses to select which part of its content to display. As the user
  /// scrolls the viewport, this value changes, which changes the content that
  /// is displayed.
  ///
  /// Typically a [ScrollPosition].
  final ViewportOffset offset;

  /// The first child in the [GrowthDirection.forward] growth direction.
  ///
  /// Children after [center] will be placed in the [axisDirection] relative to
  /// the [center]. Children before [center] will be placed in the opposite of
  /// the [axisDirection] relative to the [center].
  ///
  /// The [center] must be the key of a child of the viewport.
  ///
  /// {@macro flutter.rendering.GrowthDirection.sample}
  final Key? center;

  /// {@macro flutter.rendering.RenderViewportBase.cacheExtent}
  ///
  /// See also:
  ///
  ///  * [cacheExtentStyle], which controls the units of the [cacheExtent].
  final double? cacheExtent;

  /// {@macro flutter.rendering.RenderViewportBase.cacheExtentStyle}
  final CacheExtentStyle cacheExtentStyle;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// Given a [BuildContext] and an [AxisDirection], determine the correct cross
  /// axis direction.
  ///
  /// This depends on the [Directionality] if the `axisDirection` is vertical;
  /// otherwise, the default cross axis direction is downwards.
  static AxisDirection getDefaultCrossAxisDirection(
    BuildContext context,
    AxisDirection axisDirection,
  ) {
    switch (axisDirection) {
      case AxisDirection.up:
        assert(
          debugCheckHasDirectionality(
            context,
            why:
                "to determine the cross-axis direction when the viewport has an 'up' axisDirection",
            alternative:
                "Alternatively, consider specifying the 'crossAxisDirection' argument on the Viewport.",
          ),
        );
        return textDirectionToAxisDirection(Directionality.of(context));
      case AxisDirection.right:
        return AxisDirection.down;
      case AxisDirection.down:
        assert(
          debugCheckHasDirectionality(
            context,
            why:
                "to determine the cross-axis direction when the viewport has a 'down' axisDirection",
            alternative:
                "Alternatively, consider specifying the 'crossAxisDirection' argument on the Viewport.",
          ),
        );
        return textDirectionToAxisDirection(Directionality.of(context));
      case AxisDirection.left:
        return AxisDirection.down;
    }
  }

  @override
  RenderViewport createRenderObject(BuildContext context) {
    return RenderViewport(
      axisDirection: axisDirection,
      crossAxisDirection:
          crossAxisDirection ?? Viewport.getDefaultCrossAxisDirection(context, axisDirection),
      anchor: anchor,
      offset: offset,
      cacheExtent: cacheExtent,
      cacheExtentStyle: cacheExtentStyle,
      clipBehavior: clipBehavior,
    );
  }

  @override
  void updateRenderObject(BuildContext context, RenderViewport renderObject) {
    renderObject
      ..axisDirection = axisDirection
      ..crossAxisDirection =
          crossAxisDirection ?? Viewport.getDefaultCrossAxisDirection(context, axisDirection)
      ..anchor = anchor
      ..offset = offset
      ..cacheExtent = cacheExtent
      ..cacheExtentStyle = cacheExtentStyle
      ..clipBehavior = clipBehavior;
  }

  @override
  MultiChildRenderObjectElement createElement() => _ViewportElement(this);

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<AxisDirection>('axisDirection', axisDirection));
    properties.add(
      EnumProperty<AxisDirection>('crossAxisDirection', crossAxisDirection, defaultValue: null),
    );
    properties.add(DoubleProperty('anchor', anchor));
    properties.add(DiagnosticsProperty<ViewportOffset>('offset', offset));
    if (center != null) {
      properties.add(DiagnosticsProperty<Key>('center', center));
    } else if (children.isNotEmpty && children.first.key != null) {
      properties.add(DiagnosticsProperty<Key>('center', children.first.key, tooltip: 'implicit'));
    }
    properties.add(DiagnosticsProperty<double>('cacheExtent', cacheExtent));
    properties.add(DiagnosticsProperty<CacheExtentStyle>('cacheExtentStyle', cacheExtentStyle));
  }
}

class _ViewportElement extends MultiChildRenderObjectElement
    with NotifiableElementMixin, ViewportElementMixin {
  /// Creates an element that uses the given widget as its configuration.
  _ViewportElement(Viewport super.widget);

  bool _doingMountOrUpdate = false;
  int? _centerSlotIndex;

  @override
  RenderViewport get renderObject => super.renderObject as RenderViewport;

  @override
  void mount(Element? parent, Object? newSlot) {
    assert(!_doingMountOrUpdate);
    _doingMountOrUpdate = true;
    super.mount(parent, newSlot);
    _updateCenter();
    assert(_doingMountOrUpdate);
    _doingMountOrUpdate = false;
  }

  @override
  void update(MultiChildRenderObjectWidget newWidget) {
    assert(!_doingMountOrUpdate);
    _doingMountOrUpdate = true;
    super.update(newWidget);
    _updateCenter();
    assert(_doingMountOrUpdate);
    _doingMountOrUpdate = false;
  }

  void _updateCenter() {
    // TODO(ianh): cache the keys to make this faster
    final Viewport viewport = widget as Viewport;
    if (viewport.center != null) {
      int elementIndex = 0;
      for (final Element e in children) {
        if (e.widget.key == viewport.center) {
          renderObject.center = e.renderObject as RenderSliver?;
          break;
        }
        elementIndex++;
      }
      assert(elementIndex < children.length);
      _centerSlotIndex = elementIndex;
    } else if (children.isNotEmpty) {
      renderObject.center = children.first.renderObject as RenderSliver?;
      _centerSlotIndex = 0;
    } else {
      renderObject.center = null;
      _centerSlotIndex = null;
    }
  }

  @override
  void insertRenderObjectChild(RenderObject child, IndexedSlot<Element?> slot) {
    super.insertRenderObjectChild(child, slot);
    // Once [mount]/[update] are done, the `renderObject.center` will be updated
    // in [_updateCenter].
    if (!_doingMountOrUpdate && slot.index == _centerSlotIndex) {
      renderObject.center = child as RenderSliver?;
    }
  }

  @override
  void moveRenderObjectChild(
    RenderObject child,
    IndexedSlot<Element?> oldSlot,
    IndexedSlot<Element?> newSlot,
  ) {
    super.moveRenderObjectChild(child, oldSlot, newSlot);
    assert(_doingMountOrUpdate);
  }

  @override
  void removeRenderObjectChild(RenderObject child, Object? slot) {
    super.removeRenderObjectChild(child, slot);
    if (!_doingMountOrUpdate && renderObject.center == child) {
      renderObject.center = null;
    }
  }

  @override
  void debugVisitOnstageChildren(ElementVisitor visitor) {
    children
        .where((Element e) {
          final RenderSliver renderSliver = e.renderObject! as RenderSliver;
          return renderSliver.geometry!.visible;
        })
        .forEach(visitor);
  }
}

/// A widget that is bigger on the inside and shrink wraps its children in the
/// main axis.
///
/// [ShrinkWrappingViewport] displays a subset of its children according to its
/// own dimensions and the given [offset]. As the offset varies, different
/// children are visible through the viewport.
///
/// [ShrinkWrappingViewport] differs from [Viewport] in that [Viewport] expands
/// to fill the main axis whereas [ShrinkWrappingViewport] sizes itself to match
/// its children in the main axis. This shrink wrapping behavior is expensive
/// because the children, and hence the viewport, could potentially change size
/// whenever the [offset] changes (e.g., because of a collapsing header).
///
/// [ShrinkWrappingViewport] cannot contain box children directly. Instead, use
/// a [SliverList], [SliverFixedExtentList], [SliverGrid], or a
/// [SliverToBoxAdapter], for example.
///
/// See also:
///
///  * [ListView], [PageView], [GridView], and [CustomScrollView], which combine
///    [Scrollable] and [ShrinkWrappingViewport] into widgets that are easier to
///    use.
///  * [SliverToBoxAdapter], which allows a box widget to be placed inside a
///    sliver context (the opposite of this widget).
///  * [Viewport], a viewport that does not shrink-wrap its contents.
class ShrinkWrappingViewport extends MultiChildRenderObjectWidget {
  /// Creates a widget that is bigger on the inside and shrink wraps its
  /// children in the main axis.
  ///
  /// The viewport listens to the [offset], which means you do not need to
  /// rebuild this widget when the [offset] changes.
  const ShrinkWrappingViewport({
    super.key,
    this.axisDirection = AxisDirection.down,
    this.crossAxisDirection,
    required this.offset,
    this.clipBehavior = Clip.hardEdge,
    List<Widget> slivers = const <Widget>[],
  }) : super(children: slivers);

  /// The direction in which the [offset]'s [ViewportOffset.pixels] increases.
  ///
  /// For example, if the [axisDirection] is [AxisDirection.down], a scroll
  /// offset of zero is at the top of the viewport and increases towards the
  /// bottom of the viewport.
  final AxisDirection axisDirection;

  /// The direction in which child should be laid out in the cross axis.
  ///
  /// If the [axisDirection] is [AxisDirection.down] or [AxisDirection.up], this
  /// property defaults to [AxisDirection.left] if the ambient [Directionality]
  /// is [TextDirection.rtl] and [AxisDirection.right] if the ambient
  /// [Directionality] is [TextDirection.ltr].
  ///
  /// If the [axisDirection] is [AxisDirection.left] or [AxisDirection.right],
  /// this property defaults to [AxisDirection.down].
  final AxisDirection? crossAxisDirection;

  /// Which part of the content inside the viewport should be visible.
  ///
  /// The [ViewportOffset.pixels] value determines the scroll offset that the
  /// viewport uses to select which part of its content to display. As the user
  /// scrolls the viewport, this value changes, which changes the content that
  /// is displayed.
  ///
  /// Typically a [ScrollPosition].
  final ViewportOffset offset;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  @override
  RenderShrinkWrappingViewport createRenderObject(BuildContext context) {
    return RenderShrinkWrappingViewport(
      axisDirection: axisDirection,
      crossAxisDirection:
          crossAxisDirection ?? Viewport.getDefaultCrossAxisDirection(context, axisDirection),
      offset: offset,
      clipBehavior: clipBehavior,
    );
  }

  @override
  void updateRenderObject(BuildContext context, RenderShrinkWrappingViewport renderObject) {
    renderObject
      ..axisDirection = axisDirection
      ..crossAxisDirection =
          crossAxisDirection ?? Viewport.getDefaultCrossAxisDirection(context, axisDirection)
      ..offset = offset
      ..clipBehavior = clipBehavior;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<AxisDirection>('axisDirection', axisDirection));
    properties.add(
      EnumProperty<AxisDirection>('crossAxisDirection', crossAxisDirection, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<ViewportOffset>('offset', offset));
  }
}
