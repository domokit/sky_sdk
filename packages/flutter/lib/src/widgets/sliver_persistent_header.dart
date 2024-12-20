// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'nested_scroll_view.dart';
/// @docImport 'scroll_view.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart' show TickerProvider;

import 'framework.dart';
import 'scroll_position.dart';
import 'scrollable.dart';

/// Delegate for configuring a [SliverPersistentHeader].
abstract class SliverPersistentHeaderDelegate {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const SliverPersistentHeaderDelegate();

  /// The widget to place inside the [SliverPersistentHeader].
  ///
  /// The `context` is the [BuildContext] of the sliver.
  ///
  /// The `shrinkOffset` is a distance from [maxExtent] towards [minExtent]
  /// representing the current amount by which the sliver has been shrunk. When
  /// the `shrinkOffset` is zero, the contents will be rendered with a dimension
  /// of [maxExtent] in the main axis. When `shrinkOffset` equals the difference
  /// between [maxExtent] and [minExtent] (a positive number), the contents will
  /// be rendered with a dimension of [minExtent] in the main axis. The
  /// `shrinkOffset` will always be a positive number in that range.
  ///
  /// The `overlapsContent` argument is true if subsequent slivers (if any) will
  /// be rendered beneath this one, and false if the sliver will not have any
  /// contents below it. Typically this is used to decide whether to draw a
  /// shadow to simulate the sliver being above the contents below it. Typically
  /// this is true when `shrinkOffset` is at its greatest value and false
  /// otherwise, but that is not guaranteed. See [NestedScrollView] for an
  /// example of a case where `overlapsContent`'s value can be unrelated to
  /// `shrinkOffset`.
  Widget build(BuildContext context, double shrinkOffset, bool overlapsContent);

  /// The smallest size to allow the header to reach, when it shrinks at the
  /// start of the viewport.
  ///
  /// This must return a value equal to or less than [maxExtent].
  ///
  /// This value should not change over the lifetime of the delegate. It should
  /// be based entirely on the constructor arguments passed to the delegate. See
  /// [shouldRebuild], which must return true if a new delegate would return a
  /// different value.
  double get minExtent;

  /// The size of the header when it is not shrinking at the top of the
  /// viewport.
  ///
  /// This must return a value equal to or greater than [minExtent].
  ///
  /// This value should not change over the lifetime of the delegate. It should
  /// be based entirely on the constructor arguments passed to the delegate. See
  /// [shouldRebuild], which must return true if a new delegate would return a
  /// different value.
  double get maxExtent;

  /// A [TickerProvider] to use when animating the header's size changes.
  ///
  /// Must not be null if the persistent header is a floating header, and
  /// [snapConfiguration] or [showOnScreenConfiguration] is not null.
  TickerProvider? get vsync => null;

  /// Specifies how floating headers should animate in and out of view.
  ///
  /// If the value of this property is null, then floating headers will
  /// not animate into place.
  ///
  /// This is only used for floating headers (those with
  /// [SliverPersistentHeader.floating] set to true).
  ///
  /// Defaults to null.
  FloatingHeaderSnapConfiguration? get snapConfiguration => null;

  /// Specifies an [AsyncCallback] and offset for execution.
  ///
  /// If the value of this property is null, then callback will not be
  /// triggered.
  ///
  /// This is only used for stretching headers (those with
  /// [SliverAppBar.stretch] set to true).
  ///
  /// Defaults to null.
  OverScrollHeaderStretchConfiguration? get stretchConfiguration => null;

  /// Specifies how floating headers and pinned headers should behave in
  /// response to [RenderObject.showOnScreen] calls.
  ///
  /// Defaults to null.
  PersistentHeaderShowOnScreenConfiguration? get showOnScreenConfiguration => null;

  /// Whether this delegate is meaningfully different from the old delegate.
  ///
  /// If this returns false, then the header might not be rebuilt, even though
  /// the instance of the delegate changed.
  ///
  /// This must return true if `oldDelegate` and this object would return
  /// different values for [minExtent], [maxExtent], [snapConfiguration], or
  /// would return a meaningfully different widget tree from [build] for the
  /// same arguments.
  bool shouldRebuild(covariant SliverPersistentHeaderDelegate oldDelegate);
}

/// A sliver whose size varies when the sliver is scrolled to the edge
/// of the viewport opposite the sliver's [GrowthDirection].
///
/// In the normal case of a [CustomScrollView] with no centered sliver, this
/// sliver will vary its size when scrolled to the leading edge of the viewport.
///
/// This is the layout primitive that [SliverAppBar] uses for its
/// shrinking/growing effect.
///
/// _To learn more about slivers, see [CustomScrollView.slivers]._
class SliverPersistentHeader extends StatelessWidget {
  /// Creates a sliver that varies its size when it is scrolled to the start of
  /// a viewport.
  const SliverPersistentHeader({
    super.key,
    required this.delegate,
    this.pinned = false,
    this.floating = false,
  });

  /// Configuration for the sliver's layout.
  ///
  /// The delegate provides the following information:
  ///
  ///  * The minimum and maximum dimensions of the sliver.
  ///
  ///  * The builder for generating the widgets of the sliver.
  ///
  ///  * The instructions for snapping the scroll offset, if [floating] is true.
  final SliverPersistentHeaderDelegate delegate;

  /// Whether to stick the header to the start of the viewport once it has
  /// reached its minimum size.
  ///
  /// If this is false, the header will continue scrolling off the screen after
  /// it has shrunk to its minimum extent.
  final bool pinned;

  /// Whether the header should immediately grow again if the user reverses
  /// scroll direction.
  ///
  /// If this is false, the header only grows again once the user reaches the
  /// part of the viewport that contains the sliver.
  ///
  /// The [delegate]'s [SliverPersistentHeaderDelegate.snapConfiguration] is
  /// ignored unless [floating] is true.
  final bool floating;

  @override
  Widget build(BuildContext context) {
    if (floating && pinned) {
      return _SliverFloatingPinnedPersistentHeader(delegate: delegate);
    }
    if (pinned) {
      return _SliverPinnedPersistentHeader(delegate: delegate);
    }
    if (floating) {
      return _SliverFloatingPersistentHeader(delegate: delegate);
    }
    return _SliverScrollingPersistentHeader(delegate: delegate);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<SliverPersistentHeaderDelegate>('delegate', delegate));
    final List<String> flags = <String>[if (pinned) 'pinned', if (floating) 'floating'];
    if (flags.isEmpty) {
      flags.add('normal');
    }
    properties.add(IterableProperty<String>('mode', flags));
  }
}

class _FloatingHeader extends StatefulWidget {
  const _FloatingHeader({required this.child});

  final Widget child;

  @override
  _FloatingHeaderState createState() => _FloatingHeaderState();
}

// A wrapper for the widget created by _SliverPersistentHeaderElement that
// starts and stops the floating app bar's snap-into-view or snap-out-of-view
// animation. It also informs the float when pointer scrolling by updating the
// last known ScrollDirection when scrolling began.
class _FloatingHeaderState extends State<_FloatingHeader> {
  ScrollPosition? _position;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    if (_position != null) {
      _position!.isScrollingNotifier.removeListener(_isScrollingListener);
    }
    _position = Scrollable.maybeOf(context)?.position;
    if (_position != null) {
      _position!.isScrollingNotifier.addListener(_isScrollingListener);
    }
  }

  @override
  void dispose() {
    if (_position != null) {
      _position!.isScrollingNotifier.removeListener(_isScrollingListener);
    }
    super.dispose();
  }

  RenderSliverFloatingPersistentHeader? _headerRenderer() {
    return context.findAncestorRenderObjectOfType<RenderSliverFloatingPersistentHeader>();
  }

  void _isScrollingListener() {
    assert(_position != null);

    // When a scroll stops, then maybe snap the app bar into view.
    // Similarly, when a scroll starts, then maybe stop the snap animation.
    // Update the scrolling direction as well for pointer scrolling updates.
    final RenderSliverFloatingPersistentHeader? header = _headerRenderer();
    if (_position!.isScrollingNotifier.value) {
      header?.updateScrollStartDirection(_position!.userScrollDirection);
      // Only SliverAppBars support snapping, headers will not snap.
      header?.maybeStopSnapAnimation(_position!.userScrollDirection);
    } else {
      // Only SliverAppBars support snapping, headers will not snap.
      header?.maybeStartSnapAnimation(_position!.userScrollDirection);
    }
  }

  @override
  Widget build(BuildContext context) => widget.child;
}

class _SliverPersistentHeaderElement extends RenderObjectElement {
  _SliverPersistentHeaderElement(
    _SliverPersistentHeaderRenderObjectWidget super.widget, {
    this.floating = false,
  });

  final bool floating;

  @override
  _RenderSliverPersistentHeaderForWidgetsMixin get renderObject =>
      super.renderObject as _RenderSliverPersistentHeaderForWidgetsMixin;

  @override
  void mount(Element? parent, Object? newSlot) {
    super.mount(parent, newSlot);
    renderObject._element = this;
  }

  @override
  void unmount() {
    renderObject._element = null;
    super.unmount();
  }

  @override
  void update(_SliverPersistentHeaderRenderObjectWidget newWidget) {
    final _SliverPersistentHeaderRenderObjectWidget oldWidget =
        widget as _SliverPersistentHeaderRenderObjectWidget;
    super.update(newWidget);
    final SliverPersistentHeaderDelegate newDelegate = newWidget.delegate;
    final SliverPersistentHeaderDelegate oldDelegate = oldWidget.delegate;
    if (newDelegate != oldDelegate &&
        (newDelegate.runtimeType != oldDelegate.runtimeType ||
            newDelegate.shouldRebuild(oldDelegate))) {
      final _RenderSliverPersistentHeaderForWidgetsMixin renderObject = this.renderObject;
      _updateChild(newDelegate, renderObject.lastShrinkOffset, renderObject.lastOverlapsContent);
      renderObject.triggerRebuild();
    }
  }

  @override
  void performRebuild() {
    super.performRebuild();
    renderObject.triggerRebuild();
  }

  Element? child;

  void _updateChild(
    SliverPersistentHeaderDelegate delegate,
    double shrinkOffset,
    bool overlapsContent,
  ) {
    final Widget newWidget = delegate.build(this, shrinkOffset, overlapsContent);
    child = updateChild(child, floating ? _FloatingHeader(child: newWidget) : newWidget, null);
  }

  void _build(double shrinkOffset, bool overlapsContent) {
    owner!.buildScope(this, () {
      final _SliverPersistentHeaderRenderObjectWidget sliverPersistentHeaderRenderObjectWidget =
          widget as _SliverPersistentHeaderRenderObjectWidget;
      _updateChild(
        sliverPersistentHeaderRenderObjectWidget.delegate,
        shrinkOffset,
        overlapsContent,
      );
    });
  }

  @override
  void forgetChild(Element child) {
    assert(child == this.child);
    this.child = null;
    super.forgetChild(child);
  }

  @override
  void insertRenderObjectChild(covariant RenderBox child, Object? slot) {
    assert(renderObject.debugValidateChild(child));
    renderObject.child = child;
  }

  @override
  void moveRenderObjectChild(covariant RenderObject child, Object? oldSlot, Object? newSlot) {
    assert(false);
  }

  @override
  void removeRenderObjectChild(covariant RenderObject child, Object? slot) {
    renderObject.child = null;
  }

  @override
  void visitChildren(ElementVisitor visitor) {
    if (child != null) {
      visitor(child!);
    }
  }
}

abstract class _SliverPersistentHeaderRenderObjectWidget extends RenderObjectWidget {
  const _SliverPersistentHeaderRenderObjectWidget({required this.delegate, this.floating = false});

  final SliverPersistentHeaderDelegate delegate;
  final bool floating;

  @override
  _SliverPersistentHeaderElement createElement() =>
      _SliverPersistentHeaderElement(this, floating: floating);

  @override
  _RenderSliverPersistentHeaderForWidgetsMixin createRenderObject(BuildContext context);

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(DiagnosticsProperty<SliverPersistentHeaderDelegate>('delegate', delegate));
  }
}

mixin _RenderSliverPersistentHeaderForWidgetsMixin on RenderSliverPersistentHeader {
  _SliverPersistentHeaderElement? _element;

  @override
  double get minExtent =>
      (_element!.widget as _SliverPersistentHeaderRenderObjectWidget).delegate.minExtent;

  @override
  double get maxExtent =>
      (_element!.widget as _SliverPersistentHeaderRenderObjectWidget).delegate.maxExtent;

  @override
  void updateChild(double shrinkOffset, bool overlapsContent) {
    assert(_element != null);
    _element!._build(shrinkOffset, overlapsContent);
  }

  @protected
  void triggerRebuild() {
    markNeedsLayout();
  }
}

class _SliverScrollingPersistentHeader extends _SliverPersistentHeaderRenderObjectWidget {
  const _SliverScrollingPersistentHeader({required super.delegate});

  @override
  _RenderSliverPersistentHeaderForWidgetsMixin createRenderObject(BuildContext context) {
    return _RenderSliverScrollingPersistentHeaderForWidgets(
      stretchConfiguration: delegate.stretchConfiguration,
    );
  }

  @override
  void updateRenderObject(
    BuildContext context,
    covariant _RenderSliverScrollingPersistentHeaderForWidgets renderObject,
  ) {
    renderObject.stretchConfiguration = delegate.stretchConfiguration;
  }
}

class _RenderSliverScrollingPersistentHeaderForWidgets extends RenderSliverScrollingPersistentHeader
    with _RenderSliverPersistentHeaderForWidgetsMixin {
  _RenderSliverScrollingPersistentHeaderForWidgets({super.stretchConfiguration});
}

class _SliverPinnedPersistentHeader extends _SliverPersistentHeaderRenderObjectWidget {
  const _SliverPinnedPersistentHeader({required super.delegate});

  @override
  _RenderSliverPersistentHeaderForWidgetsMixin createRenderObject(BuildContext context) {
    return _RenderSliverPinnedPersistentHeaderForWidgets(
      stretchConfiguration: delegate.stretchConfiguration,
      showOnScreenConfiguration: delegate.showOnScreenConfiguration,
    );
  }

  @override
  void updateRenderObject(
    BuildContext context,
    covariant _RenderSliverPinnedPersistentHeaderForWidgets renderObject,
  ) {
    renderObject
      ..stretchConfiguration = delegate.stretchConfiguration
      ..showOnScreenConfiguration = delegate.showOnScreenConfiguration;
  }
}

class _RenderSliverPinnedPersistentHeaderForWidgets extends RenderSliverPinnedPersistentHeader
    with _RenderSliverPersistentHeaderForWidgetsMixin {
  _RenderSliverPinnedPersistentHeaderForWidgets({
    super.stretchConfiguration,
    super.showOnScreenConfiguration,
  });
}

class _SliverFloatingPersistentHeader extends _SliverPersistentHeaderRenderObjectWidget {
  const _SliverFloatingPersistentHeader({required super.delegate}) : super(floating: true);

  @override
  _RenderSliverPersistentHeaderForWidgetsMixin createRenderObject(BuildContext context) {
    return _RenderSliverFloatingPersistentHeaderForWidgets(
      vsync: delegate.vsync,
      snapConfiguration: delegate.snapConfiguration,
      stretchConfiguration: delegate.stretchConfiguration,
      showOnScreenConfiguration: delegate.showOnScreenConfiguration,
    );
  }

  @override
  void updateRenderObject(
    BuildContext context,
    _RenderSliverFloatingPersistentHeaderForWidgets renderObject,
  ) {
    renderObject.vsync = delegate.vsync;
    renderObject.snapConfiguration = delegate.snapConfiguration;
    renderObject.stretchConfiguration = delegate.stretchConfiguration;
    renderObject.showOnScreenConfiguration = delegate.showOnScreenConfiguration;
  }
}

class _RenderSliverFloatingPinnedPersistentHeaderForWidgets
    extends RenderSliverFloatingPinnedPersistentHeader
    with _RenderSliverPersistentHeaderForWidgetsMixin {
  _RenderSliverFloatingPinnedPersistentHeaderForWidgets({
    required super.vsync,
    super.snapConfiguration,
    super.stretchConfiguration,
    super.showOnScreenConfiguration,
  });
}

class _SliverFloatingPinnedPersistentHeader extends _SliverPersistentHeaderRenderObjectWidget {
  const _SliverFloatingPinnedPersistentHeader({required super.delegate}) : super(floating: true);

  @override
  _RenderSliverPersistentHeaderForWidgetsMixin createRenderObject(BuildContext context) {
    return _RenderSliverFloatingPinnedPersistentHeaderForWidgets(
      vsync: delegate.vsync,
      snapConfiguration: delegate.snapConfiguration,
      stretchConfiguration: delegate.stretchConfiguration,
      showOnScreenConfiguration: delegate.showOnScreenConfiguration,
    );
  }

  @override
  void updateRenderObject(
    BuildContext context,
    _RenderSliverFloatingPinnedPersistentHeaderForWidgets renderObject,
  ) {
    renderObject.vsync = delegate.vsync;
    renderObject.snapConfiguration = delegate.snapConfiguration;
    renderObject.stretchConfiguration = delegate.stretchConfiguration;
    renderObject.showOnScreenConfiguration = delegate.showOnScreenConfiguration;
  }
}

class _RenderSliverFloatingPersistentHeaderForWidgets extends RenderSliverFloatingPersistentHeader
    with _RenderSliverPersistentHeaderForWidgetsMixin {
  _RenderSliverFloatingPersistentHeaderForWidgets({
    required super.vsync,
    super.snapConfiguration,
    super.stretchConfiguration,
    super.showOnScreenConfiguration,
  });
}
