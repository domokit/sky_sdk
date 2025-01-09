// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'pinned_header_sliver.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'sliver_persistent_header.dart';
/// @docImport 'sliver_resizing_header.dart';
library;

import 'dart:math' as math;

import 'package:flutter/animation.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

import 'framework.dart';
import 'scroll_position.dart';
import 'scrollable.dart';
import 'ticker_provider.dart';

/// Specifies how a partially visible [SliverFloatingHeader] animates
/// into a view when a user scroll gesture ends.
///
/// During a user scroll gesture the header and the rest of the scrollable
/// content move in sync. If the header is partially visible when the
/// scroll gesture ends, [SliverFloatingHeader.snapMode] specifies if
/// the header should [FloatingHeaderSnapMode.overlay] the scrollable's
/// content as it expands until it's completely visible, or if the
/// content should scroll out of the way as the header expands.
enum FloatingHeaderSnapMode {
  /// At the end of a user scroll gesture, the [SliverFloatingHeader] will
  /// expand over the scrollable's content.
  overlay,

  /// At the end of a user scroll gesture, the [SliverFloatingHeader] will
  /// expand and the scrollable's content will continue to scroll out
  /// of the way.
  scroll,
}

/// A sliver that shows its [child] when the user scrolls forward and hides it
/// when the user scrolls backwards.
///
/// This sliver is preferable to the general purpose [SliverPersistentHeader]
/// for its relatively narrow use case because there's no need to create a
/// [SliverPersistentHeaderDelegate] or to predict the header's size.
///
/// {@tool dartpad}
/// This example shows how to create a SliverFloatingHeader.
///
/// ** See code in examples/api/lib/widgets/sliver/sliver_floating_header.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [PinnedHeaderSliver] - which just pins the header at the top
///    of the [CustomScrollView].
///  * [SliverResizingHeader] - which similarly pins the header at the top
///    of the [CustomScrollView] but reacts to scrolling by resizing the header
///    between its minimum and maximum extent limits.
///  * [SliverPersistentHeader] - a general purpose header that can be
///    configured as a pinned, resizing, or floating header.
class SliverFloatingHeader extends StatefulWidget {
  /// Create a floating header sliver that animates into view when the user
  /// scrolls forward, and disappears the user starts scrolling in the
  /// opposite direction.
  const SliverFloatingHeader({super.key, this.animationStyle, this.snapMode, required this.child});

  /// Non null properties override the default durations (300ms) and
  /// curves (Curves.easeInOut) for subsequent header animations.
  ///
  /// The reverse duration and curve apply to the animation that hides the header.
  final AnimationStyle? animationStyle;

  /// Specifies how a partially visible [SliverFloatingHeader] animates
  /// into a view when a user scroll gesture ends.
  ///
  /// The default is [FloatingHeaderSnapMode.overlay]. This parameter doesn't
  /// modify an animation in progress, just subsequent animations.
  final FloatingHeaderSnapMode? snapMode;

  /// The widget contained by this sliver.
  final Widget child;

  @override
  State<SliverFloatingHeader> createState() => _SliverFloatingHeaderState();
}

class _SliverFloatingHeaderState extends State<SliverFloatingHeader>
    with SingleTickerProviderStateMixin {
  ScrollPosition? position;

  @override
  Widget build(BuildContext context) {
    return _SliverFloatingHeader(
      vsync: this,
      animationStyle: widget.animationStyle,
      snapMode: widget.snapMode,
      child: _SnapTrigger(widget.child),
    );
  }
}

class _SnapTrigger extends StatefulWidget {
  const _SnapTrigger(this.child);

  final Widget child;

  @override
  _SnapTriggerState createState() => _SnapTriggerState();
}

class _SnapTriggerState extends State<_SnapTrigger> {
  ScrollPosition? position;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    if (position != null) {
      position!.isScrollingNotifier.removeListener(isScrollingListener);
    }
    position = Scrollable.maybeOf(context)?.position;
    if (position != null) {
      position!.isScrollingNotifier.addListener(isScrollingListener);
    }
  }

  @override
  void dispose() {
    if (position != null) {
      position!.isScrollingNotifier.removeListener(isScrollingListener);
    }
    super.dispose();
  }

  // Called when the sliver starts or ends scrolling.
  void isScrollingListener() {
    assert(position != null);
    final _RenderSliverFloatingHeader? renderer =
        context.findAncestorRenderObjectOfType<_RenderSliverFloatingHeader>();
    renderer?.isScrollingUpdate(position!);
  }

  @override
  Widget build(BuildContext context) => widget.child;
}

class _SliverFloatingHeader extends SingleChildRenderObjectWidget {
  const _SliverFloatingHeader({this.vsync, this.animationStyle, this.snapMode, super.child});

  final TickerProvider? vsync;
  final AnimationStyle? animationStyle;
  final FloatingHeaderSnapMode? snapMode;

  @override
  _RenderSliverFloatingHeader createRenderObject(BuildContext context) {
    return _RenderSliverFloatingHeader(
      vsync: vsync,
      animationStyle: animationStyle,
      snapMode: snapMode,
    );
  }

  @override
  void updateRenderObject(BuildContext context, _RenderSliverFloatingHeader renderObject) {
    renderObject
      ..vsync = vsync
      ..animationStyle = animationStyle
      ..snapMode = snapMode;
  }
}

class _RenderSliverFloatingHeader extends RenderSliverSingleBoxAdapter {
  _RenderSliverFloatingHeader({TickerProvider? vsync, this.animationStyle, this.snapMode})
    : _vsync = vsync;

  late Animation<double> snapAnimation;
  AnimationController? snapController;
  double? lastScrollOffset;

  // The distance from the start of the header to the start of the viewport. When the
  // header is showing it varies between 0 (completely visible) and childExtent (not visible
  // because it's just above the viewport's starting edge). It's used to compute the
  // header's paintExtent which defines where the header will appear - see paint().
  late double effectiveScrollOffset;

  TickerProvider? get vsync => _vsync;
  TickerProvider? _vsync;
  set vsync(TickerProvider? value) {
    if (value == _vsync) {
      return;
    }
    _vsync = value;
    if (value == null) {
      snapController?.dispose();
      snapController = null;
    } else {
      snapController?.resync(value);
    }
  }

  AnimationStyle? animationStyle;

  FloatingHeaderSnapMode? snapMode;

  // Called each time the position's isScrollingNotifier indicates that user scrolling has
  // stopped or started, i.e. if the sliver "is scrolling".
  void isScrollingUpdate(ScrollPosition position) {
    if (position.isScrollingNotifier.value) {
      snapController?.stop();
    } else {
      final ScrollDirection direction = position.userScrollDirection;
      final bool headerIsPartiallyVisible = switch (direction) {
        ScrollDirection.forward when effectiveScrollOffset <= 0 => false, // completely visible
        ScrollDirection.reverse when effectiveScrollOffset >= childExtent => false, // not visible
        _ => true,
      };
      if (headerIsPartiallyVisible) {
        snapController ??= AnimationController(vsync: vsync!)..addListener(() {
          if (effectiveScrollOffset != snapAnimation.value) {
            effectiveScrollOffset = snapAnimation.value;
            markNeedsLayout();
          }
        });
        snapController!.duration = switch (direction) {
          ScrollDirection.forward => animationStyle?.duration ?? const Duration(milliseconds: 300),
          _ => animationStyle?.reverseDuration ?? const Duration(milliseconds: 300),
        };
        snapAnimation = snapController!.drive(
          Tween<double>(
            begin: effectiveScrollOffset,
            end: switch (direction) {
              ScrollDirection.forward => 0,
              _ => childExtent,
            },
          ).chain(
            CurveTween(
              curve: switch (direction) {
                ScrollDirection.forward => animationStyle?.curve ?? Curves.easeInOut,
                _ => animationStyle?.reverseCurve ?? Curves.easeInOut,
              },
            ),
          ),
        );
        snapController!.forward(from: 0.0);
      }
    }
  }

  double get childExtent {
    if (child == null) {
      return 0.0;
    }
    assert(child!.hasSize);
    return switch (constraints.axis) {
      Axis.vertical => child!.size.height,
      Axis.horizontal => child!.size.width,
    };
  }

  @override
  void detach() {
    snapController?.dispose();
    snapController = null; // lazily recreated if we're reattached.
    super.detach();
  }

  // True if the header has been laid at at least once (lastScrollOffset != null) and either:
  // - We're scrolling forward: constraints.scrollOffset < lastScrollOffset
  // - The header's already partially visible: effectiveScrollOffset < childExtent
  // Scrolling forwards (towards the scrollable's start) is the trigger that causes the
  // header to be shown.
  bool get floatingHeaderNeedsToBeUpdated {
    return lastScrollOffset != null &&
        (constraints.scrollOffset < lastScrollOffset! || effectiveScrollOffset < childExtent);
  }

  @override
  void performLayout() {
    if (!floatingHeaderNeedsToBeUpdated) {
      effectiveScrollOffset = constraints.scrollOffset;
    } else {
      double delta = lastScrollOffset! - constraints.scrollOffset; // > 0 when the header is growing
      if (constraints.userScrollDirection == ScrollDirection.forward) {
        if (effectiveScrollOffset > childExtent) {
          effectiveScrollOffset =
              childExtent; // The header is now just above the start edge of viewport.
        }
      } else {
        // delta > 0 and scrolling forward is a contradiction. Assume that it's noise (set delta to 0).
        delta = clampDouble(delta, -double.infinity, 0);
      }
      effectiveScrollOffset = clampDouble(
        effectiveScrollOffset - delta,
        0.0,
        constraints.scrollOffset,
      );
    }

    child?.layout(constraints.asBoxConstraints(), parentUsesSize: true);
    final double paintExtent = childExtent - effectiveScrollOffset;
    final double layoutExtent = switch (snapMode ?? FloatingHeaderSnapMode.overlay) {
      FloatingHeaderSnapMode.overlay => childExtent - constraints.scrollOffset,
      FloatingHeaderSnapMode.scroll => paintExtent,
    };
    geometry = SliverGeometry(
      paintOrigin: math.min(constraints.overlap, 0.0),
      scrollExtent: childExtent,
      paintExtent: clampDouble(paintExtent, 0.0, constraints.remainingPaintExtent),
      layoutExtent: clampDouble(layoutExtent, 0.0, constraints.remainingPaintExtent),
      maxPaintExtent: childExtent,
      hasVisualOverflow: true, // Conservatively say we do have overflow to avoid complexity.
    );

    lastScrollOffset = constraints.scrollOffset;
  }

  @override
  double childMainAxisPosition(covariant RenderObject child) {
    return geometry == null ? 0 : math.min(0, geometry!.paintExtent - childExtent);
  }

  @override
  void applyPaintTransform(RenderObject child, Matrix4 transform) {
    assert(child == this.child);
    applyPaintTransformForBoxChild(child as RenderBox, transform);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child != null && geometry!.visible) {
      offset += switch (applyGrowthDirectionToAxisDirection(
        constraints.axisDirection,
        constraints.growthDirection,
      )) {
        AxisDirection.up => Offset(
          0.0,
          geometry!.paintExtent - childMainAxisPosition(child!) - childExtent,
        ),
        AxisDirection.left => Offset(
          geometry!.paintExtent - childMainAxisPosition(child!) - childExtent,
          0.0,
        ),
        AxisDirection.right => Offset(childMainAxisPosition(child!), 0.0),
        AxisDirection.down => Offset(0.0, childMainAxisPosition(child!)),
      };
      context.paintChild(child!, offset);
    }
  }
}
