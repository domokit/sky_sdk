// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'transitions.dart';
library;

import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'framework.dart';
import 'ticker_provider.dart';

/// Animated widget that automatically transitions its size over a given
/// duration whenever the given child's size changes.
///
/// {@tool dartpad}
/// This example defines a widget that uses [AnimatedSize] to change the size of
/// the [SizedBox] on tap.
///
/// ** See code in examples/api/lib/widgets/animated_size/animated_size.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SizeTransition], which changes its size based on an [Animation].
class AnimatedSize extends StatefulWidget {
  /// Creates a widget that animates its size to match that of its child.
  const AnimatedSize({
    super.key,
    this.child,
    this.alignment = Alignment.center,
    this.curve = Curves.linear,
    required this.duration,
    this.reverseDuration,
    this.clipBehavior = Clip.hardEdge,
    this.onEnd,
  });

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// The alignment of the child within the parent when the parent is not yet
  /// the same size as the child.
  ///
  /// The x and y values of the alignment control the horizontal and vertical
  /// alignment, respectively. An x value of -1.0 means that the left edge of
  /// the child is aligned with the left edge of the parent whereas an x value
  /// of 1.0 means that the right edge of the child is aligned with the right
  /// edge of the parent. Other values interpolate (and extrapolate) linearly.
  /// For example, a value of 0.0 means that the center of the child is aligned
  /// with the center of the parent.
  ///
  /// Defaults to [Alignment.center].
  ///
  /// See also:
  ///
  ///  * [Alignment], a class with convenient constants typically used to
  ///    specify an [AlignmentGeometry].
  ///  * [AlignmentDirectional], like [Alignment] for specifying alignments
  ///    relative to text direction.
  final AlignmentGeometry alignment;

  /// The animation curve when transitioning this widget's size to match the
  /// child's size.
  final Curve curve;

  /// The duration when transitioning this widget's size to match the child's
  /// size.
  final Duration duration;

  /// The duration when transitioning this widget's size to match the child's
  /// size when going in reverse.
  ///
  /// If not specified, defaults to [duration].
  final Duration? reverseDuration;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// Called every time an animation completes.
  ///
  /// This can be useful to trigger additional actions (e.g. another animation)
  /// at the end of the current animation.
  final VoidCallback? onEnd;

  @override
  State<AnimatedSize> createState() => _AnimatedSizeState();
}

class _AnimatedSizeState extends State<AnimatedSize> with SingleTickerProviderStateMixin {
  @override
  Widget build(BuildContext context) {
    return _AnimatedSize(
      alignment: widget.alignment,
      curve: widget.curve,
      duration: widget.duration,
      reverseDuration: widget.reverseDuration,
      vsync: this,
      clipBehavior: widget.clipBehavior,
      onEnd: widget.onEnd,
      child: widget.child,
    );
  }
}

class _AnimatedSize extends SingleChildRenderObjectWidget {
  const _AnimatedSize({
    super.child,
    this.alignment = Alignment.center,
    this.curve = Curves.linear,
    required this.duration,
    this.reverseDuration,
    required this.vsync,
    this.clipBehavior = Clip.hardEdge,
    this.onEnd,
  });

  final AlignmentGeometry alignment;
  final Curve curve;
  final Duration duration;
  final Duration? reverseDuration;

  /// The [TickerProvider] for this widget.
  final TickerProvider vsync;

  final Clip clipBehavior;

  final VoidCallback? onEnd;

  @override
  RenderAnimatedSize createRenderObject(BuildContext context) {
    return RenderAnimatedSize(
      alignment: alignment,
      duration: duration,
      reverseDuration: reverseDuration,
      curve: curve,
      vsync: vsync,
      textDirection: Directionality.maybeOf(context),
      clipBehavior: clipBehavior,
      onEnd: onEnd,
    );
  }

  @override
  void updateRenderObject(BuildContext context, RenderAnimatedSize renderObject) {
    renderObject
      ..alignment = alignment
      ..duration = duration
      ..reverseDuration = reverseDuration
      ..curve = curve
      ..vsync = vsync
      ..textDirection = Directionality.maybeOf(context)
      ..clipBehavior = clipBehavior
      ..onEnd = onEnd;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<AlignmentGeometry>(
        'alignment',
        alignment,
        defaultValue: Alignment.topCenter,
      ),
    );
    properties.add(IntProperty('duration', duration.inMilliseconds, unit: 'ms'));
    properties.add(
      IntProperty(
        'reverseDuration',
        reverseDuration?.inMilliseconds,
        unit: 'ms',
        defaultValue: null,
      ),
    );
  }
}
