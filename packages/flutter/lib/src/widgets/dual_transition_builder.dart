// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'basic.dart';
import 'framework.dart';

/// Builder callback used by [DualTransitionBuilder].
///
/// The builder is expected to return a transition powered by the provided
/// `animation` and wrapping the provided `child`.
///
/// The `animation` provided to the builder always runs forward from 0.0 to 1.0.
typedef AnimatedTransitionBuilder =
    Widget Function(BuildContext context, Animation<double> animation, Widget? child);

/// A transition builder that animates its [child] based on the
/// [AnimationStatus] of the provided [animation].
///
/// This widget can be used to specify different enter and exit transitions for
/// a [child].
///
/// While the [animation] runs forward, the [child] is animated according to
/// [forwardBuilder] and while the [animation] is running in reverse, it is
/// animated according to [reverseBuilder].
///
/// Using this builder allows the widget tree to maintain its shape by nesting
/// the enter and exit transitions. This ensures that no state information of
/// any descendant widget is lost when the transition starts or completes.
class DualTransitionBuilder extends StatefulWidget {
  /// Creates a [DualTransitionBuilder].
  const DualTransitionBuilder({
    super.key,
    required this.animation,
    required this.forwardBuilder,
    required this.reverseBuilder,
    this.child,
  });

  /// The animation that drives the [child]'s transition.
  ///
  /// When this animation runs forward, the [child] transitions as specified by
  /// [forwardBuilder]. When it runs in reverse, the child transitions according
  /// to [reverseBuilder].
  final Animation<double> animation;

  /// A builder for the transition that makes [child] appear on screen.
  ///
  /// The [child] should be fully visible when the provided `animation` reaches
  /// 1.0.
  ///
  /// The `animation` provided to this builder is running forward from 0.0 to
  /// 1.0 when [animation] runs _forward_. When [animation] runs in reverse,
  /// the given `animation` is set to [kAlwaysCompleteAnimation].
  ///
  /// See also:
  ///
  ///  * [reverseBuilder], which builds the transition for making the [child]
  ///   disappear from the screen.
  final AnimatedTransitionBuilder forwardBuilder;

  /// A builder for a transition that makes [child] disappear from the screen.
  ///
  /// The [child] should be fully invisible when the provided `animation`
  /// reaches 1.0.
  ///
  /// The `animation` provided to this builder is running forward from 0.0 to
  /// 1.0 when [animation] runs in _reverse_. When [animation] runs forward,
  /// the given `animation` is set to [kAlwaysDismissedAnimation].
  ///
  /// See also:
  ///
  ///  * [forwardBuilder], which builds the transition for making the [child]
  ///    appear on screen.
  final AnimatedTransitionBuilder reverseBuilder;

  /// The widget below this [DualTransitionBuilder] in the tree.
  ///
  /// This child widget will be wrapped by the transitions built by
  /// [forwardBuilder] and [reverseBuilder].
  final Widget? child;

  @override
  State<DualTransitionBuilder> createState() => _DualTransitionBuilderState();
}

class _DualTransitionBuilderState extends State<DualTransitionBuilder> {
  late AnimationStatus _effectiveAnimationStatus;
  final ProxyAnimation _forwardAnimation = ProxyAnimation();
  final ProxyAnimation _reverseAnimation = ProxyAnimation();

  @override
  void initState() {
    super.initState();
    _effectiveAnimationStatus = widget.animation.status;
    widget.animation.addStatusListener(_animationListener);
    _updateAnimations();
  }

  void _animationListener(AnimationStatus animationStatus) {
    final AnimationStatus oldEffective = _effectiveAnimationStatus;
    _effectiveAnimationStatus = _calculateEffectiveAnimationStatus(
      lastEffective: _effectiveAnimationStatus,
      current: animationStatus,
    );
    if (oldEffective != _effectiveAnimationStatus) {
      _updateAnimations();
    }
  }

  @override
  void didUpdateWidget(DualTransitionBuilder oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.animation != widget.animation) {
      oldWidget.animation.removeStatusListener(_animationListener);
      widget.animation.addStatusListener(_animationListener);
      _animationListener(widget.animation.status);
    }
  }

  // When a transition is interrupted midway we just want to play the ongoing
  // animation in reverse. Switching to the actual reverse transition would
  // yield a disjoint experience since the forward and reverse transitions are
  // very different.
  AnimationStatus _calculateEffectiveAnimationStatus({
    required AnimationStatus lastEffective,
    required AnimationStatus current,
  }) {
    switch (current) {
      case AnimationStatus.dismissed:
      case AnimationStatus.completed:
        return current;
      case AnimationStatus.forward:
        switch (lastEffective) {
          case AnimationStatus.dismissed:
          case AnimationStatus.completed:
          case AnimationStatus.forward:
            return current;
          case AnimationStatus.reverse:
            return lastEffective;
        }
      case AnimationStatus.reverse:
        switch (lastEffective) {
          case AnimationStatus.dismissed:
          case AnimationStatus.completed:
          case AnimationStatus.reverse:
            return current;
          case AnimationStatus.forward:
            return lastEffective;
        }
    }
  }

  void _updateAnimations() {
    switch (_effectiveAnimationStatus) {
      case AnimationStatus.dismissed:
      case AnimationStatus.forward:
        _forwardAnimation.parent = widget.animation;
        _reverseAnimation.parent = kAlwaysDismissedAnimation;
      case AnimationStatus.reverse:
      case AnimationStatus.completed:
        _forwardAnimation.parent = kAlwaysCompleteAnimation;
        _reverseAnimation.parent = ReverseAnimation(widget.animation);
    }
  }

  @override
  void dispose() {
    widget.animation.removeStatusListener(_animationListener);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return widget.forwardBuilder(
      context,
      _forwardAnimation,
      widget.reverseBuilder(context, _reverseAnimation, widget.child),
    );
  }
}
