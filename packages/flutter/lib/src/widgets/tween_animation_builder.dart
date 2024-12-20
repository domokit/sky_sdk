// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/foundation.dart';
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'animated_size.dart';
/// @docImport 'transitions.dart';
library;

import 'package:flutter/animation.dart';

import 'framework.dart';
import 'implicit_animations.dart';
import 'value_listenable_builder.dart';

/// [Widget] builder that animates a property of a [Widget] to a target value
/// whenever the target value changes.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=l9uHB8VXZOg}
///
/// The type of the animated property ([Color], [Rect], [double], etc.) is
/// defined via the type of the provided [tween] (e.g. [ColorTween],
/// [RectTween], [Tween<double>], etc.).
///
/// The [tween] also defines the target value for the animation: When the widget
/// first builds, it animates from [Tween.begin] to [Tween.end]. A new animation
/// can be triggered anytime by providing a new [tween] with a new [Tween.end]
/// value. The new animation runs from the current animation value (which may be
/// [Tween.end] of the old [tween], if that animation completed) to [Tween.end]
/// of the new [tween].
///
/// The animation is further customized by providing a [curve] and [duration].
///
/// The current value of the animation along with the [child] is passed to
/// the [builder] callback, which is expected to build a [Widget] based on the
/// current animation value. The [builder] is called throughout the animation
/// for every animation value until [Tween.end] is reached.
///
/// A provided [onEnd] callback is called whenever an animation completes.
/// Registering an [onEnd] callback my be useful to trigger an action (like
/// another animation) at the end of the current animation.
///
/// ## Performance optimizations
///
/// If your [builder] function contains a subtree that does not depend on the
/// animation, it's more efficient to build that subtree once instead of
/// rebuilding it on every animation tick.
///
/// If you pass the pre-built subtree as the [child] parameter, the
/// AnimatedBuilder will pass it back to your builder function so that you
/// can incorporate it into your build.
///
/// Using this pre-built child is entirely optional, but can improve
/// performance significantly in some cases and is therefore a good practice.
///
/// ## Ownership of the [Tween]
///
/// The [TweenAnimationBuilder] takes full ownership of the provided [tween]
/// instance and it will mutate it. Once a [Tween] has been passed to a
/// [TweenAnimationBuilder], its properties should not be accessed or changed
/// anymore to avoid interference with the [TweenAnimationBuilder].
///
/// It is good practice to never store a [Tween] provided to a
/// [TweenAnimationBuilder] in an instance variable to avoid accidental
/// modifications of the [Tween].
///
/// ## Example Code
///
/// {@tool dartpad}
/// This example shows an [IconButton] that "zooms" in when the widget first
/// builds (its size smoothly increases from 0 to 24) and whenever the button
/// is pressed, it smoothly changes its size to the new target value of either
/// 48 or 24.
///
/// ** See code in examples/api/lib/widgets/tween_animation_builder/tween_animation_builder.0.dart **
/// {@end-tool}
///
/// ## Relationship to [ImplicitlyAnimatedWidget]s and [AnimatedWidget]s
///
/// The [ImplicitlyAnimatedWidget] has many subclasses that provide animated
/// versions of regular widgets. These subclasses (like [AnimatedOpacity],
/// [AnimatedContainer], [AnimatedSize], etc.) animate changes in their
/// properties smoothly and they are easier to use than this general-purpose
/// builder. However, [TweenAnimationBuilder] (which itself is a subclass of
/// [ImplicitlyAnimatedWidget]) is handy for animating any widget property to a
/// given target value even when the framework (or third-party widget library)
/// doesn't ship with an animated version of that widget.
///
/// Those [ImplicitlyAnimatedWidget]s (including this [TweenAnimationBuilder])
/// all manage an internal [AnimationController] to drive the animation. If you
/// want more control over the animation than just setting a target value,
/// [duration], and [curve], have a look at (subclasses of) [AnimatedWidget]s.
/// For those, you have to manually manage an [AnimationController] giving you
/// full control over the animation. An example of an [AnimatedWidget] is the
/// [AnimatedBuilder], which can be used similarly to this
/// [TweenAnimationBuilder], but unlike the latter it is powered by a
/// developer-managed [AnimationController].
///
/// See also:
///
/// * [ValueListenableBuilder], a widget whose content stays synced with a
///   [ValueListenable] instead of a [Tween].
class TweenAnimationBuilder<T extends Object?> extends ImplicitlyAnimatedWidget {
  /// Creates a [TweenAnimationBuilder].
  ///
  /// The [TweenAnimationBuilder] takes full ownership of the provided [tween]
  /// instance and mutates it. Once a [Tween] has been passed to a
  /// [TweenAnimationBuilder], its properties should not be accessed or changed
  /// anymore to avoid interference with the [TweenAnimationBuilder].
  const TweenAnimationBuilder({
    super.key,
    required this.tween,
    required super.duration,
    super.curve,
    required this.builder,
    super.onEnd,
    this.child,
  });

  /// Defines the target value for the animation.
  ///
  /// When the widget first builds, the animation runs from [Tween.begin] to
  /// [Tween.end], if [Tween.begin] is non-null. A new animation can be
  /// triggered at anytime by providing a new [Tween] with a new [Tween.end]
  /// value. The new animation runs from the current animation value (which may
  /// be [Tween.end] of the old [tween], if that animation completed) to
  /// [Tween.end] of the new [tween]. The [Tween.begin] value is ignored except
  /// for the initial animation that is triggered when the widget builds for the
  /// first time.
  ///
  /// Any (subclass of) [Tween] is accepted as an argument. For example, to
  /// animate the height or width of a [Widget], use a [Tween<double>], or
  /// check out the [ColorTween] to animate the color property of a [Widget].
  ///
  /// Any [Tween] provided must have a non-null [Tween.end] value.
  ///
  /// ## Ownership
  ///
  /// The [TweenAnimationBuilder] takes full ownership of the provided [Tween]
  /// and it will mutate the [Tween]. Once a [Tween] instance has been passed
  /// to [TweenAnimationBuilder] its properties should not be accessed or
  /// changed anymore to avoid any interference with the
  /// [TweenAnimationBuilder]. If you need to change the [Tween], create a
  /// **new instance** with the new values.
  ///
  /// It is good practice to never store a [Tween] provided to a
  /// [TweenAnimationBuilder] in an instance variable to avoid accidental
  /// modifications of the [Tween].
  final Tween<T> tween;

  /// Called every time the animation value changes.
  ///
  /// The current animation value is passed to the builder along with the
  /// [child]. The builder should build a [Widget] based on the current
  /// animation value and incorporate the [child] into it, if it is non-null.
  final ValueWidgetBuilder<T> builder;

  /// The child widget to pass to the builder.
  ///
  /// If a builder callback's return value contains a subtree that does not
  /// depend on the animation, it's more efficient to build that subtree once
  /// instead of rebuilding it on every animation tick.
  ///
  /// If the pre-built subtree is passed as the child parameter, the
  /// [TweenAnimationBuilder] will pass it back to the [builder] function so
  /// that it can be incorporated into the build.
  ///
  /// Using this pre-built child is entirely optional, but can improve
  /// performance significantly in some cases and is therefore a good practice.
  final Widget? child;

  @override
  ImplicitlyAnimatedWidgetState<ImplicitlyAnimatedWidget> createState() {
    return _TweenAnimationBuilderState<T>();
  }
}

class _TweenAnimationBuilderState<T extends Object?>
    extends AnimatedWidgetBaseState<TweenAnimationBuilder<T>> {
  Tween<T>? _currentTween;

  @override
  void initState() {
    _currentTween = widget.tween;
    _currentTween!.begin ??= _currentTween!.end;
    super.initState();
    if (_currentTween!.begin != _currentTween!.end) {
      controller.forward();
    }
  }

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    assert(
      widget.tween.end != null,
      'Tween provided to TweenAnimationBuilder must have non-null Tween.end value.',
    );
    _currentTween =
        visitor(_currentTween, widget.tween.end, (dynamic value) {
              assert(false);
              throw StateError(
                'Constructor will never be called because null is never provided as current tween.',
              );
            })
            as Tween<T>?;
  }

  @override
  Widget build(BuildContext context) {
    return widget.builder(context, _currentTween!.evaluate(animation), widget.child);
  }
}
