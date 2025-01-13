// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'animated_cross_fade.dart';
/// @docImport 'animated_size.dart';
/// @docImport 'animated_switcher.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'sliver.dart';
/// @docImport 'tween_animation_builder.dart';
library;

import 'dart:ui' as ui show TextHeightBehavior;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:vector_math/vector_math_64.dart';

import 'basic.dart';
import 'container.dart';
import 'debug.dart';
import 'framework.dart';
import 'text.dart';
import 'ticker_provider.dart';
import 'transitions.dart';

// Examples can assume:
// class MyWidget extends ImplicitlyAnimatedWidget {
//   const MyWidget({super.key, this.targetColor = Colors.black}) : super(duration: const Duration(seconds: 1));
//   final Color targetColor;
//   @override
//   ImplicitlyAnimatedWidgetState<MyWidget> createState() => throw UnimplementedError(); // ignore: no_logic_in_create_state
// }
// void setState(VoidCallback fn) { }

/// An interpolation between two [BoxConstraints].
///
/// This class specializes the interpolation of [Tween<BoxConstraints>] to use
/// [BoxConstraints.lerp].
///
/// See [Tween] for a discussion on how to use interpolation objects.
class BoxConstraintsTween extends Tween<BoxConstraints> {
  /// Creates a [BoxConstraints] tween.
  ///
  /// The [begin] and [end] properties may be null; the null value
  /// is treated as a tight constraint of zero size.
  BoxConstraintsTween({super.begin, super.end});

  /// Returns the value this variable has at the given animation clock value.
  @override
  BoxConstraints lerp(double t) => BoxConstraints.lerp(begin, end, t)!;
}

/// An interpolation between two [Decoration]s.
///
/// This class specializes the interpolation of [Tween<BoxConstraints>] to use
/// [Decoration.lerp].
///
/// For [ShapeDecoration]s which know how to [ShapeDecoration.lerpTo] or
/// [ShapeDecoration.lerpFrom] each other, this will produce a smooth
/// interpolation between decorations.
///
/// See also:
///
///  * [Tween] for a discussion on how to use interpolation objects.
///  * [ShapeDecoration], [RoundedRectangleBorder], [CircleBorder], and
///    [StadiumBorder] for examples of shape borders that can be smoothly
///    interpolated.
///  * [BoxBorder] for a border that can only be smoothly interpolated between other
///    [BoxBorder]s.
class DecorationTween extends Tween<Decoration> {
  /// Creates a decoration tween.
  ///
  /// The [begin] and [end] properties may be null. If both are null, then the
  /// result is always null. If [end] is not null, then its lerping logic is
  /// used (via [Decoration.lerpTo]). Otherwise, [begin]'s lerping logic is used
  /// (via [Decoration.lerpFrom]).
  DecorationTween({super.begin, super.end});

  /// Returns the value this variable has at the given animation clock value.
  @override
  Decoration lerp(double t) => Decoration.lerp(begin, end, t)!;
}

/// An interpolation between two [EdgeInsets]s.
///
/// This class specializes the interpolation of [Tween<EdgeInsets>] to use
/// [EdgeInsets.lerp].
///
/// See [Tween] for a discussion on how to use interpolation objects.
///
/// See also:
///
///  * [EdgeInsetsGeometryTween], which interpolates between two
///    [EdgeInsetsGeometry] objects.
class EdgeInsetsTween extends Tween<EdgeInsets> {
  /// Creates an [EdgeInsets] tween.
  ///
  /// The [begin] and [end] properties may be null; the null value
  /// is treated as an [EdgeInsets] with no inset.
  EdgeInsetsTween({super.begin, super.end});

  /// Returns the value this variable has at the given animation clock value.
  @override
  EdgeInsets lerp(double t) => EdgeInsets.lerp(begin, end, t)!;
}

/// An interpolation between two [EdgeInsetsGeometry]s.
///
/// This class specializes the interpolation of [Tween<EdgeInsetsGeometry>] to
/// use [EdgeInsetsGeometry.lerp].
///
/// See [Tween] for a discussion on how to use interpolation objects.
///
/// See also:
///
///  * [EdgeInsetsTween], which interpolates between two [EdgeInsets] objects.
class EdgeInsetsGeometryTween extends Tween<EdgeInsetsGeometry> {
  /// Creates an [EdgeInsetsGeometry] tween.
  ///
  /// The [begin] and [end] properties may be null; the null value
  /// is treated as an [EdgeInsetsGeometry] with no inset.
  EdgeInsetsGeometryTween({super.begin, super.end});

  /// Returns the value this variable has at the given animation clock value.
  @override
  EdgeInsetsGeometry lerp(double t) => EdgeInsetsGeometry.lerp(begin, end, t)!;
}

/// An interpolation between two [BorderRadius]s.
///
/// This class specializes the interpolation of [Tween<BorderRadius>] to use
/// [BorderRadius.lerp].
///
/// See [Tween] for a discussion on how to use interpolation objects.
class BorderRadiusTween extends Tween<BorderRadius?> {
  /// Creates a [BorderRadius] tween.
  ///
  /// The [begin] and [end] properties may be null; the null value
  /// is treated as a right angle (no radius).
  BorderRadiusTween({super.begin, super.end});

  /// Returns the value this variable has at the given animation clock value.
  @override
  BorderRadius? lerp(double t) => BorderRadius.lerp(begin, end, t);
}

/// An interpolation between two [Border]s.
///
/// This class specializes the interpolation of [Tween<Border>] to use
/// [Border.lerp].
///
/// See [Tween] for a discussion on how to use interpolation objects.
class BorderTween extends Tween<Border?> {
  /// Creates a [Border] tween.
  ///
  /// The [begin] and [end] properties may be null; the null value
  /// is treated as having no border.
  BorderTween({super.begin, super.end});

  /// Returns the value this variable has at the given animation clock value.
  @override
  Border? lerp(double t) => Border.lerp(begin, end, t);
}

/// An interpolation between two [Matrix4]s.
///
/// This class specializes the interpolation of [Tween<Matrix4>] to be
/// appropriate for transformation matrices.
///
/// Currently this class works only for translations.
///
/// See [Tween] for a discussion on how to use interpolation objects.
class Matrix4Tween extends Tween<Matrix4> {
  /// Creates a [Matrix4] tween.
  ///
  /// The [begin] and [end] properties must be non-null before the tween is
  /// first used, but the arguments can be null if the values are going to be
  /// filled in later.
  Matrix4Tween({super.begin, super.end});

  @override
  Matrix4 lerp(double t) {
    assert(begin != null);
    assert(end != null);
    final Vector3 beginTranslation = Vector3.zero();
    final Vector3 endTranslation = Vector3.zero();
    final Quaternion beginRotation = Quaternion.identity();
    final Quaternion endRotation = Quaternion.identity();
    final Vector3 beginScale = Vector3.zero();
    final Vector3 endScale = Vector3.zero();
    begin!.decompose(beginTranslation, beginRotation, beginScale);
    end!.decompose(endTranslation, endRotation, endScale);
    final Vector3 lerpTranslation = beginTranslation * (1.0 - t) + endTranslation * t;
    // TODO(alangardner): Implement lerp for constant rotation
    final Quaternion lerpRotation =
        (beginRotation.scaled(1.0 - t) + endRotation.scaled(t)).normalized();
    final Vector3 lerpScale = beginScale * (1.0 - t) + endScale * t;
    return Matrix4.compose(lerpTranslation, lerpRotation, lerpScale);
  }
}

/// An interpolation between two [TextStyle]s.
///
/// This class specializes the interpolation of [Tween<TextStyle>] to use
/// [TextStyle.lerp].
///
/// This will not work well if the styles don't set the same fields.
///
/// See [Tween] for a discussion on how to use interpolation objects.
class TextStyleTween extends Tween<TextStyle> {
  /// Creates a text style tween.
  ///
  /// The [begin] and [end] properties must be non-null before the tween is
  /// first used, but the arguments can be null if the values are going to be
  /// filled in later.
  TextStyleTween({super.begin, super.end});

  /// Returns the value this variable has at the given animation clock value.
  @override
  TextStyle lerp(double t) => TextStyle.lerp(begin, end, t)!;
}

/// An abstract class for building widgets that animate changes to their
/// properties.
///
/// Widgets of this type will not animate when they are first added to the
/// widget tree. Rather, when they are rebuilt with different values, they will
/// respond to those _changes_ by animating the changes over a specified
/// [duration].
///
/// Which properties are animated is left up to the subclass. Subclasses' [State]s
/// must extend [ImplicitlyAnimatedWidgetState] and provide a way to visit the
/// relevant fields to animate.
///
/// ## Relationship to [AnimatedWidget]s
///
/// [ImplicitlyAnimatedWidget]s (and their subclasses) automatically animate
/// changes in their properties whenever they change. For this,
/// they create and manage their own internal [AnimationController]s to power
/// the animation. While these widgets are simple to use and don't require you
/// to manually manage the lifecycle of an [AnimationController], they
/// are also somewhat limited: Besides the target value for the animated
/// property, developers can only choose a [duration] and [curve] for the
/// animation. If you require more control over the animation (e.g. you want
/// to stop it somewhere in the middle), consider using an
/// [AnimatedWidget] or one of its subclasses. These widgets take an [Animation]
/// as an argument to power the animation. This gives the developer full control
/// over the animation at the cost of requiring you to manually manage the
/// underlying [AnimationController].
///
/// ## Common implicitly animated widgets
///
/// A number of implicitly animated widgets ship with the framework. They are
/// usually named `AnimatedFoo`, where `Foo` is the name of the non-animated
/// version of that widget. Commonly used implicitly animated widgets include:
///
///  * [TweenAnimationBuilder], which animates any property expressed by
///    a [Tween] to a specified target value.
///  * [AnimatedAlign], which is an implicitly animated version of [Align].
///  * [AnimatedContainer], which is an implicitly animated version of
///    [Container].
///  * [AnimatedDefaultTextStyle], which is an implicitly animated version of
///    [DefaultTextStyle].
///  * [AnimatedScale], which is an implicitly animated version of [Transform.scale].
///  * [AnimatedRotation], which is an implicitly animated version of [Transform.rotate].
///  * [AnimatedSlide], which implicitly animates the position of a widget relative to its normal position.
///  * [AnimatedOpacity], which is an implicitly animated version of [Opacity].
///  * [AnimatedPadding], which is an implicitly animated version of [Padding].
///  * [AnimatedPhysicalModel], which is an implicitly animated version of
///    [PhysicalModel].
///  * [AnimatedPositioned], which is an implicitly animated version of
///    [Positioned].
///  * [AnimatedPositionedDirectional], which is an implicitly animated version
///    of [PositionedDirectional].
///  * [AnimatedTheme], which is an implicitly animated version of [Theme].
///  * [AnimatedCrossFade], which cross-fades between two given children and
///    animates itself between their sizes.
///  * [AnimatedSize], which automatically transitions its size over a given
///    duration.
///  * [AnimatedSwitcher], which fades from one widget to another.
abstract class ImplicitlyAnimatedWidget extends StatefulWidget {
  /// Initializes fields for subclasses.
  const ImplicitlyAnimatedWidget({
    super.key,
    this.curve = Curves.linear,
    required this.duration,
    this.onEnd,
  });

  /// The curve to apply when animating the parameters of this container.
  final Curve curve;

  /// The duration over which to animate the parameters of this container.
  final Duration duration;

  /// Called every time an animation completes.
  ///
  /// This can be useful to trigger additional actions (e.g. another animation)
  /// at the end of the current animation.
  final VoidCallback? onEnd;

  @override
  ImplicitlyAnimatedWidgetState<ImplicitlyAnimatedWidget> createState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(IntProperty('duration', duration.inMilliseconds, unit: 'ms'));
  }
}

/// Signature for a [Tween] factory.
///
/// This is the type of one of the arguments of [TweenVisitor], the signature
/// used by [AnimatedWidgetBaseState.forEachTween].
///
/// Instances of this function are expected to take a value and return a tween
/// beginning at that value.
typedef TweenConstructor<T extends Object> = Tween<T> Function(T targetValue);

/// Signature for callbacks passed to [ImplicitlyAnimatedWidgetState.forEachTween].
///
/// {@template flutter.widgets.TweenVisitor.arguments}
/// The `tween` argument should contain the current tween value. This will
/// initially be null when the state is first initialized.
///
/// The `targetValue` argument should contain the value toward which the state
/// is animating. For instance, if the state is animating its widget's
/// opacity value, then this argument should contain the widget's current
/// opacity value.
///
/// The `constructor` argument should contain a function that takes a value
/// (the widget's value being animated) and returns a tween beginning at that
/// value.
///
/// {@endtemplate}
///
/// `forEachTween()` is expected to update its tween value to the return value
/// of this visitor.
///
/// The `<T>` parameter specifies the type of value that's being animated.
typedef TweenVisitor<T extends Object> =
    Tween<T>? Function(Tween<T>? tween, T targetValue, TweenConstructor<T> constructor);

/// A base class for the `State` of widgets with implicit animations.
///
/// [ImplicitlyAnimatedWidgetState] requires that subclasses respond to the
/// animation themselves. If you would like `setState()` to be called
/// automatically as the animation changes, use [AnimatedWidgetBaseState].
///
/// Properties that subclasses choose to animate are represented by [Tween]
/// instances. Subclasses must implement the [forEachTween] method to allow
/// [ImplicitlyAnimatedWidgetState] to iterate through the widget's fields and
/// animate them.
abstract class ImplicitlyAnimatedWidgetState<T extends ImplicitlyAnimatedWidget> extends State<T>
    with SingleTickerProviderStateMixin<T> {
  /// The animation controller driving this widget's implicit animations.
  @protected
  late final AnimationController controller = AnimationController(
    duration: widget.duration,
    debugLabel: kDebugMode ? widget.toStringShort() : null,
    vsync: this,
  );

  /// The animation driving this widget's implicit animations.
  Animation<double> get animation => _animation;
  late CurvedAnimation _animation = _createCurve();

  @protected
  @override
  void initState() {
    super.initState();
    controller.addStatusListener((AnimationStatus status) {
      if (status.isCompleted) {
        widget.onEnd?.call();
      }
    });
    _constructTweens();
    didUpdateTweens();
  }

  @protected
  @override
  void didUpdateWidget(T oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.curve != oldWidget.curve) {
      _animation.dispose();
      _animation = _createCurve();
    }
    controller.duration = widget.duration;
    if (_constructTweens()) {
      forEachTween((
        Tween<dynamic>? tween,
        dynamic targetValue,
        TweenConstructor<dynamic> constructor,
      ) {
        return tween
          ?..begin = tween.evaluate(_animation)
          ..end = targetValue;
      });
      controller.forward(from: 0.0);
      didUpdateTweens();
    }
  }

  CurvedAnimation _createCurve() {
    return CurvedAnimation(parent: controller, curve: widget.curve);
  }

  @protected
  @override
  void dispose() {
    _animation.dispose();
    controller.dispose();
    super.dispose();
  }

  bool _constructTweens() {
    bool shouldStartAnimation = false;
    forEachTween((
      Tween<dynamic>? tween,
      dynamic targetValue,
      TweenConstructor<dynamic> constructor,
    ) {
      if (targetValue != null) {
        tween ??= constructor(targetValue);
        if (targetValue != (tween.end ?? tween.begin)) {
          shouldStartAnimation = true;
        } else {
          tween.end ??= tween.begin;
        }
      } else {
        tween = null;
      }
      return tween;
    });
    return shouldStartAnimation;
  }

  /// Visits each tween controlled by this state with the specified `visitor`
  /// function.
  ///
  /// ### Subclass responsibility
  ///
  /// Properties to be animated are represented by [Tween] member variables in
  /// the state. For each such tween, [forEachTween] implementations are
  /// expected to call `visitor` with the appropriate arguments and store the
  /// result back into the member variable. The arguments to `visitor` are as
  /// follows:
  ///
  /// {@macro flutter.widgets.TweenVisitor.arguments}
  ///
  /// ### When this method will be called
  ///
  /// [forEachTween] is initially called during [initState]. It is expected that
  /// the visitor's `tween` argument will be set to null, causing the visitor to
  /// call its `constructor` argument to construct the tween for the first time.
  /// The resulting tween will have its `begin` value set to the target value
  /// and will have its `end` value set to null. The animation will not be
  /// started.
  ///
  /// When this state's [widget] is updated (thus triggering the
  /// [didUpdateWidget] method to be called), [forEachTween] will be called
  /// again to check if the target value has changed. If the target value has
  /// changed, signaling that the [animation] should start, then the visitor
  /// will update the tween's `start` and `end` values accordingly, and the
  /// animation will be started.
  ///
  /// ### Other member variables
  ///
  /// Subclasses that contain properties based on tweens created by
  /// [forEachTween] should override [didUpdateTweens] to update those
  /// properties. Dependent properties should not be updated within
  /// [forEachTween].
  ///
  /// {@tool snippet}
  ///
  /// This sample implements an implicitly animated widget's `State`.
  /// The widget animates between colors whenever `widget.targetColor`
  /// changes.
  ///
  /// ```dart
  /// class MyWidgetState extends AnimatedWidgetBaseState<MyWidget> {
  ///   ColorTween? _colorTween;
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return Text(
  ///       'Hello World',
  ///       // Computes the value of the text color at any given time.
  ///       style: TextStyle(color: _colorTween?.evaluate(animation)),
  ///     );
  ///   }
  ///
  ///   @override
  ///   void forEachTween(TweenVisitor<dynamic> visitor) {
  ///     // Update the tween using the provided visitor function.
  ///     _colorTween = visitor(
  ///       // The latest tween value. Can be `null`.
  ///       _colorTween,
  ///       // The color value toward which we are animating.
  ///       widget.targetColor,
  ///       // A function that takes a color value and returns a tween
  ///       // beginning at that value.
  ///       (dynamic value) => ColorTween(begin: value as Color?),
  ///     ) as ColorTween?;
  ///
  ///     // We could have more tweens than one by using the visitor
  ///     // multiple times.
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  @protected
  void forEachTween(TweenVisitor<dynamic> visitor);

  /// Optional hook for subclasses that runs after all tweens have been updated
  /// via [forEachTween].
  ///
  /// Any properties that depend upon tweens created by [forEachTween] should be
  /// updated within [didUpdateTweens], not within [forEachTween].
  ///
  /// This method will be called both:
  ///
  ///  1. After the tweens are _initially_ constructed (by
  ///     the `constructor` argument to the [TweenVisitor] that's passed to
  ///     [forEachTween]). In this case, the tweens are likely to contain only
  ///     a [Tween.begin] value and not a [Tween.end].
  ///
  ///  2. When the state's [widget] is updated, and one or more of the tweens
  ///     visited by [forEachTween] specifies a target value that's different
  ///     than the widget's current value, thus signaling that the [animation]
  ///     should run. In this case, the [Tween.begin] value for each tween will
  ///     an evaluation of the tween against the current [animation], and the
  ///     [Tween.end] value for each tween will be the target value.
  @protected
  void didUpdateTweens() {}
}

/// A base class for widgets with implicit animations that need to rebuild their
/// widget tree as the animation runs.
///
/// This class calls [build] each frame that the animation ticks. For a
/// variant that does not rebuild each frame, consider subclassing
/// [ImplicitlyAnimatedWidgetState] directly.
///
/// Subclasses must implement the [forEachTween] method to allow
/// [AnimatedWidgetBaseState] to iterate through the subclasses' widget's fields
/// and animate them.
abstract class AnimatedWidgetBaseState<T extends ImplicitlyAnimatedWidget>
    extends ImplicitlyAnimatedWidgetState<T> {
  @protected
  @override
  void initState() {
    super.initState();
    controller.addListener(_handleAnimationChanged);
  }

  void _handleAnimationChanged() {
    setState(() {
      /* The animation ticked. Rebuild with new animation value */
    });
  }
}

/// Animated version of [Container] that gradually changes its values over a period of time.
///
/// The [AnimatedContainer] will automatically animate between the old and
/// new values of properties when they change using the provided curve and
/// duration. Properties that are null are not animated. Its child and
/// descendants are not animated.
///
/// This class is useful for generating simple implicit transitions between
/// different parameters to [Container] with its internal [AnimationController].
/// For more complex animations, you'll likely want to use a subclass of
/// [AnimatedWidget] such as the [DecoratedBoxTransition] or use your own
/// [AnimationController].
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=yI-8QHpGIP4}
///
/// {@tool dartpad}
/// The following example (depicted above) transitions an AnimatedContainer
/// between two states. It adjusts the `height`, `width`, `color`, and
/// [alignment] properties when tapped.
///
/// ** See code in examples/api/lib/widgets/implicit_animations/animated_container.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [AnimatedPadding], which is a subset of this widget that only
///    supports animating the [padding].
///  * The [catalog of layout widgets](https://flutter.dev/widgets/layout/).
///  * [AnimatedPositioned], which, as a child of a [Stack], automatically
///    transitions its child's position over a given duration whenever the given
///    position changes.
///  * [AnimatedAlign], which automatically transitions its child's
///    position over a given duration whenever the given [AnimatedAlign.alignment] changes.
///  * [AnimatedSwitcher], which switches out a child for a new one with a customizable transition.
///  * [AnimatedCrossFade], which fades between two children and interpolates their sizes.
class AnimatedContainer extends ImplicitlyAnimatedWidget {
  /// Creates a container that animates its parameters implicitly.
  AnimatedContainer({
    super.key,
    this.alignment,
    this.padding,
    Color? color,
    Decoration? decoration,
    this.foregroundDecoration,
    double? width,
    double? height,
    BoxConstraints? constraints,
    this.margin,
    this.transform,
    this.transformAlignment,
    this.child,
    this.clipBehavior = Clip.none,
    super.curve,
    required super.duration,
    super.onEnd,
  }) : assert(margin == null || margin.isNonNegative),
       assert(padding == null || padding.isNonNegative),
       assert(decoration == null || decoration.debugAssertIsValid()),
       assert(constraints == null || constraints.debugAssertIsValid()),
       assert(
         color == null || decoration == null,
         'Cannot provide both a color and a decoration\n'
         'The color argument is just a shorthand for "decoration: BoxDecoration(color: color)".',
       ),
       decoration = decoration ?? (color != null ? BoxDecoration(color: color) : null),
       constraints =
           (width != null || height != null)
               ? constraints?.tighten(width: width, height: height) ??
                   BoxConstraints.tightFor(width: width, height: height)
               : constraints;

  /// The [child] contained by the container.
  ///
  /// If null, and if the [constraints] are unbounded or also null, the
  /// container will expand to fill all available space in its parent, unless
  /// the parent provides unbounded constraints, in which case the container
  /// will attempt to be as small as possible.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// Align the [child] within the container.
  ///
  /// If non-null, the container will expand to fill its parent and position its
  /// child within itself according to the given value. If the incoming
  /// constraints are unbounded, then the child will be shrink-wrapped instead.
  ///
  /// Ignored if [child] is null.
  ///
  /// See also:
  ///
  ///  * [Alignment], a class with convenient constants typically used to
  ///    specify an [AlignmentGeometry].
  ///  * [AlignmentDirectional], like [Alignment] for specifying alignments
  ///    relative to text direction.
  final AlignmentGeometry? alignment;

  /// Empty space to inscribe inside the [decoration]. The [child], if any, is
  /// placed inside this padding.
  final EdgeInsetsGeometry? padding;

  /// The decoration to paint behind the [child].
  ///
  /// A shorthand for specifying just a solid color is available in the
  /// constructor: set the `color` argument instead of the `decoration`
  /// argument.
  final Decoration? decoration;

  /// The decoration to paint in front of the child.
  final Decoration? foregroundDecoration;

  /// Additional constraints to apply to the child.
  ///
  /// The constructor `width` and `height` arguments are combined with the
  /// `constraints` argument to set this property.
  ///
  /// The [padding] goes inside the constraints.
  final BoxConstraints? constraints;

  /// Empty space to surround the [decoration] and [child].
  final EdgeInsetsGeometry? margin;

  /// The transformation matrix to apply before painting the container.
  final Matrix4? transform;

  /// The alignment of the origin, relative to the size of the container, if [transform] is specified.
  ///
  /// When [transform] is null, the value of this property is ignored.
  ///
  /// See also:
  ///
  ///  * [Transform.alignment], which is set by this property.
  final AlignmentGeometry? transformAlignment;

  /// The clip behavior when [AnimatedContainer.decoration] is not null.
  ///
  /// Defaults to [Clip.none]. Must be [Clip.none] if [decoration] is null.
  ///
  /// Unlike other properties of [AnimatedContainer], changes to this property
  /// apply immediately and have no animation.
  ///
  /// If a clip is to be applied, the [Decoration.getClipPath] method
  /// for the provided decoration must return a clip path. (This is not
  /// supported by all decorations; the default implementation of that
  /// method throws an [UnsupportedError].)
  final Clip clipBehavior;

  @override
  AnimatedWidgetBaseState<AnimatedContainer> createState() => _AnimatedContainerState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<AlignmentGeometry>(
        'alignment',
        alignment,
        showName: false,
        defaultValue: null,
      ),
    );
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding, defaultValue: null));
    properties.add(DiagnosticsProperty<Decoration>('bg', decoration, defaultValue: null));
    properties.add(DiagnosticsProperty<Decoration>('fg', foregroundDecoration, defaultValue: null));
    properties.add(
      DiagnosticsProperty<BoxConstraints>(
        'constraints',
        constraints,
        defaultValue: null,
        showName: false,
      ),
    );
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('margin', margin, defaultValue: null));
    properties.add(ObjectFlagProperty<Matrix4>.has('transform', transform));
    properties.add(
      DiagnosticsProperty<AlignmentGeometry>(
        'transformAlignment',
        transformAlignment,
        defaultValue: null,
      ),
    );
    properties.add(DiagnosticsProperty<Clip>('clipBehavior', clipBehavior));
  }
}

class _AnimatedContainerState extends AnimatedWidgetBaseState<AnimatedContainer> {
  AlignmentGeometryTween? _alignment;
  EdgeInsetsGeometryTween? _padding;
  DecorationTween? _decoration;
  DecorationTween? _foregroundDecoration;
  BoxConstraintsTween? _constraints;
  EdgeInsetsGeometryTween? _margin;
  Matrix4Tween? _transform;
  AlignmentGeometryTween? _transformAlignment;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _alignment =
        visitor(
              _alignment,
              widget.alignment,
              (dynamic value) => AlignmentGeometryTween(begin: value as AlignmentGeometry),
            )
            as AlignmentGeometryTween?;
    _padding =
        visitor(
              _padding,
              widget.padding,
              (dynamic value) => EdgeInsetsGeometryTween(begin: value as EdgeInsetsGeometry),
            )
            as EdgeInsetsGeometryTween?;
    _decoration =
        visitor(
              _decoration,
              widget.decoration,
              (dynamic value) => DecorationTween(begin: value as Decoration),
            )
            as DecorationTween?;
    _foregroundDecoration =
        visitor(
              _foregroundDecoration,
              widget.foregroundDecoration,
              (dynamic value) => DecorationTween(begin: value as Decoration),
            )
            as DecorationTween?;
    _constraints =
        visitor(
              _constraints,
              widget.constraints,
              (dynamic value) => BoxConstraintsTween(begin: value as BoxConstraints),
            )
            as BoxConstraintsTween?;
    _margin =
        visitor(
              _margin,
              widget.margin,
              (dynamic value) => EdgeInsetsGeometryTween(begin: value as EdgeInsetsGeometry),
            )
            as EdgeInsetsGeometryTween?;
    _transform =
        visitor(
              _transform,
              widget.transform,
              (dynamic value) => Matrix4Tween(begin: value as Matrix4),
            )
            as Matrix4Tween?;
    _transformAlignment =
        visitor(
              _transformAlignment,
              widget.transformAlignment,
              (dynamic value) => AlignmentGeometryTween(begin: value as AlignmentGeometry),
            )
            as AlignmentGeometryTween?;
  }

  @override
  Widget build(BuildContext context) {
    final Animation<double> animation = this.animation;
    return Container(
      alignment: _alignment?.evaluate(animation),
      padding: _padding?.evaluate(animation),
      decoration: _decoration?.evaluate(animation),
      foregroundDecoration: _foregroundDecoration?.evaluate(animation),
      constraints: _constraints?.evaluate(animation),
      margin: _margin?.evaluate(animation),
      transform: _transform?.evaluate(animation),
      transformAlignment: _transformAlignment?.evaluate(animation),
      clipBehavior: widget.clipBehavior,
      child: widget.child,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(
      DiagnosticsProperty<AlignmentGeometryTween>(
        'alignment',
        _alignment,
        showName: false,
        defaultValue: null,
      ),
    );
    description.add(
      DiagnosticsProperty<EdgeInsetsGeometryTween>('padding', _padding, defaultValue: null),
    );
    description.add(DiagnosticsProperty<DecorationTween>('bg', _decoration, defaultValue: null));
    description.add(
      DiagnosticsProperty<DecorationTween>('fg', _foregroundDecoration, defaultValue: null),
    );
    description.add(
      DiagnosticsProperty<BoxConstraintsTween>(
        'constraints',
        _constraints,
        showName: false,
        defaultValue: null,
      ),
    );
    description.add(
      DiagnosticsProperty<EdgeInsetsGeometryTween>('margin', _margin, defaultValue: null),
    );
    description.add(ObjectFlagProperty<Matrix4Tween>.has('transform', _transform));
    description.add(
      DiagnosticsProperty<AlignmentGeometryTween>(
        'transformAlignment',
        _transformAlignment,
        defaultValue: null,
      ),
    );
  }
}

/// Animated version of [Padding] which automatically transitions the
/// indentation over a given duration whenever the given inset changes.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=PY2m0fhGNz4}
///
/// Here's an illustration of what using this widget looks like, using a [curve]
/// of [Curves.fastOutSlowIn].
/// {@animation 250 266 https://flutter.github.io/assets-for-api-docs/assets/widgets/animated_padding.mp4}
///
/// {@tool dartpad}
/// The following code implements the [AnimatedPadding] widget, using a [curve] of
/// [Curves.easeInOut].
///
/// ** See code in examples/api/lib/widgets/implicit_animations/animated_padding.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [AnimatedContainer], which can transition more values at once.
///  * [AnimatedAlign], which automatically transitions its child's
///    position over a given duration whenever the given
///    [AnimatedAlign.alignment] changes.
class AnimatedPadding extends ImplicitlyAnimatedWidget {
  /// Creates a widget that insets its child by a value that animates
  /// implicitly.
  AnimatedPadding({
    super.key,
    required this.padding,
    this.child,
    super.curve,
    required super.duration,
    super.onEnd,
  }) : assert(padding.isNonNegative);

  /// The amount of space by which to inset the child.
  final EdgeInsetsGeometry padding;

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  @override
  AnimatedWidgetBaseState<AnimatedPadding> createState() => _AnimatedPaddingState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding));
  }
}

class _AnimatedPaddingState extends AnimatedWidgetBaseState<AnimatedPadding> {
  EdgeInsetsGeometryTween? _padding;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _padding =
        visitor(
              _padding,
              widget.padding,
              (dynamic value) => EdgeInsetsGeometryTween(begin: value as EdgeInsetsGeometry),
            )
            as EdgeInsetsGeometryTween?;
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: _padding!.evaluate(animation).clamp(EdgeInsets.zero, EdgeInsetsGeometry.infinity),
      child: widget.child,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(
      DiagnosticsProperty<EdgeInsetsGeometryTween>('padding', _padding, defaultValue: null),
    );
  }
}

/// Animated version of [Align] which automatically transitions the child's
/// position over a given duration whenever the given [alignment] changes.
///
/// Here's an illustration of what this can look like, using a [curve] of
/// [Curves.fastOutSlowIn].
/// {@animation 250 266 https://flutter.github.io/assets-for-api-docs/assets/widgets/animated_align.mp4}
///
/// For the animation, you can choose a [curve] as well as a [duration] and the
/// widget will automatically animate to the new target [alignment]. If you require
/// more control over the animation (e.g. if you want to stop it mid-animation),
/// consider using an [AlignTransition] instead, which takes a provided
/// [Animation] as argument. While that allows you to fine-tune the animation,
/// it also requires more development overhead as you have to manually manage
/// the lifecycle of the underlying [AnimationController].
///
/// {@tool dartpad}
/// The following code implements the [AnimatedAlign] widget, using a [curve] of
/// [Curves.fastOutSlowIn].
///
/// ** See code in examples/api/lib/widgets/implicit_animations/animated_align.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [AnimatedContainer], which can transition more values at once.
///  * [AnimatedPadding], which can animate the padding instead of the
///    alignment.
///  * [AnimatedSlide], which can animate the translation of child by a given offset relative to its size.
///  * [AnimatedPositioned], which, as a child of a [Stack], automatically
///    transitions its child's position over a given duration whenever the given
///    position changes.
class AnimatedAlign extends ImplicitlyAnimatedWidget {
  /// Creates a widget that positions its child by an alignment that animates
  /// implicitly.
  const AnimatedAlign({
    super.key,
    required this.alignment,
    this.child,
    this.heightFactor,
    this.widthFactor,
    super.curve,
    required super.duration,
    super.onEnd,
  }) : assert(widthFactor == null || widthFactor >= 0.0),
       assert(heightFactor == null || heightFactor >= 0.0);

  /// How to align the child.
  ///
  /// The x and y values of the [Alignment] control the horizontal and vertical
  /// alignment, respectively. An x value of -1.0 means that the left edge of
  /// the child is aligned with the left edge of the parent whereas an x value
  /// of 1.0 means that the right edge of the child is aligned with the right
  /// edge of the parent. Other values interpolate (and extrapolate) linearly.
  /// For example, a value of 0.0 means that the center of the child is aligned
  /// with the center of the parent.
  ///
  /// See also:
  ///
  ///  * [Alignment], which has more details and some convenience constants for
  ///    common positions.
  ///  * [AlignmentDirectional], which has a horizontal coordinate orientation
  ///    that depends on the [TextDirection].
  final AlignmentGeometry alignment;

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// If non-null, sets its height to the child's height multiplied by this factor.
  ///
  /// Must be greater than or equal to 0.0, defaults to null.
  final double? heightFactor;

  /// If non-null, sets its width to the child's width multiplied by this factor.
  ///
  /// Must be greater than or equal to 0.0, defaults to null.
  final double? widthFactor;

  @override
  AnimatedWidgetBaseState<AnimatedAlign> createState() => _AnimatedAlignState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<AlignmentGeometry>('alignment', alignment));
  }
}

class _AnimatedAlignState extends AnimatedWidgetBaseState<AnimatedAlign> {
  AlignmentGeometryTween? _alignment;
  Tween<double>? _heightFactorTween;
  Tween<double>? _widthFactorTween;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _alignment =
        visitor(
              _alignment,
              widget.alignment,
              (dynamic value) => AlignmentGeometryTween(begin: value as AlignmentGeometry),
            )
            as AlignmentGeometryTween?;
    if (widget.heightFactor != null) {
      _heightFactorTween =
          visitor(
                _heightFactorTween,
                widget.heightFactor,
                (dynamic value) => Tween<double>(begin: value as double),
              )
              as Tween<double>?;
    }
    if (widget.widthFactor != null) {
      _widthFactorTween =
          visitor(
                _widthFactorTween,
                widget.widthFactor,
                (dynamic value) => Tween<double>(begin: value as double),
              )
              as Tween<double>?;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Align(
      alignment: _alignment!.evaluate(animation)!,
      heightFactor: _heightFactorTween?.evaluate(animation),
      widthFactor: _widthFactorTween?.evaluate(animation),
      child: widget.child,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(
      DiagnosticsProperty<AlignmentGeometryTween>('alignment', _alignment, defaultValue: null),
    );
    description.add(
      DiagnosticsProperty<Tween<double>>('widthFactor', _widthFactorTween, defaultValue: null),
    );
    description.add(
      DiagnosticsProperty<Tween<double>>('heightFactor', _heightFactorTween, defaultValue: null),
    );
  }
}

/// Animated version of [Positioned] which automatically transitions the child's
/// position over a given duration whenever the given position changes.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=hC3s2YdtWt8}
///
/// Only works if it's the child of a [Stack].
///
/// This widget is a good choice if the _size_ of the child would end up
/// changing as a result of this animation. If the size is intended to remain
/// the same, with only the _position_ changing over time, then consider
/// [SlideTransition] instead. [SlideTransition] only triggers a repaint each
/// frame of the animation, whereas [AnimatedPositioned] will trigger a relayout
/// as well.
///
/// Here's an illustration of what using this widget looks like, using a [curve]
/// of [Curves.fastOutSlowIn].
/// {@animation 250 266 https://flutter.github.io/assets-for-api-docs/assets/widgets/animated_positioned.mp4}
///
/// For the animation, you can choose a [curve] as well as a [duration] and the
/// widget will automatically animate to the new target position. If you require
/// more control over the animation (e.g. if you want to stop it mid-animation),
/// consider using a [PositionedTransition] instead, which takes a provided
/// [Animation] as an argument. While that allows you to fine-tune the animation,
/// it also requires more development overhead as you have to manually manage
/// the lifecycle of the underlying [AnimationController].
///
/// {@tool dartpad}
/// The following example transitions an AnimatedPositioned
/// between two states. It adjusts the `height`, `width`, and
/// [Positioned] properties when tapped.
///
/// ** See code in examples/api/lib/widgets/implicit_animations/animated_positioned.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [AnimatedPositionedDirectional], which adapts to the ambient
///    [Directionality] (the same as this widget, but for animating
///    [PositionedDirectional]).
class AnimatedPositioned extends ImplicitlyAnimatedWidget {
  /// Creates a widget that animates its position implicitly.
  ///
  /// Only two out of the three horizontal values ([left], [right],
  /// [width]), and only two out of the three vertical values ([top],
  /// [bottom], [height]), can be set. In each case, at least one of
  /// the three must be null.
  const AnimatedPositioned({
    super.key,
    required this.child,
    this.left,
    this.top,
    this.right,
    this.bottom,
    this.width,
    this.height,
    super.curve,
    required super.duration,
    super.onEnd,
  }) : assert(left == null || right == null || width == null),
       assert(top == null || bottom == null || height == null);

  /// Creates a widget that animates the rectangle it occupies implicitly.
  AnimatedPositioned.fromRect({
    super.key,
    required this.child,
    required Rect rect,
    super.curve,
    required super.duration,
    super.onEnd,
  }) : left = rect.left,
       top = rect.top,
       width = rect.width,
       height = rect.height,
       right = null,
       bottom = null;

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// The offset of the child's left edge from the left of the stack.
  final double? left;

  /// The offset of the child's top edge from the top of the stack.
  final double? top;

  /// The offset of the child's right edge from the right of the stack.
  final double? right;

  /// The offset of the child's bottom edge from the bottom of the stack.
  final double? bottom;

  /// The child's width.
  ///
  /// Only two out of the three horizontal values ([left], [right], [width]) can
  /// be set. The third must be null.
  final double? width;

  /// The child's height.
  ///
  /// Only two out of the three vertical values ([top], [bottom], [height]) can
  /// be set. The third must be null.
  final double? height;

  @override
  AnimatedWidgetBaseState<AnimatedPositioned> createState() => _AnimatedPositionedState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('left', left, defaultValue: null));
    properties.add(DoubleProperty('top', top, defaultValue: null));
    properties.add(DoubleProperty('right', right, defaultValue: null));
    properties.add(DoubleProperty('bottom', bottom, defaultValue: null));
    properties.add(DoubleProperty('width', width, defaultValue: null));
    properties.add(DoubleProperty('height', height, defaultValue: null));
  }
}

class _AnimatedPositionedState extends AnimatedWidgetBaseState<AnimatedPositioned> {
  Tween<double>? _left;
  Tween<double>? _top;
  Tween<double>? _right;
  Tween<double>? _bottom;
  Tween<double>? _width;
  Tween<double>? _height;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _left =
        visitor(_left, widget.left, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
    _top =
        visitor(_top, widget.top, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
    _right =
        visitor(_right, widget.right, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
    _bottom =
        visitor(_bottom, widget.bottom, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
    _width =
        visitor(_width, widget.width, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
    _height =
        visitor(_height, widget.height, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
  }

  @override
  Widget build(BuildContext context) {
    return Positioned(
      left: _left?.evaluate(animation),
      top: _top?.evaluate(animation),
      right: _right?.evaluate(animation),
      bottom: _bottom?.evaluate(animation),
      width: _width?.evaluate(animation),
      height: _height?.evaluate(animation),
      child: widget.child,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(ObjectFlagProperty<Tween<double>>.has('left', _left));
    description.add(ObjectFlagProperty<Tween<double>>.has('top', _top));
    description.add(ObjectFlagProperty<Tween<double>>.has('right', _right));
    description.add(ObjectFlagProperty<Tween<double>>.has('bottom', _bottom));
    description.add(ObjectFlagProperty<Tween<double>>.has('width', _width));
    description.add(ObjectFlagProperty<Tween<double>>.has('height', _height));
  }
}

/// Animated version of [PositionedDirectional] which automatically transitions
/// the child's position over a given duration whenever the given position
/// changes.
///
/// The ambient [Directionality] is used to determine whether [start] is to the
/// left or to the right.
///
/// Only works if it's the child of a [Stack].
///
/// This widget is a good choice if the _size_ of the child would end up
/// changing as a result of this animation. If the size is intended to remain
/// the same, with only the _position_ changing over time, then consider
/// [SlideTransition] instead. [SlideTransition] only triggers a repaint each
/// frame of the animation, whereas [AnimatedPositionedDirectional] will trigger
/// a relayout as well. ([SlideTransition] is also text-direction-aware.)
///
/// Here's an illustration of what using this widget looks like, using a [curve]
/// of [Curves.fastOutSlowIn].
/// {@animation 250 266 https://flutter.github.io/assets-for-api-docs/assets/widgets/animated_positioned_directional.mp4}
///
/// See also:
///
///  * [AnimatedPositioned], which specifies the widget's position visually (the
///    same as this widget, but for animating [Positioned]).
class AnimatedPositionedDirectional extends ImplicitlyAnimatedWidget {
  /// Creates a widget that animates its position implicitly.
  ///
  /// Only two out of the three horizontal values ([start], [end], [width]), and
  /// only two out of the three vertical values ([top], [bottom], [height]), can
  /// be set. In each case, at least one of the three must be null.
  const AnimatedPositionedDirectional({
    super.key,
    required this.child,
    this.start,
    this.top,
    this.end,
    this.bottom,
    this.width,
    this.height,
    super.curve,
    required super.duration,
    super.onEnd,
  }) : assert(start == null || end == null || width == null),
       assert(top == null || bottom == null || height == null);

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// The offset of the child's start edge from the start of the stack.
  final double? start;

  /// The offset of the child's top edge from the top of the stack.
  final double? top;

  /// The offset of the child's end edge from the end of the stack.
  final double? end;

  /// The offset of the child's bottom edge from the bottom of the stack.
  final double? bottom;

  /// The child's width.
  ///
  /// Only two out of the three horizontal values ([start], [end], [width]) can
  /// be set. The third must be null.
  final double? width;

  /// The child's height.
  ///
  /// Only two out of the three vertical values ([top], [bottom], [height]) can
  /// be set. The third must be null.
  final double? height;

  @override
  AnimatedWidgetBaseState<AnimatedPositionedDirectional> createState() =>
      _AnimatedPositionedDirectionalState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('start', start, defaultValue: null));
    properties.add(DoubleProperty('top', top, defaultValue: null));
    properties.add(DoubleProperty('end', end, defaultValue: null));
    properties.add(DoubleProperty('bottom', bottom, defaultValue: null));
    properties.add(DoubleProperty('width', width, defaultValue: null));
    properties.add(DoubleProperty('height', height, defaultValue: null));
  }
}

class _AnimatedPositionedDirectionalState
    extends AnimatedWidgetBaseState<AnimatedPositionedDirectional> {
  Tween<double>? _start;
  Tween<double>? _top;
  Tween<double>? _end;
  Tween<double>? _bottom;
  Tween<double>? _width;
  Tween<double>? _height;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _start =
        visitor(_start, widget.start, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
    _top =
        visitor(_top, widget.top, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
    _end =
        visitor(_end, widget.end, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
    _bottom =
        visitor(_bottom, widget.bottom, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
    _width =
        visitor(_width, widget.width, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
    _height =
        visitor(_height, widget.height, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasDirectionality(context));
    return Positioned.directional(
      textDirection: Directionality.of(context),
      start: _start?.evaluate(animation),
      top: _top?.evaluate(animation),
      end: _end?.evaluate(animation),
      bottom: _bottom?.evaluate(animation),
      width: _width?.evaluate(animation),
      height: _height?.evaluate(animation),
      child: widget.child,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(ObjectFlagProperty<Tween<double>>.has('start', _start));
    description.add(ObjectFlagProperty<Tween<double>>.has('top', _top));
    description.add(ObjectFlagProperty<Tween<double>>.has('end', _end));
    description.add(ObjectFlagProperty<Tween<double>>.has('bottom', _bottom));
    description.add(ObjectFlagProperty<Tween<double>>.has('width', _width));
    description.add(ObjectFlagProperty<Tween<double>>.has('height', _height));
  }
}

/// Animated version of [Transform.scale] which automatically transitions the child's
/// scale over a given duration whenever the given scale changes.
///
/// {@tool snippet}
///
/// This code defines a widget that uses [AnimatedScale] to change the size
/// of [FlutterLogo] gradually to a new scale whenever the button is pressed.
///
/// ```dart
/// class LogoScale extends StatefulWidget {
///   const LogoScale({super.key});
///
///   @override
///   State<LogoScale> createState() => LogoScaleState();
/// }
///
/// class LogoScaleState extends State<LogoScale> {
///   double scale = 1.0;
///
///   void _changeScale() {
///     setState(() => scale = scale == 1.0 ? 3.0 : 1.0);
///   }
///
///   @override
///   Widget build(BuildContext context) {
///     return Column(
///       mainAxisAlignment: MainAxisAlignment.center,
///       children: <Widget>[
///         ElevatedButton(
///           onPressed: _changeScale,
///           child: const Text('Scale Logo'),
///         ),
///         Padding(
///           padding: const EdgeInsets.all(50),
///           child: AnimatedScale(
///             scale: scale,
///             duration: const Duration(seconds: 2),
///             child: const FlutterLogo(),
///           ),
///         ),
///       ],
///     );
///   }
/// }
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [AnimatedRotation], for animating the rotation of a child.
///  * [AnimatedSize], for animating the resize of a child based on changes
///    in layout.
///  * [AnimatedSlide], for animating the translation of a child by a given offset relative to its size.
///  * [ScaleTransition], an explicitly animated version of this widget, where
///    an [Animation] is provided by the caller instead of being built in.
class AnimatedScale extends ImplicitlyAnimatedWidget {
  /// Creates a widget that animates its scale implicitly.
  const AnimatedScale({
    super.key,
    this.child,
    required this.scale,
    this.alignment = Alignment.center,
    this.filterQuality,
    super.curve,
    required super.duration,
    super.onEnd,
  });

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// The target scale.
  final double scale;

  /// The alignment of the origin of the coordinate system in which the scale
  /// takes place, relative to the size of the box.
  ///
  /// For example, to set the origin of the scale to bottom middle, you can use
  /// an alignment of (0.0, 1.0).
  final Alignment alignment;

  /// The filter quality with which to apply the transform as a bitmap operation.
  ///
  /// {@macro flutter.widgets.Transform.optional.FilterQuality}
  final FilterQuality? filterQuality;

  @override
  ImplicitlyAnimatedWidgetState<AnimatedScale> createState() => _AnimatedScaleState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('scale', scale));
    properties.add(
      DiagnosticsProperty<Alignment>('alignment', alignment, defaultValue: Alignment.center),
    );
    properties.add(EnumProperty<FilterQuality>('filterQuality', filterQuality, defaultValue: null));
  }
}

class _AnimatedScaleState extends ImplicitlyAnimatedWidgetState<AnimatedScale> {
  Tween<double>? _scale;
  late Animation<double> _scaleAnimation;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _scale =
        visitor(_scale, widget.scale, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
  }

  @override
  void didUpdateTweens() {
    _scaleAnimation = animation.drive(_scale!);
  }

  @override
  Widget build(BuildContext context) {
    return ScaleTransition(
      scale: _scaleAnimation,
      alignment: widget.alignment,
      filterQuality: widget.filterQuality,
      child: widget.child,
    );
  }
}

/// Animated version of [Transform.rotate] which automatically transitions the child's
/// rotation over a given duration whenever the given rotation changes.
///
/// {@tool snippet}
///
/// This code defines a widget that uses [AnimatedRotation] to rotate a [FlutterLogo]
/// gradually by an eighth of a turn (45 degrees) with each press of the button.
///
/// ```dart
/// class LogoRotate extends StatefulWidget {
///   const LogoRotate({super.key});
///
///   @override
///   State<LogoRotate> createState() => LogoRotateState();
/// }
///
/// class LogoRotateState extends State<LogoRotate> {
///   double turns = 0.0;
///
///   void _changeRotation() {
///     setState(() => turns += 1.0 / 8.0);
///   }
///
///   @override
///   Widget build(BuildContext context) {
///     return Column(
///       mainAxisAlignment: MainAxisAlignment.center,
///       children: <Widget>[
///         ElevatedButton(
///           onPressed: _changeRotation,
///           child: const Text('Rotate Logo'),
///         ),
///         Padding(
///           padding: const EdgeInsets.all(50),
///           child: AnimatedRotation(
///             turns: turns,
///             duration: const Duration(seconds: 1),
///             child: const FlutterLogo(),
///           ),
///         ),
///       ],
///     );
///   }
/// }
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [AnimatedScale], for animating the scale of a child.
///  * [RotationTransition], an explicitly animated version of this widget, where
///    an [Animation] is provided by the caller instead of being built in.
class AnimatedRotation extends ImplicitlyAnimatedWidget {
  /// Creates a widget that animates its rotation implicitly.
  const AnimatedRotation({
    super.key,
    this.child,
    required this.turns,
    this.alignment = Alignment.center,
    this.filterQuality,
    super.curve,
    required super.duration,
    super.onEnd,
  });

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// The animation that controls the rotation of the child.
  ///
  /// If the current value of the turns animation is v, the child will be
  /// rotated v * 2 * pi radians before being painted.
  final double turns;

  /// The alignment of the origin of the coordinate system in which the rotation
  /// takes place, relative to the size of the box.
  ///
  /// For example, to set the origin of the rotation to bottom middle, you can use
  /// an alignment of (0.0, 1.0).
  final Alignment alignment;

  /// The filter quality with which to apply the transform as a bitmap operation.
  ///
  /// {@macro flutter.widgets.Transform.optional.FilterQuality}
  final FilterQuality? filterQuality;

  @override
  ImplicitlyAnimatedWidgetState<AnimatedRotation> createState() => _AnimatedRotationState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('turns', turns));
    properties.add(
      DiagnosticsProperty<Alignment>('alignment', alignment, defaultValue: Alignment.center),
    );
    properties.add(EnumProperty<FilterQuality>('filterQuality', filterQuality, defaultValue: null));
  }
}

class _AnimatedRotationState extends ImplicitlyAnimatedWidgetState<AnimatedRotation> {
  Tween<double>? _turns;
  late Animation<double> _turnsAnimation;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _turns =
        visitor(_turns, widget.turns, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
  }

  @override
  void didUpdateTweens() {
    _turnsAnimation = animation.drive(_turns!);
  }

  @override
  Widget build(BuildContext context) {
    return RotationTransition(
      turns: _turnsAnimation,
      alignment: widget.alignment,
      filterQuality: widget.filterQuality,
      child: widget.child,
    );
  }
}

/// Widget which automatically transitions the child's
/// offset relative to its normal position whenever the given offset changes.
///
/// The translation is expressed as an [Offset] scaled to the child's size. For
/// example, an [Offset] with a `dx` of 0.25 will result in a horizontal
/// translation of one quarter the width of the child.
///
/// {@tool dartpad}
/// This code defines a widget that uses [AnimatedSlide] to translate a [FlutterLogo]
/// up or down and right or left by dragging the X and Y axis sliders.
///
/// ** See code in examples/api/lib/widgets/implicit_animations/animated_slide.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [AnimatedPositioned], which, as a child of a [Stack], automatically
///    transitions its child's position over a given duration whenever the given
///    position changes.
///  * [AnimatedAlign], which automatically transitions its child's
///    position over a given duration whenever the given [AnimatedAlign.alignment] changes.
class AnimatedSlide extends ImplicitlyAnimatedWidget {
  /// Creates a widget that animates its offset translation implicitly.
  const AnimatedSlide({
    super.key,
    this.child,
    required this.offset,
    super.curve,
    required super.duration,
    super.onEnd,
  });

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// The target offset.
  /// The child will be translated horizontally by `width * dx` and vertically by `height * dy`
  final Offset offset;

  @override
  ImplicitlyAnimatedWidgetState<AnimatedSlide> createState() => _AnimatedSlideState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Offset>('offset', offset));
  }
}

class _AnimatedSlideState extends ImplicitlyAnimatedWidgetState<AnimatedSlide> {
  Tween<Offset>? _offset;
  late Animation<Offset> _offsetAnimation;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _offset =
        visitor(_offset, widget.offset, (dynamic value) => Tween<Offset>(begin: value as Offset))
            as Tween<Offset>?;
  }

  @override
  void didUpdateTweens() {
    _offsetAnimation = animation.drive(_offset!);
  }

  @override
  Widget build(BuildContext context) {
    return SlideTransition(position: _offsetAnimation, child: widget.child);
  }
}

/// Animated version of [Opacity] which automatically transitions the child's
/// opacity over a given duration whenever the given opacity changes.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=QZAvjqOqiLY}
///
/// Animating an opacity is relatively expensive because it requires painting
/// the child into an intermediate buffer.
///
/// Here's an illustration of what using this widget looks like, using a [curve]
/// of [Curves.fastOutSlowIn].
/// {@animation 250 266 https://flutter.github.io/assets-for-api-docs/assets/widgets/animated_opacity.mp4}
///
/// {@tool snippet}
///
/// ```dart
/// class LogoFade extends StatefulWidget {
///   const LogoFade({super.key});
///
///   @override
///   State<LogoFade> createState() => LogoFadeState();
/// }
///
/// class LogoFadeState extends State<LogoFade> {
///   double opacityLevel = 1.0;
///
///   void _changeOpacity() {
///     setState(() => opacityLevel = opacityLevel == 0 ? 1.0 : 0.0);
///   }
///
///   @override
///   Widget build(BuildContext context) {
///     return Column(
///       mainAxisAlignment: MainAxisAlignment.center,
///       children: <Widget>[
///         AnimatedOpacity(
///           opacity: opacityLevel,
///           duration: const Duration(seconds: 3),
///           child: const FlutterLogo(),
///         ),
///         ElevatedButton(
///           onPressed: _changeOpacity,
///           child: const Text('Fade Logo'),
///         ),
///       ],
///     );
///   }
/// }
/// ```
/// {@end-tool}
///
/// ## Hit testing
///
/// Setting the [opacity] to zero does not prevent hit testing from being
/// applied to the descendants of the [AnimatedOpacity] widget. This can be
/// confusing for the user, who may not see anything, and may believe the area
/// of the interface where the [AnimatedOpacity] is hiding a widget to be
/// non-interactive.
///
/// With certain widgets, such as [Flow], that compute their positions only when
/// they are painted, this can actually lead to bugs (from unexpected geometry
/// to exceptions), because those widgets are not painted by the [AnimatedOpacity]
/// widget at all when the [opacity] animation reaches zero.
///
/// To avoid such problems, it is generally a good idea to use an
/// [IgnorePointer] widget when setting the [opacity] to zero. This prevents
/// interactions with any children in the subtree when the [child] is animating
/// away.
///
/// See also:
///
///  * [AnimatedCrossFade], for fading between two children.
///  * [AnimatedSwitcher], for fading between many children in sequence.
///  * [FadeTransition], an explicitly animated version of this widget, where
///    an [Animation] is provided by the caller instead of being built in.
///  * [SliverAnimatedOpacity], for automatically transitioning a _sliver's_
///    opacity over a given duration whenever the given opacity changes.
class AnimatedOpacity extends ImplicitlyAnimatedWidget {
  /// Creates a widget that animates its opacity implicitly.
  ///
  /// The [opacity] argument must be between zero and one, inclusive.
  const AnimatedOpacity({
    super.key,
    this.child,
    required this.opacity,
    super.curve,
    required super.duration,
    super.onEnd,
    this.alwaysIncludeSemantics = false,
  }) : assert(opacity >= 0.0 && opacity <= 1.0);

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// The target opacity.
  ///
  /// An opacity of 1.0 is fully opaque. An opacity of 0.0 is fully transparent
  /// (i.e., invisible).
  final double opacity;

  /// Whether the semantic information of the children is always included.
  ///
  /// Defaults to false.
  ///
  /// When true, regardless of the opacity settings the child semantic
  /// information is exposed as if the widget were fully visible. This is
  /// useful in cases where labels may be hidden during animations that
  /// would otherwise contribute relevant semantics.
  final bool alwaysIncludeSemantics;

  @override
  ImplicitlyAnimatedWidgetState<AnimatedOpacity> createState() => _AnimatedOpacityState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('opacity', opacity));
  }
}

class _AnimatedOpacityState extends ImplicitlyAnimatedWidgetState<AnimatedOpacity> {
  Tween<double>? _opacity;
  late Animation<double> _opacityAnimation;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _opacity =
        visitor(_opacity, widget.opacity, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
  }

  @override
  void didUpdateTweens() {
    _opacityAnimation = animation.drive(_opacity!);
  }

  @override
  Widget build(BuildContext context) {
    return FadeTransition(
      opacity: _opacityAnimation,
      alwaysIncludeSemantics: widget.alwaysIncludeSemantics,
      child: widget.child,
    );
  }
}

/// Animated version of [SliverOpacity] which automatically transitions the
/// sliver child's opacity over a given duration whenever the given opacity
/// changes.
///
/// Animating an opacity is relatively expensive because it requires painting
/// the sliver child into an intermediate buffer.
///
/// Here's an illustration of what using this widget looks like, using a [curve]
/// of [Curves.fastOutSlowIn].
///
/// {@tool dartpad}
/// Creates a [CustomScrollView] with a [SliverFixedExtentList] and a
/// [FloatingActionButton]. Pressing the button animates the lists' opacity.
///
/// ** See code in examples/api/lib/widgets/implicit_animations/sliver_animated_opacity.0.dart **
/// {@end-tool}
///
/// ## Hit testing
///
/// Setting the [opacity] to zero does not prevent hit testing from being
/// applied to the descendants of the [SliverAnimatedOpacity] widget. This can
/// be confusing for the user, who may not see anything, and may believe the
/// area of the interface where the [SliverAnimatedOpacity] is hiding a widget
/// to be non-interactive.
///
/// With certain widgets, such as [Flow], that compute their positions only when
/// they are painted, this can actually lead to bugs (from unexpected geometry
/// to exceptions), because those widgets are not painted by the
/// [SliverAnimatedOpacity] widget at all when the [opacity] animation reaches
/// zero.
///
/// To avoid such problems, it is generally a good idea to use a
/// [SliverIgnorePointer] widget when setting the [opacity] to zero. This
/// prevents interactions with any children in the subtree when the [sliver] is
/// animating away.
///
/// See also:
///
///  * [SliverFadeTransition], an explicitly animated version of this widget, where
///    an [Animation] is provided by the caller instead of being built in.
///  * [AnimatedOpacity], for automatically transitioning a box child's
///    opacity over a given duration whenever the given opacity changes.
class SliverAnimatedOpacity extends ImplicitlyAnimatedWidget {
  /// Creates a widget that animates its opacity implicitly.
  ///
  /// The [opacity] argument must be between zero and one, inclusive.
  const SliverAnimatedOpacity({
    super.key,
    this.sliver,
    required this.opacity,
    super.curve,
    required super.duration,
    super.onEnd,
    this.alwaysIncludeSemantics = false,
  }) : assert(opacity >= 0.0 && opacity <= 1.0);

  /// The sliver below this widget in the tree.
  final Widget? sliver;

  /// The target opacity.
  ///
  /// An opacity of 1.0 is fully opaque. An opacity of 0.0 is fully transparent
  /// (i.e., invisible).
  final double opacity;

  /// Whether the semantic information of the children is always included.
  ///
  /// Defaults to false.
  ///
  /// When true, regardless of the opacity settings the sliver child's semantic
  /// information is exposed as if the widget were fully visible. This is
  /// useful in cases where labels may be hidden during animations that
  /// would otherwise contribute relevant semantics.
  final bool alwaysIncludeSemantics;

  @override
  ImplicitlyAnimatedWidgetState<SliverAnimatedOpacity> createState() =>
      _SliverAnimatedOpacityState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('opacity', opacity));
  }
}

class _SliverAnimatedOpacityState extends ImplicitlyAnimatedWidgetState<SliverAnimatedOpacity> {
  Tween<double>? _opacity;
  late Animation<double> _opacityAnimation;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _opacity =
        visitor(_opacity, widget.opacity, (dynamic value) => Tween<double>(begin: value as double))
            as Tween<double>?;
  }

  @override
  void didUpdateTweens() {
    _opacityAnimation = animation.drive(_opacity!);
  }

  @override
  Widget build(BuildContext context) {
    return SliverFadeTransition(
      opacity: _opacityAnimation,
      sliver: widget.sliver,
      alwaysIncludeSemantics: widget.alwaysIncludeSemantics,
    );
  }
}

/// Animated version of [DefaultTextStyle] which automatically transitions the
/// default text style (the text style to apply to descendant [Text] widgets
/// without explicit style) over a given duration whenever the given style
/// changes.
///
/// The [textAlign], [softWrap], [overflow], [maxLines], [textWidthBasis]
/// and [textHeightBehavior] properties are not animated and take effect
/// immediately when changed.
///
/// Here's an illustration of what using this widget looks like, using a [curve]
/// of [Curves.elasticInOut].
/// {@animation 250 266 https://flutter.github.io/assets-for-api-docs/assets/widgets/animated_default_text_style.mp4}
///
/// For the animation, you can choose a [curve] as well as a [duration] and the
/// widget will automatically animate to the new default text style. If you require
/// more control over the animation (e.g. if you want to stop it mid-animation),
/// consider using a [DefaultTextStyleTransition] instead, which takes a provided
/// [Animation] as argument. While that allows you to fine-tune the animation,
/// it also requires more development overhead as you have to manually manage
/// the lifecycle of the underlying [AnimationController].
class AnimatedDefaultTextStyle extends ImplicitlyAnimatedWidget {
  /// Creates a widget that animates the default text style implicitly.
  const AnimatedDefaultTextStyle({
    super.key,
    required this.child,
    required this.style,
    this.textAlign,
    this.softWrap = true,
    this.overflow = TextOverflow.clip,
    this.maxLines,
    this.textWidthBasis = TextWidthBasis.parent,
    this.textHeightBehavior,
    super.curve,
    required super.duration,
    super.onEnd,
  }) : assert(maxLines == null || maxLines > 0);

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// The target text style.
  ///
  /// When this property is changed, the style will be animated over [duration] time.
  final TextStyle style;

  /// How the text should be aligned horizontally.
  ///
  /// This property takes effect immediately when changed, it is not animated.
  final TextAlign? textAlign;

  /// Whether the text should break at soft line breaks.
  ///
  /// This property takes effect immediately when changed, it is not animated.
  ///
  /// See [DefaultTextStyle.softWrap] for more details.
  final bool softWrap;

  /// How visual overflow should be handled.
  ///
  /// This property takes effect immediately when changed, it is not animated.
  final TextOverflow overflow;

  /// An optional maximum number of lines for the text to span, wrapping if necessary.
  ///
  /// This property takes effect immediately when changed, it is not animated.
  ///
  /// See [DefaultTextStyle.maxLines] for more details.
  final int? maxLines;

  /// The strategy to use when calculating the width of the Text.
  ///
  /// See [TextWidthBasis] for possible values and their implications.
  final TextWidthBasis textWidthBasis;

  /// {@macro dart.ui.textHeightBehavior}
  final ui.TextHeightBehavior? textHeightBehavior;

  @override
  AnimatedWidgetBaseState<AnimatedDefaultTextStyle> createState() =>
      _AnimatedDefaultTextStyleState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    style.debugFillProperties(properties);
    properties.add(EnumProperty<TextAlign>('textAlign', textAlign, defaultValue: null));
    properties.add(
      FlagProperty(
        'softWrap',
        value: softWrap,
        ifTrue: 'wrapping at box width',
        ifFalse: 'no wrapping except at line break characters',
        showName: true,
      ),
    );
    properties.add(EnumProperty<TextOverflow>('overflow', overflow, defaultValue: null));
    properties.add(IntProperty('maxLines', maxLines, defaultValue: null));
    properties.add(
      EnumProperty<TextWidthBasis>(
        'textWidthBasis',
        textWidthBasis,
        defaultValue: TextWidthBasis.parent,
      ),
    );
    properties.add(
      DiagnosticsProperty<ui.TextHeightBehavior>(
        'textHeightBehavior',
        textHeightBehavior,
        defaultValue: null,
      ),
    );
  }
}

class _AnimatedDefaultTextStyleState extends AnimatedWidgetBaseState<AnimatedDefaultTextStyle> {
  TextStyleTween? _style;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _style =
        visitor(_style, widget.style, (dynamic value) => TextStyleTween(begin: value as TextStyle))
            as TextStyleTween?;
  }

  @override
  Widget build(BuildContext context) {
    return DefaultTextStyle(
      style: _style!.evaluate(animation),
      textAlign: widget.textAlign,
      softWrap: widget.softWrap,
      overflow: widget.overflow,
      maxLines: widget.maxLines,
      textWidthBasis: widget.textWidthBasis,
      textHeightBehavior: widget.textHeightBehavior,
      child: widget.child,
    );
  }
}

/// Animated version of [PhysicalModel].
///
/// The [borderRadius] and [elevation] are animated.
///
/// The [color] is animated if the [animateColor] property is set; otherwise,
/// the color changes immediately at the start of the animation for the other
/// two properties. This allows the color to be animated independently (e.g.
/// because it is being driven by an [AnimatedTheme]).
///
/// The [shape] is not animated.
///
/// Here's an illustration of what using this widget looks like, using a [curve]
/// of [Curves.fastOutSlowIn].
/// {@animation 250 266 https://flutter.github.io/assets-for-api-docs/assets/widgets/animated_physical_model.mp4}
class AnimatedPhysicalModel extends ImplicitlyAnimatedWidget {
  /// Creates a widget that animates the properties of a [PhysicalModel].
  ///
  /// The [elevation] must be non-negative.
  ///
  /// Animating [color] is optional and is controlled by the [animateColor] flag.
  ///
  /// Animating [shadowColor] is optional and is controlled by the [animateShadowColor] flag.
  const AnimatedPhysicalModel({
    super.key,
    required this.child,
    this.shape = BoxShape.rectangle,
    this.clipBehavior = Clip.none,
    this.borderRadius,
    this.elevation = 0.0,
    required this.color,
    this.animateColor = true,
    required this.shadowColor,
    this.animateShadowColor = true,
    super.curve,
    required super.duration,
    super.onEnd,
  }) : assert(elevation >= 0.0);

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// The type of shape.
  ///
  /// This property is not animated.
  final BoxShape shape;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  final Clip clipBehavior;

  /// The target border radius of the rounded corners for a rectangle shape.
  ///
  /// If null, treated as [BorderRadius.zero].
  final BorderRadius? borderRadius;

  /// The target z-coordinate relative to the parent at which to place this
  /// physical object.
  ///
  /// The value will always be non-negative.
  final double elevation;

  /// The target background color.
  final Color color;

  /// Whether the color should be animated.
  final bool animateColor;

  /// The target shadow color.
  final Color shadowColor;

  /// Whether the shadow color should be animated.
  final bool animateShadowColor;

  @override
  AnimatedWidgetBaseState<AnimatedPhysicalModel> createState() => _AnimatedPhysicalModelState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<BoxShape>('shape', shape));
    properties.add(DiagnosticsProperty<BorderRadius>('borderRadius', borderRadius));
    properties.add(DoubleProperty('elevation', elevation));
    properties.add(ColorProperty('color', color));
    properties.add(DiagnosticsProperty<bool>('animateColor', animateColor));
    properties.add(ColorProperty('shadowColor', shadowColor));
    properties.add(DiagnosticsProperty<bool>('animateShadowColor', animateShadowColor));
  }
}

class _AnimatedPhysicalModelState extends AnimatedWidgetBaseState<AnimatedPhysicalModel> {
  BorderRadiusTween? _borderRadius;
  Tween<double>? _elevation;
  ColorTween? _color;
  ColorTween? _shadowColor;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _borderRadius =
        visitor(
              _borderRadius,
              widget.borderRadius ?? BorderRadius.zero,
              (dynamic value) => BorderRadiusTween(begin: value as BorderRadius),
            )
            as BorderRadiusTween?;
    _elevation =
        visitor(
              _elevation,
              widget.elevation,
              (dynamic value) => Tween<double>(begin: value as double),
            )
            as Tween<double>?;
    _color =
        visitor(_color, widget.color, (dynamic value) => ColorTween(begin: value as Color))
            as ColorTween?;
    _shadowColor =
        visitor(
              _shadowColor,
              widget.shadowColor,
              (dynamic value) => ColorTween(begin: value as Color),
            )
            as ColorTween?;
  }

  @override
  Widget build(BuildContext context) {
    return PhysicalModel(
      shape: widget.shape,
      clipBehavior: widget.clipBehavior,
      borderRadius: _borderRadius!.evaluate(animation),
      elevation: _elevation!.evaluate(animation),
      color: widget.animateColor ? _color!.evaluate(animation)! : widget.color,
      shadowColor:
          widget.animateShadowColor ? _shadowColor!.evaluate(animation)! : widget.shadowColor,
      child: widget.child,
    );
  }
}

/// Animated version of [FractionallySizedBox] which automatically transitions the
/// child's size over a given duration whenever the given [widthFactor] or
/// [heightFactor] changes, as well as the position whenever the given [alignment]
/// changes.
///
/// For the animation, you can choose a [curve] as well as a [duration] and the
/// widget will automatically animate to the new target [widthFactor] or
/// [heightFactor].
///
/// {@tool dartpad}
/// The following example transitions an [AnimatedFractionallySizedBox]
/// between two states. It adjusts the [heightFactor], [widthFactor], and
/// [alignment] properties when tapped, using a [curve] of [Curves.fastOutSlowIn]
///
/// ** See code in examples/api/lib/widgets/implicit_animations/animated_fractionally_sized_box.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [AnimatedAlign], which is an implicitly animated version of [Align].
///  * [AnimatedContainer], which can transition more values at once.
///  * [AnimatedSlide], which can animate the translation of child by a given offset relative to its size.
///  * [AnimatedPositioned], which, as a child of a [Stack], automatically
///    transitions its child's position over a given duration whenever the given
///    position changes.
class AnimatedFractionallySizedBox extends ImplicitlyAnimatedWidget {
  /// Creates a widget that sizes its child to a fraction of the total available
  /// space that animates implicitly, and positions its child by an alignment
  /// that animates implicitly.
  ///
  /// If non-null, the [widthFactor] and [heightFactor] arguments must be
  /// non-negative.
  const AnimatedFractionallySizedBox({
    super.key,
    this.alignment = Alignment.center,
    this.child,
    this.heightFactor,
    this.widthFactor,
    super.curve,
    required super.duration,
    super.onEnd,
  }) : assert(widthFactor == null || widthFactor >= 0.0),
       assert(heightFactor == null || heightFactor >= 0.0);

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// {@macro flutter.widgets.basic.fractionallySizedBox.heightFactor}
  final double? heightFactor;

  /// {@macro flutter.widgets.basic.fractionallySizedBox.widthFactor}
  final double? widthFactor;

  /// {@macro flutter.widgets.basic.fractionallySizedBox.alignment}
  final AlignmentGeometry alignment;

  @override
  AnimatedWidgetBaseState<AnimatedFractionallySizedBox> createState() =>
      _AnimatedFractionallySizedBoxState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<AlignmentGeometry>('alignment', alignment));
    properties.add(DiagnosticsProperty<double>('widthFactor', widthFactor));
    properties.add(DiagnosticsProperty<double>('heightFactor', heightFactor));
  }
}

class _AnimatedFractionallySizedBoxState
    extends AnimatedWidgetBaseState<AnimatedFractionallySizedBox> {
  AlignmentGeometryTween? _alignment;
  Tween<double>? _heightFactorTween;
  Tween<double>? _widthFactorTween;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _alignment =
        visitor(
              _alignment,
              widget.alignment,
              (dynamic value) => AlignmentGeometryTween(begin: value as AlignmentGeometry),
            )
            as AlignmentGeometryTween?;
    if (widget.heightFactor != null) {
      _heightFactorTween =
          visitor(
                _heightFactorTween,
                widget.heightFactor,
                (dynamic value) => Tween<double>(begin: value as double),
              )
              as Tween<double>?;
    }
    if (widget.widthFactor != null) {
      _widthFactorTween =
          visitor(
                _widthFactorTween,
                widget.widthFactor,
                (dynamic value) => Tween<double>(begin: value as double),
              )
              as Tween<double>?;
    }
  }

  @override
  Widget build(BuildContext context) {
    return FractionallySizedBox(
      alignment: _alignment!.evaluate(animation)!,
      heightFactor: _heightFactorTween?.evaluate(animation),
      widthFactor: _widthFactorTween?.evaluate(animation),
      child: widget.child,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(
      DiagnosticsProperty<AlignmentGeometryTween>('alignment', _alignment, defaultValue: null),
    );
    description.add(
      DiagnosticsProperty<Tween<double>>('widthFactor', _widthFactorTween, defaultValue: null),
    );
    description.add(
      DiagnosticsProperty<Tween<double>>('heightFactor', _heightFactorTween, defaultValue: null),
    );
  }
}
