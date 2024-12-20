// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'animated_cross_fade.dart';
/// @docImport 'animated_switcher.dart';
/// @docImport 'implicit_animations.dart';
/// @docImport 'navigator.dart';
/// @docImport 'transitions.dart';
library;

import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'framework.dart';
import 'sliver.dart';
import 'ticker_provider.dart';

/// Whether to show or hide a child.
///
/// By default, the [visible] property controls whether the [child] is included
/// in the subtree or not; when it is not [visible], the [replacement] child
/// (typically a zero-sized box) is included instead.
///
/// A variety of flags can be used to tweak exactly how the child is hidden.
/// (Changing the flags dynamically is discouraged, as it can cause the [child]
/// subtree to be rebuilt, with any state in the subtree being discarded.
/// Typically, only the [visible] flag is changed dynamically.)
///
/// These widgets provide some of the facets of this one:
///
///  * [Opacity], which can stop its child from being painted.
///  * [Offstage], which can stop its child from being laid out or painted.
///  * [TickerMode], which can stop its child from being animated.
///  * [ExcludeSemantics], which can hide the child from accessibility tools.
///  * [IgnorePointer], which can disable touch interactions with the child.
///
/// Using this widget is not necessary to hide children. The simplest way to
/// hide a child is just to not include it, or, if a child _must_ be given (e.g.
/// because the parent is a [StatelessWidget]) then to use [SizedBox.shrink]
/// instead of the child that would otherwise be included.
///
/// See also:
///
///  * [AnimatedSwitcher], which can fade from one child to the next as the
///    subtree changes.
///  * [AnimatedCrossFade], which can fade between two specific children.
///  * [SliverVisibility], the sliver equivalent of this widget.
class Visibility extends StatelessWidget {
  /// Control whether the given [child] is [visible].
  ///
  /// The [maintainSemantics] and [maintainInteractivity] arguments can only be
  /// set if [maintainSize] is set.
  ///
  /// The [maintainSize] argument can only be set if [maintainAnimation] is set.
  ///
  /// The [maintainAnimation] argument can only be set if [maintainState] is
  /// set.
  const Visibility({
    super.key,
    required this.child,
    this.replacement = const SizedBox.shrink(),
    this.visible = true,
    this.maintainState = false,
    this.maintainAnimation = false,
    this.maintainSize = false,
    this.maintainSemantics = false,
    this.maintainInteractivity = false,
  }) : assert(
         maintainState || !maintainAnimation,
         'Cannot maintain animations if the state is not also maintained.',
       ),
       assert(
         maintainAnimation || !maintainSize,
         'Cannot maintain size if animations are not maintained.',
       ),
       assert(
         maintainSize || !maintainSemantics,
         'Cannot maintain semantics if size is not maintained.',
       ),
       assert(
         maintainSize || !maintainInteractivity,
         'Cannot maintain interactivity if size is not maintained.',
       );

  /// Control whether the given [child] is [visible].
  ///
  /// This is equivalent to the default [Visibility] constructor with all
  /// "maintain" fields set to true. This constructor should be used in place of
  /// an [Opacity] widget that only takes on values of `0.0` or `1.0`, as it
  /// avoids extra compositing when fully opaque.
  const Visibility.maintain({super.key, required this.child, this.visible = true})
    : maintainState = true,
      maintainAnimation = true,
      maintainSize = true,
      maintainSemantics = true,
      maintainInteractivity = true,
      replacement = const SizedBox.shrink(); // Unused since maintainState is always true.

  /// The widget to show or hide, as controlled by [visible].
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// The widget to use when the child is not [visible], assuming that none of
  /// the `maintain` flags (in particular, [maintainState]) are set.
  ///
  /// The normal behavior is to replace the widget with a zero by zero box
  /// ([SizedBox.shrink]).
  ///
  /// See also:
  ///
  ///  * [AnimatedCrossFade], which can animate between two children.
  final Widget replacement;

  /// Switches between showing the [child] or hiding it.
  ///
  /// The `maintain` flags should be set to the same values regardless of the
  /// state of the [visible] property, otherwise they will not operate correctly
  /// (specifically, the state will be lost regardless of the state of
  /// [maintainState] whenever any of the `maintain` flags are changed, since
  /// doing so will result in a subtree shape change).
  ///
  /// Unless [maintainState] is set, the [child] subtree will be disposed
  /// (removed from the tree) while hidden.
  final bool visible;

  /// Whether to maintain the [State] objects of the [child] subtree when it is
  /// not [visible].
  ///
  /// Keeping the state of the subtree is potentially expensive (because it
  /// means all the objects are still in memory; their resources are not
  /// released). It should only be maintained if it cannot be recreated on
  /// demand. One example of when the state would be maintained is if the child
  /// subtree contains a [Navigator], since that widget maintains elaborate
  /// state that cannot be recreated on the fly.
  ///
  /// If this property is true, an [Offstage] widget is used to hide the child
  /// instead of replacing it with [replacement].
  ///
  /// If this property is false, then [maintainAnimation] must also be false.
  ///
  /// Dynamically changing this value may cause the current state of the
  /// subtree to be lost (and a new instance of the subtree, with new [State]
  /// objects, to be immediately created if [visible] is true).
  final bool maintainState;

  /// Whether to maintain animations within the [child] subtree when it is
  /// not [visible].
  ///
  /// To set this, [maintainState] must also be set.
  ///
  /// Keeping animations active when the widget is not visible is even more
  /// expensive than only maintaining the state.
  ///
  /// One example when this might be useful is if the subtree is animating its
  /// layout in time with an [AnimationController], and the result of that
  /// layout is being used to influence some other logic. If this flag is false,
  /// then any [AnimationController]s hosted inside the [child] subtree will be
  /// muted while the [visible] flag is false.
  ///
  /// If this property is true, no [TickerMode] widget is used.
  ///
  /// If this property is false, then [maintainSize] must also be false.
  ///
  /// Dynamically changing this value may cause the current state of the
  /// subtree to be lost (and a new instance of the subtree, with new [State]
  /// objects, to be immediately created if [visible] is true).
  final bool maintainAnimation;

  /// Whether to maintain space for where the widget would have been.
  ///
  /// To set this, [maintainAnimation] and [maintainState] must also be set.
  ///
  /// Maintaining the size when the widget is not [visible] is not notably more
  /// expensive than just keeping animations running without maintaining the
  /// size, and may in some circumstances be slightly cheaper if the subtree is
  /// simple and the [visible] property is frequently toggled, since it avoids
  /// triggering a layout change when the [visible] property is toggled. If the
  /// [child] subtree is not trivial then it is significantly cheaper to not
  /// even keep the state (see [maintainState]).
  ///
  /// If this property is false, [Offstage] is used.
  ///
  /// If this property is false, then [maintainSemantics] and
  /// [maintainInteractivity] must also be false.
  ///
  /// Dynamically changing this value may cause the current state of the
  /// subtree to be lost (and a new instance of the subtree, with new [State]
  /// objects, to be immediately created if [visible] is true).
  ///
  /// See also:
  ///
  ///  * [AnimatedOpacity] and [FadeTransition], which apply animations to the
  ///    opacity for a more subtle effect.
  final bool maintainSize;

  /// Whether to maintain the semantics for the widget when it is hidden (e.g.
  /// for accessibility).
  ///
  /// To set this, [maintainSize] must also be set.
  ///
  /// By default, with [maintainSemantics] set to false, the [child] is not
  /// visible to accessibility tools when it is hidden from the user. If this
  /// flag is set to true, then accessibility tools will report the widget as if
  /// it was present.
  final bool maintainSemantics;

  /// Whether to allow the widget to be interactive when hidden.
  ///
  /// To set this, [maintainSize] must also be set.
  ///
  /// By default, with [maintainInteractivity] set to false, touch events cannot
  /// reach the [child] when it is hidden from the user. If this flag is set to
  /// true, then touch events will nonetheless be passed through.
  final bool maintainInteractivity;

  /// Tells the visibility state of an element in the tree based off its
  /// ancestor [Visibility] elements.
  ///
  /// If there's one or more [Visibility] widgets in the ancestor tree, this
  /// will return true if and only if all of those widgets have [visible] set
  /// to true. If there is no [Visibility] widget in the ancestor tree of the
  /// specified build context, this will return true.
  ///
  /// This will register a dependency from the specified context on any
  /// [Visibility] elements in the ancestor tree, such that if any of their
  /// visibilities changes, the specified context will be rebuilt.
  static bool of(BuildContext context) {
    bool isVisible = true;
    BuildContext ancestorContext = context;
    InheritedElement? ancestor =
        ancestorContext.getElementForInheritedWidgetOfExactType<_VisibilityScope>();
    while (isVisible && ancestor != null) {
      final _VisibilityScope scope = context.dependOnInheritedElement(ancestor) as _VisibilityScope;
      isVisible = scope.isVisible;
      ancestor.visitAncestorElements((Element parent) {
        ancestorContext = parent;
        return false;
      });
      ancestor = ancestorContext.getElementForInheritedWidgetOfExactType<_VisibilityScope>();
    }
    return isVisible;
  }

  @override
  Widget build(BuildContext context) {
    Widget result = child;
    if (maintainSize) {
      result = _Visibility(
        visible: visible,
        maintainSemantics: maintainSemantics,
        child: IgnorePointer(ignoring: !visible && !maintainInteractivity, child: result),
      );
    } else {
      assert(!maintainInteractivity);
      assert(!maintainSemantics);
      assert(!maintainSize);
      if (maintainState) {
        if (!maintainAnimation) {
          result = TickerMode(enabled: visible, child: result);
        }
        result = Offstage(offstage: !visible, child: result);
      } else {
        assert(!maintainAnimation);
        assert(!maintainState);
        result = visible ? child : replacement;
      }
    }
    return _VisibilityScope(isVisible: visible, child: result);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(FlagProperty('visible', value: visible, ifFalse: 'hidden', ifTrue: 'visible'));
    properties.add(FlagProperty('maintainState', value: maintainState, ifFalse: 'maintainState'));
    properties.add(
      FlagProperty('maintainAnimation', value: maintainAnimation, ifFalse: 'maintainAnimation'),
    );
    properties.add(FlagProperty('maintainSize', value: maintainSize, ifFalse: 'maintainSize'));
    properties.add(
      FlagProperty('maintainSemantics', value: maintainSemantics, ifFalse: 'maintainSemantics'),
    );
    properties.add(
      FlagProperty(
        'maintainInteractivity',
        value: maintainInteractivity,
        ifFalse: 'maintainInteractivity',
      ),
    );
  }
}

/// Inherited widget that allows descendants to find their visibility status.
class _VisibilityScope extends InheritedWidget {
  const _VisibilityScope({required this.isVisible, required super.child});

  final bool isVisible;

  @override
  bool updateShouldNotify(_VisibilityScope old) {
    return isVisible != old.isVisible;
  }
}

/// Whether to show or hide a sliver child.
///
/// By default, the [visible] property controls whether the [sliver] is included
/// in the subtree or not; when it is not [visible], the [replacementSliver] is
/// included instead.
///
/// A variety of flags can be used to tweak exactly how the sliver is hidden.
/// (Changing the flags dynamically is discouraged, as it can cause the [sliver]
/// subtree to be rebuilt, with any state in the subtree being discarded.
/// Typically, only the [visible] flag is changed dynamically.)
///
/// These widgets provide some of the facets of this one:
///
///  * [SliverOpacity], which can stop its sliver child from being painted.
///  * [SliverOffstage], which can stop its sliver child from being laid out or
///    painted.
///  * [TickerMode], which can stop its child from being animated.
///  * [ExcludeSemantics], which can hide the child from accessibility tools.
///  * [SliverIgnorePointer], which can disable touch interactions with the
///    sliver child.
///
/// Using this widget is not necessary to hide children. The simplest way to
/// hide a child is just to not include it. If a child _must_ be given (e.g.
/// because the parent is a [StatelessWidget]), then including a childless
/// [SliverToBoxAdapter] instead of the child that would otherwise be included
/// is typically more efficient than using [SliverVisibility].
///
/// See also:
///
///  * [Visibility], the equivalent widget for boxes.
class SliverVisibility extends StatelessWidget {
  /// Control whether the given [sliver] is [visible].
  ///
  /// The [maintainSemantics] and [maintainInteractivity] arguments can only be
  /// set if [maintainSize] is set.
  ///
  /// The [maintainSize] argument can only be set if [maintainAnimation] is set.
  ///
  /// The [maintainAnimation] argument can only be set if [maintainState] is
  /// set.
  const SliverVisibility({
    super.key,
    required this.sliver,
    this.replacementSliver = const SliverToBoxAdapter(),
    this.visible = true,
    this.maintainState = false,
    this.maintainAnimation = false,
    this.maintainSize = false,
    this.maintainSemantics = false,
    this.maintainInteractivity = false,
  }) : assert(
         maintainState || !maintainAnimation,
         'Cannot maintain animations if the state is not also maintained.',
       ),
       assert(
         maintainAnimation || !maintainSize,
         'Cannot maintain size if animations are not maintained.',
       ),
       assert(
         maintainSize || !maintainSemantics,
         'Cannot maintain semantics if size is not maintained.',
       ),
       assert(
         maintainSize || !maintainInteractivity,
         'Cannot maintain interactivity if size is not maintained.',
       );

  /// Control whether the given [sliver] is [visible].
  ///
  /// This is equivalent to the default [SliverVisibility] constructor with all
  /// "maintain" fields set to true. This constructor should be used in place of
  /// a [SliverOpacity] widget that only takes on values of `0.0` or `1.0`, as it
  /// avoids extra compositing when fully opaque.
  const SliverVisibility.maintain({
    super.key,
    required this.sliver,
    this.replacementSliver = const SliverToBoxAdapter(),
    this.visible = true,
  }) : maintainState = true,
       maintainAnimation = true,
       maintainSize = true,
       maintainSemantics = true,
       maintainInteractivity = true;

  /// The sliver to show or hide, as controlled by [visible].
  final Widget sliver;

  /// The widget to use when the sliver child is not [visible], assuming that
  /// none of the `maintain` flags (in particular, [maintainState]) are set.
  ///
  /// The normal behavior is to replace the widget with a childless
  /// [SliverToBoxAdapter], which by default has a geometry of
  /// [SliverGeometry.zero].
  final Widget replacementSliver;

  /// Switches between showing the [sliver] or hiding it.
  ///
  /// The `maintain` flags should be set to the same values regardless of the
  /// state of the [visible] property, otherwise they will not operate correctly
  /// (specifically, the state will be lost regardless of the state of
  /// [maintainState] whenever any of the `maintain` flags are changed, since
  /// doing so will result in a subtree shape change).
  ///
  /// Unless [maintainState] is set, the [sliver] subtree will be disposed
  /// (removed from the tree) while hidden.
  final bool visible;

  /// Whether to maintain the [State] objects of the [sliver] subtree when it is
  /// not [visible].
  ///
  /// Keeping the state of the subtree is potentially expensive (because it
  /// means all the objects are still in memory; their resources are not
  /// released). It should only be maintained if it cannot be recreated on
  /// demand. One example of when the state would be maintained is if the sliver
  /// subtree contains a [Navigator], since that widget maintains elaborate
  /// state that cannot be recreated on the fly.
  ///
  /// If this property is true, a [SliverOffstage] widget is used to hide the
  /// sliver instead of replacing it with [replacementSliver].
  ///
  /// If this property is false, then [maintainAnimation] must also be false.
  ///
  /// Dynamically changing this value may cause the current state of the
  /// subtree to be lost (and a new instance of the subtree, with new [State]
  /// objects, to be immediately created if [visible] is true).
  final bool maintainState;

  /// Whether to maintain animations within the [sliver] subtree when it is
  /// not [visible].
  ///
  /// To set this, [maintainState] must also be set.
  ///
  /// Keeping animations active when the widget is not visible is even more
  /// expensive than only maintaining the state.
  ///
  /// One example when this might be useful is if the subtree is animating its
  /// layout in time with an [AnimationController], and the result of that
  /// layout is being used to influence some other logic. If this flag is false,
  /// then any [AnimationController]s hosted inside the [sliver] subtree will be
  /// muted while the [visible] flag is false.
  ///
  /// If this property is true, no [TickerMode] widget is used.
  ///
  /// If this property is false, then [maintainSize] must also be false.
  ///
  /// Dynamically changing this value may cause the current state of the
  /// subtree to be lost (and a new instance of the subtree, with new [State]
  /// objects, to be immediately created if [visible] is true).
  final bool maintainAnimation;

  /// Whether to maintain space for where the sliver would have been.
  ///
  /// To set this, [maintainAnimation] must also be set.
  ///
  /// Maintaining the size when the sliver is not [visible] is not notably more
  /// expensive than just keeping animations running without maintaining the
  /// size, and may in some circumstances be slightly cheaper if the subtree is
  /// simple and the [visible] property is frequently toggled, since it avoids
  /// triggering a layout change when the [visible] property is toggled. If the
  /// [sliver] subtree is not trivial then it is significantly cheaper to not
  /// even keep the state (see [maintainState]).
  ///
  /// If this property is false, [SliverOffstage] is used.
  ///
  /// If this property is false, then [maintainSemantics] and
  /// [maintainInteractivity] must also be false.
  ///
  /// Dynamically changing this value may cause the current state of the
  /// subtree to be lost (and a new instance of the subtree, with new [State]
  /// objects, to be immediately created if [visible] is true).
  final bool maintainSize;

  /// Whether to maintain the semantics for the sliver when it is hidden (e.g.
  /// for accessibility).
  ///
  /// To set this, [maintainSize] must also be set.
  ///
  /// By default, with [maintainSemantics] set to false, the [sliver] is not
  /// visible to accessibility tools when it is hidden from the user. If this
  /// flag is set to true, then accessibility tools will report the widget as if
  /// it was present.
  final bool maintainSemantics;

  /// Whether to allow the sliver to be interactive when hidden.
  ///
  /// To set this, [maintainSize] must also be set.
  ///
  /// By default, with [maintainInteractivity] set to false, touch events cannot
  /// reach the [sliver] when it is hidden from the user. If this flag is set to
  /// true, then touch events will nonetheless be passed through.
  final bool maintainInteractivity;

  @override
  Widget build(BuildContext context) {
    if (maintainSize) {
      Widget result = sliver;
      result = SliverIgnorePointer(ignoring: !visible && !maintainInteractivity, sliver: result);
      return _SliverVisibility(
        visible: visible,
        maintainSemantics: maintainSemantics,
        sliver: result,
      );
    }
    assert(!maintainInteractivity);
    assert(!maintainSemantics);
    assert(!maintainSize);
    if (maintainState) {
      Widget result = sliver;
      if (!maintainAnimation) {
        result = TickerMode(enabled: visible, child: sliver);
      }
      return SliverOffstage(sliver: result, offstage: !visible);
    }
    assert(!maintainAnimation);
    assert(!maintainState);
    return visible ? sliver : replacementSliver;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(FlagProperty('visible', value: visible, ifFalse: 'hidden', ifTrue: 'visible'));
    properties.add(FlagProperty('maintainState', value: maintainState, ifFalse: 'maintainState'));
    properties.add(
      FlagProperty('maintainAnimation', value: maintainAnimation, ifFalse: 'maintainAnimation'),
    );
    properties.add(FlagProperty('maintainSize', value: maintainSize, ifFalse: 'maintainSize'));
    properties.add(
      FlagProperty('maintainSemantics', value: maintainSemantics, ifFalse: 'maintainSemantics'),
    );
    properties.add(
      FlagProperty(
        'maintainInteractivity',
        value: maintainInteractivity,
        ifFalse: 'maintainInteractivity',
      ),
    );
  }
}

// A widget that conditionally hides its child, but without the forced compositing of `Opacity`.
//
// A fully opaque `Opacity` widget is required to leave its opacity layer in the layer tree. This
// forces all parent render objects to also composite, which can break a simple scene into many
// different layers. This can be significantly more expensive, so the issue is avoided by a
// specialized render object that does not ever force compositing.
class _Visibility extends SingleChildRenderObjectWidget {
  const _Visibility({required this.visible, required this.maintainSemantics, super.child});

  final bool visible;
  final bool maintainSemantics;

  @override
  _RenderVisibility createRenderObject(BuildContext context) {
    return _RenderVisibility(visible, maintainSemantics);
  }

  @override
  void updateRenderObject(BuildContext context, _RenderVisibility renderObject) {
    renderObject
      ..visible = visible
      ..maintainSemantics = maintainSemantics;
  }
}

class _RenderVisibility extends RenderProxyBox {
  _RenderVisibility(this._visible, this._maintainSemantics);

  bool get visible => _visible;
  bool _visible;
  set visible(bool value) {
    if (value == visible) {
      return;
    }
    _visible = value;
    markNeedsPaint();
  }

  bool get maintainSemantics => _maintainSemantics;
  bool _maintainSemantics;
  set maintainSemantics(bool value) {
    if (value == maintainSemantics) {
      return;
    }
    _maintainSemantics = value;
    markNeedsSemanticsUpdate();
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    if (maintainSemantics || visible) {
      super.visitChildrenForSemantics(visitor);
    }
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (!visible) {
      return;
    }
    super.paint(context, offset);
  }
}

// A widget that conditionally hides its child, but without the forced compositing of `SliverOpacity`.
//
// A fully opaque `SliverOpacity` widget is required to leave its opacity layer in the layer tree.
// This forces all parent render objects to also composite, which can break a simple scene into many
// different layers. This can be significantly more expensive, so the issue is avoided by a
// specialized render object that does not ever force compositing.
class _SliverVisibility extends SingleChildRenderObjectWidget {
  const _SliverVisibility({required this.visible, required this.maintainSemantics, Widget? sliver})
    : super(child: sliver);

  final bool visible;
  final bool maintainSemantics;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _RenderSliverVisibility(visible, maintainSemantics);
  }

  @override
  void updateRenderObject(BuildContext context, _RenderSliverVisibility renderObject) {
    renderObject
      ..visible = visible
      ..maintainSemantics = maintainSemantics;
  }
}

class _RenderSliverVisibility extends RenderProxySliver {
  _RenderSliverVisibility(this._visible, this._maintainSemantics);

  bool get visible => _visible;
  bool _visible;
  set visible(bool value) {
    if (value == visible) {
      return;
    }
    _visible = value;
    markNeedsPaint();
  }

  bool get maintainSemantics => _maintainSemantics;
  bool _maintainSemantics;
  set maintainSemantics(bool value) {
    if (value == maintainSemantics) {
      return;
    }
    _maintainSemantics = value;
    markNeedsSemanticsUpdate();
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    if (maintainSemantics || visible) {
      super.visitChildrenForSemantics(visitor);
    }
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (!visible) {
      return;
    }
    super.paint(context, offset);
  }
}
