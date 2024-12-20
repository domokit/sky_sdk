// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'page_storage.dart';
/// @docImport 'page_view.dart';
/// @docImport 'scroll_metrics.dart';
/// @docImport 'scroll_notification.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'single_child_scroll_view.dart';
/// @docImport 'two_dimensional_scroll_view.dart';
/// @docImport 'two_dimensional_viewport.dart';
library;

import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

import 'basic.dart';
import 'framework.dart';
import 'gesture_detector.dart';
import 'media_query.dart';
import 'notification_listener.dart';
import 'restoration.dart';
import 'restoration_properties.dart';
import 'scroll_activity.dart';
import 'scroll_configuration.dart';
import 'scroll_context.dart';
import 'scroll_controller.dart';
import 'scroll_physics.dart';
import 'scroll_position.dart';
import 'scrollable_helpers.dart';
import 'selectable_region.dart';
import 'selection_container.dart';
import 'ticker_provider.dart';
import 'view.dart';
import 'viewport.dart';

export 'package:flutter/physics.dart' show Tolerance;

// Examples can assume:
// late BuildContext context;

/// Signature used by [Scrollable] to build the viewport through which the
/// scrollable content is displayed.
typedef ViewportBuilder = Widget Function(BuildContext context, ViewportOffset position);

/// Signature used by [TwoDimensionalScrollable] to build the viewport through
/// which the scrollable content is displayed.
typedef TwoDimensionalViewportBuilder =
    Widget Function(
      BuildContext context,
      ViewportOffset verticalPosition,
      ViewportOffset horizontalPosition,
    );

// The return type of _performEnsureVisible.
//
// The list of futures represents each pending ScrollPosition call to
// ensureVisible. The returned ScrollableState's context is used to find the
// next potential ancestor Scrollable.
typedef _EnsureVisibleResults = (List<Future<void>>, ScrollableState);

/// A widget that manages scrolling in one dimension and informs the [Viewport]
/// through which the content is viewed.
///
/// [Scrollable] implements the interaction model for a scrollable widget,
/// including gesture recognition, but does not have an opinion about how the
/// viewport, which actually displays the children, is constructed.
///
/// It's rare to construct a [Scrollable] directly. Instead, consider [ListView]
/// or [GridView], which combine scrolling, viewporting, and a layout model. To
/// combine layout models (or to use a custom layout mode), consider using
/// [CustomScrollView].
///
/// The static [Scrollable.of] and [Scrollable.ensureVisible] functions are
/// often used to interact with the [Scrollable] widget inside a [ListView] or
/// a [GridView].
///
/// To further customize scrolling behavior with a [Scrollable]:
///
/// 1. You can provide a [viewportBuilder] to customize the child model. For
///    example, [SingleChildScrollView] uses a viewport that displays a single
///    box child whereas [CustomScrollView] uses a [Viewport] or a
///    [ShrinkWrappingViewport], both of which display a list of slivers.
///
/// 2. You can provide a custom [ScrollController] that creates a custom
///    [ScrollPosition] subclass. For example, [PageView] uses a
///    [PageController], which creates a page-oriented scroll position subclass
///    that keeps the same page visible when the [Scrollable] resizes.
///
/// ## Persisting the scroll position during a session
///
/// Scrollables attempt to persist their scroll position using [PageStorage].
/// This can be disabled by setting [ScrollController.keepScrollOffset] to false
/// on the [controller]. If it is enabled, using a [PageStorageKey] for the
/// [key] of this widget (or one of its ancestors, e.g. a [ScrollView]) is
/// recommended to help disambiguate different [Scrollable]s from each other.
///
/// See also:
///
///  * [ListView], which is a commonly used [ScrollView] that displays a
///    scrolling, linear list of child widgets.
///  * [PageView], which is a scrolling list of child widgets that are each the
///    size of the viewport.
///  * [GridView], which is a [ScrollView] that displays a scrolling, 2D array
///    of child widgets.
///  * [CustomScrollView], which is a [ScrollView] that creates custom scroll
///    effects using slivers.
///  * [SingleChildScrollView], which is a scrollable widget that has a single
///    child.
///  * [ScrollNotification] and [NotificationListener], which can be used to watch
///    the scroll position without using a [ScrollController].
class Scrollable extends StatefulWidget {
  /// Creates a widget that scrolls.
  const Scrollable({
    super.key,
    this.axisDirection = AxisDirection.down,
    this.controller,
    this.physics,
    required this.viewportBuilder,
    this.incrementCalculator,
    this.excludeFromSemantics = false,
    this.semanticChildCount,
    this.dragStartBehavior = DragStartBehavior.start,
    this.restorationId,
    this.scrollBehavior,
    this.clipBehavior = Clip.hardEdge,
    this.hitTestBehavior = HitTestBehavior.opaque,
  }) : assert(semanticChildCount == null || semanticChildCount >= 0);

  /// {@template flutter.widgets.Scrollable.axisDirection}
  /// The direction in which this widget scrolls.
  ///
  /// For example, if the [Scrollable.axisDirection] is [AxisDirection.down],
  /// increasing the scroll position will cause content below the bottom of the
  /// viewport to become visible through the viewport. Similarly, if the
  /// axisDirection is [AxisDirection.right], increasing the scroll position
  /// will cause content beyond the right edge of the viewport to become visible
  /// through the viewport.
  ///
  /// Defaults to [AxisDirection.down].
  /// {@endtemplate}
  final AxisDirection axisDirection;

  /// {@template flutter.widgets.Scrollable.controller}
  /// An object that can be used to control the position to which this widget is
  /// scrolled.
  ///
  /// A [ScrollController] serves several purposes. It can be used to control
  /// the initial scroll position (see [ScrollController.initialScrollOffset]).
  /// It can be used to control whether the scroll view should automatically
  /// save and restore its scroll position in the [PageStorage] (see
  /// [ScrollController.keepScrollOffset]). It can be used to read the current
  /// scroll position (see [ScrollController.offset]), or change it (see
  /// [ScrollController.animateTo]).
  ///
  /// If null, a [ScrollController] will be created internally by [Scrollable]
  /// in order to create and manage the [ScrollPosition].
  ///
  /// See also:
  ///
  ///  * [Scrollable.ensureVisible], which animates the scroll position to
  ///    reveal a given [BuildContext].
  /// {@endtemplate}
  final ScrollController? controller;

  /// {@template flutter.widgets.Scrollable.physics}
  /// How the widgets should respond to user input.
  ///
  /// For example, determines how the widget continues to animate after the
  /// user stops dragging the scroll view.
  ///
  /// Defaults to matching platform conventions via the physics provided from
  /// the ambient [ScrollConfiguration].
  ///
  /// If an explicit [ScrollBehavior] is provided to
  /// [Scrollable.scrollBehavior], the [ScrollPhysics] provided by that behavior
  /// will take precedence after [Scrollable.physics].
  ///
  /// The physics can be changed dynamically, but new physics will only take
  /// effect if the _class_ of the provided object changes. Merely constructing
  /// a new instance with a different configuration is insufficient to cause the
  /// physics to be reapplied. (This is because the final object used is
  /// generated dynamically, which can be relatively expensive, and it would be
  /// inefficient to speculatively create this object each frame to see if the
  /// physics should be updated.)
  ///
  /// See also:
  ///
  ///  * [AlwaysScrollableScrollPhysics], which can be used to indicate that the
  ///    scrollable should react to scroll requests (and possible overscroll)
  ///    even if the scrollable's contents fit without scrolling being necessary.
  /// {@endtemplate}
  final ScrollPhysics? physics;

  /// Builds the viewport through which the scrollable content is displayed.
  ///
  /// A typical viewport uses the given [ViewportOffset] to determine which part
  /// of its content is actually visible through the viewport.
  ///
  /// See also:
  ///
  ///  * [Viewport], which is a viewport that displays a list of slivers.
  ///  * [ShrinkWrappingViewport], which is a viewport that displays a list of
  ///    slivers and sizes itself based on the size of the slivers.
  final ViewportBuilder viewportBuilder;

  /// {@template flutter.widgets.Scrollable.incrementCalculator}
  /// An optional function that will be called to calculate the distance to
  /// scroll when the scrollable is asked to scroll via the keyboard using a
  /// [ScrollAction].
  ///
  /// If not supplied, the [Scrollable] will scroll a default amount when a
  /// keyboard navigation key is pressed (e.g. pageUp/pageDown, control-upArrow,
  /// etc.), or otherwise invoked by a [ScrollAction].
  ///
  /// If [incrementCalculator] is null, the default for
  /// [ScrollIncrementType.page] is 80% of the size of the scroll window, and
  /// for [ScrollIncrementType.line], 50 logical pixels.
  /// {@endtemplate}
  final ScrollIncrementCalculator? incrementCalculator;

  /// {@template flutter.widgets.scrollable.excludeFromSemantics}
  /// Whether the scroll actions introduced by this [Scrollable] are exposed
  /// in the semantics tree.
  ///
  /// Text fields with an overflow are usually scrollable to make sure that the
  /// user can get to the beginning/end of the entered text. However, these
  /// scrolling actions are generally not exposed to the semantics layer.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  ///  * [GestureDetector.excludeFromSemantics], which is used to accomplish the
  ///    exclusion.
  final bool excludeFromSemantics;

  /// {@template flutter.widgets.scrollable.hitTestBehavior}
  /// Defines the behavior of gesture detector used in this [Scrollable].
  ///
  /// This defaults to [HitTestBehavior.opaque] which means it prevents targets
  /// behind this [Scrollable] from receiving events.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  ///  * [HitTestBehavior], for an explanation on different behaviors.
  final HitTestBehavior hitTestBehavior;

  /// The number of children that will contribute semantic information.
  ///
  /// The value will be null if the number of children is unknown or unbounded.
  ///
  /// Some subtypes of [ScrollView] can infer this value automatically. For
  /// example [ListView] will use the number of widgets in the child list,
  /// while the [ListView.separated] constructor will use half that amount.
  ///
  /// For [CustomScrollView] and other types which do not receive a builder
  /// or list of widgets, the child count must be explicitly provided.
  ///
  /// See also:
  ///
  ///  * [CustomScrollView], for an explanation of scroll semantics.
  ///  * [SemanticsConfiguration.scrollChildCount], the corresponding semantics property.
  final int? semanticChildCount;

  // TODO(jslavitz): Set the DragStartBehavior default to be start across all widgets.
  /// {@template flutter.widgets.scrollable.dragStartBehavior}
  /// Determines the way that drag start behavior is handled.
  ///
  /// If set to [DragStartBehavior.start], scrolling drag behavior will
  /// begin at the position where the drag gesture won the arena. If set to
  /// [DragStartBehavior.down] it will begin at the position where a down
  /// event is first detected.
  ///
  /// In general, setting this to [DragStartBehavior.start] will make drag
  /// animation smoother and setting it to [DragStartBehavior.down] will make
  /// drag behavior feel slightly more reactive.
  ///
  /// By default, the drag start behavior is [DragStartBehavior.start].
  ///
  /// See also:
  ///
  ///  * [DragGestureRecognizer.dragStartBehavior], which gives an example for
  ///    the different behaviors.
  ///
  /// {@endtemplate}
  final DragStartBehavior dragStartBehavior;

  /// {@template flutter.widgets.scrollable.restorationId}
  /// Restoration ID to save and restore the scroll offset of the scrollable.
  ///
  /// If a restoration id is provided, the scrollable will persist its current
  /// scroll offset and restore it during state restoration.
  ///
  /// The scroll offset is persisted in a [RestorationBucket] claimed from
  /// the surrounding [RestorationScope] using the provided restoration ID.
  ///
  /// See also:
  ///
  ///  * [RestorationManager], which explains how state restoration works in
  ///    Flutter.
  /// {@endtemplate}
  final String? restorationId;

  /// {@template flutter.widgets.scrollable.scrollBehavior}
  /// A [ScrollBehavior] that will be applied to this widget individually.
  ///
  /// Defaults to null, wherein the inherited [ScrollBehavior] is copied and
  /// modified to alter the viewport decoration, like [Scrollbar]s.
  ///
  /// [ScrollBehavior]s also provide [ScrollPhysics]. If an explicit
  /// [ScrollPhysics] is provided in [physics], it will take precedence,
  /// followed by [scrollBehavior], and then the inherited ancestor
  /// [ScrollBehavior].
  /// {@endtemplate}
  final ScrollBehavior? scrollBehavior;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  ///
  /// This is passed to decorators in [ScrollableDetails], and does not directly affect
  /// clipping of the [Scrollable]. This reflects the same [Clip] that is provided
  /// to [ScrollView.clipBehavior] and is supplied to the [Viewport].
  final Clip clipBehavior;

  /// The axis along which the scroll view scrolls.
  ///
  /// Determined by the [axisDirection].
  Axis get axis => axisDirectionToAxis(axisDirection);

  @override
  ScrollableState createState() => ScrollableState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<AxisDirection>('axisDirection', axisDirection));
    properties.add(DiagnosticsProperty<ScrollPhysics>('physics', physics));
    properties.add(StringProperty('restorationId', restorationId));
  }

  /// The state from the closest instance of this class that encloses the given
  /// context, or null if none is found.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// ScrollableState? scrollable = Scrollable.maybeOf(context);
  /// ```
  ///
  /// Calling this method will create a dependency on the [ScrollableState]
  /// that is returned, if there is one. This is typically the closest
  /// [Scrollable], but may be a more distant ancestor if [axis] is used to
  /// target a specific [Scrollable].
  ///
  /// Using the optional [Axis] is useful when Scrollables are nested and the
  /// target [Scrollable] is not the closest instance. When [axis] is provided,
  /// the nearest enclosing [ScrollableState] in that [Axis] is returned, or
  /// null if there is none.
  ///
  /// This finds the nearest _ancestor_ [Scrollable] of the `context`. This
  /// means that if the `context` is that of a [Scrollable], it will _not_ find
  /// _that_ [Scrollable].
  ///
  /// See also:
  ///
  /// * [Scrollable.of], which is similar to this method, but asserts
  ///   if no [Scrollable] ancestor is found.
  static ScrollableState? maybeOf(BuildContext context, {Axis? axis}) {
    // This is the context that will need to establish the dependency.
    final BuildContext originalContext = context;
    InheritedElement? element = context.getElementForInheritedWidgetOfExactType<_ScrollableScope>();
    while (element != null) {
      final ScrollableState scrollable = (element.widget as _ScrollableScope).scrollable;
      if (axis == null || axisDirectionToAxis(scrollable.axisDirection) == axis) {
        // Establish the dependency on the correct context.
        originalContext.dependOnInheritedElement(element);
        return scrollable;
      }
      context = scrollable.context;
      element = context.getElementForInheritedWidgetOfExactType<_ScrollableScope>();
    }
    return null;
  }

  /// The state from the closest instance of this class that encloses the given
  /// context.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// ScrollableState scrollable = Scrollable.of(context);
  /// ```
  ///
  /// Calling this method will create a dependency on the [ScrollableState]
  /// that is returned, if there is one. This is typically the closest
  /// [Scrollable], but may be a more distant ancestor if [axis] is used to
  /// target a specific [Scrollable].
  ///
  /// Using the optional [Axis] is useful when Scrollables are nested and the
  /// target [Scrollable] is not the closest instance. When [axis] is provided,
  /// the nearest enclosing [ScrollableState] in that [Axis] is returned.
  ///
  /// This finds the nearest _ancestor_ [Scrollable] of the `context`. This
  /// means that if the `context` is that of a [Scrollable], it will _not_ find
  /// _that_ [Scrollable].
  ///
  /// If no [Scrollable] ancestor is found, then this method will assert in
  /// debug mode, and throw an exception in release mode.
  ///
  /// See also:
  ///
  /// * [Scrollable.maybeOf], which is similar to this method, but returns null
  ///   if no [Scrollable] ancestor is found.
  static ScrollableState of(BuildContext context, {Axis? axis}) {
    final ScrollableState? scrollableState = maybeOf(context, axis: axis);
    assert(() {
      if (scrollableState == null) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'Scrollable.of() was called with a context that does not contain a '
            'Scrollable widget.',
          ),
          ErrorDescription(
            'No Scrollable widget ancestor could be found '
            '${axis == null ? '' : 'for the provided Axis: $axis '}'
            'starting from the context that was passed to Scrollable.of(). This '
            'can happen because you are using a widget that looks for a Scrollable '
            'ancestor, but no such ancestor exists.\n'
            'The context used was:\n'
            '  $context',
          ),
          if (axis != null)
            ErrorHint(
              'When specifying an axis, this method will only look for a Scrollable '
              'that matches the given Axis.',
            ),
        ]);
      }
      return true;
    }());
    return scrollableState!;
  }

  /// Provides a heuristic to determine if expensive frame-bound tasks should be
  /// deferred for the [context] at a specific point in time.
  ///
  /// Calling this method does _not_ create a dependency on any other widget.
  /// This also means that the value returned is only good for the point in time
  /// when it is called, and callers will not get updated if the value changes.
  ///
  /// The heuristic used is determined by the [physics] of this [Scrollable]
  /// via [ScrollPhysics.recommendDeferredLoading]. That method is called with
  /// the current [ScrollPosition.activity]'s [ScrollActivity.velocity].
  ///
  /// The optional [Axis] allows targeting of a specific [Scrollable] of that
  /// axis, useful when Scrollables are nested. When [axis] is provided,
  /// [ScrollPosition.recommendDeferredLoading] is called for the nearest
  /// [Scrollable] in that [Axis].
  ///
  /// If there is no [Scrollable] in the widget tree above the [context], this
  /// method returns false.
  static bool recommendDeferredLoadingForContext(BuildContext context, {Axis? axis}) {
    _ScrollableScope? widget = context.getInheritedWidgetOfExactType<_ScrollableScope>();
    while (widget != null) {
      if (axis == null || axisDirectionToAxis(widget.scrollable.axisDirection) == axis) {
        return widget.position.recommendDeferredLoading(context);
      }
      context = widget.scrollable.context;
      widget = context.getInheritedWidgetOfExactType<_ScrollableScope>();
    }
    return false;
  }

  /// Scrolls all scrollables that enclose the given context so as to make the
  /// given context visible.
  ///
  /// If a [Scrollable] enclosing the provided [BuildContext] is a
  /// [TwoDimensionalScrollable], both vertical and horizontal axes will ensure
  /// the target is made visible.
  static Future<void> ensureVisible(
    BuildContext context, {
    double alignment = 0.0,
    Duration duration = Duration.zero,
    Curve curve = Curves.ease,
    ScrollPositionAlignmentPolicy alignmentPolicy = ScrollPositionAlignmentPolicy.explicit,
  }) {
    final List<Future<void>> futures = <Future<void>>[];

    // The targetRenderObject is used to record the first target renderObject.
    // If there are multiple scrollable widgets nested, the targetRenderObject
    // is made to be as visible as possible to improve the user experience. If
    // the targetRenderObject is already visible, then let the outer
    // renderObject be as visible as possible.
    //
    // Also see https://github.com/flutter/flutter/issues/65100
    RenderObject? targetRenderObject;
    ScrollableState? scrollable = Scrollable.maybeOf(context);
    while (scrollable != null) {
      final List<Future<void>> newFutures;
      (newFutures, scrollable) = scrollable._performEnsureVisible(
        context.findRenderObject()!,
        alignment: alignment,
        duration: duration,
        curve: curve,
        alignmentPolicy: alignmentPolicy,
        targetRenderObject: targetRenderObject,
      );
      futures.addAll(newFutures);

      targetRenderObject ??= context.findRenderObject();
      context = scrollable.context;
      scrollable = Scrollable.maybeOf(context);
    }

    if (futures.isEmpty || duration == Duration.zero) {
      return Future<void>.value();
    }
    if (futures.length == 1) {
      return futures.single;
    }
    return Future.wait<void>(futures).then<void>((List<void> _) => null);
  }
}

// Enable Scrollable.of() to work as if ScrollableState was an inherited widget.
// ScrollableState.build() always rebuilds its _ScrollableScope.
class _ScrollableScope extends InheritedWidget {
  const _ScrollableScope({required this.scrollable, required this.position, required super.child});

  final ScrollableState scrollable;
  final ScrollPosition position;

  @override
  bool updateShouldNotify(_ScrollableScope old) {
    return position != old.position;
  }
}

/// State object for a [Scrollable] widget.
///
/// To manipulate a [Scrollable] widget's scroll position, use the object
/// obtained from the [position] property.
///
/// To be informed of when a [Scrollable] widget is scrolling, use a
/// [NotificationListener] to listen for [ScrollNotification] notifications.
///
/// This class is not intended to be subclassed. To specialize the behavior of a
/// [Scrollable], provide it with a [ScrollPhysics].
class ScrollableState extends State<Scrollable>
    with TickerProviderStateMixin, RestorationMixin
    implements ScrollContext {
  // GETTERS

  /// The manager for this [Scrollable] widget's viewport position.
  ///
  /// To control what kind of [ScrollPosition] is created for a [Scrollable],
  /// provide it with custom [ScrollController] that creates the appropriate
  /// [ScrollPosition] in its [ScrollController.createScrollPosition] method.
  ScrollPosition get position => _position!;
  ScrollPosition? _position;

  /// The resolved [ScrollPhysics] of the [ScrollableState].
  ScrollPhysics? get resolvedPhysics => _physics;
  ScrollPhysics? _physics;

  /// An [Offset] that represents the absolute distance from the origin, or 0,
  /// of the [ScrollPosition] expressed in the associated [Axis].
  ///
  /// Used by [EdgeDraggingAutoScroller] to progress the position forward when a
  /// drag gesture reaches the edge of the [Viewport].
  Offset get deltaToScrollOrigin => switch (axisDirection) {
    AxisDirection.up => Offset(0, -position.pixels),
    AxisDirection.down => Offset(0, position.pixels),
    AxisDirection.left => Offset(-position.pixels, 0),
    AxisDirection.right => Offset(position.pixels, 0),
  };

  ScrollController get _effectiveScrollController =>
      widget.controller ?? _fallbackScrollController!;

  @override
  AxisDirection get axisDirection => widget.axisDirection;

  @override
  TickerProvider get vsync => this;

  @override
  double get devicePixelRatio => _devicePixelRatio;
  late double _devicePixelRatio;

  @override
  BuildContext? get notificationContext => _gestureDetectorKey.currentContext;

  @override
  BuildContext get storageContext => context;

  @override
  String? get restorationId => widget.restorationId;
  final _RestorableScrollOffset _persistedScrollOffset = _RestorableScrollOffset();

  late ScrollBehavior _configuration;
  ScrollController? _fallbackScrollController;
  DeviceGestureSettings? _mediaQueryGestureSettings;

  // Only call this from places that will definitely trigger a rebuild.
  void _updatePosition() {
    _configuration = widget.scrollBehavior ?? ScrollConfiguration.of(context);
    final ScrollPhysics? physicsFromWidget =
        widget.physics ?? widget.scrollBehavior?.getScrollPhysics(context);
    _physics = _configuration.getScrollPhysics(context);
    _physics = physicsFromWidget?.applyTo(_physics) ?? _physics;

    final ScrollPosition? oldPosition = _position;
    if (oldPosition != null) {
      _effectiveScrollController.detach(oldPosition);
      // It's important that we not dispose the old position until after the
      // viewport has had a chance to unregister its listeners from the old
      // position. So, schedule a microtask to do it.
      scheduleMicrotask(oldPosition.dispose);
    }

    _position = _effectiveScrollController.createScrollPosition(_physics!, this, oldPosition);
    assert(_position != null);
    _effectiveScrollController.attach(position);
  }

  @protected
  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_persistedScrollOffset, 'offset');
    assert(_position != null);
    if (_persistedScrollOffset.value != null) {
      position.restoreOffset(_persistedScrollOffset.value!, initialRestore: initialRestore);
    }
  }

  @protected
  @override
  void saveOffset(double offset) {
    assert(debugIsSerializableForRestoration(offset));
    _persistedScrollOffset.value = offset;
    // [saveOffset] is called after a scrolling ends and it is usually not
    // followed by a frame. Therefore, manually flush restoration data.
    ServicesBinding.instance.restorationManager.flushData();
  }

  @protected
  @override
  void initState() {
    if (widget.controller == null) {
      _fallbackScrollController = ScrollController();
    }
    super.initState();
  }

  @protected
  @override
  void didChangeDependencies() {
    _mediaQueryGestureSettings = MediaQuery.maybeGestureSettingsOf(context);
    _devicePixelRatio =
        MediaQuery.maybeDevicePixelRatioOf(context) ?? View.of(context).devicePixelRatio;
    _updatePosition();
    super.didChangeDependencies();
  }

  bool _shouldUpdatePosition(Scrollable oldWidget) {
    if ((widget.scrollBehavior == null) != (oldWidget.scrollBehavior == null)) {
      return true;
    }
    if (widget.scrollBehavior != null &&
        oldWidget.scrollBehavior != null &&
        widget.scrollBehavior!.shouldNotify(oldWidget.scrollBehavior!)) {
      return true;
    }
    ScrollPhysics? newPhysics = widget.physics ?? widget.scrollBehavior?.getScrollPhysics(context);
    ScrollPhysics? oldPhysics =
        oldWidget.physics ?? oldWidget.scrollBehavior?.getScrollPhysics(context);
    do {
      if (newPhysics?.runtimeType != oldPhysics?.runtimeType) {
        return true;
      }
      newPhysics = newPhysics?.parent;
      oldPhysics = oldPhysics?.parent;
    } while (newPhysics != null || oldPhysics != null);

    return widget.controller?.runtimeType != oldWidget.controller?.runtimeType;
  }

  @protected
  @override
  void didUpdateWidget(Scrollable oldWidget) {
    super.didUpdateWidget(oldWidget);

    if (widget.controller != oldWidget.controller) {
      if (oldWidget.controller == null) {
        // The old controller was null, meaning the fallback cannot be null.
        // Dispose of the fallback.
        assert(_fallbackScrollController != null);
        assert(widget.controller != null);
        _fallbackScrollController!.detach(position);
        _fallbackScrollController!.dispose();
        _fallbackScrollController = null;
      } else {
        // The old controller was not null, detach.
        oldWidget.controller?.detach(position);
        if (widget.controller == null) {
          // If the new controller is null, we need to set up the fallback
          // ScrollController.
          _fallbackScrollController = ScrollController();
        }
      }
      // Attach the updated effective scroll controller.
      _effectiveScrollController.attach(position);
    }

    if (_shouldUpdatePosition(oldWidget)) {
      _updatePosition();
    }
  }

  @protected
  @override
  void dispose() {
    if (widget.controller != null) {
      widget.controller!.detach(position);
    } else {
      _fallbackScrollController?.detach(position);
      _fallbackScrollController?.dispose();
    }

    position.dispose();
    _persistedScrollOffset.dispose();
    super.dispose();
  }

  // SEMANTICS

  final GlobalKey _scrollSemanticsKey = GlobalKey();

  @override
  @protected
  void setSemanticsActions(Set<SemanticsAction> actions) {
    if (_gestureDetectorKey.currentState != null) {
      _gestureDetectorKey.currentState!.replaceSemanticsActions(actions);
    }
  }

  // GESTURE RECOGNITION AND POINTER IGNORING

  final GlobalKey<RawGestureDetectorState> _gestureDetectorKey =
      GlobalKey<RawGestureDetectorState>();
  final GlobalKey _ignorePointerKey = GlobalKey();

  // This field is set during layout, and then reused until the next time it is set.
  Map<Type, GestureRecognizerFactory> _gestureRecognizers =
      const <Type, GestureRecognizerFactory>{};
  bool _shouldIgnorePointer = false;

  bool? _lastCanDrag;
  Axis? _lastAxisDirection;

  @override
  @protected
  void setCanDrag(bool value) {
    if (value == _lastCanDrag && (!value || widget.axis == _lastAxisDirection)) {
      return;
    }
    if (!value) {
      _gestureRecognizers = const <Type, GestureRecognizerFactory>{};
      // Cancel the active hold/drag (if any) because the gesture recognizers
      // will soon be disposed by our RawGestureDetector, and we won't be
      // receiving pointer up events to cancel the hold/drag.
      _handleDragCancel();
    } else {
      switch (widget.axis) {
        case Axis.vertical:
          _gestureRecognizers = <Type, GestureRecognizerFactory>{
            VerticalDragGestureRecognizer:
                GestureRecognizerFactoryWithHandlers<VerticalDragGestureRecognizer>(
                  () => VerticalDragGestureRecognizer(supportedDevices: _configuration.dragDevices),
                  (VerticalDragGestureRecognizer instance) {
                    instance
                      ..onDown = _handleDragDown
                      ..onStart = _handleDragStart
                      ..onUpdate = _handleDragUpdate
                      ..onEnd = _handleDragEnd
                      ..onCancel = _handleDragCancel
                      ..minFlingDistance = _physics?.minFlingDistance
                      ..minFlingVelocity = _physics?.minFlingVelocity
                      ..maxFlingVelocity = _physics?.maxFlingVelocity
                      ..velocityTrackerBuilder = _configuration.velocityTrackerBuilder(context)
                      ..dragStartBehavior = widget.dragStartBehavior
                      ..multitouchDragStrategy = _configuration.getMultitouchDragStrategy(context)
                      ..gestureSettings = _mediaQueryGestureSettings
                      ..supportedDevices = _configuration.dragDevices;
                  },
                ),
          };
        case Axis.horizontal:
          _gestureRecognizers = <Type, GestureRecognizerFactory>{
            HorizontalDragGestureRecognizer:
                GestureRecognizerFactoryWithHandlers<HorizontalDragGestureRecognizer>(
                  () =>
                      HorizontalDragGestureRecognizer(supportedDevices: _configuration.dragDevices),
                  (HorizontalDragGestureRecognizer instance) {
                    instance
                      ..onDown = _handleDragDown
                      ..onStart = _handleDragStart
                      ..onUpdate = _handleDragUpdate
                      ..onEnd = _handleDragEnd
                      ..onCancel = _handleDragCancel
                      ..minFlingDistance = _physics?.minFlingDistance
                      ..minFlingVelocity = _physics?.minFlingVelocity
                      ..maxFlingVelocity = _physics?.maxFlingVelocity
                      ..velocityTrackerBuilder = _configuration.velocityTrackerBuilder(context)
                      ..dragStartBehavior = widget.dragStartBehavior
                      ..multitouchDragStrategy = _configuration.getMultitouchDragStrategy(context)
                      ..gestureSettings = _mediaQueryGestureSettings
                      ..supportedDevices = _configuration.dragDevices;
                  },
                ),
          };
      }
    }
    _lastCanDrag = value;
    _lastAxisDirection = widget.axis;
    if (_gestureDetectorKey.currentState != null) {
      _gestureDetectorKey.currentState!.replaceGestureRecognizers(_gestureRecognizers);
    }
  }

  @override
  @protected
  void setIgnorePointer(bool value) {
    if (_shouldIgnorePointer == value) {
      return;
    }
    _shouldIgnorePointer = value;
    if (_ignorePointerKey.currentContext != null) {
      final RenderIgnorePointer renderBox =
          _ignorePointerKey.currentContext!.findRenderObject()! as RenderIgnorePointer;
      renderBox.ignoring = _shouldIgnorePointer;
    }
  }

  // TOUCH HANDLERS

  Drag? _drag;
  ScrollHoldController? _hold;

  void _handleDragDown(DragDownDetails details) {
    assert(_drag == null);
    assert(_hold == null);
    _hold = position.hold(_disposeHold);
  }

  void _handleDragStart(DragStartDetails details) {
    // It's possible for _hold to become null between _handleDragDown and
    // _handleDragStart, for example if some user code calls jumpTo or otherwise
    // triggers a new activity to begin.
    assert(_drag == null);
    _drag = position.drag(details, _disposeDrag);
    assert(_drag != null);
    assert(_hold == null);
  }

  void _handleDragUpdate(DragUpdateDetails details) {
    // _drag might be null if the drag activity ended and called _disposeDrag.
    assert(_hold == null || _drag == null);
    _drag?.update(details);
  }

  void _handleDragEnd(DragEndDetails details) {
    // _drag might be null if the drag activity ended and called _disposeDrag.
    assert(_hold == null || _drag == null);
    _drag?.end(details);
    assert(_drag == null);
  }

  void _handleDragCancel() {
    if (_gestureDetectorKey.currentContext == null) {
      // The cancel was caused by the GestureDetector getting disposed, which
      // means we will get disposed momentarily as well and shouldn't do
      // any work.
      return;
    }
    // _hold might be null if the drag started.
    // _drag might be null if the drag activity ended and called _disposeDrag.
    assert(_hold == null || _drag == null);
    _hold?.cancel();
    _drag?.cancel();
    assert(_hold == null);
    assert(_drag == null);
  }

  void _disposeHold() {
    _hold = null;
  }

  void _disposeDrag() {
    _drag = null;
  }

  // SCROLL WHEEL

  // Returns the offset that should result from applying [event] to the current
  // position, taking min/max scroll extent into account.
  double _targetScrollOffsetForPointerScroll(double delta) {
    return math.min(
      math.max(position.pixels + delta, position.minScrollExtent),
      position.maxScrollExtent,
    );
  }

  // Returns the delta that should result from applying [event] with axis,
  // direction, and any modifiers specified by the ScrollBehavior taken into
  // account.
  double _pointerSignalEventDelta(PointerScrollEvent event) {
    final Set<LogicalKeyboardKey> pressed = HardwareKeyboard.instance.logicalKeysPressed;
    final bool flipAxes =
        pressed.any(_configuration.pointerAxisModifiers.contains) &&
        // Axes are only flipped for physical mouse wheel input.
        // On some platforms, like web, trackpad input is handled through pointer
        // signals, but should not be included in this axis modifying behavior.
        // This is because on a trackpad, all directional axes are available to
        // the user, while mouse scroll wheels typically are restricted to one
        // axis.
        event.kind == PointerDeviceKind.mouse;

    final Axis axis = flipAxes ? flipAxis(widget.axis) : widget.axis;
    final double delta = switch (axis) {
      Axis.horizontal => event.scrollDelta.dx,
      Axis.vertical => event.scrollDelta.dy,
    };

    return axisDirectionIsReversed(widget.axisDirection) ? -delta : delta;
  }

  void _receivedPointerSignal(PointerSignalEvent event) {
    if (event is PointerScrollEvent && _position != null) {
      if (_physics != null && !_physics!.shouldAcceptUserOffset(position)) {
        // The handler won't use the `event`, so allow the platform to trigger
        // any default native actions.
        event.respond(allowPlatformDefault: true);
        return;
      }
      final double delta = _pointerSignalEventDelta(event);
      final double targetScrollOffset = _targetScrollOffsetForPointerScroll(delta);
      // Only express interest in the event if it would actually result in a scroll.
      if (delta != 0.0 && targetScrollOffset != position.pixels) {
        GestureBinding.instance.pointerSignalResolver.register(event, _handlePointerScroll);
        return;
      }
      // The `event` won't result in a scroll, so allow the platform to trigger
      // any default native actions.
      event.respond(allowPlatformDefault: true);
    } else if (event is PointerScrollInertiaCancelEvent) {
      position.pointerScroll(0);
      // Don't use the pointer signal resolver, all hit-tested scrollables should stop.
    }
  }

  void _handlePointerScroll(PointerEvent event) {
    assert(event is PointerScrollEvent);
    final double delta = _pointerSignalEventDelta(event as PointerScrollEvent);
    final double targetScrollOffset = _targetScrollOffsetForPointerScroll(delta);
    if (delta != 0.0 && targetScrollOffset != position.pixels) {
      position.pointerScroll(delta);
    }
  }

  bool _handleScrollMetricsNotification(ScrollMetricsNotification notification) {
    if (notification.depth == 0) {
      final RenderObject? scrollSemanticsRenderObject =
          _scrollSemanticsKey.currentContext?.findRenderObject();
      if (scrollSemanticsRenderObject != null) {
        scrollSemanticsRenderObject.markNeedsSemanticsUpdate();
      }
    }
    return false;
  }

  Widget _buildChrome(BuildContext context, Widget child) {
    final ScrollableDetails details = ScrollableDetails(
      direction: widget.axisDirection,
      controller: _effectiveScrollController,
      decorationClipBehavior: widget.clipBehavior,
    );

    return _configuration.buildScrollbar(
      context,
      _configuration.buildOverscrollIndicator(context, child, details),
      details,
    );
  }

  // DESCRIPTION

  @protected
  @override
  Widget build(BuildContext context) {
    assert(_position != null);
    // _ScrollableScope must be placed above the BuildContext returned by notificationContext
    // so that we can get this ScrollableState by doing the following:
    //
    // ScrollNotification notification;
    // Scrollable.of(notification.context)
    //
    // Since notificationContext is pointing to _gestureDetectorKey.context, _ScrollableScope
    // must be placed above the widget using it: RawGestureDetector
    Widget result = _ScrollableScope(
      scrollable: this,
      position: position,
      child: Listener(
        onPointerSignal: _receivedPointerSignal,
        child: RawGestureDetector(
          key: _gestureDetectorKey,
          gestures: _gestureRecognizers,
          behavior: widget.hitTestBehavior,
          excludeFromSemantics: widget.excludeFromSemantics,
          child: Semantics(
            explicitChildNodes: !widget.excludeFromSemantics,
            child: IgnorePointer(
              key: _ignorePointerKey,
              ignoring: _shouldIgnorePointer,
              child: widget.viewportBuilder(context, position),
            ),
          ),
        ),
      ),
    );

    if (!widget.excludeFromSemantics) {
      result = NotificationListener<ScrollMetricsNotification>(
        onNotification: _handleScrollMetricsNotification,
        child: _ScrollSemantics(
          key: _scrollSemanticsKey,
          position: position,
          allowImplicitScrolling: _physics!.allowImplicitScrolling,
          axis: widget.axis,
          semanticChildCount: widget.semanticChildCount,
          child: result,
        ),
      );
    }

    result = _buildChrome(context, result);

    // Selection is only enabled when there is a parent registrar.
    final SelectionRegistrar? registrar = SelectionContainer.maybeOf(context);
    if (registrar != null) {
      result = _ScrollableSelectionHandler(
        state: this,
        position: position,
        registrar: registrar,
        child: result,
      );
    }

    return result;
  }

  // Returns the Future from calling ensureVisible for the ScrollPosition, as
  // as well as this ScrollableState instance so its context can be used to
  // check for other ancestor Scrollables in executing ensureVisible.
  _EnsureVisibleResults _performEnsureVisible(
    RenderObject object, {
    double alignment = 0.0,
    Duration duration = Duration.zero,
    Curve curve = Curves.ease,
    ScrollPositionAlignmentPolicy alignmentPolicy = ScrollPositionAlignmentPolicy.explicit,
    RenderObject? targetRenderObject,
  }) {
    final Future<void> ensureVisibleFuture = position.ensureVisible(
      object,
      alignment: alignment,
      duration: duration,
      curve: curve,
      alignmentPolicy: alignmentPolicy,
      targetRenderObject: targetRenderObject,
    );
    return (<Future<void>>[ensureVisibleFuture], this);
  }

  @protected
  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<ScrollPosition>('position', _position));
    properties.add(DiagnosticsProperty<ScrollPhysics>('effective physics', _physics));
  }
}

/// A widget to handle selection for a scrollable.
///
/// This widget registers itself to the [registrar] and uses
/// [SelectionContainer] to collect selectables from its subtree.
class _ScrollableSelectionHandler extends StatefulWidget {
  const _ScrollableSelectionHandler({
    required this.state,
    required this.position,
    required this.registrar,
    required this.child,
  });

  final ScrollableState state;
  final ScrollPosition position;
  final Widget child;
  final SelectionRegistrar registrar;

  @override
  _ScrollableSelectionHandlerState createState() => _ScrollableSelectionHandlerState();
}

class _ScrollableSelectionHandlerState extends State<_ScrollableSelectionHandler> {
  late _ScrollableSelectionContainerDelegate _selectionDelegate;

  @override
  void initState() {
    super.initState();
    _selectionDelegate = _ScrollableSelectionContainerDelegate(
      state: widget.state,
      position: widget.position,
    );
  }

  @override
  void didUpdateWidget(_ScrollableSelectionHandler oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.position != widget.position) {
      _selectionDelegate.position = widget.position;
    }
  }

  @override
  void dispose() {
    _selectionDelegate.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return SelectionContainer(
      registrar: widget.registrar,
      delegate: _selectionDelegate,
      child: widget.child,
    );
  }
}

/// This updater handles the case where the selectables change frequently, and
/// it optimizes toward scrolling updates.
///
/// It keeps track of the drag start offset relative to scroll origin for every
/// selectable. The records are used to determine whether the selection is up to
/// date with the scroll position when it sends the drag update event to a
/// selectable.
class _ScrollableSelectionContainerDelegate extends MultiSelectableSelectionContainerDelegate {
  _ScrollableSelectionContainerDelegate({required this.state, required ScrollPosition position})
    : _position = position,
      _autoScroller = EdgeDraggingAutoScroller(
        state,
        velocityScalar: _kDefaultSelectToScrollVelocityScalar,
      ) {
    _position.addListener(_scheduleLayoutChange);
  }

  // Pointer drag is a single point, it should not have a size.
  static const double _kDefaultDragTargetSize = 0;

  // An eye-balled value for a smooth scrolling speed.
  static const double _kDefaultSelectToScrollVelocityScalar = 30;

  final ScrollableState state;
  final EdgeDraggingAutoScroller _autoScroller;
  bool _scheduledLayoutChange = false;
  Offset? _currentDragStartRelatedToOrigin;
  Offset? _currentDragEndRelatedToOrigin;

  // The scrollable only auto scrolls if the selection starts in the scrollable.
  bool _selectionStartsInScrollable = false;

  ScrollPosition get position => _position;
  ScrollPosition _position;
  set position(ScrollPosition other) {
    if (other == _position) {
      return;
    }
    _position.removeListener(_scheduleLayoutChange);
    _position = other;
    _position.addListener(_scheduleLayoutChange);
  }

  // The layout will only be updated a frame later than position changes.
  // Schedule PostFrameCallback to capture the accurate layout.
  void _scheduleLayoutChange() {
    if (_scheduledLayoutChange) {
      return;
    }
    _scheduledLayoutChange = true;
    SchedulerBinding.instance.addPostFrameCallback((Duration timeStamp) {
      if (!_scheduledLayoutChange) {
        return;
      }
      _scheduledLayoutChange = false;
      layoutDidChange();
    }, debugLabel: 'ScrollableSelectionContainer.layoutDidChange');
  }

  /// Stores the scroll offset when a scrollable receives the last
  /// [SelectionEdgeUpdateEvent].
  ///
  /// The stored scroll offset may be null if a scrollable never receives a
  /// [SelectionEdgeUpdateEvent].
  ///
  /// When a new [SelectionEdgeUpdateEvent] is dispatched to a selectable, this
  /// updater checks the current scroll offset against the one stored in these
  /// records. If the scroll offset is different, it synthesizes an opposite
  /// [SelectionEdgeUpdateEvent] and dispatches the event before dispatching the
  /// new event.
  ///
  /// For example, if a selectable receives an end [SelectionEdgeUpdateEvent]
  /// and its scroll offset in the records is different from the current value,
  /// it synthesizes a start [SelectionEdgeUpdateEvent] and dispatches it before
  /// dispatching the original end [SelectionEdgeUpdateEvent].
  final Map<Selectable, double> _selectableStartEdgeUpdateRecords = <Selectable, double>{};
  final Map<Selectable, double> _selectableEndEdgeUpdateRecords = <Selectable, double>{};

  @override
  void didChangeSelectables() {
    final Set<Selectable> selectableSet = selectables.toSet();
    _selectableStartEdgeUpdateRecords.removeWhere(
      (Selectable key, double value) => !selectableSet.contains(key),
    );
    _selectableEndEdgeUpdateRecords.removeWhere(
      (Selectable key, double value) => !selectableSet.contains(key),
    );
    super.didChangeSelectables();
  }

  @override
  SelectionResult handleClearSelection(ClearSelectionEvent event) {
    _selectableStartEdgeUpdateRecords.clear();
    _selectableEndEdgeUpdateRecords.clear();
    _currentDragStartRelatedToOrigin = null;
    _currentDragEndRelatedToOrigin = null;
    _selectionStartsInScrollable = false;
    return super.handleClearSelection(event);
  }

  @override
  SelectionResult handleSelectionEdgeUpdate(SelectionEdgeUpdateEvent event) {
    if (_currentDragEndRelatedToOrigin == null && _currentDragStartRelatedToOrigin == null) {
      assert(!_selectionStartsInScrollable);
      _selectionStartsInScrollable = _globalPositionInScrollable(event.globalPosition);
    }
    final Offset deltaToOrigin = _getDeltaToScrollOrigin(state);
    if (event.type == SelectionEventType.endEdgeUpdate) {
      _currentDragEndRelatedToOrigin = _inferPositionRelatedToOrigin(event.globalPosition);
      final Offset endOffset = _currentDragEndRelatedToOrigin!.translate(
        -deltaToOrigin.dx,
        -deltaToOrigin.dy,
      );
      event = SelectionEdgeUpdateEvent.forEnd(
        globalPosition: endOffset,
        granularity: event.granularity,
      );
    } else {
      _currentDragStartRelatedToOrigin = _inferPositionRelatedToOrigin(event.globalPosition);
      final Offset startOffset = _currentDragStartRelatedToOrigin!.translate(
        -deltaToOrigin.dx,
        -deltaToOrigin.dy,
      );
      event = SelectionEdgeUpdateEvent.forStart(
        globalPosition: startOffset,
        granularity: event.granularity,
      );
    }
    final SelectionResult result = super.handleSelectionEdgeUpdate(event);

    // Result may be pending if one of the selectable child is also a scrollable.
    // In that case, the parent scrollable needs to wait for the child to finish
    // scrolling.
    if (result == SelectionResult.pending) {
      _autoScroller.stopAutoScroll();
      return result;
    }
    if (_selectionStartsInScrollable) {
      _autoScroller.startAutoScrollIfNecessary(_dragTargetFromEvent(event));
      if (_autoScroller.scrolling) {
        return SelectionResult.pending;
      }
    }
    return result;
  }

  Offset _inferPositionRelatedToOrigin(Offset globalPosition) {
    final RenderBox box = state.context.findRenderObject()! as RenderBox;
    final Offset localPosition = box.globalToLocal(globalPosition);
    if (!_selectionStartsInScrollable) {
      // If the selection starts outside of the scrollable, selecting across the
      // scrollable boundary will act as selecting the entire content in the
      // scrollable. This logic move the offset to the 0.0 or infinity to cover
      // the entire content if the input position is outside of the scrollable.
      if (localPosition.dy < 0 || localPosition.dx < 0) {
        return box.localToGlobal(Offset.zero);
      }
      if (localPosition.dy > box.size.height || localPosition.dx > box.size.width) {
        return Offset.infinite;
      }
    }
    final Offset deltaToOrigin = _getDeltaToScrollOrigin(state);
    return box.localToGlobal(localPosition.translate(deltaToOrigin.dx, deltaToOrigin.dy));
  }

  /// Infers the [_currentDragStartRelatedToOrigin] and
  /// [_currentDragEndRelatedToOrigin] from the geometry.
  ///
  /// This method is called after a select word and select all event where the
  /// selection is triggered by none drag events. The
  /// [_currentDragStartRelatedToOrigin] and [_currentDragEndRelatedToOrigin]
  /// are essential to handle future [SelectionEdgeUpdateEvent]s.
  void _updateDragLocationsFromGeometries({
    bool forceUpdateStart = true,
    bool forceUpdateEnd = true,
  }) {
    final Offset deltaToOrigin = _getDeltaToScrollOrigin(state);
    final RenderBox box = state.context.findRenderObject()! as RenderBox;
    final Matrix4 transform = box.getTransformTo(null);
    if (currentSelectionStartIndex != -1 &&
        (_currentDragStartRelatedToOrigin == null || forceUpdateStart)) {
      final SelectionGeometry geometry = selectables[currentSelectionStartIndex].value;
      assert(geometry.hasSelection);
      final SelectionPoint start = geometry.startSelectionPoint!;
      final Matrix4 childTransform = selectables[currentSelectionStartIndex].getTransformTo(box);
      final Offset localDragStart = MatrixUtils.transformPoint(
        childTransform,
        start.localPosition + Offset(0, -start.lineHeight / 2),
      );
      _currentDragStartRelatedToOrigin = MatrixUtils.transformPoint(
        transform,
        localDragStart + deltaToOrigin,
      );
    }
    if (currentSelectionEndIndex != -1 &&
        (_currentDragEndRelatedToOrigin == null || forceUpdateEnd)) {
      final SelectionGeometry geometry = selectables[currentSelectionEndIndex].value;
      assert(geometry.hasSelection);
      final SelectionPoint end = geometry.endSelectionPoint!;
      final Matrix4 childTransform = selectables[currentSelectionEndIndex].getTransformTo(box);
      final Offset localDragEnd = MatrixUtils.transformPoint(
        childTransform,
        end.localPosition + Offset(0, -end.lineHeight / 2),
      );
      _currentDragEndRelatedToOrigin = MatrixUtils.transformPoint(
        transform,
        localDragEnd + deltaToOrigin,
      );
    }
  }

  @override
  SelectionResult handleSelectAll(SelectAllSelectionEvent event) {
    assert(!_selectionStartsInScrollable);
    final SelectionResult result = super.handleSelectAll(event);
    assert((currentSelectionStartIndex == -1) == (currentSelectionEndIndex == -1));
    if (currentSelectionStartIndex != -1) {
      _updateDragLocationsFromGeometries();
    }
    return result;
  }

  @override
  SelectionResult handleSelectWord(SelectWordSelectionEvent event) {
    _selectionStartsInScrollable = _globalPositionInScrollable(event.globalPosition);
    final SelectionResult result = super.handleSelectWord(event);
    _updateDragLocationsFromGeometries();
    return result;
  }

  @override
  SelectionResult handleGranularlyExtendSelection(GranularlyExtendSelectionEvent event) {
    final SelectionResult result = super.handleGranularlyExtendSelection(event);
    // The selection geometry may not have the accurate offset for the edges
    // that are outside of the viewport whose transform may not be valid. Only
    // the edge this event is updating is sure to be accurate.
    _updateDragLocationsFromGeometries(forceUpdateStart: !event.isEnd, forceUpdateEnd: event.isEnd);
    if (_selectionStartsInScrollable) {
      _jumpToEdge(event.isEnd);
    }
    return result;
  }

  @override
  SelectionResult handleDirectionallyExtendSelection(DirectionallyExtendSelectionEvent event) {
    final SelectionResult result = super.handleDirectionallyExtendSelection(event);
    // The selection geometry may not have the accurate offset for the edges
    // that are outside of the viewport whose transform may not be valid. Only
    // the edge this event is updating is sure to be accurate.
    _updateDragLocationsFromGeometries(forceUpdateStart: !event.isEnd, forceUpdateEnd: event.isEnd);
    if (_selectionStartsInScrollable) {
      _jumpToEdge(event.isEnd);
    }
    return result;
  }

  void _jumpToEdge(bool isExtent) {
    final Selectable selectable;
    final double? lineHeight;
    final SelectionPoint? edge;
    if (isExtent) {
      selectable = selectables[currentSelectionEndIndex];
      edge = selectable.value.endSelectionPoint;
      lineHeight = selectable.value.endSelectionPoint!.lineHeight;
    } else {
      selectable = selectables[currentSelectionStartIndex];
      edge = selectable.value.startSelectionPoint;
      lineHeight = selectable.value.startSelectionPoint?.lineHeight;
    }
    if (lineHeight == null || edge == null) {
      return;
    }
    final RenderBox scrollableBox = state.context.findRenderObject()! as RenderBox;
    final Matrix4 transform = selectable.getTransformTo(scrollableBox);
    final Offset edgeOffsetInScrollableCoordinates = MatrixUtils.transformPoint(
      transform,
      edge.localPosition,
    );
    final Rect scrollableRect = Rect.fromLTRB(
      0,
      0,
      scrollableBox.size.width,
      scrollableBox.size.height,
    );
    switch (state.axisDirection) {
      case AxisDirection.up:
        final double edgeBottom = edgeOffsetInScrollableCoordinates.dy;
        final double edgeTop = edgeOffsetInScrollableCoordinates.dy - lineHeight;
        if (edgeBottom >= scrollableRect.bottom && edgeTop <= scrollableRect.top) {
          return;
        }
        if (edgeBottom > scrollableRect.bottom) {
          position.jumpTo(position.pixels + scrollableRect.bottom - edgeBottom);
          return;
        }
        if (edgeTop < scrollableRect.top) {
          position.jumpTo(position.pixels + scrollableRect.top - edgeTop);
        }
        return;
      case AxisDirection.right:
        final double edge = edgeOffsetInScrollableCoordinates.dx;
        if (edge >= scrollableRect.right && edge <= scrollableRect.left) {
          return;
        }
        if (edge > scrollableRect.right) {
          position.jumpTo(position.pixels + edge - scrollableRect.right);
          return;
        }
        if (edge < scrollableRect.left) {
          position.jumpTo(position.pixels + edge - scrollableRect.left);
        }
        return;
      case AxisDirection.down:
        final double edgeBottom = edgeOffsetInScrollableCoordinates.dy;
        final double edgeTop = edgeOffsetInScrollableCoordinates.dy - lineHeight;
        if (edgeBottom >= scrollableRect.bottom && edgeTop <= scrollableRect.top) {
          return;
        }
        if (edgeBottom > scrollableRect.bottom) {
          position.jumpTo(position.pixels + edgeBottom - scrollableRect.bottom);
          return;
        }
        if (edgeTop < scrollableRect.top) {
          position.jumpTo(position.pixels + edgeTop - scrollableRect.top);
        }
        return;
      case AxisDirection.left:
        final double edge = edgeOffsetInScrollableCoordinates.dx;
        if (edge >= scrollableRect.right && edge <= scrollableRect.left) {
          return;
        }
        if (edge > scrollableRect.right) {
          position.jumpTo(position.pixels + scrollableRect.right - edge);
          return;
        }
        if (edge < scrollableRect.left) {
          position.jumpTo(position.pixels + scrollableRect.left - edge);
        }
        return;
    }
  }

  bool _globalPositionInScrollable(Offset globalPosition) {
    final RenderBox box = state.context.findRenderObject()! as RenderBox;
    final Offset localPosition = box.globalToLocal(globalPosition);
    final Rect rect = Rect.fromLTWH(0, 0, box.size.width, box.size.height);
    return rect.contains(localPosition);
  }

  Rect _dragTargetFromEvent(SelectionEdgeUpdateEvent event) {
    return Rect.fromCenter(
      center: event.globalPosition,
      width: _kDefaultDragTargetSize,
      height: _kDefaultDragTargetSize,
    );
  }

  @override
  SelectionResult dispatchSelectionEventToChild(Selectable selectable, SelectionEvent event) {
    switch (event.type) {
      case SelectionEventType.startEdgeUpdate:
        _selectableStartEdgeUpdateRecords[selectable] = state.position.pixels;
        ensureChildUpdated(selectable);
      case SelectionEventType.endEdgeUpdate:
        _selectableEndEdgeUpdateRecords[selectable] = state.position.pixels;
        ensureChildUpdated(selectable);
      case SelectionEventType.granularlyExtendSelection:
      case SelectionEventType.directionallyExtendSelection:
        ensureChildUpdated(selectable);
        _selectableStartEdgeUpdateRecords[selectable] = state.position.pixels;
        _selectableEndEdgeUpdateRecords[selectable] = state.position.pixels;
      case SelectionEventType.clear:
        _selectableEndEdgeUpdateRecords.remove(selectable);
        _selectableStartEdgeUpdateRecords.remove(selectable);
      case SelectionEventType.selectAll:
      case SelectionEventType.selectWord:
      case SelectionEventType.selectParagraph:
        _selectableEndEdgeUpdateRecords[selectable] = state.position.pixels;
        _selectableStartEdgeUpdateRecords[selectable] = state.position.pixels;
    }
    return super.dispatchSelectionEventToChild(selectable, event);
  }

  @override
  void ensureChildUpdated(Selectable selectable) {
    final double newRecord = state.position.pixels;
    final double? previousStartRecord = _selectableStartEdgeUpdateRecords[selectable];
    if (_currentDragStartRelatedToOrigin != null &&
        (previousStartRecord == null ||
            (newRecord - previousStartRecord).abs() > precisionErrorTolerance)) {
      // Make sure the selectable has up to date events.
      final Offset deltaToOrigin = _getDeltaToScrollOrigin(state);
      final Offset startOffset = _currentDragStartRelatedToOrigin!.translate(
        -deltaToOrigin.dx,
        -deltaToOrigin.dy,
      );
      selectable.dispatchSelectionEvent(
        SelectionEdgeUpdateEvent.forStart(globalPosition: startOffset),
      );
      // Make sure we track that we have synthesized a start event for this selectable,
      // so we don't synthesize events unnecessarily.
      _selectableStartEdgeUpdateRecords[selectable] = state.position.pixels;
    }
    final double? previousEndRecord = _selectableEndEdgeUpdateRecords[selectable];
    if (_currentDragEndRelatedToOrigin != null &&
        (previousEndRecord == null ||
            (newRecord - previousEndRecord).abs() > precisionErrorTolerance)) {
      // Make sure the selectable has up to date events.
      final Offset deltaToOrigin = _getDeltaToScrollOrigin(state);
      final Offset endOffset = _currentDragEndRelatedToOrigin!.translate(
        -deltaToOrigin.dx,
        -deltaToOrigin.dy,
      );
      selectable.dispatchSelectionEvent(SelectionEdgeUpdateEvent.forEnd(globalPosition: endOffset));
      // Make sure we track that we have synthesized an end event for this selectable,
      // so we don't synthesize events unnecessarily.
      _selectableEndEdgeUpdateRecords[selectable] = state.position.pixels;
    }
  }

  @override
  void dispose() {
    _selectableStartEdgeUpdateRecords.clear();
    _selectableEndEdgeUpdateRecords.clear();
    _scheduledLayoutChange = false;
    _autoScroller.stopAutoScroll();
    super.dispose();
  }
}

Offset _getDeltaToScrollOrigin(ScrollableState scrollableState) {
  return switch (scrollableState.axisDirection) {
    AxisDirection.up => Offset(0, -scrollableState.position.pixels),
    AxisDirection.down => Offset(0, scrollableState.position.pixels),
    AxisDirection.left => Offset(-scrollableState.position.pixels, 0),
    AxisDirection.right => Offset(scrollableState.position.pixels, 0),
  };
}

/// With [_ScrollSemantics] certain child [SemanticsNode]s can be
/// excluded from the scrollable area for semantics purposes.
///
/// Nodes, that are to be excluded, have to be tagged with
/// [RenderViewport.excludeFromScrolling] and the [RenderAbstractViewport] in
/// use has to add the [RenderViewport.useTwoPaneSemantics] tag to its
/// [SemanticsConfiguration] by overriding
/// [RenderObject.describeSemanticsConfiguration].
///
/// If the tag [RenderViewport.useTwoPaneSemantics] is present on the viewport,
/// two semantics nodes will be used to represent the [Scrollable]: The outer
/// node will contain all children, that are excluded from scrolling. The inner
/// node, which is annotated with the scrolling actions, will house the
/// scrollable children.
class _ScrollSemantics extends SingleChildRenderObjectWidget {
  const _ScrollSemantics({
    super.key,
    required this.position,
    required this.allowImplicitScrolling,
    required this.axis,
    required this.semanticChildCount,
    super.child,
  }) : assert(semanticChildCount == null || semanticChildCount >= 0);

  final ScrollPosition position;
  final bool allowImplicitScrolling;
  final int? semanticChildCount;
  final Axis axis;

  @override
  _RenderScrollSemantics createRenderObject(BuildContext context) {
    return _RenderScrollSemantics(
      position: position,
      allowImplicitScrolling: allowImplicitScrolling,
      semanticChildCount: semanticChildCount,
      axis: axis,
    );
  }

  @override
  void updateRenderObject(BuildContext context, _RenderScrollSemantics renderObject) {
    renderObject
      ..allowImplicitScrolling = allowImplicitScrolling
      ..axis = axis
      ..position = position
      ..semanticChildCount = semanticChildCount;
  }
}

class _RenderScrollSemantics extends RenderProxyBox {
  _RenderScrollSemantics({
    required ScrollPosition position,
    required bool allowImplicitScrolling,
    required this.axis,
    required int? semanticChildCount,
    RenderBox? child,
  }) : _position = position,
       _allowImplicitScrolling = allowImplicitScrolling,
       _semanticChildCount = semanticChildCount,
       super(child) {
    position.addListener(markNeedsSemanticsUpdate);
  }

  /// Whether this render object is excluded from the semantic tree.
  ScrollPosition get position => _position;
  ScrollPosition _position;
  set position(ScrollPosition value) {
    if (value == _position) {
      return;
    }
    _position.removeListener(markNeedsSemanticsUpdate);
    _position = value;
    _position.addListener(markNeedsSemanticsUpdate);
    markNeedsSemanticsUpdate();
  }

  /// Whether this node can be scrolled implicitly.
  bool get allowImplicitScrolling => _allowImplicitScrolling;
  bool _allowImplicitScrolling;
  set allowImplicitScrolling(bool value) {
    if (value == _allowImplicitScrolling) {
      return;
    }
    _allowImplicitScrolling = value;
    markNeedsSemanticsUpdate();
  }

  Axis axis;

  int? get semanticChildCount => _semanticChildCount;
  int? _semanticChildCount;
  set semanticChildCount(int? value) {
    if (value == semanticChildCount) {
      return;
    }
    _semanticChildCount = value;
    markNeedsSemanticsUpdate();
  }

  void _onScrollToOffset(Offset targetOffset) {
    final double offset = switch (axis) {
      Axis.horizontal => targetOffset.dx,
      Axis.vertical => targetOffset.dy,
    };
    _position.jumpTo(offset);
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    config.isSemanticBoundary = true;
    if (position.haveDimensions) {
      config
        ..hasImplicitScrolling = allowImplicitScrolling
        ..scrollPosition = _position.pixels
        ..scrollExtentMax = _position.maxScrollExtent
        ..scrollExtentMin = _position.minScrollExtent
        ..scrollChildCount = semanticChildCount;
      if (position.maxScrollExtent > position.minScrollExtent && allowImplicitScrolling) {
        config.onScrollToOffset = _onScrollToOffset;
      }
    }
  }

  SemanticsNode? _innerNode;

  @override
  void assembleSemanticsNode(
    SemanticsNode node,
    SemanticsConfiguration config,
    Iterable<SemanticsNode> children,
  ) {
    if (children.isEmpty || !children.first.isTagged(RenderViewport.useTwoPaneSemantics)) {
      _innerNode = null;
      super.assembleSemanticsNode(node, config, children);
      return;
    }

    (_innerNode ??= SemanticsNode(showOnScreen: showOnScreen)).rect = node.rect;

    int? firstVisibleIndex;
    final List<SemanticsNode> excluded = <SemanticsNode>[_innerNode!];
    final List<SemanticsNode> included = <SemanticsNode>[];
    for (final SemanticsNode child in children) {
      assert(child.isTagged(RenderViewport.useTwoPaneSemantics));
      if (child.isTagged(RenderViewport.excludeFromScrolling)) {
        excluded.add(child);
      } else {
        if (!child.hasFlag(SemanticsFlag.isHidden)) {
          firstVisibleIndex ??= child.indexInParent;
        }
        included.add(child);
      }
    }
    config.scrollIndex = firstVisibleIndex;
    node.updateWith(config: null, childrenInInversePaintOrder: excluded);
    _innerNode!.updateWith(config: config, childrenInInversePaintOrder: included);
  }

  @override
  void clearSemantics() {
    super.clearSemantics();
    _innerNode = null;
  }
}

// Not using a RestorableDouble because we want to allow null values and override
// [enabled].
class _RestorableScrollOffset extends RestorableValue<double?> {
  @override
  double? createDefaultValue() => null;

  @override
  void didUpdateValue(double? oldValue) {
    notifyListeners();
  }

  @override
  double fromPrimitives(Object? data) {
    return data! as double;
  }

  @override
  Object? toPrimitives() {
    return value;
  }

  @override
  bool get enabled => value != null;
}

// 2D SCROLLING

/// Specifies how to configure the [DragGestureRecognizer]s of a
/// [TwoDimensionalScrollable].
// TODO(Piinks): Add sample code, https://github.com/flutter/flutter/issues/126298
enum DiagonalDragBehavior {
  /// This behavior will not allow for any diagonal scrolling.
  ///
  /// Drag gestures in one direction or the other will lock the input axis until
  /// the gesture is released.
  none,

  /// This behavior will only allow diagonal scrolling on a weighted
  /// scale per gesture event.
  ///
  /// This means that after initially evaluating the drag gesture, the weighted
  /// evaluation (based on [kTouchSlop]) stands until the gesture is released.
  weightedEvent,

  /// This behavior will only allow diagonal scrolling on a weighted
  /// scale that is evaluated throughout a gesture event.
  ///
  /// This means that during each update to the drag gesture, the scrolling
  /// axis will be allowed to scroll diagonally if it exceeds the
  /// [kTouchSlop].
  weightedContinuous,

  /// This behavior allows free movement in any and all directions when
  /// dragging.
  free,
}

/// A widget that manages scrolling in both the vertical and horizontal
/// dimensions and informs the [TwoDimensionalViewport] through which the
/// content is viewed.
///
/// [TwoDimensionalScrollable] implements the interaction model for a scrollable
/// widget in both the vertical and horizontal axes, including gesture
/// recognition, but does not have an opinion about how the
/// [TwoDimensionalViewport], which actually displays the children, is
/// constructed.
///
/// It's rare to construct a [TwoDimensionalScrollable] directly. Instead,
/// consider subclassing [TwoDimensionalScrollView], which combines scrolling,
/// viewporting, and a layout model in both dimensions.
///
/// See also:
///
///  * [TwoDimensionalScrollView], an abstract base class for displaying a
///    scrolling array of children in both directions.
///  * [TwoDimensionalViewport], which can be used to customize the child layout
///    model.
class TwoDimensionalScrollable extends StatefulWidget {
  /// Creates a widget that scrolls in two dimensions.
  ///
  /// The [horizontalDetails], [verticalDetails], and [viewportBuilder] must not
  /// be null.
  const TwoDimensionalScrollable({
    super.key,
    required this.horizontalDetails,
    required this.verticalDetails,
    required this.viewportBuilder,
    this.incrementCalculator,
    this.restorationId,
    this.excludeFromSemantics = false,
    this.diagonalDragBehavior = DiagonalDragBehavior.none,
    this.dragStartBehavior = DragStartBehavior.start,
    this.hitTestBehavior = HitTestBehavior.opaque,
  });

  /// How scrolling gestures should lock to one axis, or allow free movement
  /// in both axes.
  final DiagonalDragBehavior diagonalDragBehavior;

  /// The configuration of the horizontal [Scrollable].
  ///
  /// These [ScrollableDetails] can be used to set the [AxisDirection],
  /// [ScrollController], [ScrollPhysics] and more for the horizontal axis.
  final ScrollableDetails horizontalDetails;

  /// The configuration of the vertical [Scrollable].
  ///
  /// These [ScrollableDetails] can be used to set the [AxisDirection],
  /// [ScrollController], [ScrollPhysics] and more for the vertical axis.
  final ScrollableDetails verticalDetails;

  /// Builds the viewport through which the scrollable content is displayed.
  ///
  /// A [TwoDimensionalViewport] uses two given [ViewportOffset]s to determine
  /// which part of its content is actually visible through the viewport.
  ///
  /// See also:
  ///
  ///  * [TwoDimensionalViewport], which is a viewport that displays a span of
  ///    widgets in both dimensions.
  final TwoDimensionalViewportBuilder viewportBuilder;

  /// {@macro flutter.widgets.Scrollable.incrementCalculator}
  ///
  /// This value applies in both axes.
  final ScrollIncrementCalculator? incrementCalculator;

  /// {@macro flutter.widgets.scrollable.restorationId}
  ///
  /// Internally, the [TwoDimensionalScrollable] will introduce a
  /// [RestorationScope] that will be assigned this value. The two [Scrollable]s
  /// within will then be given unique IDs within this scope.
  final String? restorationId;

  /// {@macro flutter.widgets.scrollable.excludeFromSemantics}
  ///
  /// This value applies to both axes.
  final bool excludeFromSemantics;

  /// {@macro flutter.widgets.scrollable.hitTestBehavior}
  ///
  /// This value applies to both axes.
  final HitTestBehavior hitTestBehavior;

  /// {@macro flutter.widgets.scrollable.dragStartBehavior}
  ///
  /// This value applies in both axes.
  final DragStartBehavior dragStartBehavior;

  @override
  State<TwoDimensionalScrollable> createState() => TwoDimensionalScrollableState();

  /// The state from the closest instance of this class that encloses the given
  /// context, or null if none is found.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// TwoDimensionalScrollableState? scrollable = TwoDimensionalScrollable.maybeOf(context);
  /// ```
  ///
  /// Calling this method will create a dependency on the closest
  /// [TwoDimensionalScrollable] in the [context]. The internal [Scrollable]s
  /// can be accessed through [TwoDimensionalScrollableState.verticalScrollable]
  /// and [TwoDimensionalScrollableState.horizontalScrollable].
  ///
  /// Alternatively, [Scrollable.maybeOf] can be used by providing the desired
  /// [Axis] to the `axis` parameter.
  ///
  /// See also:
  ///
  /// * [TwoDimensionalScrollable.of], which is similar to this method, but
  ///   asserts if no [Scrollable] ancestor is found.
  static TwoDimensionalScrollableState? maybeOf(BuildContext context) {
    final _TwoDimensionalScrollableScope? widget =
        context.dependOnInheritedWidgetOfExactType<_TwoDimensionalScrollableScope>();
    return widget?.twoDimensionalScrollable;
  }

  /// The state from the closest instance of this class that encloses the given
  /// context.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// TwoDimensionalScrollableState scrollable = TwoDimensionalScrollable.of(context);
  /// ```
  ///
  /// Calling this method will create a dependency on the closest
  /// [TwoDimensionalScrollable] in the [context]. The internal [Scrollable]s
  /// can be accessed through [TwoDimensionalScrollableState.verticalScrollable]
  /// and [TwoDimensionalScrollableState.horizontalScrollable].
  ///
  /// If no [TwoDimensionalScrollable] ancestor is found, then this method will
  /// assert in debug mode, and throw an exception in release mode.
  ///
  /// Alternatively, [Scrollable.of] can be used by providing the desired [Axis]
  /// to the `axis` parameter.
  ///
  /// See also:
  ///
  /// * [TwoDimensionalScrollable.maybeOf], which is similar to this method,
  ///   but returns null if no [TwoDimensionalScrollable] ancestor is found.
  static TwoDimensionalScrollableState of(BuildContext context) {
    final TwoDimensionalScrollableState? scrollableState = maybeOf(context);
    assert(() {
      if (scrollableState == null) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'TwoDimensionalScrollable.of() was called with a context that does '
            'not contain a TwoDimensionalScrollable widget.\n',
          ),
          ErrorDescription(
            'No TwoDimensionalScrollable widget ancestor could be found starting '
            'from the context that was passed to TwoDimensionalScrollable.of(). '
            'This can happen because you are using a widget that looks for a '
            'TwoDimensionalScrollable ancestor, but no such ancestor exists.\n'
            'The context used was:\n'
            '  $context',
          ),
        ]);
      }
      return true;
    }());
    return scrollableState!;
  }
}

/// State object for a [TwoDimensionalScrollable] widget.
///
/// To manipulate one of the internal [Scrollable] widget's scroll position, use
/// the object obtained from the [verticalScrollable] or [horizontalScrollable]
/// property.
///
/// To be informed of when a [TwoDimensionalScrollable] widget is scrolling,
/// use a [NotificationListener] to listen for [ScrollNotification]s.
/// Both axes will have the same viewport depth since there is only one
/// viewport, and so should be differentiated by the [Axis] of the
/// [ScrollMetrics] provided by the notification.
class TwoDimensionalScrollableState extends State<TwoDimensionalScrollable> {
  ScrollController? _verticalFallbackController;
  ScrollController? _horizontalFallbackController;
  final GlobalKey<ScrollableState> _verticalOuterScrollableKey = GlobalKey<ScrollableState>();
  final GlobalKey<ScrollableState> _horizontalInnerScrollableKey = GlobalKey<ScrollableState>();

  /// The [ScrollableState] of the vertical axis.
  ///
  /// Accessible by calling [TwoDimensionalScrollable.of].
  ///
  /// Alternatively, [Scrollable.of] can be used by providing [Axis.vertical]
  /// to the `axis` parameter.
  ScrollableState get verticalScrollable {
    assert(_verticalOuterScrollableKey.currentState != null);
    return _verticalOuterScrollableKey.currentState!;
  }

  /// The [ScrollableState] of the horizontal axis.
  ///
  /// Accessible by calling [TwoDimensionalScrollable.of].
  ///
  /// Alternatively, [Scrollable.of] can be used by providing [Axis.horizontal]
  /// to the `axis` parameter.
  ScrollableState get horizontalScrollable {
    assert(_horizontalInnerScrollableKey.currentState != null);
    return _horizontalInnerScrollableKey.currentState!;
  }

  @protected
  @override
  void initState() {
    if (widget.verticalDetails.controller == null) {
      _verticalFallbackController = ScrollController();
    }
    if (widget.horizontalDetails.controller == null) {
      _horizontalFallbackController = ScrollController();
    }
    super.initState();
  }

  @protected
  @override
  void didUpdateWidget(TwoDimensionalScrollable oldWidget) {
    super.didUpdateWidget(oldWidget);
    // Handle changes in the provided/fallback scroll controllers

    // Vertical
    if (oldWidget.verticalDetails.controller != widget.verticalDetails.controller) {
      if (oldWidget.verticalDetails.controller == null) {
        // The old controller was null, meaning the fallback cannot be null.
        // Dispose of the fallback.
        assert(_verticalFallbackController != null);
        assert(widget.verticalDetails.controller != null);
        _verticalFallbackController!.dispose();
        _verticalFallbackController = null;
      } else if (widget.verticalDetails.controller == null) {
        // If the new controller is null, we need to set up the fallback
        // ScrollController.
        assert(_verticalFallbackController == null);
        _verticalFallbackController = ScrollController();
      }
    }

    // Horizontal
    if (oldWidget.horizontalDetails.controller != widget.horizontalDetails.controller) {
      if (oldWidget.horizontalDetails.controller == null) {
        // The old controller was null, meaning the fallback cannot be null.
        // Dispose of the fallback.
        assert(_horizontalFallbackController != null);
        assert(widget.horizontalDetails.controller != null);
        _horizontalFallbackController!.dispose();
        _horizontalFallbackController = null;
      } else if (widget.horizontalDetails.controller == null) {
        // If the new controller is null, we need to set up the fallback
        // ScrollController.
        assert(_horizontalFallbackController == null);
        _horizontalFallbackController = ScrollController();
      }
    }
  }

  @protected
  @override
  Widget build(BuildContext context) {
    assert(
      axisDirectionToAxis(widget.verticalDetails.direction) == Axis.vertical,
      'TwoDimensionalScrollable.verticalDetails are not Axis.vertical.',
    );
    assert(
      axisDirectionToAxis(widget.horizontalDetails.direction) == Axis.horizontal,
      'TwoDimensionalScrollable.horizontalDetails are not Axis.horizontal.',
    );

    final Widget result = RestorationScope(
      restorationId: widget.restorationId,
      child: _VerticalOuterDimension(
        key: _verticalOuterScrollableKey,
        // For gesture forwarding
        horizontalKey: _horizontalInnerScrollableKey,
        axisDirection: widget.verticalDetails.direction,
        controller: widget.verticalDetails.controller ?? _verticalFallbackController!,
        physics: widget.verticalDetails.physics,
        clipBehavior:
            widget.verticalDetails.clipBehavior ??
            widget.verticalDetails.decorationClipBehavior ??
            Clip.hardEdge,
        incrementCalculator: widget.incrementCalculator,
        excludeFromSemantics: widget.excludeFromSemantics,
        restorationId: 'OuterVerticalTwoDimensionalScrollable',
        dragStartBehavior: widget.dragStartBehavior,
        diagonalDragBehavior: widget.diagonalDragBehavior,
        hitTestBehavior: widget.hitTestBehavior,
        viewportBuilder: (BuildContext context, ViewportOffset verticalOffset) {
          return _HorizontalInnerDimension(
            key: _horizontalInnerScrollableKey,
            verticalOuterKey: _verticalOuterScrollableKey,
            axisDirection: widget.horizontalDetails.direction,
            controller: widget.horizontalDetails.controller ?? _horizontalFallbackController!,
            physics: widget.horizontalDetails.physics,
            clipBehavior:
                widget.horizontalDetails.clipBehavior ??
                widget.horizontalDetails.decorationClipBehavior ??
                Clip.hardEdge,
            incrementCalculator: widget.incrementCalculator,
            excludeFromSemantics: widget.excludeFromSemantics,
            restorationId: 'InnerHorizontalTwoDimensionalScrollable',
            dragStartBehavior: widget.dragStartBehavior,
            diagonalDragBehavior: widget.diagonalDragBehavior,
            hitTestBehavior: widget.hitTestBehavior,
            viewportBuilder: (BuildContext context, ViewportOffset horizontalOffset) {
              return widget.viewportBuilder(context, verticalOffset, horizontalOffset);
            },
          );
        },
      ),
    );

    // TODO(Piinks): Build scrollbars for 2 dimensions instead of 1,
    //  https://github.com/flutter/flutter/issues/122348

    return _TwoDimensionalScrollableScope(twoDimensionalScrollable: this, child: result);
  }

  @protected
  @override
  void dispose() {
    _verticalFallbackController?.dispose();
    _horizontalFallbackController?.dispose();
    super.dispose();
  }
}

// Enable TwoDimensionalScrollable.of() to work as if
// TwoDimensionalScrollableState was an inherited widget.
// TwoDimensionalScrollableState.build() always rebuilds its
// _TwoDimensionalScrollableScope.
class _TwoDimensionalScrollableScope extends InheritedWidget {
  const _TwoDimensionalScrollableScope({
    required this.twoDimensionalScrollable,
    required super.child,
  });

  final TwoDimensionalScrollableState twoDimensionalScrollable;

  @override
  bool updateShouldNotify(_TwoDimensionalScrollableScope old) => false;
}

// Vertical outer scrollable of 2D scrolling
class _VerticalOuterDimension extends Scrollable {
  const _VerticalOuterDimension({
    super.key,
    required this.horizontalKey,
    required super.viewportBuilder,
    required super.axisDirection,
    super.controller,
    super.physics,
    super.clipBehavior,
    super.incrementCalculator,
    super.excludeFromSemantics,
    super.dragStartBehavior,
    super.restorationId,
    super.hitTestBehavior,
    this.diagonalDragBehavior = DiagonalDragBehavior.none,
  }) : assert(axisDirection == AxisDirection.up || axisDirection == AxisDirection.down);

  final DiagonalDragBehavior diagonalDragBehavior;
  final GlobalKey<ScrollableState> horizontalKey;

  @override
  _VerticalOuterDimensionState createState() => _VerticalOuterDimensionState();
}

class _VerticalOuterDimensionState extends ScrollableState {
  DiagonalDragBehavior get diagonalDragBehavior =>
      (widget as _VerticalOuterDimension).diagonalDragBehavior;
  ScrollableState get horizontalScrollable =>
      (widget as _VerticalOuterDimension).horizontalKey.currentState!;

  Axis? lockedAxis;
  Offset? lastDragOffset;

  // Implemented in the _HorizontalInnerDimension instead.
  @override
  _EnsureVisibleResults _performEnsureVisible(
    RenderObject object, {
    double alignment = 0.0,
    Duration duration = Duration.zero,
    Curve curve = Curves.ease,
    ScrollPositionAlignmentPolicy alignmentPolicy = ScrollPositionAlignmentPolicy.explicit,
    RenderObject? targetRenderObject,
  }) {
    assert(
      false,
      'The _performEnsureVisible method was called for the vertical scrollable '
      'of a TwoDimensionalScrollable. This should not happen as the horizontal '
      'scrollable handles both axes.',
    );
    return (<Future<void>>[], this);
  }

  void _evaluateLockedAxis(Offset offset) {
    assert(lastDragOffset != null);
    final Offset offsetDelta = lastDragOffset! - offset;
    final double axisDifferential = offsetDelta.dx.abs() - offsetDelta.dy.abs();
    if (axisDifferential.abs() >= kTouchSlop) {
      // We have single axis winner.
      lockedAxis = axisDifferential > 0.0 ? Axis.horizontal : Axis.vertical;
    } else {
      lockedAxis = null;
    }
  }

  @override
  void _handleDragDown(DragDownDetails details) {
    switch (diagonalDragBehavior) {
      case DiagonalDragBehavior.none:
        break;
      case DiagonalDragBehavior.weightedEvent:
      case DiagonalDragBehavior.weightedContinuous:
      case DiagonalDragBehavior.free:
        // Initiate hold. If one or the other wins the gesture, cancel the
        // opposite axis.
        horizontalScrollable._handleDragDown(details);
    }
    super._handleDragDown(details);
  }

  @override
  void _handleDragStart(DragStartDetails details) {
    lastDragOffset = details.globalPosition;
    switch (diagonalDragBehavior) {
      case DiagonalDragBehavior.none:
        break;
      case DiagonalDragBehavior.free:
        // Prepare to scroll both.
        // vertical - will call super below after switch.
        horizontalScrollable._handleDragStart(details);
      case DiagonalDragBehavior.weightedEvent:
      case DiagonalDragBehavior.weightedContinuous:
        // See if one axis wins the drag.
        _evaluateLockedAxis(details.globalPosition);
        switch (lockedAxis) {
          case null:
            // Prepare to scroll both, null means no winner yet.
            // vertical - will call super below after switch.
            horizontalScrollable._handleDragStart(details);
          case Axis.horizontal:
            // Prepare to scroll horizontally.
            horizontalScrollable._handleDragStart(details);
            return;
          case Axis.vertical:
          // Prepare to scroll vertically - will call super below after switch.
        }
    }
    super._handleDragStart(details);
  }

  @override
  void _handleDragUpdate(DragUpdateDetails details) {
    final DragUpdateDetails verticalDragDetails = DragUpdateDetails(
      sourceTimeStamp: details.sourceTimeStamp,
      delta: Offset(0.0, details.delta.dy),
      primaryDelta: details.delta.dy,
      globalPosition: details.globalPosition,
      localPosition: details.localPosition,
    );
    final DragUpdateDetails horizontalDragDetails = DragUpdateDetails(
      sourceTimeStamp: details.sourceTimeStamp,
      delta: Offset(details.delta.dx, 0.0),
      primaryDelta: details.delta.dx,
      globalPosition: details.globalPosition,
      localPosition: details.localPosition,
    );

    switch (diagonalDragBehavior) {
      case DiagonalDragBehavior.none:
        // Default gesture handling from super class.
        super._handleDragUpdate(verticalDragDetails);
        return;
      case DiagonalDragBehavior.free:
        // Scroll both axes
        horizontalScrollable._handleDragUpdate(horizontalDragDetails);
        super._handleDragUpdate(verticalDragDetails);
        return;
      case DiagonalDragBehavior.weightedContinuous:
        // Re-evaluate locked axis for every update.
        _evaluateLockedAxis(details.globalPosition);
        lastDragOffset = details.globalPosition;
      case DiagonalDragBehavior.weightedEvent:
        // Lock axis only once per gesture.
        if (lockedAxis == null && lastDragOffset != null) {
          // A winner has not been declared yet.
          // See if one axis has won the drag.
          _evaluateLockedAxis(details.globalPosition);
        }
    }
    switch (lockedAxis) {
      case null:
        // Scroll both - vertical after switch
        horizontalScrollable._handleDragUpdate(horizontalDragDetails);
      case Axis.horizontal:
        // Scroll horizontally
        horizontalScrollable._handleDragUpdate(horizontalDragDetails);
        return;
      case Axis.vertical:
      // Scroll vertically - after switch
    }
    super._handleDragUpdate(verticalDragDetails);
  }

  @override
  void _handleDragEnd(DragEndDetails details) {
    lastDragOffset = null;
    lockedAxis = null;
    final double dx = details.velocity.pixelsPerSecond.dx;
    final double dy = details.velocity.pixelsPerSecond.dy;
    final DragEndDetails verticalDragDetails = DragEndDetails(
      velocity: Velocity(pixelsPerSecond: Offset(0.0, dy)),
      primaryVelocity: dy,
    );
    final DragEndDetails horizontalDragDetails = DragEndDetails(
      velocity: Velocity(pixelsPerSecond: Offset(dx, 0.0)),
      primaryVelocity: dx,
    );

    switch (diagonalDragBehavior) {
      case DiagonalDragBehavior.none:
        break;
      case DiagonalDragBehavior.weightedEvent:
      case DiagonalDragBehavior.weightedContinuous:
      case DiagonalDragBehavior.free:
        horizontalScrollable._handleDragEnd(horizontalDragDetails);
    }
    super._handleDragEnd(verticalDragDetails);
  }

  @override
  void _handleDragCancel() {
    lastDragOffset = null;
    lockedAxis = null;
    switch (diagonalDragBehavior) {
      case DiagonalDragBehavior.none:
        break;
      case DiagonalDragBehavior.weightedEvent:
      case DiagonalDragBehavior.weightedContinuous:
      case DiagonalDragBehavior.free:
        horizontalScrollable._handleDragCancel();
    }
    super._handleDragCancel();
  }

  @override
  void setCanDrag(bool value) {
    switch (diagonalDragBehavior) {
      case DiagonalDragBehavior.none:
        // If we aren't scrolling diagonally, the default drag gesture recognizer
        // is used.
        super.setCanDrag(value);
        return;
      case DiagonalDragBehavior.weightedEvent:
      case DiagonalDragBehavior.weightedContinuous:
      case DiagonalDragBehavior.free:
        if (value) {
          // Replaces the typical vertical/horizontal drag gesture recognizers
          // with a pan gesture recognizer to allow bidirectional scrolling.
          // Based on the diagonalDragBehavior, valid vertical deltas are
          // applied to this scrollable, while horizontal deltas are routed to
          // the horizontal scrollable.
          _gestureRecognizers = <Type, GestureRecognizerFactory>{
            PanGestureRecognizer: GestureRecognizerFactoryWithHandlers<PanGestureRecognizer>(
              () => PanGestureRecognizer(supportedDevices: _configuration.dragDevices),
              (PanGestureRecognizer instance) {
                instance
                  ..onDown = _handleDragDown
                  ..onStart = _handleDragStart
                  ..onUpdate = _handleDragUpdate
                  ..onEnd = _handleDragEnd
                  ..onCancel = _handleDragCancel
                  ..minFlingDistance = _physics?.minFlingDistance
                  ..minFlingVelocity = _physics?.minFlingVelocity
                  ..maxFlingVelocity = _physics?.maxFlingVelocity
                  ..velocityTrackerBuilder = _configuration.velocityTrackerBuilder(context)
                  ..dragStartBehavior = widget.dragStartBehavior
                  ..gestureSettings = _mediaQueryGestureSettings;
              },
            ),
          };
          // Cancel the active hold/drag (if any) because the gesture recognizers
          // will soon be disposed by our RawGestureDetector, and we won't be
          // receiving pointer up events to cancel the hold/drag.
          _handleDragCancel();
          _lastCanDrag = value;
          _lastAxisDirection = widget.axis;
          if (_gestureDetectorKey.currentState != null) {
            _gestureDetectorKey.currentState!.replaceGestureRecognizers(_gestureRecognizers);
          }
        }
        return;
    }
  }

  @override
  Widget _buildChrome(BuildContext context, Widget child) {
    final ScrollableDetails details = ScrollableDetails(
      direction: widget.axisDirection,
      controller: _effectiveScrollController,
      clipBehavior: widget.clipBehavior,
    );
    // Skip building a scrollbar here, the dual scrollbar is added in
    // TwoDimensionalScrollableState.
    return _configuration.buildOverscrollIndicator(context, child, details);
  }
}

// Horizontal inner scrollable of 2D scrolling
class _HorizontalInnerDimension extends Scrollable {
  const _HorizontalInnerDimension({
    super.key,
    required this.verticalOuterKey,
    required super.viewportBuilder,
    required super.axisDirection,
    super.controller,
    super.physics,
    super.clipBehavior,
    super.incrementCalculator,
    super.excludeFromSemantics,
    super.dragStartBehavior,
    super.restorationId,
    super.hitTestBehavior,
    this.diagonalDragBehavior = DiagonalDragBehavior.none,
  }) : assert(axisDirection == AxisDirection.left || axisDirection == AxisDirection.right);

  final GlobalKey<ScrollableState> verticalOuterKey;
  final DiagonalDragBehavior diagonalDragBehavior;

  @override
  _HorizontalInnerDimensionState createState() => _HorizontalInnerDimensionState();
}

class _HorizontalInnerDimensionState extends ScrollableState {
  late ScrollableState verticalScrollable;

  GlobalKey<ScrollableState> get verticalOuterKey =>
      (widget as _HorizontalInnerDimension).verticalOuterKey;
  DiagonalDragBehavior get diagonalDragBehavior =>
      (widget as _HorizontalInnerDimension).diagonalDragBehavior;

  @override
  void didChangeDependencies() {
    verticalScrollable = Scrollable.of(context);
    assert(axisDirectionToAxis(verticalScrollable.axisDirection) == Axis.vertical);
    super.didChangeDependencies();
  }

  // Returns the Future from calling ensureVisible for the ScrollPosition, as
  // as well as the vertical ScrollableState instance so its context can be
  // used to check for other ancestor Scrollables in executing ensureVisible.
  @override
  _EnsureVisibleResults _performEnsureVisible(
    RenderObject object, {
    double alignment = 0.0,
    Duration duration = Duration.zero,
    Curve curve = Curves.ease,
    ScrollPositionAlignmentPolicy alignmentPolicy = ScrollPositionAlignmentPolicy.explicit,
    RenderObject? targetRenderObject,
  }) {
    final List<Future<void>> newFutures = <Future<void>>[
      position.ensureVisible(
        object,
        alignment: alignment,
        duration: duration,
        curve: curve,
        alignmentPolicy: alignmentPolicy,
      ),
      verticalScrollable.position.ensureVisible(
        object,
        alignment: alignment,
        duration: duration,
        curve: curve,
        alignmentPolicy: alignmentPolicy,
      ),
    ];

    return (newFutures, verticalScrollable);
  }

  @override
  void setCanDrag(bool value) {
    switch (diagonalDragBehavior) {
      case DiagonalDragBehavior.none:
        // If we aren't scrolling diagonally, the default drag gesture
        // recognizer is used.
        super.setCanDrag(value);
        return;
      case DiagonalDragBehavior.weightedEvent:
      case DiagonalDragBehavior.weightedContinuous:
      case DiagonalDragBehavior.free:
        if (value) {
          // If a type of diagonal scrolling is enabled, a panning gesture
          // recognizer will be created for the _VerticalOuterDimension. So in
          // this case, the _HorizontalInnerDimension does not require a gesture
          // recognizer, meanwhile we should ensure the outer dimension has
          // updated in case it did not have enough content to enable dragging.
          _gestureRecognizers = const <Type, GestureRecognizerFactory>{};
          verticalOuterKey.currentState!.setCanDrag(value);
          // Cancel the active hold/drag (if any) because the gesture recognizers
          // will soon be disposed by our RawGestureDetector, and we won't be
          // receiving pointer up events to cancel the hold/drag.
          _handleDragCancel();
          _lastCanDrag = value;
          _lastAxisDirection = widget.axis;
          if (_gestureDetectorKey.currentState != null) {
            _gestureDetectorKey.currentState!.replaceGestureRecognizers(_gestureRecognizers);
          }
        }
        return;
    }
  }

  @override
  Widget _buildChrome(BuildContext context, Widget child) {
    final ScrollableDetails details = ScrollableDetails(
      direction: widget.axisDirection,
      controller: _effectiveScrollController,
      clipBehavior: widget.clipBehavior,
    );
    // Skip building a scrollbar here, the dual scrollbar is added in
    // TwoDimensionalScrollableState.
    return _configuration.buildOverscrollIndicator(context, child, details);
  }
}
