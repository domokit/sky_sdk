// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter_test/flutter_test.dart';
///
/// @docImport 'framework.dart';
/// @docImport 'notification_listener.dart';
/// @docImport 'page_storage.dart';
/// @docImport 'page_view.dart';
/// @docImport 'scroll_configuration.dart';
/// @docImport 'scroll_metrics.dart';
/// @docImport 'scroll_notification.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'scrollable.dart';
library;

import 'package:flutter/animation.dart';
import 'package:flutter/foundation.dart';

import 'scroll_context.dart';
import 'scroll_physics.dart';
import 'scroll_position.dart';
import 'scroll_position_with_single_context.dart';

// Examples can assume:
// TrackingScrollController _trackingScrollController = TrackingScrollController();

/// Signature for when a [ScrollController] has added or removed a
/// [ScrollPosition].
///
/// Since a [ScrollPosition] is not created and attached to a controller until
/// the [Scrollable] is built, this can be used to respond to the position being
/// attached to a controller.
///
/// By having access to the position directly, additional listeners can be
/// applied to aspects of the scroll position, like
/// [ScrollPosition.isScrollingNotifier].
///
/// Used by [ScrollController.onAttach] and [ScrollController.onDetach].
typedef ScrollControllerCallback = void Function(ScrollPosition position);

/// Controls a scrollable widget.
///
/// Scroll controllers are typically stored as member variables in [State]
/// objects and are reused in each [State.build]. A single scroll controller can
/// be used to control multiple scrollable widgets, but some operations, such
/// as reading the scroll [offset], require the controller to be used with a
/// single scrollable widget.
///
/// A scroll controller creates a [ScrollPosition] to manage the state specific
/// to an individual [Scrollable] widget. To use a custom [ScrollPosition],
/// subclass [ScrollController] and override [createScrollPosition].
///
/// {@macro flutter.widgets.scrollPosition.listening}
///
/// Typically used with [ListView], [GridView], [CustomScrollView].
///
/// See also:
///
///  * [ListView], [GridView], [CustomScrollView], which can be controlled by a
///    [ScrollController].
///  * [Scrollable], which is the lower-level widget that creates and associates
///    [ScrollPosition] objects with [ScrollController] objects.
///  * [PageController], which is an analogous object for controlling a
///    [PageView].
///  * [ScrollPosition], which manages the scroll offset for an individual
///    scrolling widget.
///  * [ScrollNotification] and [NotificationListener], which can be used to
///    listen to scrolling occur without using a [ScrollController].
class ScrollController extends ChangeNotifier {
  /// Creates a controller for a scrollable widget.
  ScrollController({
    double initialScrollOffset = 0.0,
    this.keepScrollOffset = true,
    this.debugLabel,
    this.onAttach,
    this.onDetach,
  }) : _initialScrollOffset = initialScrollOffset {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  /// The initial value to use for [offset].
  ///
  /// New [ScrollPosition] objects that are created and attached to this
  /// controller will have their offset initialized to this value
  /// if [keepScrollOffset] is false or a scroll offset hasn't been saved yet.
  ///
  /// Defaults to 0.0.
  double get initialScrollOffset => _initialScrollOffset;
  final double _initialScrollOffset;

  /// Each time a scroll completes, save the current scroll [offset] with
  /// [PageStorage] and restore it if this controller's scrollable is recreated.
  ///
  /// If this property is set to false, the scroll offset is never saved
  /// and [initialScrollOffset] is always used to initialize the scroll
  /// offset. If true (the default), the initial scroll offset is used the
  /// first time the controller's scrollable is created, since there's no
  /// scroll offset to restore yet. Subsequently the saved offset is
  /// restored and [initialScrollOffset] is ignored.
  ///
  /// See also:
  ///
  ///  * [PageStorageKey], which should be used when more than one
  ///    scrollable appears in the same route, to distinguish the [PageStorage]
  ///    locations used to save scroll offsets.
  final bool keepScrollOffset;

  /// Called when a [ScrollPosition] is attached to the scroll controller.
  ///
  /// Since a scroll position is not attached until a [Scrollable] is actually
  /// built, this can be used to respond to a new position being attached.
  ///
  /// At the time that a scroll position is attached, the [ScrollMetrics], such as
  /// the [ScrollMetrics.maxScrollExtent], are not yet available. These are not
  /// determined until the [Scrollable] has finished laying out its contents and
  /// computing things like the full extent of that content.
  /// [ScrollPosition.hasContentDimensions] can be used to know when the
  /// metrics are available, or a [ScrollMetricsNotification] can be used,
  /// discussed further below.
  ///
  /// {@tool dartpad}
  /// This sample shows how to apply a listener to the
  /// [ScrollPosition.isScrollingNotifier] using [ScrollController.onAttach].
  /// This is used to change the [AppBar]'s color when scrolling is occurring.
  ///
  /// ** See code in examples/api/lib/widgets/scroll_position/scroll_controller_on_attach.0.dart **
  /// {@end-tool}
  final ScrollControllerCallback? onAttach;

  /// Called when a [ScrollPosition] is detached from the scroll controller.
  ///
  /// {@tool dartpad}
  /// This sample shows how to apply a listener to the
  /// [ScrollPosition.isScrollingNotifier] using [ScrollController.onAttach]
  /// & [ScrollController.onDetach].
  /// This is used to change the [AppBar]'s color when scrolling is occurring.
  ///
  /// ** See code in examples/api/lib/widgets/scroll_position/scroll_controller_on_attach.0.dart **
  /// {@end-tool}
  final ScrollControllerCallback? onDetach;

  /// A label that is used in the [toString] output. Intended to aid with
  /// identifying scroll controller instances in debug output.
  final String? debugLabel;

  /// The currently attached positions.
  ///
  /// This should not be mutated directly. [ScrollPosition] objects can be added
  /// and removed using [attach] and [detach].
  Iterable<ScrollPosition> get positions => _positions;
  final List<ScrollPosition> _positions = <ScrollPosition>[];

  /// Whether any [ScrollPosition] objects have attached themselves to the
  /// [ScrollController] using the [attach] method.
  ///
  /// If this is false, then members that interact with the [ScrollPosition],
  /// such as [position], [offset], [animateTo], and [jumpTo], must not be
  /// called.
  bool get hasClients => _positions.isNotEmpty;

  /// Returns the attached [ScrollPosition], from which the actual scroll offset
  /// of the [ScrollView] can be obtained.
  ///
  /// Calling this is only valid when only a single position is attached.
  ScrollPosition get position {
    assert(_positions.isNotEmpty, 'ScrollController not attached to any scroll views.');
    assert(_positions.length == 1, 'ScrollController attached to multiple scroll views.');
    return _positions.single;
  }

  /// The current scroll offset of the scrollable widget.
  ///
  /// Requires the controller to be controlling exactly one scrollable widget.
  double get offset => position.pixels;

  /// Animates the position from its current value to the given value.
  ///
  /// Any active animation is canceled. If the user is currently scrolling, that
  /// action is canceled.
  ///
  /// The returned [Future] will complete when the animation ends, whether it
  /// completed successfully or whether it was interrupted prematurely.
  ///
  /// An animation will be interrupted whenever the user attempts to scroll
  /// manually, or whenever another activity is started, or whenever the
  /// animation reaches the edge of the viewport and attempts to overscroll. (If
  /// the [ScrollPosition] does not overscroll but instead allows scrolling
  /// beyond the extents, then going beyond the extents will not interrupt the
  /// animation.)
  ///
  /// The animation is indifferent to changes to the viewport or content
  /// dimensions.
  ///
  /// Once the animation has completed, the scroll position will attempt to
  /// begin a ballistic activity in case its value is not stable (for example,
  /// if it is scrolled beyond the extents and in that situation the scroll
  /// position would normally bounce back).
  ///
  /// The duration must not be zero. To jump to a particular value without an
  /// animation, use [jumpTo].
  ///
  /// When calling [animateTo] in widget tests, `await`ing the returned
  /// [Future] may cause the test to hang and timeout. Instead, use
  /// [WidgetTester.pumpAndSettle].
  Future<void> animateTo(double offset, {required Duration duration, required Curve curve}) async {
    assert(_positions.isNotEmpty, 'ScrollController not attached to any scroll views.');
    await Future.wait<void>(<Future<void>>[
      for (int i = 0; i < _positions.length; i += 1)
        _positions[i].animateTo(offset, duration: duration, curve: curve),
    ]);
  }

  /// Jumps the scroll position from its current value to the given value,
  /// without animation, and without checking if the new value is in range.
  ///
  /// Any active animation is canceled. If the user is currently scrolling, that
  /// action is canceled.
  ///
  /// If this method changes the scroll position, a sequence of start/update/end
  /// scroll notifications will be dispatched. No overscroll notifications can
  /// be generated by this method.
  ///
  /// Immediately after the jump, a ballistic activity is started, in case the
  /// value was out of range.
  void jumpTo(double value) {
    assert(_positions.isNotEmpty, 'ScrollController not attached to any scroll views.');
    for (final ScrollPosition position in List<ScrollPosition>.of(_positions)) {
      position.jumpTo(value);
    }
  }

  /// Register the given position with this controller.
  ///
  /// After this function returns, the [animateTo] and [jumpTo] methods on this
  /// controller will manipulate the given position.
  void attach(ScrollPosition position) {
    assert(!_positions.contains(position));
    _positions.add(position);
    position.addListener(notifyListeners);
    onAttach?.call(position);
  }

  /// Unregister the given position with this controller.
  ///
  /// After this function returns, the [animateTo] and [jumpTo] methods on this
  /// controller will not manipulate the given position.
  void detach(ScrollPosition position) {
    assert(_positions.contains(position));
    onDetach?.call(position);
    position.removeListener(notifyListeners);
    _positions.remove(position);
  }

  @override
  void dispose() {
    for (final ScrollPosition position in _positions) {
      position.removeListener(notifyListeners);
    }
    super.dispose();
  }

  /// Creates a [ScrollPosition] for use by a [Scrollable] widget.
  ///
  /// Subclasses can override this function to customize the [ScrollPosition]
  /// used by the scrollable widgets they control. For example, [PageController]
  /// overrides this function to return a page-oriented scroll position
  /// subclass that keeps the same page visible when the scrollable widget
  /// resizes.
  ///
  /// By default, returns a [ScrollPositionWithSingleContext].
  ///
  /// The arguments are generally passed to the [ScrollPosition] being created:
  ///
  ///  * `physics`: An instance of [ScrollPhysics] that determines how the
  ///    [ScrollPosition] should react to user interactions, how it should
  ///    simulate scrolling when released or flung, etc. The value will not be
  ///    null. It typically comes from the [ScrollView] or other widget that
  ///    creates the [Scrollable], or, if none was provided, from the ambient
  ///    [ScrollConfiguration].
  ///  * `context`: A [ScrollContext] used for communicating with the object
  ///    that is to own the [ScrollPosition] (typically, this is the
  ///    [Scrollable] itself).
  ///  * `oldPosition`: If this is not the first time a [ScrollPosition] has
  ///    been created for this [Scrollable], this will be the previous instance.
  ///    This is used when the environment has changed and the [Scrollable]
  ///    needs to recreate the [ScrollPosition] object. It is null the first
  ///    time the [ScrollPosition] is created.
  ScrollPosition createScrollPosition(
    ScrollPhysics physics,
    ScrollContext context,
    ScrollPosition? oldPosition,
  ) {
    return ScrollPositionWithSingleContext(
      physics: physics,
      context: context,
      initialPixels: initialScrollOffset,
      keepScrollOffset: keepScrollOffset,
      oldPosition: oldPosition,
      debugLabel: debugLabel,
    );
  }

  @override
  String toString() {
    final List<String> description = <String>[];
    debugFillDescription(description);
    return '${describeIdentity(this)}(${description.join(", ")})';
  }

  /// Add additional information to the given description for use by [toString].
  ///
  /// This method makes it easier for subclasses to coordinate to provide a
  /// high-quality [toString] implementation. The [toString] implementation on
  /// the [ScrollController] base class calls [debugFillDescription] to collect
  /// useful information from subclasses to incorporate into its return value.
  ///
  /// Implementations of this method should start with a call to the inherited
  /// method, as in `super.debugFillDescription(description)`.
  @mustCallSuper
  void debugFillDescription(List<String> description) {
    if (debugLabel != null) {
      description.add(debugLabel!);
    }
    if (initialScrollOffset != 0.0) {
      description.add('initialScrollOffset: ${initialScrollOffset.toStringAsFixed(1)}, ');
    }
    if (_positions.isEmpty) {
      description.add('no clients');
    } else if (_positions.length == 1) {
      // Don't actually list the client itself, since its toString may refer to us.
      description.add('one client, offset ${offset.toStringAsFixed(1)}');
    } else {
      description.add('${_positions.length} clients');
    }
  }
}

// Examples can assume:
// TrackingScrollController? _trackingScrollController;

/// A [ScrollController] whose [initialScrollOffset] tracks its most recently
/// updated [ScrollPosition].
///
/// This class can be used to synchronize the scroll offset of two or more
/// lazily created scroll views that share a single [TrackingScrollController].
/// It tracks the most recently updated scroll position and reports it as its
/// `initialScrollOffset`.
///
/// {@tool snippet}
///
/// In this example each [PageView] page contains a [ListView] and all three
/// [ListView]'s share a [TrackingScrollController]. The scroll offsets of all
/// three list views will track each other, to the extent that's possible given
/// the different list lengths.
///
/// ```dart
/// PageView(
///   children: <Widget>[
///     ListView(
///       controller: _trackingScrollController,
///       children: List<Widget>.generate(100, (int i) => Text('page 0 item $i')).toList(),
///     ),
///     ListView(
///       controller: _trackingScrollController,
///       children: List<Widget>.generate(200, (int i) => Text('page 1 item $i')).toList(),
///     ),
///     ListView(
///      controller: _trackingScrollController,
///      children: List<Widget>.generate(300, (int i) => Text('page 2 item $i')).toList(),
///     ),
///   ],
/// )
/// ```
/// {@end-tool}
///
/// In this example the `_trackingController` would have been created by the
/// stateful widget that built the widget tree.
class TrackingScrollController extends ScrollController {
  /// Creates a scroll controller that continually updates its
  /// [initialScrollOffset] to match the last scroll notification it received.
  TrackingScrollController({
    super.initialScrollOffset,
    super.keepScrollOffset,
    super.debugLabel,
    super.onAttach,
    super.onDetach,
  });

  final Map<ScrollPosition, VoidCallback> _positionToListener = <ScrollPosition, VoidCallback>{};
  ScrollPosition? _lastUpdated;
  double? _lastUpdatedOffset;

  /// The last [ScrollPosition] to change. Returns null if there aren't any
  /// attached scroll positions, or there hasn't been any scrolling yet, or the
  /// last [ScrollPosition] to change has since been removed.
  ScrollPosition? get mostRecentlyUpdatedPosition => _lastUpdated;

  /// Returns the scroll offset of the [mostRecentlyUpdatedPosition] or, if that
  /// is null, the initial scroll offset provided to the constructor.
  ///
  /// See also:
  ///
  ///  * [ScrollController.initialScrollOffset], which this overrides.
  @override
  double get initialScrollOffset => _lastUpdatedOffset ?? super.initialScrollOffset;

  @override
  void attach(ScrollPosition position) {
    super.attach(position);
    assert(!_positionToListener.containsKey(position));
    _positionToListener[position] = () {
      _lastUpdated = position;
      _lastUpdatedOffset = position.pixels;
    };
    position.addListener(_positionToListener[position]!);
  }

  @override
  void detach(ScrollPosition position) {
    super.detach(position);
    assert(_positionToListener.containsKey(position));
    position.removeListener(_positionToListener[position]!);
    _positionToListener.remove(position);
    if (_lastUpdated == position) {
      _lastUpdated = null;
    }
    if (_positionToListener.isEmpty) {
      _lastUpdatedOffset = null;
    }
  }

  @override
  void dispose() {
    for (final ScrollPosition position in positions) {
      assert(_positionToListener.containsKey(position));
      position.removeListener(_positionToListener[position]!);
    }
    super.dispose();
  }
}
