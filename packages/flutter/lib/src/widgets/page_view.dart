// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'single_child_scroll_view.dart';
/// @docImport 'text.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart' show clampDouble, precisionErrorTolerance;
import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'debug.dart';
import 'framework.dart';
import 'notification_listener.dart';
import 'page_storage.dart';
import 'scroll_configuration.dart';
import 'scroll_context.dart';
import 'scroll_controller.dart';
import 'scroll_delegate.dart';
import 'scroll_metrics.dart';
import 'scroll_notification.dart';
import 'scroll_physics.dart';
import 'scroll_position.dart';
import 'scroll_position_with_single_context.dart';
import 'scroll_view.dart';
import 'scrollable.dart';
import 'sliver_fill.dart';
import 'viewport.dart';

/// A controller for [PageView].
///
/// A page controller lets you manipulate which page is visible in a [PageView].
/// In addition to being able to control the pixel offset of the content inside
/// the [PageView], a [PageController] also lets you control the offset in terms
/// of pages, which are increments of the viewport size.
///
/// See also:
///
///  * [PageView], which is the widget this object controls.
///
/// {@tool snippet}
///
/// This widget introduces a [MaterialApp], [Scaffold] and [PageView] with two pages
/// using the default constructor. Both pages contain an [ElevatedButton] allowing you
/// to animate the [PageView] using a [PageController].
///
/// ```dart
/// class MyPageView extends StatefulWidget {
///   const MyPageView({super.key});
///
///   @override
///   State<MyPageView> createState() => _MyPageViewState();
/// }
///
/// class _MyPageViewState extends State<MyPageView> {
///   final PageController _pageController = PageController();
///
///   @override
///   void dispose() {
///     _pageController.dispose();
///     super.dispose();
///   }
///
///   @override
///   Widget build(BuildContext context) {
///     return MaterialApp(
///       home: Scaffold(
///         body: PageView(
///           controller: _pageController,
///           children: <Widget>[
///             ColoredBox(
///               color: Colors.red,
///               child: Center(
///                 child: ElevatedButton(
///                   onPressed: () {
///                     if (_pageController.hasClients) {
///                       _pageController.animateToPage(
///                         1,
///                         duration: const Duration(milliseconds: 400),
///                         curve: Curves.easeInOut,
///                       );
///                     }
///                   },
///                   child: const Text('Next'),
///                 ),
///               ),
///             ),
///             ColoredBox(
///               color: Colors.blue,
///               child: Center(
///                 child: ElevatedButton(
///                   onPressed: () {
///                     if (_pageController.hasClients) {
///                       _pageController.animateToPage(
///                         0,
///                         duration: const Duration(milliseconds: 400),
///                         curve: Curves.easeInOut,
///                       );
///                     }
///                   },
///                   child: const Text('Previous'),
///                 ),
///               ),
///             ),
///           ],
///         ),
///       ),
///     );
///   }
/// }
/// ```
/// {@end-tool}
class PageController extends ScrollController {
  /// Creates a page controller.
  PageController({
    this.initialPage = 0,
    this.keepPage = true,
    this.viewportFraction = 1.0,
    super.onAttach,
    super.onDetach,
  }) : assert(viewportFraction > 0.0);

  /// The page to show when first creating the [PageView].
  final int initialPage;

  /// Save the current [page] with [PageStorage] and restore it if
  /// this controller's scrollable is recreated.
  ///
  /// If this property is set to false, the current [page] is never saved
  /// and [initialPage] is always used to initialize the scroll offset.
  /// If true (the default), the initial page is used the first time the
  /// controller's scrollable is created, since there's isn't a page to
  /// restore yet. Subsequently the saved page is restored and
  /// [initialPage] is ignored.
  ///
  /// See also:
  ///
  ///  * [PageStorageKey], which should be used when more than one
  ///    scrollable appears in the same route, to distinguish the [PageStorage]
  ///    locations used to save scroll offsets.
  final bool keepPage;

  /// {@template flutter.widgets.pageview.viewportFraction}
  /// The fraction of the viewport that each page should occupy.
  ///
  /// Defaults to 1.0, which means each page fills the viewport in the scrolling
  /// direction.
  /// {@endtemplate}
  final double viewportFraction;

  /// The current page displayed in the controlled [PageView].
  ///
  /// There are circumstances that this [PageController] can't know the current
  /// page. Reading [page] will throw an [AssertionError] in the following cases:
  ///
  /// 1. No [PageView] is currently using this [PageController]. Once a
  /// [PageView] starts using this [PageController], the new [page]
  /// position will be derived:
  ///
  ///   * First, based on the attached [PageView]'s [BuildContext] and the
  ///     position saved at that context's [PageStorage] if [keepPage] is true.
  ///   * Second, from the [PageController]'s [initialPage].
  ///
  /// 2. More than one [PageView] using the same [PageController].
  ///
  /// The [hasClients] property can be used to check if a [PageView] is attached
  /// prior to accessing [page].
  double? get page {
    assert(
      positions.isNotEmpty,
      'PageController.page cannot be accessed before a PageView is built with it.',
    );
    assert(
      positions.length == 1,
      'The page property cannot be read when multiple PageViews are attached to '
      'the same PageController.',
    );
    final _PagePosition position = this.position as _PagePosition;
    return position.page;
  }

  /// Animates the controlled [PageView] from the current page to the given page.
  ///
  /// The animation lasts for the given duration and follows the given curve.
  /// The returned [Future] resolves when the animation completes.
  Future<void> animateToPage(int page, {required Duration duration, required Curve curve}) {
    final _PagePosition position = this.position as _PagePosition;
    if (position._cachedPage != null) {
      position._cachedPage = page.toDouble();
      return Future<void>.value();
    }

    if (!position.hasViewportDimension) {
      position._pageToUseOnStartup = page.toDouble();
      return Future<void>.value();
    }

    return position.animateTo(
      position.getPixelsFromPage(page.toDouble()),
      duration: duration,
      curve: curve,
    );
  }

  /// Changes which page is displayed in the controlled [PageView].
  ///
  /// Jumps the page position from its current value to the given value,
  /// without animation, and without checking if the new value is in range.
  void jumpToPage(int page) {
    final _PagePosition position = this.position as _PagePosition;
    if (position._cachedPage != null) {
      position._cachedPage = page.toDouble();
      return;
    }

    if (!position.hasViewportDimension) {
      position._pageToUseOnStartup = page.toDouble();
      return;
    }

    position.jumpTo(position.getPixelsFromPage(page.toDouble()));
  }

  /// Animates the controlled [PageView] to the next page.
  ///
  /// The animation lasts for the given duration and follows the given curve.
  /// The returned [Future] resolves when the animation completes.
  Future<void> nextPage({required Duration duration, required Curve curve}) {
    return animateToPage(page!.round() + 1, duration: duration, curve: curve);
  }

  /// Animates the controlled [PageView] to the previous page.
  ///
  /// The animation lasts for the given duration and follows the given curve.
  /// The returned [Future] resolves when the animation completes.
  Future<void> previousPage({required Duration duration, required Curve curve}) {
    return animateToPage(page!.round() - 1, duration: duration, curve: curve);
  }

  @override
  ScrollPosition createScrollPosition(
    ScrollPhysics physics,
    ScrollContext context,
    ScrollPosition? oldPosition,
  ) {
    return _PagePosition(
      physics: physics,
      context: context,
      initialPage: initialPage,
      keepPage: keepPage,
      viewportFraction: viewportFraction,
      oldPosition: oldPosition,
    );
  }

  @override
  void attach(ScrollPosition position) {
    super.attach(position);
    final _PagePosition pagePosition = position as _PagePosition;
    pagePosition.viewportFraction = viewportFraction;
  }
}

/// Metrics for a [PageView].
///
/// The metrics are available on [ScrollNotification]s generated from
/// [PageView]s.
class PageMetrics extends FixedScrollMetrics {
  /// Creates an immutable snapshot of values associated with a [PageView].
  PageMetrics({
    required super.minScrollExtent,
    required super.maxScrollExtent,
    required super.pixels,
    required super.viewportDimension,
    required super.axisDirection,
    required this.viewportFraction,
    required super.devicePixelRatio,
  });

  @override
  PageMetrics copyWith({
    double? minScrollExtent,
    double? maxScrollExtent,
    double? pixels,
    double? viewportDimension,
    AxisDirection? axisDirection,
    double? viewportFraction,
    double? devicePixelRatio,
  }) {
    return PageMetrics(
      minScrollExtent: minScrollExtent ?? (hasContentDimensions ? this.minScrollExtent : null),
      maxScrollExtent: maxScrollExtent ?? (hasContentDimensions ? this.maxScrollExtent : null),
      pixels: pixels ?? (hasPixels ? this.pixels : null),
      viewportDimension:
          viewportDimension ?? (hasViewportDimension ? this.viewportDimension : null),
      axisDirection: axisDirection ?? this.axisDirection,
      viewportFraction: viewportFraction ?? this.viewportFraction,
      devicePixelRatio: devicePixelRatio ?? this.devicePixelRatio,
    );
  }

  /// The current page displayed in the [PageView].
  double? get page {
    return math.max(0.0, clampDouble(pixels, minScrollExtent, maxScrollExtent)) /
        math.max(1.0, viewportDimension * viewportFraction);
  }

  /// The fraction of the viewport that each page occupies.
  ///
  /// Used to compute [page] from the current [pixels].
  final double viewportFraction;
}

class _PagePosition extends ScrollPositionWithSingleContext implements PageMetrics {
  _PagePosition({
    required super.physics,
    required super.context,
    this.initialPage = 0,
    bool keepPage = true,
    double viewportFraction = 1.0,
    super.oldPosition,
  }) : assert(viewportFraction > 0.0),
       _viewportFraction = viewportFraction,
       _pageToUseOnStartup = initialPage.toDouble(),
       super(initialPixels: null, keepScrollOffset: keepPage);

  final int initialPage;
  double _pageToUseOnStartup;
  // When the viewport has a zero-size, the `page` can not
  // be retrieved by `getPageFromPixels`, so we need to cache the page
  // for use when resizing the viewport to non-zero next time.
  double? _cachedPage;

  @override
  Future<void> ensureVisible(
    RenderObject object, {
    double alignment = 0.0,
    Duration duration = Duration.zero,
    Curve curve = Curves.ease,
    ScrollPositionAlignmentPolicy alignmentPolicy = ScrollPositionAlignmentPolicy.explicit,
    RenderObject? targetRenderObject,
  }) {
    // Since the _PagePosition is intended to cover the available space within
    // its viewport, stop trying to move the target render object to the center
    // - otherwise, could end up changing which page is visible and moving the
    // targetRenderObject out of the viewport.
    return super.ensureVisible(
      object,
      alignment: alignment,
      duration: duration,
      curve: curve,
      alignmentPolicy: alignmentPolicy,
    );
  }

  @override
  double get viewportFraction => _viewportFraction;
  double _viewportFraction;
  set viewportFraction(double value) {
    if (_viewportFraction == value) {
      return;
    }
    final double? oldPage = page;
    _viewportFraction = value;
    if (oldPage != null) {
      forcePixels(getPixelsFromPage(oldPage));
    }
  }

  // The amount of offset that will be added to [minScrollExtent] and subtracted
  // from [maxScrollExtent], such that every page will properly snap to the center
  // of the viewport when viewportFraction is greater than 1.
  //
  // The value is 0 if viewportFraction is less than or equal to 1, larger than 0
  // otherwise.
  double get _initialPageOffset => math.max(0, viewportDimension * (viewportFraction - 1) / 2);

  double getPageFromPixels(double pixels, double viewportDimension) {
    assert(viewportDimension > 0.0);
    final double actual =
        math.max(0.0, pixels - _initialPageOffset) / (viewportDimension * viewportFraction);
    final double round = actual.roundToDouble();
    if ((actual - round).abs() < precisionErrorTolerance) {
      return round;
    }
    return actual;
  }

  double getPixelsFromPage(double page) {
    return page * viewportDimension * viewportFraction + _initialPageOffset;
  }

  @override
  double? get page {
    if (!hasPixels) {
      return null;
    }
    assert(
      hasContentDimensions || !haveDimensions,
      'Page value is only available after content dimensions are established.',
    );
    return hasContentDimensions || haveDimensions
        ? _cachedPage ??
            getPageFromPixels(
              clampDouble(pixels, minScrollExtent, maxScrollExtent),
              viewportDimension,
            )
        : null;
  }

  @override
  void saveScrollOffset() {
    PageStorage.maybeOf(context.storageContext)?.writeState(
      context.storageContext,
      _cachedPage ?? getPageFromPixels(pixels, viewportDimension),
    );
  }

  @override
  void restoreScrollOffset() {
    if (!hasPixels) {
      final double? value =
          PageStorage.maybeOf(context.storageContext)?.readState(context.storageContext) as double?;
      if (value != null) {
        _pageToUseOnStartup = value;
      }
    }
  }

  @override
  void saveOffset() {
    context.saveOffset(_cachedPage ?? getPageFromPixels(pixels, viewportDimension));
  }

  @override
  void restoreOffset(double offset, {bool initialRestore = false}) {
    if (initialRestore) {
      _pageToUseOnStartup = offset;
    } else {
      jumpTo(getPixelsFromPage(offset));
    }
  }

  @override
  bool applyViewportDimension(double viewportDimension) {
    final double? oldViewportDimensions = hasViewportDimension ? this.viewportDimension : null;
    if (viewportDimension == oldViewportDimensions) {
      return true;
    }
    final bool result = super.applyViewportDimension(viewportDimension);
    final double? oldPixels = hasPixels ? pixels : null;
    double page;
    if (oldPixels == null) {
      page = _pageToUseOnStartup;
    } else if (oldViewportDimensions == 0.0) {
      // If resize from zero, we should use the _cachedPage to recover the state.
      page = _cachedPage!;
    } else {
      page = getPageFromPixels(oldPixels, oldViewportDimensions!);
    }
    final double newPixels = getPixelsFromPage(page);

    // If the viewportDimension is zero, cache the page
    // in case the viewport is resized to be non-zero.
    _cachedPage = (viewportDimension == 0.0) ? page : null;

    if (newPixels != oldPixels) {
      correctPixels(newPixels);
      return false;
    }
    return result;
  }

  @override
  void absorb(ScrollPosition other) {
    super.absorb(other);
    assert(_cachedPage == null);

    if (other is! _PagePosition) {
      return;
    }

    if (other._cachedPage != null) {
      _cachedPage = other._cachedPage;
    }
  }

  @override
  bool applyContentDimensions(double minScrollExtent, double maxScrollExtent) {
    final double newMinScrollExtent = minScrollExtent + _initialPageOffset;
    return super.applyContentDimensions(
      newMinScrollExtent,
      math.max(newMinScrollExtent, maxScrollExtent - _initialPageOffset),
    );
  }

  @override
  PageMetrics copyWith({
    double? minScrollExtent,
    double? maxScrollExtent,
    double? pixels,
    double? viewportDimension,
    AxisDirection? axisDirection,
    double? viewportFraction,
    double? devicePixelRatio,
  }) {
    return PageMetrics(
      minScrollExtent: minScrollExtent ?? (hasContentDimensions ? this.minScrollExtent : null),
      maxScrollExtent: maxScrollExtent ?? (hasContentDimensions ? this.maxScrollExtent : null),
      pixels: pixels ?? (hasPixels ? this.pixels : null),
      viewportDimension:
          viewportDimension ?? (hasViewportDimension ? this.viewportDimension : null),
      axisDirection: axisDirection ?? this.axisDirection,
      viewportFraction: viewportFraction ?? this.viewportFraction,
      devicePixelRatio: devicePixelRatio ?? this.devicePixelRatio,
    );
  }
}

class _ForceImplicitScrollPhysics extends ScrollPhysics {
  const _ForceImplicitScrollPhysics({required this.allowImplicitScrolling, super.parent});

  @override
  _ForceImplicitScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return _ForceImplicitScrollPhysics(
      allowImplicitScrolling: allowImplicitScrolling,
      parent: buildParent(ancestor),
    );
  }

  @override
  final bool allowImplicitScrolling;
}

/// Scroll physics used by a [PageView].
///
/// These physics cause the page view to snap to page boundaries.
///
/// See also:
///
///  * [ScrollPhysics], the base class which defines the API for scrolling
///    physics.
///  * [PageView.physics], which can override the physics used by a page view.
class PageScrollPhysics extends ScrollPhysics {
  /// Creates physics for a [PageView].
  const PageScrollPhysics({super.parent});

  @override
  PageScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return PageScrollPhysics(parent: buildParent(ancestor));
  }

  double _getPage(ScrollMetrics position) {
    if (position is _PagePosition) {
      return position.page!;
    }
    return position.pixels / position.viewportDimension;
  }

  double _getPixels(ScrollMetrics position, double page) {
    if (position is _PagePosition) {
      return position.getPixelsFromPage(page);
    }
    return page * position.viewportDimension;
  }

  double _getTargetPixels(ScrollMetrics position, Tolerance tolerance, double velocity) {
    double page = _getPage(position);
    if (velocity < -tolerance.velocity) {
      page -= 0.5;
    } else if (velocity > tolerance.velocity) {
      page += 0.5;
    }
    return _getPixels(position, page.roundToDouble());
  }

  @override
  Simulation? createBallisticSimulation(ScrollMetrics position, double velocity) {
    // If we're out of range and not headed back in range, defer to the parent
    // ballistics, which should put us back in range at a page boundary.
    if ((velocity <= 0.0 && position.pixels <= position.minScrollExtent) ||
        (velocity >= 0.0 && position.pixels >= position.maxScrollExtent)) {
      return super.createBallisticSimulation(position, velocity);
    }
    final Tolerance tolerance = toleranceFor(position);
    final double target = _getTargetPixels(position, tolerance, velocity);
    if (target != position.pixels) {
      return ScrollSpringSimulation(
        spring,
        position.pixels,
        target,
        velocity,
        tolerance: tolerance,
      );
    }
    return null;
  }

  @override
  bool get allowImplicitScrolling => false;
}

const PageScrollPhysics _kPagePhysics = PageScrollPhysics();

/// A scrollable list that works page by page.
///
/// Each child of a page view is forced to be the same size as the viewport.
///
/// You can use a [PageController] to control which page is visible in the view.
/// In addition to being able to control the pixel offset of the content inside
/// the [PageView], a [PageController] also lets you control the offset in terms
/// of pages, which are increments of the viewport size.
///
/// The [PageController] can also be used to control the
/// [PageController.initialPage], which determines which page is shown when the
/// [PageView] is first constructed, and the [PageController.viewportFraction],
/// which determines the size of the pages as a fraction of the viewport size.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=J1gE9xvph-A}
///
/// {@tool dartpad}
/// Here is an example of [PageView]. It creates a centered [Text] in each of the three pages
/// which scroll horizontally.
///
/// ** See code in examples/api/lib/widgets/page_view/page_view.0.dart **
/// {@end-tool}
///
/// ## Persisting the scroll position during a session
///
/// Scroll views attempt to persist their scroll position using [PageStorage].
/// For a [PageView], this can be disabled by setting [PageController.keepPage]
/// to false on the [controller]. If it is enabled, using a [PageStorageKey] for
/// the [key] of this widget is recommended to help disambiguate different
/// scroll views from each other.
///
/// See also:
///
///  * [PageController], which controls which page is visible in the view.
///  * [SingleChildScrollView], when you need to make a single child scrollable.
///  * [ListView], for a scrollable list of boxes.
///  * [GridView], for a scrollable grid of boxes.
///  * [ScrollNotification] and [NotificationListener], which can be used to watch
///    the scroll position without using a [ScrollController].
class PageView extends StatefulWidget {
  /// Creates a scrollable list that works page by page from an explicit [List]
  /// of widgets.
  ///
  /// This constructor is appropriate for page views with a small number of
  /// children because constructing the [List] requires doing work for every
  /// child that could possibly be displayed in the page view, instead of just
  /// those children that are actually visible.
  ///
  /// Like other widgets in the framework, this widget expects that
  /// the [children] list will not be mutated after it has been passed in here.
  /// See the documentation at [SliverChildListDelegate.children] for more details.
  ///
  /// {@template flutter.widgets.PageView.allowImplicitScrolling}
  /// If [allowImplicitScrolling] is true, the [PageView] will participate in
  /// accessibility scrolling more like a [ListView], where implicit scroll
  /// actions will move to the next page rather than into the contents of the
  /// [PageView].
  /// {@endtemplate}
  PageView({
    super.key,
    this.scrollDirection = Axis.horizontal,
    this.reverse = false,
    this.controller,
    this.physics,
    this.pageSnapping = true,
    this.onPageChanged,
    List<Widget> children = const <Widget>[],
    this.dragStartBehavior = DragStartBehavior.start,
    this.allowImplicitScrolling = false,
    this.restorationId,
    this.clipBehavior = Clip.hardEdge,
    this.hitTestBehavior = HitTestBehavior.opaque,
    this.scrollBehavior,
    this.padEnds = true,
  }) : childrenDelegate = SliverChildListDelegate(children);

  /// Creates a scrollable list that works page by page using widgets that are
  /// created on demand.
  ///
  /// This constructor is appropriate for page views with a large (or infinite)
  /// number of children because the builder is called only for those children
  /// that are actually visible.
  ///
  /// Providing a non-null [itemCount] lets the [PageView] compute the maximum
  /// scroll extent.
  ///
  /// [itemBuilder] will be called only with indices greater than or equal to
  /// zero and less than [itemCount].
  ///
  /// {@macro flutter.widgets.ListView.builder.itemBuilder}
  ///
  /// {@template flutter.widgets.PageView.findChildIndexCallback}
  /// The [findChildIndexCallback] corresponds to the
  /// [SliverChildBuilderDelegate.findChildIndexCallback] property. If null,
  /// a child widget may not map to its existing [RenderObject] when the order
  /// of children returned from the children builder changes.
  /// This may result in state-loss. This callback needs to be implemented if
  /// the order of the children may change at a later time.
  /// {@endtemplate}
  ///
  /// {@macro flutter.widgets.PageView.allowImplicitScrolling}
  PageView.builder({
    super.key,
    this.scrollDirection = Axis.horizontal,
    this.reverse = false,
    this.controller,
    this.physics,
    this.pageSnapping = true,
    this.onPageChanged,
    required NullableIndexedWidgetBuilder itemBuilder,
    ChildIndexGetter? findChildIndexCallback,
    int? itemCount,
    this.dragStartBehavior = DragStartBehavior.start,
    this.allowImplicitScrolling = false,
    this.restorationId,
    this.clipBehavior = Clip.hardEdge,
    this.hitTestBehavior = HitTestBehavior.opaque,
    this.scrollBehavior,
    this.padEnds = true,
  }) : childrenDelegate = SliverChildBuilderDelegate(
         itemBuilder,
         findChildIndexCallback: findChildIndexCallback,
         childCount: itemCount,
       );

  /// Creates a scrollable list that works page by page with a custom child
  /// model.
  ///
  /// {@tool dartpad}
  /// This example shows a [PageView] that uses a custom [SliverChildBuilderDelegate] to support child
  /// reordering.
  ///
  /// ** See code in examples/api/lib/widgets/page_view/page_view.1.dart **
  /// {@end-tool}
  ///
  /// {@macro flutter.widgets.PageView.allowImplicitScrolling}
  const PageView.custom({
    super.key,
    this.scrollDirection = Axis.horizontal,
    this.reverse = false,
    this.controller,
    this.physics,
    this.pageSnapping = true,
    this.onPageChanged,
    required this.childrenDelegate,
    this.dragStartBehavior = DragStartBehavior.start,
    this.allowImplicitScrolling = false,
    this.restorationId,
    this.clipBehavior = Clip.hardEdge,
    this.hitTestBehavior = HitTestBehavior.opaque,
    this.scrollBehavior,
    this.padEnds = true,
  });

  /// Controls whether the widget's pages will respond to
  /// [RenderObject.showOnScreen], which will allow for implicit accessibility
  /// scrolling.
  ///
  /// With this flag set to false, when accessibility focus reaches the end of
  /// the current page and the user attempts to move it to the next element, the
  /// focus will traverse to the next widget outside of the page view.
  ///
  /// With this flag set to true, when accessibility focus reaches the end of
  /// the current page and user attempts to move it to the next element, focus
  /// will traverse to the next page in the page view.
  final bool allowImplicitScrolling;

  /// {@macro flutter.widgets.scrollable.restorationId}
  final String? restorationId;

  /// The [Axis] along which the scroll view's offset increases with each page.
  ///
  /// For the direction in which active scrolling may be occurring, see
  /// [ScrollDirection].
  ///
  /// Defaults to [Axis.horizontal].
  final Axis scrollDirection;

  /// Whether the page view scrolls in the reading direction.
  ///
  /// For example, if the reading direction is left-to-right and
  /// [scrollDirection] is [Axis.horizontal], then the page view scrolls from
  /// left to right when [reverse] is false and from right to left when
  /// [reverse] is true.
  ///
  /// Similarly, if [scrollDirection] is [Axis.vertical], then the page view
  /// scrolls from top to bottom when [reverse] is false and from bottom to top
  /// when [reverse] is true.
  ///
  /// Defaults to false.
  final bool reverse;

  /// An object that can be used to control the position to which this page
  /// view is scrolled.
  final PageController? controller;

  /// How the page view should respond to user input.
  ///
  /// For example, determines how the page view continues to animate after the
  /// user stops dragging the page view.
  ///
  /// The physics are modified to snap to page boundaries using
  /// [PageScrollPhysics] prior to being used.
  ///
  /// If an explicit [ScrollBehavior] is provided to [scrollBehavior], the
  /// [ScrollPhysics] provided by that behavior will take precedence after
  /// [physics].
  ///
  /// Defaults to matching platform conventions.
  final ScrollPhysics? physics;

  /// Set to false to disable page snapping, useful for custom scroll behavior.
  ///
  /// If the [padEnds] is false and [PageController.viewportFraction] < 1.0,
  /// the page will snap to the beginning of the viewport; otherwise, the page
  /// will snap to the center of the viewport.
  final bool pageSnapping;

  /// Called whenever the page in the center of the viewport changes.
  final ValueChanged<int>? onPageChanged;

  /// A delegate that provides the children for the [PageView].
  ///
  /// The [PageView.custom] constructor lets you specify this delegate
  /// explicitly. The [PageView] and [PageView.builder] constructors create a
  /// [childrenDelegate] that wraps the given [List] and [IndexedWidgetBuilder],
  /// respectively.
  final SliverChildDelegate childrenDelegate;

  /// {@macro flutter.widgets.scrollable.dragStartBehavior}
  final DragStartBehavior dragStartBehavior;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// {@macro flutter.widgets.scrollable.hitTestBehavior}
  ///
  /// Defaults to [HitTestBehavior.opaque].
  final HitTestBehavior hitTestBehavior;

  /// {@macro flutter.widgets.scrollable.scrollBehavior}
  ///
  /// The [ScrollBehavior] of the inherited [ScrollConfiguration] will be
  /// modified by default to not apply a [Scrollbar].
  final ScrollBehavior? scrollBehavior;

  /// Whether to add padding to both ends of the list.
  ///
  /// If this is set to true and [PageController.viewportFraction] < 1.0, padding will be added
  /// such that the first and last child slivers will be in the center of
  /// the viewport when scrolled all the way to the start or end, respectively.
  ///
  /// If [PageController.viewportFraction] >= 1.0, this property has no effect.
  ///
  /// This property defaults to true.
  final bool padEnds;

  @override
  State<PageView> createState() => _PageViewState();
}

class _PageViewState extends State<PageView> {
  int _lastReportedPage = 0;

  late PageController _controller;

  @override
  void initState() {
    super.initState();
    _initController();
    _lastReportedPage = _controller.initialPage;
  }

  @override
  void dispose() {
    if (widget.controller == null) {
      _controller.dispose();
    }
    super.dispose();
  }

  void _initController() {
    _controller = widget.controller ?? PageController();
  }

  @override
  void didUpdateWidget(PageView oldWidget) {
    if (oldWidget.controller != widget.controller) {
      if (oldWidget.controller == null) {
        _controller.dispose();
      }
      _initController();
    }
    super.didUpdateWidget(oldWidget);
  }

  AxisDirection _getDirection(BuildContext context) {
    switch (widget.scrollDirection) {
      case Axis.horizontal:
        assert(debugCheckHasDirectionality(context));
        final TextDirection textDirection = Directionality.of(context);
        final AxisDirection axisDirection = textDirectionToAxisDirection(textDirection);
        return widget.reverse ? flipAxisDirection(axisDirection) : axisDirection;
      case Axis.vertical:
        return widget.reverse ? AxisDirection.up : AxisDirection.down;
    }
  }

  @override
  Widget build(BuildContext context) {
    final AxisDirection axisDirection = _getDirection(context);
    final ScrollPhysics physics = _ForceImplicitScrollPhysics(
      allowImplicitScrolling: widget.allowImplicitScrolling,
    ).applyTo(
      widget.pageSnapping
          ? _kPagePhysics.applyTo(
            widget.physics ?? widget.scrollBehavior?.getScrollPhysics(context),
          )
          : widget.physics ?? widget.scrollBehavior?.getScrollPhysics(context),
    );

    return NotificationListener<ScrollNotification>(
      onNotification: (ScrollNotification notification) {
        if (notification.depth == 0 &&
            widget.onPageChanged != null &&
            notification is ScrollUpdateNotification) {
          final PageMetrics metrics = notification.metrics as PageMetrics;
          final int currentPage = metrics.page!.round();
          if (currentPage != _lastReportedPage) {
            _lastReportedPage = currentPage;
            widget.onPageChanged!(currentPage);
          }
        }
        return false;
      },
      child: Scrollable(
        dragStartBehavior: widget.dragStartBehavior,
        axisDirection: axisDirection,
        controller: _controller,
        physics: physics,
        restorationId: widget.restorationId,
        hitTestBehavior: widget.hitTestBehavior,
        scrollBehavior:
            widget.scrollBehavior ?? ScrollConfiguration.of(context).copyWith(scrollbars: false),
        viewportBuilder: (BuildContext context, ViewportOffset position) {
          return Viewport(
            // TODO(dnfield): we should provide a way to set cacheExtent
            // independent of implicit scrolling:
            // https://github.com/flutter/flutter/issues/45632
            cacheExtent: widget.allowImplicitScrolling ? 1.0 : 0.0,
            cacheExtentStyle: CacheExtentStyle.viewport,
            axisDirection: axisDirection,
            offset: position,
            clipBehavior: widget.clipBehavior,
            slivers: <Widget>[
              SliverFillViewport(
                viewportFraction: _controller.viewportFraction,
                delegate: widget.childrenDelegate,
                padEnds: widget.padEnds,
              ),
            ],
          );
        },
      ),
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(EnumProperty<Axis>('scrollDirection', widget.scrollDirection));
    description.add(FlagProperty('reverse', value: widget.reverse, ifTrue: 'reversed'));
    description.add(
      DiagnosticsProperty<PageController>('controller', _controller, showName: false),
    );
    description.add(DiagnosticsProperty<ScrollPhysics>('physics', widget.physics, showName: false));
    description.add(
      FlagProperty('pageSnapping', value: widget.pageSnapping, ifFalse: 'snapping disabled'),
    );
    description.add(
      FlagProperty(
        'allowImplicitScrolling',
        value: widget.allowImplicitScrolling,
        ifTrue: 'allow implicit scrolling',
      ),
    );
  }
}
