// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'color_scheme.dart';
library;

import 'dart:math' as math;

import 'package:collection/collection.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'ink_well.dart';
import 'material.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// A Material Design carousel widget.
///
/// The [CarouselView] presents a scrollable list of items, each of which can dynamically
/// change size based on the chosen layout.
///
/// Material Design 3 introduced 4 carousel layouts:
///  * Multi-browse: This layout shows at least one large, medium, and small
///    carousel item at a time. This layout is supported by [CarouselView.weighted].
///  * Uncontained (default): This layout show items that scroll to the edge of the
///    container. This layout is supported by [CarouselView].
///  * Hero: This layout shows at least one large and one small item at a time.
///    This layout is supported by [CarouselView.weighted].
///  * Full-screen: This layout shows one edge-to-edge large item at a time and
///    scrolls vertically. The full-screen layout can be supported by both
///    constructors.
///
/// The default constructor implements the uncontained layout model. It shows
/// items that scroll to the edge of the container, behaving similarly to a
/// [ListView] where all children are a uniform size. [CarouselView.weighted]
/// enables dynamic item sizing. Each item is assigned a weight that determines
/// the portion of the viewport it occupies. This constructor helps to create
/// layouts like multi-browse, and hero. In order to have a full-screen layout,
/// if [CarouselView] is used, then set the [itemExtent] to screen size; if
/// [CarouselView.weighted] is used, then set the [flexWeights] to only have
/// one integer in the array.
///
/// {@tool snippet}
///
/// This code snippet shows how to get a vertical full-screen carousel by using
/// [itemExtent] in [CarouselView].
///
/// ```dart
/// Scaffold(
///   body: CarouselView(
///     scrollDirection: Axis.vertical,
///     itemExtent: double.infinity,
///     children: List<Widget>.generate(10, (int index) {
///       return Center(child: Text('Item $index'));
///     }),
///   ),
/// ),
/// ```
///
/// This code snippet below shows how to achieve the same vertical full-screen
/// carousel by using [flexWeights] in [CarouselView.weighted].
///
/// ```dart
/// Scaffold(
///   body: CarouselView.weighted(
///     scrollDirection: Axis.vertical,
///     flexWeights: const <int>[1], // Or any positive integers as long as the length of the array is 1.
///     children: List<Widget>.generate(10, (int index) {
///       return Center(child: Text('Item $index'));
///     }),
///   ),
/// ),
/// ```
/// {@end-tool}
///
/// In [CarouselView.weighted], weights are relative proportions. For example,
/// if the layout weights is `[3, 2, 1]`, it means the first visible item occupies
/// 3/6 of the viewport; the second visible item occupies 2/6 of the viewport;
/// the last visible item occupies 1/6 of the viewport. As the carousel scrolls,
/// the size of the latter one gradually changes to the size of the former one.
/// As a result, when the first visible item is completely off-screen, the
/// following items will follow the same layout as before. Using [CarouselView.weighted]
/// helps build the multi-browse, hero, center-aligned hero and full-screen layouts,
/// as indicated in [Carousel sepcs](https://m3.material.io/components/carousel/specs).
///
/// The [CarouselController] is used to control the
/// [CarouselController.initialItem], which determines the first fully expanded
/// item when the [CarouselView] or [CarouselView.weighted] is initially displayed.
/// This is straightforward for [CarouselView] because each item in the view
/// has fixed size. In [CarouselView.weighted], for instance, if the layout
/// weights are `[1, 2, 3, 2, 1]` and the initial item is 4 (the fourth item), the
/// view will display items 2, 3, 4, 5, and 6 with weights 1, 2, 3, 2 and 1
/// respectively.
///
/// The [CarouselView.itemExtent] property must be non-null and defines the base
/// size of items. While items typically maintain this size, the first and last
/// visible items may be slightly compressed during scrolling. The [shrinkExtent]
/// property controls the minimum allowable size for these compressed items.
///
/// {@tool dartpad}
/// Here is an example to show different carousel layouts that [CarouselView]
/// and [CarouselView.weighted] can build.
///
/// ** See code in examples/api/lib/material/carousel/carousel.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [CarouselController], which controls the first fully visible item in the
///    view.
///  * [PageView], which is a scrollable list that works page by page.
class CarouselView extends StatefulWidget {
  /// Creates a Material Design carousel.
  const CarouselView({
    super.key,
    this.padding,
    this.backgroundColor,
    this.elevation,
    this.shape,
    this.overlayColor,
    this.itemSnapping = false,
    this.shrinkExtent = 0.0,
    this.controller,
    this.scrollDirection = Axis.horizontal,
    this.reverse = false,
    this.onTap,
    this.enableSplash = true,
    required double this.itemExtent,
    required this.children,
  }) : consumeMaxWeight = true,
       flexWeights = null;

  /// Creates a scrollable list where the size of each child widget is dynamically
  /// determined by the provided [flexWeights].
  ///
  /// The [flexWeights] parameter is required and defines the relative size
  /// proportions of each child widget.
  ///
  /// While scrolling, the main-axis extent (size) of each visible item changes
  /// dynamically based on the scrolling progress. The cross-axis extent is determined
  /// by the parent constraints. As the first visible item scrolls completely
  /// off-screen, the next item becomes the first visible item, and has the same
  /// size as the previously first item. The rest of the visible items maintain
  /// their relative layout.
  ///
  /// For example, if the layout weights are `[1, 6, 1]`, the length of [flexWeights]
  /// indicates three items will be visible at a time. The layout of these items
  /// would be:
  ///  * First item: Extent is (1 / (1 + 6 + 1)) * viewport extent.
  ///  * Second item: Extent is (6 / (1 + 6 + 1)) * viewport extent.
  ///  * Third item: Extent is (1 / (1 + 6 + 1)) * viewport extent.
  ///
  /// Assuming a viewport extent of 800 in the main axis and the first item is
  /// item 0, there would be three visible items with extents of 100, 600, and 100.
  /// As item 0 scrolls off-screen, the extent of item 1 smoothly decreases from 600
  /// to 100. For instance, if item 0 is 30% off-screen, item 1 should have decreased
  /// its size to 30% of the difference from 600 to 100; its extent would be
  /// 600 - 0.3 * (600 - 100). Similarly, item 2's extent would increase from 100
  /// to 600, becoming 100 + 0.3 * (600 - 100).
  ///
  /// As the initially visible items change size during scrolling, item 3 enters
  /// the view to fill the remaining space. Its extent starts at a minimum of
  /// [shrinkExtent] (or 0 if [shrinkExtent] is not provided) and gradually
  /// increases to match the extent of the last visible item (100 in this example).
  ///
  /// When [consumeMaxWeight] is set to `true`, each child can be expanded to occupy
  /// the maximum weight while scrolling. For example, with [flexWeights] of `[1, 7, 1]`,
  /// the initial weight of the first item is 1. However, by enabling
  /// [consumeMaxWeight] and scrolling forward, the first item can expand to occupy
  /// a weight of 7, leaving a weight of 1 as some empty space before it. This feature
  /// is particularly useful for achieving [Hero](https://m3.material.io/components/carousel/specs#b33a5579-d648-42a9-b934-98718d65454f)
  /// and [Center-aligned hero](https://m3.material.io/components/carousel/specs#92c779ce-de8b-4dee-8201-95d3e429204f)
  /// layouts indicated in the Material Design 3.
  const CarouselView.weighted({
    super.key,
    this.padding,
    this.backgroundColor,
    this.elevation,
    this.shape,
    this.overlayColor,
    this.itemSnapping = false,
    this.shrinkExtent = 0.0,
    this.controller,
    this.scrollDirection = Axis.horizontal,
    this.reverse = false,
    this.consumeMaxWeight = true,
    this.onTap,
    this.enableSplash = true,
    required List<int> this.flexWeights,
    required this.children,
  }) : itemExtent = null;

  /// The amount of space to surround each carousel item with.
  ///
  /// Defaults to [EdgeInsets.all] of 4 pixels.
  final EdgeInsets? padding;

  /// The background color for each carousel item.
  ///
  /// Defaults to [ColorScheme.surface].
  final Color? backgroundColor;

  /// The z-coordinate of each carousel item.
  ///
  /// Defaults to 0.0.
  final double? elevation;

  /// The shape of each carousel item's [Material].
  ///
  /// Defines each item's [Material.shape].
  ///
  /// Defaults to a [RoundedRectangleBorder] with a circular corner radius
  /// of 28.0.
  final ShapeBorder? shape;

  /// The highlight color to indicate the carousel items are in pressed, hovered
  /// or focused states.
  ///
  /// The default values are:
  ///   * [WidgetState.pressed] - [ColorScheme.onSurface] with an opacity of 0.1
  ///   * [WidgetState.hovered] - [ColorScheme.onSurface] with an opacity of 0.08
  ///   * [WidgetState.focused] - [ColorScheme.onSurface] with an opacity of 0.1
  final WidgetStateProperty<Color?>? overlayColor;

  /// The minimum allowable extent (size) in the main axis for carousel items
  /// during scrolling transitions.
  ///
  /// As the carousel scrolls, the first visible item is pinned and gradually
  /// shrinks until it reaches this minimum extent before scrolling off-screen.
  /// Similarly, the last visible item enters the viewport at this minimum size
  /// and expands to its full [itemExtent].
  ///
  /// In cases where the remaining viewport space for the last visible item is
  /// larger than the defined [shrinkExtent], the [shrinkExtent] is dynamically
  /// adjusted to match this remaining space, ensuring a smooth size transition.
  ///
  /// Defaults to 0.0. Setting to 0.0 allows items to shrink/expand completely,
  /// transitioning between 0.0 and the full item size. In cases where the
  /// remaining viewport space for the last visible item is larger than the
  /// defined [shrinkExtent], the [shrinkExtent] is dynamically adjusted to match
  /// this remaining space, ensuring a smooth size transition.
  final double shrinkExtent;

  /// Whether the carousel should keep scrolling to the next/previous items to
  /// maintain the original layout.
  ///
  /// Defaults to false.
  final bool itemSnapping;

  /// An object that can be used to control the position to which this scroll
  /// view is scrolled.
  final CarouselController? controller;

  /// The [Axis] along which the scroll view's offset increases with each item.
  ///
  /// Defaults to [Axis.horizontal].
  final Axis scrollDirection;

  /// Whether the carousel list scrolls in the reading direction.
  ///
  /// For example, if the reading direction is left-to-right and
  /// [scrollDirection] is [Axis.horizontal], then the carousel scrolls from
  /// left to right when [reverse] is false and from right to left when
  /// [reverse] is true.
  ///
  /// Similarly, if [scrollDirection] is [Axis.vertical], then the carousel view
  /// scrolls from top to bottom when [reverse] is false and from bottom to top
  /// when [reverse] is true.
  ///
  /// Defaults to false.
  final bool reverse;

  /// Whether the collapsed items are allowed to expand to the max size.
  ///
  /// If this is false, the layout of the carousel doesn't change. This is especially
  /// useful when a weight list in [CarouselView.weighted] has a max item in the
  /// middle and at least one small item on either side, such as `[1, 7, 1, 1]`.
  /// In this case, if this is false, the first and the last two items cannot
  /// expand to the max size. If this is true, there will be some space before
  /// the first item or after the last item coming so every item has a chance to
  /// be fully expanded.
  ///
  /// Defaults to true.
  final bool consumeMaxWeight;

  /// Called when one of the [children] is tapped.
  final ValueChanged<int>? onTap;

  /// Determines whether an [InkWell] will cover each Carousel item.
  ///
  /// If true, tapping an item will create an ink splash
  /// as defined by the [ThemeData.splashFactory].
  ///
  /// Setting this to false allows the [children] to respond to user gestures.
  ///
  /// Defaults to true.
  final bool enableSplash;

  /// The extent the children are forced to have in the main axis.
  ///
  /// The item extent should not exceed the available space that the carousel view
  /// occupies to ensure at least one item is fully visible.
  ///
  /// This is required for [CarouselView]. In [CarouselView.weighted], this is null.
  final double? itemExtent;

  /// The weights that each visible child should occupy in the viewport.
  ///
  /// The length of [flexWeights] represents how many items should be visible
  /// at a time in the viewport. For example, setting [flexWeights] to
  /// `<int>[3, 2, 1]` means there are 3 carousel items and their extents are
  /// 3/6, 2/6 and 1/6 of the viewport extent.
  ///
  /// This is a required property in [CarouselView.weighted]. This is null
  /// for default [CarouselView]. The integers must be greater than 0.
  final List<int>? flexWeights;

  /// The child widgets for the carousel.
  final List<Widget> children;

  @override
  State<CarouselView> createState() => _CarouselViewState();
}

class _CarouselViewState extends State<CarouselView> {
  double? _itemExtent;
  List<int>? get _flexWeights => widget.flexWeights;
  bool get _consumeMaxWeight => widget.consumeMaxWeight;
  CarouselController? _internalController;
  CarouselController get _controller => widget.controller ?? _internalController!;

  @override
  void initState() {
    super.initState();
    _itemExtent = widget.itemExtent;
    if (widget.controller == null) {
      _internalController = CarouselController();
    }
    _controller._attach(this);
  }

  @override
  void didUpdateWidget(covariant CarouselView oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.controller != oldWidget.controller) {
      oldWidget.controller?._detach(this);
      if (widget.controller != null) {
        _internalController?._detach(this);
        _internalController = null;
        widget.controller?._attach(this);
      } else {
        // widget.controller == null && oldWidget.controller != null
        assert(_internalController == null);
        _internalController = CarouselController();
        _controller._attach(this);
      }
    }
    if (widget.flexWeights != oldWidget.flexWeights) {
      (_controller.position as _CarouselPosition).flexWeights = _flexWeights;
    }
    if (widget.itemExtent != oldWidget.itemExtent) {
      _itemExtent = widget.itemExtent;
      (_controller.position as _CarouselPosition).itemExtent = _itemExtent;
    }
    if (widget.consumeMaxWeight != oldWidget.consumeMaxWeight) {
      (_controller.position as _CarouselPosition).consumeMaxWeight = _consumeMaxWeight;
    }
  }

  @override
  void dispose() {
    _controller._detach(this);
    _internalController?.dispose();
    super.dispose();
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

  Widget _buildCarouselItem(ThemeData theme, int index) {
    final EdgeInsets effectivePadding = widget.padding ?? const EdgeInsets.all(4.0);
    final Color effectiveBackgroundColor =
        widget.backgroundColor ?? Theme.of(context).colorScheme.surface;
    final double effectiveElevation = widget.elevation ?? 0.0;
    final ShapeBorder effectiveShape =
        widget.shape ??
        const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(28.0)));
    final WidgetStateProperty<Color?> effectiveOverlayColor =
        widget.overlayColor ??
        WidgetStateProperty.resolveWith((Set<WidgetState> states) {
          if (states.contains(WidgetState.pressed)) {
            return theme.colorScheme.onSurface.withOpacity(0.1);
          }
          if (states.contains(WidgetState.hovered)) {
            return theme.colorScheme.onSurface.withOpacity(0.08);
          }
          if (states.contains(WidgetState.focused)) {
            return theme.colorScheme.onSurface.withOpacity(0.1);
          }
          return null;
        });

    Widget contents = widget.children[index];
    if (widget.enableSplash) {
      contents = Stack(
        fit: StackFit.expand,
        children: <Widget>[
          contents,
          Material(
            color: Colors.transparent,
            child: InkWell(
              onTap: () => widget.onTap?.call(index),
              overlayColor: effectiveOverlayColor,
            ),
          ),
        ],
      );
    } else if (widget.onTap != null) {
      contents = GestureDetector(onTap: () => widget.onTap!(index), child: contents);
    }

    return Padding(
      padding: effectivePadding,
      child: Material(
        clipBehavior: Clip.antiAlias,
        color: effectiveBackgroundColor,
        elevation: effectiveElevation,
        shape: effectiveShape,
        child: contents,
      ),
    );
  }

  Widget _buildSliverCarousel(ThemeData theme) {
    if (_itemExtent != null) {
      return _SliverFixedExtentCarousel(
        itemExtent: _itemExtent!,
        minExtent: widget.shrinkExtent,
        delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
          return _buildCarouselItem(theme, index);
        }, childCount: widget.children.length),
      );
    }

    assert(
      _flexWeights != null && _flexWeights!.every((int weight) => weight > 0),
      'flexWeights is null or it contains non-positive integers',
    );
    return _SliverWeightedCarousel(
      consumeMaxWeight: _consumeMaxWeight,
      shrinkExtent: widget.shrinkExtent,
      weights: _flexWeights!,
      delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
        return _buildCarouselItem(theme, index);
      }, childCount: widget.children.length),
    );
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final AxisDirection axisDirection = _getDirection(context);
    final ScrollPhysics physics =
        widget.itemSnapping
            ? const CarouselScrollPhysics()
            : ScrollConfiguration.of(context).getScrollPhysics(context);

    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        final double mainAxisExtent = switch (widget.scrollDirection) {
          Axis.horizontal => constraints.maxWidth,
          Axis.vertical => constraints.maxHeight,
        };
        _itemExtent =
            _itemExtent == null ? _itemExtent : clampDouble(_itemExtent!, 0, mainAxisExtent);

        return Scrollable(
          axisDirection: axisDirection,
          controller: _controller,
          physics: physics,
          viewportBuilder: (BuildContext context, ViewportOffset position) {
            return Viewport(
              cacheExtent: 0.0,
              cacheExtentStyle: CacheExtentStyle.viewport,
              axisDirection: axisDirection,
              offset: position,
              clipBehavior: Clip.antiAlias,
              slivers: <Widget>[_buildSliverCarousel(theme)],
            );
          },
        );
      },
    );
  }
}

/// A sliver that displays its box children in a linear array with a fixed extent
/// per item.
///
/// _To learn more about slivers, see [CustomScrollView.slivers]._
///
/// This sliver list arranges its children in a line along the main axis starting
/// at offset zero and without gaps. Each child is constrained to a fixed extent
/// along the main axis and the [SliverConstraints.crossAxisExtent]
/// along the cross axis. The difference between this and a list view with a fixed
/// extent is the first item and last item can be collapsed a little during scrolling
/// transition. This compression is controlled by the `minExtent` property and
/// aligns with the [Material Design Carousel specifications]
/// (https://m3.material.io/components/carousel/guidelines#96c5c157-fe5b-4ee3-a9b4-72bf8efab7e9).
class _SliverFixedExtentCarousel extends SliverMultiBoxAdaptorWidget {
  const _SliverFixedExtentCarousel({
    required super.delegate,
    required this.minExtent,
    required this.itemExtent,
  });

  final double itemExtent;
  final double minExtent;

  @override
  RenderSliverFixedExtentBoxAdaptor createRenderObject(BuildContext context) {
    final SliverMultiBoxAdaptorElement element = context as SliverMultiBoxAdaptorElement;
    return _RenderSliverFixedExtentCarousel(
      childManager: element,
      minExtent: minExtent,
      maxExtent: itemExtent,
    );
  }

  @override
  void updateRenderObject(BuildContext context, _RenderSliverFixedExtentCarousel renderObject) {
    renderObject.maxExtent = itemExtent;
    renderObject.minExtent = minExtent;
  }
}

class _RenderSliverFixedExtentCarousel extends RenderSliverFixedExtentBoxAdaptor {
  _RenderSliverFixedExtentCarousel({
    required super.childManager,
    required double maxExtent,
    required double minExtent,
  }) : _maxExtent = maxExtent,
       _minExtent = minExtent;

  double get maxExtent => _maxExtent;
  double _maxExtent;
  set maxExtent(double value) {
    if (_maxExtent == value) {
      return;
    }
    _maxExtent = value;
    markNeedsLayout();
  }

  double get minExtent => _minExtent;
  double _minExtent;
  set minExtent(double value) {
    if (_minExtent == value) {
      return;
    }
    _minExtent = value;
    markNeedsLayout();
  }

  // This implements the [itemExtentBuilder] callback.
  double _buildItemExtent(int index, SliverLayoutDimensions currentLayoutDimensions) {
    final int firstVisibleIndex = (constraints.scrollOffset / maxExtent).floor();

    // Calculate how many items have been completely scroll off screen.
    final int offscreenItems = (constraints.scrollOffset / maxExtent).floor();

    // If an item is partially off screen and partially on screen,
    // `constraints.scrollOffset` must be greater than
    // `offscreenItems * maxExtent`, so the difference between these two is how
    // much the current first visible item is off screen.
    final double offscreenExtent = constraints.scrollOffset - offscreenItems * maxExtent;

    // If there is not enough space to place the last visible item but the remaining
    // space is larger than `minExtent`, the extent for last item should be at
    // least the remaining extent to ensure a smooth size transition.
    final double effectiveMinExtent = math.max(
      constraints.remainingPaintExtent % maxExtent,
      minExtent,
    );

    // Two special cases are the first and last visible items. Other items' extent
    // should all return `maxExtent`.
    if (index == firstVisibleIndex) {
      final double effectiveExtent = maxExtent - offscreenExtent;
      return math.max(effectiveExtent, effectiveMinExtent);
    }

    final double scrollOffsetForLastIndex =
        constraints.scrollOffset + constraints.remainingPaintExtent;
    if (index == getMaxChildIndexForScrollOffset(scrollOffsetForLastIndex, maxExtent)) {
      return clampDouble(
        scrollOffsetForLastIndex - maxExtent * index,
        effectiveMinExtent,
        maxExtent,
      );
    }

    return maxExtent;
  }

  late SliverLayoutDimensions _currentLayoutDimensions;

  @override
  void performLayout() {
    _currentLayoutDimensions = SliverLayoutDimensions(
      scrollOffset: constraints.scrollOffset,
      precedingScrollExtent: constraints.precedingScrollExtent,
      viewportMainAxisExtent: constraints.viewportMainAxisExtent,
      crossAxisExtent: constraints.crossAxisExtent,
    );
    super.performLayout();
  }

  /// The layout offset for the child with the given index.
  @override
  double indexToLayoutOffset(
    @Deprecated(
      'The itemExtent is already available within the scope of this function. '
      'This feature was deprecated after v3.20.0-7.0.pre.',
    )
    double itemExtent,
    int index,
  ) {
    final int firstVisibleIndex = (constraints.scrollOffset / maxExtent).floor();

    // If there is not enough space to place the last visible item but the remaining
    // space is larger than `minExtent`, the extent for last item should be at
    // least the remaining extent to make sure a smooth size transition.
    final double effectiveMinExtent = math.max(
      constraints.remainingPaintExtent % maxExtent,
      minExtent,
    );
    if (index == firstVisibleIndex) {
      final double firstVisibleItemExtent = _buildItemExtent(index, _currentLayoutDimensions);

      // If the first item is collapsed to be less than `effectiveMinExtent`,
      // then it should stop changing its size and should start to scroll off screen.
      if (firstVisibleItemExtent <= effectiveMinExtent) {
        return maxExtent * index - effectiveMinExtent + maxExtent;
      }
      return constraints.scrollOffset;
    }
    return maxExtent * index;
  }

  /// The minimum child index that is visible at the given scroll offset.
  @override
  int getMinChildIndexForScrollOffset(
    double scrollOffset,
    @Deprecated(
      'The itemExtent is already available within the scope of this function. '
      'This feature was deprecated after v3.20.0-7.0.pre.',
    )
    double itemExtent,
  ) {
    final int firstVisibleIndex = (constraints.scrollOffset / maxExtent).floor();
    return math.max(firstVisibleIndex, 0);
  }

  /// The maximum child index that is visible at the given scroll offset.
  @override
  int getMaxChildIndexForScrollOffset(
    double scrollOffset,
    @Deprecated(
      'The itemExtent is already available within the scope of this function. '
      'This feature was deprecated after v3.20.0-7.0.pre.',
    )
    double itemExtent,
  ) {
    if (maxExtent > 0.0) {
      final double actual = scrollOffset / maxExtent - 1;
      final int round = actual.round();
      if ((actual * maxExtent - round * maxExtent).abs() < precisionErrorTolerance) {
        return math.max(0, round);
      }
      return math.max(0, actual.ceil());
    }
    return 0;
  }

  @override
  double? get itemExtent => null;

  @override
  ItemExtentBuilder? get itemExtentBuilder => _buildItemExtent;
}

/// A sliver that arranges its box children in a linear array, constraining them
/// to specific weights determined by the [weights] property.
///
/// _To learn more about slivers, see [CustomScrollView.slivers]._
///
/// This sliver arranges its children in a line along the main axis, starting
/// at offset zero without gaps. Each child is constrained to its corresponding
/// weight along the main axis and to the [SliverConstraints.crossAxisExtent]
/// along the cross axis.
///
/// See [CarouselView.weighted] to get more calculation explanations.
class _SliverWeightedCarousel extends SliverMultiBoxAdaptorWidget {
  const _SliverWeightedCarousel({
    required super.delegate,
    required this.consumeMaxWeight,
    required this.shrinkExtent,
    required this.weights,
  });

  // Determine whether extra scroll offset should be calculate so that every
  // item have a chance to scroll to the maximum extent.
  //
  // This is useful when the leading/trailing items have smaller weights, such
  // as [1, 7], and [3, 2, 1].
  final bool consumeMaxWeight;

  // The starting extent for items when they gradually show on/off screen.
  //
  // This is useful to avoid a hairline shape. This value should also smaller
  // than the last item extent to make sure a smooth transition. So in calculation,
  // this is limited to [0, weight for the last visible item].
  final double shrinkExtent;

  // The layout arrangement.
  //
  // When items are laying out, each item will be arranged based on the order of
  // the weights and the extent is based on the corresponding weight out of the
  // sum of weights. The length of weights means how many items we can put in the
  // view at a time.
  final List<int> weights;

  @override
  RenderSliverFixedExtentBoxAdaptor createRenderObject(BuildContext context) {
    final SliverMultiBoxAdaptorElement element = context as SliverMultiBoxAdaptorElement;
    return _RenderSliverWeightedCarousel(
      childManager: element,
      consumeMaxWeight: consumeMaxWeight,
      shrinkExtent: shrinkExtent,
      weights: weights,
    );
  }

  @override
  void updateRenderObject(BuildContext context, _RenderSliverWeightedCarousel renderObject) {
    renderObject
      ..consumeMaxWeight = consumeMaxWeight
      ..shrinkExtent = shrinkExtent
      ..weights = weights;
  }
}

// A sliver that places its box children in a linear array and constrains them
// to have the corresponding weight which is determined by [weights].
class _RenderSliverWeightedCarousel extends RenderSliverFixedExtentBoxAdaptor {
  _RenderSliverWeightedCarousel({
    required super.childManager,
    required bool consumeMaxWeight,
    required double shrinkExtent,
    required List<int> weights,
  }) : _consumeMaxWeight = consumeMaxWeight,
       _shrinkExtent = shrinkExtent,
       _weights = weights;

  bool get consumeMaxWeight => _consumeMaxWeight;
  bool _consumeMaxWeight;
  set consumeMaxWeight(bool value) {
    if (_consumeMaxWeight == value) {
      return;
    }
    _consumeMaxWeight = value;
    markNeedsLayout();
  }

  double get shrinkExtent => _shrinkExtent;
  double _shrinkExtent;
  set shrinkExtent(double value) {
    if (_shrinkExtent == value) {
      return;
    }
    _shrinkExtent = value;
    markNeedsLayout();
  }

  List<int> get weights => _weights;
  List<int> _weights;
  set weights(List<int> value) {
    if (_weights == value) {
      return;
    }
    _weights = value;
    markNeedsLayout();
  }

  late SliverLayoutDimensions _currentLayoutDimensions;

  // This is to implement the itemExtentBuilder callback to return each item extent
  // while scrolling.
  //
  // The given `index` is compared with `_firstVisibleItemIndex` to know how
  // many items are placed before the current one in the view.
  double _buildItemExtent(int index, SliverLayoutDimensions currentLayoutDimensions) {
    double extent;
    if (index == _firstVisibleItemIndex) {
      extent = math.max(_distanceToLeadingEdge, effectiveShrinkExtent);
    }
    // Calculate the extents of items located within the range defined by the
    // weights array relative to the first visible item. This allows us to
    // precisely determine each item's extent based on its initial extent
    // (calculated from the weights) and the scrolling progress (the off-screen
    // portion of the first item).
    else if (index > _firstVisibleItemIndex &&
        index - _firstVisibleItemIndex + 1 <= weights.length) {
      assert(index - _firstVisibleItemIndex < weights.length);
      final int currIndexOnWeightList = index - _firstVisibleItemIndex;
      final int currWeight = weights[currIndexOnWeightList];
      extent = extentUnit * currWeight; // initial extent
      final double progress = _firstVisibleItemOffscreenExtent / firstChildExtent;

      final int prevWeight = weights[currIndexOnWeightList - 1];
      final double finalIncrease = (prevWeight - currWeight) / weights.max;
      extent = extent + finalIncrease * progress * maxChildExtent;
    }
    // Calculate the extents of items located beyond the range defined by the
    // weights array relative to the first visible item. During scrolling transition,
    // it is possible that the number of visible items is larger than the length
    // of `weights`. The extra item extent should be calculated here to fill
    // the remaining space.
    else if (index > _firstVisibleItemIndex &&
        index - _firstVisibleItemIndex + 1 > weights.length) {
      double visibleItemsTotalExtent = _distanceToLeadingEdge;
      for (int i = _firstVisibleItemIndex + 1; i < index; i++) {
        visibleItemsTotalExtent += _buildItemExtent(i, currentLayoutDimensions);
      }
      extent = math.max(
        constraints.remainingPaintExtent - visibleItemsTotalExtent,
        effectiveShrinkExtent,
      );
    } else {
      extent = math.max(minChildExtent, effectiveShrinkExtent);
    }
    return extent;
  }

  // To ge the extent unit based on the viewport extent and the sum of weights.
  double get extentUnit =>
      constraints.viewportMainAxisExtent /
      (weights.reduce((int total, int extent) => total + extent));

  double get firstChildExtent => weights.first * extentUnit;
  double get maxChildExtent => weights.max * extentUnit;
  double get minChildExtent => weights.min * extentUnit;

  // The shrink extent for first and last visible items should be no larger
  // than [minChildExtent] to ensure a smooth transition.
  double get effectiveShrinkExtent => clampDouble(shrinkExtent, 0, minChildExtent);

  // The index of the first visible item. The returned value can be negative when
  // the leading items with smaller weights need to be fully expanded. For example,
  // assuming a weights [1, 7, 1], when item 0 is expanding to the maximum size
  // (with weight 7), we leave some space before item 0 assuming there is another
  // item -1 as the first visible item.
  int get _firstVisibleItemIndex {
    int smallerWeightCount = 0;
    for (final int weight in weights) {
      if (weight == weights.max) {
        break;
      }
      smallerWeightCount += 1;
    }
    int index;

    final double actual = constraints.scrollOffset / firstChildExtent;
    final int round = (constraints.scrollOffset / firstChildExtent).round();
    if ((actual - round).abs() < precisionErrorTolerance) {
      index = round;
    } else {
      index = actual.floor();
    }
    return consumeMaxWeight ? index - smallerWeightCount : index;
  }

  // This value indicates the scrolling progress of items following the first
  // item. It informs them how much the first item has moved off-screen,
  // enabling them to adjust their sizes (grow or shrink) accordingly.
  double get _firstVisibleItemOffscreenExtent {
    int index;
    final double actual = constraints.scrollOffset / firstChildExtent;
    final int round = (constraints.scrollOffset / firstChildExtent).round();
    if ((actual - round).abs() < precisionErrorTolerance) {
      index = round;
    } else {
      index = actual.floor();
    }
    return constraints.scrollOffset - index * firstChildExtent;
  }

  // Given the off-screen extent for the first visible item, we can know the
  // on-screen extent for the first visible item.
  double get _distanceToLeadingEdge => firstChildExtent - _firstVisibleItemOffscreenExtent;

  // Given an index, this method returns the layout offset for the item. The `index`
  // is firstly compared to `_firstVisibleItemIndex` and compute the distance
  // between them, then compute all the current extents for items that are located
  // in front.
  @override
  double indexToLayoutOffset(
    @Deprecated(
      'The itemExtent is already available within the scope of this function. '
      'This feature was deprecated after v3.20.0-7.0.pre.',
    )
    double itemExtent,
    int index,
  ) {
    if (index == _firstVisibleItemIndex) {
      if (_distanceToLeadingEdge <= effectiveShrinkExtent) {
        return constraints.scrollOffset - effectiveShrinkExtent + _distanceToLeadingEdge;
      }
      return constraints.scrollOffset;
    }
    double visibleItemsTotalExtent = _distanceToLeadingEdge;
    for (int i = _firstVisibleItemIndex + 1; i < index; i++) {
      visibleItemsTotalExtent += _buildItemExtent(i, _currentLayoutDimensions);
    }
    return constraints.scrollOffset + visibleItemsTotalExtent;
  }

  @override
  int getMinChildIndexForScrollOffset(
    double scrollOffset,
    @Deprecated(
      'The itemExtent is already available within the scope of this function. '
      'This feature was deprecated after v3.20.0-7.0.pre.',
    )
    double itemExtent,
  ) {
    return math.max(_firstVisibleItemIndex, 0);
  }

  @override
  int getMaxChildIndexForScrollOffset(
    double scrollOffset,
    @Deprecated(
      'The itemExtent is already available within the scope of this function. '
      'This feature was deprecated after v3.20.0-7.0.pre.',
    )
    double itemExtent,
  ) {
    final int? childCount = childManager.estimatedChildCount;
    if (childCount != null) {
      double visibleItemsTotalExtent = _distanceToLeadingEdge;
      for (int i = _firstVisibleItemIndex + 1; i < childCount; i++) {
        visibleItemsTotalExtent += _buildItemExtent(i, _currentLayoutDimensions);
        if (visibleItemsTotalExtent >= constraints.viewportMainAxisExtent) {
          return i;
        }
      }
    }
    return childCount ?? 0;
  }

  @override
  double computeMaxScrollOffset(
    SliverConstraints constraints,
    @Deprecated(
      'The itemExtent is already available within the scope of this function. '
      'This feature was deprecated after v3.20.0-7.0.pre.',
    )
    double itemExtent,
  ) {
    return childManager.childCount * maxChildExtent;
  }

  BoxConstraints _getChildConstraints(int index) {
    final double extent = itemExtentBuilder!(index, _currentLayoutDimensions)!;
    return constraints.asBoxConstraints(minExtent: extent, maxExtent: extent);
  }

  // This method is mostly the same as its parent class [RenderSliverFixedExtentList].
  // The difference is when we allow some space before the leading items or after
  // the trailing items with smaller weights, we leave extra scroll offset.
  // TODO(quncCccccc): add the calculation for the extra scroll offset on the super class to simplify the implementation here.
  @override
  void performLayout() {
    assert(
      (itemExtent != null && itemExtentBuilder == null) ||
          (itemExtent == null && itemExtentBuilder != null),
    );
    assert(itemExtentBuilder != null || (itemExtent!.isFinite && itemExtent! >= 0));

    final SliverConstraints constraints = this.constraints;
    childManager.didStartLayout();
    childManager.setDidUnderflow(false);

    final double scrollOffset = constraints.scrollOffset + constraints.cacheOrigin;
    assert(scrollOffset >= 0.0);
    final double remainingExtent = constraints.remainingCacheExtent;
    assert(remainingExtent >= 0.0);
    final double targetEndScrollOffset = scrollOffset + remainingExtent;
    _currentLayoutDimensions = SliverLayoutDimensions(
      scrollOffset: constraints.scrollOffset,
      precedingScrollExtent: constraints.precedingScrollExtent,
      viewportMainAxisExtent: constraints.viewportMainAxisExtent,
      crossAxisExtent: constraints.crossAxisExtent,
    );
    // TODO(Piinks): Clean up when deprecation expires.
    const double deprecatedExtraItemExtent = -1;

    final int firstIndex = getMinChildIndexForScrollOffset(scrollOffset, deprecatedExtraItemExtent);
    final int? targetLastIndex =
        targetEndScrollOffset.isFinite
            ? getMaxChildIndexForScrollOffset(targetEndScrollOffset, deprecatedExtraItemExtent)
            : null;

    if (firstChild != null) {
      final int leadingGarbage = calculateLeadingGarbage(firstIndex: firstIndex);
      final int trailingGarbage =
          targetLastIndex != null ? calculateTrailingGarbage(lastIndex: targetLastIndex) : 0;
      collectGarbage(leadingGarbage, trailingGarbage);
    } else {
      collectGarbage(0, 0);
    }

    if (firstChild == null) {
      final double layoutOffset = indexToLayoutOffset(deprecatedExtraItemExtent, firstIndex);
      if (!addInitialChild(index: firstIndex, layoutOffset: layoutOffset)) {
        // There are either no children, or we are past the end of all our children.
        final double max;
        if (firstIndex <= 0) {
          max = 0.0;
        } else {
          max = computeMaxScrollOffset(constraints, deprecatedExtraItemExtent);
        }
        geometry = SliverGeometry(scrollExtent: max, maxPaintExtent: max);
        childManager.didFinishLayout();
        return;
      }
    }

    RenderBox? trailingChildWithLayout;

    for (int index = indexOf(firstChild!) - 1; index >= firstIndex; --index) {
      final RenderBox? child = insertAndLayoutLeadingChild(_getChildConstraints(index));
      if (child == null) {
        // Items before the previously first child are no longer present.
        // Reset the scroll offset to offset all items prior and up to the
        // missing item. Let parent re-layout everything.
        geometry = SliverGeometry(
          scrollOffsetCorrection: indexToLayoutOffset(deprecatedExtraItemExtent, index),
        );
        return;
      }
      final SliverMultiBoxAdaptorParentData childParentData =
          child.parentData! as SliverMultiBoxAdaptorParentData;
      childParentData.layoutOffset = indexToLayoutOffset(deprecatedExtraItemExtent, index);
      assert(childParentData.index == index);
      trailingChildWithLayout ??= child;
    }

    if (trailingChildWithLayout == null) {
      firstChild!.layout(_getChildConstraints(indexOf(firstChild!)));
      final SliverMultiBoxAdaptorParentData childParentData =
          firstChild!.parentData! as SliverMultiBoxAdaptorParentData;
      childParentData.layoutOffset = indexToLayoutOffset(deprecatedExtraItemExtent, firstIndex);
      trailingChildWithLayout = firstChild;
    }

    // From the last item to the firstly encountered max item
    double extraLayoutOffset = 0;
    if (consumeMaxWeight) {
      for (int i = weights.length - 1; i >= 0; i--) {
        if (weights[i] == weights.max) {
          break;
        }
        extraLayoutOffset += weights[i] * extentUnit;
      }
    }

    double estimatedMaxScrollOffset = double.infinity;
    // Layout visible items after the first visible item.
    for (
      int index = indexOf(trailingChildWithLayout!) + 1;
      targetLastIndex == null || index <= targetLastIndex;
      ++index
    ) {
      RenderBox? child = childAfter(trailingChildWithLayout!);
      if (child == null || indexOf(child) != index) {
        child = insertAndLayoutChild(_getChildConstraints(index), after: trailingChildWithLayout);
        if (child == null) {
          // We have run out of children.
          estimatedMaxScrollOffset =
              indexToLayoutOffset(deprecatedExtraItemExtent, index) + extraLayoutOffset;
          break;
        }
      } else {
        child.layout(_getChildConstraints(index));
      }
      trailingChildWithLayout = child;
      final SliverMultiBoxAdaptorParentData childParentData =
          child.parentData! as SliverMultiBoxAdaptorParentData;
      assert(childParentData.index == index);
      childParentData.layoutOffset = indexToLayoutOffset(
        deprecatedExtraItemExtent,
        childParentData.index!,
      );
    }

    final int lastIndex = indexOf(lastChild!);
    final double leadingScrollOffset = indexToLayoutOffset(deprecatedExtraItemExtent, firstIndex);
    double trailingScrollOffset;

    if (lastIndex + 1 == childManager.childCount) {
      trailingScrollOffset = indexToLayoutOffset(deprecatedExtraItemExtent, lastIndex);

      trailingScrollOffset += math.max(
        weights.last * extentUnit,
        _buildItemExtent(lastIndex, _currentLayoutDimensions),
      );
      trailingScrollOffset += extraLayoutOffset;
    } else {
      trailingScrollOffset = indexToLayoutOffset(deprecatedExtraItemExtent, lastIndex + 1);
    }

    assert(debugAssertChildListIsNonEmptyAndContiguous());
    assert(indexOf(firstChild!) == firstIndex);
    assert(targetLastIndex == null || lastIndex <= targetLastIndex);

    estimatedMaxScrollOffset = math.min(
      estimatedMaxScrollOffset,
      estimateMaxScrollOffset(
        constraints,
        firstIndex: firstIndex,
        lastIndex: lastIndex,
        leadingScrollOffset: leadingScrollOffset,
        trailingScrollOffset: trailingScrollOffset,
      ),
    );

    final double paintExtent = calculatePaintOffset(
      constraints,
      from: consumeMaxWeight ? 0 : leadingScrollOffset,
      to: trailingScrollOffset,
    );

    final double cacheExtent = calculateCacheOffset(
      constraints,
      from: consumeMaxWeight ? 0 : leadingScrollOffset,
      to: trailingScrollOffset,
    );

    final double targetEndScrollOffsetForPaint =
        constraints.scrollOffset + constraints.remainingPaintExtent;
    final int? targetLastIndexForPaint =
        targetEndScrollOffsetForPaint.isFinite
            ? getMaxChildIndexForScrollOffset(
              targetEndScrollOffsetForPaint,
              deprecatedExtraItemExtent,
            )
            : null;

    geometry = SliverGeometry(
      scrollExtent: estimatedMaxScrollOffset,
      paintExtent: paintExtent,
      cacheExtent: cacheExtent,
      maxPaintExtent: estimatedMaxScrollOffset,
      // Conservative to avoid flickering away the clip during scroll.
      hasVisualOverflow:
          (targetLastIndexForPaint != null && lastIndex >= targetLastIndexForPaint) ||
          constraints.scrollOffset > 0.0,
    );

    // We may have started the layout while scrolled to the end, which would not
    // expose a new child.
    if (estimatedMaxScrollOffset == trailingScrollOffset) {
      childManager.setDidUnderflow(true);
    }
    childManager.didFinishLayout();
  }

  @override
  double? get itemExtent => null;

  /// The main-axis extent builder of each item.
  ///
  /// If this is non-null, the [itemExtent] must be null.
  /// If this is null, the [itemExtent] must be non-null.
  @override
  ItemExtentBuilder? get itemExtentBuilder => _buildItemExtent;
}

/// Scroll physics used by a [CarouselView].
///
/// These physics cause the carousel item to snap to item boundaries.
///
/// See also:
///
///  * [ScrollPhysics], the base class which defines the API for scrolling
///    physics.
///  * [PageScrollPhysics], scroll physics used by a [PageView].
class CarouselScrollPhysics extends ScrollPhysics {
  /// Creates physics for a [CarouselView].
  const CarouselScrollPhysics({super.parent});

  @override
  CarouselScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return CarouselScrollPhysics(parent: buildParent(ancestor));
  }

  double _getTargetPixels(_CarouselPosition position, Tolerance tolerance, double velocity) {
    double fraction;

    if (position.itemExtent != null) {
      fraction = position.itemExtent! / position.viewportDimension;
    } else {
      assert(position.flexWeights != null);
      fraction = position.flexWeights!.first / position.flexWeights!.sum;
    }

    final double itemWidth = position.viewportDimension * fraction;

    final double actual = math.max(0.0, position.pixels) / itemWidth;
    final double round = actual.roundToDouble();
    double item;
    if ((actual - round).abs() < precisionErrorTolerance) {
      item = round;
    } else {
      item = actual;
    }
    if (velocity < -tolerance.velocity) {
      item -= 0.5;
    } else if (velocity > tolerance.velocity) {
      item += 0.5;
    }
    return item.roundToDouble() * itemWidth;
  }

  @override
  Simulation? createBallisticSimulation(ScrollMetrics position, double velocity) {
    assert(
      position is _CarouselPosition,
      'CarouselScrollPhysics can only be used with Scrollables that uses '
      'the CarouselController',
    );

    final _CarouselPosition metrics = position as _CarouselPosition;
    if ((velocity <= 0.0 && metrics.pixels <= metrics.minScrollExtent) ||
        (velocity >= 0.0 && metrics.pixels >= metrics.maxScrollExtent)) {
      return super.createBallisticSimulation(metrics, velocity);
    }

    final Tolerance tolerance = toleranceFor(metrics);
    final double target = _getTargetPixels(metrics, tolerance, velocity);
    if (target != metrics.pixels) {
      return ScrollSpringSimulation(spring, metrics.pixels, target, velocity, tolerance: tolerance);
    }
    return null;
  }

  @override
  bool get allowImplicitScrolling => true;
}

/// Metrics for a [CarouselView].
class _CarouselMetrics extends FixedScrollMetrics {
  /// Creates an immutable snapshot of values associated with a [CarouselView].
  _CarouselMetrics({
    required super.minScrollExtent,
    required super.maxScrollExtent,
    required super.pixels,
    required super.viewportDimension,
    required super.axisDirection,
    this.itemExtent,
    this.flexWeights,
    this.consumeMaxWeight,
    required super.devicePixelRatio,
  });

  /// Extent for the carousel item.
  ///
  /// Used to compute the first item from the current [pixels].
  final double? itemExtent;

  /// The fraction of the viewport that the first item occupies.
  ///
  /// Used to compute the extent of each carousel item from the current [pixels],
  /// if [itemExtent] is null.
  final List<int>? flexWeights;

  /// Determine whether each child can be expanded to occupy the maximum weight while scrolling.
  final bool? consumeMaxWeight;

  @override
  _CarouselMetrics copyWith({
    double? minScrollExtent,
    double? maxScrollExtent,
    double? pixels,
    double? viewportDimension,
    AxisDirection? axisDirection,
    double? itemExtent,
    List<int>? flexWeights,
    bool? consumeMaxWeight,
    double? devicePixelRatio,
  }) {
    return _CarouselMetrics(
      minScrollExtent: minScrollExtent ?? (hasContentDimensions ? this.minScrollExtent : null),
      maxScrollExtent: maxScrollExtent ?? (hasContentDimensions ? this.maxScrollExtent : null),
      pixels: pixels ?? (hasPixels ? this.pixels : null),
      viewportDimension:
          viewportDimension ?? (hasViewportDimension ? this.viewportDimension : null),
      axisDirection: axisDirection ?? this.axisDirection,
      itemExtent: itemExtent ?? this.itemExtent,
      flexWeights: flexWeights ?? this.flexWeights,
      consumeMaxWeight: consumeMaxWeight ?? this.consumeMaxWeight,
      devicePixelRatio: devicePixelRatio ?? this.devicePixelRatio,
    );
  }
}

class _CarouselPosition extends ScrollPositionWithSingleContext implements _CarouselMetrics {
  _CarouselPosition({
    required super.physics,
    required super.context,
    this.initialItem = 0,
    double? itemExtent,
    List<int>? flexWeights,
    bool consumeMaxWeight = true,
    super.oldPosition,
  }) : assert(
         flexWeights != null && itemExtent == null || flexWeights == null && itemExtent != null,
       ),
       _itemToShowOnStartup = initialItem.toDouble(),
       _consumeMaxWeight = consumeMaxWeight,
       super(initialPixels: null);

  int initialItem;
  final double _itemToShowOnStartup;
  // When the viewport has a zero-size, the item can not
  // be retrieved by `getItemFromPixels`, so we need to cache the item
  // for use when resizing the viewport to non-zero next time.
  double? _cachedItem;

  @override
  bool get consumeMaxWeight => _consumeMaxWeight;
  bool _consumeMaxWeight;
  set consumeMaxWeight(bool value) {
    if (_consumeMaxWeight == value) {
      return;
    }
    if (hasPixels && flexWeights != null) {
      final double leadingItem = updateLeadingItem(flexWeights, value);
      final double newPixel = getPixelsFromItem(leadingItem, flexWeights, itemExtent);
      forcePixels(newPixel);
    }
    _consumeMaxWeight = value;
  }

  @override
  double? get itemExtent => _itemExtent;
  double? _itemExtent;
  set itemExtent(double? value) {
    if (_itemExtent == value) {
      return;
    }
    if (hasPixels && _itemExtent != null) {
      final double leadingItem = getItemFromPixels(pixels, viewportDimension);
      final double newPixel = getPixelsFromItem(leadingItem, flexWeights, value);
      forcePixels(newPixel);
    }
    _itemExtent = value;
  }

  @override
  List<int>? get flexWeights => _flexWeights;
  List<int>? _flexWeights;
  set flexWeights(List<int>? value) {
    if (flexWeights == value) {
      return;
    }
    final List<int>? oldWeights = _flexWeights;
    if (hasPixels && oldWeights != null) {
      final double leadingItem = updateLeadingItem(value, consumeMaxWeight);
      final double newPixel = getPixelsFromItem(leadingItem, value, itemExtent);
      forcePixels(newPixel);
    }
    _flexWeights = value;
  }

  double updateLeadingItem(List<int>? newFlexWeights, bool newConsumeMaxWeight) {
    final double maxItem;
    if (hasPixels && flexWeights != null) {
      final double leadingItem = getItemFromPixels(pixels, viewportDimension);
      maxItem =
          consumeMaxWeight ? leadingItem : leadingItem + flexWeights!.indexOf(flexWeights!.max);
    } else {
      maxItem = _itemToShowOnStartup;
    }
    if (newFlexWeights != null && !newConsumeMaxWeight) {
      int smallerWeights = 0;
      for (final int weight in newFlexWeights) {
        if (weight == newFlexWeights.max) {
          break;
        }
        smallerWeights += 1;
      }
      return maxItem - smallerWeights;
    }
    return maxItem;
  }

  double getItemFromPixels(double pixels, double viewportDimension) {
    assert(viewportDimension > 0.0);
    double fraction;
    if (itemExtent != null) {
      fraction = itemExtent! / viewportDimension;
    } else {
      // If itemExtent is null, flexWeights cannot be null.
      assert(flexWeights != null);
      fraction = flexWeights!.first / flexWeights!.sum;
    }

    final double actual = math.max(0.0, pixels) / (viewportDimension * fraction);
    final double round = actual.roundToDouble();
    if ((actual - round).abs() < precisionErrorTolerance) {
      return round;
    }
    return actual;
  }

  double getPixelsFromItem(double item, List<int>? flexWeights, double? itemExtent) {
    double fraction;
    if (itemExtent != null) {
      fraction = itemExtent / viewportDimension;
    } else {
      // If itemExtent is null, flexWeights cannot be null.
      assert(flexWeights != null);
      fraction = flexWeights!.first / flexWeights.sum;
    }

    return item * viewportDimension * fraction;
  }

  @override
  bool applyViewportDimension(double viewportDimension) {
    final double? oldViewportDimensions = hasViewportDimension ? this.viewportDimension : null;
    if (viewportDimension == oldViewportDimensions) {
      return true;
    }
    final bool result = super.applyViewportDimension(viewportDimension);
    final double? oldPixels = hasPixels ? pixels : null;
    double item;
    if (oldPixels == null) {
      item = updateLeadingItem(flexWeights, consumeMaxWeight);
    } else if (oldViewportDimensions == 0.0) {
      // If resize from zero, we should use the _cachedItem to recover the state.
      item = _cachedItem!;
    } else {
      item = getItemFromPixels(oldPixels, oldViewportDimensions!);
    }
    final double newPixels = getPixelsFromItem(item, flexWeights, itemExtent);
    // If the viewportDimension is zero, cache the item
    // in case the viewport is resized to be non-zero.
    _cachedItem = (viewportDimension == 0.0) ? item : null;

    if (newPixels != oldPixels) {
      correctPixels(newPixels);
      return false;
    }
    return result;
  }

  @override
  _CarouselMetrics copyWith({
    double? minScrollExtent,
    double? maxScrollExtent,
    double? pixels,
    double? viewportDimension,
    AxisDirection? axisDirection,
    double? itemExtent,
    List<int>? flexWeights,
    bool? consumeMaxWeight,
    double? devicePixelRatio,
  }) {
    return _CarouselMetrics(
      minScrollExtent: minScrollExtent ?? (hasContentDimensions ? this.minScrollExtent : null),
      maxScrollExtent: maxScrollExtent ?? (hasContentDimensions ? this.maxScrollExtent : null),
      pixels: pixels ?? (hasPixels ? this.pixels : null),
      viewportDimension:
          viewportDimension ?? (hasViewportDimension ? this.viewportDimension : null),
      axisDirection: axisDirection ?? this.axisDirection,
      itemExtent: itemExtent ?? this.itemExtent,
      flexWeights: flexWeights ?? this.flexWeights,
      consumeMaxWeight: consumeMaxWeight ?? this.consumeMaxWeight,
      devicePixelRatio: devicePixelRatio ?? this.devicePixelRatio,
    );
  }
}

/// A controller for [CarouselView].
///
/// Using a carousel controller helps to show the first visible item on the
/// carousel list.
class CarouselController extends ScrollController {
  /// Creates a carousel controller.
  CarouselController({this.initialItem = 0});

  /// The item that expands to the maximum size when first creating the [CarouselView].
  final int initialItem;

  _CarouselViewState? _carouselState;

  // ignore: use_setters_to_change_properties
  void _attach(_CarouselViewState anchor) {
    _carouselState = anchor;
  }

  void _detach(_CarouselViewState anchor) {
    if (_carouselState == anchor) {
      _carouselState = null;
    }
  }

  @override
  ScrollPosition createScrollPosition(
    ScrollPhysics physics,
    ScrollContext context,
    ScrollPosition? oldPosition,
  ) {
    assert(_carouselState != null);
    return _CarouselPosition(
      physics: physics,
      context: context,
      initialItem: initialItem,
      itemExtent: _carouselState!._itemExtent,
      consumeMaxWeight: _carouselState!._consumeMaxWeight,
      flexWeights: _carouselState!._flexWeights,
      oldPosition: oldPosition,
    );
  }

  @override
  void attach(ScrollPosition position) {
    super.attach(position);
    final _CarouselPosition carouselPosition = position as _CarouselPosition;
    carouselPosition.flexWeights = _carouselState!._flexWeights;
    carouselPosition.itemExtent = _carouselState!._itemExtent;
    carouselPosition.consumeMaxWeight = _carouselState!._consumeMaxWeight;
  }
}
