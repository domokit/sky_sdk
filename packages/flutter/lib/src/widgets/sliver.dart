// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'animated_scroll_view.dart';
/// @docImport 'container.dart';
/// @docImport 'implicit_animations.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'sliver_fill.dart';
/// @docImport 'sliver_persistent_header.dart';
/// @docImport 'sliver_prototype_extent_list.dart';
/// @docImport 'text.dart';
/// @docImport 'two_dimensional_viewport.dart';
/// @docImport 'viewport.dart';
/// @docImport 'visibility.dart';
library;

import 'dart:collection' show HashMap, SplayTreeMap;
import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

import 'automatic_keep_alive.dart';
import 'basic.dart';
import 'framework.dart';
import 'scroll_delegate.dart';

/// A base class for slivers that have [KeepAlive] children.
///
/// See also:
///
/// * [KeepAlive], which marks whether its child widget should be kept alive.
/// * [SliverChildBuilderDelegate] and [SliverChildListDelegate], slivers
///    which make use of the keep alive functionality through the
///    `addAutomaticKeepAlives` property.
/// * [SliverGrid] and [SliverList], two sliver widgets that are commonly
///    wrapped with [KeepAlive] widgets to preserve their sliver child subtrees.
abstract class SliverWithKeepAliveWidget extends RenderObjectWidget {
  /// Initializes fields for subclasses.
  const SliverWithKeepAliveWidget({super.key});

  @override
  RenderSliverWithKeepAliveMixin createRenderObject(BuildContext context);
}

/// A base class for slivers that have multiple box children.
///
/// Helps subclasses build their children lazily using a [SliverChildDelegate].
///
/// The widgets returned by the [delegate] are cached and the delegate is only
/// consulted again if it changes and the new delegate's
/// [SliverChildDelegate.shouldRebuild] method returns true.
abstract class SliverMultiBoxAdaptorWidget extends SliverWithKeepAliveWidget {
  /// Initializes fields for subclasses.
  const SliverMultiBoxAdaptorWidget({super.key, required this.delegate});

  /// {@template flutter.widgets.SliverMultiBoxAdaptorWidget.delegate}
  /// The delegate that provides the children for this widget.
  ///
  /// The children are constructed lazily using this delegate to avoid creating
  /// more children than are visible through the [Viewport].
  ///
  /// ## Using more than one delegate in a [Viewport]
  ///
  /// If multiple delegates are used in a single scroll view, the first child of
  /// each delegate will always be laid out, even if it extends beyond the
  /// currently viewable area. This is because at least one child is required in
  /// order to estimate the max scroll offset for the whole scroll view, as it
  /// uses the currently built children to estimate the remaining children's
  /// extent.
  ///
  /// See also:
  ///
  ///  * [SliverChildBuilderDelegate] and [SliverChildListDelegate], which are
  ///    commonly used subclasses of [SliverChildDelegate] that use a builder
  ///    callback and an explicit child list, respectively.
  /// {@endtemplate}
  final SliverChildDelegate delegate;

  @override
  SliverMultiBoxAdaptorElement createElement() => SliverMultiBoxAdaptorElement(this);

  @override
  RenderSliverMultiBoxAdaptor createRenderObject(BuildContext context);

  /// Returns an estimate of the max scroll extent for all the children.
  ///
  /// Subclasses should override this function if they have additional
  /// information about their max scroll extent.
  ///
  /// This is used by [SliverMultiBoxAdaptorElement] to implement part of the
  /// [RenderSliverBoxChildManager] API.
  ///
  /// The default implementation defers to [delegate] via its
  /// [SliverChildDelegate.estimateMaxScrollOffset] method.
  double? estimateMaxScrollOffset(
    SliverConstraints? constraints,
    int firstIndex,
    int lastIndex,
    double leadingScrollOffset,
    double trailingScrollOffset,
  ) {
    assert(lastIndex >= firstIndex);
    return delegate.estimateMaxScrollOffset(
      firstIndex,
      lastIndex,
      leadingScrollOffset,
      trailingScrollOffset,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<SliverChildDelegate>('delegate', delegate));
  }
}

/// A sliver that places multiple box children in a linear array along the main
/// axis.
///
/// _To learn more about slivers, see [CustomScrollView.slivers]._
///
/// Each child is forced to have the [SliverConstraints.crossAxisExtent] in the
/// cross axis but determines its own main axis extent.
///
/// [SliverList] determines its scroll offset by "dead reckoning" because
/// children outside the visible part of the sliver are not materialized, which
/// means [SliverList] cannot learn their main axis extent. Instead, newly
/// materialized children are placed adjacent to existing children.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=ORiTTaVY6mM}
///
/// If the children have a fixed extent in the main axis, consider using
/// [SliverFixedExtentList] rather than [SliverList] because
/// [SliverFixedExtentList] does not need to perform layout on its children to
/// obtain their extent in the main axis and is therefore more efficient.
///
/// {@macro flutter.widgets.SliverChildDelegate.lifecycle}
///
/// See also:
///
///  * <https://docs.flutter.dev/ui/layout/scrolling/slivers>, a description
///    of what slivers are and how to use them.
///  * [SliverFixedExtentList], which is more efficient for children with
///    the same extent in the main axis.
///  * [SliverPrototypeExtentList], which is similar to [SliverFixedExtentList]
///    except that it uses a prototype list item instead of a pixel value to define
///    the main axis extent of each item.
///  * [SliverAnimatedList], which animates items added to or removed from a
///    list.
///  * [SliverGrid], which places multiple children in a two dimensional grid.
///  * [SliverAnimatedGrid], a sliver which animates items when they are
///    inserted into or removed from a grid.
class SliverList extends SliverMultiBoxAdaptorWidget {
  /// Creates a sliver that places box children in a linear array.
  const SliverList({super.key, required super.delegate});

  /// A sliver that places multiple box children in a linear array along the main
  /// axis.
  ///
  /// This constructor is appropriate for sliver lists with a large (or
  /// infinite) number of children because the builder is called only for those
  /// children that are actually visible.
  ///
  /// Providing a non-null `itemCount` improves the ability of the [SliverList]
  /// to estimate the maximum scroll extent.
  ///
  /// `itemBuilder` will be called only with indices greater than or equal to
  /// zero and less than `itemCount`.
  ///
  /// {@macro flutter.widgets.ListView.builder.itemBuilder}
  ///
  /// {@macro flutter.widgets.PageView.findChildIndexCallback}
  ///
  /// The `addAutomaticKeepAlives` argument corresponds to the
  /// [SliverChildBuilderDelegate.addAutomaticKeepAlives] property. The
  /// `addRepaintBoundaries` argument corresponds to the
  /// [SliverChildBuilderDelegate.addRepaintBoundaries] property. The
  /// `addSemanticIndexes` argument corresponds to the
  /// [SliverChildBuilderDelegate.addSemanticIndexes] property.
  ///
  /// {@tool snippet}
  /// This example, which would be provided in [CustomScrollView.slivers],
  /// shows an infinite number of items in varying shades of blue:
  ///
  /// ```dart
  /// SliverList.builder(
  ///   itemBuilder: (BuildContext context, int index) {
  ///     return Container(
  ///       alignment: Alignment.center,
  ///       color: Colors.lightBlue[100 * (index % 9)],
  ///       child: Text('list item $index'),
  ///     );
  ///   },
  /// )
  /// ```
  /// {@end-tool}
  SliverList.builder({
    super.key,
    required NullableIndexedWidgetBuilder itemBuilder,
    ChildIndexGetter? findChildIndexCallback,
    int? itemCount,
    bool addAutomaticKeepAlives = true,
    bool addRepaintBoundaries = true,
    bool addSemanticIndexes = true,
  }) : super(
         delegate: SliverChildBuilderDelegate(
           itemBuilder,
           findChildIndexCallback: findChildIndexCallback,
           childCount: itemCount,
           addAutomaticKeepAlives: addAutomaticKeepAlives,
           addRepaintBoundaries: addRepaintBoundaries,
           addSemanticIndexes: addSemanticIndexes,
         ),
       );

  /// A sliver that places multiple box children, separated by box widgets, in a
  /// linear array along the main axis.
  ///
  /// This constructor is appropriate for sliver lists with a large (or
  /// infinite) number of children because the builder is called only for those
  /// children that are actually visible.
  ///
  /// Providing a non-null `itemCount` improves the ability of the [SliverList]
  /// to estimate the maximum scroll extent.
  ///
  /// `itemBuilder` will be called only with indices greater than or equal to
  /// zero and less than `itemCount`.
  ///
  /// {@macro flutter.widgets.ListView.builder.itemBuilder}
  ///
  /// {@macro flutter.widgets.PageView.findChildIndexCallback}
  ///
  ///
  /// The `separatorBuilder` is similar to `itemBuilder`, except it is the widget
  /// that gets placed between itemBuilder(context, index) and itemBuilder(context, index + 1).
  ///
  /// The `addAutomaticKeepAlives` argument corresponds to the
  /// [SliverChildBuilderDelegate.addAutomaticKeepAlives] property. The
  /// `addRepaintBoundaries` argument corresponds to the
  /// [SliverChildBuilderDelegate.addRepaintBoundaries] property. The
  /// `addSemanticIndexes` argument corresponds to the
  /// [SliverChildBuilderDelegate.addSemanticIndexes] property.
  ///
  /// {@tool snippet}
  /// This example shows how to create a [SliverList] whose [Container] items
  /// are separated by [Divider]s. The [SliverList] would be provided in
  /// [CustomScrollView.slivers].
  ///
  /// ```dart
  /// SliverList.separated(
  ///   itemBuilder: (BuildContext context, int index) {
  ///     return Container(
  ///       alignment: Alignment.center,
  ///       color: Colors.lightBlue[100 * (index % 9)],
  ///       child: Text('list item $index'),
  ///     );
  ///   },
  ///   separatorBuilder: (BuildContext context, int index) => const Divider(),
  /// )
  /// ```
  /// {@end-tool}
  SliverList.separated({
    super.key,
    required NullableIndexedWidgetBuilder itemBuilder,
    ChildIndexGetter? findChildIndexCallback,
    required NullableIndexedWidgetBuilder separatorBuilder,
    int? itemCount,
    bool addAutomaticKeepAlives = true,
    bool addRepaintBoundaries = true,
    bool addSemanticIndexes = true,
  }) : super(
         delegate: SliverChildBuilderDelegate(
           (BuildContext context, int index) {
             final int itemIndex = index ~/ 2;
             final Widget? widget;
             if (index.isEven) {
               widget = itemBuilder(context, itemIndex);
             } else {
               widget = separatorBuilder(context, itemIndex);
               assert(() {
                 if (widget == null) {
                   throw FlutterError('separatorBuilder cannot return null.');
                 }
                 return true;
               }());
             }
             return widget;
           },
           findChildIndexCallback: findChildIndexCallback,
           childCount: itemCount == null ? null : math.max(0, itemCount * 2 - 1),
           addAutomaticKeepAlives: addAutomaticKeepAlives,
           addRepaintBoundaries: addRepaintBoundaries,
           addSemanticIndexes: addSemanticIndexes,
           semanticIndexCallback: (Widget _, int index) {
             return index.isEven ? index ~/ 2 : null;
           },
         ),
       );

  /// A sliver that places multiple box children in a linear array along the main
  /// axis.
  ///
  /// This constructor uses a list of [Widget]s to build the sliver.
  ///
  /// The `addAutomaticKeepAlives` argument corresponds to the
  /// [SliverChildBuilderDelegate.addAutomaticKeepAlives] property. The
  /// `addRepaintBoundaries` argument corresponds to the
  /// [SliverChildBuilderDelegate.addRepaintBoundaries] property. The
  /// `addSemanticIndexes` argument corresponds to the
  /// [SliverChildBuilderDelegate.addSemanticIndexes] property.
  ///
  /// {@tool snippet}
  /// This example, which would be provided in [CustomScrollView.slivers],
  /// shows a list containing two [Text] widgets:
  ///
  /// ```dart
  /// SliverList.list(
  ///   children: const <Widget>[
  ///     Text('Hello'),
  ///     Text('World!'),
  ///   ],
  /// );
  /// ```
  /// {@end-tool}
  SliverList.list({
    super.key,
    required List<Widget> children,
    bool addAutomaticKeepAlives = true,
    bool addRepaintBoundaries = true,
    bool addSemanticIndexes = true,
  }) : super(
         delegate: SliverChildListDelegate(
           children,
           addAutomaticKeepAlives: addAutomaticKeepAlives,
           addRepaintBoundaries: addRepaintBoundaries,
           addSemanticIndexes: addSemanticIndexes,
         ),
       );

  @override
  SliverMultiBoxAdaptorElement createElement() =>
      SliverMultiBoxAdaptorElement(this, replaceMovedChildren: true);

  @override
  RenderSliverList createRenderObject(BuildContext context) {
    final SliverMultiBoxAdaptorElement element = context as SliverMultiBoxAdaptorElement;
    return RenderSliverList(childManager: element);
  }
}

/// A sliver that places multiple box children with the same main axis extent in
/// a linear array.
///
/// _To learn more about slivers, see [CustomScrollView.slivers]._
///
/// [SliverFixedExtentList] places its children in a linear array along the main
/// axis starting at offset zero and without gaps. Each child is forced to have
/// the [itemExtent] in the main axis and the
/// [SliverConstraints.crossAxisExtent] in the cross axis.
///
/// [SliverFixedExtentList] is more efficient than [SliverList] because
/// [SliverFixedExtentList] does not need to perform layout on its children to
/// obtain their extent in the main axis.
///
/// {@tool snippet}
///
/// This example, which would be inserted into a [CustomScrollView.slivers]
/// list, shows an infinite number of items in varying shades of blue:
///
/// ```dart
/// SliverFixedExtentList(
///   itemExtent: 50.0,
///   delegate: SliverChildBuilderDelegate(
///     (BuildContext context, int index) {
///       return Container(
///         alignment: Alignment.center,
///         color: Colors.lightBlue[100 * (index % 9)],
///         child: Text('list item $index'),
///       );
///     },
///   ),
/// )
/// ```
/// {@end-tool}
///
/// {@macro flutter.widgets.SliverChildDelegate.lifecycle}
///
/// See also:
///
///  * [SliverPrototypeExtentList], which is similar to [SliverFixedExtentList]
///    except that it uses a prototype list item instead of a pixel value to define
///    the main axis extent of each item.
///  * [SliverVariedExtentList], which supports children with varying (but known
///    upfront) extents.
///  * [SliverFillViewport], which determines the [itemExtent] based on
///    [SliverConstraints.viewportMainAxisExtent].
///  * [SliverList], which does not require its children to have the same
///    extent in the main axis.
class SliverFixedExtentList extends SliverMultiBoxAdaptorWidget {
  /// Creates a sliver that places box children with the same main axis extent
  /// in a linear array.
  const SliverFixedExtentList({super.key, required super.delegate, required this.itemExtent});

  /// A sliver that places multiple box children in a linear array along the main
  /// axis.
  ///
  /// [SliverFixedExtentList] places its children in a linear array along the main
  /// axis starting at offset zero and without gaps. Each child is forced to have
  /// the [itemExtent] in the main axis and the
  /// [SliverConstraints.crossAxisExtent] in the cross axis.
  ///
  /// This constructor is appropriate for sliver lists with a large (or
  /// infinite) number of children whose extent is already determined.
  ///
  /// Providing a non-null `itemCount` improves the ability of the [SliverFixedExtentList]
  /// to estimate the maximum scroll extent.
  ///
  /// `itemBuilder` will be called only with indices greater than or equal to
  /// zero and less than `itemCount`.
  ///
  /// {@macro flutter.widgets.ListView.builder.itemBuilder}
  ///
  /// The `itemExtent` argument is the extent of each item.
  ///
  /// {@macro flutter.widgets.PageView.findChildIndexCallback}
  ///
  /// The `addAutomaticKeepAlives` argument corresponds to the
  /// [SliverChildBuilderDelegate.addAutomaticKeepAlives] property. The
  /// `addRepaintBoundaries` argument corresponds to the
  /// [SliverChildBuilderDelegate.addRepaintBoundaries] property. The
  /// `addSemanticIndexes` argument corresponds to the
  /// [SliverChildBuilderDelegate.addSemanticIndexes] property.
  /// {@tool snippet}
  ///
  /// This example, which would be inserted into a [CustomScrollView.slivers]
  /// list, shows an infinite number of items in varying shades of blue:
  ///
  /// ```dart
  /// SliverFixedExtentList.builder(
  ///   itemExtent: 50.0,
  ///   itemBuilder: (BuildContext context, int index) {
  ///     return Container(
  ///       alignment: Alignment.center,
  ///       color: Colors.lightBlue[100 * (index % 9)],
  ///       child: Text('list item $index'),
  ///     );
  ///   },
  /// )
  /// ```
  /// {@end-tool}
  SliverFixedExtentList.builder({
    super.key,
    required NullableIndexedWidgetBuilder itemBuilder,
    required this.itemExtent,
    ChildIndexGetter? findChildIndexCallback,
    int? itemCount,
    bool addAutomaticKeepAlives = true,
    bool addRepaintBoundaries = true,
    bool addSemanticIndexes = true,
  }) : super(
         delegate: SliverChildBuilderDelegate(
           itemBuilder,
           findChildIndexCallback: findChildIndexCallback,
           childCount: itemCount,
           addAutomaticKeepAlives: addAutomaticKeepAlives,
           addRepaintBoundaries: addRepaintBoundaries,
           addSemanticIndexes: addSemanticIndexes,
         ),
       );

  /// A sliver that places multiple box children in a linear array along the main
  /// axis.
  ///
  /// [SliverFixedExtentList] places its children in a linear array along the main
  /// axis starting at offset zero and without gaps. Each child is forced to have
  /// the [itemExtent] in the main axis and the
  /// [SliverConstraints.crossAxisExtent] in the cross axis.
  ///
  /// This constructor uses a list of [Widget]s to build the sliver.
  ///
  /// The `addAutomaticKeepAlives` argument corresponds to the
  /// [SliverChildBuilderDelegate.addAutomaticKeepAlives] property. The
  /// `addRepaintBoundaries` argument corresponds to the
  /// [SliverChildBuilderDelegate.addRepaintBoundaries] property. The
  /// `addSemanticIndexes` argument corresponds to the
  /// [SliverChildBuilderDelegate.addSemanticIndexes] property.
  ///
  /// {@tool snippet}
  /// This example, which would be inserted into a [CustomScrollView.slivers]
  /// list, shows an infinite number of items in varying shades of blue:
  ///
  /// ```dart
  /// SliverFixedExtentList.list(
  ///   itemExtent: 50.0,
  ///   children: const <Widget>[
  ///     Text('Hello'),
  ///     Text('World!'),
  ///   ],
  /// );
  /// ```
  /// {@end-tool}
  SliverFixedExtentList.list({
    super.key,
    required List<Widget> children,
    required this.itemExtent,
    bool addAutomaticKeepAlives = true,
    bool addRepaintBoundaries = true,
    bool addSemanticIndexes = true,
  }) : super(
         delegate: SliverChildListDelegate(
           children,
           addAutomaticKeepAlives: addAutomaticKeepAlives,
           addRepaintBoundaries: addRepaintBoundaries,
           addSemanticIndexes: addSemanticIndexes,
         ),
       );

  /// The extent the children are forced to have in the main axis.
  final double itemExtent;

  @override
  RenderSliverFixedExtentList createRenderObject(BuildContext context) {
    final SliverMultiBoxAdaptorElement element = context as SliverMultiBoxAdaptorElement;
    return RenderSliverFixedExtentList(childManager: element, itemExtent: itemExtent);
  }

  @override
  void updateRenderObject(BuildContext context, RenderSliverFixedExtentList renderObject) {
    renderObject.itemExtent = itemExtent;
  }
}

/// A sliver that places its box children in a linear array and constrains them
/// to have the corresponding extent returned by [itemExtentBuilder].
///
/// _To learn more about slivers, see [CustomScrollView.slivers]._
///
/// [SliverVariedExtentList] arranges its children in a line along
/// the main axis starting at offset zero and without gaps. Each child is
/// constrained to the corresponding extent along the main axis
/// and the [SliverConstraints.crossAxisExtent] along the cross axis.
///
/// [SliverVariedExtentList] is more efficient than [SliverList] because
/// [SliverVariedExtentList] does not need to lay out its children to obtain
/// their extent along the main axis. It's a little more flexible than
/// [SliverFixedExtentList] because this allow the children to have different extents.
///
/// See also:
///
///  * [SliverFixedExtentList], whose children are forced to a given pixel
///    extent.
///  * [SliverPrototypeExtentList], which is similar to [SliverFixedExtentList]
///    except that it uses a prototype list item instead of a pixel value to define
///    the main axis extent of each item.
///  * [SliverList], which does not require its children to have the same
///    extent in the main axis.
///  * [SliverFillViewport], which sizes its children based on the
///    size of the viewport, regardless of what else is in the scroll view.
class SliverVariedExtentList extends SliverMultiBoxAdaptorWidget {
  /// Creates a sliver that places box children with the same main axis extent
  /// in a linear array.
  const SliverVariedExtentList({
    super.key,
    required super.delegate,
    required this.itemExtentBuilder,
  });

  /// A sliver that places multiple box children in a linear array along the main
  /// axis.
  ///
  /// [SliverVariedExtentList] places its children in a linear array along the main
  /// axis starting at offset zero and without gaps. Each child is forced to have
  /// the returned extent of [itemExtentBuilder] in the main axis and the
  /// [SliverConstraints.crossAxisExtent] in the cross axis.
  ///
  /// This constructor is appropriate for sliver lists with a large (or
  /// infinite) number of children whose extent is already determined.
  ///
  /// Providing a non-null `itemCount` improves the ability of the [SliverVariedExtentList]
  /// to estimate the maximum scroll extent.
  SliverVariedExtentList.builder({
    super.key,
    required NullableIndexedWidgetBuilder itemBuilder,
    required this.itemExtentBuilder,
    ChildIndexGetter? findChildIndexCallback,
    int? itemCount,
    bool addAutomaticKeepAlives = true,
    bool addRepaintBoundaries = true,
    bool addSemanticIndexes = true,
  }) : super(
         delegate: SliverChildBuilderDelegate(
           itemBuilder,
           findChildIndexCallback: findChildIndexCallback,
           childCount: itemCount,
           addAutomaticKeepAlives: addAutomaticKeepAlives,
           addRepaintBoundaries: addRepaintBoundaries,
           addSemanticIndexes: addSemanticIndexes,
         ),
       );

  /// A sliver that places multiple box children in a linear array along the main
  /// axis.
  ///
  /// [SliverVariedExtentList] places its children in a linear array along the main
  /// axis starting at offset zero and without gaps. Each child is forced to have
  /// the returned extent of [itemExtentBuilder] in the main axis and the
  /// [SliverConstraints.crossAxisExtent] in the cross axis.
  ///
  /// This constructor uses a list of [Widget]s to build the sliver.
  SliverVariedExtentList.list({
    super.key,
    required List<Widget> children,
    required this.itemExtentBuilder,
    bool addAutomaticKeepAlives = true,
    bool addRepaintBoundaries = true,
    bool addSemanticIndexes = true,
  }) : super(
         delegate: SliverChildListDelegate(
           children,
           addAutomaticKeepAlives: addAutomaticKeepAlives,
           addRepaintBoundaries: addRepaintBoundaries,
           addSemanticIndexes: addSemanticIndexes,
         ),
       );

  /// The children extent builder.
  ///
  /// Should return null if asked to build an item extent with a greater index than
  /// exists.
  final ItemExtentBuilder itemExtentBuilder;

  @override
  RenderSliverVariedExtentList createRenderObject(BuildContext context) {
    final SliverMultiBoxAdaptorElement element = context as SliverMultiBoxAdaptorElement;
    return RenderSliverVariedExtentList(
      childManager: element,
      itemExtentBuilder: itemExtentBuilder,
    );
  }

  @override
  void updateRenderObject(BuildContext context, RenderSliverVariedExtentList renderObject) {
    renderObject.itemExtentBuilder = itemExtentBuilder;
  }
}

/// A sliver that places multiple box children in a two dimensional arrangement.
///
/// _To learn more about slivers, see [CustomScrollView.slivers]._
///
/// [SliverGrid] places its children in arbitrary positions determined by
/// [gridDelegate]. Each child is forced to have the size specified by the
/// [gridDelegate].
///
/// The main axis direction of a grid is the direction in which it scrolls; the
/// cross axis direction is the orthogonal direction.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=ORiTTaVY6mM}
///
/// {@tool snippet}
///
/// This example, which would be inserted into a [CustomScrollView.slivers]
/// list, shows twenty boxes in a pretty teal grid:
///
/// ```dart
/// SliverGrid(
///   gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
///     maxCrossAxisExtent: 200.0,
///     mainAxisSpacing: 10.0,
///     crossAxisSpacing: 10.0,
///     childAspectRatio: 4.0,
///   ),
///   delegate: SliverChildBuilderDelegate(
///     (BuildContext context, int index) {
///       return Container(
///         alignment: Alignment.center,
///         color: Colors.teal[100 * (index % 9)],
///         child: Text('grid item $index'),
///       );
///     },
///     childCount: 20,
///   ),
/// )
/// ```
/// {@end-tool}
///
/// {@macro flutter.widgets.SliverChildDelegate.lifecycle}
///
/// See also:
///
///  * [SliverList], which places its children in a linear array.
///  * [SliverFixedExtentList], which places its children in a linear
///    array with a fixed extent in the main axis.
///  * [SliverPrototypeExtentList], which is similar to [SliverFixedExtentList]
///    except that it uses a prototype list item instead of a pixel value to define
///    the main axis extent of each item.
class SliverGrid extends SliverMultiBoxAdaptorWidget {
  /// Creates a sliver that places multiple box children in a two dimensional
  /// arrangement.
  const SliverGrid({super.key, required super.delegate, required this.gridDelegate});

  /// A sliver that creates a 2D array of widgets that are created on demand.
  ///
  /// This constructor is appropriate for sliver grids with a large (or
  /// infinite) number of children because the builder is called only for those
  /// children that are actually visible.
  ///
  /// Providing a non-null `itemCount` improves the ability of the [SliverGrid]
  /// to estimate the maximum scroll extent.
  ///
  /// `itemBuilder` will be called only with indices greater than or equal to
  /// zero and less than `itemCount`.
  ///
  /// {@macro flutter.widgets.ListView.builder.itemBuilder}
  ///
  /// {@macro flutter.widgets.PageView.findChildIndexCallback}
  ///
  /// The [gridDelegate] argument is required.
  ///
  /// The `addAutomaticKeepAlives` argument corresponds to the
  /// [SliverChildBuilderDelegate.addAutomaticKeepAlives] property. The
  /// `addRepaintBoundaries` argument corresponds to the
  /// [SliverChildBuilderDelegate.addRepaintBoundaries] property. The
  /// `addSemanticIndexes` argument corresponds to the
  /// [SliverChildBuilderDelegate.addSemanticIndexes] property.
  SliverGrid.builder({
    super.key,
    required this.gridDelegate,
    required NullableIndexedWidgetBuilder itemBuilder,
    ChildIndexGetter? findChildIndexCallback,
    int? itemCount,
    bool addAutomaticKeepAlives = true,
    bool addRepaintBoundaries = true,
    bool addSemanticIndexes = true,
  }) : super(
         delegate: SliverChildBuilderDelegate(
           itemBuilder,
           findChildIndexCallback: findChildIndexCallback,
           childCount: itemCount,
           addAutomaticKeepAlives: addAutomaticKeepAlives,
           addRepaintBoundaries: addRepaintBoundaries,
           addSemanticIndexes: addSemanticIndexes,
         ),
       );

  /// Creates a sliver that places multiple box children in a two dimensional
  /// arrangement with a fixed number of tiles in the cross axis.
  ///
  /// Uses a [SliverGridDelegateWithFixedCrossAxisCount] as the [gridDelegate],
  /// and a [SliverChildListDelegate] as the [delegate].
  ///
  /// See also:
  ///
  ///  * [GridView.count], the equivalent constructor for [GridView] widgets.
  SliverGrid.count({
    super.key,
    required int crossAxisCount,
    double mainAxisSpacing = 0.0,
    double crossAxisSpacing = 0.0,
    double childAspectRatio = 1.0,
    List<Widget> children = const <Widget>[],
  }) : gridDelegate = SliverGridDelegateWithFixedCrossAxisCount(
         crossAxisCount: crossAxisCount,
         mainAxisSpacing: mainAxisSpacing,
         crossAxisSpacing: crossAxisSpacing,
         childAspectRatio: childAspectRatio,
       ),
       super(delegate: SliverChildListDelegate(children));

  /// Creates a sliver that places multiple box children in a two dimensional
  /// arrangement with tiles that each have a maximum cross-axis extent.
  ///
  /// Uses a [SliverGridDelegateWithMaxCrossAxisExtent] as the [gridDelegate],
  /// and a [SliverChildListDelegate] as the [delegate].
  ///
  /// See also:
  ///
  ///  * [GridView.extent], the equivalent constructor for [GridView] widgets.
  SliverGrid.extent({
    super.key,
    required double maxCrossAxisExtent,
    double mainAxisSpacing = 0.0,
    double crossAxisSpacing = 0.0,
    double childAspectRatio = 1.0,
    List<Widget> children = const <Widget>[],
  }) : gridDelegate = SliverGridDelegateWithMaxCrossAxisExtent(
         maxCrossAxisExtent: maxCrossAxisExtent,
         mainAxisSpacing: mainAxisSpacing,
         crossAxisSpacing: crossAxisSpacing,
         childAspectRatio: childAspectRatio,
       ),
       super(delegate: SliverChildListDelegate(children));

  /// The delegate that controls the size and position of the children.
  final SliverGridDelegate gridDelegate;

  @override
  RenderSliverGrid createRenderObject(BuildContext context) {
    final SliverMultiBoxAdaptorElement element = context as SliverMultiBoxAdaptorElement;
    return RenderSliverGrid(childManager: element, gridDelegate: gridDelegate);
  }

  @override
  void updateRenderObject(BuildContext context, RenderSliverGrid renderObject) {
    renderObject.gridDelegate = gridDelegate;
  }

  @override
  double estimateMaxScrollOffset(
    SliverConstraints? constraints,
    int firstIndex,
    int lastIndex,
    double leadingScrollOffset,
    double trailingScrollOffset,
  ) {
    return super.estimateMaxScrollOffset(
          constraints,
          firstIndex,
          lastIndex,
          leadingScrollOffset,
          trailingScrollOffset,
        ) ??
        gridDelegate.getLayout(constraints!).computeMaxScrollOffset(delegate.estimatedChildCount!);
  }
}

/// An element that lazily builds children for a [SliverMultiBoxAdaptorWidget].
///
/// Implements [RenderSliverBoxChildManager], which lets this element manage
/// the children of subclasses of [RenderSliverMultiBoxAdaptor].
class SliverMultiBoxAdaptorElement extends RenderObjectElement
    implements RenderSliverBoxChildManager {
  /// Creates an element that lazily builds children for the given widget.
  ///
  /// If `replaceMovedChildren` is set to true, a new child is proactively
  /// inflate for the index that was previously occupied by a child that moved
  /// to a new index. The layout offset of the moved child is copied over to the
  /// new child. RenderObjects, that depend on the layout offset of existing
  /// children during [RenderObject.performLayout] should set this to true
  /// (example: [RenderSliverList]). For RenderObjects that figure out the
  /// layout offset of their children without looking at the layout offset of
  /// existing children this should be set to false (example:
  /// [RenderSliverFixedExtentList]) to avoid inflating unnecessary children.
  SliverMultiBoxAdaptorElement(
    SliverMultiBoxAdaptorWidget super.widget, {
    bool replaceMovedChildren = false,
  }) : _replaceMovedChildren = replaceMovedChildren;

  final bool _replaceMovedChildren;

  @override
  RenderSliverMultiBoxAdaptor get renderObject => super.renderObject as RenderSliverMultiBoxAdaptor;

  @override
  void update(covariant SliverMultiBoxAdaptorWidget newWidget) {
    final SliverMultiBoxAdaptorWidget oldWidget = widget as SliverMultiBoxAdaptorWidget;
    super.update(newWidget);
    final SliverChildDelegate newDelegate = newWidget.delegate;
    final SliverChildDelegate oldDelegate = oldWidget.delegate;
    if (newDelegate != oldDelegate &&
        (newDelegate.runtimeType != oldDelegate.runtimeType ||
            newDelegate.shouldRebuild(oldDelegate))) {
      performRebuild();
    }
  }

  final SplayTreeMap<int, Element?> _childElements = SplayTreeMap<int, Element?>();
  RenderBox? _currentBeforeChild;

  @override
  void performRebuild() {
    super.performRebuild();
    _currentBeforeChild = null;
    bool childrenUpdated = false;
    assert(_currentlyUpdatingChildIndex == null);
    try {
      final SplayTreeMap<int, Element?> newChildren = SplayTreeMap<int, Element?>();
      final Map<int, double> indexToLayoutOffset = HashMap<int, double>();
      final SliverMultiBoxAdaptorWidget adaptorWidget = widget as SliverMultiBoxAdaptorWidget;
      void processElement(int index) {
        _currentlyUpdatingChildIndex = index;
        if (_childElements[index] != null && _childElements[index] != newChildren[index]) {
          // This index has an old child that isn't used anywhere and should be deactivated.
          _childElements[index] = updateChild(_childElements[index], null, index);
          childrenUpdated = true;
        }
        final Element? newChild = updateChild(
          newChildren[index],
          _build(index, adaptorWidget),
          index,
        );
        if (newChild != null) {
          childrenUpdated = childrenUpdated || _childElements[index] != newChild;
          _childElements[index] = newChild;
          final SliverMultiBoxAdaptorParentData parentData =
              newChild.renderObject!.parentData! as SliverMultiBoxAdaptorParentData;
          if (index == 0) {
            parentData.layoutOffset = 0.0;
          } else if (indexToLayoutOffset.containsKey(index)) {
            parentData.layoutOffset = indexToLayoutOffset[index];
          }
          if (!parentData.keptAlive) {
            _currentBeforeChild = newChild.renderObject as RenderBox?;
          }
        } else {
          childrenUpdated = true;
          _childElements.remove(index);
        }
      }

      for (final int index in _childElements.keys.toList()) {
        final Key? key = _childElements[index]!.widget.key;
        final int? newIndex = key == null ? null : adaptorWidget.delegate.findIndexByKey(key);
        final SliverMultiBoxAdaptorParentData? childParentData =
            _childElements[index]!.renderObject?.parentData as SliverMultiBoxAdaptorParentData?;

        if (childParentData != null && childParentData.layoutOffset != null) {
          indexToLayoutOffset[index] = childParentData.layoutOffset!;
        }

        if (newIndex != null && newIndex != index) {
          // The layout offset of the child being moved is no longer accurate.
          if (childParentData != null) {
            childParentData.layoutOffset = null;
          }

          newChildren[newIndex] = _childElements[index];
          if (_replaceMovedChildren) {
            // We need to make sure the original index gets processed.
            newChildren.putIfAbsent(index, () => null);
          }
          // We do not want the remapped child to get deactivated during processElement.
          _childElements.remove(index);
        } else {
          newChildren.putIfAbsent(index, () => _childElements[index]);
        }
      }

      renderObject.debugChildIntegrityEnabled =
          false; // Moving children will temporary violate the integrity.
      newChildren.keys.forEach(processElement);
      // An element rebuild only updates existing children. The underflow check
      // is here to make sure we look ahead one more child if we were at the end
      // of the child list before the update. By doing so, we can update the max
      // scroll offset during the layout phase. Otherwise, the layout phase may
      // be skipped, and the scroll view may be stuck at the previous max
      // scroll offset.
      //
      // This logic is not needed if any existing children has been updated,
      // because we will not skip the layout phase if that happens.
      if (!childrenUpdated && _didUnderflow) {
        final int lastKey = _childElements.lastKey() ?? -1;
        final int rightBoundary = lastKey + 1;
        newChildren[rightBoundary] = _childElements[rightBoundary];
        processElement(rightBoundary);
      }
    } finally {
      _currentlyUpdatingChildIndex = null;
      renderObject.debugChildIntegrityEnabled = true;
    }
  }

  Widget? _build(int index, SliverMultiBoxAdaptorWidget widget) {
    return widget.delegate.build(this, index);
  }

  @override
  void createChild(int index, {required RenderBox? after}) {
    assert(_currentlyUpdatingChildIndex == null);
    owner!.buildScope(this, () {
      final bool insertFirst = after == null;
      assert(insertFirst || _childElements[index - 1] != null);
      _currentBeforeChild =
          insertFirst ? null : (_childElements[index - 1]!.renderObject as RenderBox?);
      Element? newChild;
      try {
        final SliverMultiBoxAdaptorWidget adaptorWidget = widget as SliverMultiBoxAdaptorWidget;
        _currentlyUpdatingChildIndex = index;
        newChild = updateChild(_childElements[index], _build(index, adaptorWidget), index);
      } finally {
        _currentlyUpdatingChildIndex = null;
      }
      if (newChild != null) {
        _childElements[index] = newChild;
      } else {
        _childElements.remove(index);
      }
    });
  }

  @override
  Element? updateChild(Element? child, Widget? newWidget, Object? newSlot) {
    final SliverMultiBoxAdaptorParentData? oldParentData =
        child?.renderObject?.parentData as SliverMultiBoxAdaptorParentData?;
    final Element? newChild = super.updateChild(child, newWidget, newSlot);
    final SliverMultiBoxAdaptorParentData? newParentData =
        newChild?.renderObject?.parentData as SliverMultiBoxAdaptorParentData?;

    // Preserve the old layoutOffset if the renderObject was swapped out.
    if (oldParentData != newParentData && oldParentData != null && newParentData != null) {
      newParentData.layoutOffset = oldParentData.layoutOffset;
    }
    return newChild;
  }

  @override
  void forgetChild(Element child) {
    assert(child.slot != null);
    assert(_childElements.containsKey(child.slot));
    _childElements.remove(child.slot);
    super.forgetChild(child);
  }

  @override
  void removeChild(RenderBox child) {
    final int index = renderObject.indexOf(child);
    assert(_currentlyUpdatingChildIndex == null);
    assert(index >= 0);
    owner!.buildScope(this, () {
      assert(_childElements.containsKey(index));
      try {
        _currentlyUpdatingChildIndex = index;
        final Element? result = updateChild(_childElements[index], null, index);
        assert(result == null);
      } finally {
        _currentlyUpdatingChildIndex = null;
      }
      _childElements.remove(index);
      assert(!_childElements.containsKey(index));
    });
  }

  static double _extrapolateMaxScrollOffset(
    int firstIndex,
    int lastIndex,
    double leadingScrollOffset,
    double trailingScrollOffset,
    int childCount,
  ) {
    if (lastIndex == childCount - 1) {
      return trailingScrollOffset;
    }
    final int reifiedCount = lastIndex - firstIndex + 1;
    final double averageExtent = (trailingScrollOffset - leadingScrollOffset) / reifiedCount;
    final int remainingCount = childCount - lastIndex - 1;
    return trailingScrollOffset + averageExtent * remainingCount;
  }

  @override
  double estimateMaxScrollOffset(
    SliverConstraints? constraints, {
    int? firstIndex,
    int? lastIndex,
    double? leadingScrollOffset,
    double? trailingScrollOffset,
  }) {
    final int? childCount = estimatedChildCount;
    if (childCount == null) {
      return double.infinity;
    }
    return (widget as SliverMultiBoxAdaptorWidget).estimateMaxScrollOffset(
          constraints,
          firstIndex!,
          lastIndex!,
          leadingScrollOffset!,
          trailingScrollOffset!,
        ) ??
        _extrapolateMaxScrollOffset(
          firstIndex,
          lastIndex,
          leadingScrollOffset,
          trailingScrollOffset,
          childCount,
        );
  }

  @override
  int? get estimatedChildCount =>
      (widget as SliverMultiBoxAdaptorWidget).delegate.estimatedChildCount;

  @override
  int get childCount {
    int? result = estimatedChildCount;
    if (result == null) {
      // Since childCount was called, we know that we reached the end of
      // the list (as in, _build return null once), so we know that the
      // list is finite.
      // Let's do an open-ended binary search to find the end of the list
      // manually.
      int lo = 0;
      int hi = 1;
      final SliverMultiBoxAdaptorWidget adaptorWidget = widget as SliverMultiBoxAdaptorWidget;
      const int max =
          kIsWeb
              ? 9007199254740992 // max safe integer on JS (from 0 to this number x != x+1)
              : ((1 << 63) - 1);
      while (_build(hi - 1, adaptorWidget) != null) {
        lo = hi - 1;
        if (hi < max ~/ 2) {
          hi *= 2;
        } else if (hi < max) {
          hi = max;
        } else {
          throw FlutterError(
            'Could not find the number of children in ${adaptorWidget.delegate}.\n'
            "The childCount getter was called (implying that the delegate's builder returned null "
            'for a positive index), but even building the child with index $hi (the maximum '
            'possible integer) did not return null. Consider implementing childCount to avoid '
            'the cost of searching for the final child.',
          );
        }
      }
      while (hi - lo > 1) {
        final int mid = (hi - lo) ~/ 2 + lo;
        if (_build(mid - 1, adaptorWidget) == null) {
          hi = mid;
        } else {
          lo = mid;
        }
      }
      result = lo;
    }
    return result;
  }

  @override
  void didStartLayout() {
    assert(debugAssertChildListLocked());
  }

  @override
  void didFinishLayout() {
    assert(debugAssertChildListLocked());
    final int firstIndex = _childElements.firstKey() ?? 0;
    final int lastIndex = _childElements.lastKey() ?? 0;
    (widget as SliverMultiBoxAdaptorWidget).delegate.didFinishLayout(firstIndex, lastIndex);
  }

  int? _currentlyUpdatingChildIndex;

  @override
  bool debugAssertChildListLocked() {
    assert(_currentlyUpdatingChildIndex == null);
    return true;
  }

  @override
  void didAdoptChild(RenderBox child) {
    assert(_currentlyUpdatingChildIndex != null);
    final SliverMultiBoxAdaptorParentData childParentData =
        child.parentData! as SliverMultiBoxAdaptorParentData;
    childParentData.index = _currentlyUpdatingChildIndex;
  }

  bool _didUnderflow = false;

  @override
  void setDidUnderflow(bool value) {
    _didUnderflow = value;
  }

  @override
  void insertRenderObjectChild(covariant RenderObject child, int slot) {
    assert(_currentlyUpdatingChildIndex == slot);
    assert(renderObject.debugValidateChild(child));
    renderObject.insert(child as RenderBox, after: _currentBeforeChild);
    assert(() {
      final SliverMultiBoxAdaptorParentData childParentData =
          child.parentData! as SliverMultiBoxAdaptorParentData;
      assert(slot == childParentData.index);
      return true;
    }());
  }

  @override
  void moveRenderObjectChild(covariant RenderObject child, int oldSlot, int newSlot) {
    assert(_currentlyUpdatingChildIndex == newSlot);
    renderObject.move(child as RenderBox, after: _currentBeforeChild);
  }

  @override
  void removeRenderObjectChild(covariant RenderObject child, int slot) {
    assert(_currentlyUpdatingChildIndex != null);
    renderObject.remove(child as RenderBox);
  }

  @override
  void visitChildren(ElementVisitor visitor) {
    // The toList() is to make a copy so that the underlying list can be modified by
    // the visitor:
    assert(!_childElements.values.any((Element? child) => child == null));
    _childElements.values.cast<Element>().toList().forEach(visitor);
  }

  @override
  void debugVisitOnstageChildren(ElementVisitor visitor) {
    _childElements.values
        .cast<Element>()
        .where((Element child) {
          final SliverMultiBoxAdaptorParentData parentData =
              child.renderObject!.parentData! as SliverMultiBoxAdaptorParentData;
          final double itemExtent = switch (renderObject.constraints.axis) {
            Axis.horizontal => child.renderObject!.paintBounds.width,
            Axis.vertical => child.renderObject!.paintBounds.height,
          };

          return parentData.layoutOffset != null &&
              parentData.layoutOffset! <
                  renderObject.constraints.scrollOffset +
                      renderObject.constraints.remainingPaintExtent &&
              parentData.layoutOffset! + itemExtent > renderObject.constraints.scrollOffset;
        })
        .forEach(visitor);
  }
}

/// A sliver widget that makes its sliver child partially transparent.
///
/// This class paints its sliver child into an intermediate buffer and then
/// blends the sliver back into the scene partially transparent.
///
/// For values of opacity other than 0.0 and 1.0, this class is relatively
/// expensive because it requires painting the sliver child into an intermediate
/// buffer. For the value 0.0, the sliver child is not painted at all.
/// For the value 1.0, the sliver child is painted immediately without an
/// intermediate buffer.
///
/// {@tool dartpad}
///
/// This example shows a [SliverList] when the `_visible` member field is true,
/// and hides it when it is false.
///
/// This is more efficient than adding and removing the sliver child widget from
/// the tree on demand, but it does not affect how much the list scrolls (the
/// [SliverList] is still present, merely invisible).
///
/// ** See code in examples/api/lib/widgets/sliver/sliver_opacity.1.dart **
/// {@end-tool}
///
/// See also:
///
///  * [Opacity], which can apply a uniform alpha effect to its child using the
///    [RenderBox] layout protocol.
///  * [AnimatedOpacity], which uses an animation internally to efficiently
///    animate [Opacity].
///  * [SliverVisibility], which can hide a child more efficiently (albeit less
///    subtly, because it is either visible or hidden, rather than allowing
///    fractional opacity values). Specifically, the [SliverVisibility.maintain]
///    constructor is equivalent to using a sliver opacity widget with values of
///    `0.0` or `1.0`.
class SliverOpacity extends SingleChildRenderObjectWidget {
  /// Creates a sliver that makes its sliver child partially transparent.
  ///
  /// The [opacity] argument must be between zero and one, inclusive.
  const SliverOpacity({
    super.key,
    required this.opacity,
    this.alwaysIncludeSemantics = false,
    Widget? sliver,
  }) : assert(opacity >= 0.0 && opacity <= 1.0),
       super(child: sliver);

  /// The fraction to scale the sliver child's alpha value.
  ///
  /// An opacity of 1.0 is fully opaque. An opacity of 0.0 is fully transparent
  /// (i.e. invisible).
  ///
  /// Values 1.0 and 0.0 are painted with a fast path. Other values
  /// require painting the sliver child into an intermediate buffer, which is
  /// expensive.
  final double opacity;

  /// Whether the semantic information of the sliver child is always included.
  ///
  /// Defaults to false.
  ///
  /// When true, regardless of the opacity settings, the sliver child semantic
  /// information is exposed as if the widget were fully visible. This is
  /// useful in cases where labels may be hidden during animations that
  /// would otherwise contribute relevant semantics.
  final bool alwaysIncludeSemantics;

  @override
  RenderSliverOpacity createRenderObject(BuildContext context) {
    return RenderSliverOpacity(opacity: opacity, alwaysIncludeSemantics: alwaysIncludeSemantics);
  }

  @override
  void updateRenderObject(BuildContext context, RenderSliverOpacity renderObject) {
    renderObject
      ..opacity = opacity
      ..alwaysIncludeSemantics = alwaysIncludeSemantics;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<double>('opacity', opacity));
    properties.add(
      FlagProperty(
        'alwaysIncludeSemantics',
        value: alwaysIncludeSemantics,
        ifTrue: 'alwaysIncludeSemantics',
      ),
    );
  }
}

/// A sliver widget that is invisible during hit testing.
///
/// When [ignoring] is true, this widget (and its subtree) is invisible
/// to hit testing. It still consumes space during layout and paints its sliver
/// child as usual. It just cannot be the target of located events, because it
/// returns false from [RenderSliver.hitTest].
///
/// ## Semantics
///
/// Using this class may also affect how the semantics subtree underneath is
/// collected.
///
/// {@macro flutter.widgets.IgnorePointer.semantics}
///
/// {@macro flutter.widgets.IgnorePointer.ignoringSemantics}
///
/// See also:
///
///  * [IgnorePointer], the equivalent widget for boxes.
class SliverIgnorePointer extends SingleChildRenderObjectWidget {
  /// Creates a sliver widget that is invisible to hit testing.
  const SliverIgnorePointer({
    super.key,
    this.ignoring = true,
    @Deprecated(
      'Create a custom sliver ignore pointer widget instead. '
      'This feature was deprecated after v3.8.0-12.0.pre.',
    )
    this.ignoringSemantics,
    Widget? sliver,
  }) : super(child: sliver);

  /// Whether this sliver is ignored during hit testing.
  ///
  /// Regardless of whether this sliver is ignored during hit testing, it will
  /// still consume space during layout and be visible during painting.
  ///
  /// {@macro flutter.widgets.IgnorePointer.semantics}
  final bool ignoring;

  /// Whether the semantics of this sliver is ignored when compiling the
  /// semantics tree.
  ///
  /// {@macro flutter.widgets.IgnorePointer.ignoringSemantics}
  @Deprecated(
    'Create a custom sliver ignore pointer widget instead. '
    'This feature was deprecated after v3.8.0-12.0.pre.',
  )
  final bool? ignoringSemantics;

  @override
  RenderSliverIgnorePointer createRenderObject(BuildContext context) {
    return RenderSliverIgnorePointer(ignoring: ignoring, ignoringSemantics: ignoringSemantics);
  }

  @override
  void updateRenderObject(BuildContext context, RenderSliverIgnorePointer renderObject) {
    renderObject
      ..ignoring = ignoring
      ..ignoringSemantics = ignoringSemantics;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<bool>('ignoring', ignoring));
    properties.add(
      DiagnosticsProperty<bool>('ignoringSemantics', ignoringSemantics, defaultValue: null),
    );
  }
}

/// A sliver that lays its sliver child out as if it was in the tree, but
/// without painting anything, without making the sliver child available for hit
/// testing, and without taking any room in the parent.
///
/// Animations continue to run in offstage sliver children, and therefore use
/// battery and CPU time, regardless of whether the animations end up being
/// visible.
///
/// To hide a sliver widget from view while it is
/// not needed, prefer removing the widget from the tree entirely rather than
/// keeping it alive in an [Offstage] subtree.
///
/// See also:
///
///  * [Offstage], the equivalent widget for boxes.
class SliverOffstage extends SingleChildRenderObjectWidget {
  /// Creates a sliver that visually hides its sliver child.
  const SliverOffstage({super.key, this.offstage = true, Widget? sliver}) : super(child: sliver);

  /// Whether the sliver child is hidden from the rest of the tree.
  ///
  /// If true, the sliver child is laid out as if it was in the tree, but
  /// without painting anything, without making the child available for hit
  /// testing, and without taking any room in the parent.
  ///
  /// If false, the sliver child is included in the tree as normal.
  final bool offstage;

  @override
  RenderSliverOffstage createRenderObject(BuildContext context) =>
      RenderSliverOffstage(offstage: offstage);

  @override
  void updateRenderObject(BuildContext context, RenderSliverOffstage renderObject) {
    renderObject.offstage = offstage;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<bool>('offstage', offstage));
  }

  @override
  SingleChildRenderObjectElement createElement() => _SliverOffstageElement(this);
}

class _SliverOffstageElement extends SingleChildRenderObjectElement {
  _SliverOffstageElement(SliverOffstage super.widget);

  @override
  void debugVisitOnstageChildren(ElementVisitor visitor) {
    if (!(widget as SliverOffstage).offstage) {
      super.debugVisitOnstageChildren(visitor);
    }
  }
}

/// Mark a child as needing to stay alive even when it's in a lazy list that
/// would otherwise remove it.
///
/// This widget is for use in a [RenderAbstractViewport]s, such as
/// [Viewport] or [TwoDimensionalViewport].
///
/// This widget is rarely used directly. The [SliverChildBuilderDelegate] and
/// [SliverChildListDelegate] delegates, used with [SliverList] and
/// [SliverGrid], as well as the scroll view counterparts [ListView] and
/// [GridView], have an `addAutomaticKeepAlives` feature, which is enabled by
/// default, and which causes [AutomaticKeepAlive] widgets to be inserted around
/// each child, causing [KeepAlive] widgets to be automatically added and
/// configured in response to [KeepAliveNotification]s.
///
/// The same `addAutomaticKeepAlives` feature is supported by the
/// [TwoDimensionalChildBuilderDelegate] and [TwoDimensionalChildListDelegate].
///
/// Therefore, to keep a widget alive, it is more common to use those
/// notifications than to directly deal with [KeepAlive] widgets.
///
/// In practice, the simplest way to deal with these notifications is to mix
/// [AutomaticKeepAliveClientMixin] into one's [State]. See the documentation
/// for that mixin class for details.
class KeepAlive extends ParentDataWidget<KeepAliveParentDataMixin> {
  /// Marks a child as needing to remain alive.
  const KeepAlive({super.key, required this.keepAlive, required super.child});

  /// Whether to keep the child alive.
  ///
  /// If this is false, it is as if this widget was omitted.
  final bool keepAlive;

  @override
  void applyParentData(RenderObject renderObject) {
    assert(renderObject.parentData is KeepAliveParentDataMixin);
    final KeepAliveParentDataMixin parentData =
        renderObject.parentData! as KeepAliveParentDataMixin;
    if (parentData.keepAlive != keepAlive) {
      // No need to redo layout if it became true.
      parentData.keepAlive = keepAlive;
      if (!keepAlive) {
        renderObject.parent?.markNeedsLayout();
      }
    }
  }

  // We only return true if [keepAlive] is true, because turning _off_ keep
  // alive requires a layout to do the garbage collection (but turning it on
  // requires nothing, since by definition the widget is already alive and won't
  // go away _unless_ we do a layout).
  @override
  bool debugCanApplyOutOfTurn() => keepAlive;

  @override
  Type get debugTypicalAncestorWidgetClass =>
      throw FlutterError(
        'Multiple Types are supported, use debugTypicalAncestorWidgetDescription.',
      );

  @override
  String get debugTypicalAncestorWidgetDescription =>
      'SliverWithKeepAliveWidget or TwoDimensionalViewport';

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<bool>('keepAlive', keepAlive));
  }
}

/// A sliver that constrains the cross axis extent of its sliver child.
///
/// The [SliverConstrainedCrossAxis] takes a [maxExtent] parameter and uses it as
/// the cross axis extent of the [SliverConstraints] passed to the sliver child.
/// The widget ensures that the [maxExtent] is a nonnegative value.
///
/// This is useful when you want to apply a custom cross-axis extent constraint
/// to a sliver child, as slivers typically consume the full cross axis extent.
///
/// This widget also sets its parent data's [SliverPhysicalParentData.crossAxisFlex]
/// to 0, so that it informs [SliverCrossAxisGroup] that it should not flex
/// in the cross axis direction.
///
/// {@tool dartpad}
/// In this sample the [SliverConstrainedCrossAxis] sizes its child so that the
/// cross axis extent takes up less space than the actual viewport.
///
/// ** See code in examples/api/lib/widgets/sliver/sliver_constrained_cross_axis.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SliverCrossAxisGroup], the widget which makes use of 0 flex factor set by
///    this widget.
class SliverConstrainedCrossAxis extends StatelessWidget {
  /// Creates a sliver that constrains the cross axis extent of its sliver child.
  ///
  /// The [maxExtent] parameter is required and must be nonnegative.
  const SliverConstrainedCrossAxis({super.key, required this.maxExtent, required this.sliver});

  /// The cross axis extent to apply to the sliver child.
  ///
  /// This value must be nonnegative.
  final double maxExtent;

  /// The widget below this widget in the tree.
  ///
  /// Must be a sliver.
  final Widget sliver;

  @override
  Widget build(BuildContext context) {
    return _SliverZeroFlexParentDataWidget(
      sliver: _SliverConstrainedCrossAxis(maxExtent: maxExtent, sliver: sliver),
    );
  }
}

class _SliverZeroFlexParentDataWidget extends ParentDataWidget<SliverPhysicalParentData> {
  const _SliverZeroFlexParentDataWidget({required Widget sliver}) : super(child: sliver);

  @override
  void applyParentData(RenderObject renderObject) {
    assert(renderObject.parentData is SliverPhysicalParentData);
    final SliverPhysicalParentData parentData =
        renderObject.parentData! as SliverPhysicalParentData;
    bool needsLayout = false;
    if (parentData.crossAxisFlex != 0) {
      parentData.crossAxisFlex = 0;
      needsLayout = true;
    }

    if (needsLayout) {
      renderObject.parent?.markNeedsLayout();
    }
  }

  @override
  Type get debugTypicalAncestorWidgetClass => SliverCrossAxisGroup;
}

class _SliverConstrainedCrossAxis extends SingleChildRenderObjectWidget {
  const _SliverConstrainedCrossAxis({required this.maxExtent, required Widget sliver})
    : assert(maxExtent >= 0.0),
      super(child: sliver);

  /// The cross axis extent to apply to the sliver child.
  ///
  /// This value must be nonnegative.
  final double maxExtent;

  @override
  RenderSliverConstrainedCrossAxis createRenderObject(BuildContext context) {
    return RenderSliverConstrainedCrossAxis(maxExtent: maxExtent);
  }

  @override
  void updateRenderObject(BuildContext context, RenderSliverConstrainedCrossAxis renderObject) {
    renderObject.maxExtent = maxExtent;
  }
}

/// Set a flex factor for allocating space in the cross axis direction.
///
/// This is a [ParentDataWidget] to be used in [SliverCrossAxisGroup].
/// After all slivers with null or zero flex (e.g. [SliverConstrainedCrossAxis])
/// are laid out (which should determine their own [SliverGeometry.crossAxisExtent]),
/// the remaining space is laid out among the slivers with nonzero flex
/// proportionally to their flex value.
class SliverCrossAxisExpanded extends ParentDataWidget<SliverPhysicalContainerParentData> {
  /// Creates an object that assigns a [flex] value to the child sliver.
  ///
  /// The provided [flex] value must be greater than 0.
  const SliverCrossAxisExpanded({super.key, required this.flex, required Widget sliver})
    : assert(flex > 0 && flex < double.infinity),
      super(child: sliver);

  /// Flex value for allocating cross axis extent left after laying out the children with
  /// constrained cross axis. The children with flex values will have the remaining extent
  /// allocated proportionally to their flex value. This must an integer between
  /// 0 and infinity, exclusive.
  final int flex;

  @override
  void applyParentData(RenderObject renderObject) {
    assert(renderObject.parentData is SliverPhysicalContainerParentData);
    assert(renderObject.parent is RenderSliverCrossAxisGroup);
    final SliverPhysicalParentData parentData =
        renderObject.parentData! as SliverPhysicalParentData;
    bool needsLayout = false;

    if (parentData.crossAxisFlex != flex) {
      parentData.crossAxisFlex = flex;
      needsLayout = true;
    }

    if (needsLayout) {
      renderObject.parent?.markNeedsLayout();
    }
  }

  @override
  Type get debugTypicalAncestorWidgetClass => SliverCrossAxisGroup;
}

/// A sliver that places multiple sliver children in a linear array along
/// the cross axis.
///
/// ## Layout algorithm
///
/// _This section describes how the framework causes [RenderSliverCrossAxisGroup]
/// to position its children._
///
/// Layout for a [RenderSliverCrossAxisGroup] has four steps:
///
/// 1. Layout each child with a null or zero flex factor with cross axis constraint
///    being whatever cross axis space is remaining after laying out any previous
///    sliver. Slivers with null or zero flex factor should determine their own
///    [SliverGeometry.crossAxisExtent]. For example, the [SliverConstrainedCrossAxis]
///    widget uses either [SliverConstrainedCrossAxis.maxExtent] or
///    [SliverConstraints.crossAxisExtent], deciding between whichever is smaller.
/// 2. Divide up the remaining cross axis space among the children with non-zero flex
///    factors according to their flex factor. For example, a child with a flex
///    factor of 2.0 will receive twice the amount of cross axis space as a child
///    with a flex factor 1.0.
/// 3. Layout each of the remaining children with the cross axis constraint
///    allocated in the previous step.
/// 4. Set the geometry to that of whichever child has the longest
///    [SliverGeometry.scrollExtent] with the [SliverGeometry.crossAxisExtent] adjusted
///    to [SliverConstraints.crossAxisExtent].
///
/// {@tool dartpad}
/// In this sample the [SliverCrossAxisGroup] sizes its three [children] so that
/// the first normal [SliverList] has a flex factor of 1, the second [SliverConstrainedCrossAxis]
/// has a flex factor of 0 and a maximum cross axis extent of 200.0, and the third
/// [SliverCrossAxisExpanded] has a flex factor of 2.
///
/// ** See code in examples/api/lib/widgets/sliver/sliver_cross_axis_group.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SliverCrossAxisExpanded], which is the [ParentDataWidget] for setting a flex
///    value to a widget.
///  * [SliverConstrainedCrossAxis], which is a [RenderObjectWidget] for setting
///    an extent to constrain the widget to.
///  * [SliverMainAxisGroup], which is the [RenderObjectWidget] for laying out
///    multiple slivers along the main axis.
class SliverCrossAxisGroup extends MultiChildRenderObjectWidget {
  /// Creates a sliver that places sliver children in a linear array along
  /// the cross axis.
  const SliverCrossAxisGroup({super.key, required List<Widget> slivers}) : super(children: slivers);

  @override
  RenderSliverCrossAxisGroup createRenderObject(BuildContext context) {
    return RenderSliverCrossAxisGroup();
  }
}

/// A sliver that places multiple sliver children in a linear array along
/// the main axis, one after another.
///
/// ## Layout algorithm
///
/// _This section describes how the framework causes [RenderSliverMainAxisGroup]
/// to position its children._
///
/// Layout for a [RenderSliverMainAxisGroup] has four steps:
///
/// 1. Keep track of an offset variable which is the total [SliverGeometry.scrollExtent]
///    of the slivers laid out so far.
/// 2. To determine the constraints for the next sliver child to layout, calculate the
///    amount of paint extent occupied from 0.0 to the offset variable and subtract this from
///    [SliverConstraints.remainingPaintExtent] minus to use as the child's
///    [SliverConstraints.remainingPaintExtent]. For the [SliverConstraints.scrollOffset],
///    take the provided constraint's value and subtract out the offset variable, using
///    0.0 if negative.
/// 3. Once we finish laying out all the slivers, this offset variable represents
///    the total [SliverGeometry.scrollExtent] of the sliver group. Since it is possible
///    for specialized slivers to try to paint itself outside of the bounds of the
///    sliver group's scroll extent (see [SliverPersistentHeader]), we must do a
///    second pass to set a [SliverPhysicalParentData.paintOffset] to make sure it
///    is within the bounds of the sliver group.
/// 4. Finally, set the [RenderSliverMainAxisGroup.geometry] with the total
///    [SliverGeometry.scrollExtent], [SliverGeometry.paintExtent] calculated from
///    the constraints and [SliverGeometry.scrollExtent], and [SliverGeometry.maxPaintExtent].
///
/// {@tool dartpad}
/// In this sample the [CustomScrollView] renders a [SliverMainAxisGroup] and a
/// [SliverToBoxAdapter] with some content. The [SliverMainAxisGroup] renders a
/// [SliverAppBar], [SliverList], and [SliverToBoxAdapter]. Notice that when the
/// [SliverMainAxisGroup] goes out of view, so does the pinned [SliverAppBar].
///
/// ** See code in examples/api/lib/widgets/sliver/sliver_main_axis_group.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SliverPersistentHeader], which is a [RenderObjectWidget] which may require
///    adjustment to its [SliverPhysicalParentData.paintOffset] to make it fit
///    within the computed [SliverGeometry.scrollExtent] of the [SliverMainAxisGroup].
///  * [SliverCrossAxisGroup], which is the [RenderObjectWidget] for laying out
///    multiple slivers along the cross axis.
class SliverMainAxisGroup extends MultiChildRenderObjectWidget {
  /// Creates a sliver that places sliver children in a linear array along
  /// the main axis.
  const SliverMainAxisGroup({super.key, required List<Widget> slivers}) : super(children: slivers);

  @override
  MultiChildRenderObjectElement createElement() => _SliverMainAxisGroupElement(this);

  @override
  RenderSliverMainAxisGroup createRenderObject(BuildContext context) {
    return RenderSliverMainAxisGroup();
  }
}

class _SliverMainAxisGroupElement extends MultiChildRenderObjectElement {
  _SliverMainAxisGroupElement(SliverMainAxisGroup super.widget);

  @override
  void debugVisitOnstageChildren(ElementVisitor visitor) {
    children
        .where((Element e) {
          final RenderSliver renderSliver = e.renderObject! as RenderSliver;
          return renderSliver.geometry!.visible;
        })
        .forEach(visitor);
  }
}
