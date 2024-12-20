// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'page_storage.dart';
/// @docImport 'primary_scroll_controller.dart';
library;

import 'package:flutter/foundation.dart';

import 'basic.dart';
import 'framework.dart';
import 'media_query.dart';
import 'scroll_controller.dart';
import 'scroll_delegate.dart';
import 'scroll_physics.dart';
import 'scroll_view.dart';
import 'sliver.dart';
import 'ticker_provider.dart';

/// A scrolling container that animates items when they are inserted or removed.
///
/// This widget's [AnimatedListState] can be used to dynamically insert or
/// remove items. To refer to the [AnimatedListState] either provide a
/// [GlobalKey] or use the static [of] method from an item's input callback.
///
/// This widget is similar to one created by [ListView.builder].
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=ZtfItHwFlZ8}
///
/// {@tool dartpad}
/// This sample application uses an [AnimatedList] to create an effect when
/// items are removed or added to the list.
///
/// ** See code in examples/api/lib/widgets/animated_list/animated_list.0.dart **
/// {@end-tool}
///
/// By default, [AnimatedList] will automatically pad the limits of the
/// list's scrollable to avoid partial obstructions indicated by
/// [MediaQuery]'s padding. To avoid this behavior, override with a
/// zero [padding] property.
///
/// {@tool snippet}
/// The following example demonstrates how to override the default top and
/// bottom padding using [MediaQuery.removePadding].
///
/// ```dart
/// Widget myWidget(BuildContext context) {
///   return MediaQuery.removePadding(
///     context: context,
///     removeTop: true,
///     removeBottom: true,
///     child: AnimatedList(
///       initialItemCount: 50,
///       itemBuilder: (BuildContext context, int index, Animation<double> animation) {
///         return Card(
///           color: Colors.amber,
///           child: Center(child: Text('$index')),
///         );
///       }
///     ),
///   );
/// }
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [SliverAnimatedList], a sliver that animates items when they are inserted
///    or removed from a list.
///  * [SliverAnimatedGrid], a sliver which animates items when they are
///    inserted or removed from a grid.
///  * [AnimatedGrid], a non-sliver scrolling container that animates items when
///    they are inserted or removed in a grid.
class AnimatedList extends _AnimatedScrollView {
  /// Creates a scrolling container that animates items when they are inserted
  /// or removed.
  const AnimatedList({
    super.key,
    required super.itemBuilder,
    super.initialItemCount = 0,
    super.scrollDirection = Axis.vertical,
    super.reverse = false,
    super.controller,
    super.primary,
    super.physics,
    super.shrinkWrap = false,
    super.padding,
    super.clipBehavior = Clip.hardEdge,
  }) : assert(initialItemCount >= 0);

  /// A scrolling container that animates items with separators when they are inserted or removed.
  ///
  /// This widget's [AnimatedListState] can be used to dynamically insert or
  /// remove items. To refer to the [AnimatedListState] either provide a
  /// [GlobalKey] or use the static [of] method from an item's input callback.
  ///
  /// This widget is similar to one created by [ListView.separated].
  ///
  /// {@tool dartpad}
  /// This sample application uses an [AnimatedList.separated] to create an effect when
  /// items are removed or added to the list.
  ///
  /// ** See code in examples/api/lib/widgets/animated_list/animated_list_separated.0.dart **
  /// {@end-tool}
  ///
  /// By default, [AnimatedList.separated] will automatically pad the limits of the
  /// list's scrollable to avoid partial obstructions indicated by
  /// [MediaQuery]'s padding. To avoid this behavior, override with a
  /// zero [padding] property.
  ///
  /// {@tool snippet}
  /// The following example demonstrates how to override the default top and
  /// bottom padding using [MediaQuery.removePadding].
  ///
  /// ```dart
  /// Widget myWidget(BuildContext context) {
  ///   return MediaQuery.removePadding(
  ///     context: context,
  ///     removeTop: true,
  ///     removeBottom: true,
  ///     child: AnimatedList.separated(
  ///       initialItemCount: 50,
  ///       itemBuilder: (BuildContext context, int index, Animation<double> animation) {
  ///         return Card(
  ///           color: Colors.amber,
  ///           child: Center(child: Text('$index')),
  ///         );
  ///       },
  ///       separatorBuilder: (BuildContext context, int index, Animation<double> animation) {
  ///         return const Divider();
  ///       },
  ///       removedSeparatorBuilder: (BuildContext context, int index, Animation<double> animation) {
  ///         return const Divider();
  ///       }
  ///     ),
  ///   );
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [SliverAnimatedList], a sliver that animates items when they are inserted
  ///    or removed from a list.
  ///  * [SliverAnimatedGrid], a sliver which animates items when they are
  ///    inserted or removed from a grid.
  ///  * [AnimatedGrid], a non-sliver scrolling container that animates items when
  ///    they are inserted or removed in a grid.
  ///  * [AnimatedList], which animates items added and removed from a list instead
  ///    of a grid.
  AnimatedList.separated({
    super.key,
    required AnimatedItemBuilder itemBuilder,
    required AnimatedItemBuilder separatorBuilder,
    required AnimatedItemBuilder super.removedSeparatorBuilder,
    int initialItemCount = 0,
    super.scrollDirection = Axis.vertical,
    super.reverse = false,
    super.controller,
    super.primary,
    super.physics,
    super.shrinkWrap = false,
    super.padding,
    super.clipBehavior = Clip.hardEdge,
  }) : assert(initialItemCount >= 0),
       super(
         initialItemCount: _computeChildCountWithSeparators(initialItemCount),
         itemBuilder: (BuildContext context, int index, Animation<double> animation) {
           final int itemIndex = index ~/ 2;
           if (index.isEven) {
             return itemBuilder(context, itemIndex, animation);
           }
           return separatorBuilder(context, itemIndex, animation);
         },
       );

  /// The state from the closest instance of this class that encloses the given
  /// context.
  ///
  /// This method is typically used by [AnimatedList] item widgets that insert
  /// or remove items in response to user input.
  ///
  /// If no [AnimatedList] surrounds the context given, then this function will
  /// assert in debug mode and throw an exception in release mode.
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// This method does not create a dependency, and so will not cause rebuilding
  /// when the state changes.
  ///
  /// See also:
  ///
  ///  * [maybeOf], a similar function that will return null if no
  ///    [AnimatedList] ancestor is found.
  static AnimatedListState of(BuildContext context) {
    final AnimatedListState? result = AnimatedList.maybeOf(context);
    assert(() {
      if (result == null) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'AnimatedList.of() called with a context that does not contain an AnimatedList.',
          ),
          ErrorDescription(
            'No AnimatedList ancestor could be found starting from the context that was passed to AnimatedList.of().',
          ),
          ErrorHint(
            'This can happen when the context provided is from the same StatefulWidget that '
            'built the AnimatedList. Please see the AnimatedList documentation for examples '
            'of how to refer to an AnimatedListState object:\n'
            '  https://api.flutter.dev/flutter/widgets/AnimatedListState-class.html',
          ),
          context.describeElement('The context used was'),
        ]);
      }
      return true;
    }());
    return result!;
  }

  /// The [AnimatedListState] from the closest instance of [AnimatedList] that encloses the given
  /// context.
  ///
  /// This method is typically used by [AnimatedList] item widgets that insert
  /// or remove items in response to user input.
  ///
  /// If no [AnimatedList] surrounds the context given, then this function will
  /// return null.
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// This method does not create a dependency, and so will not cause rebuilding
  /// when the state changes.
  ///
  /// See also:
  ///
  ///  * [of], a similar function that will throw if no [AnimatedList] ancestor
  ///    is found.
  static AnimatedListState? maybeOf(BuildContext context) {
    return context.findAncestorStateOfType<AnimatedListState>();
  }

  // Helper method to compute the actual child count when taking separators into account.
  static int _computeChildCountWithSeparators(int itemCount) {
    if (itemCount == 0) {
      return 0;
    }
    return itemCount * 2 - 1;
  }

  @override
  AnimatedListState createState() => AnimatedListState();
}

/// The [AnimatedListState] for [AnimatedList], a scrolling list container that
/// animates items when they are inserted or removed.
///
/// When an item is inserted with [insertItem] an animation begins running. The
/// animation is passed to [AnimatedList.itemBuilder] whenever the item's widget
/// is needed.
///
/// When multiple items are inserted with [insertAllItems] an animation begins running.
/// The animation is passed to [AnimatedList.itemBuilder] whenever the item's widget
/// is needed.
///
/// If using [AnimatedList.separated], the animation is also passed to
/// `AnimatedList.separatorBuilder` whenever the separator's widget is needed.
///
/// When an item is removed with [removeItem] its animation is reversed.
/// The removed item's animation is passed to the [removeItem] builder
/// parameter. If using [AnimatedList.separated], the corresponding separator's
/// animation is also passed to the [AnimatedList.removedSeparatorBuilder] parameter.
///
/// An app that needs to insert or remove items in response to an event
/// can refer to the [AnimatedList]'s state with a global key:
///
/// ```dart
/// // (e.g. in a stateful widget)
/// GlobalKey<AnimatedListState> listKey = GlobalKey<AnimatedListState>();
///
/// // ...
///
/// @override
/// Widget build(BuildContext context) {
///   return AnimatedList(
///     key: listKey,
///     itemBuilder: (BuildContext context, int index, Animation<double> animation) {
///       return const Placeholder();
///     },
///   );
/// }
///
/// // ...
///
/// void _updateList() {
///   // adds "123" to the AnimatedList
///   listKey.currentState!.insertItem(123);
/// }
/// ```
///
/// [AnimatedList] item input handlers can also refer to their [AnimatedListState]
/// with the static [AnimatedList.of] method.
class AnimatedListState extends _AnimatedScrollViewState<AnimatedList> {
  @protected
  @override
  Widget build(BuildContext context) {
    return _wrap(
      SliverAnimatedList(
        key: _sliverAnimatedMultiBoxKey,
        itemBuilder: widget.itemBuilder,
        initialItemCount: widget.initialItemCount,
      ),
      widget.scrollDirection,
    );
  }
}

/// A scrolling container that animates items when they are inserted into or removed from a grid.
/// in a grid.
///
/// This widget's [AnimatedGridState] can be used to dynamically insert or
/// remove items. To refer to the [AnimatedGridState] either provide a
/// [GlobalKey] or use the static [of] method from an item's input callback.
///
/// This widget is similar to one created by [GridView.builder].
///
/// {@tool dartpad}
/// This sample application uses an [AnimatedGrid] to create an effect when
/// items are removed or added to the grid.
///
/// ** See code in examples/api/lib/widgets/animated_grid/animated_grid.0.dart **
/// {@end-tool}
///
/// By default, [AnimatedGrid] will automatically pad the limits of the
/// grid's scrollable to avoid partial obstructions indicated by
/// [MediaQuery]'s padding. To avoid this behavior, override with a
/// zero [padding] property.
///
/// {@tool snippet}
/// The following example demonstrates how to override the default top and
/// bottom padding using [MediaQuery.removePadding].
///
/// ```dart
/// Widget myWidget(BuildContext context) {
///   return MediaQuery.removePadding(
///     context: context,
///     removeTop: true,
///     removeBottom: true,
///     child: AnimatedGrid(
///       gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
///         crossAxisCount: 3,
///       ),
///       initialItemCount: 50,
///       itemBuilder: (BuildContext context, int index, Animation<double> animation) {
///         return Card(
///           color: Colors.amber,
///           child: Center(child: Text('$index')),
///         );
///       }
///     ),
///   );
/// }
/// ```
/// {@end-tool}
///
/// See also:
///
/// * [SliverAnimatedGrid], a sliver which animates items when they are inserted
///   into or removed from a grid.
/// * [SliverAnimatedList], a sliver which animates items added and removed from
///   a list instead of a grid.
/// * [AnimatedList], which animates items added and removed from a list instead
///   of a grid.
class AnimatedGrid extends _AnimatedScrollView {
  /// Creates a scrolling container that animates items when they are inserted
  /// or removed.
  const AnimatedGrid({
    super.key,
    required super.itemBuilder,
    required this.gridDelegate,
    super.initialItemCount = 0,
    super.scrollDirection = Axis.vertical,
    super.reverse = false,
    super.controller,
    super.primary,
    super.physics,
    super.padding,
    super.clipBehavior = Clip.hardEdge,
  }) : assert(initialItemCount >= 0);

  /// {@template flutter.widgets.AnimatedGrid.gridDelegate}
  /// A delegate that controls the layout of the children within the
  /// [AnimatedGrid].
  ///
  /// See also:
  ///
  ///  * [SliverGridDelegateWithFixedCrossAxisCount], which creates a layout with
  ///    a fixed number of tiles in the cross axis.
  ///  * [SliverGridDelegateWithMaxCrossAxisExtent], which creates a layout with
  ///    tiles that have a maximum cross-axis extent.
  /// {@endtemplate}
  final SliverGridDelegate gridDelegate;

  /// The state from the closest instance of this class that encloses the given
  /// context.
  ///
  /// This method is typically used by [AnimatedGrid] item widgets that insert
  /// or remove items in response to user input.
  ///
  /// If no [AnimatedGrid] surrounds the context given, then this function will
  /// assert in debug mode and throw an exception in release mode.
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// This method does not create a dependency, and so will not cause rebuilding
  /// when the state changes.
  ///
  /// See also:
  ///
  ///  * [maybeOf], a similar function that will return null if no
  ///    [AnimatedGrid] ancestor is found.
  static AnimatedGridState of(BuildContext context) {
    final AnimatedGridState? result = AnimatedGrid.maybeOf(context);
    assert(() {
      if (result == null) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary(
            'AnimatedGrid.of() called with a context that does not contain an AnimatedGrid.',
          ),
          ErrorDescription(
            'No AnimatedGrid ancestor could be found starting from the context that was passed to AnimatedGrid.of().',
          ),
          ErrorHint(
            'This can happen when the context provided is from the same StatefulWidget that '
            'built the AnimatedGrid. Please see the AnimatedGrid documentation for examples '
            'of how to refer to an AnimatedGridState object:\n'
            '  https://api.flutter.dev/flutter/widgets/AnimatedGridState-class.html',
          ),
          context.describeElement('The context used was'),
        ]);
      }
      return true;
    }());
    return result!;
  }

  /// The state from the closest instance of this class that encloses the given
  /// context.
  ///
  /// This method is typically used by [AnimatedGrid] item widgets that insert
  /// or remove items in response to user input.
  ///
  /// If no [AnimatedGrid] surrounds the context given, then this function will
  /// return null.
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// This method does not create a dependency, and so will not cause rebuilding
  /// when the state changes.
  ///
  /// See also:
  ///
  ///  * [of], a similar function that will throw if no [AnimatedGrid] ancestor
  ///    is found.
  static AnimatedGridState? maybeOf(BuildContext context) {
    return context.findAncestorStateOfType<AnimatedGridState>();
  }

  @override
  AnimatedGridState createState() => AnimatedGridState();
}

/// The [State] for an [AnimatedGrid] that animates items when they are
/// inserted or removed.
///
/// When an item is inserted with [insertItem] an animation begins running. The
/// animation is passed to [AnimatedGrid.itemBuilder] whenever the item's widget
/// is needed.
///
/// When an item is removed with [removeItem] its animation is reversed.
/// The removed item's animation is passed to the [removeItem] builder
/// parameter.
///
/// An app that needs to insert or remove items in response to an event
/// can refer to the [AnimatedGrid]'s state with a global key:
///
/// ```dart
/// // (e.g. in a stateful widget)
/// GlobalKey<AnimatedGridState> gridKey = GlobalKey<AnimatedGridState>();
///
/// // ...
///
/// @override
/// Widget build(BuildContext context) {
///   return AnimatedGrid(
///     key: gridKey,
///     itemBuilder: (BuildContext context, int index, Animation<double> animation) {
///       return const Placeholder();
///     },
///     gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(maxCrossAxisExtent: 100.0),
///   );
/// }
///
/// // ...
///
/// void _updateGrid() {
///   // adds "123" to the AnimatedGrid
///   gridKey.currentState!.insertItem(123);
/// }
/// ```
///
/// [AnimatedGrid] item input handlers can also refer to their [AnimatedGridState]
/// with the static [AnimatedGrid.of] method.
class AnimatedGridState extends _AnimatedScrollViewState<AnimatedGrid> {
  @protected
  @override
  Widget build(BuildContext context) {
    return _wrap(
      SliverAnimatedGrid(
        key: _sliverAnimatedMultiBoxKey,
        gridDelegate: widget.gridDelegate,
        itemBuilder: widget.itemBuilder,
        initialItemCount: widget.initialItemCount,
      ),
      widget.scrollDirection,
    );
  }
}

abstract class _AnimatedScrollView extends StatefulWidget {
  /// Creates a scrolling container that animates items when they are inserted
  /// or removed.
  const _AnimatedScrollView({
    super.key,
    required this.itemBuilder,
    this.removedSeparatorBuilder,
    this.initialItemCount = 0,
    this.scrollDirection = Axis.vertical,
    this.reverse = false,
    this.controller,
    this.primary,
    this.physics,
    this.shrinkWrap = false,
    this.padding,
    this.clipBehavior = Clip.hardEdge,
  }) : assert(initialItemCount >= 0);

  /// {@template flutter.widgets.AnimatedScrollView.itemBuilder}
  /// Called, as needed, to build children widgets.
  ///
  /// Children are only built when they're scrolled into view.
  ///
  /// The [AnimatedItemBuilder] index parameter indicates the item's
  /// position in the scroll view. The value of the index parameter will be
  /// between 0 and [initialItemCount] plus the total number of items that have
  /// been inserted with [AnimatedListState.insertItem] or
  /// [AnimatedGridState.insertItem] and less the total number of items that
  /// have been removed with [AnimatedListState.removeItem] or
  /// [AnimatedGridState.removeItem].
  ///
  /// Implementations of this callback should assume that
  /// `removeItem` removes an item immediately.
  /// {@endtemplate}
  final AnimatedItemBuilder itemBuilder;

  /// {@template flutter.widgets.AnimatedScrollView.removedSeparatorBuilder}
  /// Called, as needed, to build separator widgets.
  ///
  /// Separators are only built when they're scrolled into view.
  ///
  /// The [AnimatedItemBuilder] index parameter indicates the
  /// separator's corresponding item's position in the scroll view. The value
  /// of the index parameter will be between 0 and [initialItemCount] plus the
  /// total number of items that have been inserted with [AnimatedListState.insertItem]
  /// and less the total number of items that have been removed with [AnimatedListState.removeItem].
  ///
  /// Implementations of this callback should assume that
  /// `removeItem` removes an item immediately.
  /// {@endtemplate}
  final AnimatedItemBuilder? removedSeparatorBuilder;

  /// {@template flutter.widgets.AnimatedScrollView.initialItemCount}
  /// The number of items the [AnimatedList] or [AnimatedGrid] will start with.
  ///
  /// The appearance of the initial items is not animated. They
  /// are created, as needed, by [itemBuilder] with an animation parameter
  /// of [kAlwaysCompleteAnimation].
  /// {@endtemplate}
  final int initialItemCount;

  /// {@macro flutter.widgets.scroll_view.scrollDirection}
  final Axis scrollDirection;

  /// Whether the scroll view scrolls in the reading direction.
  ///
  /// For example, if the reading direction is left-to-right and
  /// [scrollDirection] is [Axis.horizontal], then the scroll view scrolls from
  /// left to right when [reverse] is false and from right to left when
  /// [reverse] is true.
  ///
  /// Similarly, if [scrollDirection] is [Axis.vertical], then the scroll view
  /// scrolls from top to bottom when [reverse] is false and from bottom to top
  /// when [reverse] is true.
  ///
  /// Defaults to false.
  final bool reverse;

  /// An object that can be used to control the position to which this scroll
  /// view is scrolled.
  ///
  /// Must be null if [primary] is true.
  ///
  /// A [ScrollController] serves several purposes. It can be used to control
  /// the initial scroll position (see [ScrollController.initialScrollOffset]).
  /// It can be used to control whether the scroll view should automatically
  /// save and restore its scroll position in the [PageStorage] (see
  /// [ScrollController.keepScrollOffset]). It can be used to read the current
  /// scroll position (see [ScrollController.offset]), or change it (see
  /// [ScrollController.animateTo]).
  final ScrollController? controller;

  /// Whether this is the primary scroll view associated with the parent
  /// [PrimaryScrollController].
  ///
  /// On iOS, this identifies the scroll view that will scroll to top in
  /// response to a tap in the status bar.
  ///
  /// Defaults to true when [scrollDirection] is [Axis.vertical] and
  /// [controller] is null.
  final bool? primary;

  /// How the scroll view should respond to user input.
  ///
  /// For example, this determines how the scroll view continues to animate after the
  /// user stops dragging the scroll view.
  ///
  /// Defaults to matching platform conventions.
  final ScrollPhysics? physics;

  /// Whether the extent of the scroll view in the [scrollDirection] should be
  /// determined by the contents being viewed.
  ///
  /// If the scroll view does not shrink wrap, then the scroll view will expand
  /// to the maximum allowed size in the [scrollDirection]. If the scroll view
  /// has unbounded constraints in the [scrollDirection], then [shrinkWrap] must
  /// be true.
  ///
  /// Shrink wrapping the content of the scroll view is significantly more
  /// expensive than expanding to the maximum allowed size because the content
  /// can expand and contract during scrolling, which means the size of the
  /// scroll view needs to be recomputed whenever the scroll position changes.
  ///
  /// Defaults to false.
  final bool shrinkWrap;

  /// The amount of space by which to inset the children.
  final EdgeInsetsGeometry? padding;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;
}

abstract class _AnimatedScrollViewState<T extends _AnimatedScrollView> extends State<T>
    with TickerProviderStateMixin {
  final GlobalKey<_SliverAnimatedMultiBoxAdaptorState<_SliverAnimatedMultiBoxAdaptor>>
  _sliverAnimatedMultiBoxKey = GlobalKey();

  /// Insert an item at [index] and start an animation that will be passed
  /// to [AnimatedGrid.itemBuilder] or [AnimatedList.itemBuilder] when the item
  /// is visible.
  ///
  /// If using [AnimatedList.separated] the animation will also be passed
  /// to `separatorBuilder`.
  ///
  /// This method's semantics are the same as Dart's [List.insert] method: it
  /// increases the length of the list of items by one and shifts
  /// all items at or after [index] towards the end of the list of items.
  void insertItem(int index, {Duration duration = _kDuration}) {
    if (widget.removedSeparatorBuilder == null) {
      _sliverAnimatedMultiBoxKey.currentState!.insertItem(index, duration: duration);
    } else {
      final int itemIndex = _computeItemIndex(index);
      _sliverAnimatedMultiBoxKey.currentState!.insertItem(itemIndex, duration: duration);
      if (_itemsCount > 1) {
        // Because `insertItem` moves the items after the index, we need to insert the separator
        // at the same index as the item. If there is only one item, we don't need to insert a separator.
        _sliverAnimatedMultiBoxKey.currentState!.insertItem(itemIndex, duration: duration);
      }
    }
  }

  /// Insert multiple items at [index] and start an animation that will be passed
  /// to [AnimatedGrid.itemBuilder] or [AnimatedList.itemBuilder] when the items
  /// are visible.
  ///
  /// If using [AnimatedList.separated] the animation will also be passed to `separatorBuilder`.
  void insertAllItems(
    int index,
    int length, {
    Duration duration = _kDuration,
    bool isAsync = false,
  }) {
    if (widget.removedSeparatorBuilder == null) {
      _sliverAnimatedMultiBoxKey.currentState!.insertAllItems(index, length, duration: duration);
    } else {
      final int itemIndex = _computeItemIndex(index);
      final int lengthWithSeparators = _itemsCount == 0 ? length * 2 - 1 : length * 2;
      _sliverAnimatedMultiBoxKey.currentState!.insertAllItems(
        itemIndex,
        lengthWithSeparators,
        duration: duration,
      );
    }
  }

  /// Remove the item at [index] and start an animation that will be passed to
  /// [builder] when the item is visible.
  ///
  /// If using [AnimatedList.separated], the animation will also be passed to the
  /// corresponding separator's [AnimatedList.removedSeparatorBuilder].
  ///
  /// Items are removed immediately. After an item has been removed, its index
  /// will no longer be passed to the [builder]. However, the
  /// item will still appear for [duration] and during that time
  /// [builder] must construct its widget as needed.
  ///
  /// This method's semantics are the same as Dart's [List.remove] method: it
  /// decreases the length of items by one and shifts all items at or before
  /// [index] towards the beginning of the list of items.
  ///
  /// See also:
  ///
  ///   * [AnimatedRemovedItemBuilder], which describes the arguments to the
  ///     [builder] argument.
  void removeItem(int index, AnimatedRemovedItemBuilder builder, {Duration duration = _kDuration}) {
    final AnimatedItemBuilder? removedSeparatorBuilder = widget.removedSeparatorBuilder;
    if (removedSeparatorBuilder == null) {
      // There are no separators. Remove only the item.
      _sliverAnimatedMultiBoxKey.currentState!.removeItem(index, builder, duration: duration);
    } else {
      final int itemIndex = _computeItemIndex(index);
      // Remove the item
      _sliverAnimatedMultiBoxKey.currentState!.removeItem(itemIndex, builder, duration: duration);
      if (_itemsCount > 1) {
        if (itemIndex == _itemsCount - 1) {
          // The item was removed from the end of the list, so the separator to remove is the one at `last index` - 1.
          _sliverAnimatedMultiBoxKey.currentState!.removeItem(
            itemIndex - 1,
            _toRemovedItemBuilder(removedSeparatorBuilder, index - 1),
            duration: duration,
          );
        } else {
          // The item was removed from the middle or beginning of the list,
          // so the corresponding separator took its place and needs to be removed at `itemIndex`.
          _sliverAnimatedMultiBoxKey.currentState!.removeItem(
            itemIndex,
            _toRemovedItemBuilder(removedSeparatorBuilder, index),
            duration: duration,
          );
        }
      }
    }
  }

  /// Remove all the items and start an animation that will be passed to
  /// [builder] when the items are visible.
  ///
  /// If using [AnimatedList.separated], the animation will also be passed
  /// to the corresponding separator's [AnimatedList.removedSeparatorBuilder].
  ///
  /// Items are removed immediately. However, the
  /// items will still appear for [duration], and during that time
  /// [builder] must construct its widget as needed.
  ///
  /// This method's semantics are the same as Dart's [List.clear] method: it
  /// removes all the items in the list.
  ///
  /// See also:
  ///
  ///   * [AnimatedRemovedItemBuilder], which describes the arguments to the
  ///     [builder] argument.
  void removeAllItems(AnimatedRemovedItemBuilder builder, {Duration duration = _kDuration}) {
    final AnimatedItemBuilder? removedSeparatorBuilder = widget.removedSeparatorBuilder;
    if (removedSeparatorBuilder == null) {
      // There are no separators. We can remove all items with the same builder.
      _sliverAnimatedMultiBoxKey.currentState!.removeAllItems(builder, duration: duration);
      return;
    }

    // There are separators. We need to remove items and separators separately
    // with the corresponding builders.
    for (int index = _itemsCount - 1; index >= 0; index--) {
      if (index.isEven) {
        _sliverAnimatedMultiBoxKey.currentState!.removeItem(index, builder, duration: duration);
      } else {
        // The index of the separator's corresponding item
        final int itemIndex = index ~/ 2;
        _sliverAnimatedMultiBoxKey.currentState!.removeItem(
          index,
          _toRemovedItemBuilder(removedSeparatorBuilder, itemIndex),
          duration: duration,
        );
      }
    }
  }

  int get _itemsCount => _sliverAnimatedMultiBoxKey.currentState!._itemsCount;

  // Helper method to compute the index for the item to insert or remove considering the separators in between.
  int _computeItemIndex(int index) {
    if (index == 0) {
      return index;
    }
    final int itemsAndSeparatorsCount = _itemsCount;
    final int separatorsCount = itemsAndSeparatorsCount ~/ 2;
    final int separatedItemsCount = _itemsCount - separatorsCount;

    final bool isNewLastIndex = index == separatedItemsCount;
    final int indexAdjustedForSeparators = index * 2;
    return isNewLastIndex ? indexAdjustedForSeparators - 1 : indexAdjustedForSeparators;
  }

  // Helper method to create an [AnimatedRemovedItemBuilder]
  // from an [AnimatedItemBuilder] for given [index].
  AnimatedRemovedItemBuilder _toRemovedItemBuilder(AnimatedItemBuilder builder, int index) {
    return (BuildContext context, Animation<double> animation) {
      return builder(context, index, animation);
    };
  }

  Widget _wrap(Widget sliver, Axis direction) {
    EdgeInsetsGeometry? effectivePadding = widget.padding;
    if (widget.padding == null) {
      final MediaQueryData? mediaQuery = MediaQuery.maybeOf(context);
      if (mediaQuery != null) {
        // Automatically pad sliver with padding from MediaQuery.
        final EdgeInsets mediaQueryHorizontalPadding = mediaQuery.padding.copyWith(
          top: 0.0,
          bottom: 0.0,
        );
        final EdgeInsets mediaQueryVerticalPadding = mediaQuery.padding.copyWith(
          left: 0.0,
          right: 0.0,
        );
        // Consume the main axis padding with SliverPadding.
        effectivePadding =
            direction == Axis.vertical ? mediaQueryVerticalPadding : mediaQueryHorizontalPadding;
        // Leave behind the cross axis padding.
        sliver = MediaQuery(
          data: mediaQuery.copyWith(
            padding:
                direction == Axis.vertical
                    ? mediaQueryHorizontalPadding
                    : mediaQueryVerticalPadding,
          ),
          child: sliver,
        );
      }
    }

    if (effectivePadding != null) {
      sliver = SliverPadding(padding: effectivePadding, sliver: sliver);
    }
    return CustomScrollView(
      scrollDirection: widget.scrollDirection,
      reverse: widget.reverse,
      controller: widget.controller,
      primary: widget.primary,
      physics: widget.physics,
      clipBehavior: widget.clipBehavior,
      shrinkWrap: widget.shrinkWrap,
      slivers: <Widget>[sliver],
    );
  }
}

/// Signature for the builder callback used by [AnimatedList], [AnimatedList.separated]
/// & [AnimatedGrid] to build their animated children.
///
/// This signature is also used by [AnimatedList.separated] to build its separators and
/// to animate their exit transition after their corresponding item has been removed.
///
/// The [context] argument is the build context where the widget will be
/// created, the [index] is the index of the item to be built, and the
/// [animation] is an [Animation] that should be used to animate an entry
/// transition for the widget that is built.
///
/// For [AnimatedList.separated], the [index] is the index
/// of the corresponding item of the separator that is built or removed.
/// For [AnimatedList.separated] `removedSeparatorBuilder`, the [animation] should be used
/// to animate an exit transition for the widget that is built.
///
/// See also:
///
/// * [AnimatedRemovedItemBuilder], a builder that is for removing items with
///   animations instead of adding them.
typedef AnimatedItemBuilder =
    Widget Function(BuildContext context, int index, Animation<double> animation);

/// Signature for the builder callback used in [AnimatedListState.removeItem] and
/// [AnimatedGridState.removeItem] to animate their children after they have
/// been removed.
///
/// The [context] argument is the build context where the widget will be
/// created, and the [animation] is an [Animation] that should be used to
/// animate an exit transition for the widget that is built.
///
/// See also:
///
/// * [AnimatedItemBuilder], a builder that is for adding items with animations
///   instead of removing them.
typedef AnimatedRemovedItemBuilder =
    Widget Function(BuildContext context, Animation<double> animation);

// The default insert/remove animation duration.
const Duration _kDuration = Duration(milliseconds: 300);

// Incoming and outgoing animated items.
class _ActiveItem implements Comparable<_ActiveItem> {
  _ActiveItem.incoming(this.controller, this.itemIndex) : removedItemBuilder = null;
  _ActiveItem.outgoing(this.controller, this.itemIndex, this.removedItemBuilder);
  _ActiveItem.index(this.itemIndex) : controller = null, removedItemBuilder = null;

  final AnimationController? controller;
  final AnimatedRemovedItemBuilder? removedItemBuilder;
  int itemIndex;

  @override
  int compareTo(_ActiveItem other) => itemIndex - other.itemIndex;
}

/// A [SliverList] that animates items when they are inserted or removed.
///
/// This widget's [SliverAnimatedListState] can be used to dynamically insert or
/// remove items. To refer to the [SliverAnimatedListState] either provide a
/// [GlobalKey] or use the static [SliverAnimatedList.of] method from a list item's
/// input callback.
///
/// {@tool dartpad}
/// This sample application uses a [SliverAnimatedList] to create an animated
/// effect when items are removed or added to the list.
///
/// ** See code in examples/api/lib/widgets/animated_list/sliver_animated_list.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SliverList], which does not animate items when they are inserted or
///    removed.
///  * [AnimatedList], a non-sliver scrolling container that animates items when
///    they are inserted or removed.
///  * [SliverAnimatedGrid], a sliver which animates items when they are
///    inserted into or removed from a grid.
///  * [AnimatedGrid], a non-sliver scrolling container that animates items when
///    they are inserted into or removed from a grid.
class SliverAnimatedList extends _SliverAnimatedMultiBoxAdaptor {
  /// Creates a [SliverList] that animates items when they are inserted or
  /// removed.
  const SliverAnimatedList({
    super.key,
    required super.itemBuilder,
    super.findChildIndexCallback,
    super.initialItemCount = 0,
  }) : assert(initialItemCount >= 0);

  @override
  SliverAnimatedListState createState() => SliverAnimatedListState();

  /// The [SliverAnimatedListState] from the closest instance of this class that encloses the given
  /// context.
  ///
  /// This method is typically used by [SliverAnimatedList] item widgets that
  /// insert or remove items in response to user input.
  ///
  /// If no [SliverAnimatedList] surrounds the context given, then this function
  /// will assert in debug mode and throw an exception in release mode.
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// This method does not create a dependency, and so will not cause rebuilding
  /// when the state changes.
  ///
  /// See also:
  ///
  ///  * [maybeOf], a similar function that will return null if no
  ///    [SliverAnimatedList] ancestor is found.
  static SliverAnimatedListState of(BuildContext context) {
    final SliverAnimatedListState? result = SliverAnimatedList.maybeOf(context);
    assert(() {
      if (result == null) {
        throw FlutterError(
          'SliverAnimatedList.of() called with a context that does not contain a SliverAnimatedList.\n'
          'No SliverAnimatedListState ancestor could be found starting from the '
          'context that was passed to SliverAnimatedListState.of(). This can '
          'happen when the context provided is from the same StatefulWidget that '
          'built the AnimatedList. Please see the SliverAnimatedList documentation '
          'for examples of how to refer to an AnimatedListState object: '
          'https://api.flutter.dev/flutter/widgets/SliverAnimatedListState-class.html\n'
          'The context used was:\n'
          '  $context',
        );
      }
      return true;
    }());
    return result!;
  }

  /// The [SliverAnimatedListState] from the closest instance of this class that encloses the given
  /// context.
  ///
  /// This method is typically used by [SliverAnimatedList] item widgets that
  /// insert or remove items in response to user input.
  ///
  /// If no [SliverAnimatedList] surrounds the context given, then this function
  /// will return null.
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// This method does not create a dependency, and so will not cause rebuilding
  /// when the state changes.
  ///
  /// See also:
  ///
  ///  * [of], a similar function that will throw if no [SliverAnimatedList]
  ///    ancestor is found.
  static SliverAnimatedListState? maybeOf(BuildContext context) {
    return context.findAncestorStateOfType<SliverAnimatedListState>();
  }
}

/// The state for a [SliverAnimatedList] that animates items when they are
/// inserted or removed.
///
/// When an item is inserted with [insertItem] an animation begins running. The
/// animation is passed to [SliverAnimatedList.itemBuilder] whenever the item's
/// widget is needed.
///
/// When an item is removed with [removeItem] its animation is reversed.
/// The removed item's animation is passed to the [removeItem] builder
/// parameter.
///
/// An app that needs to insert or remove items in response to an event
/// can refer to the [SliverAnimatedList]'s state with a global key:
///
/// ```dart
/// // (e.g. in a stateful widget)
/// GlobalKey<AnimatedListState> listKey = GlobalKey<AnimatedListState>();
///
/// // ...
///
/// @override
/// Widget build(BuildContext context) {
///   return AnimatedList(
///     key: listKey,
///     itemBuilder: (BuildContext context, int index, Animation<double> animation) {
///       return const Placeholder();
///     },
///   );
/// }
///
/// // ...
///
/// void _updateList() {
///   // adds "123" to the AnimatedList
///   listKey.currentState!.insertItem(123);
/// }
/// ```
///
/// [SliverAnimatedList] item input handlers can also refer to their
/// [SliverAnimatedListState] with the static [SliverAnimatedList.of] method.
class SliverAnimatedListState extends _SliverAnimatedMultiBoxAdaptorState<SliverAnimatedList> {
  @protected
  @override
  Widget build(BuildContext context) {
    return SliverList(delegate: _createDelegate());
  }
}

/// A [SliverGrid] that animates items when they are inserted or removed.
///
/// This widget's [SliverAnimatedGridState] can be used to dynamically insert or
/// remove items. To refer to the [SliverAnimatedGridState] either provide a
/// [GlobalKey] or use the static [SliverAnimatedGrid.of] method from an item's
/// input callback.
///
/// {@tool dartpad}
/// This sample application uses a [SliverAnimatedGrid] to create an animated
/// effect when items are removed or added to the grid.
///
/// ** See code in examples/api/lib/widgets/animated_grid/sliver_animated_grid.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [AnimatedGrid], a non-sliver scrolling container that animates items when
///    they are inserted into or removed from a grid.
///  * [SliverGrid], which does not animate items when they are inserted or
///    removed from a grid.
///  * [SliverList], which displays a non-animated list of items.
///  * [SliverAnimatedList], which animates items added and removed from a list
///    instead of a grid.
class SliverAnimatedGrid extends _SliverAnimatedMultiBoxAdaptor {
  /// Creates a [SliverGrid] that animates items when they are inserted or
  /// removed.
  const SliverAnimatedGrid({
    super.key,
    required super.itemBuilder,
    required this.gridDelegate,
    super.findChildIndexCallback,
    super.initialItemCount = 0,
  }) : assert(initialItemCount >= 0);

  @override
  SliverAnimatedGridState createState() => SliverAnimatedGridState();

  /// {@macro flutter.widgets.AnimatedGrid.gridDelegate}
  final SliverGridDelegate gridDelegate;

  /// The state from the closest instance of this class that encloses the given
  /// context.
  ///
  /// This method is typically used by [SliverAnimatedGrid] item widgets that
  /// insert or remove items in response to user input.
  ///
  /// If no [SliverAnimatedGrid] surrounds the context given, then this function
  /// will assert in debug mode and throw an exception in release mode.
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// See also:
  ///
  ///  * [maybeOf], a similar function that will return null if no
  ///    [SliverAnimatedGrid] ancestor is found.
  static SliverAnimatedGridState of(BuildContext context) {
    final SliverAnimatedGridState? result =
        context.findAncestorStateOfType<SliverAnimatedGridState>();
    assert(() {
      if (result == null) {
        throw FlutterError(
          'SliverAnimatedGrid.of() called with a context that does not contain a SliverAnimatedGrid.\n'
          'No SliverAnimatedGridState ancestor could be found starting from the '
          'context that was passed to SliverAnimatedGridState.of(). This can '
          'happen when the context provided is from the same StatefulWidget that '
          'built the AnimatedGrid. Please see the SliverAnimatedGrid documentation '
          'for examples of how to refer to an AnimatedGridState object: '
          'https://api.flutter.dev/flutter/widgets/SliverAnimatedGridState-class.html\n'
          'The context used was:\n'
          '  $context',
        );
      }
      return true;
    }());
    return result!;
  }

  /// The state from the closest instance of this class that encloses the given
  /// context.
  ///
  /// This method is typically used by [SliverAnimatedGrid] item widgets that
  /// insert or remove items in response to user input.
  ///
  /// If no [SliverAnimatedGrid] surrounds the context given, then this function
  /// will return null.
  ///
  /// This method can be expensive (it walks the element tree).
  ///
  /// See also:
  ///
  ///  * [of], a similar function that will throw if no [SliverAnimatedGrid]
  ///    ancestor is found.
  static SliverAnimatedGridState? maybeOf(BuildContext context) {
    return context.findAncestorStateOfType<SliverAnimatedGridState>();
  }
}

/// The state for a [SliverAnimatedGrid] that animates items when they are
/// inserted or removed.
///
/// When an item is inserted with [insertItem] an animation begins running. The
/// animation is passed to [SliverAnimatedGrid.itemBuilder] whenever the item's
/// widget is needed.
///
/// When an item is removed with [removeItem] its animation is reversed.
/// The removed item's animation is passed to the [removeItem] builder
/// parameter.
///
/// An app that needs to insert or remove items in response to an event
/// can refer to the [SliverAnimatedGrid]'s state with a global key:
///
/// ```dart
/// // (e.g. in a stateful widget)
/// GlobalKey<AnimatedGridState> gridKey = GlobalKey<AnimatedGridState>();
///
/// // ...
///
/// @override
/// Widget build(BuildContext context) {
///   return AnimatedGrid(
///     key: gridKey,
///     itemBuilder: (BuildContext context, int index, Animation<double> animation) {
///       return const Placeholder();
///     },
///     gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(maxCrossAxisExtent: 100.0),
///   );
/// }
///
/// // ...
///
/// void _updateGrid() {
///   // adds "123" to the AnimatedGrid
///   gridKey.currentState!.insertItem(123);
/// }
/// ```
///
/// [SliverAnimatedGrid] item input handlers can also refer to their
/// [SliverAnimatedGridState] with the static [SliverAnimatedGrid.of] method.
class SliverAnimatedGridState extends _SliverAnimatedMultiBoxAdaptorState<SliverAnimatedGrid> {
  @protected
  @override
  Widget build(BuildContext context) {
    return SliverGrid(gridDelegate: widget.gridDelegate, delegate: _createDelegate());
  }
}

abstract class _SliverAnimatedMultiBoxAdaptor extends StatefulWidget {
  /// Creates a sliver that animates items when they are inserted or removed.
  const _SliverAnimatedMultiBoxAdaptor({
    super.key,
    required this.itemBuilder,
    this.findChildIndexCallback,
    this.initialItemCount = 0,
  }) : assert(initialItemCount >= 0);

  /// {@macro flutter.widgets.AnimatedScrollView.itemBuilder}
  final AnimatedItemBuilder itemBuilder;

  /// {@macro flutter.widgets.SliverChildBuilderDelegate.findChildIndexCallback}
  final ChildIndexGetter? findChildIndexCallback;

  /// {@macro flutter.widgets.AnimatedScrollView.initialItemCount}
  final int initialItemCount;
}

abstract class _SliverAnimatedMultiBoxAdaptorState<T extends _SliverAnimatedMultiBoxAdaptor>
    extends State<T>
    with TickerProviderStateMixin {
  @override
  void initState() {
    super.initState();
    _itemsCount = widget.initialItemCount;
  }

  @override
  void dispose() {
    for (final _ActiveItem item in _incomingItems.followedBy(_outgoingItems)) {
      item.controller!.dispose();
    }
    super.dispose();
  }

  final List<_ActiveItem> _incomingItems = <_ActiveItem>[];
  final List<_ActiveItem> _outgoingItems = <_ActiveItem>[];
  int _itemsCount = 0;

  _ActiveItem? _removeActiveItemAt(List<_ActiveItem> items, int itemIndex) {
    final int i = binarySearch(items, _ActiveItem.index(itemIndex));
    return i == -1 ? null : items.removeAt(i);
  }

  _ActiveItem? _activeItemAt(List<_ActiveItem> items, int itemIndex) {
    final int i = binarySearch(items, _ActiveItem.index(itemIndex));
    return i == -1 ? null : items[i];
  }

  // The insertItem() and removeItem() index parameters are defined as if the
  // removeItem() operation removed the corresponding list/grid entry
  // immediately. The entry is only actually removed from the
  // ListView/GridView when the remove animation finishes. The entry is added
  // to _outgoingItems when removeItem is called and removed from
  // _outgoingItems when the remove animation finishes.

  int _indexToItemIndex(int index) {
    int itemIndex = index;
    for (final _ActiveItem item in _outgoingItems) {
      if (item.itemIndex <= itemIndex) {
        itemIndex += 1;
      } else {
        break;
      }
    }
    return itemIndex;
  }

  int _itemIndexToIndex(int itemIndex) {
    int index = itemIndex;
    for (final _ActiveItem item in _outgoingItems) {
      assert(item.itemIndex != itemIndex);
      if (item.itemIndex < itemIndex) {
        index -= 1;
      } else {
        break;
      }
    }
    return index;
  }

  SliverChildDelegate _createDelegate() {
    return SliverChildBuilderDelegate(
      _itemBuilder,
      childCount: _itemsCount,
      findChildIndexCallback:
          widget.findChildIndexCallback == null
              ? null
              : (Key key) {
                final int? index = widget.findChildIndexCallback!(key);
                return index != null ? _indexToItemIndex(index) : null;
              },
    );
  }

  Widget _itemBuilder(BuildContext context, int itemIndex) {
    final _ActiveItem? outgoingItem = _activeItemAt(_outgoingItems, itemIndex);
    if (outgoingItem != null) {
      return outgoingItem.removedItemBuilder!(context, outgoingItem.controller!.view);
    }

    final _ActiveItem? incomingItem = _activeItemAt(_incomingItems, itemIndex);
    final Animation<double> animation = incomingItem?.controller?.view ?? kAlwaysCompleteAnimation;
    return widget.itemBuilder(context, _itemIndexToIndex(itemIndex), animation);
  }

  /// Insert an item at [index] and start an animation that will be passed to
  /// [SliverAnimatedGrid.itemBuilder] or [SliverAnimatedList.itemBuilder] when
  /// the item is visible.
  ///
  /// This method's semantics are the same as Dart's [List.insert] method: it
  /// increases the length of the list of items by one and shifts
  /// all items at or after [index] towards the end of the list of items.
  void insertItem(int index, {Duration duration = _kDuration}) {
    assert(index >= 0);

    final int itemIndex = _indexToItemIndex(index);
    assert(itemIndex >= 0 && itemIndex <= _itemsCount);

    // Increment the incoming and outgoing item indices to account
    // for the insertion.
    for (final _ActiveItem item in _incomingItems) {
      if (item.itemIndex >= itemIndex) {
        item.itemIndex += 1;
      }
    }
    for (final _ActiveItem item in _outgoingItems) {
      if (item.itemIndex >= itemIndex) {
        item.itemIndex += 1;
      }
    }

    final AnimationController controller = AnimationController(duration: duration, vsync: this);
    final _ActiveItem incomingItem = _ActiveItem.incoming(controller, itemIndex);
    setState(() {
      _incomingItems
        ..add(incomingItem)
        ..sort();
      _itemsCount += 1;
    });

    controller.forward().then<void>((_) {
      _removeActiveItemAt(_incomingItems, incomingItem.itemIndex)!.controller!.dispose();
    });
  }

  /// Insert multiple items at [index] and start an animation that will be passed
  /// to [AnimatedGrid.itemBuilder] or [AnimatedList.itemBuilder] when the items
  /// are visible.
  void insertAllItems(int index, int length, {Duration duration = _kDuration}) {
    for (int i = 0; i < length; i++) {
      insertItem(index + i, duration: duration);
    }
  }

  /// Remove the item at [index] and start an animation that will be passed
  /// to [builder] when the item is visible.
  ///
  /// Items are removed immediately. After an item has been removed, its index
  /// will no longer be passed to the subclass' [SliverAnimatedGrid.itemBuilder]
  /// or [SliverAnimatedList.itemBuilder]. However the item will still appear
  /// for [duration], and during that time [builder] must construct its widget
  /// as needed.
  ///
  /// This method's semantics are the same as Dart's [List.remove] method: it
  /// decreases the length of items by one and shifts
  /// all items at or before [index] towards the beginning of the list of items.
  void removeItem(int index, AnimatedRemovedItemBuilder builder, {Duration duration = _kDuration}) {
    assert(index >= 0);

    final int itemIndex = _indexToItemIndex(index);
    assert(itemIndex >= 0 && itemIndex < _itemsCount);
    assert(_activeItemAt(_outgoingItems, itemIndex) == null);

    final _ActiveItem? incomingItem = _removeActiveItemAt(_incomingItems, itemIndex);
    final AnimationController controller =
        incomingItem?.controller ??
        AnimationController(duration: duration, value: 1.0, vsync: this);
    final _ActiveItem outgoingItem = _ActiveItem.outgoing(controller, itemIndex, builder);
    setState(() {
      _outgoingItems
        ..add(outgoingItem)
        ..sort();
    });

    controller.reverse().then<void>((void value) {
      _removeActiveItemAt(_outgoingItems, outgoingItem.itemIndex)!.controller!.dispose();

      // Decrement the incoming and outgoing item indices to account
      // for the removal.
      for (final _ActiveItem item in _incomingItems) {
        if (item.itemIndex > outgoingItem.itemIndex) {
          item.itemIndex -= 1;
        }
      }
      for (final _ActiveItem item in _outgoingItems) {
        if (item.itemIndex > outgoingItem.itemIndex) {
          item.itemIndex -= 1;
        }
      }

      setState(() => _itemsCount -= 1);
    });
  }

  /// Remove all the items and start an animation that will be passed to
  /// `builder` when the items are visible.
  ///
  /// Items are removed immediately. However, the
  /// items will still appear for `duration` and during that time
  /// `builder` must construct its widget as needed.
  ///
  /// This method's semantics are the same as Dart's [List.clear] method: it
  /// removes all the items in the list.
  void removeAllItems(AnimatedRemovedItemBuilder builder, {Duration duration = _kDuration}) {
    for (int i = _itemsCount - 1; i >= 0; i--) {
      removeItem(i, builder, duration: duration);
    }
  }
}
