// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';

import 'box.dart';
import 'layer.dart';
import 'object.dart';
import 'sliver.dart';
import 'sliver_fixed_extent_list.dart';
import 'sliver_multi_box_adaptor.dart';

/// Represents the animation of the children of a parent [TreeSliverNode] that
/// are animating into or out of view.
///
/// The `fromIndex` and `toIndex` are identify the animating children following
/// the parent, with the `value` representing the status of the current
/// animation. The value of `toIndex` is inclusive, meaning the child at that
/// index is included in the animating segment.
///
/// Provided to [RenderTreeSliver] as part of
/// [RenderTreeSliver.activeAnimations] by [TreeSliver] to properly offset
/// animating children.
typedef TreeSliverNodesAnimation = ({int fromIndex, int toIndex, double value});

/// Used to pass information down to [RenderTreeSliver].
class TreeSliverNodeParentData extends SliverMultiBoxAdaptorParentData {
  /// The depth of the node, used by [RenderTreeSliver] to offset children by
  /// by the [TreeSliverIndentationType].
  int depth = 0;
}

/// The style of indentation for [TreeSliverNode]s in a [TreeSliver], as
/// handled by [RenderTreeSliver].
///
/// {@template flutter.rendering.TreeSliverIndentationType}
/// By default, the indentation is handled by [RenderTreeSliver]. Child nodes
/// are offset by the indentation specified by
/// [TreeSliverIndentationType.value] in the cross axis of the viewport. This
/// means the space allotted to the indentation will not be part of the space
/// made available to the Widget returned by [TreeSliver.treeNodeBuilder].
///
/// Alternatively, the indentation can be implemented in
/// [TreeSliver.treeNodeBuilder], with the depth of the given tree row
/// accessed by [TreeSliverNode.depth]. This allows for more customization in
/// building tree rows, such as filling the indented area with decorations or
/// ink effects.
///
/// {@tool dartpad}
/// This example shows a highly customized [TreeSliver] configured to
/// [TreeSliverIndentationType.none]. This allows the indentation to be handled
/// by the developer in [TreeSliver.treeNodeBuilder], where a decoration is
/// used to fill the indented space.
///
/// ** See code in examples/api/lib/widgets/sliver/sliver_tree.1.dart **
/// {@end-tool}
///
/// {@endtemplate}
class TreeSliverIndentationType {
  const TreeSliverIndentationType._internal(double value) : _value = value;

  /// The number of pixels by which [TreeSliverNode]s will be offset according
  /// to their [TreeSliverNode.depth].
  double get value => _value;
  final double _value;

  /// The default indentation of child [TreeSliverNode]s in a [TreeSliver].
  ///
  /// Child nodes will be offset by 10 pixels for each level in the tree.
  static const TreeSliverIndentationType standard = TreeSliverIndentationType._internal(10.0);

  /// Configures no offsetting of child nodes in a [TreeSliver].
  ///
  /// Useful if the indentation is implemented in the
  /// [TreeSliver.treeNodeBuilder] instead for more customization options.
  ///
  /// Child nodes will not be offset in the tree.
  static const TreeSliverIndentationType none = TreeSliverIndentationType._internal(0.0);

  /// Configures a custom offset for indenting child nodes in a
  /// [TreeSliver].
  ///
  /// Child nodes will be offset by the provided number of pixels in the tree.
  /// The [value] must be a non negative number.
  static TreeSliverIndentationType custom(double value) {
    assert(value >= 0.0);
    return TreeSliverIndentationType._internal(value);
  }
}

// Used during paint to delineate animating portions of the tree.
typedef _PaintSegment = ({int leadingIndex, int trailingIndex});

/// A sliver that places multiple [TreeSliverNode]s in a linear array along the
/// main access, while staggering nodes that are animating into and out of view.
///
/// The extent of each child node is determined by the [itemExtentBuilder].
///
/// See also:
///
///   * [TreeSliver], the widget that creates and manages this render
///     object.
class RenderTreeSliver extends RenderSliverVariedExtentList {
  /// Creates the render object that lays out the [TreeSliverNode]s of a
  /// [TreeSliver].
  RenderTreeSliver({
    required super.childManager,
    required super.itemExtentBuilder,
    required Map<UniqueKey, TreeSliverNodesAnimation> activeAnimations,
    required double indentation,
  }) : _activeAnimations = activeAnimations,
       _indentation = indentation;

  // TODO(Piinks): There are some opportunities to cache even further as far as
  // extents and layout offsets when using itemExtentBuilder from the super
  // class as we do here. I want to yak shave that in a separate change.

  /// The currently active [TreeSliverNode] animations.
  ///
  /// Since the index of animating nodes can change at any time, the unique key
  /// is used to track an animation of nodes across frames.
  Map<UniqueKey, TreeSliverNodesAnimation> get activeAnimations => _activeAnimations;
  Map<UniqueKey, TreeSliverNodesAnimation> _activeAnimations;
  set activeAnimations(Map<UniqueKey, TreeSliverNodesAnimation> value) {
    if (_activeAnimations == value) {
      return;
    }
    _activeAnimations = value;
    markNeedsLayout();
  }

  /// The number of pixels by which child nodes will be offset in the cross axis
  /// based on their [TreeSliverNodeParentData.depth].
  ///
  /// If zero, can alternatively offset children in
  /// [TreeSliver.treeNodeBuilder] for more options to customize the
  /// indented space.
  double get indentation => _indentation;
  double _indentation;
  set indentation(double value) {
    if (_indentation == value) {
      return;
    }
    assert(indentation >= 0.0);
    _indentation = value;
    markNeedsLayout();
  }

  // Maps the index of parents to the animation key of their children.
  final Map<int, UniqueKey> _animationLeadingIndices = <int, UniqueKey>{};
  // Maps the key of child node animations to the fixed distance they are
  // traversing during the animation. Determined at the start of the animation.
  final Map<UniqueKey, double> _animationOffsets = <UniqueKey, double>{};
  void _updateAnimationCache() {
    _animationLeadingIndices.clear();
    _activeAnimations.forEach((UniqueKey key, TreeSliverNodesAnimation animation) {
      _animationLeadingIndices[animation.fromIndex - 1] = key;
    });
    // Remove any stored offsets or clip layers that are no longer actively
    // animating.
    _animationOffsets.removeWhere((UniqueKey key, _) => !_activeAnimations.keys.contains(key));
    _clipHandles.removeWhere((UniqueKey key, LayerHandle<ClipRectLayer> handle) {
      if (!_activeAnimations.keys.contains(key)) {
        handle.layer = null;
        return true;
      }
      return false;
    });
  }

  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! TreeSliverNodeParentData) {
      child.parentData = TreeSliverNodeParentData();
    }
  }

  @override
  void dispose() {
    _clipHandles.removeWhere((UniqueKey key, LayerHandle<ClipRectLayer> handle) {
      handle.layer = null;
      return true;
    });
    super.dispose();
  }

  // TODO(Piinks): This should be made a public getter on the super class.
  // Multiple subclasses are making use of it now, yak shave that refactor
  // separately.
  late SliverLayoutDimensions _currentLayoutDimensions;

  @override
  void performLayout() {
    assert(
      constraints.axisDirection == AxisDirection.down,
      'TreeSliver is only supported in Viewports with an AxisDirection.down. '
      'The current axis direction is: ${constraints.axisDirection}.',
    );
    _updateAnimationCache();
    _currentLayoutDimensions = SliverLayoutDimensions(
      scrollOffset: constraints.scrollOffset,
      precedingScrollExtent: constraints.precedingScrollExtent,
      viewportMainAxisExtent: constraints.viewportMainAxisExtent,
      crossAxisExtent: constraints.crossAxisExtent,
    );
    super.performLayout();
  }

  @override
  int getMinChildIndexForScrollOffset(double scrollOffset, double itemExtent) {
    // itemExtent is deprecated in the super class, we ignore it because we use
    // the builder anyways.
    return _getChildIndexForScrollOffset(scrollOffset);
  }

  @override
  int getMaxChildIndexForScrollOffset(double scrollOffset, double itemExtent) {
    // itemExtent is deprecated in the super class, we ignore it because we use
    // the builder anyways.
    return _getChildIndexForScrollOffset(scrollOffset);
  }

  int _getChildIndexForScrollOffset(double scrollOffset) {
    if (scrollOffset == 0.0) {
      return 0;
    }
    double position = 0.0;
    int index = 0;
    double totalAnimationOffset = 0.0;
    double? itemExtent;
    final int? childCount = childManager.estimatedChildCount;
    while (position < scrollOffset) {
      if (childCount != null && index > childCount - 1) {
        break;
      }

      itemExtent = itemExtentBuilder(index, _currentLayoutDimensions);
      if (itemExtent == null) {
        break;
      }
      if (_animationLeadingIndices.keys.contains(index)) {
        final UniqueKey animationKey = _animationLeadingIndices[index]!;
        if (_animationOffsets[animationKey] == null) {
          // We have not computed the distance this block is traversing over the
          // lifetime of the animation.
          _computeAnimationOffsetFor(animationKey, position);
        }
        // We add the offset accounting for the animation value.
        totalAnimationOffset +=
            _animationOffsets[animationKey]! * (1 - _activeAnimations[animationKey]!.value);
      }
      position += itemExtent - totalAnimationOffset;
      ++index;
    }
    return index - 1;
  }

  void _computeAnimationOffsetFor(UniqueKey key, double position) {
    assert(_activeAnimations[key] != null);
    final double targetPosition = constraints.scrollOffset + constraints.remainingCacheExtent;
    double currentPosition = position;
    final int startingIndex = _activeAnimations[key]!.fromIndex;
    final int lastIndex = _activeAnimations[key]!.toIndex;
    int currentIndex = startingIndex;
    double totalAnimatingOffset = 0.0;
    // We animate only a portion of children that would be visible/in the cache
    // extent, unless all children would fit on the screen.
    while (currentIndex <= lastIndex && currentPosition < targetPosition) {
      final double itemExtent = itemExtentBuilder(currentIndex, _currentLayoutDimensions)!;
      totalAnimatingOffset += itemExtent;
      currentPosition += itemExtent;
      currentIndex++;
    }
    // For the life of this animation, which affects all children following
    // startingIndex (regardless of if they are a child of the triggering
    // parent), they will be offset by totalAnimatingOffset * the
    // animation value. This is because even though more children can be
    // scrolled into view, the same distance must be maintained for a smooth
    // animation.
    _animationOffsets[key] = totalAnimatingOffset;
  }

  @override
  double indexToLayoutOffset(double itemExtent, int index) {
    // itemExtent is deprecated in the super class, we ignore it because we use
    // the builder anyways.
    double position = 0.0;
    int currentIndex = 0;
    double totalAnimationOffset = 0.0;
    double? itemExtent;
    final int? childCount = childManager.estimatedChildCount;
    while (currentIndex < index) {
      if (childCount != null && currentIndex > childCount - 1) {
        break;
      }

      itemExtent = itemExtentBuilder(currentIndex, _currentLayoutDimensions);
      if (itemExtent == null) {
        break;
      }
      if (_animationLeadingIndices.keys.contains(currentIndex)) {
        final UniqueKey animationKey = _animationLeadingIndices[currentIndex]!;
        assert(_animationOffsets[animationKey] != null);
        // We add the offset accounting for the animation value.
        totalAnimationOffset +=
            _animationOffsets[animationKey]! * (1 - _activeAnimations[animationKey]!.value);
      }
      position += itemExtent;
      currentIndex++;
    }
    return position - totalAnimationOffset;
  }

  final Map<UniqueKey, LayerHandle<ClipRectLayer>> _clipHandles =
      <UniqueKey, LayerHandle<ClipRectLayer>>{};

  @override
  void paint(PaintingContext context, Offset offset) {
    if (firstChild == null) {
      return;
    }

    RenderBox? nextChild = firstChild;
    void paintUpTo(int index, RenderBox? startWith, PaintingContext context, Offset offset) {
      RenderBox? child = startWith;
      while (child != null && indexOf(child) <= index) {
        final double mainAxisDelta = childMainAxisPosition(child);
        final TreeSliverNodeParentData parentData = child.parentData! as TreeSliverNodeParentData;
        final Offset childOffset = Offset(parentData.depth * indentation, parentData.layoutOffset!);

        // If the child's visible interval (mainAxisDelta, mainAxisDelta + paintExtentOf(child))
        // does not intersect the paint extent interval (0, constraints.remainingPaintExtent), it's hidden.
        if (mainAxisDelta < constraints.remainingPaintExtent &&
            mainAxisDelta + paintExtentOf(child) > 0) {
          context.paintChild(child, childOffset);
        }
        child = childAfter(child);
      }
      nextChild = child;
    }

    if (_animationLeadingIndices.isEmpty) {
      // There are no animations running.
      paintUpTo(indexOf(lastChild!), firstChild, context, offset);
      return;
    }

    // We are animating.
    // Separate animating segments to clip for any overlap.
    int leadingIndex = indexOf(firstChild!);
    final List<int> animationIndices = _animationLeadingIndices.keys.toList()..sort();
    final List<_PaintSegment> paintSegments = <_PaintSegment>[];
    while (animationIndices.isNotEmpty) {
      final int trailingIndex = animationIndices.removeAt(0);
      paintSegments.add((leadingIndex: leadingIndex, trailingIndex: trailingIndex));
      leadingIndex = trailingIndex + 1;
    }
    paintSegments.add((leadingIndex: leadingIndex, trailingIndex: indexOf(lastChild!)));

    // Paint, clipping for all but the first segment.
    paintUpTo(paintSegments.removeAt(0).trailingIndex, nextChild, context, offset);
    // Paint the rest with clip layers.
    while (paintSegments.isNotEmpty) {
      final _PaintSegment segment = paintSegments.removeAt(0);

      // Rect is calculated by the trailing edge of the parent (preceding
      // leadingIndex), and the trailing edge of the trailing index. We cannot
      // rely on the leading edge of the leading index, because it is currently
      // moving.
      final int parentIndex = math.max(segment.leadingIndex - 1, 0);
      final double leadingOffset =
          indexToLayoutOffset(0.0, parentIndex) +
          (parentIndex == 0 ? 0.0 : itemExtentBuilder(parentIndex, _currentLayoutDimensions)!);
      final double trailingOffset =
          indexToLayoutOffset(0.0, segment.trailingIndex) +
          itemExtentBuilder(segment.trailingIndex, _currentLayoutDimensions)!;
      final Rect rect = Rect.fromPoints(
        Offset(0.0, leadingOffset),
        Offset(constraints.crossAxisExtent, trailingOffset),
      );
      // We use the same animation key to keep track of the clip layer, unless
      // this is the odd man out segment.
      final UniqueKey key = _animationLeadingIndices[parentIndex]!;
      _clipHandles[key] ??= LayerHandle<ClipRectLayer>();
      _clipHandles[key]!.layer = context.pushClipRect(needsCompositing, offset, rect, (
        PaintingContext context,
        Offset offset,
      ) {
        paintUpTo(segment.trailingIndex, nextChild, context, offset);
      }, oldLayer: _clipHandles[key]!.layer);
    }
  }
}
