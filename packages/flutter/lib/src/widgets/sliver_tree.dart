// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'viewport.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'framework.dart';
import 'gesture_detector.dart';
import 'icon.dart';
import 'icon_data.dart';
import 'implicit_animations.dart';
import 'scroll_delegate.dart';
import 'sliver.dart';
import 'text.dart';
import 'ticker_provider.dart';

const double _kDefaultRowExtent = 40.0;

/// A data structure for configuring children of a [TreeSliver].
///
/// A [TreeSliverNode.content] can be of any type [T], but must correspond with
/// the same type of the [TreeSliver].
///
/// The values returned by [depth], [parent] and [isExpanded] getters are
/// managed by the [TreeSliver]'s state.
class TreeSliverNode<T> {
  /// Creates a [TreeSliverNode] instance for use in a [TreeSliver].
  TreeSliverNode(T content, {List<TreeSliverNode<T>>? children, bool expanded = false})
    : _expanded = (children?.isNotEmpty ?? false) && expanded,
      _content = content,
      _children = children ?? <TreeSliverNode<T>>[];

  /// The subject matter of the node.
  ///
  /// Must correspond with the type of [TreeSliver].
  T get content => _content;
  final T _content;

  /// Other [TreeSliverNode]s that this node will be [parent] to.
  ///
  /// Modifying the children of nodes in a [TreeSliver] will cause the tree to be
  /// rebuilt so that newly added active nodes are reflected in the tree.
  List<TreeSliverNode<T>> get children => _children;
  final List<TreeSliverNode<T>> _children;

  /// Whether or not this node is expanded in the tree.
  ///
  /// Cannot be expanded if there are no children.
  bool get isExpanded => _expanded;
  bool _expanded;

  /// The number of parent nodes between this node and the root of the tree.
  int? get depth => _depth;
  int? _depth;

  /// The parent [TreeSliverNode] of this node.
  TreeSliverNode<T>? get parent => _parent;
  TreeSliverNode<T>? _parent;

  @override
  String toString() {
    return 'TreeSliverNode: $content, depth: ${depth == 0 ? 'root' : depth}, '
        '${children.isEmpty ? 'leaf' : 'parent, expanded: $isExpanded'}';
  }
}

/// Signature for a function that creates a [Widget] to represent the given
/// [TreeSliverNode] in the [TreeSliver].
///
/// Used by [TreeSliver.treeNodeBuilder] to build rows on demand for the
/// tree.
typedef TreeSliverNodeBuilder =
    Widget Function(
      BuildContext context,
      TreeSliverNode<Object?> node,
      AnimationStyle animationStyle,
    );

/// Signature for a function that returns an extent for the given
/// [TreeSliverNode] in the [TreeSliver].
///
/// Used by [TreeSliver.treeRowExtentBuilder] to size rows on demand in the
/// tree. The provided [SliverLayoutDimensions] provide information about the
/// current scroll state and [Viewport] dimensions.
///
/// See also:
///
///   * [SliverVariedExtentList], which uses a similar item extent builder for
///     dynamic child sizing in the list.
typedef TreeSliverRowExtentBuilder =
    double Function(TreeSliverNode<Object?> node, SliverLayoutDimensions dimensions);

/// Signature for a function that is called when a [TreeSliverNode] is toggled,
/// changing its expanded state.
///
/// See also:
///
///   * [TreeSliver.onNodeToggle], for controlling node expansion
///     programmatically.
typedef TreeSliverNodeCallback = void Function(TreeSliverNode<Object?> node);

/// A mixin for classes implementing a tree structure as expected by a
/// [TreeSliverController].
///
/// Used by [TreeSliver] to implement an interface for the
/// [TreeSliverController].
///
/// This allows the [TreeSliverController] to be used in other widgets that
/// implement this interface.
///
/// The type [T] correlates to the type of [TreeSliver] and [TreeSliverNode],
/// representing the type of [TreeSliverNode.content].
mixin TreeSliverStateMixin<T> {
  /// Returns whether or not the given [TreeSliverNode] is expanded.
  bool isExpanded(TreeSliverNode<T> node);

  /// Returns whether or not the given [TreeSliverNode] is enclosed within its
  /// parent [TreeSliverNode].
  ///
  /// If the [TreeSliverNode.parent] [isExpanded] (and all its parents are
  /// expanded), or this is a root node, the given node is active and this
  /// method will return true. This does not reflect whether or not the node is
  /// visible in the [Viewport].
  bool isActive(TreeSliverNode<T> node);

  /// Switches the given [TreeSliverNode]s expanded state.
  ///
  /// May trigger an animation to reveal or hide the node's children based on
  /// the [TreeSliver.toggleAnimationStyle].
  ///
  /// If the node does not have any children, nothing will happen.
  void toggleNode(TreeSliverNode<T> node);

  /// Closes all parent [TreeSliverNode]s in the tree.
  void collapseAll();

  /// Expands all parent [TreeSliverNode]s in the tree.
  void expandAll();

  /// Retrieves the [TreeSliverNode] containing the associated content, if it
  /// exists.
  ///
  /// If no node exists, this will return null. This does not reflect whether
  /// or not a node [isActive], or if it is visible in the viewport.
  TreeSliverNode<T>? getNodeFor(T content);

  /// Returns the current row index of the given [TreeSliverNode].
  ///
  /// If the node is not currently active in the tree, meaning its parent is
  /// collapsed, this will return null.
  int? getActiveIndexFor(TreeSliverNode<T> node);
}

/// Enables control over the [TreeSliverNode]s of a [TreeSliver].
///
/// It can be useful to expand or collapse nodes of the tree
/// programmatically, for example to reconfigure an existing node
/// based on a system event. To do so, create a [TreeSliver]
/// with a [TreeSliverController] that's owned by a stateful widget
/// or look up the tree's automatically created [TreeSliverController]
/// with [TreeSliverController.of]
///
/// The controller's methods to expand or collapse nodes cause the
/// the [TreeSliver] to rebuild, so they may not be called from
/// a build method.
class TreeSliverController {
  /// Create a controller to be used with [TreeSliver.controller].
  TreeSliverController();

  TreeSliverStateMixin<Object?>? _state;

  /// Whether the given [TreeSliverNode] built with this controller is in an
  /// expanded state.
  ///
  /// See also:
  ///
  ///  * [expandNode], which expands a given [TreeSliverNode].
  ///  * [collapseNode], which collapses a given [TreeSliverNode].
  ///  * [TreeSliver.controller] to create a TreeSliver with a controller.
  bool isExpanded(TreeSliverNode<Object?> node) {
    assert(_state != null);
    return _state!.isExpanded(node);
  }

  /// Whether or not the given [TreeSliverNode] is enclosed within its parent
  /// [TreeSliverNode].
  ///
  /// If the [TreeSliverNode.parent] [isExpanded], or this is a root node, the
  /// given node is active and this method will return true. This does not
  /// reflect whether or not the node is visible in the [Viewport].
  bool isActive(TreeSliverNode<Object?> node) {
    assert(_state != null);
    return _state!.isActive(node);
  }

  /// Returns the [TreeSliverNode] containing the associated content, if it
  /// exists.
  ///
  /// If no node exists, this will return null. This does not reflect whether
  /// or not a node [isActive], or if it is currently visible in the viewport.
  TreeSliverNode<Object?>? getNodeFor(Object? content) {
    assert(_state != null);
    return _state!.getNodeFor(content);
  }

  /// Switches the given [TreeSliverNode]s expanded state.
  ///
  /// May trigger an animation to reveal or hide the node's children based on
  /// the [TreeSliver.toggleAnimationStyle].
  ///
  /// If the node does not have any children, nothing will happen.
  void toggleNode(TreeSliverNode<Object?> node) {
    assert(_state != null);
    return _state!.toggleNode(node);
  }

  /// Expands the [TreeSliverNode] that was built with this controller.
  ///
  /// If the node is already in the expanded state (see [isExpanded]), calling
  /// this method has no effect.
  ///
  /// Calling this method may cause the [TreeSliver] to rebuild, so it may
  /// not be called from a build method.
  ///
  /// Calling this method will trigger the [TreeSliver.onNodeToggle]
  /// callback.
  ///
  /// See also:
  ///
  ///  * [collapseNode], which collapses the [TreeSliverNode].
  ///  * [isExpanded] to check whether the tile is expanded.
  ///  * [TreeSliver.controller] to create a TreeSliver with a controller.
  void expandNode(TreeSliverNode<Object?> node) {
    assert(_state != null);
    if (!node.isExpanded) {
      _state!.toggleNode(node);
    }
  }

  /// Expands all parent [TreeSliverNode]s in the tree.
  void expandAll() {
    assert(_state != null);
    _state!.expandAll();
  }

  /// Closes all parent [TreeSliverNode]s in the tree.
  void collapseAll() {
    assert(_state != null);
    _state!.collapseAll();
  }

  /// Collapses the [TreeSliverNode] that was built with this controller.
  ///
  /// If the node is already in the collapsed state (see [isExpanded]), calling
  /// this method has no effect.
  ///
  /// Calling this method may cause the [TreeSliver] to rebuild, so it may
  /// not be called from a build method.
  ///
  /// Calling this method will trigger the [TreeSliver.onNodeToggle]
  /// callback.
  ///
  /// See also:
  ///
  ///  * [expandNode], which expands the tile.
  ///  * [isExpanded] to check whether the tile is expanded.
  ///  * [TreeSliver.controller] to create a TreeSliver with a controller.
  void collapseNode(TreeSliverNode<Object?> node) {
    assert(_state != null);
    if (node.isExpanded) {
      _state!.toggleNode(node);
    }
  }

  /// Returns the current row index of the given [TreeSliverNode].
  ///
  /// If the node is not currently active in the tree, meaning its parent is
  /// collapsed, this will return null.
  int? getActiveIndexFor(TreeSliverNode<Object?> node) {
    assert(_state != null);
    return _state!.getActiveIndexFor(node);
  }

  /// Finds the [TreeSliverController] for the closest [TreeSliver] instance
  /// that encloses the given context.
  ///
  /// If no [TreeSliver] encloses the given context, calling this
  /// method will cause an assert in debug mode, and throw an
  /// exception in release mode.
  ///
  /// To return null if there is no [TreeSliver] use [maybeOf] instead.
  ///
  /// Typical usage of the [TreeSliverController.of] function is to call it
  /// from within the `build` method of a descendant of a [TreeSliver].
  ///
  /// When the [TreeSliver] is actually created in the same `build`
  /// function as the callback that refers to the controller, then the
  /// `context` argument to the `build` function can't be used to find
  /// the [TreeSliverController] (since it's "above" the widget
  /// being returned in the widget tree). In cases like that you can
  /// add a [Builder] widget, which provides a new scope with a
  /// [BuildContext] that is "under" the [TreeSliver].
  static TreeSliverController of(BuildContext context) {
    final _TreeSliverState<Object?>? result =
        context.findAncestorStateOfType<_TreeSliverState<Object?>>();
    if (result != null) {
      return result.controller;
    }
    throw FlutterError.fromParts(<DiagnosticsNode>[
      ErrorSummary(
        'TreeController.of() called with a context that does not contain a '
        'TreeSliver.',
      ),
      ErrorDescription(
        'No TreeSliver ancestor could be found starting from the context that '
        'was passed to TreeController.of(). '
        'This usually happens when the context provided is from the same '
        'StatefulWidget as that whose build function actually creates the '
        'TreeSliver widget being sought.',
      ),
      ErrorHint(
        'There are several ways to avoid this problem. The simplest is to use '
        'a Builder to get a context that is "under" the TreeSliver.',
      ),
      ErrorHint(
        'A more efficient solution is to split your build function into '
        'several widgets. This introduces a new context from which you can '
        'obtain the TreeSliver. In this solution, you would have an outer '
        'widget that creates the TreeSliver populated by instances of your new '
        'inner widgets, and then in these inner widgets you would use '
        'TreeController.of().',
      ),
      context.describeElement('The context used was'),
    ]);
  }

  /// Finds the [TreeSliver] from the closest instance of this class that
  /// encloses the given context and returns its [TreeSliverController].
  ///
  /// If no [TreeSliver] encloses the given context then return null.
  /// To throw an exception instead, use [of] instead of this function.
  ///
  /// See also:
  ///
  ///  * [of], a similar function to this one that throws if no [TreeSliver]
  ///    encloses the given context. Also includes some sample code in its
  ///    documentation.
  static TreeSliverController? maybeOf(BuildContext context) {
    return context.findAncestorStateOfType<_TreeSliverState<Object?>>()?.controller;
  }
}

int _kDefaultSemanticIndexCallback(Widget _, int localIndex) => localIndex;

/// A widget that displays [TreeSliverNode]s that expand and collapse in a
/// vertically and horizontally scrolling [Viewport].
///
/// The type [T] correlates to the type of [TreeSliver] and [TreeSliverNode],
/// representing the type of [TreeSliverNode.content].
///
/// The rows of the tree are laid out on demand by the [Viewport]'s render
/// object, using [TreeSliver.treeNodeBuilder]. This will only be called for the
/// nodes that are visible, or within the [Viewport.cacheExtent].
///
/// The [TreeSliver.treeNodeBuilder] returns the [Widget] that represents the
/// given [TreeSliverNode].
///
/// The [TreeSliver.treeRowExtentBuilder] returns a double representing the
/// extent of a given node in the main axis.
///
/// Providing a [TreeSliverController] will enable querying and controlling the
/// state of nodes in the tree.
///
/// A [TreeSliver] only supports a vertical axis direction of
/// [AxisDirection.down] and a horizontal axis direction of
/// [AxisDirection.right].
///
///{@tool dartpad}
/// This example uses a [TreeSliver] to display nodes, highlighting nodes as
/// they are selected.
///
/// ** See code in examples/api/lib/widgets/sliver/sliver_tree.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows a highly customized [TreeSliver] configured to
/// [TreeSliverIndentationType.none]. This allows the indentation to be handled
/// by the developer in [TreeSliver.treeNodeBuilder], where a decoration is
/// used to fill the indented space.
///
/// ** See code in examples/api/lib/widgets/sliver/sliver_tree.1.dart **
/// {@end-tool}
class TreeSliver<T> extends StatefulWidget {
  /// Creates an instance of a [TreeSliver] for displaying [TreeSliverNode]s
  /// that animate expanding and collapsing of nodes.
  const TreeSliver({
    super.key,
    required this.tree,
    this.treeNodeBuilder = TreeSliver.defaultTreeNodeBuilder,
    this.treeRowExtentBuilder = TreeSliver.defaultTreeRowExtentBuilder,
    this.controller,
    this.onNodeToggle,
    this.toggleAnimationStyle,
    this.indentation = TreeSliverIndentationType.standard,
    this.addAutomaticKeepAlives = true,
    this.addRepaintBoundaries = true,
    this.addSemanticIndexes = true,
    this.semanticIndexCallback = _kDefaultSemanticIndexCallback,
    this.semanticIndexOffset = 0,
    this.findChildIndexCallback,
  });

  /// The list of [TreeSliverNode]s that may be displayed in the [TreeSliver].
  ///
  /// Beyond root nodes, whether or not a given [TreeSliverNode] is displayed
  /// depends on the [TreeSliverNode.isExpanded] value of its parent. The
  /// [TreeSliver] will set the [TreeSliverNode.parent] and
  /// [TreeSliverNode.depth] as nodes are built on demand to ensure the
  /// integrity of the tree.
  final List<TreeSliverNode<T>> tree;

  /// Called to build and entry of the [TreeSliver] for the given node.
  ///
  /// By default, if this is unset, the [TreeSliver.defaultTreeNodeBuilder]
  /// is used.
  final TreeSliverNodeBuilder treeNodeBuilder;

  /// Called to calculate the extent of the widget built for the given
  /// [TreeSliverNode].
  ///
  /// By default, if this is unset, the
  /// [TreeSliver.defaultTreeRowExtentBuilder] is used.
  ///
  /// See also:
  ///
  ///   * [SliverVariedExtentList.itemExtentBuilder], a very similar method that
  ///     allows users to dynamically compute extents on demand.
  final TreeSliverRowExtentBuilder treeRowExtentBuilder;

  /// If provided, the controller can be used to expand and collapse
  /// [TreeSliverNode]s, or lookup information about the current state of the
  /// [TreeSliver].
  final TreeSliverController? controller;

  /// Called when a [TreeSliverNode] expands or collapses.
  ///
  /// This will not be called if a [TreeSliverNode] does not have any children.
  final TreeSliverNodeCallback? onNodeToggle;

  /// The default [AnimationStyle] for expanding and collapsing nodes in the
  /// [TreeSliver].
  ///
  /// The default [AnimationStyle.duration] uses
  /// [TreeSliver.defaultAnimationDuration], which is 150 milliseconds.
  ///
  /// The default [AnimationStyle.curve] uses [TreeSliver.defaultAnimationCurve],
  /// which is [Curves.linear].
  ///
  /// To disable the tree animation, use [AnimationStyle.noAnimation].
  final AnimationStyle? toggleAnimationStyle;

  /// The number of pixels children will be offset by in the cross axis based on
  /// their [TreeSliverNode.depth].
  ///
  /// {@macro flutter.rendering.TreeSliverIndentationType}
  final TreeSliverIndentationType indentation;

  /// {@macro flutter.widgets.SliverChildBuilderDelegate.addAutomaticKeepAlives}
  final bool addAutomaticKeepAlives;

  /// {@macro flutter.widgets.SliverChildBuilderDelegate.addRepaintBoundaries}
  final bool addRepaintBoundaries;

  /// {@macro flutter.widgets.SliverChildBuilderDelegate.addSemanticIndexes}
  final bool addSemanticIndexes;

  /// {@macro flutter.widgets.SliverChildBuilderDelegate.semanticIndexCallback}
  final SemanticIndexCallback semanticIndexCallback;

  /// {@macro flutter.widgets.SliverChildBuilderDelegate.semanticIndexOffset}
  final int semanticIndexOffset;

  /// {@macro flutter.widgets.SliverChildBuilderDelegate.findChildIndexCallback}
  final int? Function(Key)? findChildIndexCallback;

  /// The default [AnimationStyle] used for node expand and collapse animations,
  /// when one has not been provided in [toggleAnimationStyle].
  static AnimationStyle defaultToggleAnimationStyle = AnimationStyle(
    curve: defaultAnimationCurve,
    duration: defaultAnimationDuration,
  );

  /// A default of [Curves.linear], which is used in the tree's expanding and
  /// collapsing node animation.
  static const Curve defaultAnimationCurve = Curves.linear;

  /// A default [Duration] of 150 milliseconds, which is used in the tree's
  /// expanding and collapsing node animation.
  static const Duration defaultAnimationDuration = Duration(milliseconds: 150);

  /// A wrapper method for triggering the expansion or collapse of a
  /// [TreeSliverNode].
  ///
  /// Used as part of [TreeSliver.defaultTreeNodeBuilder] to wrap the leading
  /// icon of parent [TreeSliverNode]s such that tapping on it triggers the
  /// animation.
  ///
  /// If defining your own [TreeSliver.treeNodeBuilder], this method can be used
  /// to wrap any part, or all, of the returned widget in order to trigger the
  /// change in state for the node.
  static Widget wrapChildToToggleNode({
    required TreeSliverNode<Object?> node,
    required Widget child,
  }) {
    return Builder(
      builder: (BuildContext context) {
        return GestureDetector(
          onTap: () {
            TreeSliverController.of(context).toggleNode(node);
          },
          child: child,
        );
      },
    );
  }

  /// Returns the fixed default extent for rows in the tree, which is 40 pixels.
  ///
  /// Used by [TreeSliver.treeRowExtentBuilder].
  static double defaultTreeRowExtentBuilder(
    TreeSliverNode<Object?> node,
    SliverLayoutDimensions dimensions,
  ) {
    return _kDefaultRowExtent;
  }

  /// Returns the default tree row for a given [TreeSliverNode].
  ///
  /// Used by [TreeSliver.treeNodeBuilder].
  ///
  /// This will return a [Row] containing the [toString] of
  /// [TreeSliverNode.content]. If the [TreeSliverNode] is a parent of
  /// additional nodes, a arrow icon will precede the content, and will trigger
  /// an expand and collapse animation when tapped.
  static Widget defaultTreeNodeBuilder(
    BuildContext context,
    TreeSliverNode<Object?> node,
    AnimationStyle toggleAnimationStyle,
  ) {
    final Duration animationDuration =
        toggleAnimationStyle.duration ?? TreeSliver.defaultAnimationDuration;
    final Curve animationCurve = toggleAnimationStyle.curve ?? TreeSliver.defaultAnimationCurve;
    final int index = TreeSliverController.of(context).getActiveIndexFor(node)!;
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: Row(
        children: <Widget>[
          // Icon for parent nodes
          TreeSliver.wrapChildToToggleNode(
            node: node,
            child: SizedBox.square(
              dimension: 30.0,
              child:
                  node.children.isNotEmpty
                      ? AnimatedRotation(
                        key: ValueKey<int>(index),
                        turns: node.isExpanded ? 0.25 : 0.0,
                        duration: animationDuration,
                        curve: animationCurve,
                        // Renders a unicode right-facing arrow. >
                        child: const Icon(IconData(0x25BA), size: 14),
                      )
                      : null,
            ),
          ),
          // Spacer
          const SizedBox(width: 8.0),
          // Content
          Text(node.content.toString()),
        ],
      ),
    );
  }

  @override
  State<TreeSliver<T>> createState() => _TreeSliverState<T>();
}

// Used in _SliverTreeState for code simplicity.
typedef _AnimationRecord =
    ({AnimationController controller, CurvedAnimation animation, UniqueKey key});

class _TreeSliverState<T> extends State<TreeSliver<T>>
    with TickerProviderStateMixin, TreeSliverStateMixin<T> {
  TreeSliverController get controller => _treeController!;
  TreeSliverController? _treeController;

  final List<TreeSliverNode<T>> _activeNodes = <TreeSliverNode<T>>[];
  bool _shouldUnpackNode(TreeSliverNode<T> node) {
    if (node.children.isEmpty) {
      // No children to unpack.
      return false;
    }
    if (_currentAnimationForParent[node] != null) {
      // Whether expanding or collapsing, the child nodes are still active, so
      // unpack.
      return true;
    }
    // If we are not animating, respect node.isExpanded.
    return node.isExpanded;
  }

  void _unpackActiveNodes({
    int depth = 0,
    List<TreeSliverNode<T>>? nodes,
    TreeSliverNode<T>? parent,
  }) {
    if (nodes == null) {
      _activeNodes.clear();
      nodes = widget.tree;
    }
    for (final TreeSliverNode<T> node in nodes) {
      node._depth = depth;
      node._parent = parent;
      _activeNodes.add(node);
      if (_shouldUnpackNode(node)) {
        _unpackActiveNodes(depth: depth + 1, nodes: node.children, parent: node);
      }
    }
  }

  final Map<TreeSliverNode<T>, _AnimationRecord> _currentAnimationForParent =
      <TreeSliverNode<T>, _AnimationRecord>{};
  final Map<UniqueKey, TreeSliverNodesAnimation> _activeAnimations =
      <UniqueKey, TreeSliverNodesAnimation>{};

  @override
  void initState() {
    _unpackActiveNodes();
    assert(
      widget.controller?._state == null,
      'The provided TreeSliverController is already associated with another '
      'TreeSliver. A TreeSliverController can only be associated with one '
      'TreeSliver.',
    );
    _treeController = widget.controller ?? TreeSliverController();
    _treeController!._state = this;
    super.initState();
  }

  @override
  void didUpdateWidget(TreeSliver<T> oldWidget) {
    super.didUpdateWidget(oldWidget);
    // Internal or provided, there is always a tree controller.
    assert(_treeController != null);
    if (oldWidget.controller == null && widget.controller != null) {
      // A new tree controller has been provided, update and dispose of the
      // internally generated one.
      _treeController!._state = null;
      _treeController = widget.controller;
      _treeController!._state = this;
    } else if (oldWidget.controller != null && widget.controller == null) {
      // A tree controller had been provided, but was removed. We need to create
      // one internally.
      assert(oldWidget.controller == _treeController);
      oldWidget.controller!._state = null;
      _treeController = TreeSliverController();
      _treeController!._state = this;
    } else if (oldWidget.controller != widget.controller) {
      assert(oldWidget.controller != null);
      assert(widget.controller != null);
      assert(oldWidget.controller == _treeController);
      // The tree is still being provided a controller, but it has changed. Just
      // update it.
      _treeController!._state = null;
      _treeController = widget.controller;
      _treeController!._state = this;
    }
    // Internal or provided, there is always a tree controller.
    assert(_treeController != null);
    assert(_treeController!._state != null);
    _unpackActiveNodes();
  }

  @override
  void dispose() {
    _treeController!._state = null;
    for (final _AnimationRecord record in _currentAnimationForParent.values) {
      record.animation.dispose();
      record.controller.dispose();
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return _SliverTree(
      itemCount: _activeNodes.length,
      activeAnimations: _activeAnimations,
      itemBuilder: (BuildContext context, int index) {
        final TreeSliverNode<T> node = _activeNodes[index];
        Widget child = widget.treeNodeBuilder(
          context,
          node,
          widget.toggleAnimationStyle ?? TreeSliver.defaultToggleAnimationStyle,
        );

        if (widget.addRepaintBoundaries) {
          child = RepaintBoundary(child: child);
        }
        if (widget.addSemanticIndexes) {
          final int? semanticIndex = widget.semanticIndexCallback(child, index);
          if (semanticIndex != null) {
            child = IndexedSemantics(
              index: semanticIndex + widget.semanticIndexOffset,
              child: child,
            );
          }
        }

        return _TreeNodeParentDataWidget(depth: node.depth!, child: child);
      },
      itemExtentBuilder: (int index, SliverLayoutDimensions dimensions) {
        return widget.treeRowExtentBuilder(_activeNodes[index], dimensions);
      },
      addAutomaticKeepAlives: widget.addAutomaticKeepAlives,
      findChildIndexCallback: widget.findChildIndexCallback,
      indentation: widget.indentation.value,
    );
  }

  // TreeStateMixin Implementation

  @override
  bool isExpanded(TreeSliverNode<T> node) {
    return _getNode(node.content, widget.tree)?.isExpanded ?? false;
  }

  @override
  bool isActive(TreeSliverNode<T> node) => _activeNodes.contains(node);

  @override
  TreeSliverNode<T>? getNodeFor(T content) => _getNode(content, widget.tree);
  TreeSliverNode<T>? _getNode(T content, List<TreeSliverNode<T>> tree) {
    final List<TreeSliverNode<T>> nextDepth = <TreeSliverNode<T>>[];
    for (final TreeSliverNode<T> node in tree) {
      if (node.content == content) {
        return node;
      }
      if (node.children.isNotEmpty) {
        nextDepth.addAll(node.children);
      }
    }
    if (nextDepth.isNotEmpty) {
      return _getNode(content, nextDepth);
    }
    return null;
  }

  @override
  int? getActiveIndexFor(TreeSliverNode<T> node) {
    if (_activeNodes.contains(node)) {
      return _activeNodes.indexOf(node);
    }
    return null;
  }

  @override
  void expandAll() {
    final List<TreeSliverNode<T>> activeNodesToExpand = <TreeSliverNode<T>>[];
    _expandAll(widget.tree, activeNodesToExpand);
    activeNodesToExpand.reversed.forEach(toggleNode);
  }

  void _expandAll(List<TreeSliverNode<T>> tree, List<TreeSliverNode<T>> activeNodesToExpand) {
    for (final TreeSliverNode<T> node in tree) {
      if (node.children.isNotEmpty) {
        // This is a parent node.
        // Expand all the children, and their children.
        _expandAll(node.children, activeNodesToExpand);
        if (!node.isExpanded) {
          // The node itself needs to be expanded.
          if (_activeNodes.contains(node)) {
            // This is an active node in the tree, add to
            // the list to toggle once all hidden nodes
            // have been handled.
            activeNodesToExpand.add(node);
          } else {
            // This is a hidden node. Update its expanded state.
            node._expanded = true;
          }
        }
      }
    }
  }

  @override
  void collapseAll() {
    final List<TreeSliverNode<T>> activeNodesToCollapse = <TreeSliverNode<T>>[];
    _collapseAll(widget.tree, activeNodesToCollapse);
    activeNodesToCollapse.reversed.forEach(toggleNode);
  }

  void _collapseAll(List<TreeSliverNode<T>> tree, List<TreeSliverNode<T>> activeNodesToCollapse) {
    for (final TreeSliverNode<T> node in tree) {
      if (node.children.isNotEmpty) {
        // This is a parent node.
        // Collapse all the children, and their children.
        _collapseAll(node.children, activeNodesToCollapse);
        if (node.isExpanded) {
          // The node itself needs to be collapsed.
          if (_activeNodes.contains(node)) {
            // This is an active node in the tree, add to
            // the list to toggle once all hidden nodes
            // have been handled.
            activeNodesToCollapse.add(node);
          } else {
            // This is a hidden node. Update its expanded state.
            node._expanded = false;
          }
        }
      }
    }
  }

  void _updateActiveAnimations() {
    // The indexes of various child node animations can change constantly based
    // on more nodes being expanded or collapsed. Compile the indexes and their
    // animations keys each time we build with an updated active node list.
    _activeAnimations.clear();
    for (final TreeSliverNode<T> node in _currentAnimationForParent.keys) {
      final _AnimationRecord animationRecord = _currentAnimationForParent[node]!;
      final int leadingChildIndex = _activeNodes.indexOf(node) + 1;
      final TreeSliverNodesAnimation animatingChildren = (
        fromIndex: leadingChildIndex,
        toIndex: leadingChildIndex + node.children.length - 1,
        value: animationRecord.animation.value,
      );
      _activeAnimations[animationRecord.key] = animatingChildren;
    }
  }

  @override
  void toggleNode(TreeSliverNode<T> node) {
    assert(_activeNodes.contains(node));
    if (node.children.isEmpty) {
      // No state to change.
      return;
    }

    setState(() {
      node._expanded = !node._expanded;
      if (widget.onNodeToggle != null) {
        widget.onNodeToggle!(node);
      }
      if (_currentAnimationForParent[node] != null) {
        // Dispose of the old animation if this node was already animating.
        _currentAnimationForParent[node]!.animation.dispose();
      }

      // If animation is disabled or the duration is zero, we skip the animation
      // and immediately update the active nodes. This prevents the app from freezing
      // due to the tree being incorrectly updated when the animation duration is zero.
      // This is because, in this case, the node's children are no longer active.
      if (widget.toggleAnimationStyle == AnimationStyle.noAnimation ||
          widget.toggleAnimationStyle?.duration == Duration.zero) {
        _unpackActiveNodes();
        return;
      }

      final AnimationController controller =
          _currentAnimationForParent[node]?.controller ??
                AnimationController(
                  value: node._expanded ? 0.0 : 1.0,
                  vsync: this,
                  duration:
                      widget.toggleAnimationStyle?.duration ?? TreeSliver.defaultAnimationDuration,
                )
            ..addStatusListener((AnimationStatus status) {
              switch (status) {
                case AnimationStatus.dismissed:
                case AnimationStatus.completed:
                  _currentAnimationForParent[node]!.animation.dispose();
                  _currentAnimationForParent[node]!.controller.dispose();
                  _currentAnimationForParent.remove(node);
                  _updateActiveAnimations();
                  // If the node is collapsing, we need to unpack the active
                  // nodes to remove the ones that were removed from the tree.
                  // This is only necessary if the node is collapsing.
                  if (!node._expanded) {
                    _unpackActiveNodes();
                  }
                case AnimationStatus.forward:
                case AnimationStatus.reverse:
              }
            })
            ..addListener(() {
              setState(() {
                _updateActiveAnimations();
              });
            });

      switch (controller.status) {
        case AnimationStatus.forward:
        case AnimationStatus.reverse:
          // We're interrupting an animation already in progress.
          controller.stop();
        case AnimationStatus.dismissed:
        case AnimationStatus.completed:
      }

      final CurvedAnimation newAnimation = CurvedAnimation(
        parent: controller,
        curve: widget.toggleAnimationStyle?.curve ?? TreeSliver.defaultAnimationCurve,
      );
      _currentAnimationForParent[node] = (
        controller: controller,
        animation: newAnimation,
        // This key helps us keep track of the lifetime of this animation in the
        // render object, since the indexes can change at any time.
        key: UniqueKey(),
      );
      switch (node._expanded) {
        case true:
          // Expanding
          _unpackActiveNodes();
          controller.forward();
        case false:
          // Collapsing
          controller.reverse();
      }
    });
  }
}

class _TreeNodeParentDataWidget extends ParentDataWidget<TreeSliverNodeParentData> {
  const _TreeNodeParentDataWidget({required this.depth, required super.child}) : assert(depth >= 0);

  final int depth;

  @override
  void applyParentData(RenderObject renderObject) {
    final TreeSliverNodeParentData parentData =
        renderObject.parentData! as TreeSliverNodeParentData;
    bool needsLayout = false;

    if (parentData.depth != depth) {
      assert(depth >= 0);
      parentData.depth = depth;
      needsLayout = true;
    }

    if (needsLayout) {
      renderObject.parent?.markNeedsLayout();
    }
  }

  @override
  Type get debugTypicalAncestorWidgetClass => _SliverTree;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(IntProperty('depth', depth));
  }
}

class _SliverTree extends SliverVariedExtentList {
  _SliverTree({
    required NullableIndexedWidgetBuilder itemBuilder,
    required super.itemExtentBuilder,
    required this.activeAnimations,
    required this.indentation,
    ChildIndexGetter? findChildIndexCallback,
    required int itemCount,
    bool addAutomaticKeepAlives = true,
  }) : super(
         delegate: SliverChildBuilderDelegate(
           itemBuilder,
           findChildIndexCallback: findChildIndexCallback,
           childCount: itemCount,
           addAutomaticKeepAlives: addAutomaticKeepAlives,
           addRepaintBoundaries: false, // Added in the _SliverTreeState
           addSemanticIndexes: false, // Added in the _SliverTreeState
         ),
       );

  final Map<UniqueKey, TreeSliverNodesAnimation> activeAnimations;
  final double indentation;

  @override
  RenderTreeSliver createRenderObject(BuildContext context) {
    final SliverMultiBoxAdaptorElement element = context as SliverMultiBoxAdaptorElement;
    return RenderTreeSliver(
      itemExtentBuilder: itemExtentBuilder,
      activeAnimations: activeAnimations,
      indentation: indentation,
      childManager: element,
    );
  }

  @override
  void updateRenderObject(BuildContext context, RenderTreeSliver renderObject) {
    renderObject
      ..itemExtentBuilder = itemExtentBuilder
      ..activeAnimations = activeAnimations
      ..indentation = indentation;
  }
}
