// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/semantics.dart';
import 'package:flutter/widgets.dart';

/// Provides an iterable that efficiently returns all the [Element]s
/// rooted at the given [Element]. See [CachingIterable] for details.
///
/// This function must be called again if the tree changes. You cannot
/// call this function once, then reuse the iterable after having
/// changed the state of the tree, because the iterable returned by
/// this function caches the results and only walks the tree once.
///
/// The same applies to any iterable obtained indirectly through this
/// one, for example the results of calling `where` on this iterable
/// are also cached.
Iterable<Element> collectAllElementsFrom(Element rootElement, {required bool skipOffstage}) {
  return CachingIterable<Element>(_DepthFirstElementTreeIterator(rootElement, !skipOffstage));
}

/// Provides an iterable that efficiently returns all the [SemanticsNode]s
/// rooted at the given [SemanticsNode]. See [CachingIterable] for details.
///
/// By default, this will traverse the semantics tree in semantic traversal
/// order, but the traversal order can be changed by passing in a different
/// value to `order`.
///
/// This function must be called again if the semantics change. You cannot call
/// this function once, then reuse the iterable after having changed the state
/// of the tree, because the iterable returned by this function caches the
/// results and only walks the tree once.
///
/// The same applies to any iterable obtained indirectly through this
/// one, for example the results of calling `where` on this iterable
/// are also cached.
Iterable<SemanticsNode> collectAllSemanticsNodesFrom(
  SemanticsNode root, {
  DebugSemanticsDumpOrder order = DebugSemanticsDumpOrder.traversalOrder,
}) {
  return CachingIterable<SemanticsNode>(_DepthFirstSemanticsTreeIterator(root, order));
}

/// Provides a recursive, efficient, depth first search of a tree.
///
/// This iterator executes a depth first search as an iterable, and iterates in
/// a left to right order:
///
///       1
///     /   \
///    2     3
///   / \   / \
///  4   5 6   7
///
/// Will iterate in order 2, 4, 5, 3, 6, 7. The given root element is not
/// included in the traversal.
abstract class _DepthFirstTreeIterator<ItemType> implements Iterator<ItemType> {
  _DepthFirstTreeIterator(ItemType root) {
    _fillStack(_collectChildren(root));
  }

  @override
  ItemType get current => _current!;
  late ItemType _current;

  final List<ItemType> _stack = <ItemType>[];

  @override
  bool moveNext() {
    if (_stack.isEmpty) {
      return false;
    }

    _current = _stack.removeLast();
    _fillStack(_collectChildren(_current));
    return true;
  }

  /// Fills the stack in such a way that the next element of a depth first
  /// traversal is easily and efficiently accessible when calling `moveNext`.
  void _fillStack(List<ItemType> children) {
    // We reverse the list of children so we don't have to do use expensive
    // `insert` or `remove` operations, and so the order of the traversal
    // is depth first when built lazily through the iterator.
    //
    // This is faster than `_stack.addAll(children.reversed)`, presumably since
    // we don't actually care about maintaining an iteration pointer.
    while (children.isNotEmpty) {
      _stack.add(children.removeLast());
    }
  }

  /// Collect the children from [root] in the order they are expected to traverse.
  List<ItemType> _collectChildren(ItemType root);
}

/// [Element.visitChildren] does not guarantee order, but does guarantee stable
/// order. This iterator also guarantees stable order, and iterates in a left
/// to right order:
///
///       1
///     /   \
///    2     3
///   / \   / \
///  4   5 6   7
///
/// Will iterate in order 2, 4, 5, 3, 6, 7.
///
/// Performance is important here because this class is on the critical path
/// for flutter_driver and package:integration_test performance tests.
/// Performance is measured in the all_elements_bench microbenchmark.
/// Any changes to this implementation should check the before and after numbers
/// on that benchmark to avoid regressions in general performance test overhead.
///
/// If we could use RTL order, we could save on performance, but numerous tests
/// have been written (and developers clearly expect) that LTR order will be
/// respected.
class _DepthFirstElementTreeIterator extends _DepthFirstTreeIterator<Element> {
  _DepthFirstElementTreeIterator(super.root, this.includeOffstage);

  final bool includeOffstage;

  @override
  List<Element> _collectChildren(Element root) {
    final List<Element> children = <Element>[];
    if (includeOffstage) {
      root.visitChildren(children.add);
    } else {
      root.debugVisitOnstageChildren(children.add);
    }

    return children;
  }
}

/// Iterates the semantics tree starting at the given `root`.
///
/// This will iterate in the same order expected from accessibility services,
/// so the results can be used to simulate the same traversal the engine will
/// make. The results are not filtered based on flags or visibility, so they
/// will need to be further filtered to fully simulate an accessibility service.
class _DepthFirstSemanticsTreeIterator extends _DepthFirstTreeIterator<SemanticsNode> {
  _DepthFirstSemanticsTreeIterator(super.root, this.order);

  final DebugSemanticsDumpOrder order;

  @override
  List<SemanticsNode> _collectChildren(SemanticsNode root) {
    return root.debugListChildrenInOrder(order);
  }
}
