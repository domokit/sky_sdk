// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

// This file gets mutated by //dev/devicelab/bin/tasks/flutter_test_performance.dart
// during device lab performance tests. When editing this file, check to make sure
// that it didn't break that test.

/// Deprecated. Unused by the framework and will be removed in a future version
/// of Flutter. If needed, inline any required functionality of this class
/// directly in the subclass.
///
/// An abstract node in a tree.
///
/// AbstractNode has as notion of depth, attachment, and parent, but does not
/// have a model for children.
///
/// When a subclass is changing the parent of a child, it should call either
/// `parent.adoptChild(child)` or `parent.dropChild(child)` as appropriate.
/// Subclasses can expose an API for manipulating the tree if desired (e.g. a
/// setter for a `child` property, or an `add()` method to manipulate a list).
///
/// The current parent node is exposed by the [parent] property.
///
/// The current attachment state is exposed by [attached]. The root of any tree
/// that is to be considered attached should be manually attached by calling
/// [attach]. Other than that, the [attach] and [detach] methods should not be
/// called directly; attachment is managed automatically by the aforementioned
/// [adoptChild] and [dropChild] methods.
///
/// Subclasses that have children must override [attach] and [detach] as
/// described in the documentation for those methods.
///
/// Nodes always have a [depth] greater than their ancestors'. There's no
/// guarantee regarding depth between siblings. The depth of a node is used to
/// ensure that nodes are processed in depth order. The [depth] of a child can
/// be more than one greater than the [depth] of the parent, because the [depth]
/// values are never decreased: all that matters is that it's greater than the
/// parent. Consider a tree with a root node A, a child B, and a grandchild C.
/// Initially, A will have [depth] 0, B [depth] 1, and C [depth] 2. If C is
/// moved to be a child of A, sibling of B, then the numbers won't change. C's
/// [depth] will still be 2. The [depth] is automatically maintained by the
/// [adoptChild] and [dropChild] methods.
@Deprecated(
  'If needed, inline any required functionality of AbstractNode in your class directly. '
  'This feature was deprecated after v3.12.0-4.0.pre.',
)
class AbstractNode {
  /// The depth of this node in the tree.
  ///
  /// The depth of nodes in a tree monotonically increases as you traverse down
  /// the tree.
  int get depth => _depth;
  int _depth = 0;

  /// Adjust the [depth] of the given [child] to be greater than this node's own
  /// [depth].
  ///
  /// Only call this method from overrides of [redepthChildren].
  @protected
  void redepthChild(AbstractNode child) {
    assert(child.owner == owner);
    if (child._depth <= _depth) {
      child._depth = _depth + 1;
      child.redepthChildren();
    }
  }

  /// Adjust the [depth] of this node's children, if any.
  ///
  /// Override this method in subclasses with child nodes to call [redepthChild]
  /// for each child. Do not call this method directly.
  void redepthChildren() {}

  /// The owner for this node (null if unattached).
  ///
  /// The entire subtree that this node belongs to will have the same owner.
  Object? get owner => _owner;
  Object? _owner;

  /// Whether this node is in a tree whose root is attached to something.
  ///
  /// This becomes true during the call to [attach].
  ///
  /// This becomes false during the call to [detach].
  bool get attached => _owner != null;

  /// Mark this node as attached to the given owner.
  ///
  /// Typically called only from the [parent]'s [attach] method, and by the
  /// [owner] to mark the root of a tree as attached.
  ///
  /// Subclasses with children should override this method to
  /// [attach] all their children to the same [owner]
  /// after calling the inherited method, as in `super.attach(owner)`.
  @mustCallSuper
  void attach(covariant Object owner) {
    assert(_owner == null);
    _owner = owner;
  }

  /// Mark this node as detached.
  ///
  /// Typically called only from the [parent]'s [detach], and by the [owner] to
  /// mark the root of a tree as detached.
  ///
  /// Subclasses with children should override this method to
  /// [detach] all their children after calling the inherited method,
  /// as in `super.detach()`.
  @mustCallSuper
  void detach() {
    assert(_owner != null);
    _owner = null;
    assert(parent == null || attached == parent!.attached);
  }

  /// The parent of this node in the tree.
  AbstractNode? get parent => _parent;
  AbstractNode? _parent;

  /// Mark the given node as being a child of this node.
  ///
  /// Subclasses should call this function when they acquire a new child.
  @protected
  @mustCallSuper
  void adoptChild(covariant AbstractNode child) {
    assert(child._parent == null);
    assert(() {
      AbstractNode node = this;
      while (node.parent != null) {
        node = node.parent!;
      }
      assert(node != child); // indicates we are about to create a cycle
      return true;
    }());
    child._parent = this;
    if (attached) {
      child.attach(_owner!);
    }
    redepthChild(child);
  }

  /// Disconnect the given node from this node.
  ///
  /// Subclasses should call this function when they lose a child.
  @protected
  @mustCallSuper
  void dropChild(covariant AbstractNode child) {
    assert(child._parent == this);
    assert(child.attached == attached);
    child._parent = null;
    if (attached) {
      child.detach();
    }
  }
}
