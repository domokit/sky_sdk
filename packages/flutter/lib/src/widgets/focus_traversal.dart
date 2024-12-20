// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
/// @docImport 'package:flutter/services.dart';
///
/// @docImport 'app.dart';
library;

import 'package:flutter/foundation.dart';

import 'actions.dart';
import 'basic.dart';
import 'focus_manager.dart';
import 'focus_scope.dart';
import 'framework.dart';
import 'scroll_position.dart';
import 'scrollable.dart';

// Examples can assume:
// late BuildContext context;
// FocusNode focusNode = FocusNode();

// BuildContext/Element doesn't have a parent accessor, but it can be simulated
// with visitAncestorElements. _getAncestor is needed because
// context.getElementForInheritedWidgetOfExactType will return itself if it
// happens to be of the correct type. _getAncestor should be O(count), since we
// always return false at a specific ancestor. By default it returns the parent,
// which is O(1).
BuildContext? _getAncestor(BuildContext context, {int count = 1}) {
  BuildContext? target;
  context.visitAncestorElements((Element ancestor) {
    count--;
    if (count == 0) {
      target = ancestor;
      return false;
    }
    return true;
  });
  return target;
}

/// Signature for the callback that's called when a traversal policy
/// requests focus.
typedef TraversalRequestFocusCallback =
    void Function(
      FocusNode node, {
      ScrollPositionAlignmentPolicy? alignmentPolicy,
      double? alignment,
      Duration? duration,
      Curve? curve,
    });

// A class to temporarily hold information about FocusTraversalGroups when
// sorting their contents.
class _FocusTraversalGroupInfo {
  _FocusTraversalGroupInfo(
    _FocusTraversalGroupNode? group, {
    FocusTraversalPolicy? defaultPolicy,
    List<FocusNode>? members,
  }) : groupNode = group,
       policy = group?.policy ?? defaultPolicy ?? ReadingOrderTraversalPolicy(),
       members = members ?? <FocusNode>[];

  final FocusNode? groupNode;
  final FocusTraversalPolicy policy;
  final List<FocusNode> members;
}

/// A direction along either the horizontal or vertical axes.
///
/// This is used by the [DirectionalFocusTraversalPolicyMixin], and
/// [FocusNode.focusInDirection] to indicate which direction to look in for the
/// next focus.
enum TraversalDirection {
  /// Indicates a direction above the currently focused widget.
  up,

  /// Indicates a direction to the right of the currently focused widget.
  ///
  /// This direction is unaffected by the [Directionality] of the current
  /// context.
  right,

  /// Indicates a direction below the currently focused widget.
  down,

  /// Indicates a direction to the left of the currently focused widget.
  ///
  /// This direction is unaffected by the [Directionality] of the current
  /// context.
  left,
}

/// Controls the transfer of focus beyond the first and the last items of a
/// [FocusScopeNode].
///
/// This enumeration only controls the traversal behavior performed by
/// [FocusTraversalPolicy]. Other methods of focus transfer, such as direct
/// calls to [FocusNode.requestFocus] and [FocusNode.unfocus], are not affected
/// by this enumeration.
///
/// See also:
///
/// * [FocusTraversalPolicy], which implements the logic behind this enum.
/// * [FocusScopeNode], which is configured by this enum.
enum TraversalEdgeBehavior {
  /// Keeps the focus among the items of the focus scope.
  ///
  /// Requesting the next focus after the last focusable item will transfer the
  /// focus to the first item, and requesting focus previous to the first item
  /// will transfer the focus to the last item, thus forming a closed loop of
  /// focusable items.
  closedLoop,

  /// Allows the focus to leave the [FlutterView].
  ///
  /// Requesting next focus after the last focusable item or previous to the
  /// first item will unfocus any focused nodes. If the focus traversal action
  /// was initiated by the embedder (e.g. the Flutter Engine) the embedder
  /// receives a result indicating that the focus is no longer within the
  /// current [FlutterView]. For example, [NextFocusAction] invoked via keyboard
  /// (typically the TAB key) would receive [KeyEventResult.skipRemainingHandlers]
  /// allowing the embedder handle the shortcut. On the web, typically the
  /// control is transferred to the browser, allowing the user to reach the
  /// address bar, escape an `iframe`, or focus on HTML elements other than
  /// those managed by Flutter.
  leaveFlutterView,

  /// Allows focus to traverse up to parent scope.
  ///
  /// When reaching the edge of the current scope, requesting the next focus
  /// will look up to the parent scope of the current scope and focus the focus
  /// node next to the current scope.
  ///
  /// If there is no parent scope above the current scope, fallback to
  /// [closedLoop] behavior.
  parentScope,
}

/// Determines how focusable widgets are traversed within a [FocusTraversalGroup].
///
/// The focus traversal policy is what determines which widget is "next",
/// "previous", or in a direction from the widget associated with the currently
/// focused [FocusNode] (usually a [Focus] widget).
///
/// One of the pre-defined subclasses may be used, or define a custom policy to
/// create a unique focus order.
///
/// When defining your own, your subclass should implement [sortDescendants] to
/// provide the order in which you would like the descendants to be traversed.
///
/// See also:
///
///  * [FocusNode], for a description of the focus system.
///  * [FocusTraversalGroup], a widget that groups together and imposes a
///    traversal policy on the [Focus] nodes below it in the widget hierarchy.
///  * [FocusNode], which is affected by the traversal policy.
///  * [WidgetOrderTraversalPolicy], a policy that relies on the widget
///    creation order to describe the order of traversal.
///  * [ReadingOrderTraversalPolicy], a policy that describes the order as the
///    natural "reading order" for the current [Directionality].
///  * [OrderedTraversalPolicy], a policy that describes the order
///    explicitly using [FocusTraversalOrder] widgets.
///  * [DirectionalFocusTraversalPolicyMixin] a mixin class that implements
///    focus traversal in a direction.
@immutable
abstract class FocusTraversalPolicy with Diagnosticable {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  ///
  /// {@template flutter.widgets.FocusTraversalPolicy.requestFocusCallback}
  /// The `requestFocusCallback` can be used to override the default behavior
  /// of the focus requests. If `requestFocusCallback`
  /// is null, it defaults to [FocusTraversalPolicy.defaultTraversalRequestFocusCallback].
  /// {@endtemplate}
  const FocusTraversalPolicy({TraversalRequestFocusCallback? requestFocusCallback})
    : requestFocusCallback = requestFocusCallback ?? defaultTraversalRequestFocusCallback;

  /// The callback used to move the focus from one focus node to another when
  /// traversing them using a keyboard. By default it requests focus on the next
  /// node and ensures the node is visible if it's in a scrollable.
  final TraversalRequestFocusCallback requestFocusCallback;

  /// The default value for [requestFocusCallback].
  /// Requests focus from `node` and ensures the node is visible
  /// by calling [Scrollable.ensureVisible].
  static void defaultTraversalRequestFocusCallback(
    FocusNode node, {
    ScrollPositionAlignmentPolicy? alignmentPolicy,
    double? alignment,
    Duration? duration,
    Curve? curve,
  }) {
    node.requestFocus();
    Scrollable.ensureVisible(
      node.context!,
      alignment: alignment ?? 1,
      alignmentPolicy: alignmentPolicy ?? ScrollPositionAlignmentPolicy.explicit,
      duration: duration ?? Duration.zero,
      curve: curve ?? Curves.ease,
    );
  }

  /// Request focus on a focus node as a result of a tab traversal.
  ///
  /// If the `node` is a [FocusScopeNode], this method will recursively find
  /// the next focus from its descendants until it find a regular [FocusNode].
  ///
  /// Returns true if this method focused a new focus node.
  bool _requestTabTraversalFocus(
    FocusNode node, {
    ScrollPositionAlignmentPolicy? alignmentPolicy,
    double? alignment,
    Duration? duration,
    Curve? curve,
    required bool forward,
  }) {
    if (node is FocusScopeNode) {
      if (node.focusedChild != null) {
        // Can't stop here as the `focusedChild` may be a focus scope node
        // without a first focus. The first focus will be picked in the
        // next iteration.
        return _requestTabTraversalFocus(
          node.focusedChild!,
          alignmentPolicy: alignmentPolicy,
          alignment: alignment,
          duration: duration,
          curve: curve,
          forward: forward,
        );
      }
      final List<FocusNode> sortedChildren = _sortAllDescendants(node, node);
      if (sortedChildren.isNotEmpty) {
        _requestTabTraversalFocus(
          forward ? sortedChildren.first : sortedChildren.last,
          alignmentPolicy: alignmentPolicy,
          alignment: alignment,
          duration: duration,
          curve: curve,
          forward: forward,
        );
        // Regardless if _requestTabTraversalFocus return true or false, a first
        // focus has been picked.
        return true;
      }
    }
    final bool nodeHadPrimaryFocus = node.hasPrimaryFocus;
    requestFocusCallback(
      node,
      alignmentPolicy: alignmentPolicy,
      alignment: alignment,
      duration: duration,
      curve: curve,
    );
    return !nodeHadPrimaryFocus;
  }

  /// Returns the node that should receive focus if focus is traversing
  /// forwards, and there is no current focus.
  ///
  /// The node returned is the node that should receive focus if focus is
  /// traversing forwards (i.e. with [next]), and there is no current focus in
  /// the nearest [FocusScopeNode] that `currentNode` belongs to.
  ///
  /// If `ignoreCurrentFocus` is false or not given, this function returns the
  /// [FocusScopeNode.focusedChild], if set, on the nearest scope of the
  /// `currentNode`, otherwise, returns the first node from [sortDescendants],
  /// or the given `currentNode` if there are no descendants.
  ///
  /// If `ignoreCurrentFocus` is true, then the algorithm returns the first node
  /// from [sortDescendants], or the given `currentNode` if there are no
  /// descendants.
  ///
  /// See also:
  ///
  /// * [next], the function that is called to move the focus to the next node.
  /// * [DirectionalFocusTraversalPolicyMixin.findFirstFocusInDirection], a
  ///   function that finds the first focusable widget in a particular
  ///   direction.
  FocusNode? findFirstFocus(FocusNode currentNode, {bool ignoreCurrentFocus = false}) {
    return _findInitialFocus(currentNode, ignoreCurrentFocus: ignoreCurrentFocus);
  }

  /// Returns the node that should receive focus if focus is traversing
  /// backwards, and there is no current focus.
  ///
  /// The node returned is the one that should receive focus if focus is
  /// traversing backwards (i.e. with [previous]), and there is no current focus
  /// in the nearest [FocusScopeNode] that `currentNode` belongs to.
  ///
  /// If `ignoreCurrentFocus` is false or not given, this function returns the
  /// [FocusScopeNode.focusedChild], if set, on the nearest scope of the
  /// `currentNode`, otherwise, returns the last node from [sortDescendants],
  /// or the given `currentNode` if there are no descendants.
  ///
  /// If `ignoreCurrentFocus` is true, then the algorithm returns the last node
  /// from [sortDescendants], or the given `currentNode` if there are no
  /// descendants.
  ///
  /// See also:
  ///
  ///  * [previous], the function that is called to move the focus to the previous node.
  ///  * [DirectionalFocusTraversalPolicyMixin.findFirstFocusInDirection], a
  ///    function that finds the first focusable widget in a particular direction.
  FocusNode findLastFocus(FocusNode currentNode, {bool ignoreCurrentFocus = false}) {
    return _findInitialFocus(currentNode, fromEnd: true, ignoreCurrentFocus: ignoreCurrentFocus);
  }

  FocusNode _findInitialFocus(
    FocusNode currentNode, {
    bool fromEnd = false,
    bool ignoreCurrentFocus = false,
  }) {
    final FocusScopeNode scope = currentNode.nearestScope!;
    FocusNode? candidate = scope.focusedChild;
    if (ignoreCurrentFocus || candidate == null && scope.descendants.isNotEmpty) {
      final Iterable<FocusNode> sorted = _sortAllDescendants(
        scope,
        currentNode,
      ).where((FocusNode node) => _canRequestTraversalFocus(node));
      if (sorted.isEmpty) {
        candidate = null;
      } else {
        candidate = fromEnd ? sorted.last : sorted.first;
      }
    }

    // If we still didn't find any candidate, use the current node as a
    // fallback.
    candidate ??= currentNode;
    return candidate;
  }

  /// Returns the first node in the given `direction` that should receive focus
  /// if there is no current focus in the scope to which the `currentNode`
  /// belongs.
  ///
  /// This is typically used by [inDirection] to determine which node to focus
  /// if it is called when no node is currently focused.
  FocusNode? findFirstFocusInDirection(FocusNode currentNode, TraversalDirection direction);

  /// Clears the data associated with the given [FocusScopeNode] for this object.
  ///
  /// This is used to indicate that the focus policy has changed its mode, and
  /// so any cached policy data should be invalidated. For example, changing the
  /// direction in which focus is moving, or changing from directional to
  /// next/previous navigation modes.
  ///
  /// The default implementation does nothing.
  @mustCallSuper
  void invalidateScopeData(FocusScopeNode node) {}

  /// This is called whenever the given [node] is re-parented into a new scope,
  /// so that the policy has a chance to update or invalidate any cached data
  /// that it maintains per scope about the node.
  ///
  /// The [oldScope] is the previous scope that this node belonged to, if any.
  ///
  /// The default implementation does nothing.
  @mustCallSuper
  void changedScope({FocusNode? node, FocusScopeNode? oldScope}) {}

  /// Focuses the next widget in the focus scope that contains the given
  /// [currentNode].
  ///
  /// This should determine what the next node to receive focus should be by
  /// inspecting the node tree, and then calling [FocusNode.requestFocus] on
  /// the node that has been selected.
  ///
  /// Returns true if it successfully found a node and requested focus.
  bool next(FocusNode currentNode) => _moveFocus(currentNode, forward: true);

  /// Focuses the previous widget in the focus scope that contains the given
  /// [currentNode].
  ///
  /// This should determine what the previous node to receive focus should be by
  /// inspecting the node tree, and then calling [FocusNode.requestFocus] on
  /// the node that has been selected.
  ///
  /// Returns true if it successfully found a node and requested focus.
  bool previous(FocusNode currentNode) => _moveFocus(currentNode, forward: false);

  /// Focuses the next widget in the given [direction] in the focus scope that
  /// contains the given [currentNode].
  ///
  /// This should determine what the next node to receive focus in the given
  /// [direction] should be by inspecting the node tree, and then calling
  /// [FocusNode.requestFocus] on the node that has been selected.
  ///
  /// Returns true if it successfully found a node and requested focus.
  bool inDirection(FocusNode currentNode, TraversalDirection direction);

  /// Sorts the given `descendants` into focus order.
  ///
  /// Subclasses should override this to implement a different sort for [next]
  /// and [previous] to use in their ordering. If the returned iterable omits a
  /// node that is a descendant of the given scope, then the user will be unable
  /// to use next/previous keyboard traversal to reach that node.
  ///
  /// The node used to initiate the traversal (the one passed to [next] or
  /// [previous]) is passed as `currentNode`.
  ///
  /// Having the current node in the list is what allows the algorithm to
  /// determine which nodes are adjacent to the current node. If the
  /// `currentNode` is removed from the list, then the focus will be unchanged
  /// when [next] or [previous] are called, and they will return false.
  ///
  /// This is not used for directional focus ([inDirection]), only for
  /// determining the focus order for [next] and [previous].
  ///
  /// When implementing an override for this function, be sure to use
  /// [mergeSort] instead of Dart's default list sorting algorithm when sorting
  /// items, since the default algorithm is not stable (items deemed to be equal
  /// can appear in arbitrary order, and change positions between sorts), whereas
  /// [mergeSort] is stable.
  @protected
  Iterable<FocusNode> sortDescendants(Iterable<FocusNode> descendants, FocusNode currentNode);

  static bool _canRequestTraversalFocus(FocusNode node) {
    return node.canRequestFocus && !node.skipTraversal;
  }

  static Iterable<FocusNode> _getDescendantsWithoutExpandingScope(FocusNode node) {
    final List<FocusNode> result = <FocusNode>[];
    for (final FocusNode child in node.children) {
      result.add(child);
      if (child is! FocusScopeNode) {
        result.addAll(_getDescendantsWithoutExpandingScope(child));
      }
    }
    return result;
  }

  static Map<FocusNode?, _FocusTraversalGroupInfo> _findGroups(
    FocusScopeNode scope,
    _FocusTraversalGroupNode? scopeGroupNode,
    FocusNode currentNode,
  ) {
    final FocusTraversalPolicy defaultPolicy =
        scopeGroupNode?.policy ?? ReadingOrderTraversalPolicy();
    final Map<FocusNode?, _FocusTraversalGroupInfo> groups =
        <FocusNode?, _FocusTraversalGroupInfo>{};
    for (final FocusNode node in _getDescendantsWithoutExpandingScope(scope)) {
      final _FocusTraversalGroupNode? groupNode = FocusTraversalGroup._getGroupNode(node);
      // Group nodes need to be added to their parent's node, or to the "null"
      // node if no parent is found. This creates the hierarchy of group nodes
      // and makes it so the entire group is sorted along with the other members
      // of the parent group.
      if (node == groupNode) {
        // To find the parent of the group node, we need to skip over the parent
        // of the Focus node added in _FocusTraversalGroupState.build, and start
        // looking with that node's parent, since _getGroupNode will return the
        // node it was called on if it matches the type.
        final _FocusTraversalGroupNode? parentGroup = FocusTraversalGroup._getGroupNode(
          groupNode!.parent!,
        );
        groups[parentGroup] ??= _FocusTraversalGroupInfo(
          parentGroup,
          members: <FocusNode>[],
          defaultPolicy: defaultPolicy,
        );
        assert(!groups[parentGroup]!.members.contains(node));
        groups[parentGroup]!.members.add(groupNode);
        continue;
      }
      // Skip non-focusable and non-traversable nodes in the same way that
      // FocusScopeNode.traversalDescendants would.
      //
      // Current focused node needs to be in the group so that the caller can
      // find the next traversable node from the current focused node.
      if (node == currentNode || (node.canRequestFocus && !node.skipTraversal)) {
        groups[groupNode] ??= _FocusTraversalGroupInfo(
          groupNode,
          members: <FocusNode>[],
          defaultPolicy: defaultPolicy,
        );
        assert(!groups[groupNode]!.members.contains(node));
        groups[groupNode]!.members.add(node);
      }
    }
    return groups;
  }

  // Sort all descendants, taking into account the FocusTraversalGroup
  // that they are each in, and filtering out non-traversable/focusable nodes.
  static List<FocusNode> _sortAllDescendants(FocusScopeNode scope, FocusNode currentNode) {
    final _FocusTraversalGroupNode? scopeGroupNode = FocusTraversalGroup._getGroupNode(scope);
    // Build the sorting data structure, separating descendants into groups.
    final Map<FocusNode?, _FocusTraversalGroupInfo> groups = _findGroups(
      scope,
      scopeGroupNode,
      currentNode,
    );

    // Sort the member lists using the individual policy sorts.
    for (final FocusNode? key in groups.keys) {
      final List<FocusNode> sortedMembers =
          groups[key]!.policy.sortDescendants(groups[key]!.members, currentNode).toList();
      groups[key]!.members.clear();
      groups[key]!.members.addAll(sortedMembers);
    }

    // Traverse the group tree, adding the children of members in the order they
    // appear in the member lists.
    final List<FocusNode> sortedDescendants = <FocusNode>[];
    void visitGroups(_FocusTraversalGroupInfo info) {
      for (final FocusNode node in info.members) {
        if (groups.containsKey(node)) {
          // This is a policy group focus node. Replace it with the members of
          // the corresponding policy group.
          visitGroups(groups[node]!);
        } else {
          sortedDescendants.add(node);
        }
      }
    }

    // Visit the children of the scope, if any.
    if (groups.isNotEmpty && groups.containsKey(scopeGroupNode)) {
      visitGroups(groups[scopeGroupNode]!);
    }

    // Remove the FocusTraversalGroup nodes themselves, which aren't focusable.
    // They were left in above because they were needed to find their members
    // during sorting.
    sortedDescendants.removeWhere((FocusNode node) {
      return node != currentNode && !_canRequestTraversalFocus(node);
    });

    // Sanity check to make sure that the algorithm above doesn't diverge from
    // the one in FocusScopeNode.traversalDescendants in terms of which nodes it
    // finds.
    assert(() {
      final Set<FocusNode> difference = sortedDescendants.toSet().difference(
        scope.traversalDescendants.toSet(),
      );
      if (!_canRequestTraversalFocus(currentNode)) {
        // The scope.traversalDescendants will not contain currentNode if it
        // skips traversal or not focusable.
        assert(
          difference.isEmpty || (difference.length == 1 && difference.contains(currentNode)),
          'Difference between sorted descendants and FocusScopeNode.traversalDescendants contains '
          'something other than the current skipped node. This is the difference: $difference',
        );
        return true;
      }
      assert(
        difference.isEmpty,
        'Sorted descendants contains different nodes than FocusScopeNode.traversalDescendants would. '
        'These are the different nodes: $difference',
      );
      return true;
    }());
    return sortedDescendants;
  }

  /// Moves the focus to the next node in the FocusScopeNode nearest to the
  /// currentNode argument, either in a forward or reverse direction, depending
  /// on the value of the forward argument.
  ///
  /// This function is called by the next and previous members to move to the
  /// next or previous node, respectively.
  ///
  /// Uses [findFirstFocus]/[findLastFocus] to find the first/last node if there is
  /// no [FocusScopeNode.focusedChild] set. If there is a focused child for the
  /// scope, then it calls sortDescendants to get a sorted list of descendants,
  /// and then finds the node after the current first focus of the scope if
  /// forward is true, and the node before it if forward is false.
  ///
  /// Returns true if a node requested focus.
  @protected
  bool _moveFocus(FocusNode currentNode, {required bool forward}) {
    final FocusScopeNode nearestScope = currentNode.nearestScope!;
    invalidateScopeData(nearestScope);
    FocusNode? focusedChild = nearestScope.focusedChild;
    if (focusedChild == null) {
      final FocusNode? firstFocus =
          forward ? findFirstFocus(currentNode) : findLastFocus(currentNode);
      if (firstFocus != null) {
        return _requestTabTraversalFocus(
          firstFocus,
          alignmentPolicy:
              forward
                  ? ScrollPositionAlignmentPolicy.keepVisibleAtEnd
                  : ScrollPositionAlignmentPolicy.keepVisibleAtStart,
          forward: forward,
        );
      }
    }
    focusedChild ??= nearestScope;
    final List<FocusNode> sortedNodes = _sortAllDescendants(nearestScope, focusedChild);
    assert(sortedNodes.contains(focusedChild));

    if (forward && focusedChild == sortedNodes.last) {
      switch (nearestScope.traversalEdgeBehavior) {
        case TraversalEdgeBehavior.leaveFlutterView:
          focusedChild.unfocus();
          return false;
        case TraversalEdgeBehavior.parentScope:
          final FocusScopeNode? parentScope = nearestScope.enclosingScope;
          if (parentScope != null && parentScope != FocusManager.instance.rootScope) {
            focusedChild.unfocus();
            parentScope.nextFocus();
            // Verify the focus really has changed.
            return focusedChild.enclosingScope?.focusedChild != focusedChild;
          }
          // No valid parent scope. Fallback to closed loop behavior.
          return _requestTabTraversalFocus(
            sortedNodes.first,
            alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
            forward: forward,
          );
        case TraversalEdgeBehavior.closedLoop:
          return _requestTabTraversalFocus(
            sortedNodes.first,
            alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
            forward: forward,
          );
      }
    }
    if (!forward && focusedChild == sortedNodes.first) {
      switch (nearestScope.traversalEdgeBehavior) {
        case TraversalEdgeBehavior.leaveFlutterView:
          focusedChild.unfocus();
          return false;
        case TraversalEdgeBehavior.parentScope:
          final FocusScopeNode? parentScope = nearestScope.enclosingScope;
          if (parentScope != null && parentScope != FocusManager.instance.rootScope) {
            focusedChild.unfocus();
            parentScope.previousFocus();
            // Verify the focus really has changed.
            return focusedChild.enclosingScope?.focusedChild != focusedChild;
          }
          // No valid parent scope. Fallback to closed loop behavior.
          return _requestTabTraversalFocus(
            sortedNodes.last,
            alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
            forward: forward,
          );
        case TraversalEdgeBehavior.closedLoop:
          return _requestTabTraversalFocus(
            sortedNodes.last,
            alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
            forward: forward,
          );
      }
    }

    final Iterable<FocusNode> maybeFlipped = forward ? sortedNodes : sortedNodes.reversed;
    FocusNode? previousNode;
    for (final FocusNode node in maybeFlipped) {
      if (previousNode == focusedChild) {
        return _requestTabTraversalFocus(
          node,
          alignmentPolicy:
              forward
                  ? ScrollPositionAlignmentPolicy.keepVisibleAtEnd
                  : ScrollPositionAlignmentPolicy.keepVisibleAtStart,
          forward: forward,
        );
      }
      previousNode = node;
    }
    return false;
  }
}

// A policy data object for use by the DirectionalFocusTraversalPolicyMixin so
// it can keep track of the traversal history.
class _DirectionalPolicyDataEntry {
  const _DirectionalPolicyDataEntry({required this.direction, required this.node});

  final TraversalDirection direction;
  final FocusNode node;
}

class _DirectionalPolicyData {
  const _DirectionalPolicyData({required this.history});

  /// A queue of entries that describe the path taken to the current node.
  final List<_DirectionalPolicyDataEntry> history;
}

/// A mixin class that provides an implementation for finding a node in a
/// particular direction.
///
/// This can be mixed in to other [FocusTraversalPolicy] implementations that
/// only want to implement new next/previous policies.
///
/// Since hysteresis in the navigation order is undesirable, this implementation
/// maintains a stack of previous locations that have been visited on the policy
/// data for the affected [FocusScopeNode]. If the previous direction was the
/// opposite of the current direction, then the this policy will request focus
/// on the previously focused node. Change to another direction other than the
/// current one or its opposite will clear the stack.
///
/// For instance, if the focus moves down, down, down, and then up, up, up, it
/// will follow the same path through the widgets in both directions. However,
/// if it moves down, down, down, left, right, and then up, up, up, it may not
/// follow the same path on the way up as it did on the way down, since changing
/// the axis of motion resets the history.
///
/// This class implements an algorithm that considers an infinite band extending
/// along the direction of movement, the width or height (depending on
/// direction) of the currently focused widget, and finds the closest widget in
/// that band along the direction of movement. If nothing is found in that band,
/// then it picks the widget with an edge closest to the band in the
/// perpendicular direction. If two out-of-band widgets are the same distance
/// from the band, then it picks the one closest along the direction of
/// movement.
///
/// The goal of this algorithm is to pick a widget that (to the user) doesn't
/// appear to traverse along the wrong axis, as it might if it only sorted
/// widgets by distance along one axis, but also jumps to the next logical
/// widget in a direction without skipping over widgets.
///
/// See also:
///
/// * [FocusNode], for a description of the focus system.
/// * [FocusTraversalGroup], a widget that groups together and imposes a
///   traversal policy on the [Focus] nodes below it in the widget hierarchy.
/// * [WidgetOrderTraversalPolicy], a policy that relies on the widget creation
///   order to describe the order of traversal.
/// * [ReadingOrderTraversalPolicy], a policy that describes the order as the
///   natural "reading order" for the current [Directionality].
/// * [OrderedTraversalPolicy], a policy that describes the order explicitly
///   using [FocusTraversalOrder] widgets.
mixin DirectionalFocusTraversalPolicyMixin on FocusTraversalPolicy {
  final Map<FocusScopeNode, _DirectionalPolicyData> _policyData =
      <FocusScopeNode, _DirectionalPolicyData>{};

  @override
  void invalidateScopeData(FocusScopeNode node) {
    super.invalidateScopeData(node);
    _policyData.remove(node);
  }

  @override
  void changedScope({FocusNode? node, FocusScopeNode? oldScope}) {
    super.changedScope(node: node, oldScope: oldScope);
    if (oldScope != null) {
      _policyData[oldScope]?.history.removeWhere((_DirectionalPolicyDataEntry entry) {
        return entry.node == node;
      });
    }
  }

  @override
  FocusNode? findFirstFocusInDirection(FocusNode currentNode, TraversalDirection direction) {
    final Iterable<FocusNode> nodes = currentNode.nearestScope!.traversalDescendants;
    final List<FocusNode> sorted = nodes.toList();
    final (bool vertical, bool first) = switch (direction) {
      TraversalDirection.up => (true, false), // Start with the bottom-most node.
      TraversalDirection.down => (true, true), // Start with the topmost node.
      TraversalDirection.left => (false, false), // Start with the rightmost node.
      TraversalDirection.right => (false, true), // Start with the leftmost node.
    };
    mergeSort<FocusNode>(
      sorted,
      compare: (FocusNode a, FocusNode b) {
        if (vertical) {
          if (first) {
            return a.rect.top.compareTo(b.rect.top);
          } else {
            return b.rect.bottom.compareTo(a.rect.bottom);
          }
        } else {
          if (first) {
            return a.rect.left.compareTo(b.rect.left);
          } else {
            return b.rect.right.compareTo(a.rect.right);
          }
        }
      },
    );

    return sorted.firstOrNull;
  }

  static int _verticalCompare(Offset target, Offset a, Offset b) {
    return (a.dy - target.dy).abs().compareTo((b.dy - target.dy).abs());
  }

  static int _horizontalCompare(Offset target, Offset a, Offset b) {
    return (a.dx - target.dx).abs().compareTo((b.dx - target.dx).abs());
  }

  // Sort the ones that are closest to target vertically first, and if two are
  // the same vertical distance, pick the one that is closest horizontally.
  static Iterable<FocusNode> _sortByDistancePreferVertical(
    Offset target,
    Iterable<FocusNode> nodes,
  ) {
    final List<FocusNode> sorted = nodes.toList();
    mergeSort<FocusNode>(
      sorted,
      compare: (FocusNode nodeA, FocusNode nodeB) {
        final Offset a = nodeA.rect.center;
        final Offset b = nodeB.rect.center;
        final int vertical = _verticalCompare(target, a, b);
        if (vertical == 0) {
          return _horizontalCompare(target, a, b);
        }
        return vertical;
      },
    );
    return sorted;
  }

  // Sort the ones that are closest horizontally first, and if two are the same
  // horizontal distance, pick the one that is closest vertically.
  static Iterable<FocusNode> _sortByDistancePreferHorizontal(
    Offset target,
    Iterable<FocusNode> nodes,
  ) {
    final List<FocusNode> sorted = nodes.toList();
    mergeSort<FocusNode>(
      sorted,
      compare: (FocusNode nodeA, FocusNode nodeB) {
        final Offset a = nodeA.rect.center;
        final Offset b = nodeB.rect.center;
        final int horizontal = _horizontalCompare(target, a, b);
        if (horizontal == 0) {
          return _verticalCompare(target, a, b);
        }
        return horizontal;
      },
    );
    return sorted;
  }

  static int _verticalCompareClosestEdge(Offset target, Rect a, Rect b) {
    // Find which edge is closest to the target for each.
    final double aCoord =
        (a.top - target.dy).abs() < (a.bottom - target.dy).abs() ? a.top : a.bottom;
    final double bCoord =
        (b.top - target.dy).abs() < (b.bottom - target.dy).abs() ? b.top : b.bottom;
    return (aCoord - target.dy).abs().compareTo((bCoord - target.dy).abs());
  }

  static int _horizontalCompareClosestEdge(Offset target, Rect a, Rect b) {
    // Find which edge is closest to the target for each.
    final double aCoord =
        (a.left - target.dx).abs() < (a.right - target.dx).abs() ? a.left : a.right;
    final double bCoord =
        (b.left - target.dx).abs() < (b.right - target.dx).abs() ? b.left : b.right;
    return (aCoord - target.dx).abs().compareTo((bCoord - target.dx).abs());
  }

  // Sort the ones that have edges that are closest horizontally first, and if
  // two are the same horizontal distance, pick the one that is closest
  // vertically.
  static Iterable<FocusNode> _sortClosestEdgesByDistancePreferHorizontal(
    Offset target,
    Iterable<FocusNode> nodes,
  ) {
    final List<FocusNode> sorted = nodes.toList();
    mergeSort<FocusNode>(
      sorted,
      compare: (FocusNode nodeA, FocusNode nodeB) {
        final int horizontal = _horizontalCompareClosestEdge(target, nodeA.rect, nodeB.rect);
        if (horizontal == 0) {
          // If they're the same distance horizontally, pick the closest one
          // vertically.
          return _verticalCompare(target, nodeA.rect.center, nodeB.rect.center);
        }
        return horizontal;
      },
    );
    return sorted;
  }

  // Sort the ones that have edges that are closest vertically first, and if
  // two are the same vertical distance, pick the one that is closest
  // horizontally.
  static Iterable<FocusNode> _sortClosestEdgesByDistancePreferVertical(
    Offset target,
    Iterable<FocusNode> nodes,
  ) {
    final List<FocusNode> sorted = nodes.toList();
    mergeSort<FocusNode>(
      sorted,
      compare: (FocusNode nodeA, FocusNode nodeB) {
        final int vertical = _verticalCompareClosestEdge(target, nodeA.rect, nodeB.rect);
        if (vertical == 0) {
          // If they're the same distance vertically, pick the closest one
          // horizontally.
          return _horizontalCompare(target, nodeA.rect.center, nodeB.rect.center);
        }
        return vertical;
      },
    );
    return sorted;
  }

  // Sorts nodes from left to right horizontally, and removes nodes that are
  // either to the right of the left side of the target node if we're going
  // left, or to the left of the right side of the target node if we're going
  // right.
  //
  // This doesn't need to take into account directionality because it is
  // typically intending to actually go left or right, not in a reading
  // direction.
  Iterable<FocusNode> _sortAndFilterHorizontally(
    TraversalDirection direction,
    Rect target,
    Iterable<FocusNode> nodes,
  ) {
    assert(direction == TraversalDirection.left || direction == TraversalDirection.right);
    final List<FocusNode> sorted =
        nodes.where(switch (direction) {
          TraversalDirection.left =>
            (FocusNode node) => node.rect != target && node.rect.center.dx <= target.left,
          TraversalDirection.right =>
            (FocusNode node) => node.rect != target && node.rect.center.dx >= target.right,
          TraversalDirection.up || TraversalDirection.down =>
            throw ArgumentError('Invalid direction $direction'),
        }).toList();
    // Sort all nodes from left to right.
    mergeSort<FocusNode>(
      sorted,
      compare: (FocusNode a, FocusNode b) => a.rect.center.dx.compareTo(b.rect.center.dx),
    );
    return sorted;
  }

  // Sorts nodes from top to bottom vertically, and removes nodes that are
  // either below the top of the target node if we're going up, or above the
  // bottom of the target node if we're going down.
  Iterable<FocusNode> _sortAndFilterVertically(
    TraversalDirection direction,
    Rect target,
    Iterable<FocusNode> nodes,
  ) {
    assert(direction == TraversalDirection.up || direction == TraversalDirection.down);
    final List<FocusNode> sorted =
        nodes.where(switch (direction) {
          TraversalDirection.up =>
            (FocusNode node) => node.rect != target && node.rect.center.dy <= target.top,
          TraversalDirection.down =>
            (FocusNode node) => node.rect != target && node.rect.center.dy >= target.bottom,
          TraversalDirection.left || TraversalDirection.right =>
            throw ArgumentError('Invalid direction $direction'),
        }).toList();
    mergeSort<FocusNode>(
      sorted,
      compare: (FocusNode a, FocusNode b) => a.rect.center.dy.compareTo(b.rect.center.dy),
    );
    return sorted;
  }

  // Updates the policy data to keep the previously visited node so that we can
  // avoid hysteresis when we change directions in navigation.
  //
  // Returns true if focus was requested on a previous node.
  bool _popPolicyDataIfNeeded(
    TraversalDirection direction,
    FocusScopeNode nearestScope,
    FocusNode focusedChild,
  ) {
    final _DirectionalPolicyData? policyData = _policyData[nearestScope];
    if (policyData != null &&
        policyData.history.isNotEmpty &&
        policyData.history.first.direction != direction) {
      if (policyData.history.last.node.parent == null) {
        // If a node has been removed from the tree, then we should stop
        // referencing it and reset the scope data so that we don't try and
        // request focus on it. This can happen in slivers where the rendered
        // node has been unmounted. This has the side effect that hysteresis
        // might not be avoided when items that go off screen get unmounted.
        invalidateScopeData(nearestScope);
        return false;
      }

      // Returns true if successfully popped the history.
      bool popOrInvalidate(TraversalDirection direction) {
        final FocusNode lastNode = policyData.history.removeLast().node;
        if (Scrollable.maybeOf(lastNode.context!) != Scrollable.maybeOf(primaryFocus!.context!)) {
          invalidateScopeData(nearestScope);
          return false;
        }
        final ScrollPositionAlignmentPolicy alignmentPolicy;
        switch (direction) {
          case TraversalDirection.up:
          case TraversalDirection.left:
            alignmentPolicy = ScrollPositionAlignmentPolicy.keepVisibleAtStart;
          case TraversalDirection.right:
          case TraversalDirection.down:
            alignmentPolicy = ScrollPositionAlignmentPolicy.keepVisibleAtEnd;
        }
        requestFocusCallback(lastNode, alignmentPolicy: alignmentPolicy);
        return true;
      }

      switch (direction) {
        case TraversalDirection.down:
        case TraversalDirection.up:
          switch (policyData.history.first.direction) {
            case TraversalDirection.left:
            case TraversalDirection.right:
              // Reset the policy data if we change directions.
              invalidateScopeData(nearestScope);
            case TraversalDirection.up:
            case TraversalDirection.down:
              if (popOrInvalidate(direction)) {
                return true;
              }
          }
        case TraversalDirection.left:
        case TraversalDirection.right:
          switch (policyData.history.first.direction) {
            case TraversalDirection.left:
            case TraversalDirection.right:
              if (popOrInvalidate(direction)) {
                return true;
              }
            case TraversalDirection.up:
            case TraversalDirection.down:
              // Reset the policy data if we change directions.
              invalidateScopeData(nearestScope);
          }
      }
    }
    if (policyData != null && policyData.history.isEmpty) {
      invalidateScopeData(nearestScope);
    }
    return false;
  }

  void _pushPolicyData(
    TraversalDirection direction,
    FocusScopeNode nearestScope,
    FocusNode focusedChild,
  ) {
    final _DirectionalPolicyData? policyData = _policyData[nearestScope];
    final _DirectionalPolicyDataEntry newEntry = _DirectionalPolicyDataEntry(
      node: focusedChild,
      direction: direction,
    );
    if (policyData != null) {
      policyData.history.add(newEntry);
    } else {
      _policyData[nearestScope] = _DirectionalPolicyData(
        history: <_DirectionalPolicyDataEntry>[newEntry],
      );
    }
  }

  /// Focuses the next widget in the given [direction] in the [FocusScope] that
  /// contains the [currentNode].
  ///
  /// This determines what the next node to receive focus in the given
  /// [direction] will be by inspecting the node tree, and then calling
  /// [FocusNode.requestFocus] on it.
  ///
  /// Returns true if it successfully found a node and requested focus.
  ///
  /// Maintains a stack of previous locations that have been visited on the
  /// policy data for the affected [FocusScopeNode]. If the previous direction
  /// was the opposite of the current direction, then the this policy will
  /// request focus on the previously focused node. Change to another direction
  /// other than the current one or its opposite will clear the stack.
  ///
  /// If this function returns true when called by a subclass, then the subclass
  /// should return true and not request focus from any node.
  @mustCallSuper
  @override
  bool inDirection(FocusNode currentNode, TraversalDirection direction) {
    final FocusScopeNode nearestScope = currentNode.nearestScope!;
    final FocusNode? focusedChild = nearestScope.focusedChild;
    if (focusedChild == null) {
      final FocusNode firstFocus = findFirstFocusInDirection(currentNode, direction) ?? currentNode;
      switch (direction) {
        case TraversalDirection.up:
        case TraversalDirection.left:
          requestFocusCallback(
            firstFocus,
            alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
          );
        case TraversalDirection.right:
        case TraversalDirection.down:
          requestFocusCallback(
            firstFocus,
            alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
          );
      }
      return true;
    }
    if (_popPolicyDataIfNeeded(direction, nearestScope, focusedChild)) {
      return true;
    }
    FocusNode? found;
    final ScrollableState? focusedScrollable = Scrollable.maybeOf(focusedChild.context!);
    switch (direction) {
      case TraversalDirection.down:
      case TraversalDirection.up:
        Iterable<FocusNode> eligibleNodes = _sortAndFilterVertically(
          direction,
          focusedChild.rect,
          nearestScope.traversalDescendants,
        );
        if (eligibleNodes.isEmpty) {
          break;
        }
        if (focusedScrollable != null && !focusedScrollable.position.atEdge) {
          final Iterable<FocusNode> filteredEligibleNodes = eligibleNodes.where(
            (FocusNode node) => Scrollable.maybeOf(node.context!) == focusedScrollable,
          );
          if (filteredEligibleNodes.isNotEmpty) {
            eligibleNodes = filteredEligibleNodes;
          }
        }
        if (direction == TraversalDirection.up) {
          eligibleNodes = eligibleNodes.toList().reversed;
        }
        // Find any nodes that intersect the band of the focused child.
        final Rect band = Rect.fromLTRB(
          focusedChild.rect.left,
          -double.infinity,
          focusedChild.rect.right,
          double.infinity,
        );
        final Iterable<FocusNode> inBand = eligibleNodes.where(
          (FocusNode node) => !node.rect.intersect(band).isEmpty,
        );
        if (inBand.isNotEmpty) {
          found = _sortByDistancePreferVertical(focusedChild.rect.center, inBand).first;
          break;
        }
        // Only out-of-band targets are eligible, so pick the one that is
        // closest to the center line horizontally, and if any are the same
        // distance horizontally, pick the closest one of those vertically.
        found =
            _sortClosestEdgesByDistancePreferHorizontal(
              focusedChild.rect.center,
              eligibleNodes,
            ).first;
      case TraversalDirection.right:
      case TraversalDirection.left:
        Iterable<FocusNode> eligibleNodes = _sortAndFilterHorizontally(
          direction,
          focusedChild.rect,
          nearestScope.traversalDescendants,
        );
        if (eligibleNodes.isEmpty) {
          break;
        }
        if (focusedScrollable != null && !focusedScrollable.position.atEdge) {
          final Iterable<FocusNode> filteredEligibleNodes = eligibleNodes.where(
            (FocusNode node) => Scrollable.maybeOf(node.context!) == focusedScrollable,
          );
          if (filteredEligibleNodes.isNotEmpty) {
            eligibleNodes = filteredEligibleNodes;
          }
        }
        if (direction == TraversalDirection.left) {
          eligibleNodes = eligibleNodes.toList().reversed;
        }
        // Find any nodes that intersect the band of the focused child.
        final Rect band = Rect.fromLTRB(
          -double.infinity,
          focusedChild.rect.top,
          double.infinity,
          focusedChild.rect.bottom,
        );
        final Iterable<FocusNode> inBand = eligibleNodes.where(
          (FocusNode node) => !node.rect.intersect(band).isEmpty,
        );
        if (inBand.isNotEmpty) {
          found = _sortByDistancePreferHorizontal(focusedChild.rect.center, inBand).first;
          break;
        }
        // Only out-of-band targets are eligible, so pick the one that is
        // closest to the center line vertically, and if any are the same
        // distance vertically, pick the closest one of those horizontally.
        found =
            _sortClosestEdgesByDistancePreferVertical(
              focusedChild.rect.center,
              eligibleNodes,
            ).first;
    }
    if (found != null) {
      _pushPolicyData(direction, nearestScope, focusedChild);
      switch (direction) {
        case TraversalDirection.up:
        case TraversalDirection.left:
          requestFocusCallback(
            found,
            alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
          );
        case TraversalDirection.down:
        case TraversalDirection.right:
          requestFocusCallback(
            found,
            alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
          );
      }
      return true;
    }
    return false;
  }
}

/// A [FocusTraversalPolicy] that traverses the focus order in widget hierarchy
/// order.
///
/// This policy is used when the order desired is the order in which widgets are
/// created in the widget hierarchy.
///
/// See also:
///
///  * [FocusNode], for a description of the focus system.
///  * [FocusTraversalGroup], a widget that groups together and imposes a
///    traversal policy on the [Focus] nodes below it in the widget hierarchy.
///  * [ReadingOrderTraversalPolicy], a policy that describes the order as the
///    natural "reading order" for the current [Directionality].
///  * [DirectionalFocusTraversalPolicyMixin] a mixin class that implements
///    focus traversal in a direction.
///  * [OrderedTraversalPolicy], a policy that describes the order
///    explicitly using [FocusTraversalOrder] widgets.
class WidgetOrderTraversalPolicy extends FocusTraversalPolicy
    with DirectionalFocusTraversalPolicyMixin {
  /// Constructs a traversal policy that orders widgets for keyboard traversal
  /// based on the widget hierarchy order.
  ///
  /// {@macro flutter.widgets.FocusTraversalPolicy.requestFocusCallback}
  WidgetOrderTraversalPolicy({super.requestFocusCallback});
  @override
  Iterable<FocusNode> sortDescendants(Iterable<FocusNode> descendants, FocusNode currentNode) =>
      descendants;
}

// This class exists mainly for efficiency reasons: the rect is copied out of
// the node, because it will be accessed many times in the reading order
// algorithm, and the FocusNode.rect accessor does coordinate transformation. If
// not for this optimization, it could just be removed, and the node used
// directly.
//
// It's also a convenient place to put some utility functions having to do with
// the sort data.
class _ReadingOrderSortData with Diagnosticable {
  _ReadingOrderSortData(this.node)
    : rect = node.rect,
      directionality = _findDirectionality(node.context!);

  final TextDirection? directionality;
  final Rect rect;
  final FocusNode node;

  // Find the directionality in force for a build context without creating a
  // dependency.
  static TextDirection? _findDirectionality(BuildContext context) {
    return context.getInheritedWidgetOfExactType<Directionality>()?.textDirection;
  }

  /// Finds the common Directional ancestor of an entire list of groups.
  static TextDirection? commonDirectionalityOf(List<_ReadingOrderSortData> list) {
    final Iterable<Set<Directionality>> allAncestors = list.map<Set<Directionality>>(
      (_ReadingOrderSortData member) => member.directionalAncestors.toSet(),
    );
    Set<Directionality>? common;
    for (final Set<Directionality> ancestorSet in allAncestors) {
      common ??= ancestorSet;
      common = common.intersection(ancestorSet);
    }
    if (common!.isEmpty) {
      // If there is no common ancestor, then arbitrarily pick the
      // directionality of the first group, which is the equivalent of the
      // "first strongly typed" item in a bidirectional algorithm.
      return list.first.directionality;
    }
    // Find the closest common ancestor. The memberAncestors list contains the
    // ancestors for all members, but the first member's ancestry was
    // added in order from nearest to furthest, so we can still use that
    // to determine the closest one.
    return list.first.directionalAncestors.firstWhere(common.contains).textDirection;
  }

  static void sortWithDirectionality(
    List<_ReadingOrderSortData> list,
    TextDirection directionality,
  ) {
    mergeSort<_ReadingOrderSortData>(
      list,
      compare:
          (_ReadingOrderSortData a, _ReadingOrderSortData b) => switch (directionality) {
            TextDirection.ltr => a.rect.left.compareTo(b.rect.left),
            TextDirection.rtl => b.rect.right.compareTo(a.rect.right),
          },
    );
  }

  /// Returns the list of Directionality ancestors, in order from nearest to
  /// furthest.
  Iterable<Directionality> get directionalAncestors {
    List<Directionality> getDirectionalityAncestors(BuildContext context) {
      final List<Directionality> result = <Directionality>[];
      InheritedElement? directionalityElement =
          context.getElementForInheritedWidgetOfExactType<Directionality>();
      while (directionalityElement != null) {
        result.add(directionalityElement.widget as Directionality);
        directionalityElement =
            _getAncestor(
              directionalityElement,
            )?.getElementForInheritedWidgetOfExactType<Directionality>();
      }
      return result;
    }

    _directionalAncestors ??= getDirectionalityAncestors(node.context!);
    return _directionalAncestors!;
  }

  List<Directionality>? _directionalAncestors;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<TextDirection>('directionality', directionality));
    properties.add(StringProperty('name', node.debugLabel, defaultValue: null));
    properties.add(DiagnosticsProperty<Rect>('rect', rect));
  }
}

// A class for containing group data while sorting in reading order while taking
// into account the ambient directionality.
class _ReadingOrderDirectionalGroupData with Diagnosticable {
  _ReadingOrderDirectionalGroupData(this.members);

  final List<_ReadingOrderSortData> members;

  TextDirection? get directionality => members.first.directionality;

  Rect? _rect;
  Rect get rect {
    if (_rect == null) {
      for (final Rect rect in members.map<Rect>((_ReadingOrderSortData data) => data.rect)) {
        _rect ??= rect;
        _rect = _rect!.expandToInclude(rect);
      }
    }
    return _rect!;
  }

  List<Directionality> get memberAncestors {
    if (_memberAncestors == null) {
      _memberAncestors = <Directionality>[];
      for (final _ReadingOrderSortData member in members) {
        _memberAncestors!.addAll(member.directionalAncestors);
      }
    }
    return _memberAncestors!;
  }

  List<Directionality>? _memberAncestors;

  static void sortWithDirectionality(
    List<_ReadingOrderDirectionalGroupData> list,
    TextDirection directionality,
  ) {
    mergeSort<_ReadingOrderDirectionalGroupData>(
      list,
      compare:
          (_ReadingOrderDirectionalGroupData a, _ReadingOrderDirectionalGroupData b) =>
              switch (directionality) {
                TextDirection.ltr => a.rect.left.compareTo(b.rect.left),
                TextDirection.rtl => b.rect.right.compareTo(a.rect.right),
              },
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<TextDirection>('directionality', directionality));
    properties.add(DiagnosticsProperty<Rect>('rect', rect));
    properties.add(
      IterableProperty<String>(
        'members',
        members.map<String>((_ReadingOrderSortData member) {
          return '"${member.node.debugLabel}"(${member.rect})';
        }),
      ),
    );
  }
}

/// Traverses the focus order in "reading order".
///
/// By default, reading order traversal goes in the reading direction, and then
/// down, using this algorithm:
///
/// 1. Find the node rectangle that has the highest `top` on the screen.
/// 2. Find any other nodes that intersect the infinite horizontal band defined
///    by the highest rectangle's top and bottom edges.
/// 3. Pick the closest to the beginning of the reading order from among the
///    nodes discovered above.
///
/// It uses the ambient [Directionality] in the context for the enclosing
/// [FocusTraversalGroup] to determine which direction is "reading order".
///
/// See also:
///
///  * [FocusNode], for a description of the focus system.
///  * [FocusTraversalGroup], a widget that groups together and imposes a
///    traversal policy on the [Focus] nodes below it in the widget hierarchy.
///  * [WidgetOrderTraversalPolicy], a policy that relies on the widget
///    creation order to describe the order of traversal.
///  * [DirectionalFocusTraversalPolicyMixin] a mixin class that implements
///    focus traversal in a direction.
///  * [OrderedTraversalPolicy], a policy that describes the order
///    explicitly using [FocusTraversalOrder] widgets.
class ReadingOrderTraversalPolicy extends FocusTraversalPolicy
    with DirectionalFocusTraversalPolicyMixin {
  /// Constructs a traversal policy that orders the widgets in "reading order".
  ///
  /// {@macro flutter.widgets.FocusTraversalPolicy.requestFocusCallback}
  ReadingOrderTraversalPolicy({super.requestFocusCallback});

  // Collects the given candidates into groups by directionality. The candidates
  // have already been sorted as if they all had the directionality of the
  // nearest Directionality ancestor.
  List<_ReadingOrderDirectionalGroupData> _collectDirectionalityGroups(
    Iterable<_ReadingOrderSortData> candidates,
  ) {
    TextDirection? currentDirection = candidates.first.directionality;
    List<_ReadingOrderSortData> currentGroup = <_ReadingOrderSortData>[];
    final List<_ReadingOrderDirectionalGroupData> result = <_ReadingOrderDirectionalGroupData>[];
    // Split candidates into runs of the same directionality.
    for (final _ReadingOrderSortData candidate in candidates) {
      if (candidate.directionality == currentDirection) {
        currentGroup.add(candidate);
        continue;
      }
      currentDirection = candidate.directionality;
      result.add(_ReadingOrderDirectionalGroupData(currentGroup));
      currentGroup = <_ReadingOrderSortData>[candidate];
    }
    if (currentGroup.isNotEmpty) {
      result.add(_ReadingOrderDirectionalGroupData(currentGroup));
    }
    // Sort each group separately. Each group has the same directionality.
    for (final _ReadingOrderDirectionalGroupData bandGroup in result) {
      if (bandGroup.members.length == 1) {
        continue; // No need to sort one node.
      }
      _ReadingOrderSortData.sortWithDirectionality(bandGroup.members, bandGroup.directionality!);
    }
    return result;
  }

  _ReadingOrderSortData _pickNext(List<_ReadingOrderSortData> candidates) {
    // Find the topmost node by sorting on the top of the rectangles.
    mergeSort<_ReadingOrderSortData>(
      candidates,
      compare:
          (_ReadingOrderSortData a, _ReadingOrderSortData b) => a.rect.top.compareTo(b.rect.top),
    );
    final _ReadingOrderSortData topmost = candidates.first;

    // Find the candidates that are in the same horizontal band as the current one.
    List<_ReadingOrderSortData> inBand(
      _ReadingOrderSortData current,
      Iterable<_ReadingOrderSortData> candidates,
    ) {
      final Rect band = Rect.fromLTRB(
        double.negativeInfinity,
        current.rect.top,
        double.infinity,
        current.rect.bottom,
      );
      return candidates.where((_ReadingOrderSortData item) {
        return !item.rect.intersect(band).isEmpty;
      }).toList();
    }

    final List<_ReadingOrderSortData> inBandOfTop = inBand(topmost, candidates);
    // It has to have at least topmost in it if the topmost is not degenerate.
    assert(topmost.rect.isEmpty || inBandOfTop.isNotEmpty);

    // The topmost rect is in a band by itself, so just return that one.
    if (inBandOfTop.length <= 1) {
      return topmost;
    }

    // Now that we know there are others in the same band as the topmost, then pick
    // the one at the beginning, depending on the text direction in force.

    // Find out the directionality of the nearest common Directionality
    // ancestor for all nodes. This provides a base directionality to use for
    // the ordering of the groups.
    final TextDirection? nearestCommonDirectionality = _ReadingOrderSortData.commonDirectionalityOf(
      inBandOfTop,
    );

    // Do an initial common-directionality-based sort to get consistent geometric
    // ordering for grouping into directionality groups. It has to use the
    // common directionality to be able to group into sane groups for the
    // given directionality, since rectangles can overlap and give different
    // results for different directionalities.
    _ReadingOrderSortData.sortWithDirectionality(inBandOfTop, nearestCommonDirectionality!);

    // Collect the top band into internally sorted groups with shared directionality.
    final List<_ReadingOrderDirectionalGroupData> bandGroups = _collectDirectionalityGroups(
      inBandOfTop,
    );
    if (bandGroups.length == 1) {
      // There's only one directionality group, so just send back the first
      // one in that group, since it's already sorted.
      return bandGroups.first.members.first;
    }

    // Sort the groups based on the common directionality and bounding boxes.
    _ReadingOrderDirectionalGroupData.sortWithDirectionality(
      bandGroups,
      nearestCommonDirectionality,
    );
    return bandGroups.first.members.first;
  }

  // Sorts the list of nodes based on their geometry into the desired reading
  // order based on the directionality of the context for each node.
  @override
  Iterable<FocusNode> sortDescendants(Iterable<FocusNode> descendants, FocusNode currentNode) {
    if (descendants.length <= 1) {
      return descendants;
    }

    final List<_ReadingOrderSortData> data = <_ReadingOrderSortData>[
      for (final FocusNode node in descendants) _ReadingOrderSortData(node),
    ];

    final List<FocusNode> sortedList = <FocusNode>[];
    final List<_ReadingOrderSortData> unplaced = data;

    // Pick the initial widget as the one that is at the beginning of the band
    // of the topmost, or the topmost, if there are no others in its band.
    _ReadingOrderSortData current = _pickNext(unplaced);
    sortedList.add(current.node);
    unplaced.remove(current);

    // Go through each node, picking the next one after eliminating the previous
    // one, since removing the previously picked node will expose a new band in
    // which to choose candidates.
    while (unplaced.isNotEmpty) {
      final _ReadingOrderSortData next = _pickNext(unplaced);
      current = next;
      sortedList.add(current.node);
      unplaced.remove(current);
    }
    return sortedList;
  }
}

/// Base class for all sort orders for [OrderedTraversalPolicy] traversal.
///
/// {@template flutter.widgets.FocusOrder.comparable}
/// Only orders of the same type are comparable. If a set of widgets in the same
/// [FocusTraversalGroup] contains orders that are not comparable with each
/// other, it will assert, since the ordering between such keys is undefined. To
/// avoid collisions, use a [FocusTraversalGroup] to group similarly ordered
/// widgets together.
///
/// When overriding, [FocusOrder.doCompare] must be overridden instead of
/// [FocusOrder.compareTo], which calls [FocusOrder.doCompare] to do the actual
/// comparison.
/// {@endtemplate}
///
/// See also:
///
/// * [FocusTraversalGroup], a widget that groups together and imposes a
///   traversal policy on the [Focus] nodes below it in the widget hierarchy.
/// * [FocusTraversalOrder], a widget that assigns an order to a widget subtree
///   for the [OrderedTraversalPolicy] to use.
/// * [NumericFocusOrder], for a focus order that describes its order with a
///   `double`.
/// * [LexicalFocusOrder], a focus order that assigns a string-based lexical
///   traversal order to a [FocusTraversalOrder] widget.
@immutable
abstract class FocusOrder with Diagnosticable implements Comparable<FocusOrder> {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const FocusOrder();

  /// Compares this object to another [Comparable].
  ///
  /// When overriding [FocusOrder], implement [doCompare] instead of this
  /// function to do the actual comparison.
  ///
  /// Returns a value like a [Comparator] when comparing `this` to [other].
  /// That is, it returns a negative integer if `this` is ordered before [other],
  /// a positive integer if `this` is ordered after [other],
  /// and zero if `this` and [other] are ordered together.
  ///
  /// The [other] argument must be a value that is comparable to this object.
  @override
  @nonVirtual
  int compareTo(FocusOrder other) {
    assert(
      runtimeType == other.runtimeType,
      "The sorting algorithm must not compare incomparable keys, since they don't "
      'know how to order themselves relative to each other. Comparing $this with $other',
    );
    return doCompare(other);
  }

  /// The subclass implementation called by [compareTo] to compare orders.
  ///
  /// The argument is guaranteed to be of the same [runtimeType] as this object.
  ///
  /// The method should return a negative number if this object comes earlier in
  /// the sort order than the `other` argument; and a positive number if it
  /// comes later in the sort order than `other`. Returning zero causes the
  /// system to fall back to the secondary sort order defined by
  /// [OrderedTraversalPolicy.secondary]
  @protected
  int doCompare(covariant FocusOrder other);
}

/// Can be given to a [FocusTraversalOrder] widget to assign a numerical order
/// to a widget subtree that is using a [OrderedTraversalPolicy] to define the
/// order in which widgets should be traversed with the keyboard.
///
/// {@macro flutter.widgets.FocusOrder.comparable}
///
/// See also:
///
///  * [FocusTraversalOrder], a widget that assigns an order to a widget subtree
///    for the [OrderedTraversalPolicy] to use.
class NumericFocusOrder extends FocusOrder {
  /// Creates an object that describes a focus traversal order numerically.
  const NumericFocusOrder(this.order);

  /// The numerical order to assign to the widget subtree using
  /// [FocusTraversalOrder].
  ///
  /// Determines the placement of this widget in a sequence of widgets that defines
  /// the order in which this node is traversed by the focus policy.
  ///
  /// Lower values will be traversed first.
  final double order;

  @override
  int doCompare(NumericFocusOrder other) => order.compareTo(other.order);

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('order', order));
  }
}

/// Can be given to a [FocusTraversalOrder] widget to use a String to assign a
/// lexical order to a widget subtree that is using a
/// [OrderedTraversalPolicy] to define the order in which widgets should be
/// traversed with the keyboard.
///
/// This sorts strings using Dart's default string comparison, which is not
/// locale-specific.
///
/// {@macro flutter.widgets.FocusOrder.comparable}
///
/// See also:
///
///  * [FocusTraversalOrder], a widget that assigns an order to a widget subtree
///    for the [OrderedTraversalPolicy] to use.
class LexicalFocusOrder extends FocusOrder {
  /// Creates an object that describes a focus traversal order lexically.
  const LexicalFocusOrder(this.order);

  /// The String that defines the lexical order to assign to the widget subtree
  /// using [FocusTraversalOrder].
  ///
  /// Determines the placement of this widget in a sequence of widgets that defines
  /// the order in which this node is traversed by the focus policy.
  ///
  /// Lower lexical values will be traversed first (e.g. 'a' comes before 'z').
  final String order;

  @override
  int doCompare(LexicalFocusOrder other) => order.compareTo(other.order);

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(StringProperty('order', order));
  }
}

// Used to help sort the focus nodes in an OrderedFocusTraversalPolicy.
class _OrderedFocusInfo {
  const _OrderedFocusInfo({required this.node, required this.order});

  final FocusNode node;
  final FocusOrder order;
}

/// A [FocusTraversalPolicy] that orders nodes by an explicit order that resides
/// in the nearest [FocusTraversalOrder] widget ancestor.
///
/// {@macro flutter.widgets.FocusOrder.comparable}
///
/// {@tool dartpad}
/// This sample shows how to assign a traversal order to a widget. In the
/// example, the focus order goes from bottom right (the "One" button) to top
/// left (the "Six" button).
///
/// ** See code in examples/api/lib/widgets/focus_traversal/ordered_traversal_policy.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [FocusTraversalGroup], a widget that groups together and imposes a
///    traversal policy on the [Focus] nodes below it in the widget hierarchy.
///  * [WidgetOrderTraversalPolicy], a policy that relies on the widget
///    creation order to describe the order of traversal.
///  * [ReadingOrderTraversalPolicy], a policy that describes the order as the
///    natural "reading order" for the current [Directionality].
///  * [NumericFocusOrder], a focus order that assigns a numeric traversal order
///    to a [FocusTraversalOrder] widget.
///  * [LexicalFocusOrder], a focus order that assigns a string-based lexical
///    traversal order to a [FocusTraversalOrder] widget.
///  * [FocusOrder], an abstract base class for all types of focus traversal
///    orderings.
class OrderedTraversalPolicy extends FocusTraversalPolicy
    with DirectionalFocusTraversalPolicyMixin {
  /// Constructs a traversal policy that orders widgets for keyboard traversal
  /// based on an explicit order.
  ///
  /// If [secondary] is null, it will default to [ReadingOrderTraversalPolicy].
  OrderedTraversalPolicy({this.secondary, super.requestFocusCallback});

  /// This is the policy that is used when a node doesn't have an order
  /// assigned, or when multiple nodes have orders which are identical.
  ///
  /// If not set, this defaults to [ReadingOrderTraversalPolicy].
  ///
  /// This policy determines the secondary sorting order of nodes which evaluate
  /// as having an identical order (including those with no order specified).
  ///
  /// Nodes with no order specified will be sorted after nodes with an explicit
  /// order.
  final FocusTraversalPolicy? secondary;

  @override
  Iterable<FocusNode> sortDescendants(Iterable<FocusNode> descendants, FocusNode currentNode) {
    final FocusTraversalPolicy secondaryPolicy = secondary ?? ReadingOrderTraversalPolicy();
    final Iterable<FocusNode> sortedDescendants = secondaryPolicy.sortDescendants(
      descendants,
      currentNode,
    );
    final List<FocusNode> unordered = <FocusNode>[];
    final List<_OrderedFocusInfo> ordered = <_OrderedFocusInfo>[];
    for (final FocusNode node in sortedDescendants) {
      final FocusOrder? order = FocusTraversalOrder.maybeOf(node.context!);
      if (order != null) {
        ordered.add(_OrderedFocusInfo(node: node, order: order));
      } else {
        unordered.add(node);
      }
    }
    mergeSort<_OrderedFocusInfo>(
      ordered,
      compare: (_OrderedFocusInfo a, _OrderedFocusInfo b) {
        assert(
          a.order.runtimeType == b.order.runtimeType,
          'When sorting nodes for determining focus order, the order (${a.order}) of '
          "node ${a.node}, isn't the same type as the order (${b.order}) of ${b.node}. "
          "Incompatible order types can't be compared. Use a FocusTraversalGroup to group "
          'similar orders together.',
        );
        return a.order.compareTo(b.order);
      },
    );
    return ordered.map<FocusNode>((_OrderedFocusInfo info) => info.node).followedBy(unordered);
  }
}

/// An inherited widget that describes the order in which its child subtree
/// should be traversed.
///
/// {@macro flutter.widgets.FocusOrder.comparable}
///
/// The order for a widget is determined by the [FocusOrder] returned by
/// [FocusTraversalOrder.of] for a particular context.
class FocusTraversalOrder extends InheritedWidget {
  /// Creates an inherited widget used to describe the focus order of
  /// the [child] subtree.
  const FocusTraversalOrder({super.key, required this.order, required super.child});

  /// The order for the widget descendants of this [FocusTraversalOrder].
  final FocusOrder order;

  /// Finds the [FocusOrder] in the nearest ancestor [FocusTraversalOrder] widget.
  ///
  /// It does not create a rebuild dependency because changing the traversal
  /// order doesn't change the widget tree, so nothing needs to be rebuilt as a
  /// result of an order change.
  ///
  /// If no [FocusTraversalOrder] ancestor exists, or the order is null, this
  /// will assert in debug mode, and throw an exception in release mode.
  static FocusOrder of(BuildContext context) {
    final FocusTraversalOrder? marker =
        context.getInheritedWidgetOfExactType<FocusTraversalOrder>();
    assert(() {
      if (marker == null) {
        throw FlutterError(
          'FocusTraversalOrder.of() was called with a context that '
          'does not contain a FocusTraversalOrder widget. No TraversalOrder widget '
          'ancestor could be found starting from the context that was passed to '
          'FocusTraversalOrder.of().\n'
          'The context used was:\n'
          '  $context',
        );
      }
      return true;
    }());
    return marker!.order;
  }

  /// Finds the [FocusOrder] in the nearest ancestor [FocusTraversalOrder] widget.
  ///
  /// It does not create a rebuild dependency because changing the traversal
  /// order doesn't change the widget tree, so nothing needs to be rebuilt as a
  /// result of an order change.
  ///
  /// If no [FocusTraversalOrder] ancestor exists, or the order is null, returns null.
  static FocusOrder? maybeOf(BuildContext context) {
    final FocusTraversalOrder? marker =
        context.getInheritedWidgetOfExactType<FocusTraversalOrder>();
    return marker?.order;
  }

  // Since the order of traversal doesn't affect display of anything, we don't
  // need to force a rebuild of anything that depends upon it.
  @override
  bool updateShouldNotify(InheritedWidget oldWidget) => false;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<FocusOrder>('order', order));
  }
}

/// A widget that describes the inherited focus policy for focus traversal for
/// its descendants, grouping them into a separate traversal group.
///
/// A traversal group is treated as one entity when sorted by the traversal
/// algorithm, so it can be used to segregate different parts of the widget tree
/// that need to be sorted using different algorithms and/or sort orders when
/// using an [OrderedTraversalPolicy].
///
/// Within the group, it will use the given [policy] to order the elements. The
/// group itself will be ordered using the parent group's policy.
///
/// By default, traverses in reading order using [ReadingOrderTraversalPolicy].
///
/// To prevent the members of the group from being focused, set the
/// [descendantsAreFocusable] attribute to false.
///
/// {@tool dartpad}
/// This sample shows three rows of buttons, each grouped by a
/// [FocusTraversalGroup], each with different traversal order policies. Use tab
/// traversal to see the order they are traversed in. The first row follows a
/// numerical order, the second follows a lexical order (ordered to traverse
/// right to left), and the third ignores the numerical order assigned to it and
/// traverses in widget order.
///
/// ** See code in examples/api/lib/widgets/focus_traversal/focus_traversal_group.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [FocusNode], for a description of the focus system.
///  * [WidgetOrderTraversalPolicy], a policy that relies on the widget
///    creation order to describe the order of traversal.
///  * [ReadingOrderTraversalPolicy], a policy that describes the order as the
///    natural "reading order" for the current [Directionality].
///  * [DirectionalFocusTraversalPolicyMixin] a mixin class that implements
///    focus traversal in a direction.
class FocusTraversalGroup extends StatefulWidget {
  /// Creates a [FocusTraversalGroup] object.
  FocusTraversalGroup({
    super.key,
    FocusTraversalPolicy? policy,
    this.descendantsAreFocusable = true,
    this.descendantsAreTraversable = true,
    required this.child,
  }) : policy = policy ?? ReadingOrderTraversalPolicy();

  /// The policy used to move the focus from one focus node to another when
  /// traversing them using a keyboard.
  ///
  /// If not specified, traverses in reading order using
  /// [ReadingOrderTraversalPolicy].
  ///
  /// See also:
  ///
  ///  * [FocusTraversalPolicy] for the API used to impose traversal order
  ///    policy.
  ///  * [WidgetOrderTraversalPolicy] for a traversal policy that traverses
  ///    nodes in the order they are added to the widget tree.
  ///  * [ReadingOrderTraversalPolicy] for a traversal policy that traverses
  ///    nodes in the reading order defined in the widget tree, and then top to
  ///    bottom.
  final FocusTraversalPolicy policy;

  /// {@macro flutter.widgets.Focus.descendantsAreFocusable}
  final bool descendantsAreFocusable;

  /// {@macro flutter.widgets.Focus.descendantsAreTraversable}
  final bool descendantsAreTraversable;

  /// The child widget of this [FocusTraversalGroup].
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// Returns the [FocusTraversalPolicy] that applies to the nearest ancestor of
  /// the given [FocusNode].
  ///
  /// Will return null if no [FocusTraversalPolicy] ancestor applies to the
  /// given [FocusNode].
  ///
  /// The [FocusTraversalPolicy] is set by introducing a [FocusTraversalGroup]
  /// into the widget tree, which will associate a policy with the focus tree
  /// under the nearest ancestor [Focus] widget.
  ///
  /// This function differs from [maybeOf] in that it takes a [FocusNode] and
  /// only traverses the focus tree to determine the policy in effect. Unlike
  /// this function, the [maybeOf] function takes a [BuildContext] and first
  /// walks up the widget tree to find the nearest ancestor [Focus] or
  /// [FocusScope] widget, and then calls this function with the focus node
  /// associated with that widget to determine the policy in effect.
  static FocusTraversalPolicy? maybeOfNode(FocusNode node) {
    return _getGroupNode(node)?.policy;
  }

  static _FocusTraversalGroupNode? _getGroupNode(FocusNode node) {
    while (node.parent != null) {
      if (node.context == null) {
        return null;
      }
      if (node is _FocusTraversalGroupNode) {
        return node;
      }
      node = node.parent!;
    }
    return null;
  }

  /// Returns the [FocusTraversalPolicy] that applies to the [FocusNode] of the
  /// nearest ancestor [Focus] widget, given a [BuildContext].
  ///
  /// Will throw a [FlutterError] in debug mode, and throw a null check
  /// exception in release mode, if no [Focus] ancestor is found, or if no
  /// [FocusTraversalPolicy] applies to the associated [FocusNode].
  ///
  /// {@template flutter.widgets.focus_traversal.FocusTraversalGroup.of}
  /// This function looks up the nearest ancestor [Focus] (or [FocusScope])
  /// widget, and uses its [FocusNode] (or [FocusScopeNode]) to walk up the
  /// focus tree to find the applicable [FocusTraversalPolicy] for that node.
  ///
  /// Calling this function does not create a rebuild dependency because
  /// changing the traversal order doesn't change the widget tree, so nothing
  /// needs to be rebuilt as a result of an order change.
  ///
  /// The [FocusTraversalPolicy] is set by introducing a [FocusTraversalGroup]
  /// into the widget tree, which will associate a policy with the focus tree
  /// under the nearest ancestor [Focus] widget.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  /// * [maybeOf] for a similar function that will return null if no
  ///   [FocusTraversalGroup] ancestor is found.
  /// * [maybeOfNode] for a function that will look for a policy using a given
  ///   [FocusNode], and return null if no policy applies.
  static FocusTraversalPolicy of(BuildContext context) {
    final FocusTraversalPolicy? policy = maybeOf(context);
    assert(() {
      if (policy == null) {
        throw FlutterError(
          'Unable to find a Focus or FocusScope widget in the given context, or the FocusNode '
          'from with the widget that was found is not associated with a FocusTraversalPolicy.\n'
          'FocusTraversalGroup.of() was called with a context that does not contain a '
          'Focus or FocusScope widget, or there was no FocusTraversalPolicy in effect.\n'
          'This can happen if there is not a FocusTraversalGroup that defines the policy, '
          'or if the context comes from a widget that is above the WidgetsApp, MaterialApp, '
          'or CupertinoApp widget (those widgets introduce an implicit default policy) \n'
          'The context used was:\n'
          '  $context',
        );
      }
      return true;
    }());
    return policy!;
  }

  /// Returns the [FocusTraversalPolicy] that applies to the [FocusNode] of the
  /// nearest ancestor [Focus] widget, or null, given a [BuildContext].
  ///
  /// Will return null if it doesn't find an ancestor [Focus] or [FocusScope]
  /// widget, or doesn't find a [FocusTraversalPolicy] that applies to the node.
  ///
  /// {@macro flutter.widgets.focus_traversal.FocusTraversalGroup.of}
  ///
  /// See also:
  ///
  /// * [maybeOfNode] for a similar function that will look for a policy using a
  ///   given [FocusNode].
  /// * [of] for a similar function that will throw if no [FocusTraversalPolicy]
  ///   applies.
  static FocusTraversalPolicy? maybeOf(BuildContext context) {
    final FocusNode? node = Focus.maybeOf(context, scopeOk: true, createDependency: false);
    if (node == null) {
      return null;
    }
    return FocusTraversalGroup.maybeOfNode(node);
  }

  @override
  State<FocusTraversalGroup> createState() => _FocusTraversalGroupState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<FocusTraversalPolicy>('policy', policy));
  }
}

// A special focus node subclass that only FocusTraversalGroup uses so that it
// can be used to cache the policy in the focus tree, and so that the traversal
// code can find groups in the focus tree.
class _FocusTraversalGroupNode extends FocusNode {
  _FocusTraversalGroupNode({super.debugLabel, required this.policy}) {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  FocusTraversalPolicy policy;
}

class _FocusTraversalGroupState extends State<FocusTraversalGroup> {
  // The internal focus node used to collect the children of this node into a
  // group, and to provide a context for the traversal algorithm to sort the
  // group with. It's a special subclass of FocusNode just so that it can be
  // identified when walking the focus tree during traversal, and hold the
  // current policy.
  late final _FocusTraversalGroupNode focusNode = _FocusTraversalGroupNode(
    debugLabel: 'FocusTraversalGroup',
    policy: widget.policy,
  );

  @override
  void dispose() {
    focusNode.dispose();
    super.dispose();
  }

  @override
  void didUpdateWidget(FocusTraversalGroup oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.policy != widget.policy) {
      focusNode.policy = widget.policy;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Focus(
      focusNode: focusNode,
      canRequestFocus: false,
      skipTraversal: true,
      includeSemantics: false,
      descendantsAreFocusable: widget.descendantsAreFocusable,
      descendantsAreTraversable: widget.descendantsAreTraversable,
      child: widget.child,
    );
  }
}

/// An intent for use with the [RequestFocusAction], which supplies the
/// [FocusNode] that should be focused.
class RequestFocusIntent extends Intent {
  /// Creates an intent used with [RequestFocusAction].
  ///
  /// {@macro flutter.widgets.FocusTraversalPolicy.requestFocusCallback}
  const RequestFocusIntent(this.focusNode, {TraversalRequestFocusCallback? requestFocusCallback})
    : requestFocusCallback =
          requestFocusCallback ?? FocusTraversalPolicy.defaultTraversalRequestFocusCallback;

  /// The callback used to move the focus to the node [focusNode].
  /// By default it requests focus on the node and ensures the node is visible
  /// if it's in a scrollable.
  final TraversalRequestFocusCallback requestFocusCallback;

  /// The [FocusNode] that is to be focused.
  final FocusNode focusNode;
}

/// An [Action] that requests the focus on the node it is given in its
/// [RequestFocusIntent].
///
/// This action can be used to request focus for a particular node, by calling
/// [Action.invoke] like so:
///
/// ```dart
/// Actions.invoke(context, RequestFocusIntent(focusNode));
/// ```
///
/// Where the `focusNode` is the node for which the focus will be requested.
///
/// The difference between requesting focus in this way versus calling
/// [FocusNode.requestFocus] directly is that it will use the [Action]
/// registered in the nearest [Actions] widget associated with
/// [RequestFocusIntent] to make the request, rather than just requesting focus
/// directly. This allows the action to have additional side effects, like
/// logging, or undo and redo functionality.
///
/// This [RequestFocusAction] class is the default action associated with the
/// [RequestFocusIntent] in the [WidgetsApp]. It requests focus. You
/// can redefine the associated action with your own [Actions] widget.
///
/// See [FocusTraversalPolicy] for more information about focus traversal.
class RequestFocusAction extends Action<RequestFocusIntent> {
  @override
  void invoke(RequestFocusIntent intent) {
    intent.requestFocusCallback(intent.focusNode);
  }
}

/// An [Intent] bound to [NextFocusAction], which moves the focus to the next
/// focusable node in the focus traversal order.
///
/// See [FocusTraversalPolicy] for more information about focus traversal.
class NextFocusIntent extends Intent {
  /// Creates an intent that is used with [NextFocusAction].
  const NextFocusIntent();
}

/// An [Action] that moves the focus to the next focusable node in the focus
/// order.
///
/// This action is the default action registered for the [NextFocusIntent], and
/// by default is bound to the [LogicalKeyboardKey.tab] key in the [WidgetsApp].
///
/// See [FocusTraversalPolicy] for more information about focus traversal.
class NextFocusAction extends Action<NextFocusIntent> {
  /// Attempts to pass the focus to the next widget.
  ///
  /// Returns true if a widget was focused as a result of invoking this action.
  ///
  /// Returns false when the traversal reached the end and the engine must pass
  /// focus to platform UI.
  @override
  bool invoke(NextFocusIntent intent) {
    return primaryFocus!.nextFocus();
  }

  @override
  KeyEventResult toKeyEventResult(NextFocusIntent intent, bool invokeResult) {
    return invokeResult ? KeyEventResult.handled : KeyEventResult.skipRemainingHandlers;
  }
}

/// An [Intent] bound to [PreviousFocusAction], which moves the focus to the
/// previous focusable node in the focus traversal order.
///
/// See [FocusTraversalPolicy] for more information about focus traversal.
class PreviousFocusIntent extends Intent {
  /// Creates an intent that is used with [PreviousFocusAction].
  const PreviousFocusIntent();
}

/// An [Action] that moves the focus to the previous focusable node in the focus
/// order.
///
/// This action is the default action registered for the [PreviousFocusIntent],
/// and by default is bound to a combination of the [LogicalKeyboardKey.tab] key
/// and the [LogicalKeyboardKey.shift] key in the [WidgetsApp].
///
/// See [FocusTraversalPolicy] for more information about focus traversal.
class PreviousFocusAction extends Action<PreviousFocusIntent> {
  /// Attempts to pass the focus to the previous widget.
  ///
  /// Returns true if a widget was focused as a result of invoking this action.
  ///
  /// Returns false when the traversal reached the beginning and the engine must
  /// pass focus to platform UI.
  @override
  bool invoke(PreviousFocusIntent intent) {
    return primaryFocus!.previousFocus();
  }

  @override
  KeyEventResult toKeyEventResult(PreviousFocusIntent intent, bool invokeResult) {
    return invokeResult ? KeyEventResult.handled : KeyEventResult.skipRemainingHandlers;
  }
}

/// An [Intent] that represents moving to the next focusable node in the given
/// [direction].
///
/// This is the [Intent] bound by default to the [LogicalKeyboardKey.arrowUp],
/// [LogicalKeyboardKey.arrowDown], [LogicalKeyboardKey.arrowLeft], and
/// [LogicalKeyboardKey.arrowRight] keys in the [WidgetsApp], with the
/// appropriate associated directions.
///
/// See [FocusTraversalPolicy] for more information about focus traversal.
class DirectionalFocusIntent extends Intent {
  /// Creates an intent used to move the focus in the given [direction].
  const DirectionalFocusIntent(this.direction, {this.ignoreTextFields = true});

  /// The direction in which to look for the next focusable node when the
  /// associated [DirectionalFocusAction] is invoked.
  final TraversalDirection direction;

  /// If true, then directional focus actions that occur within a text field
  /// will not happen when the focus node which received the key is a text
  /// field.
  ///
  /// Defaults to true.
  final bool ignoreTextFields;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<TraversalDirection>('direction', direction));
  }
}

/// An [Action] that moves the focus to the focusable node in the direction
/// configured by the associated [DirectionalFocusIntent.direction].
///
/// This is the [Action] associated with [DirectionalFocusIntent] and bound by
/// default to the [LogicalKeyboardKey.arrowUp], [LogicalKeyboardKey.arrowDown],
/// [LogicalKeyboardKey.arrowLeft], and [LogicalKeyboardKey.arrowRight] keys in
/// the [WidgetsApp], with the appropriate associated directions.
class DirectionalFocusAction extends Action<DirectionalFocusIntent> {
  /// Creates a [DirectionalFocusAction].
  DirectionalFocusAction() : _isForTextField = false;

  /// Creates a [DirectionalFocusAction] that ignores [DirectionalFocusIntent]s
  /// whose `ignoreTextFields` field is true.
  DirectionalFocusAction.forTextField() : _isForTextField = true;

  // Whether this action is defined in a text field.
  final bool _isForTextField;
  @override
  void invoke(DirectionalFocusIntent intent) {
    if (!intent.ignoreTextFields || !_isForTextField) {
      primaryFocus!.focusInDirection(intent.direction);
    }
  }
}

/// A widget that controls whether or not the descendants of this widget are
/// traversable.
///
/// Does not affect the value of [FocusNode.skipTraversal] of the descendants.
///
/// See also:
///
///  * [Focus], a widget for adding and managing a [FocusNode] in the widget tree.
///  * [ExcludeFocus], a widget that excludes its descendants from focusability.
///  * [FocusTraversalGroup], a widget that groups widgets for focus traversal,
///    and can also be used in the same way as this widget by setting its
///    `descendantsAreFocusable` attribute.
class ExcludeFocusTraversal extends StatelessWidget {
  /// Const constructor for [ExcludeFocusTraversal] widget.
  const ExcludeFocusTraversal({super.key, this.excluding = true, required this.child});

  /// If true, will make this widget's descendants untraversable.
  ///
  /// Defaults to true.
  ///
  /// Does not affect the value of [FocusNode.skipTraversal] on the descendants.
  ///
  /// See also:
  ///
  /// * [Focus.descendantsAreTraversable], the attribute of a [Focus] widget that
  ///   controls this same property for focus widgets.
  /// * [FocusTraversalGroup], a widget used to group together and configure the
  ///   focus traversal policy for a widget subtree that has a
  ///   `descendantsAreFocusable` parameter to conditionally block focus for a
  ///   subtree.
  final bool excluding;

  /// The child widget of this [ExcludeFocusTraversal].
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  @override
  Widget build(BuildContext context) {
    return Focus(
      canRequestFocus: false,
      skipTraversal: true,
      includeSemantics: false,
      descendantsAreTraversable: !excluding,
      child: child,
    );
  }
}
