// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:io';
///
/// @docImport 'package:flutter/scheduler.dart';
/// @docImport 'package:flutter_driver/flutter_driver.dart';
///
/// @docImport 'binding.dart';
/// @docImport 'finders.dart';
/// @docImport 'matchers.dart';
/// @docImport 'widget_tester.dart';
library;

import 'package:clock/clock.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'event_simulation.dart';
import 'finders.dart' as finders;
import 'test_async_utils.dart';
import 'test_pointer.dart';
import 'tree_traversal.dart';
import 'window.dart';

/// The default drag touch slop used to break up a large drag into multiple
/// smaller moves.
///
/// This value must be greater than [kTouchSlop].
const double kDragSlopDefault = 20.0;

// Finds the end index (exclusive) of the span at `startIndex`, or `endIndex` if
// there are no other spans between `startIndex` and `endIndex`.
// The InlineSpan protocol doesn't expose the length of the span so we'll
// have to iterate through the whole range.
(InlineSpan, int)? _findEndOfSpan(InlineSpan rootSpan, int startIndex, int endIndex) {
  assert(endIndex > startIndex);
  final InlineSpan? subspan = rootSpan.getSpanForPosition(TextPosition(offset: startIndex));
  if (subspan == null) {
    return null;
  }
  int i = startIndex + 1;
  while (i < endIndex && rootSpan.getSpanForPosition(TextPosition(offset: i)) == subspan) {
    i += 1;
  }
  return (subspan, i);
}

// Examples can assume:
// typedef MyWidget = Placeholder;

/// Class that programmatically interacts with the [Semantics] tree.
///
/// Allows for testing of the [Semantics] tree, which is used by assistive
/// technology, search engines, and other analysis software to determine the
/// meaning of an application.
///
/// Should be accessed through [WidgetController.semantics]. If no custom
/// implementation is provided, a default [SemanticsController] will be created.
class SemanticsController {
  /// Creates a [SemanticsController] that uses the given binding. Will be
  /// automatically created as part of instantiating a [WidgetController], but
  /// a custom implementation can be passed via the [WidgetController] constructor.
  SemanticsController._(this._controller);

  static final int _scrollingActions =
      SemanticsAction.scrollUp.index |
      SemanticsAction.scrollDown.index |
      SemanticsAction.scrollLeft.index |
      SemanticsAction.scrollRight.index |
      SemanticsAction.scrollToOffset.index;

  /// Based on Android's FOCUSABLE_FLAGS. See [flutter/engine/AccessibilityBridge.java](https://github.com/flutter/engine/blob/main/shell/platform/android/io/flutter/view/AccessibilityBridge.java).
  static final int _importantFlagsForAccessibility =
      SemanticsFlag.hasCheckedState.index |
      SemanticsFlag.hasToggledState.index |
      SemanticsFlag.hasEnabledState.index |
      SemanticsFlag.isButton.index |
      SemanticsFlag.isTextField.index |
      SemanticsFlag.isFocusable.index |
      SemanticsFlag.isSlider.index |
      SemanticsFlag.isInMutuallyExclusiveGroup.index;

  final WidgetController _controller;

  /// Attempts to find the [SemanticsNode] of first result from `finder`.
  ///
  /// If the object identified by the finder doesn't own its semantic node,
  /// this will return the semantics data of the first ancestor with semantics.
  /// The ancestor's semantic data will include the child's as well as
  /// other nodes that have been merged together.
  ///
  /// If the [SemanticsNode] of the object identified by the finder is
  /// force-merged into an ancestor (e.g. via the [MergeSemantics] widget)
  /// the node into which it is merged is returned. That node will include
  /// all the semantics information of the nodes merged into it.
  ///
  /// Will throw a [StateError] if the finder returns more than one element or
  /// if no semantics are found or are not enabled.
  SemanticsNode find(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    final Iterable<Element> candidates = finder.evaluate();
    if (candidates.isEmpty) {
      throw StateError('Finder returned no matching elements.');
    }
    if (candidates.length > 1) {
      throw StateError('Finder returned more than one element.');
    }
    final Element element = candidates.single;
    RenderObject? renderObject = element.findRenderObject();
    SemanticsNode? result = renderObject?.debugSemantics;
    while (renderObject != null && (result == null || result.isMergedIntoParent)) {
      renderObject = renderObject.parent;
      result = renderObject?.debugSemantics;
    }
    if (result == null) {
      throw StateError('No Semantics data found.');
    }
    return result;
  }

  /// Simulates a traversal of the currently visible semantics tree as if by
  /// assistive technologies.
  ///
  /// Starts at the node for `startNode`. If `startNode` is not provided, then
  /// the traversal begins with the first accessible node in the tree. If
  /// `startNode` finds zero elements or more than one element, a [StateError]
  /// will be thrown.
  ///
  /// Ends at the node for `endNode`, inclusive. If `endNode` is not provided,
  /// then the traversal ends with the last accessible node in the currently
  /// available tree. If `endNode` finds zero elements or more than one element,
  /// a [StateError] will be thrown.
  ///
  /// If provided, the nodes for `endNode` and `startNode` must be part of the
  /// same semantics tree, i.e. they must be part of the same view.
  ///
  /// If neither `startNode` or `endNode` is provided, `view` can be provided to
  /// specify the semantics tree to traverse. If `view` is left unspecified,
  /// [WidgetTester.view] is traversed by default.
  ///
  /// Since the order is simulated, edge cases that differ between platforms
  /// (such as how the last visible item in a scrollable list is handled) may be
  /// inconsistent with platform behavior, but are expected to be sufficient for
  /// testing order, availability to assistive technologies, and interactions.
  ///
  /// ## Sample Code
  ///
  /// ```dart
  /// testWidgets('MyWidget', (WidgetTester tester) async {
  ///   await tester.pumpWidget(const MyWidget());
  ///
  ///   expect(
  ///     tester.semantics.simulatedAccessibilityTraversal(),
  ///     containsAllInOrder(<Matcher>[
  ///       containsSemantics(label: 'My Widget'),
  ///       containsSemantics(label: 'is awesome!', isChecked: true),
  ///     ]),
  ///   );
  /// });
  /// ```
  ///
  /// See also:
  ///
  /// * [containsSemantics] and [matchesSemantics], which can be used to match
  ///   against a single node in the traversal.
  /// * [containsAllInOrder], which can be given an [Iterable<Matcher>] to fuzzy
  ///   match the order allowing extra nodes before after and between matching
  ///   parts of the traversal.
  /// * [orderedEquals], which can be given an [Iterable<Matcher>] to exactly
  ///   match the order of the traversal.
  Iterable<SemanticsNode> simulatedAccessibilityTraversal({
    @Deprecated(
      'Use startNode instead. '
      'This method was originally created before semantics finders were available. '
      'Semantics finders avoid edge cases where some nodes are not discoverable by widget finders and should be preferred for semantics testing. '
      'This feature was deprecated after v3.15.0-15.2.pre.',
    )
    finders.FinderBase<Element>? start,
    @Deprecated(
      'Use endNode instead. '
      'This method was originally created before semantics finders were available. '
      'Semantics finders avoid edge cases where some nodes are not discoverable by widget finders and should be preferred for semantics testing. '
      'This feature was deprecated after v3.15.0-15.2.pre.',
    )
    finders.FinderBase<Element>? end,
    finders.FinderBase<SemanticsNode>? startNode,
    finders.FinderBase<SemanticsNode>? endNode,
    FlutterView? view,
  }) {
    TestAsyncUtils.guardSync();
    assert(
      start == null || startNode == null,
      'Cannot provide both start and startNode. Prefer startNode as start is deprecated.',
    );
    assert(
      end == null || endNode == null,
      'Cannot provide both end and endNode. Prefer endNode as end is deprecated.',
    );

    FlutterView? startView;
    if (start != null) {
      startView = _controller.viewOf(start);
      if (view != null && startView != view) {
        throw StateError(
          'The start node is not part of the provided view.\n'
          'Finder: ${start.toString(describeSelf: true)}\n'
          'View of start node: $startView\n'
          'Specified view: $view',
        );
      }
    } else if (startNode != null) {
      final SemanticsOwner owner = startNode.evaluate().single.owner!;
      final RenderView renderView = _controller.binding.renderViews.firstWhere(
        (RenderView render) => render.owner!.semanticsOwner == owner,
      );
      startView = renderView.flutterView;
      if (view != null && startView != view) {
        throw StateError(
          'The start node is not part of the provided view.\n'
          'Finder: ${startNode.toString(describeSelf: true)}\n'
          'View of start node: $startView\n'
          'Specified view: $view',
        );
      }
    }

    FlutterView? endView;
    if (end != null) {
      endView = _controller.viewOf(end);
      if (view != null && endView != view) {
        throw StateError(
          'The end node is not part of the provided view.\n'
          'Finder: ${end.toString(describeSelf: true)}\n'
          'View of end node: $endView\n'
          'Specified view: $view',
        );
      }
    } else if (endNode != null) {
      final SemanticsOwner owner = endNode.evaluate().single.owner!;
      final RenderView renderView = _controller.binding.renderViews.firstWhere(
        (RenderView render) => render.owner!.semanticsOwner == owner,
      );
      endView = renderView.flutterView;
      if (view != null && endView != view) {
        throw StateError(
          'The end node is not part of the provided view.\n'
          'Finder: ${endNode.toString(describeSelf: true)}\n'
          'View of end node: $endView\n'
          'Specified view: $view',
        );
      }
    }

    if (endView != null && startView != null && endView != startView) {
      throw StateError(
        'The start and end node are in different views.\n'
        'Start finder: ${start!.toString(describeSelf: true)}\n'
        'End finder: ${end!.toString(describeSelf: true)}\n'
        'View of start node: $startView\n'
        'View of end node: $endView',
      );
    }

    final FlutterView actualView = view ?? startView ?? endView ?? _controller.view;
    final RenderView renderView = _controller.binding.renderViews.firstWhere(
      (RenderView r) => r.flutterView == actualView,
    );

    final List<SemanticsNode> traversal = <SemanticsNode>[];
    _accessibilityTraversal(renderView.owner!.semanticsOwner!.rootSemanticsNode!, traversal);

    // Setting the range
    SemanticsNode? node;
    String? errorString;

    int startIndex;
    if (start != null) {
      node = find(start);
      startIndex = traversal.indexOf(node);
      errorString = start.toString(describeSelf: true);
    } else if (startNode != null) {
      node = startNode.evaluate().single;
      startIndex = traversal.indexOf(node);
      errorString = startNode.toString(describeSelf: true);
    } else {
      startIndex = 0;
    }
    if (startIndex == -1) {
      throw StateError(
        'The expected starting node was not found.\n'
        'Finder: $errorString\n\n'
        'Expected Start Node: $node\n\n'
        'Traversal: [\n  ${traversal.join('\n  ')}\n]',
      );
    }

    int? endIndex;
    if (end != null) {
      node = find(end);
      endIndex = traversal.indexOf(node);
      errorString = end.toString(describeSelf: true);
    } else if (endNode != null) {
      node = endNode.evaluate().single;
      endIndex = traversal.indexOf(node);
      errorString = endNode.toString(describeSelf: true);
    }
    if (endIndex == -1) {
      throw StateError(
        'The expected ending node was not found.\n'
        'Finder: $errorString\n\n'
        'Expected End Node: $node\n\n'
        'Traversal: [\n  ${traversal.join('\n  ')}\n]',
      );
    }
    endIndex ??= traversal.length - 1;

    return traversal.getRange(startIndex, endIndex + 1);
  }

  /// Recursive depth first traversal of the specified `node`, adding nodes
  /// that are important for semantics to the `traversal` list.
  void _accessibilityTraversal(SemanticsNode node, List<SemanticsNode> traversal) {
    if (_isImportantForAccessibility(node)) {
      traversal.add(node);
    }

    final List<SemanticsNode> children = node.debugListChildrenInOrder(
      DebugSemanticsDumpOrder.traversalOrder,
    );
    for (final SemanticsNode child in children) {
      _accessibilityTraversal(child, traversal);
    }
  }

  /// Whether or not the node is important for semantics. Should match most cases
  /// on the platforms, but certain edge cases will be inconsistent.
  ///
  /// Based on:
  ///
  /// * [flutter/engine/AccessibilityBridge.java#SemanticsNode.isFocusable()](https://github.com/flutter/engine/blob/main/shell/platform/android/io/flutter/view/AccessibilityBridge.java#L2641)
  /// * [flutter/engine/SemanticsObject.mm#SemanticsObject.isAccessibilityElement](https://github.com/flutter/engine/blob/main/shell/platform/darwin/ios/framework/Source/SemanticsObject.mm#L449)
  bool _isImportantForAccessibility(SemanticsNode node) {
    if (node.isMergedIntoParent) {
      // If this node is merged, all its information are present on an ancestor
      // node.
      return false;
    }
    final SemanticsData data = node.getSemanticsData();
    // If the node scopes a route, it doesn't matter what other flags/actions it
    // has, it is _not_ important for accessibility, so we short circuit.
    if (data.hasFlag(SemanticsFlag.scopesRoute)) {
      return false;
    }

    final bool hasNonScrollingAction = data.actions & ~_scrollingActions != 0;
    if (hasNonScrollingAction) {
      return true;
    }

    final bool hasImportantFlag = data.flags & _importantFlagsForAccessibility != 0;
    if (hasImportantFlag) {
      return true;
    }

    final bool hasContent = data.label.isNotEmpty || data.value.isNotEmpty || data.hint.isNotEmpty;
    if (hasContent) {
      return true;
    }

    return false;
  }

  /// Performs the given [SemanticsAction] on the [SemanticsNode] found by `finder`.
  ///
  /// If `args` are provided, they will be passed unmodified with the `action`.
  /// The `checkForAction` argument allows for attempting to perform `action` on
  /// `node` even if it doesn't report supporting that action. This is useful
  /// for implicitly supported actions such as [SemanticsAction.showOnScreen].
  void performAction(
    finders.FinderBase<SemanticsNode> finder,
    SemanticsAction action, {
    Object? args,
    bool checkForAction = true,
  }) {
    final SemanticsNode node = finder.evaluate().single;
    if (checkForAction && !node.getSemanticsData().hasAction(action)) {
      throw StateError(
        'The given node does not support $action. If the action is implicitly '
        'supported or an unsupported action is being tested for this node, '
        'set `checkForAction` to false.\n'
        'Node: $node',
      );
    }

    node.owner!.performAction(node.id, action, args);
  }

  /// Performs a [SemanticsAction.tap] action on the [SemanticsNode] found
  /// by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.tap].
  void tap(finders.FinderBase<SemanticsNode> finder) {
    performAction(finder, SemanticsAction.tap);
  }

  /// Performs a [SemanticsAction.longPress] action on the [SemanticsNode] found
  /// by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.longPress].
  void longPress(finders.FinderBase<SemanticsNode> finder) {
    performAction(finder, SemanticsAction.longPress);
  }

  /// Performs a [SemanticsAction.scrollLeft] action on the [SemanticsNode]
  /// found by `scrollable` or the first scrollable node in the default
  /// semantics tree if no `scrollable` is provided.
  ///
  /// Throws a [StateError] if:
  /// * The given `scrollable` returns zero or more than one result.
  /// * The [SemanticsNode] found with `scrollable` does not support
  ///   [SemanticsAction.scrollLeft].
  void scrollLeft({finders.FinderBase<SemanticsNode>? scrollable}) {
    performAction(scrollable ?? finders.find.semantics.scrollable(), SemanticsAction.scrollLeft);
  }

  /// Performs a [SemanticsAction.scrollRight] action on the [SemanticsNode]
  /// found by `scrollable` or the first scrollable node in the default
  /// semantics tree if no `scrollable` is provided.
  ///
  /// Throws a [StateError] if:
  /// * The given `scrollable` returns zero or more than one result.
  /// * The [SemanticsNode] found with `scrollable` does not support
  ///   [SemanticsAction.scrollRight].
  void scrollRight({finders.FinderBase<SemanticsNode>? scrollable}) {
    performAction(scrollable ?? finders.find.semantics.scrollable(), SemanticsAction.scrollRight);
  }

  /// Performs a [SemanticsAction.scrollUp] action on the [SemanticsNode] found
  /// by `scrollable` or the first scrollable node in the default semantics
  /// tree if no `scrollable` is provided.
  ///
  /// Throws a [StateError] if:
  /// * The given `scrollable` returns zero or more than one result.
  /// * The [SemanticsNode] found with `scrollable` does not support
  ///   [SemanticsAction.scrollUp].
  void scrollUp({finders.FinderBase<SemanticsNode>? scrollable}) {
    performAction(scrollable ?? finders.find.semantics.scrollable(), SemanticsAction.scrollUp);
  }

  /// Performs a [SemanticsAction.scrollDown] action on the [SemanticsNode]
  /// found by `scrollable` or the first scrollable node in the default
  /// semantics tree if no `scrollable` is provided.
  ///
  /// Throws a [StateError] if:
  /// * The given `scrollable` returns zero or more than one result.
  /// * The [SemanticsNode] found with `scrollable` does not support
  ///   [SemanticsAction.scrollDown].
  void scrollDown({finders.FinderBase<SemanticsNode>? scrollable}) {
    performAction(scrollable ?? finders.find.semantics.scrollable(), SemanticsAction.scrollDown);
  }

  /// Performs a [SemanticsAction.increase] action on the [SemanticsNode]
  /// found by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.increase].
  void increase(finders.FinderBase<SemanticsNode> finder) {
    performAction(finder, SemanticsAction.increase);
  }

  /// Performs a [SemanticsAction.decrease] action on the [SemanticsNode]
  /// found by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.decrease].
  void decrease(finders.FinderBase<SemanticsNode> finder) {
    performAction(finder, SemanticsAction.decrease);
  }

  /// Performs a [SemanticsAction.showOnScreen] action on the [SemanticsNode]
  /// found by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.showOnScreen].
  void showOnScreen(finders.FinderBase<SemanticsNode> finder) {
    performAction(finder, SemanticsAction.showOnScreen, checkForAction: false);
  }

  /// Performs a [SemanticsAction.moveCursorForwardByCharacter] action on the
  /// [SemanticsNode] found by `finder`.
  ///
  /// If `shouldModifySelection` is true, then the cursor will begin or extend
  /// a selection.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.moveCursorForwardByCharacter].
  void moveCursorForwardByCharacter(
    finders.FinderBase<SemanticsNode> finder, {
    bool shouldModifySelection = false,
  }) {
    performAction(
      finder,
      SemanticsAction.moveCursorForwardByCharacter,
      args: shouldModifySelection,
    );
  }

  /// Performs a [SemanticsAction.moveCursorForwardByWord] action on the
  /// [SemanticsNode] found by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.moveCursorForwardByWord].
  void moveCursorForwardByWord(
    finders.FinderBase<SemanticsNode> finder, {
    bool shouldModifySelection = false,
  }) {
    performAction(finder, SemanticsAction.moveCursorForwardByWord, args: shouldModifySelection);
  }

  /// Performs a [SemanticsAction.moveCursorBackwardByCharacter] action on the
  /// [SemanticsNode] found by `finder`.
  ///
  /// If `shouldModifySelection` is true, then the cursor will begin or extend
  /// a selection.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.moveCursorBackwardByCharacter].
  void moveCursorBackwardByCharacter(
    finders.FinderBase<SemanticsNode> finder, {
    bool shouldModifySelection = false,
  }) {
    performAction(
      finder,
      SemanticsAction.moveCursorBackwardByCharacter,
      args: shouldModifySelection,
    );
  }

  /// Performs a [SemanticsAction.moveCursorBackwardByWord] action on the
  /// [SemanticsNode] found by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.moveCursorBackwardByWord].
  void moveCursorBackwardByWord(
    finders.FinderBase<SemanticsNode> finder, {
    bool shouldModifySelection = false,
  }) {
    performAction(finder, SemanticsAction.moveCursorBackwardByWord, args: shouldModifySelection);
  }

  /// Performs a [SemanticsAction.setText] action on the [SemanticsNode]
  /// found by `finder` using the given `text`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.setText].
  void setText(finders.FinderBase<SemanticsNode> finder, String text) {
    performAction(finder, SemanticsAction.setText, args: text);
  }

  /// Performs a [SemanticsAction.setSelection] action on the [SemanticsNode]
  /// found by `finder`.
  ///
  /// The `base` parameter is the start index of selection, and the `extent`
  /// parameter is the length of the selection. Each value should be limited
  /// between 0 and the length of the found [SemanticsNode]'s `value`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.setSelection].
  void setSelection(
    finders.FinderBase<SemanticsNode> finder, {
    required int base,
    required int extent,
  }) {
    performAction(
      finder,
      SemanticsAction.setSelection,
      args: <String, int>{'base': base, 'extent': extent},
    );
  }

  /// Performs a [SemanticsAction.copy] action on the [SemanticsNode]
  /// found by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.copy].
  void copy(finders.FinderBase<SemanticsNode> finder) {
    performAction(finder, SemanticsAction.copy);
  }

  /// Performs a [SemanticsAction.cut] action on the [SemanticsNode]
  /// found by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.cut].
  void cut(finders.FinderBase<SemanticsNode> finder) {
    performAction(finder, SemanticsAction.cut);
  }

  /// Performs a [SemanticsAction.paste] action on the [SemanticsNode]
  /// found by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.paste].
  void paste(finders.FinderBase<SemanticsNode> finder) {
    performAction(finder, SemanticsAction.paste);
  }

  /// Performs a [SemanticsAction.didGainAccessibilityFocus] action on the
  /// [SemanticsNode] found by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.didGainAccessibilityFocus].
  void didGainAccessibilityFocus(finders.FinderBase<SemanticsNode> finder) {
    performAction(finder, SemanticsAction.didGainAccessibilityFocus);
  }

  /// Performs a [SemanticsAction.didLoseAccessibilityFocus] action on the
  /// [SemanticsNode] found by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.didLoseAccessibilityFocus].
  void didLoseAccessibilityFocus(finders.FinderBase<SemanticsNode> finder) {
    performAction(finder, SemanticsAction.didLoseAccessibilityFocus);
  }

  /// Performs a [SemanticsAction.customAction] action on the
  /// [SemanticsNode] found by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.customAction].
  void customAction(finders.FinderBase<SemanticsNode> finder, CustomSemanticsAction action) {
    performAction(
      finder,
      SemanticsAction.customAction,
      args: CustomSemanticsAction.getIdentifier(action),
    );
  }

  /// Performs a [SemanticsAction.dismiss] action on the [SemanticsNode]
  /// found by `finder`.
  ///
  /// Throws a [StateError] if:
  /// * The given `finder` returns zero or more than one result.
  /// * The [SemanticsNode] found with `finder` does not support
  ///   [SemanticsAction.dismiss].
  void dismiss(finders.FinderBase<SemanticsNode> finder) {
    performAction(finder, SemanticsAction.dismiss);
  }
}

/// Class that programmatically interacts with widgets.
///
/// For a variant of this class suited specifically for unit tests, see
/// [WidgetTester]. For one suitable for live tests on a device, consider
/// [LiveWidgetController].
///
/// Concrete subclasses must implement the [pump] method.
abstract class WidgetController {
  /// Creates a widget controller that uses the given binding.
  WidgetController(this.binding);

  /// A reference to the current instance of the binding.
  final WidgetsBinding binding;

  /// The [TestPlatformDispatcher] that is being used in this test.
  ///
  /// This will be injected into the framework such that calls to
  /// [WidgetsBinding.platformDispatcher] will use this. This allows
  /// users to change platform specific properties for testing.
  ///
  /// See also:
  ///
  ///   * [TestFlutterView] which allows changing view specific properties
  ///     for testing
  ///   * [view] and [viewOf] which are used to find
  ///     [TestFlutterView]s from the widget tree
  TestPlatformDispatcher get platformDispatcher =>
      binding.platformDispatcher as TestPlatformDispatcher;

  /// The [TestFlutterView] provided by default when testing with
  /// [WidgetTester.pumpWidget].
  ///
  /// If the test uses multiple views, this will return the view that is painted
  /// into by [WidgetTester.pumpWidget]. If a different view needs to be
  /// accessed use [viewOf] to ensure that the view related to the widget being
  /// evaluated is the one that gets updated.
  ///
  /// See also:
  ///
  ///   * [viewOf], which can find a [TestFlutterView] related to a given finder.
  ///     This is how to modify view properties for testing when dealing with
  ///     multiple views.
  TestFlutterView get view => platformDispatcher.implicitView!;

  /// Provides access to a [SemanticsController] for testing anything related to
  /// the [Semantics] tree.
  ///
  /// Assistive technologies, search engines, and other analysis tools all make
  /// use of the [Semantics] tree to determine the meaning of an application.
  /// If semantics has been disabled for the test, this will throw a [StateError].
  SemanticsController get semantics {
    if (!binding.semanticsEnabled) {
      throw StateError(
        'Semantics are not enabled. Enable them by passing '
        '`semanticsEnabled: true` to `testWidgets`, or by manually creating a '
        '`SemanticsHandle` with `WidgetController.ensureSemantics()`.',
      );
    }

    return _semantics;
  }

  late final SemanticsController _semantics = SemanticsController._(this);

  // FINDER API

  // TODO(ianh): verify that the return values are of type T and throw
  // a good message otherwise, in all the generic methods below

  /// Finds the [TestFlutterView] that is the closest ancestor of the widget
  /// found by [finder].
  ///
  /// [TestFlutterView] can be used to modify view specific properties for testing.
  ///
  /// See also:
  ///
  ///   * [view] which returns the [TestFlutterView] used when only a single
  ///     view is being used.
  TestFlutterView viewOf(finders.FinderBase<Element> finder) {
    return _viewOf(finder) as TestFlutterView;
  }

  FlutterView _viewOf(finders.FinderBase<Element> finder) {
    return firstWidget<View>(
      finders.find.ancestor(of: finder, matching: finders.find.byType(View)),
    ).view;
  }

  /// Checks if `finder` exists in the tree.
  bool any(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return finder.evaluate().isNotEmpty;
  }

  /// All widgets currently in the widget tree (lazy pre-order traversal).
  ///
  /// Can contain duplicates, since widgets can be used in multiple
  /// places in the widget tree.
  Iterable<Widget> get allWidgets {
    TestAsyncUtils.guardSync();
    return allElements.map<Widget>((Element element) => element.widget);
  }

  /// The matching widget in the widget tree.
  ///
  /// Throws a [StateError] if `finder` is empty or matches more than
  /// one widget.
  ///
  /// * Use [firstWidget] if you expect to match several widgets but only want the first.
  /// * Use [widgetList] if you expect to match several widgets and want all of them.
  T widget<T extends Widget>(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return finder.evaluate().single.widget as T;
  }

  /// The first matching widget according to a depth-first pre-order
  /// traversal of the widget tree.
  ///
  /// Throws a [StateError] if `finder` is empty.
  ///
  /// * Use [widget] if you only expect to match one widget.
  T firstWidget<T extends Widget>(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return finder.evaluate().first.widget as T;
  }

  /// The matching widgets in the widget tree.
  ///
  /// * Use [widget] if you only expect to match one widget.
  /// * Use [firstWidget] if you expect to match several but only want the first.
  Iterable<T> widgetList<T extends Widget>(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return finder.evaluate().map<T>((Element element) {
      final T result = element.widget as T;
      return result;
    });
  }

  /// Find all layers that are children of the provided [finder].
  ///
  /// The [finder] must match exactly one element.
  Iterable<Layer> layerListOf(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    final Element element = finder.evaluate().single;
    final RenderObject object = element.renderObject!;
    RenderObject current = object;
    while (current.debugLayer == null) {
      current = current.parent!;
    }
    final ContainerLayer layer = current.debugLayer!;
    return _walkLayers(layer);
  }

  /// All elements currently in the widget tree (lazy pre-order traversal).
  ///
  /// The returned iterable is lazy. It does not walk the entire widget tree
  /// immediately, but rather a chunk at a time as the iteration progresses
  /// using [Iterator.moveNext].
  Iterable<Element> get allElements {
    TestAsyncUtils.guardSync();
    return collectAllElementsFrom(binding.rootElement!, skipOffstage: false);
  }

  /// The matching element in the widget tree.
  ///
  /// Throws a [StateError] if `finder` is empty or matches more than
  /// one element.
  ///
  /// * Use [firstElement] if you expect to match several elements but only want the first.
  /// * Use [elementList] if you expect to match several elements and want all of them.
  T element<T extends Element>(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return finder.evaluate().single as T;
  }

  /// The first matching element according to a depth-first pre-order
  /// traversal of the widget tree.
  ///
  /// Throws a [StateError] if `finder` is empty.
  ///
  /// * Use [element] if you only expect to match one element.
  T firstElement<T extends Element>(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return finder.evaluate().first as T;
  }

  /// The matching elements in the widget tree.
  ///
  /// * Use [element] if you only expect to match one element.
  /// * Use [firstElement] if you expect to match several but only want the first.
  Iterable<T> elementList<T extends Element>(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return finder.evaluate().cast<T>();
  }

  /// All states currently in the widget tree (lazy pre-order traversal).
  ///
  /// The returned iterable is lazy. It does not walk the entire widget tree
  /// immediately, but rather a chunk at a time as the iteration progresses
  /// using [Iterator.moveNext].
  Iterable<State> get allStates {
    TestAsyncUtils.guardSync();
    return allElements.whereType<StatefulElement>().map<State>(
      (StatefulElement element) => element.state,
    );
  }

  /// The matching state in the widget tree.
  ///
  /// Throws a [StateError] if `finder` is empty, matches more than
  /// one state, or matches a widget that has no state.
  ///
  /// * Use [firstState] if you expect to match several states but only want the first.
  /// * Use [stateList] if you expect to match several states and want all of them.
  T state<T extends State>(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return _stateOf<T>(finder.evaluate().single, finder);
  }

  /// The first matching state according to a depth-first pre-order
  /// traversal of the widget tree.
  ///
  /// Throws a [StateError] if `finder` is empty or if the first
  /// matching widget has no state.
  ///
  /// * Use [state] if you only expect to match one state.
  T firstState<T extends State>(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return _stateOf<T>(finder.evaluate().first, finder);
  }

  /// The matching states in the widget tree.
  ///
  /// Throws a [StateError] if any of the elements in `finder` match a widget
  /// that has no state.
  ///
  /// * Use [state] if you only expect to match one state.
  /// * Use [firstState] if you expect to match several but only want the first.
  Iterable<T> stateList<T extends State>(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return finder.evaluate().map<T>((Element element) => _stateOf<T>(element, finder));
  }

  T _stateOf<T extends State>(Element element, finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    if (element is StatefulElement) {
      return element.state as T;
    }
    throw StateError(
      'Widget of type ${element.widget.runtimeType}, with ${finder.describeMatch(finders.Plurality.many)}, is not a StatefulWidget.',
    );
  }

  /// Render objects of all the widgets currently in the widget tree
  /// (lazy pre-order traversal).
  ///
  /// This will almost certainly include many duplicates since the
  /// render object of a [StatelessWidget] or [StatefulWidget] is the
  /// render object of its child; only [RenderObjectWidget]s have
  /// their own render object.
  Iterable<RenderObject> get allRenderObjects {
    TestAsyncUtils.guardSync();
    return allElements.map<RenderObject>((Element element) => element.renderObject!);
  }

  /// The render object of the matching widget in the widget tree.
  ///
  /// Throws a [StateError] if `finder` is empty or matches more than
  /// one widget (even if they all have the same render object).
  ///
  /// * Use [firstRenderObject] if you expect to match several render objects but only want the first.
  /// * Use [renderObjectList] if you expect to match several render objects and want all of them.
  T renderObject<T extends RenderObject>(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return finder.evaluate().single.renderObject! as T;
  }

  /// The render object of the first matching widget according to a
  /// depth-first pre-order traversal of the widget tree.
  ///
  /// Throws a [StateError] if `finder` is empty.
  ///
  /// * Use [renderObject] if you only expect to match one render object.
  T firstRenderObject<T extends RenderObject>(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return finder.evaluate().first.renderObject! as T;
  }

  /// The render objects of the matching widgets in the widget tree.
  ///
  /// * Use [renderObject] if you only expect to match one render object.
  /// * Use [firstRenderObject] if you expect to match several but only want the first.
  Iterable<T> renderObjectList<T extends RenderObject>(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    return finder.evaluate().map<T>((Element element) {
      final T result = element.renderObject! as T;
      return result;
    });
  }

  /// Returns a list of all the [Layer] objects in the rendering.
  List<Layer> get layers {
    return <Layer>[
      for (final RenderView renderView in binding.renderViews)
        ..._walkLayers(renderView.debugLayer!),
    ];
  }

  Iterable<Layer> _walkLayers(Layer layer) sync* {
    TestAsyncUtils.guardSync();
    yield layer;
    if (layer is ContainerLayer) {
      final ContainerLayer root = layer;
      Layer? child = root.firstChild;
      while (child != null) {
        yield* _walkLayers(child);
        child = child.nextSibling;
      }
    }
  }

  // INTERACTION

  /// Dispatch a pointer down / pointer up sequence at the center of
  /// the given widget, assuming it is exposed.
  ///
  /// {@template flutter.flutter_test.WidgetController.tap.warnIfMissed}
  /// The `warnIfMissed` argument, if true (the default), causes a warning to be
  /// displayed on the console if the specified [Finder] indicates a widget and
  /// location that, were a pointer event to be sent to that location, would not
  /// actually send any events to the widget (e.g. because the widget is
  /// obscured, or the location is off-screen, or the widget is transparent to
  /// pointer events).
  ///
  /// Set the argument to false to silence that warning if you intend to not
  /// actually hit the specified element.
  /// {@endtemplate}
  ///
  /// For example, a test that verifies that tapping a disabled button does not
  /// trigger the button would set `warnIfMissed` to false, because the button
  /// would ignore the tap.
  Future<void> tap(
    finders.FinderBase<Element> finder, {
    int? pointer,
    int buttons = kPrimaryButton,
    bool warnIfMissed = true,
    PointerDeviceKind kind = PointerDeviceKind.touch,
  }) {
    return tapAt(
      getCenter(finder, warnIfMissed: warnIfMissed, callee: 'tap'),
      pointer: pointer,
      buttons: buttons,
      kind: kind,
    );
  }

  /// Dispatch a pointer down / pointer up sequence at a hit-testable
  /// [InlineSpan] (typically a [TextSpan]) within the given text range.
  ///
  /// This method performs a more spatially precise tap action on a piece of
  /// static text, than the widget-based [tap] method.
  ///
  /// The given [Finder] must find one and only one matching substring, and the
  /// substring must be hit-testable (meaning, it must not be off-screen, or be
  /// obscured by other widgets, or in a disabled widget). Otherwise this method
  /// throws a [FlutterError].
  ///
  /// If the target substring contains more than one hit-testable [InlineSpan]s,
  /// [tapOnText] taps on one of them, but does not guarantee which.
  ///
  /// The `pointer` and `button` arguments specify [PointerEvent.pointer] and
  /// [PointerEvent.buttons] of the tap event.
  Future<void> tapOnText(
    finders.FinderBase<finders.TextRangeContext> textRangeFinder, {
    int? pointer,
    int buttons = kPrimaryButton,
  }) {
    final Iterable<finders.TextRangeContext> ranges = textRangeFinder.evaluate();
    if (ranges.isEmpty) {
      throw FlutterError(textRangeFinder.toString());
    }
    if (ranges.length > 1) {
      throw FlutterError(
        '$textRangeFinder. The "tapOnText" method needs a single non-empty TextRange.',
      );
    }
    final Offset? tapLocation = _findHitTestableOffsetIn(ranges.single);
    if (tapLocation == null) {
      final finders.TextRangeContext found = textRangeFinder.evaluate().single;
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('Finder specifies a TextRange that can not receive pointer events.'),
        ErrorDescription('The finder used was: ${textRangeFinder.toString(describeSelf: true)}'),
        ErrorDescription(
          'Found a matching substring in a static text widget, within ${found.textRange}.',
        ),
        ErrorDescription(
          'But the "tapOnText" method could not find a hit-testable Offset with in that text range.',
        ),
        found.renderObject.toDiagnosticsNode(
          name: 'The RenderBox of that static text widget was',
          style: DiagnosticsTreeStyle.shallow,
        ),
      ]);
    }
    return tapAt(tapLocation, pointer: pointer, buttons: buttons);
  }

  /// Dispatch a pointer down / pointer up sequence at the given location.
  Future<void> tapAt(
    Offset location, {
    int? pointer,
    int buttons = kPrimaryButton,
    PointerDeviceKind kind = PointerDeviceKind.touch,
  }) {
    return TestAsyncUtils.guard<void>(() async {
      final TestGesture gesture = await startGesture(
        location,
        pointer: pointer,
        buttons: buttons,
        kind: kind,
      );
      await gesture.up();
    });
  }

  /// Dispatch a pointer down at the center of the given widget, assuming it is
  /// exposed.
  ///
  /// {@macro flutter.flutter_test.WidgetController.tap.warnIfMissed}
  ///
  /// The return value is a [TestGesture] object that can be used to continue the
  /// gesture (e.g. moving the pointer or releasing it).
  ///
  /// See also:
  ///
  ///  * [tap], which presses and releases a pointer at the given location.
  ///  * [longPress], which presses and releases a pointer with a gap in
  ///    between long enough to trigger the long-press gesture.
  Future<TestGesture> press(
    finders.FinderBase<Element> finder, {
    int? pointer,
    int buttons = kPrimaryButton,
    bool warnIfMissed = true,
    PointerDeviceKind kind = PointerDeviceKind.touch,
  }) {
    return TestAsyncUtils.guard<TestGesture>(() {
      return startGesture(
        getCenter(finder, warnIfMissed: warnIfMissed, callee: 'press'),
        pointer: pointer,
        buttons: buttons,
        kind: kind,
      );
    });
  }

  /// Dispatch a pointer down / pointer up sequence (with a delay of
  /// [kLongPressTimeout] + [kPressTimeout] between the two events) at the
  /// center of the given widget, assuming it is exposed.
  ///
  /// {@macro flutter.flutter_test.WidgetController.tap.warnIfMissed}
  ///
  /// For example, consider a widget that, when long-pressed, shows an overlay
  /// that obscures the original widget. A test for that widget might first
  /// long-press that widget with `warnIfMissed` at its default value true, then
  /// later verify that long-pressing the same location (using the same finder)
  /// has no effect (since the widget is now obscured), setting `warnIfMissed`
  /// to false on that second call.
  Future<void> longPress(
    finders.FinderBase<Element> finder, {
    int? pointer,
    int buttons = kPrimaryButton,
    bool warnIfMissed = true,
    PointerDeviceKind kind = PointerDeviceKind.touch,
  }) {
    return longPressAt(
      getCenter(finder, warnIfMissed: warnIfMissed, callee: 'longPress'),
      pointer: pointer,
      buttons: buttons,
      kind: kind,
    );
  }

  /// Dispatch a pointer down / pointer up sequence at the given location with
  /// a delay of [kLongPressTimeout] + [kPressTimeout] between the two events.
  Future<void> longPressAt(
    Offset location, {
    int? pointer,
    int buttons = kPrimaryButton,
    PointerDeviceKind kind = PointerDeviceKind.touch,
  }) {
    return TestAsyncUtils.guard<void>(() async {
      final TestGesture gesture = await startGesture(
        location,
        pointer: pointer,
        buttons: buttons,
        kind: kind,
      );
      await pump(kLongPressTimeout + kPressTimeout);
      await gesture.up();
    });
  }

  /// Attempts a fling gesture starting from the center of the given
  /// widget, moving the given distance, reaching the given speed.
  ///
  /// {@macro flutter.flutter_test.WidgetController.tap.warnIfMissed}
  ///
  /// {@template flutter.flutter_test.WidgetController.fling.offset}
  /// The `offset` represents a distance the pointer moves in the global
  /// coordinate system of the screen.
  ///
  /// Positive [Offset.dy] values mean the pointer moves downward. Negative
  /// [Offset.dy] values mean the pointer moves upwards. Accordingly, positive
  /// [Offset.dx] values mean the pointer moves towards the right. Negative
  /// [Offset.dx] values mean the pointer moves towards left.
  /// {@endtemplate}
  ///
  /// {@template flutter.flutter_test.WidgetController.fling}
  /// This can pump frames.
  ///
  /// Exactly 50 pointer events are synthesized.
  ///
  /// The `speed` is in pixels per second in the direction given by `offset`.
  ///
  /// The `offset` and `speed` control the interval between each pointer event.
  /// For example, if the `offset` is 200 pixels down, and the `speed` is 800
  /// pixels per second, the pointer events will be sent for each increment
  /// of 4 pixels (200/50), over 250ms (200/800), meaning events will be sent
  /// every 1.25ms (250/200).
  ///
  /// To make tests more realistic, frames may be pumped during this time (using
  /// calls to [pump]). If the total duration is longer than `frameInterval`,
  /// then one frame is pumped each time that amount of time elapses while
  /// sending events, or each time an event is synthesized, whichever is rarer.
  ///
  /// See [LiveTestWidgetsFlutterBindingFramePolicy.benchmarkLive] if the method
  /// is used in a live environment and accurate time control is important.
  ///
  /// The `initialOffset` argument, if non-zero, causes the pointer to first
  /// apply that offset, then pump a delay of `initialOffsetDelay`. This can be
  /// used to simulate a drag followed by a fling, including dragging in the
  /// opposite direction of the fling (e.g. dragging 200 pixels to the right,
  /// then fling to the left over 200 pixels, ending at the exact point that the
  /// drag started).
  /// {@endtemplate}
  ///
  /// A fling is essentially a drag that ends at a particular speed. If you
  /// just want to drag and end without a fling, use [drag].
  Future<void> fling(
    finders.FinderBase<Element> finder,
    Offset offset,
    double speed, {
    int? pointer,
    int buttons = kPrimaryButton,
    Duration frameInterval = const Duration(milliseconds: 16),
    Offset initialOffset = Offset.zero,
    Duration initialOffsetDelay = const Duration(seconds: 1),
    bool warnIfMissed = true,
    PointerDeviceKind deviceKind = PointerDeviceKind.touch,
  }) {
    return flingFrom(
      getCenter(finder, warnIfMissed: warnIfMissed, callee: 'fling'),
      offset,
      speed,
      pointer: pointer,
      buttons: buttons,
      frameInterval: frameInterval,
      initialOffset: initialOffset,
      initialOffsetDelay: initialOffsetDelay,
      deviceKind: deviceKind,
    );
  }

  /// Attempts a fling gesture starting from the given location, moving the
  /// given distance, reaching the given speed.
  ///
  /// {@macro flutter.flutter_test.WidgetController.fling}
  ///
  /// A fling is essentially a drag that ends at a particular speed. If you
  /// just want to drag and end without a fling, use [dragFrom].
  Future<void> flingFrom(
    Offset startLocation,
    Offset offset,
    double speed, {
    int? pointer,
    int buttons = kPrimaryButton,
    Duration frameInterval = const Duration(milliseconds: 16),
    Offset initialOffset = Offset.zero,
    Duration initialOffsetDelay = const Duration(seconds: 1),
    PointerDeviceKind deviceKind = PointerDeviceKind.touch,
  }) {
    assert(offset.distance > 0.0);
    assert(speed > 0.0); // speed is pixels/second
    return TestAsyncUtils.guard<void>(() async {
      final TestPointer testPointer = TestPointer(
        pointer ?? _getNextPointer(),
        deviceKind,
        null,
        buttons,
      );
      const int kMoveCount =
          50; // Needs to be >= kHistorySize, see _LeastSquaresVelocityTrackerStrategy
      final double timeStampDelta = 1000000.0 * offset.distance / (kMoveCount * speed);
      double timeStamp = 0.0;
      double lastTimeStamp = timeStamp;
      await sendEventToBinding(
        testPointer.down(startLocation, timeStamp: Duration(microseconds: timeStamp.round())),
      );
      if (initialOffset.distance > 0.0) {
        await sendEventToBinding(
          testPointer.move(
            startLocation + initialOffset,
            timeStamp: Duration(microseconds: timeStamp.round()),
          ),
        );
        timeStamp += initialOffsetDelay.inMicroseconds;
        await pump(initialOffsetDelay);
      }
      for (int i = 0; i <= kMoveCount; i += 1) {
        final Offset location =
            startLocation + initialOffset + Offset.lerp(Offset.zero, offset, i / kMoveCount)!;
        await sendEventToBinding(
          testPointer.move(location, timeStamp: Duration(microseconds: timeStamp.round())),
        );
        timeStamp += timeStampDelta;
        if (timeStamp - lastTimeStamp > frameInterval.inMicroseconds) {
          await pump(Duration(microseconds: (timeStamp - lastTimeStamp).truncate()));
          lastTimeStamp = timeStamp;
        }
      }
      await sendEventToBinding(
        testPointer.up(timeStamp: Duration(microseconds: timeStamp.round())),
      );
    });
  }

  /// Attempts a trackpad fling gesture starting from the center of the given
  /// widget, moving the given distance, reaching the given speed. A trackpad
  /// fling sends PointerPanZoom events instead of a sequence of touch events.
  ///
  /// {@macro flutter.flutter_test.WidgetController.tap.warnIfMissed}
  ///
  /// {@macro flutter.flutter_test.WidgetController.fling}
  ///
  /// A fling is essentially a drag that ends at a particular speed. If you
  /// just want to drag and end without a fling, use [drag].
  Future<void> trackpadFling(
    finders.FinderBase<Element> finder,
    Offset offset,
    double speed, {
    int? pointer,
    int buttons = kPrimaryButton,
    Duration frameInterval = const Duration(milliseconds: 16),
    Offset initialOffset = Offset.zero,
    Duration initialOffsetDelay = const Duration(seconds: 1),
    bool warnIfMissed = true,
  }) {
    return trackpadFlingFrom(
      getCenter(finder, warnIfMissed: warnIfMissed, callee: 'fling'),
      offset,
      speed,
      pointer: pointer,
      buttons: buttons,
      frameInterval: frameInterval,
      initialOffset: initialOffset,
      initialOffsetDelay: initialOffsetDelay,
    );
  }

  /// Attempts a fling gesture starting from the given location, moving the
  /// given distance, reaching the given speed. A trackpad fling sends
  /// PointerPanZoom events instead of a sequence of touch events.
  ///
  /// {@macro flutter.flutter_test.WidgetController.fling}
  ///
  /// A fling is essentially a drag that ends at a particular speed. If you
  /// just want to drag and end without a fling, use [dragFrom].
  Future<void> trackpadFlingFrom(
    Offset startLocation,
    Offset offset,
    double speed, {
    int? pointer,
    int buttons = kPrimaryButton,
    Duration frameInterval = const Duration(milliseconds: 16),
    Offset initialOffset = Offset.zero,
    Duration initialOffsetDelay = const Duration(seconds: 1),
  }) {
    assert(offset.distance > 0.0);
    assert(speed > 0.0); // speed is pixels/second
    return TestAsyncUtils.guard<void>(() async {
      final TestPointer testPointer = TestPointer(
        pointer ?? _getNextPointer(),
        PointerDeviceKind.trackpad,
        null,
        buttons,
      );
      const int kMoveCount =
          50; // Needs to be >= kHistorySize, see _LeastSquaresVelocityTrackerStrategy
      final double timeStampDelta = 1000000.0 * offset.distance / (kMoveCount * speed);
      double timeStamp = 0.0;
      double lastTimeStamp = timeStamp;
      await sendEventToBinding(
        testPointer.panZoomStart(
          startLocation,
          timeStamp: Duration(microseconds: timeStamp.round()),
        ),
      );
      if (initialOffset.distance > 0.0) {
        await sendEventToBinding(
          testPointer.panZoomUpdate(
            startLocation,
            pan: initialOffset,
            timeStamp: Duration(microseconds: timeStamp.round()),
          ),
        );
        timeStamp += initialOffsetDelay.inMicroseconds;
        await pump(initialOffsetDelay);
      }
      for (int i = 0; i <= kMoveCount; i += 1) {
        final Offset pan = initialOffset + Offset.lerp(Offset.zero, offset, i / kMoveCount)!;
        await sendEventToBinding(
          testPointer.panZoomUpdate(
            startLocation,
            pan: pan,
            timeStamp: Duration(microseconds: timeStamp.round()),
          ),
        );
        timeStamp += timeStampDelta;
        if (timeStamp - lastTimeStamp > frameInterval.inMicroseconds) {
          await pump(Duration(microseconds: (timeStamp - lastTimeStamp).truncate()));
          lastTimeStamp = timeStamp;
        }
      }
      await sendEventToBinding(
        testPointer.panZoomEnd(timeStamp: Duration(microseconds: timeStamp.round())),
      );
    });
  }

  /// A simulator of how the framework handles a series of [PointerEvent]s
  /// received from the Flutter engine.
  ///
  /// The [PointerEventRecord.timeDelay] is used as the time delay of the events
  /// injection relative to the starting point of the method call.
  ///
  /// Returns a list of the difference between the real delay time when the
  /// [PointerEventRecord.events] are processed and
  /// [PointerEventRecord.timeDelay].
  /// - For [AutomatedTestWidgetsFlutterBinding] where the clock is fake, the
  ///   return value should be exact zeros.
  /// - For [LiveTestWidgetsFlutterBinding], the values are typically small
  /// positives, meaning the event happens a little later than the set time,
  /// but a very small portion may have a tiny negative value for about tens of
  /// microseconds. This is due to the nature of [Future.delayed].
  ///
  /// The closer the return values are to zero the more faithful it is to the
  /// `records`.
  ///
  /// See [PointerEventRecord].
  Future<List<Duration>> handlePointerEventRecord(List<PointerEventRecord> records);

  /// Called to indicate that there should be a new frame after an optional
  /// delay.
  ///
  /// The frame is pumped after a delay of [duration] if [duration] is not null,
  /// or immediately otherwise.
  ///
  /// This is invoked by [flingFrom], for instance, so that the sequence of
  /// pointer events occurs over time.
  ///
  /// The [WidgetTester] subclass implements this by deferring to the [binding].
  ///
  /// See also:
  ///
  ///  * [SchedulerBinding.endOfFrame], which returns a future that could be
  ///    appropriate to return in the implementation of this method.
  Future<void> pump([Duration duration]);

  /// Repeatedly calls [pump] with the given `duration` until there are no
  /// longer any frames scheduled. This will call [pump] at least once, even if
  /// no frames are scheduled when the function is called, to flush any pending
  /// microtasks which may themselves schedule a frame.
  ///
  /// This essentially waits for all animations to have completed.
  ///
  /// If it takes longer that the given `timeout` to settle, then the test will
  /// fail (this method will throw an exception). In particular, this means that
  /// if there is an infinite animation in progress (for example, if there is an
  /// indeterminate progress indicator spinning), this method will throw.
  ///
  /// The default timeout is ten minutes, which is longer than most reasonable
  /// finite animations would last.
  ///
  /// If the function returns, it returns the number of pumps that it performed.
  ///
  /// In general, it is better practice to figure out exactly why each frame is
  /// needed, and then to [pump] exactly as many frames as necessary. This will
  /// help catch regressions where, for instance, an animation is being started
  /// one frame later than it should.
  ///
  /// Alternatively, one can check that the return value from this function
  /// matches the expected number of pumps.
  Future<int> pumpAndSettle([Duration duration = const Duration(milliseconds: 100)]);

  /// Attempts to drag the given widget by the given offset, by
  /// starting a drag in the middle of the widget.
  ///
  /// {@macro flutter.flutter_test.WidgetController.tap.warnIfMissed}
  ///
  /// If you want the drag to end with a speed so that the gesture recognition
  /// system identifies the gesture as a fling, consider using [fling] instead.
  ///
  /// The operation happens at once. If you want the drag to last for a period
  /// of time, consider using [timedDrag].
  ///
  /// {@macro flutter.flutter_test.WidgetController.fling.offset}
  ///
  /// {@template flutter.flutter_test.WidgetController.drag}
  /// By default, if the x or y component of offset is greater than
  /// [kDragSlopDefault], the gesture is broken up into two separate moves
  /// calls. Changing `touchSlopX` or `touchSlopY` will change the minimum
  /// amount of movement in the respective axis before the drag will be broken
  /// into multiple calls. To always send the drag with just a single call to
  /// [TestGesture.moveBy], `touchSlopX` and `touchSlopY` should be set to 0.
  ///
  /// Breaking the drag into multiple moves is necessary for accurate execution
  /// of drag update calls with a [DragStartBehavior] variable set to
  /// [DragStartBehavior.start]. Without such a change, the dragUpdate callback
  /// from a drag recognizer will never be invoked.
  ///
  /// To force this function to a send a single move event, the `touchSlopX` and
  /// `touchSlopY` variables should be set to 0. However, generally, these values
  /// should be left to their default values.
  /// {@endtemplate}
  Future<void> drag(
    finders.FinderBase<Element> finder,
    Offset offset, {
    int? pointer,
    int buttons = kPrimaryButton,
    double touchSlopX = kDragSlopDefault,
    double touchSlopY = kDragSlopDefault,
    bool warnIfMissed = true,
    PointerDeviceKind kind = PointerDeviceKind.touch,
  }) {
    return dragFrom(
      getCenter(finder, warnIfMissed: warnIfMissed, callee: 'drag'),
      offset,
      pointer: pointer,
      buttons: buttons,
      touchSlopX: touchSlopX,
      touchSlopY: touchSlopY,
      kind: kind,
    );
  }

  /// Attempts a drag gesture consisting of a pointer down, a move by
  /// the given offset, and a pointer up.
  ///
  /// If you want the drag to end with a speed so that the gesture recognition
  /// system identifies the gesture as a fling, consider using [flingFrom]
  /// instead.
  ///
  /// The operation happens at once. If you want the drag to last for a period
  /// of time, consider using [timedDragFrom].
  ///
  /// {@macro flutter.flutter_test.WidgetController.drag}
  Future<void> dragFrom(
    Offset startLocation,
    Offset offset, {
    int? pointer,
    int buttons = kPrimaryButton,
    double touchSlopX = kDragSlopDefault,
    double touchSlopY = kDragSlopDefault,
    PointerDeviceKind kind = PointerDeviceKind.touch,
  }) {
    assert(kDragSlopDefault > kTouchSlop);
    return TestAsyncUtils.guard<void>(() async {
      final TestGesture gesture = await startGesture(
        startLocation,
        pointer: pointer,
        buttons: buttons,
        kind: kind,
      );

      final double xSign = offset.dx.sign;
      final double ySign = offset.dy.sign;

      final double offsetX = offset.dx;
      final double offsetY = offset.dy;

      final bool separateX = offset.dx.abs() > touchSlopX && touchSlopX > 0;
      final bool separateY = offset.dy.abs() > touchSlopY && touchSlopY > 0;

      if (separateY || separateX) {
        final double offsetSlope = offsetY / offsetX;
        final double inverseOffsetSlope = offsetX / offsetY;
        final double slopSlope = touchSlopY / touchSlopX;
        final double absoluteOffsetSlope = offsetSlope.abs();
        final double signedSlopX = touchSlopX * xSign;
        final double signedSlopY = touchSlopY * ySign;
        if (absoluteOffsetSlope != slopSlope) {
          // The drag goes through one or both of the extents of the edges of the box.
          if (absoluteOffsetSlope < slopSlope) {
            assert(offsetX.abs() > touchSlopX);
            // The drag goes through the vertical edge of the box.
            // It is guaranteed that the |offsetX| > touchSlopX.
            final double diffY = offsetSlope.abs() * touchSlopX * ySign;

            // The vector from the origin to the vertical edge.
            await gesture.moveBy(Offset(signedSlopX, diffY));
            if (offsetY.abs() <= touchSlopY) {
              // The drag ends on or before getting to the horizontal extension of the horizontal edge.
              await gesture.moveBy(Offset(offsetX - signedSlopX, offsetY - diffY));
            } else {
              final double diffY2 = signedSlopY - diffY;
              final double diffX2 = inverseOffsetSlope * diffY2;

              // The vector from the edge of the box to the horizontal extension of the horizontal edge.
              await gesture.moveBy(Offset(diffX2, diffY2));
              await gesture.moveBy(Offset(offsetX - diffX2 - signedSlopX, offsetY - signedSlopY));
            }
          } else {
            assert(offsetY.abs() > touchSlopY);
            // The drag goes through the horizontal edge of the box.
            // It is guaranteed that the |offsetY| > touchSlopY.
            final double diffX = inverseOffsetSlope.abs() * touchSlopY * xSign;

            // The vector from the origin to the vertical edge.
            await gesture.moveBy(Offset(diffX, signedSlopY));
            if (offsetX.abs() <= touchSlopX) {
              // The drag ends on or before getting to the vertical extension of the vertical edge.
              await gesture.moveBy(Offset(offsetX - diffX, offsetY - signedSlopY));
            } else {
              final double diffX2 = signedSlopX - diffX;
              final double diffY2 = offsetSlope * diffX2;

              // The vector from the edge of the box to the vertical extension of the vertical edge.
              await gesture.moveBy(Offset(diffX2, diffY2));
              await gesture.moveBy(Offset(offsetX - signedSlopX, offsetY - diffY2 - signedSlopY));
            }
          }
        } else {
          // The drag goes through the corner of the box.
          await gesture.moveBy(Offset(signedSlopX, signedSlopY));
          await gesture.moveBy(Offset(offsetX - signedSlopX, offsetY - signedSlopY));
        }
      } else {
        // The drag ends inside the box.
        await gesture.moveBy(offset);
      }
      await gesture.up();
    });
  }

  /// Attempts to drag the given widget by the given offset in the `duration`
  /// time, starting in the middle of the widget.
  ///
  /// {@macro flutter.flutter_test.WidgetController.tap.warnIfMissed}
  ///
  /// {@macro flutter.flutter_test.WidgetController.fling.offset}
  ///
  /// This is the timed version of [drag]. This may or may not result in a
  /// [fling] or ballistic animation, depending on the speed from
  /// `offset/duration`.
  ///
  /// {@template flutter.flutter_test.WidgetController.timedDrag}
  /// The move events are sent at a given `frequency` in Hz (or events per
  /// second). It defaults to 60Hz.
  ///
  /// The movement is linear in time.
  ///
  /// See also [LiveTestWidgetsFlutterBindingFramePolicy.benchmarkLive] for
  /// more accurate time control.
  /// {@endtemplate}
  Future<void> timedDrag(
    finders.FinderBase<Element> finder,
    Offset offset,
    Duration duration, {
    int? pointer,
    int buttons = kPrimaryButton,
    double frequency = 60.0,
    bool warnIfMissed = true,
  }) {
    return timedDragFrom(
      getCenter(finder, warnIfMissed: warnIfMissed, callee: 'timedDrag'),
      offset,
      duration,
      pointer: pointer,
      buttons: buttons,
      frequency: frequency,
    );
  }

  /// Attempts a series of [PointerEvent]s to simulate a drag operation in the
  /// `duration` time.
  ///
  /// This is the timed version of [dragFrom]. This may or may not result in a
  /// [flingFrom] or ballistic animation, depending on the speed from
  /// `offset/duration`.
  ///
  /// {@macro flutter.flutter_test.WidgetController.timedDrag}
  Future<void> timedDragFrom(
    Offset startLocation,
    Offset offset,
    Duration duration, {
    int? pointer,
    int buttons = kPrimaryButton,
    double frequency = 60.0,
  }) {
    assert(frequency > 0);
    final int intervals = duration.inMicroseconds * frequency ~/ 1E6;
    assert(intervals > 1);
    pointer ??= _getNextPointer();
    final List<Duration> timeStamps = <Duration>[
      for (int t = 0; t <= intervals; t += 1) duration * t ~/ intervals,
    ];
    final List<Offset> offsets = <Offset>[
      startLocation,
      for (int t = 0; t <= intervals; t += 1) startLocation + offset * (t / intervals),
    ];
    final List<PointerEventRecord> records = <PointerEventRecord>[
      PointerEventRecord(Duration.zero, <PointerEvent>[
        PointerAddedEvent(position: startLocation),
        PointerDownEvent(position: startLocation, pointer: pointer, buttons: buttons),
      ]),
      ...<PointerEventRecord>[
        for (int t = 0; t <= intervals; t += 1)
          PointerEventRecord(timeStamps[t], <PointerEvent>[
            PointerMoveEvent(
              timeStamp: timeStamps[t],
              position: offsets[t + 1],
              delta: offsets[t + 1] - offsets[t],
              pointer: pointer,
              buttons: buttons,
            ),
          ]),
      ],
      PointerEventRecord(duration, <PointerEvent>[
        PointerUpEvent(
          timeStamp: duration,
          position: offsets.last,
          pointer: pointer,
          // The PointerData received from the engine with
          // change = PointerChange.up, which translates to PointerUpEvent,
          // doesn't provide the button field.
          // buttons: buttons,
        ),
      ]),
    ];
    return TestAsyncUtils.guard<void>(() async {
      await handlePointerEventRecord(records);
    });
  }

  /// The next available pointer identifier.
  ///
  /// This is the default pointer identifier that will be used the next time the
  /// [startGesture] method is called without an explicit pointer identifier.
  int get nextPointer => _nextPointer;

  static int _nextPointer = 1;

  static int _getNextPointer() {
    final int result = _nextPointer;
    _nextPointer += 1;
    return result;
  }

  TestGesture _createGesture({
    int? pointer,
    required PointerDeviceKind kind,
    required int buttons,
  }) {
    return TestGesture(
      dispatcher: sendEventToBinding,
      kind: kind,
      pointer: pointer ?? _getNextPointer(),
      buttons: buttons,
    );
  }

  /// Creates gesture and returns the [TestGesture] object which you can use
  /// to continue the gesture using calls on the [TestGesture] object.
  ///
  /// You can use [startGesture] instead if your gesture begins with a down
  /// event.
  Future<TestGesture> createGesture({
    int? pointer,
    PointerDeviceKind kind = PointerDeviceKind.touch,
    int buttons = kPrimaryButton,
  }) async {
    return _createGesture(pointer: pointer, kind: kind, buttons: buttons);
  }

  /// Creates a gesture with an initial appropriate starting gesture at a
  /// particular point, and returns the [TestGesture] object which you can use
  /// to continue the gesture. Usually, the starting gesture will be a down event,
  /// but if [kind] is set to [PointerDeviceKind.trackpad], the gesture will start
  /// with a panZoomStart gesture.
  ///
  /// You can use [createGesture] if your gesture doesn't begin with an initial
  /// down or panZoomStart gesture.
  ///
  /// See also:
  ///  * [WidgetController.drag], a method to simulate a drag.
  ///  * [WidgetController.timedDrag], a method to simulate the drag of a given
  ///    widget in a given duration. It sends move events at a given frequency and
  ///    it is useful when there are listeners involved.
  ///  * [WidgetController.fling], a method to simulate a fling.
  Future<TestGesture> startGesture(
    Offset downLocation, {
    int? pointer,
    PointerDeviceKind kind = PointerDeviceKind.touch,
    int buttons = kPrimaryButton,
  }) async {
    final TestGesture result = _createGesture(pointer: pointer, kind: kind, buttons: buttons);
    if (kind == PointerDeviceKind.trackpad) {
      await result.panZoomStart(downLocation);
    } else {
      await result.down(downLocation);
    }
    return result;
  }

  /// Forwards the given location to the binding's hitTest logic.
  HitTestResult hitTestOnBinding(Offset location, {int? viewId}) {
    viewId ??= view.viewId;
    final HitTestResult result = HitTestResult();
    binding.hitTestInView(result, location, viewId);
    return result;
  }

  /// Forwards the given pointer event to the binding.
  Future<void> sendEventToBinding(PointerEvent event) {
    return TestAsyncUtils.guard<void>(() async {
      binding.handlePointerEvent(event);
    });
  }

  /// Calls [debugPrint] with the given message.
  ///
  /// This is overridden by the WidgetTester subclass to use the test binding's
  /// [TestWidgetsFlutterBinding.debugPrintOverride], so that it appears on the
  /// console even if the test is logging output from the application.
  @protected
  void printToConsole(String message) {
    debugPrint(message);
  }

  // GEOMETRY

  /// Returns the point at the center of the given widget.
  ///
  /// {@template flutter.flutter_test.WidgetController.getCenter.warnIfMissed}
  /// If `warnIfMissed` is true (the default is false), then the returned
  /// coordinate is checked to see if a hit test at the returned location would
  /// actually include the specified element in the [HitTestResult], and if not,
  /// a warning is printed to the console.
  ///
  /// The `callee` argument is used to identify the method that should be
  /// referenced in messages regarding `warnIfMissed`. It can be ignored unless
  /// this method is being called from another that is forwarding its own
  /// `warnIfMissed` parameter (see e.g. the implementation of [tap]).
  /// {@endtemplate}
  Offset getCenter(
    finders.FinderBase<Element> finder, {
    bool warnIfMissed = false,
    String callee = 'getCenter',
  }) {
    return _getElementPoint(
      finder,
      (Size size) => size.center(Offset.zero),
      warnIfMissed: warnIfMissed,
      callee: callee,
    );
  }

  /// Returns the point at the top left of the given widget.
  ///
  /// {@macro flutter.flutter_test.WidgetController.getCenter.warnIfMissed}
  Offset getTopLeft(
    finders.FinderBase<Element> finder, {
    bool warnIfMissed = false,
    String callee = 'getTopLeft',
  }) {
    return _getElementPoint(
      finder,
      (Size size) => Offset.zero,
      warnIfMissed: warnIfMissed,
      callee: callee,
    );
  }

  /// Returns the point at the top right of the given widget. This
  /// point is not inside the object's hit test area.
  ///
  /// {@macro flutter.flutter_test.WidgetController.getCenter.warnIfMissed}
  Offset getTopRight(
    finders.FinderBase<Element> finder, {
    bool warnIfMissed = false,
    String callee = 'getTopRight',
  }) {
    return _getElementPoint(
      finder,
      (Size size) => size.topRight(Offset.zero),
      warnIfMissed: warnIfMissed,
      callee: callee,
    );
  }

  /// Returns the point at the bottom left of the given widget. This
  /// point is not inside the object's hit test area.
  ///
  /// {@macro flutter.flutter_test.WidgetController.getCenter.warnIfMissed}
  Offset getBottomLeft(
    finders.FinderBase<Element> finder, {
    bool warnIfMissed = false,
    String callee = 'getBottomLeft',
  }) {
    return _getElementPoint(
      finder,
      (Size size) => size.bottomLeft(Offset.zero),
      warnIfMissed: warnIfMissed,
      callee: callee,
    );
  }

  /// Returns the point at the bottom right of the given widget. This
  /// point is not inside the object's hit test area.
  ///
  /// {@macro flutter.flutter_test.WidgetController.getCenter.warnIfMissed}
  Offset getBottomRight(
    finders.FinderBase<Element> finder, {
    bool warnIfMissed = false,
    String callee = 'getBottomRight',
  }) {
    return _getElementPoint(
      finder,
      (Size size) => size.bottomRight(Offset.zero),
      warnIfMissed: warnIfMissed,
      callee: callee,
    );
  }

  /// Whether warnings relating to hit tests not hitting their mark should be
  /// fatal (cause the test to fail).
  ///
  /// Some methods, e.g. [tap], have an argument `warnIfMissed` which causes a
  /// warning to be displayed if the specified [Finder] indicates a widget and
  /// location that, were a pointer event to be sent to that location, would not
  /// actually send any events to the widget (e.g. because the widget is
  /// obscured, or the location is off-screen, or the widget is transparent to
  /// pointer events).
  ///
  /// This warning was added in 2021. In ordinary operation this warning is
  /// non-fatal since making it fatal would be a significantly breaking change
  /// for anyone who already has tests relying on the ability to target events
  /// using finders where the events wouldn't reach the widgets specified by the
  /// finders in question.
  ///
  /// However, doing this is usually unintentional. To make the warning fatal,
  /// thus failing any tests where it occurs, this property can be set to true.
  ///
  /// Typically this is done using a `flutter_test_config.dart` file, as described
  /// in the documentation for the [flutter_test] library.
  static bool hitTestWarningShouldBeFatal = false;

  /// Finds one hit-testable Offset in the given `textRangeContext`'s render
  /// object.
  Offset? _findHitTestableOffsetIn(finders.TextRangeContext textRangeContext) {
    TestAsyncUtils.guardSync();
    final TextRange range = textRangeContext.textRange;
    assert(range.isNormalized);
    assert(range.isValid);
    final Offset renderParagraphPaintOffset = textRangeContext.renderObject.localToGlobal(
      Offset.zero,
    );
    assert(renderParagraphPaintOffset.isFinite);

    int spanStart = range.start;
    while (spanStart < range.end) {
      switch (_findEndOfSpan(textRangeContext.renderObject.text, spanStart, range.end)) {
        case (final HitTestTarget target, final int endIndex):
          // Uses BoxHeightStyle.tight in getBoxesForSelection to make sure the
          // returned boxes don't extend outside of the hit-testable region.
          final Iterable<Offset> testOffsets = textRangeContext.renderObject
              .getBoxesForSelection(TextSelection(baseOffset: spanStart, extentOffset: endIndex))
              // Try hit-testing the center of each TextBox.
              .map((TextBox textBox) => textBox.toRect().center);

          for (final Offset localOffset in testOffsets) {
            final HitTestResult result = HitTestResult();
            final Offset globalOffset = localOffset + renderParagraphPaintOffset;
            binding.hitTestInView(result, globalOffset, textRangeContext.view.view.viewId);
            if (result.path.any((HitTestEntry entry) => entry.target == target)) {
              return globalOffset;
            }
          }
          spanStart = endIndex;
        case (_, final int endIndex):
          spanStart = endIndex;
        case null:
          break;
      }
    }
    return null;
  }

  Offset _getElementPoint(
    finders.FinderBase<Element> finder,
    Offset Function(Size size) sizeToPoint, {
    required bool warnIfMissed,
    required String callee,
  }) {
    TestAsyncUtils.guardSync();
    final Iterable<Element> elements = finder.evaluate();
    if (elements.isEmpty) {
      throw FlutterError(
        'The finder "$finder" (used in a call to "$callee()") could not find any matching widgets.',
      );
    }
    if (elements.length > 1) {
      throw FlutterError(
        'The finder "$finder" (used in a call to "$callee()") ambiguously found multiple matching widgets. The "$callee()" method needs a single target.',
      );
    }
    final Element element = elements.single;
    final RenderObject? renderObject = element.renderObject;
    if (renderObject == null) {
      throw FlutterError(
        'The finder "$finder" (used in a call to "$callee()") found an element, but it does not have a corresponding render object. '
        'Maybe the element has not yet been rendered?',
      );
    }
    if (renderObject is! RenderBox) {
      throw FlutterError(
        'The finder "$finder" (used in a call to "$callee()") found an element whose corresponding render object is not a RenderBox (it is a ${renderObject.runtimeType}: "$renderObject"). '
        'Unfortunately "$callee()" only supports targeting widgets that correspond to RenderBox objects in the rendering.',
      );
    }
    final RenderBox box = element.renderObject! as RenderBox;
    final Offset location = box.localToGlobal(sizeToPoint(box.size));
    if (warnIfMissed) {
      final FlutterView view = _viewOf(finder);
      final HitTestResult result = HitTestResult();
      binding.hitTestInView(result, location, view.viewId);
      final bool found = result.path.any((HitTestEntry entry) => entry.target == box);
      if (!found) {
        final RenderView renderView = binding.renderViews.firstWhere(
          (RenderView r) => r.flutterView == view,
        );
        final bool outOfBounds = !(Offset.zero & renderView.size).contains(location);
        if (hitTestWarningShouldBeFatal) {
          throw FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary('Finder specifies a widget that would not receive pointer events.'),
            ErrorDescription(
              'A call to $callee() with finder "$finder" derived an Offset ($location) that would not hit test on the specified widget.',
            ),
            ErrorHint(
              'Maybe the widget is actually off-screen, or another widget is obscuring it, or the widget cannot receive pointer events.',
            ),
            if (outOfBounds)
              ErrorHint(
                'Indeed, $location is outside the bounds of the root of the render tree, ${renderView.size}.',
              ),
            box.toDiagnosticsNode(
              name: 'The finder corresponds to this RenderBox',
              style: DiagnosticsTreeStyle.singleLine,
            ),
            ErrorDescription('The hit test result at that offset is: $result'),
            ErrorDescription(
              'If you expected this target not to be able to receive pointer events, pass "warnIfMissed: false" to "$callee()".',
            ),
            ErrorDescription(
              'To make this error into a non-fatal warning, set WidgetController.hitTestWarningShouldBeFatal to false.',
            ),
          ]);
        }
        printToConsole(
          '\n'
          'Warning: A call to $callee() with finder "$finder" derived an Offset ($location) that would not hit test on the specified widget.\n'
          'Maybe the widget is actually off-screen, or another widget is obscuring it, or the widget cannot receive pointer events.\n'
          '${outOfBounds ? "Indeed, $location is outside the bounds of the root of the render tree, ${renderView.size}.\n" : ""}'
          'The finder corresponds to this RenderBox: $box\n'
          'The hit test result at that offset is: $result\n'
          '${StackTrace.current}'
          'To silence this warning, pass "warnIfMissed: false" to "$callee()".\n'
          'To make this warning fatal, set WidgetController.hitTestWarningShouldBeFatal to true.\n',
        );
      }
    }
    return location;
  }

  /// Returns the size of the given widget. This is only valid once
  /// the widget's render object has been laid out at least once.
  Size getSize(finders.FinderBase<Element> finder) {
    TestAsyncUtils.guardSync();
    final Element element = finder.evaluate().single;
    final RenderBox box = element.renderObject! as RenderBox;
    return box.size;
  }

  /// Simulates sending physical key down and up events.
  ///
  /// This only simulates key events coming from a physical keyboard, not from a
  /// soft keyboard.
  ///
  /// Specify `platform` as one of the platforms allowed in
  /// [Platform.operatingSystem] to make the event appear to be from
  /// that type of system. If not specified, defaults to "web" on web, and the
  /// operating system name based on [defaultTargetPlatform] everywhere else.
  ///
  /// Specify the `physicalKey` for the event to override what is included in
  /// the simulated event. If not specified, it uses a default from the US
  /// keyboard layout for the corresponding logical `key`.
  ///
  /// Specify the `character` for the event to override what is included in the
  /// simulated event. If not specified, it uses a default derived from the
  /// logical `key`.
  ///
  /// Keys that are down when the test completes are cleared after each test.
  ///
  /// This method sends both the key down and the key up events, to simulate a
  /// key press. To simulate individual down and/or up events, see
  /// [sendKeyDownEvent] and [sendKeyUpEvent].
  ///
  /// Returns true if the key down event was handled by the framework.
  ///
  /// See also:
  ///
  ///  - [sendKeyDownEvent] to simulate only a key down event.
  ///  - [sendKeyUpEvent] to simulate only a key up event.
  Future<bool> sendKeyEvent(
    LogicalKeyboardKey key, {
    String? platform,
    String? character,
    PhysicalKeyboardKey? physicalKey,
  }) async {
    final bool handled = await simulateKeyDownEvent(
      key,
      platform: platform,
      character: character,
      physicalKey: physicalKey,
    );
    // Internally wrapped in async guard.
    await simulateKeyUpEvent(key, platform: platform, physicalKey: physicalKey);
    return handled;
  }

  /// Simulates sending a physical key down event.
  ///
  /// This only simulates key down events coming from a physical keyboard, not
  /// from a soft keyboard.
  ///
  /// Specify `platform` as one of the platforms allowed in
  /// [Platform.operatingSystem] to make the event appear to be from
  /// that type of system. If not specified, defaults to "web" on web, and the
  /// operating system name based on [defaultTargetPlatform] everywhere else.
  ///
  /// Specify the `physicalKey` for the event to override what is included in
  /// the simulated event. If not specified, it uses a default from the US
  /// keyboard layout for the corresponding logical `key`.
  ///
  /// Specify the `character` for the event to override what is included in the
  /// simulated event. If not specified, it uses a default derived from the
  /// logical `key`.
  ///
  /// Keys that are down when the test completes are cleared after each test.
  ///
  /// Returns true if the key event was handled by the framework.
  ///
  /// See also:
  ///
  ///  - [sendKeyUpEvent] and [sendKeyRepeatEvent] to simulate the corresponding
  ///    key up and repeat event.
  ///  - [sendKeyEvent] to simulate both the key up and key down in the same call.
  Future<bool> sendKeyDownEvent(
    LogicalKeyboardKey key, {
    String? platform,
    String? character,
    PhysicalKeyboardKey? physicalKey,
  }) async {
    // Internally wrapped in async guard.
    return simulateKeyDownEvent(
      key,
      platform: platform,
      character: character,
      physicalKey: physicalKey,
    );
  }

  /// Simulates sending a physical key up event through the system channel.
  ///
  /// This only simulates key up events coming from a physical keyboard,
  /// not from a soft keyboard.
  ///
  /// Specify `platform` as one of the platforms allowed in
  /// [Platform.operatingSystem] to make the event appear to be from
  /// that type of system. If not specified, defaults to "web" on web, and the
  /// operating system name based on [defaultTargetPlatform] everywhere else.
  ///
  /// Specify the `physicalKey` for the event to override what is included in
  /// the simulated event. If not specified, it uses a default from the US
  /// keyboard layout for the corresponding logical `key`.
  ///
  /// Returns true if the key event was handled by the framework.
  ///
  /// See also:
  ///
  ///  - [sendKeyDownEvent] and [sendKeyRepeatEvent] to simulate the
  ///    corresponding key down and repeat event.
  ///  - [sendKeyEvent] to simulate both the key up and key down in the same call.
  Future<bool> sendKeyUpEvent(
    LogicalKeyboardKey key, {
    String? platform,
    PhysicalKeyboardKey? physicalKey,
  }) async {
    // Internally wrapped in async guard.
    return simulateKeyUpEvent(key, platform: platform, physicalKey: physicalKey);
  }

  /// Simulates sending a key repeat event from a physical keyboard.
  ///
  /// This only simulates key repeat events coming from a physical keyboard, not
  /// from a soft keyboard.
  ///
  /// Specify `platform` as one of the platforms allowed in
  /// [Platform.operatingSystem] to make the event appear to be from
  /// that type of system. If not specified, defaults to "web" on web, and the
  /// operating system name based on [defaultTargetPlatform] everywhere else.
  ///
  /// Specify the `physicalKey` for the event to override what is included in
  /// the simulated event. If not specified, it uses a default from the US
  /// keyboard layout for the corresponding logical `key`.
  ///
  /// Specify the `character` for the event to override what is included in the
  /// simulated event. If not specified, it uses a default derived from the
  /// logical `key`.
  ///
  /// Keys that are down when the test completes are cleared after each test.
  ///
  /// Returns true if the key event was handled by the framework.
  ///
  /// See also:
  ///
  ///  - [sendKeyDownEvent] and [sendKeyUpEvent] to simulate the corresponding
  ///    key down and up event.
  ///  - [sendKeyEvent] to simulate both the key up and key down in the same call.
  Future<bool> sendKeyRepeatEvent(
    LogicalKeyboardKey key, {
    String? platform,
    String? character,
    PhysicalKeyboardKey? physicalKey,
  }) async {
    // Internally wrapped in async guard.
    return simulateKeyRepeatEvent(
      key,
      platform: platform,
      character: character,
      physicalKey: physicalKey,
    );
  }

  /// Returns the rect of the given widget. This is only valid once
  /// the widget's render object has been laid out at least once.
  Rect getRect(finders.FinderBase<Element> finder) =>
      Rect.fromPoints(getTopLeft(finder), getBottomRight(finder));

  /// Attempts to find the [SemanticsNode] of first result from `finder`.
  ///
  /// If the object identified by the finder doesn't own it's semantic node,
  /// this will return the semantics data of the first ancestor with semantics.
  /// The ancestor's semantic data will include the child's as well as
  /// other nodes that have been merged together.
  ///
  /// If the [SemanticsNode] of the object identified by the finder is
  /// force-merged into an ancestor (e.g. via the [MergeSemantics] widget)
  /// the node into which it is merged is returned. That node will include
  /// all the semantics information of the nodes merged into it.
  ///
  /// Will throw a [StateError] if the finder returns more than one element or
  /// if no semantics are found or are not enabled.
  // TODO(pdblasi-google): Deprecate this and point references to semantics.find. See https://github.com/flutter/flutter/issues/112670.
  SemanticsNode getSemantics(finders.FinderBase<Element> finder) => semantics.find(finder);

  /// Enable semantics in a test by creating a [SemanticsHandle].
  ///
  /// The handle must be disposed at the end of the test.
  SemanticsHandle ensureSemantics() {
    return binding.ensureSemantics();
  }

  /// Given a widget `W` specified by [finder] and a [Scrollable] widget `S` in
  /// its ancestry tree, this scrolls `S` so as to make `W` visible.
  ///
  /// Usually the `finder` for this method should be labeled `skipOffstage:
  /// false`, so that the [Finder] deals with widgets that are off the screen
  /// correctly.
  ///
  /// This does not work when `S` is long enough, and `W` far away enough from
  /// the displayed part of `S`, that `S` has not yet cached `W`'s element.
  /// Consider using [scrollUntilVisible] in such a situation.
  ///
  /// See also:
  ///
  ///  * [Scrollable.ensureVisible], which is the production API used to
  ///    implement this method.
  Future<void> ensureVisible(finders.FinderBase<Element> finder) =>
      Scrollable.ensureVisible(element(finder));

  /// Repeatedly scrolls a [Scrollable] by `delta` in the
  /// [Scrollable.axisDirection] direction until a widget matching `finder` is
  /// visible.
  ///
  /// Between each scroll, advances the clock by `duration` time.
  ///
  /// Scrolling is performed until the start of the `finder` is visible. This is
  /// due to the default parameter values of the [Scrollable.ensureVisible] method.
  ///
  /// If `scrollable` is `null`, a [Finder] that looks for a [Scrollable] is
  /// used instead.
  ///
  /// Throws a [StateError] if `finder` is not found after `maxScrolls` scrolls.
  ///
  /// This is different from [ensureVisible] in that this allows looking for
  /// `finder` that is not yet built. The caller must specify the scrollable
  /// that will build child specified by `finder` when there are multiple
  /// [Scrollable]s.
  ///
  /// See also:
  ///
  ///  * [dragUntilVisible], which implements the body of this method.
  Future<void> scrollUntilVisible(
    finders.FinderBase<Element> finder,
    double delta, {
    finders.FinderBase<Element>? scrollable,
    int maxScrolls = 50,
    Duration duration = const Duration(milliseconds: 50),
  }) {
    assert(maxScrolls > 0);
    scrollable ??= finders.find.byType(Scrollable);
    return TestAsyncUtils.guard<void>(() async {
      Offset moveStep;
      switch (widget<Scrollable>(scrollable!).axisDirection) {
        case AxisDirection.up:
          moveStep = Offset(0, delta);
        case AxisDirection.down:
          moveStep = Offset(0, -delta);
        case AxisDirection.left:
          moveStep = Offset(delta, 0);
        case AxisDirection.right:
          moveStep = Offset(-delta, 0);
      }
      await dragUntilVisible(
        finder,
        scrollable,
        moveStep,
        maxIteration: maxScrolls,
        duration: duration,
      );
    });
  }

  /// Repeatedly drags `view` by `moveStep` until `finder` is visible.
  ///
  /// Between each drag, advances the clock by `duration`.
  ///
  /// Throws a [StateError] if `finder` is not found after `maxIteration`
  /// drags.
  ///
  /// See also:
  ///
  ///  * [scrollUntilVisible], which wraps this method with an API that is more
  ///    convenient when dealing with a [Scrollable].
  Future<void> dragUntilVisible(
    finders.FinderBase<Element> finder,
    finders.FinderBase<Element> view,
    Offset moveStep, {
    int maxIteration = 50,
    Duration duration = const Duration(milliseconds: 50),
  }) {
    return TestAsyncUtils.guard<void>(() async {
      while (maxIteration > 0 && finder.evaluate().isEmpty) {
        await drag(view, moveStep);
        await pump(duration);
        maxIteration -= 1;
      }
      await Scrollable.ensureVisible(element(finder));
    });
  }
}

/// Variant of [WidgetController] that can be used in tests running
/// on a device.
///
/// This is used, for instance, by [FlutterDriver].
class LiveWidgetController extends WidgetController {
  /// Creates a widget controller that uses the given binding.
  LiveWidgetController(super.binding);

  @override
  Future<void> pump([Duration? duration]) async {
    if (duration != null) {
      await Future<void>.delayed(duration);
    }
    binding.scheduleFrame();
    await binding.endOfFrame;
  }

  @override
  Future<int> pumpAndSettle([Duration duration = const Duration(milliseconds: 100)]) {
    assert(duration > Duration.zero);
    return TestAsyncUtils.guard<int>(() async {
      int count = 0;
      do {
        await pump(duration);
        count += 1;
      } while (binding.hasScheduledFrame);
      return count;
    });
  }

  @override
  Future<List<Duration>> handlePointerEventRecord(List<PointerEventRecord> records) {
    assert(records.isNotEmpty);
    return TestAsyncUtils.guard<List<Duration>>(() async {
      final List<Duration> handleTimeStampDiff = <Duration>[];
      DateTime? startTime;
      for (final PointerEventRecord record in records) {
        final DateTime now = clock.now();
        startTime ??= now;
        // So that the first event is promised to receive a zero timeDiff.
        final Duration timeDiff = record.timeDelay - now.difference(startTime);
        if (timeDiff.isNegative) {
          // This happens when something (e.g. GC) takes a long time during the
          // processing of the events.
          // Flush all past events.
          handleTimeStampDiff.add(-timeDiff);
          record.events.forEach(binding.handlePointerEvent);
        } else {
          await Future<void>.delayed(timeDiff);
          handleTimeStampDiff.add(
            // Recalculating the time diff for getting exact time when the event
            // packet is sent. For a perfect Future.delayed like the one in a
            // fake async this new diff should be zero.
            clock.now().difference(startTime) - record.timeDelay,
          );
          record.events.forEach(binding.handlePointerEvent);
        }
      }

      return handleTimeStampDiff;
    });
  }
}
