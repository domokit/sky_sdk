// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/physics.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

export 'dart:ui' show SemanticsAction, SemanticsFlag;
export 'package:flutter/rendering.dart' show SemanticsData;

const String _matcherHelp =
    'Try dumping the semantics with debugDumpSemanticsTree(DebugSemanticsDumpOrder.inverseHitTest) from the package:flutter/rendering.dart library to see what the semantics tree looks like.';

/// Test semantics data that is compared against real semantics tree.
///
/// Useful with [hasSemantics] and [SemanticsTester] to test the contents of the
/// semantics tree.
class TestSemantics {
  /// Creates an object with some test semantics data.
  ///
  /// The [id] field is required. The root node has an id of zero. Other nodes
  /// are given a unique id when they are created, in a predictable fashion, and
  /// so these values can be hard-coded.
  ///
  /// The [rect] field is required and has no default. Convenient values are
  /// available:
  ///
  ///  * [TestSemantics.rootRect]: 2400x1600, the test screen's size in physical
  ///    pixels, useful for the node with id zero.
  ///
  ///  * [TestSemantics.fullScreen] 800x600, the test screen's size in logical
  ///    pixels, useful for other full-screen widgets.
  TestSemantics({
    this.id,
    this.flags = 0,
    this.actions = 0,
    this.label = '',
    this.value = '',
    this.tooltip = '',
    this.headingLevel,
    this.increasedValue = '',
    this.decreasedValue = '',
    this.hint = '',
    this.textDirection,
    this.rect,
    this.transform,
    this.elevation,
    this.thickness,
    this.textSelection,
    this.children = const <TestSemantics>[],
    this.scrollIndex,
    this.scrollChildren,
    Iterable<SemanticsTag>? tags,
  }) : assert(flags is int || flags is List<SemanticsFlag>),
       assert(actions is int || actions is List<SemanticsAction>),
       tags = tags?.toSet() ?? <SemanticsTag>{};

  /// Creates an object with some test semantics data, with the [id] and [rect]
  /// set to the appropriate values for the root node.
  TestSemantics.root({
    this.flags = 0,
    this.actions = 0,
    this.label = '',
    this.value = '',
    this.increasedValue = '',
    this.decreasedValue = '',
    this.hint = '',
    this.tooltip = '',
    this.headingLevel,
    this.textDirection,
    this.transform,
    this.textSelection,
    this.children = const <TestSemantics>[],
    this.scrollIndex,
    this.scrollChildren,
    Iterable<SemanticsTag>? tags,
  }) : id = 0,
       assert(flags is int || flags is List<SemanticsFlag>),
       assert(actions is int || actions is List<SemanticsAction>),
       rect = TestSemantics.rootRect,
       elevation = 0.0,
       thickness = 0.0,
       tags = tags?.toSet() ?? <SemanticsTag>{};

  /// Creates an object with some test semantics data, with the [id] and [rect]
  /// set to the appropriate values for direct children of the root node.
  ///
  /// The [transform] is set to a 3.0 scale (to account for the
  /// [dart:ui.FlutterView.devicePixelRatio] being 3.0 on the test
  /// pseudo-device).
  ///
  /// The [rect] field is required and has no default. The
  /// [TestSemantics.fullScreen] property may be useful as a value; it describes
  /// an 800x600 rectangle, which is the test screen's size in logical pixels.
  TestSemantics.rootChild({
    this.id,
    this.flags = 0,
    this.actions = 0,
    this.label = '',
    this.hint = '',
    this.value = '',
    this.tooltip = '',
    this.headingLevel,
    this.increasedValue = '',
    this.decreasedValue = '',
    this.textDirection,
    this.rect,
    Matrix4? transform,
    this.elevation,
    this.thickness,
    this.textSelection,
    this.children = const <TestSemantics>[],
    this.scrollIndex,
    this.scrollChildren,
    Iterable<SemanticsTag>? tags,
  }) : assert(flags is int || flags is List<SemanticsFlag>),
       assert(actions is int || actions is List<SemanticsAction>),
       transform = _applyRootChildScale(transform),
       tags = tags?.toSet() ?? <SemanticsTag>{};

  /// The unique identifier for this node.
  ///
  /// The root node has an id of zero. Other nodes are given a unique id when
  /// they are created.
  final int? id;

  /// The [SemanticsFlag]s set on this node.
  ///
  /// There are two ways to specify this property: as an `int` that encodes the
  /// flags as a bit field, or as a `List<SemanticsFlag>` that are _on_.
  ///
  /// Using `List<SemanticsFlag>` is recommended due to better readability.
  final dynamic flags;

  /// The [SemanticsAction]s set on this node.
  ///
  /// There are two ways to specify this property: as an `int` that encodes the
  /// actions as a bit field, or as a `List<SemanticsAction>`.
  ///
  /// Using `List<SemanticsAction>` is recommended due to better readability.
  ///
  /// The tester does not check the function corresponding to the action, but
  /// only its existence.
  final dynamic actions;

  /// A textual description of this node.
  final String label;

  /// A textual description for the value of this node.
  final String value;

  /// What [value] will become after [SemanticsAction.increase] has been
  /// performed.
  final String increasedValue;

  /// What [value] will become after [SemanticsAction.decrease] has been
  /// performed.
  final String decreasedValue;

  /// A brief textual description of the result of the action that can be
  /// performed on this node.
  final String hint;

  /// A textual tooltip of this node.
  final String tooltip;

  /// The reading direction of the [label].
  ///
  /// Even if this is not set, the [hasSemantics] matcher will verify that if a
  /// label is present on the [SemanticsNode], a [SemanticsNode.textDirection]
  /// is also set.
  final TextDirection? textDirection;

  /// The bounding box for this node in its coordinate system.
  ///
  /// Convenient values are available:
  ///
  ///  * [TestSemantics.rootRect]: 2400x1600, the test screen's size in physical
  ///    pixels, useful for the node with id zero.
  ///
  ///  * [TestSemantics.fullScreen] 800x600, the test screen's size in logical
  ///    pixels, useful for other full-screen widgets.
  final Rect? rect;

  /// The test screen's size in physical pixels, typically used as the [rect]
  /// for the node with id zero.
  ///
  /// See also:
  ///
  ///  * [TestSemantics.root], which uses this value to describe the root
  ///    node.
  static const Rect rootRect = Rect.fromLTWH(0.0, 0.0, 2400.0, 1800.0);

  /// The test screen's size in logical pixels, useful for the [rect] of
  /// full-screen widgets other than the root node.
  static const Rect fullScreen = Rect.fromLTWH(0.0, 0.0, 800.0, 600.0);

  /// The transform from this node's coordinate system to its parent's coordinate system.
  ///
  /// By default, the transform is null, which represents the identity
  /// transformation (i.e., that this node has the same coordinate system as its
  /// parent).
  final Matrix4? transform;

  /// The elevation of this node relative to the parent node.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.elevation] for a detailed discussion regarding
  ///    elevation and semantics.
  final double? elevation;

  /// The extend that this node occupies in z-direction starting at [elevation].
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.thickness] for a more detailed definition.
  final double? thickness;

  /// The index of the first visible semantic node within a scrollable.
  final int? scrollIndex;

  /// The total number of semantic nodes within a scrollable.
  final int? scrollChildren;

  final TextSelection? textSelection;

  static Matrix4 _applyRootChildScale(Matrix4? transform) {
    final Matrix4 result = Matrix4.diagonal3Values(3.0, 3.0, 1.0);
    if (transform != null) {
      result.multiply(transform);
    }
    return result;
  }

  /// The children of this node.
  final List<TestSemantics> children;

  /// The tags of this node.
  final Set<SemanticsTag> tags;

  final int? headingLevel;

  bool _matches(
    SemanticsNode? node,
    Map<dynamic, dynamic> matchState, {
    bool ignoreRect = false,
    bool ignoreTransform = false,
    bool ignoreId = false,
    DebugSemanticsDumpOrder childOrder = DebugSemanticsDumpOrder.inverseHitTest,
  }) {
    bool fail(String message) {
      matchState[TestSemantics] = message;
      return false;
    }

    if (node == null) {
      return fail('could not find node with id $id.');
    }
    if (!ignoreId && id != node.id) {
      return fail('expected node id $id but found id ${node.id}.');
    }

    final SemanticsData nodeData = node.getSemanticsData();

    final int flagsBitmask =
        flags is int
            ? flags as int
            : (flags as List<SemanticsFlag>).fold<int>(
              0,
              (int bitmask, SemanticsFlag flag) => bitmask | flag.index,
            );
    if (flagsBitmask != nodeData.flags) {
      return fail('expected node id $id to have flags $flags but found flags ${nodeData.flags}.');
    }

    final int actionsBitmask =
        actions is int
            ? actions as int
            : (actions as List<SemanticsAction>).fold<int>(
              0,
              (int bitmask, SemanticsAction action) => bitmask | action.index,
            );
    if (actionsBitmask != nodeData.actions) {
      return fail(
        'expected node id $id to have actions $actions but found actions ${nodeData.actions}.',
      );
    }

    if (label != nodeData.label) {
      return fail(
        'expected node id $id to have label "$label" but found label "${nodeData.label}".',
      );
    }
    if (value != nodeData.value) {
      return fail(
        'expected node id $id to have value "$value" but found value "${nodeData.value}".',
      );
    }
    if (increasedValue != nodeData.increasedValue) {
      return fail(
        'expected node id $id to have increasedValue "$increasedValue" but found value "${nodeData.increasedValue}".',
      );
    }
    if (decreasedValue != nodeData.decreasedValue) {
      return fail(
        'expected node id $id to have decreasedValue "$decreasedValue" but found value "${nodeData.decreasedValue}".',
      );
    }
    if (hint != nodeData.hint) {
      return fail('expected node id $id to have hint "$hint" but found hint "${nodeData.hint}".');
    }
    if (tooltip != nodeData.tooltip) {
      return fail(
        'expected node id $id to have tooltip "$tooltip" but found hint "${nodeData.tooltip}".',
      );
    }
    if (textDirection != null && textDirection != nodeData.textDirection) {
      return fail(
        'expected node id $id to have textDirection "$textDirection" but found "${nodeData.textDirection}".',
      );
    }
    if ((nodeData.label != '' ||
            nodeData.value != '' ||
            nodeData.hint != '' ||
            node.increasedValue != '' ||
            node.decreasedValue != '') &&
        nodeData.textDirection == null) {
      return fail(
        'expected node id $id, which has a label, value, or hint, to have a textDirection, but it did not.',
      );
    }
    if (!ignoreRect && rect != nodeData.rect) {
      return fail('expected node id $id to have rect $rect but found rect ${nodeData.rect}.');
    }
    if (!ignoreTransform && transform != nodeData.transform) {
      return fail(
        'expected node id $id to have transform $transform but found transform:\n${nodeData.transform}.',
      );
    }
    if (elevation != null && elevation != nodeData.elevation) {
      return fail(
        'expected node id $id to have elevation $elevation but found elevation:\n${nodeData.elevation}.',
      );
    }
    if (thickness != null && thickness != nodeData.thickness) {
      return fail(
        'expected node id $id to have thickness $thickness but found thickness:\n${nodeData.thickness}.',
      );
    }
    if (textSelection?.baseOffset != nodeData.textSelection?.baseOffset ||
        textSelection?.extentOffset != nodeData.textSelection?.extentOffset) {
      return fail(
        'expected node id $id to have textSelection [${textSelection?.baseOffset}, ${textSelection?.end}] but found: [${nodeData.textSelection?.baseOffset}, ${nodeData.textSelection?.extentOffset}].',
      );
    }
    if (scrollIndex != null && scrollIndex != nodeData.scrollIndex) {
      return fail(
        'expected node id $id to have scrollIndex $scrollIndex but found scrollIndex ${nodeData.scrollIndex}.',
      );
    }
    if (scrollChildren != null && scrollChildren != nodeData.scrollChildCount) {
      return fail(
        'expected node id $id to have scrollIndex $scrollChildren but found scrollIndex ${nodeData.scrollChildCount}.',
      );
    }
    final int childrenCount = node.mergeAllDescendantsIntoThisNode ? 0 : node.childrenCount;
    if (children.length != childrenCount) {
      return fail(
        'expected node id $id to have ${children.length} child${children.length == 1 ? "" : "ren"} but found $childrenCount.',
      );
    }
    if (headingLevel != null && headingLevel != node.headingLevel) {
      return fail(
        'expected node id $id to have headingLevel $headingLevel but found headingLevel ${node.headingLevel}',
      );
    }

    if (children.isEmpty) {
      return true;
    }
    bool result = true;
    final Iterator<TestSemantics> it = children.iterator;
    for (final SemanticsNode child in node.debugListChildrenInOrder(childOrder)) {
      it.moveNext();
      final bool childMatches = it.current._matches(
        child,
        matchState,
        ignoreRect: ignoreRect,
        ignoreTransform: ignoreTransform,
        ignoreId: ignoreId,
        childOrder: childOrder,
      );
      if (!childMatches) {
        result = false;
        return false;
      }
    }
    if (it.moveNext()) {
      return false;
    }
    return result;
  }

  @override
  String toString([int indentAmount = 0]) {
    final String indent = '  ' * indentAmount;
    final StringBuffer buf = StringBuffer();
    buf.writeln('$indent${objectRuntimeType(this, 'TestSemantics')}(');
    if (id != null) {
      buf.writeln('$indent  id: $id,');
    }
    if (flags is int && flags != 0 ||
        flags is List<SemanticsFlag> && (flags as List<SemanticsFlag>).isNotEmpty) {
      buf.writeln('$indent  flags: ${SemanticsTester._flagsToSemanticsFlagExpression(flags)},');
    }
    if (actions is int && actions != 0 ||
        actions is List<SemanticsAction> && (actions as List<SemanticsAction>).isNotEmpty) {
      buf.writeln(
        '$indent  actions: ${SemanticsTester._actionsToSemanticsActionExpression(actions)},',
      );
    }
    if (label != '') {
      buf.writeln("$indent  label: '$label',");
    }
    if (value != '') {
      buf.writeln("$indent  value: '$value',");
    }
    if (increasedValue != '') {
      buf.writeln("$indent  increasedValue: '$increasedValue',");
    }
    if (decreasedValue != '') {
      buf.writeln("$indent  decreasedValue: '$decreasedValue',");
    }
    if (hint != '') {
      buf.writeln("$indent  hint: '$hint',");
    }
    if (tooltip != '') {
      buf.writeln("$indent  tooltip: '$tooltip',");
    }
    if (textDirection != null) {
      buf.writeln('$indent  textDirection: $textDirection,');
    }
    if (textSelection?.isValid ?? false) {
      buf.writeln('$indent  textSelection:\n[${textSelection!.start}, ${textSelection!.end}],');
    }
    if (scrollIndex != null) {
      buf.writeln('$indent scrollIndex: $scrollIndex,');
    }
    if (rect != null) {
      buf.writeln('$indent  rect: $rect,');
    }
    if (transform != null) {
      buf.writeln(
        '$indent  transform:\n${transform.toString().trim().split('\n').map<String>((String line) => '$indent    $line').join('\n')},',
      );
    }
    if (elevation != null) {
      buf.writeln('$indent  elevation: $elevation,');
    }
    if (thickness != null) {
      buf.writeln('$indent  thickness: $thickness,');
    }
    buf.writeln('$indent  children: <TestSemantics>[');
    for (final TestSemantics child in children) {
      buf.writeln('${child.toString(indentAmount + 2)},');
    }
    buf.writeln('$indent  ],');
    buf.write('$indent)');
    return buf.toString();
  }
}

/// Ensures that the given widget tester has a semantics tree to test.
///
/// Useful with [hasSemantics] to test the contents of the semantics tree.
class SemanticsTester {
  /// Creates a semantics tester for the given widget tester.
  ///
  /// You should call [dispose] at the end of a test that creates a semantics
  /// tester.
  SemanticsTester(this.tester) {
    _semanticsHandle = tester.ensureSemantics();

    // This _extra_ clean-up is needed for the case when a test fails and
    // therefore fails to call dispose() explicitly. The test is still required
    // to call dispose() explicitly, because the semanticsOwner check is
    // performed irrespective of whether the owner was created via
    // SemanticsTester or directly. When the test succeeds, this tear-down
    // becomes a no-op.
    addTearDown(dispose);
  }

  /// The widget tester that this object is testing the semantics of.
  final WidgetTester tester;
  SemanticsHandle? _semanticsHandle;

  /// Release resources held by this semantics tester.
  ///
  /// Call this function at the end of any test that uses a semantics tester. It
  /// is OK to call this function multiple times. If the resources have already
  /// been released, the subsequent calls have no effect.
  @mustCallSuper
  void dispose() {
    _semanticsHandle?.dispose();
    _semanticsHandle = null;
  }

  @override
  String toString() =>
      'SemanticsTester for ${tester.binding.pipelineOwner.semanticsOwner?.rootSemanticsNode}';

  bool _stringAttributesEqual(List<StringAttribute> first, List<StringAttribute> second) {
    if (first.length != second.length) {
      return false;
    }
    for (int i = 0; i < first.length; i++) {
      if (first[i] is SpellOutStringAttribute &&
          (second[i] is! SpellOutStringAttribute || second[i].range != first[i].range)) {
        return false;
      }
      if (first[i] is LocaleStringAttribute &&
          (second[i] is! LocaleStringAttribute ||
              second[i].range != first[i].range ||
              (second[i] as LocaleStringAttribute).locale !=
                  (second[i] as LocaleStringAttribute).locale)) {
        return false;
      }
    }
    return true;
  }

  /// Returns all semantics nodes in the current semantics tree whose properties
  /// match the non-null arguments.
  ///
  /// If multiple arguments are non-null, each of the returned nodes must match
  /// on all of them.
  ///
  /// If `ancestor` is not null, only the descendants of it are returned.
  Iterable<SemanticsNode> nodesWith({
    AttributedString? attributedLabel,
    AttributedString? attributedValue,
    AttributedString? attributedHint,
    String? label,
    String? value,
    String? hint,
    TextDirection? textDirection,
    List<SemanticsAction>? actions,
    List<SemanticsFlag>? flags,
    Set<SemanticsTag>? tags,
    double? scrollPosition,
    double? scrollExtentMax,
    double? scrollExtentMin,
    int? currentValueLength,
    int? maxValueLength,
    SemanticsNode? ancestor,
  }) {
    bool checkNode(SemanticsNode node) {
      if (label != null && node.label != label) {
        return false;
      }
      if (attributedLabel != null &&
          (attributedLabel.string != node.attributedLabel.string ||
              !_stringAttributesEqual(
                attributedLabel.attributes,
                node.attributedLabel.attributes,
              ))) {
        return false;
      }
      if (value != null && node.value != value) {
        return false;
      }
      if (attributedValue != null &&
          (attributedValue.string != node.attributedValue.string ||
              !_stringAttributesEqual(
                attributedValue.attributes,
                node.attributedValue.attributes,
              ))) {
        return false;
      }
      if (hint != null && node.hint != hint) {
        return false;
      }
      if (attributedHint != null &&
          (attributedHint.string != node.attributedHint.string ||
              !_stringAttributesEqual(attributedHint.attributes, node.attributedHint.attributes))) {
        return false;
      }
      if (textDirection != null && node.textDirection != textDirection) {
        return false;
      }

      if (actions != null) {
        final int expectedActions = actions.fold<int>(
          0,
          (int value, SemanticsAction action) => value | action.index,
        );
        final int actualActions = node.getSemanticsData().actions;
        if (expectedActions != actualActions) {
          return false;
        }
      }
      if (flags != null) {
        final int expectedFlags = flags.fold<int>(
          0,
          (int value, SemanticsFlag flag) => value | flag.index,
        );
        final int actualFlags = node.getSemanticsData().flags;
        if (expectedFlags != actualFlags) {
          return false;
        }
      }
      if (tags != null) {
        final Set<SemanticsTag>? actualTags = node.getSemanticsData().tags;
        if (!setEquals<SemanticsTag>(actualTags, tags)) {
          return false;
        }
      }
      if (scrollPosition != null && !nearEqual(node.scrollPosition, scrollPosition, 0.1)) {
        return false;
      }
      if (scrollExtentMax != null && !nearEqual(node.scrollExtentMax, scrollExtentMax, 0.1)) {
        return false;
      }
      if (scrollExtentMin != null && !nearEqual(node.scrollExtentMin, scrollExtentMin, 0.1)) {
        return false;
      }
      if (currentValueLength != null && node.currentValueLength != currentValueLength) {
        return false;
      }
      if (maxValueLength != null && node.maxValueLength != maxValueLength) {
        return false;
      }
      return true;
    }

    final List<SemanticsNode> result = <SemanticsNode>[];
    bool visit(SemanticsNode node) {
      if (checkNode(node)) {
        result.add(node);
      }
      node.visitChildren(visit);
      return true;
    }

    visit(ancestor ?? tester.binding.pipelineOwner.semanticsOwner!.rootSemanticsNode!);

    return result;
  }

  /// Generates an expression that creates a [TestSemantics] reflecting the
  /// current tree of [SemanticsNode]s.
  ///
  /// Use this method to generate code for unit tests. It works similar to
  /// screenshot testing. The very first time you add semantics to a widget you
  /// verify manually that the widget behaves correctly. You then use this
  /// method to generate test code for this widget.
  ///
  /// Example:
  ///
  /// ```dart
  /// testWidgets('generate code for MyWidget', (WidgetTester tester) async {
  ///   var semantics = SemanticsTester(tester);
  ///   await tester.pumpWidget(MyWidget());
  ///   print(semantics.generateTestSemanticsExpressionForCurrentSemanticsTree());
  ///   semantics.dispose();
  /// });
  /// ```
  ///
  /// You can now copy the code printed to the console into a unit test:
  ///
  /// ```dart
  /// testWidgets('generate code for MyWidget', (WidgetTester tester) async {
  ///   var semantics = SemanticsTester(tester);
  ///   await tester.pumpWidget(MyWidget());
  ///   expect(semantics, hasSemantics(
  ///     // Generated code:
  ///     TestSemantics(
  ///       ... properties and child nodes ...
  ///     ),
  ///     ignoreRect: true,
  ///     ignoreTransform: true,
  ///     ignoreId: true,
  ///   ));
  ///   semantics.dispose();
  /// });
  /// ```
  ///
  /// At this point the unit test should automatically pass because it was
  /// generated from the actual [SemanticsNode]s. Next time the semantics tree
  /// changes, the test code may either be updated manually, or regenerated and
  /// replaced using this method again.
  ///
  /// Avoid submitting huge piles of generated test code. This will make test
  /// code hard to review and it will make it tempting to regenerate test code
  /// every time and ignore potential regressions. Make sure you do not
  /// over-test. Prefer breaking your widgets into smaller widgets and test them
  /// individually.
  String generateTestSemanticsExpressionForCurrentSemanticsTree(
    DebugSemanticsDumpOrder childOrder,
  ) {
    final SemanticsNode? node = tester.binding.pipelineOwner.semanticsOwner?.rootSemanticsNode;
    return _generateSemanticsTestForNode(node, 0, childOrder);
  }

  static String _flagsToSemanticsFlagExpression(dynamic flags) {
    Iterable<SemanticsFlag> list;
    if (flags is int) {
      list = SemanticsFlag.values.where((SemanticsFlag flag) => (flag.index & flags) != 0);
    } else {
      list = flags as List<SemanticsFlag>;
    }
    return '<SemanticsFlag>[${list.join(', ')}]';
  }

  static String _tagsToSemanticsTagExpression(Set<SemanticsTag> tags) {
    return '<SemanticsTag>[${tags.map<String>((SemanticsTag tag) => "const SemanticsTag('${tag.name}')").join(', ')}]';
  }

  static String _actionsToSemanticsActionExpression(dynamic actions) {
    Iterable<SemanticsAction> list;
    if (actions is int) {
      list = SemanticsAction.values.where(
        (SemanticsAction action) => (action.index & actions) != 0,
      );
    } else {
      list = actions as List<SemanticsAction>;
    }
    return '<SemanticsAction>[${list.join(', ')}]';
  }

  /// Recursively generates [TestSemantics] code for [node] and its children,
  /// indenting the expression by `indentAmount`.
  static String _generateSemanticsTestForNode(
    SemanticsNode? node,
    int indentAmount,
    DebugSemanticsDumpOrder childOrder,
  ) {
    if (node == null) {
      return 'null';
    }
    final String indent = '  ' * indentAmount;
    final StringBuffer buf = StringBuffer();
    final SemanticsData nodeData = node.getSemanticsData();
    final bool isRoot = node.id == 0;
    buf.writeln('TestSemantics${isRoot ? '.root' : ''}(');
    if (!isRoot) {
      buf.writeln('  id: ${node.id},');
    }
    if (nodeData.tags != null) {
      buf.writeln('  tags: ${_tagsToSemanticsTagExpression(nodeData.tags!)},');
    }
    if (nodeData.flags != 0) {
      buf.writeln('  flags: ${_flagsToSemanticsFlagExpression(nodeData.flags)},');
    }
    if (nodeData.actions != 0) {
      buf.writeln('  actions: ${_actionsToSemanticsActionExpression(nodeData.actions)},');
    }
    if (node.label.isNotEmpty) {
      // Escape newlines and text directionality control characters.
      final String escapedLabel = node.label
          .replaceAll('\n', r'\n')
          .replaceAll('\u202a', r'\u202a')
          .replaceAll('\u202c', r'\u202c');
      buf.writeln("  label: '$escapedLabel',");
    }
    if (node.value.isNotEmpty) {
      buf.writeln("  value: '${node.value}',");
    }
    if (node.increasedValue.isNotEmpty) {
      buf.writeln("  increasedValue: '${node.increasedValue}',");
    }
    if (node.decreasedValue.isNotEmpty) {
      buf.writeln("  decreasedValue: '${node.decreasedValue}',");
    }
    if (node.hint.isNotEmpty) {
      buf.writeln("  hint: '${node.hint}',");
    }
    if (node.textDirection != null) {
      buf.writeln('  textDirection: ${node.textDirection},');
    }
    if (node.hasChildren) {
      buf.writeln('  children: <TestSemantics>[');
      for (final SemanticsNode child in node.debugListChildrenInOrder(childOrder)) {
        buf
          ..write(_generateSemanticsTestForNode(child, 2, childOrder))
          ..writeln(',');
      }
      buf.writeln('  ],');
    }

    buf.write(')');
    return buf.toString().split('\n').map<String>((String l) => '$indent$l').join('\n');
  }
}

class _HasSemantics extends Matcher {
  const _HasSemantics(
    this._semantics, {
    required this.ignoreRect,
    required this.ignoreTransform,
    required this.ignoreId,
    required this.childOrder,
  });

  final TestSemantics _semantics;
  final bool ignoreRect;
  final bool ignoreTransform;
  final bool ignoreId;
  final DebugSemanticsDumpOrder childOrder;

  @override
  bool matches(covariant SemanticsTester item, Map<dynamic, dynamic> matchState) {
    final bool doesMatch = _semantics._matches(
      item.tester.binding.pipelineOwner.semanticsOwner?.rootSemanticsNode,
      matchState,
      ignoreTransform: ignoreTransform,
      ignoreRect: ignoreRect,
      ignoreId: ignoreId,
      childOrder: childOrder,
    );
    if (!doesMatch) {
      matchState['would-match'] = item.generateTestSemanticsExpressionForCurrentSemanticsTree(
        childOrder,
      );
    }
    if (item.tester.binding.pipelineOwner.semanticsOwner == null) {
      matchState['additional-notes'] =
          '(Check that the SemanticsTester has not been disposed early.)';
    }
    return doesMatch;
  }

  @override
  Description describe(Description description) {
    return description.add('semantics node matching:\n$_semantics');
  }

  String _indent(String? text) {
    return text
        .toString()
        .trimRight()
        .split('\n')
        .map<String>((String line) => '  $line')
        .join('\n');
  }

  @override
  Description describeMismatch(
    dynamic item,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    Description result = mismatchDescription
        .add('${matchState[TestSemantics]}\n')
        .add('Current SemanticsNode tree:\n')
        .add(
          _indent(
            RendererBinding.instance.renderView.debugSemantics?.toStringDeep(
              childOrder: childOrder,
            ),
          ),
        )
        .add('\n')
        .add('The semantics tree would have matched the following configuration:\n')
        .add(_indent(matchState['would-match'] as String));
    if (matchState.containsKey('additional-notes')) {
      result = result.add('\n').add(matchState['additional-notes'] as String);
    }
    return result;
  }
}

/// Asserts that a [SemanticsTester] has a semantics tree that exactly matches the given semantics.
Matcher hasSemantics(
  TestSemantics semantics, {
  bool ignoreRect = false,
  bool ignoreTransform = false,
  bool ignoreId = false,
  DebugSemanticsDumpOrder childOrder = DebugSemanticsDumpOrder.traversalOrder,
}) {
  return _HasSemantics(
    semantics,
    ignoreRect: ignoreRect,
    ignoreTransform: ignoreTransform,
    ignoreId: ignoreId,
    childOrder: childOrder,
  );
}

class _IncludesNodeWith extends Matcher {
  const _IncludesNodeWith({
    this.attributedLabel,
    this.attributedValue,
    this.attributedHint,
    this.label,
    this.value,
    this.hint,
    this.textDirection,
    this.actions,
    this.flags,
    this.tags,
    this.scrollPosition,
    this.scrollExtentMax,
    this.scrollExtentMin,
    this.maxValueLength,
    this.currentValueLength,
  }) : assert(
         label != null ||
             value != null ||
             actions != null ||
             flags != null ||
             tags != null ||
             scrollPosition != null ||
             scrollExtentMax != null ||
             scrollExtentMin != null ||
             maxValueLength != null ||
             currentValueLength != null,
       );
  final AttributedString? attributedLabel;
  final AttributedString? attributedValue;
  final AttributedString? attributedHint;
  final String? label;
  final String? value;
  final String? hint;
  final TextDirection? textDirection;
  final List<SemanticsAction>? actions;
  final List<SemanticsFlag>? flags;
  final Set<SemanticsTag>? tags;
  final double? scrollPosition;
  final double? scrollExtentMax;
  final double? scrollExtentMin;
  final int? currentValueLength;
  final int? maxValueLength;

  @override
  bool matches(covariant SemanticsTester item, Map<dynamic, dynamic> matchState) {
    return item
        .nodesWith(
          attributedLabel: attributedLabel,
          attributedValue: attributedValue,
          attributedHint: attributedHint,
          label: label,
          value: value,
          hint: hint,
          textDirection: textDirection,
          actions: actions,
          flags: flags,
          tags: tags,
          scrollPosition: scrollPosition,
          scrollExtentMax: scrollExtentMax,
          scrollExtentMin: scrollExtentMin,
          currentValueLength: currentValueLength,
          maxValueLength: maxValueLength,
        )
        .isNotEmpty;
  }

  @override
  Description describe(Description description) {
    return description.add('includes node with $_configAsString');
  }

  @override
  Description describeMismatch(
    dynamic item,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    return mismatchDescription.add('could not find node with $_configAsString.\n$_matcherHelp');
  }

  String get _configAsString {
    final List<String> strings = <String>[
      if (label != null) 'label "$label"',
      if (value != null) 'value "$value"',
      if (hint != null) 'hint "$hint"',
      if (textDirection != null) ' (${textDirection!.name})',
      if (actions != null) 'actions "${actions!.join(', ')}"',
      if (flags != null) 'flags "${flags!.join(', ')}"',
      if (tags != null) 'tags "${tags!.join(', ')}"',
      if (scrollPosition != null) 'scrollPosition "$scrollPosition"',
      if (scrollExtentMax != null) 'scrollExtentMax "$scrollExtentMax"',
      if (scrollExtentMin != null) 'scrollExtentMin "$scrollExtentMin"',
      if (currentValueLength != null) 'currentValueLength "$currentValueLength"',
      if (maxValueLength != null) 'maxValueLength "$maxValueLength"',
    ];
    return strings.join(', ');
  }
}

/// Asserts that a node in the semantics tree of [SemanticsTester] has `label`,
/// `textDirection`, and `actions`.
///
/// If null is provided for an argument, it will match against any value.
Matcher includesNodeWith({
  String? label,
  AttributedString? attributedLabel,
  String? value,
  AttributedString? attributedValue,
  String? hint,
  AttributedString? attributedHint,
  TextDirection? textDirection,
  List<SemanticsAction>? actions,
  List<SemanticsFlag>? flags,
  Set<SemanticsTag>? tags,
  double? scrollPosition,
  double? scrollExtentMax,
  double? scrollExtentMin,
  int? maxValueLength,
  int? currentValueLength,
}) {
  return _IncludesNodeWith(
    label: label,
    attributedLabel: attributedLabel,
    value: value,
    attributedValue: attributedValue,
    hint: hint,
    attributedHint: attributedHint,
    textDirection: textDirection,
    actions: actions,
    flags: flags,
    tags: tags,
    scrollPosition: scrollPosition,
    scrollExtentMax: scrollExtentMax,
    scrollExtentMin: scrollExtentMin,
    maxValueLength: maxValueLength,
    currentValueLength: currentValueLength,
  );
}
