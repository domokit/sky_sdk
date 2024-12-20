// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
///
/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter/rendering.dart';
library;

import 'dart:math' as math;
import 'dart:ui'
    show
        Offset,
        Rect,
        SemanticsAction,
        SemanticsFlag,
        SemanticsUpdate,
        SemanticsUpdateBuilder,
        StringAttribute,
        TextDirection;

import 'package:collection/collection.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart' show MatrixUtils, TransformProperty;
import 'package:flutter/services.dart';
import 'package:vector_math/vector_math_64.dart';

import 'binding.dart' show SemanticsBinding;
import 'semantics_event.dart';

export 'dart:ui'
    show Offset, Rect, SemanticsAction, SemanticsFlag, StringAttribute, TextDirection, VoidCallback;

export 'package:flutter/foundation.dart'
    show
        DiagnosticLevel,
        DiagnosticPropertiesBuilder,
        DiagnosticsNode,
        DiagnosticsTreeStyle,
        Key,
        TextTreeConfiguration;
export 'package:flutter/services.dart' show TextSelection;
export 'package:vector_math/vector_math_64.dart' show Matrix4;

export 'semantics_event.dart' show SemanticsEvent;

/// Signature for a function that is called for each [SemanticsNode].
///
/// Return false to stop visiting nodes.
///
/// Used by [SemanticsNode.visitChildren].
typedef SemanticsNodeVisitor = bool Function(SemanticsNode node);

/// Signature for [SemanticsAction]s that move the cursor.
///
/// If `extendSelection` is set to true the cursor movement should extend the
/// current selection or (if nothing is currently selected) start a selection.
typedef MoveCursorHandler = void Function(bool extendSelection);

/// Signature for the [SemanticsAction.setSelection] handlers to change the
/// text selection (or re-position the cursor) to `selection`.
typedef SetSelectionHandler = void Function(TextSelection selection);

/// Signature for the [SemanticsAction.setText] handlers to replace the
/// current text with the input `text`.
typedef SetTextHandler = void Function(String text);

/// Signature for the [SemanticsAction.scrollToOffset] handlers to scroll the
/// scrollable container to the given `targetOffset`.
typedef ScrollToOffsetHandler = void Function(Offset targetOffset);

/// Signature for a handler of a [SemanticsAction].
///
/// Returned by [SemanticsConfiguration.getActionHandler].
typedef SemanticsActionHandler = void Function(Object? args);

/// Signature for a function that receives a semantics update and returns no result.
///
/// Used by [SemanticsOwner.onSemanticsUpdate].
typedef SemanticsUpdateCallback = void Function(SemanticsUpdate update);

/// Signature for the [SemanticsConfiguration.childConfigurationsDelegate].
///
/// The input list contains all [SemanticsConfiguration]s that rendering
/// children want to merge upward. One can tag a render child with a
/// [SemanticsTag] and look up its [SemanticsConfiguration]s through
/// [SemanticsConfiguration.tagsChildrenWith].
///
/// The return value is the arrangement of these configs, including which
/// configs continue to merge upward and which configs form sibling merge group.
///
/// Use [ChildSemanticsConfigurationsResultBuilder] to generate the return
/// value.
typedef ChildSemanticsConfigurationsDelegate =
    ChildSemanticsConfigurationsResult Function(List<SemanticsConfiguration>);

final int _kUnblockedUserActions =
    SemanticsAction.didGainAccessibilityFocus.index |
    SemanticsAction.didLoseAccessibilityFocus.index;

/// A tag for a [SemanticsNode].
///
/// Tags can be interpreted by the parent of a [SemanticsNode]
/// and depending on the presence of a tag the parent can for example decide
/// how to add the tagged node as a child. Tags are not sent to the engine.
///
/// As an example, the [RenderSemanticsGestureHandler] uses tags to determine
/// if a child node should be excluded from the scrollable area for semantic
/// purposes.
///
/// The provided [name] is only used for debugging. Two tags created with the
/// same [name] and the `new` operator are not considered identical. However,
/// two tags created with the same [name] and the `const` operator are always
/// identical.
class SemanticsTag {
  /// Creates a [SemanticsTag].
  ///
  /// The provided [name] is only used for debugging. Two tags created with the
  /// same [name] and the `new` operator are not considered identical. However,
  /// two tags created with the same [name] and the `const` operator are always
  /// identical.
  const SemanticsTag(this.name);

  /// A human-readable name for this tag used for debugging.
  ///
  /// This string is not used to determine if two tags are identical.
  final String name;

  @override
  String toString() => '${objectRuntimeType(this, 'SemanticsTag')}($name)';
}

/// The result that contains the arrangement for the child
/// [SemanticsConfiguration]s.
///
/// When the [PipelineOwner] builds the semantics tree, it uses the returned
/// [ChildSemanticsConfigurationsResult] from
/// [SemanticsConfiguration.childConfigurationsDelegate] to decide how semantics nodes
/// should form.
///
/// Use [ChildSemanticsConfigurationsResultBuilder] to build the result.
class ChildSemanticsConfigurationsResult {
  ChildSemanticsConfigurationsResult._(this.mergeUp, this.siblingMergeGroups);

  /// Returns the [SemanticsConfiguration]s that are supposed to be merged into
  /// the parent semantics node.
  ///
  /// [SemanticsConfiguration]s that are either semantics boundaries or are
  /// conflicting with other [SemanticsConfiguration]s will form explicit
  /// semantics nodes. All others will be merged into the parent.
  final List<SemanticsConfiguration> mergeUp;

  /// The groups of child semantics configurations that want to merge together
  /// and form a sibling [SemanticsNode].
  ///
  /// All the [SemanticsConfiguration]s in a given group that are either
  /// semantics boundaries or are conflicting with other
  /// [SemanticsConfiguration]s of the same group will be excluded from the
  /// sibling merge group and form independent semantics nodes as usual.
  ///
  /// The result [SemanticsNode]s from the merges are attached as the sibling
  /// nodes of the immediate parent semantics node. For example, a `RenderObjectA`
  /// has a rendering child, `RenderObjectB`. If both of them form their own
  /// semantics nodes, `SemanticsNodeA` and `SemanticsNodeB`, any semantics node
  /// created from sibling merge groups of `RenderObjectB` will be attach to
  /// `SemanticsNodeA` as a sibling of `SemanticsNodeB`.
  final List<List<SemanticsConfiguration>> siblingMergeGroups;
}

/// The builder to build a [ChildSemanticsConfigurationsResult] based on its
/// annotations.
///
/// To use this builder, one can use [markAsMergeUp] and
/// [markAsSiblingMergeGroup] to annotate the arrangement of
/// [SemanticsConfiguration]s. Once all the configs are annotated, use [build]
/// to generate the [ChildSemanticsConfigurationsResult].
class ChildSemanticsConfigurationsResultBuilder {
  /// Creates a [ChildSemanticsConfigurationsResultBuilder].
  ChildSemanticsConfigurationsResultBuilder();

  final List<SemanticsConfiguration> _mergeUp = <SemanticsConfiguration>[];
  final List<List<SemanticsConfiguration>> _siblingMergeGroups = <List<SemanticsConfiguration>>[];

  /// Marks the [SemanticsConfiguration] to be merged into the parent semantics
  /// node.
  ///
  /// The [SemanticsConfiguration] will be added to the
  /// [ChildSemanticsConfigurationsResult.mergeUp] that this builder builds.
  void markAsMergeUp(SemanticsConfiguration config) => _mergeUp.add(config);

  /// Marks a group of [SemanticsConfiguration]s to merge together
  /// and form a sibling [SemanticsNode].
  ///
  /// The group of [SemanticsConfiguration]s will be added to the
  /// [ChildSemanticsConfigurationsResult.siblingMergeGroups] that this builder builds.
  void markAsSiblingMergeGroup(List<SemanticsConfiguration> configs) =>
      _siblingMergeGroups.add(configs);

  /// Builds a [ChildSemanticsConfigurationsResult] contains the arrangement.
  ChildSemanticsConfigurationsResult build() {
    assert(() {
      final Set<SemanticsConfiguration> seenConfigs = <SemanticsConfiguration>{};
      for (final SemanticsConfiguration config in <SemanticsConfiguration>[
        ..._mergeUp,
        ..._siblingMergeGroups.flattened,
      ]) {
        assert(
          seenConfigs.add(config),
          'Duplicated SemanticsConfigurations. This can happen if the same '
          'SemanticsConfiguration was marked twice in markAsMergeUp and/or '
          'markAsSiblingMergeGroup',
        );
      }
      return true;
    }());
    return ChildSemanticsConfigurationsResult._(_mergeUp, _siblingMergeGroups);
  }
}

/// An identifier of a custom semantics action.
///
/// Custom semantics actions can be provided to make complex user
/// interactions more accessible. For instance, if an application has a
/// drag-and-drop list that requires the user to press and hold an item
/// to move it, users interacting with the application using a hardware
/// switch may have difficulty. This can be made accessible by creating custom
/// actions and pairing them with handlers that move a list item up or down in
/// the list.
///
/// In Android, these actions are presented in the local context menu. In iOS,
/// these are presented in the radial context menu.
///
/// Localization and text direction do not automatically apply to the provided
/// label or hint.
///
/// Instances of this class should either be instantiated with const or
/// new instances cached in static fields.
///
/// See also:
///
///  * [SemanticsProperties], where the handler for a custom action is provided.
@immutable
class CustomSemanticsAction {
  /// Creates a new [CustomSemanticsAction].
  ///
  /// The [label] must not be empty.
  const CustomSemanticsAction({required String this.label})
    : assert(label != ''),
      hint = null,
      action = null;

  /// Creates a new [CustomSemanticsAction] that overrides a standard semantics
  /// action.
  ///
  /// The [hint] must not be empty.
  const CustomSemanticsAction.overridingAction({
    required String this.hint,
    required SemanticsAction this.action,
  }) : assert(hint != ''),
       label = null;

  /// The user readable name of this custom semantics action.
  final String? label;

  /// The hint description of this custom semantics action.
  final String? hint;

  /// The standard semantics action this action replaces.
  final SemanticsAction? action;

  @override
  int get hashCode => Object.hash(label, hint, action);

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is CustomSemanticsAction &&
        other.label == label &&
        other.hint == hint &&
        other.action == action;
  }

  @override
  String toString() {
    return 'CustomSemanticsAction(${_ids[this]}, label:$label, hint:$hint, action:$action)';
  }

  // Logic to assign a unique id to each custom action without requiring
  // user specification.
  static int _nextId = 0;
  static final Map<int, CustomSemanticsAction> _actions = <int, CustomSemanticsAction>{};
  static final Map<CustomSemanticsAction, int> _ids = <CustomSemanticsAction, int>{};

  /// Get the identifier for a given `action`.
  static int getIdentifier(CustomSemanticsAction action) {
    int? result = _ids[action];
    if (result == null) {
      result = _nextId++;
      _ids[action] = result;
      _actions[result] = action;
    }
    return result;
  }

  /// Get the `action` for a given identifier.
  static CustomSemanticsAction? getAction(int id) {
    return _actions[id];
  }

  /// Resets internal state between tests. Does nothing if asserts are disabled.
  @visibleForTesting
  static void resetForTests() {
    assert(() {
      _actions.clear();
      _ids.clear();
      _nextId = 0;
      return true;
    }());
  }
}

/// A string that carries a list of [StringAttribute]s.
@immutable
class AttributedString {
  /// Creates a attributed string.
  ///
  /// The [TextRange] in the [attributes] must be inside the length of the
  /// [string].
  ///
  /// The [attributes] must not be changed after the attributed string is
  /// created.
  AttributedString(this.string, {this.attributes = const <StringAttribute>[]})
    : assert(string.isNotEmpty || attributes.isEmpty),
      assert(() {
        for (final StringAttribute attribute in attributes) {
          assert(
            string.length >= attribute.range.start && string.length >= attribute.range.end,
            'The range in $attribute is outside of the string $string',
          );
        }
        return true;
      }());

  /// The plain string stored in the attributed string.
  final String string;

  /// The attributes this string carries.
  ///
  /// The list must not be modified after this string is created.
  final List<StringAttribute> attributes;

  /// Returns a new [AttributedString] by concatenate the operands
  ///
  /// The string attribute list of the returned [AttributedString] will contains
  /// the string attributes from both operands with updated text ranges.
  AttributedString operator +(AttributedString other) {
    if (string.isEmpty) {
      return other;
    }
    if (other.string.isEmpty) {
      return this;
    }

    // None of the strings is empty.
    final String newString = string + other.string;
    final List<StringAttribute> newAttributes = List<StringAttribute>.of(attributes);
    if (other.attributes.isNotEmpty) {
      final int offset = string.length;
      for (final StringAttribute attribute in other.attributes) {
        final TextRange newRange = TextRange(
          start: attribute.range.start + offset,
          end: attribute.range.end + offset,
        );
        final StringAttribute adjustedAttribute = attribute.copy(range: newRange);
        newAttributes.add(adjustedAttribute);
      }
    }
    return AttributedString(newString, attributes: newAttributes);
  }

  /// Two [AttributedString]s are equal if their string and attributes are.
  @override
  bool operator ==(Object other) {
    return other.runtimeType == runtimeType &&
        other is AttributedString &&
        other.string == string &&
        listEquals<StringAttribute>(other.attributes, attributes);
  }

  @override
  int get hashCode => Object.hash(string, attributes);

  @override
  String toString() {
    return "${objectRuntimeType(this, 'AttributedString')}('$string', attributes: $attributes)";
  }
}

/// A [DiagnosticsProperty] for [AttributedString]s, which shows a string
/// when there are no attributes, and more details otherwise.
class AttributedStringProperty extends DiagnosticsProperty<AttributedString> {
  /// Create a diagnostics property for an [AttributedString] object.
  ///
  /// Such properties are used with [SemanticsData] objects.
  AttributedStringProperty(
    String super.name,
    super.value, {
    super.showName,
    this.showWhenEmpty = false,
    super.defaultValue,
    super.level,
    super.description,
  });

  /// Whether to show the property when the [value] is an [AttributedString]
  /// whose [AttributedString.string] is the empty string.
  ///
  /// This overrides [defaultValue].
  final bool showWhenEmpty;

  @override
  bool get isInteresting =>
      super.isInteresting && (showWhenEmpty || (value != null && value!.string.isNotEmpty));

  @override
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    if (value == null) {
      return 'null';
    }
    String text = value!.string;
    if (parentConfiguration != null && !parentConfiguration.lineBreakProperties) {
      // This follows a similar pattern to StringProperty.
      text = text.replaceAll('\n', r'\n');
    }
    if (value!.attributes.isEmpty) {
      return '"$text"';
    }
    return '"$text" ${value!.attributes}'; // the attributes will be in square brackets since they're a list
  }
}

/// Summary information about a [SemanticsNode] object.
///
/// A semantics node might [SemanticsNode.mergeAllDescendantsIntoThisNode],
/// which means the individual fields on the semantics node don't fully describe
/// the semantics at that node. This data structure contains the full semantics
/// for the node.
///
/// Typically obtained from [SemanticsNode.getSemanticsData].
@immutable
class SemanticsData with Diagnosticable {
  /// Creates a semantics data object.
  ///
  /// If [label] is not empty, then [textDirection] must also not be null.
  SemanticsData({
    required this.flags,
    required this.actions,
    required this.identifier,
    required this.attributedLabel,
    required this.attributedValue,
    required this.attributedIncreasedValue,
    required this.attributedDecreasedValue,
    required this.attributedHint,
    required this.tooltip,
    required this.textDirection,
    required this.rect,
    required this.elevation,
    required this.thickness,
    required this.textSelection,
    required this.scrollIndex,
    required this.scrollChildCount,
    required this.scrollPosition,
    required this.scrollExtentMax,
    required this.scrollExtentMin,
    required this.platformViewId,
    required this.maxValueLength,
    required this.currentValueLength,
    required this.headingLevel,
    required this.linkUrl,
    this.tags,
    this.transform,
    this.customSemanticsActionIds,
  }) : assert(
         tooltip == '' || textDirection != null,
         'A SemanticsData object with tooltip "$tooltip" had a null textDirection.',
       ),
       assert(
         attributedLabel.string == '' || textDirection != null,
         'A SemanticsData object with label "${attributedLabel.string}" had a null textDirection.',
       ),
       assert(
         attributedValue.string == '' || textDirection != null,
         'A SemanticsData object with value "${attributedValue.string}" had a null textDirection.',
       ),
       assert(
         attributedDecreasedValue.string == '' || textDirection != null,
         'A SemanticsData object with decreasedValue "${attributedDecreasedValue.string}" had a null textDirection.',
       ),
       assert(
         attributedIncreasedValue.string == '' || textDirection != null,
         'A SemanticsData object with increasedValue "${attributedIncreasedValue.string}" had a null textDirection.',
       ),
       assert(
         attributedHint.string == '' || textDirection != null,
         'A SemanticsData object with hint "${attributedHint.string}" had a null textDirection.',
       ),
       assert(headingLevel >= 0 && headingLevel <= 6, 'Heading level must be between 0 and 6'),
       assert(
         linkUrl == null || (flags & SemanticsFlag.isLink.index) != 0,
         'A SemanticsData object with a linkUrl must have the isLink flag set to true',
       );

  /// A bit field of [SemanticsFlag]s that apply to this node.
  final int flags;

  /// A bit field of [SemanticsAction]s that apply to this node.
  final int actions;

  /// {@macro flutter.semantics.SemanticsProperties.identifier}
  final String identifier;

  /// A textual description for the current label of the node.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// This exposes the raw text of the [attributedLabel].
  String get label => attributedLabel.string;

  /// A textual description for the current label of the node in
  /// [AttributedString] format.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also [label], which exposes just the raw text.
  final AttributedString attributedLabel;

  /// A textual description for the current value of the node.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// This exposes the raw text of the [attributedValue].
  String get value => attributedValue.string;

  /// A textual description for the current value of the node in
  /// [AttributedString] format.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also [value], which exposes just the raw text.
  final AttributedString attributedValue;

  /// The value that [value] will become after performing a
  /// [SemanticsAction.increase] action.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// This exposes the raw text of the [attributedIncreasedValue].
  String get increasedValue => attributedIncreasedValue.string;

  /// The value that [value] will become after performing a
  /// [SemanticsAction.increase] action in [AttributedString] format.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also [increasedValue], which exposes just the raw text.
  final AttributedString attributedIncreasedValue;

  /// The value that [value] will become after performing a
  /// [SemanticsAction.decrease] action.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// This exposes the raw text of the [attributedDecreasedValue].
  String get decreasedValue => attributedDecreasedValue.string;

  /// The value that [value] will become after performing a
  /// [SemanticsAction.decrease] action in [AttributedString] format.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also [decreasedValue], which exposes just the raw text.
  final AttributedString attributedDecreasedValue;

  /// A brief description of the result of performing an action on this node.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// This exposes the raw text of the [attributedHint].
  String get hint => attributedHint.string;

  /// A brief description of the result of performing an action on this node
  /// in [AttributedString] format.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also [hint], which exposes just the raw text.
  final AttributedString attributedHint;

  /// A textual description of the widget's tooltip.
  ///
  /// The reading direction is given by [textDirection].
  final String tooltip;

  /// Indicates that this subtree represents a heading.
  ///
  /// A value of 0 indicates that it is not a heading. The value should be a
  /// number between 1 and 6, indicating the hierarchical level as a heading.
  final int headingLevel;

  /// The reading direction for the text in [label], [value],
  /// [increasedValue], [decreasedValue], and [hint].
  final TextDirection? textDirection;

  /// The currently selected text (or the position of the cursor) within [value]
  /// if this node represents a text field.
  final TextSelection? textSelection;

  /// The total number of scrollable children that contribute to semantics.
  ///
  /// If the number of children are unknown or unbounded, this value will be
  /// null.
  final int? scrollChildCount;

  /// The index of the first visible semantic child of a scroll node.
  final int? scrollIndex;

  /// Indicates the current scrolling position in logical pixels if the node is
  /// scrollable.
  ///
  /// The properties [scrollExtentMin] and [scrollExtentMax] indicate the valid
  /// in-range values for this property. The value for [scrollPosition] may
  /// (temporarily) be outside that range, e.g. during an overscroll.
  ///
  /// See also:
  ///
  ///  * [ScrollPosition.pixels], from where this value is usually taken.
  final double? scrollPosition;

  /// Indicates the maximum in-range value for [scrollPosition] if the node is
  /// scrollable.
  ///
  /// This value may be infinity if the scroll is unbound.
  ///
  /// See also:
  ///
  ///  * [ScrollPosition.maxScrollExtent], from where this value is usually taken.
  final double? scrollExtentMax;

  /// Indicates the minimum in-range value for [scrollPosition] if the node is
  /// scrollable.
  ///
  /// This value may be infinity if the scroll is unbound.
  ///
  /// See also:
  ///
  ///  * [ScrollPosition.minScrollExtent], from where this value is usually taken.
  final double? scrollExtentMin;

  /// The id of the platform view, whose semantics nodes will be added as
  /// children to this node.
  ///
  /// If this value is non-null, the SemanticsNode must not have any children
  /// as those would be replaced by the semantics nodes of the referenced
  /// platform view.
  ///
  /// See also:
  ///
  ///  * [AndroidView], which is the platform view for Android.
  ///  * [UiKitView], which is the platform view for iOS.
  final int? platformViewId;

  /// The maximum number of characters that can be entered into an editable
  /// text field.
  ///
  /// For the purpose of this function a character is defined as one Unicode
  /// scalar value.
  ///
  /// This should only be set when [SemanticsFlag.isTextField] is set. Defaults
  /// to null, which means no limit is imposed on the text field.
  final int? maxValueLength;

  /// The current number of characters that have been entered into an editable
  /// text field.
  ///
  /// For the purpose of this function a character is defined as one Unicode
  /// scalar value.
  ///
  /// This should only be set when [SemanticsFlag.isTextField] is set. This must
  /// be set when [maxValueLength] is set.
  final int? currentValueLength;

  /// The URL that this node links to.
  ///
  /// See also:
  ///
  /// * [SemanticsFlag.isLink], which indicates that this node is a link.
  final Uri? linkUrl;

  /// The bounding box for this node in its coordinate system.
  final Rect rect;

  /// The set of [SemanticsTag]s associated with this node.
  final Set<SemanticsTag>? tags;

  /// The transform from this node's coordinate system to its parent's coordinate system.
  ///
  /// By default, the transform is null, which represents the identity
  /// transformation (i.e., that this node has the same coordinate system as its
  /// parent).
  final Matrix4? transform;

  /// The elevation of this node relative to the parent semantics node.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.elevation] for a detailed discussion regarding
  ///    elevation and semantics.
  final double elevation;

  /// The extent of this node along the z-axis beyond its [elevation]
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.thickness] for a more detailed definition.
  final double thickness;

  /// The identifiers for the custom semantics actions and standard action
  /// overrides for this node.
  ///
  /// The list must be sorted in increasing order.
  ///
  /// See also:
  ///
  ///  * [CustomSemanticsAction], for an explanation of custom actions.
  final List<int>? customSemanticsActionIds;

  /// Whether [flags] contains the given flag.
  bool hasFlag(SemanticsFlag flag) => (flags & flag.index) != 0;

  /// Whether [actions] contains the given action.
  bool hasAction(SemanticsAction action) => (actions & action.index) != 0;

  @override
  String toStringShort() => objectRuntimeType(this, 'SemanticsData');

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Rect>('rect', rect, showName: false));
    properties.add(TransformProperty('transform', transform, showName: false, defaultValue: null));
    properties.add(DoubleProperty('elevation', elevation, defaultValue: 0.0));
    properties.add(DoubleProperty('thickness', thickness, defaultValue: 0.0));
    final List<String> actionSummary = <String>[
      for (final SemanticsAction action in SemanticsAction.values)
        if ((actions & action.index) != 0) action.name,
    ];
    final List<String?> customSemanticsActionSummary =
        customSemanticsActionIds!
            .map<String?>((int actionId) => CustomSemanticsAction.getAction(actionId)!.label)
            .toList();
    properties.add(IterableProperty<String>('actions', actionSummary, ifEmpty: null));
    properties.add(
      IterableProperty<String?>('customActions', customSemanticsActionSummary, ifEmpty: null),
    );

    final List<String> flagSummary = <String>[
      for (final SemanticsFlag flag in SemanticsFlag.values)
        if ((flags & flag.index) != 0) flag.name,
    ];
    properties.add(IterableProperty<String>('flags', flagSummary, ifEmpty: null));
    properties.add(StringProperty('identifier', identifier, defaultValue: ''));
    properties.add(AttributedStringProperty('label', attributedLabel));
    properties.add(AttributedStringProperty('value', attributedValue));
    properties.add(AttributedStringProperty('increasedValue', attributedIncreasedValue));
    properties.add(AttributedStringProperty('decreasedValue', attributedDecreasedValue));
    properties.add(AttributedStringProperty('hint', attributedHint));
    properties.add(StringProperty('tooltip', tooltip, defaultValue: ''));
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection, defaultValue: null));
    if (textSelection?.isValid ?? false) {
      properties.add(
        MessageProperty('textSelection', '[${textSelection!.start}, ${textSelection!.end}]'),
      );
    }
    properties.add(IntProperty('platformViewId', platformViewId, defaultValue: null));
    properties.add(IntProperty('maxValueLength', maxValueLength, defaultValue: null));
    properties.add(IntProperty('currentValueLength', currentValueLength, defaultValue: null));
    properties.add(IntProperty('scrollChildren', scrollChildCount, defaultValue: null));
    properties.add(IntProperty('scrollIndex', scrollIndex, defaultValue: null));
    properties.add(DoubleProperty('scrollExtentMin', scrollExtentMin, defaultValue: null));
    properties.add(DoubleProperty('scrollPosition', scrollPosition, defaultValue: null));
    properties.add(DoubleProperty('scrollExtentMax', scrollExtentMax, defaultValue: null));
    properties.add(IntProperty('headingLevel', headingLevel, defaultValue: 0));
    properties.add(DiagnosticsProperty<Uri>('linkUrl', linkUrl, defaultValue: null));
  }

  @override
  bool operator ==(Object other) {
    return other is SemanticsData &&
        other.flags == flags &&
        other.actions == actions &&
        other.identifier == identifier &&
        other.attributedLabel == attributedLabel &&
        other.attributedValue == attributedValue &&
        other.attributedIncreasedValue == attributedIncreasedValue &&
        other.attributedDecreasedValue == attributedDecreasedValue &&
        other.attributedHint == attributedHint &&
        other.tooltip == tooltip &&
        other.textDirection == textDirection &&
        other.rect == rect &&
        setEquals(other.tags, tags) &&
        other.scrollChildCount == scrollChildCount &&
        other.scrollIndex == scrollIndex &&
        other.textSelection == textSelection &&
        other.scrollPosition == scrollPosition &&
        other.scrollExtentMax == scrollExtentMax &&
        other.scrollExtentMin == scrollExtentMin &&
        other.platformViewId == platformViewId &&
        other.maxValueLength == maxValueLength &&
        other.currentValueLength == currentValueLength &&
        other.transform == transform &&
        other.elevation == elevation &&
        other.thickness == thickness &&
        other.headingLevel == headingLevel &&
        other.linkUrl == linkUrl &&
        _sortedListsEqual(other.customSemanticsActionIds, customSemanticsActionIds);
  }

  @override
  int get hashCode => Object.hash(
    flags,
    actions,
    identifier,
    attributedLabel,
    attributedValue,
    attributedIncreasedValue,
    attributedDecreasedValue,
    attributedHint,
    tooltip,
    textDirection,
    rect,
    tags,
    textSelection,
    scrollChildCount,
    scrollIndex,
    scrollPosition,
    scrollExtentMax,
    scrollExtentMin,
    platformViewId,
    Object.hash(
      maxValueLength,
      currentValueLength,
      transform,
      elevation,
      thickness,
      headingLevel,
      linkUrl,
      customSemanticsActionIds == null ? null : Object.hashAll(customSemanticsActionIds!),
    ),
  );

  static bool _sortedListsEqual(List<int>? left, List<int>? right) {
    if (left == null && right == null) {
      return true;
    }
    if (left != null && right != null) {
      if (left.length != right.length) {
        return false;
      }
      for (int i = 0; i < left.length; i++) {
        if (left[i] != right[i]) {
          return false;
        }
      }
      return true;
    }
    return false;
  }
}

class _SemanticsDiagnosticableNode extends DiagnosticableNode<SemanticsNode> {
  _SemanticsDiagnosticableNode({
    super.name,
    required super.value,
    required super.style,
    required this.childOrder,
  });

  final DebugSemanticsDumpOrder childOrder;

  @override
  List<DiagnosticsNode> getChildren() => value.debugDescribeChildren(childOrder: childOrder);
}

/// Provides hint values which override the default hints on supported
/// platforms.
///
/// On iOS, these values are always ignored.
@immutable
class SemanticsHintOverrides extends DiagnosticableTree {
  /// Creates a semantics hint overrides.
  const SemanticsHintOverrides({this.onTapHint, this.onLongPressHint})
    : assert(onTapHint != ''),
      assert(onLongPressHint != '');

  /// The hint text for a tap action.
  ///
  /// If null, the standard hint is used instead.
  ///
  /// The hint should describe what happens when a tap occurs, not the
  /// manner in which a tap is accomplished.
  ///
  /// Bad: 'Double tap to show movies'.
  /// Good: 'show movies'.
  final String? onTapHint;

  /// The hint text for a long press action.
  ///
  /// If null, the standard hint is used instead.
  ///
  /// The hint should describe what happens when a long press occurs, not
  /// the manner in which the long press is accomplished.
  ///
  /// Bad: 'Double tap and hold to show tooltip'.
  /// Good: 'show tooltip'.
  final String? onLongPressHint;

  /// Whether there are any non-null hint values.
  bool get isNotEmpty => onTapHint != null || onLongPressHint != null;

  @override
  int get hashCode => Object.hash(onTapHint, onLongPressHint);

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SemanticsHintOverrides &&
        other.onTapHint == onTapHint &&
        other.onLongPressHint == onLongPressHint;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(StringProperty('onTapHint', onTapHint, defaultValue: null));
    properties.add(StringProperty('onLongPressHint', onLongPressHint, defaultValue: null));
  }
}

/// Contains properties used by assistive technologies to make the application
/// more accessible.
///
/// The properties of this class are used to generate a [SemanticsNode]s in the
/// semantics tree.
@immutable
class SemanticsProperties extends DiagnosticableTree {
  /// Creates a semantic annotation.
  const SemanticsProperties({
    this.enabled,
    this.checked,
    this.mixed,
    this.expanded,
    this.selected,
    this.toggled,
    this.button,
    this.link,
    this.linkUrl,
    this.header,
    this.headingLevel,
    this.textField,
    this.slider,
    this.keyboardKey,
    this.readOnly,
    this.focusable,
    this.focused,
    this.inMutuallyExclusiveGroup,
    this.hidden,
    this.obscured,
    this.multiline,
    this.scopesRoute,
    this.namesRoute,
    this.image,
    this.liveRegion,
    this.maxValueLength,
    this.currentValueLength,
    this.identifier,
    this.label,
    this.attributedLabel,
    this.value,
    this.attributedValue,
    this.increasedValue,
    this.attributedIncreasedValue,
    this.decreasedValue,
    this.attributedDecreasedValue,
    this.hint,
    this.tooltip,
    this.attributedHint,
    this.hintOverrides,
    this.textDirection,
    this.sortKey,
    this.tagForChildren,
    this.onTap,
    this.onLongPress,
    this.onScrollLeft,
    this.onScrollRight,
    this.onScrollUp,
    this.onScrollDown,
    this.onIncrease,
    this.onDecrease,
    this.onCopy,
    this.onCut,
    this.onPaste,
    this.onMoveCursorForwardByCharacter,
    this.onMoveCursorBackwardByCharacter,
    this.onMoveCursorForwardByWord,
    this.onMoveCursorBackwardByWord,
    this.onSetSelection,
    this.onSetText,
    this.onDidGainAccessibilityFocus,
    this.onDidLoseAccessibilityFocus,
    this.onFocus,
    this.onDismiss,
    this.customSemanticsActions,
  }) : assert(
         label == null || attributedLabel == null,
         'Only one of label or attributedLabel should be provided',
       ),
       assert(
         value == null || attributedValue == null,
         'Only one of value or attributedValue should be provided',
       ),
       assert(
         increasedValue == null || attributedIncreasedValue == null,
         'Only one of increasedValue or attributedIncreasedValue should be provided',
       ),
       assert(
         decreasedValue == null || attributedDecreasedValue == null,
         'Only one of decreasedValue or attributedDecreasedValue should be provided',
       ),
       assert(
         hint == null || attributedHint == null,
         'Only one of hint or attributedHint should be provided',
       ),
       assert(
         headingLevel == null || (headingLevel > 0 && headingLevel <= 6),
         'Heading level must be between 1 and 6',
       ),
       assert(linkUrl == null || (link ?? false), 'If linkUrl is set then link must be true');

  /// If non-null, indicates that this subtree represents something that can be
  /// in an enabled or disabled state.
  ///
  /// For example, a button that a user can currently interact with would set
  /// this field to true. A button that currently does not respond to user
  /// interactions would set this field to false.
  final bool? enabled;

  /// If non-null, indicates that this subtree represents a checkbox
  /// or similar widget with a "checked" state, and what its current
  /// state is.
  ///
  /// When the [Checkbox.value] of a tristate Checkbox is null,
  /// indicating a mixed-state, this value shall be false, in which
  /// case, [mixed] will be true.
  ///
  /// This is mutually exclusive with [toggled] and [mixed].
  final bool? checked;

  /// If non-null, indicates that this subtree represents a checkbox
  /// or similar widget with a "half-checked" state or similar, and
  /// whether it is currently in this half-checked state.
  ///
  /// This must be null when [Checkbox.tristate] is false, or
  /// when the widget is not a checkbox. When a tristate
  /// checkbox is fully unchecked/checked, this value shall
  /// be false.
  ///
  /// This is mutually exclusive with [checked] and [toggled].
  final bool? mixed;

  /// If non-null, indicates that this subtree represents something
  /// that can be in an "expanded" or "collapsed" state.
  ///
  /// For example, if a [SubmenuButton] is opened, this property
  /// should be set to true; otherwise, this property should be
  /// false.
  final bool? expanded;

  /// If non-null, indicates that this subtree represents a toggle switch
  /// or similar widget with an "on" state, and what its current
  /// state is.
  ///
  /// This is mutually exclusive with [checked] and [mixed].
  final bool? toggled;

  /// If non-null indicates that this subtree represents something that can be
  /// in a selected or unselected state, and what its current state is.
  ///
  /// The active tab in a tab bar for example is considered "selected", whereas
  /// all other tabs are unselected.
  final bool? selected;

  /// If non-null, indicates that this subtree represents a button.
  ///
  /// TalkBack/VoiceOver provides users with the hint "button" when a button
  /// is focused.
  final bool? button;

  /// If non-null, indicates that this subtree represents a link.
  ///
  /// iOS's VoiceOver provides users with a unique hint when a link is focused.
  /// Android's Talkback will announce a link hint the same way it does a
  /// button.
  final bool? link;

  /// If non-null, indicates that this subtree represents a header.
  ///
  /// A header divides into sections. For example, an address book application
  /// might define headers A, B, C, etc. to divide the list of alphabetically
  /// sorted contacts into sections.
  final bool? header;

  /// If non-null, indicates that this subtree represents a text field.
  ///
  /// TalkBack/VoiceOver provide special affordances to enter text into a
  /// text field.
  final bool? textField;

  /// If non-null, indicates that this subtree represents a slider.
  ///
  /// Talkback/\VoiceOver provides users with the hint "slider" when a
  /// slider is focused.
  final bool? slider;

  /// If non-null, indicates that this subtree represents a keyboard key.
  final bool? keyboardKey;

  /// If non-null, indicates that this subtree is read only.
  ///
  /// Only applicable when [textField] is true.
  ///
  /// TalkBack/VoiceOver will treat it as non-editable text field.
  final bool? readOnly;

  /// If non-null, whether the node is able to hold input focus.
  ///
  /// If [focusable] is set to false, then [focused] must not be true.
  ///
  /// Input focus indicates that the node will receive keyboard events. It is not
  /// to be confused with accessibility focus. Accessibility focus is the
  /// green/black rectangular highlight that TalkBack/VoiceOver draws around the
  /// element it is reading, and is separate from input focus.
  final bool? focusable;

  /// If non-null, whether the node currently holds input focus.
  ///
  /// At most one node in the tree should hold input focus at any point in time,
  /// and it should not be set to true if [focusable] is false.
  ///
  /// Input focus indicates that the node will receive keyboard events. It is not
  /// to be confused with accessibility focus. Accessibility focus is the
  /// green/black rectangular highlight that TalkBack/VoiceOver draws around the
  /// element it is reading, and is separate from input focus.
  final bool? focused;

  /// If non-null, whether a semantic node is in a mutually exclusive group.
  ///
  /// For example, a radio button is in a mutually exclusive group because only
  /// one radio button in that group can be marked as [checked].
  final bool? inMutuallyExclusiveGroup;

  /// If non-null, whether the node is considered hidden.
  ///
  /// Hidden elements are currently not visible on screen. They may be covered
  /// by other elements or positioned outside of the visible area of a viewport.
  ///
  /// Hidden elements cannot gain accessibility focus though regular touch. The
  /// only way they can be focused is by moving the focus to them via linear
  /// navigation.
  ///
  /// Platforms are free to completely ignore hidden elements and new platforms
  /// are encouraged to do so.
  ///
  /// Instead of marking an element as hidden it should usually be excluded from
  /// the semantics tree altogether. Hidden elements are only included in the
  /// semantics tree to work around platform limitations and they are mainly
  /// used to implement accessibility scrolling on iOS.
  final bool? hidden;

  /// If non-null, whether [value] should be obscured.
  ///
  /// This option is usually set in combination with [textField] to indicate
  /// that the text field contains a password (or other sensitive information).
  /// Doing so instructs screen readers to not read out the [value].
  final bool? obscured;

  /// Whether the [value] is coming from a field that supports multiline text
  /// editing.
  ///
  /// This option is only meaningful when [textField] is true to indicate
  /// whether it's a single-line or multiline text field.
  ///
  /// This option is null when [textField] is false.
  final bool? multiline;

  /// If non-null, whether the node corresponds to the root of a subtree for
  /// which a route name should be announced.
  ///
  /// Generally, this is set in combination with
  /// [SemanticsConfiguration.explicitChildNodes], since nodes with this flag
  /// are not considered focusable by Android or iOS.
  ///
  /// See also:
  ///
  ///  * [SemanticsFlag.scopesRoute] for a description of how the announced
  ///    value is selected.
  final bool? scopesRoute;

  /// If non-null, whether the node contains the semantic label for a route.
  ///
  /// See also:
  ///
  ///  * [SemanticsFlag.namesRoute] for a description of how the name is used.
  final bool? namesRoute;

  /// If non-null, whether the node represents an image.
  ///
  /// See also:
  ///
  ///  * [SemanticsFlag.isImage], for the flag this setting controls.
  final bool? image;

  /// If non-null, whether the node should be considered a live region.
  ///
  /// A live region indicates that updates to semantics node are important.
  /// Platforms may use this information to make polite announcements to the
  /// user to inform them of updates to this node.
  ///
  /// An example of a live region is a [SnackBar] widget. On Android and iOS,
  /// live region causes a polite announcement to be generated automatically,
  /// even if the widget does not have accessibility focus. This announcement
  /// may not be spoken if the OS accessibility services are already
  /// announcing something else, such as reading the label of a focused widget
  /// or providing a system announcement.
  ///
  /// See also:
  ///
  ///  * [SemanticsFlag.isLiveRegion], the semantics flag this setting controls.
  ///  * [SemanticsConfiguration.liveRegion], for a full description of a live region.
  final bool? liveRegion;

  /// The maximum number of characters that can be entered into an editable
  /// text field.
  ///
  /// For the purpose of this function a character is defined as one Unicode
  /// scalar value.
  ///
  /// This should only be set when [textField] is true. Defaults to null,
  /// which means no limit is imposed on the text field.
  final int? maxValueLength;

  /// The current number of characters that have been entered into an editable
  /// text field.
  ///
  /// For the purpose of this function a character is defined as one Unicode
  /// scalar value.
  ///
  /// This should only be set when [textField] is true. Must be set when
  /// [maxValueLength] is set.
  final int? currentValueLength;

  /// {@template flutter.semantics.SemanticsProperties.identifier}
  /// Provides an identifier for the semantics node in native accessibility hierarchy.
  ///
  /// This value is not exposed to the users of the app.
  ///
  /// It's usually used for UI testing with tools that work by querying the
  /// native accessibility, like UIAutomator, XCUITest, or Appium. It can be
  /// matched with [CommonFinders.bySemanticsIdentifier].
  ///
  /// On Android, this is used for `AccessibilityNodeInfo.setViewIdResourceName`.
  /// It'll be appear in accessibility hierarchy as `resource-id`.
  ///
  /// On iOS, this will set `UIAccessibilityElement.accessibilityIdentifier`.
  ///
  /// On web, this will set a `flt-semantics-identifier` attribute on the DOM element
  /// that corresponds to the semantics node.
  /// {@endtemplate}
  final String? identifier;

  /// Provides a textual description of the widget.
  ///
  /// If a label is provided, there must either by an ambient [Directionality]
  /// or an explicit [textDirection] should be provided.
  ///
  /// Callers must not provide both [label] and [attributedLabel]. One or both
  /// must be null.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.label] for a description of how this is exposed
  ///    in TalkBack and VoiceOver.
  ///  * [attributedLabel] for an [AttributedString] version of this property.
  final String? label;

  /// Provides an [AttributedString] version of textual description of the widget.
  ///
  /// If a [attributedLabel] is provided, there must either by an ambient
  /// [Directionality] or an explicit [textDirection] should be provided.
  ///
  /// Callers must not provide both [label] and [attributedLabel]. One or both
  /// must be null.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.attributedLabel] for a description of how this
  ///    is exposed in TalkBack and VoiceOver.
  ///  * [label] for a plain string version of this property.
  final AttributedString? attributedLabel;

  /// Provides a textual description of the value of the widget.
  ///
  /// If a value is provided, there must either by an ambient [Directionality]
  /// or an explicit [textDirection] should be provided.
  ///
  /// Callers must not provide both [value] and [attributedValue], One or both
  /// must be null.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.value] for a description of how this is exposed
  ///    in TalkBack and VoiceOver.
  ///  * [attributedLabel] for an [AttributedString] version of this property.
  final String? value;

  /// Provides an [AttributedString] version of textual description of the value
  /// of the widget.
  ///
  /// If a [attributedValue] is provided, there must either by an ambient
  /// [Directionality] or an explicit [textDirection] should be provided.
  ///
  /// Callers must not provide both [value] and [attributedValue], One or both
  /// must be null.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.attributedValue] for a description of how this
  ///    is exposed in TalkBack and VoiceOver.
  ///  * [value] for a plain string version of this property.
  final AttributedString? attributedValue;

  /// The value that [value] or [attributedValue] will become after a
  /// [SemanticsAction.increase] action has been performed on this widget.
  ///
  /// If a value is provided, [onIncrease] must also be set and there must
  /// either be an ambient [Directionality] or an explicit [textDirection]
  /// must be provided.
  ///
  /// Callers must not provide both [increasedValue] and
  /// [attributedIncreasedValue], One or both must be null.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.increasedValue] for a description of how this
  ///    is exposed in TalkBack and VoiceOver.
  ///  * [attributedIncreasedValue] for an [AttributedString] version of this
  ///    property.
  final String? increasedValue;

  /// The [AttributedString] that [value] or [attributedValue] will become after
  /// a [SemanticsAction.increase] action has been performed on this widget.
  ///
  /// If a [attributedIncreasedValue] is provided, [onIncrease] must also be set
  /// and there must either be an ambient [Directionality] or an explicit
  /// [textDirection] must be provided.
  ///
  /// Callers must not provide both [increasedValue] and
  /// [attributedIncreasedValue], One or both must be null.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.attributedIncreasedValue] for a description of
  ///    how this is exposed in TalkBack and VoiceOver.
  ///  * [increasedValue] for a plain string version of this property.
  final AttributedString? attributedIncreasedValue;

  /// The value that [value] or [attributedValue] will become after a
  /// [SemanticsAction.decrease] action has been performed on this widget.
  ///
  /// If a value is provided, [onDecrease] must also be set and there must
  /// either be an ambient [Directionality] or an explicit [textDirection]
  /// must be provided.
  ///
  /// Callers must not provide both [decreasedValue] and
  /// [attributedDecreasedValue], One or both must be null.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.decreasedValue] for a description of how this
  ///    is exposed in TalkBack and VoiceOver.
  ///  * [attributedDecreasedValue] for an [AttributedString] version of this
  ///    property.
  final String? decreasedValue;

  /// The [AttributedString] that [value] or [attributedValue] will become after
  /// a [SemanticsAction.decrease] action has been performed on this widget.
  ///
  /// If a [attributedDecreasedValue] is provided, [onDecrease] must also be set
  /// and there must either be an ambient [Directionality] or an explicit
  /// [textDirection] must be provided.
  ///
  /// Callers must not provide both [decreasedValue] and
  /// [attributedDecreasedValue], One or both must be null/// provided.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.attributedDecreasedValue] for a description of
  ///    how this is exposed in TalkBack and VoiceOver.
  ///  * [decreasedValue] for a plain string version of this property.
  final AttributedString? attributedDecreasedValue;

  /// Provides a brief textual description of the result of an action performed
  /// on the widget.
  ///
  /// If a hint is provided, there must either be an ambient [Directionality]
  /// or an explicit [textDirection] should be provided.
  ///
  /// Callers must not provide both [hint] and [attributedHint], One or both
  /// must be null.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.hint] for a description of how this is exposed
  ///    in TalkBack and VoiceOver.
  ///  * [attributedHint] for an [AttributedString] version of this property.
  final String? hint;

  /// Provides an [AttributedString] version of brief textual description of the
  /// result of an action performed on the widget.
  ///
  /// If a [attributedHint] is provided, there must either by an ambient
  /// [Directionality] or an explicit [textDirection] should be provided.
  ///
  /// Callers must not provide both [hint] and [attributedHint], One or both
  /// must be null.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.attributedHint] for a description of how this
  ///    is exposed in TalkBack and VoiceOver.
  ///  * [hint] for a plain string version of this property.
  final AttributedString? attributedHint;

  /// Provides a textual description of the widget's tooltip.
  ///
  /// In Android, this property sets the `AccessibilityNodeInfo.setTooltipText`.
  /// In iOS, this property is appended to the end of the
  /// `UIAccessibilityElement.accessibilityLabel`.
  ///
  /// If a [tooltip] is provided, there must either by an ambient
  /// [Directionality] or an explicit [textDirection] should be provided.
  final String? tooltip;

  /// The heading level in the DOM document structure.
  ///
  /// This is only applied to web semantics and is ignored on other platforms.
  ///
  /// Screen readers will use this value to determine which part of the page
  /// structure this heading represents. A level 1 heading, indicated
  /// with aria-level="1", usually indicates the main heading of a page,
  /// a level 2 heading, defined with aria-level="2" the first subsection,
  /// a level 3 is a subsection of that, and so on.
  final int? headingLevel;

  /// Provides hint values which override the default hints on supported
  /// platforms.
  ///
  /// On Android, If no hint overrides are used then default [hint] will be
  /// combined with the [label]. Otherwise, the [hint] will be ignored as long
  /// as there as at least one non-null hint override.
  ///
  /// On iOS, these are always ignored and the default [hint] is used instead.
  final SemanticsHintOverrides? hintOverrides;

  /// The reading direction of the [label], [value], [increasedValue],
  /// [decreasedValue], and [hint].
  ///
  /// Defaults to the ambient [Directionality].
  final TextDirection? textDirection;

  /// Determines the position of this node among its siblings in the traversal
  /// sort order.
  ///
  /// This is used to describe the order in which the semantic node should be
  /// traversed by the accessibility services on the platform (e.g. VoiceOver
  /// on iOS and TalkBack on Android).
  final SemanticsSortKey? sortKey;

  /// A tag to be applied to the child [SemanticsNode]s of this widget.
  ///
  /// The tag is added to all child [SemanticsNode]s that pass through the
  /// [RenderObject] corresponding to this widget while looking to be attached
  /// to a parent SemanticsNode.
  ///
  /// Tags are used to communicate to a parent SemanticsNode that a child
  /// SemanticsNode was passed through a particular RenderObject. The parent can
  /// use this information to determine the shape of the semantics tree.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.addTagForChildren], to which the tags provided
  ///    here will be passed.
  final SemanticsTag? tagForChildren;

  /// The URL that this node links to.
  ///
  /// On the web, this is used to set the `href` attribute of the DOM element.
  ///
  /// See also:
  ///
  /// * https://developer.mozilla.org/en-US/docs/Web/HTML/Element/a#href
  final Uri? linkUrl;

  /// The handler for [SemanticsAction.tap].
  ///
  /// This is the semantic equivalent of a user briefly tapping the screen with
  /// the finger without moving it. For example, a button should implement this
  /// action.
  ///
  /// VoiceOver users on iOS and TalkBack users on Android *may* trigger this
  /// action by double-tapping the screen while an element is focused.
  ///
  /// Note: different OSes or assistive technologies may decide to interpret
  /// user inputs differently. Some may simulate real screen taps, while others
  /// may call semantics tap. One way to handle taps properly is to provide the
  /// same handler to both gesture tap and semantics tap.
  final VoidCallback? onTap;

  /// The handler for [SemanticsAction.longPress].
  ///
  /// This is the semantic equivalent of a user pressing and holding the screen
  /// with the finger for a few seconds without moving it.
  ///
  /// VoiceOver users on iOS and TalkBack users on Android *may* trigger this
  /// action by double-tapping the screen without lifting the finger after the
  /// second tap.
  ///
  /// Note: different OSes or assistive technologies may decide to interpret
  /// user inputs differently. Some may simulate real long presses, while others
  /// may call semantics long press. One way to handle long press properly is to
  /// provide the same handler to both gesture long press and semantics long
  /// press.
  final VoidCallback? onLongPress;

  /// The handler for [SemanticsAction.scrollLeft].
  ///
  /// This is the semantic equivalent of a user moving their finger across the
  /// screen from right to left. It should be recognized by controls that are
  /// horizontally scrollable.
  ///
  /// VoiceOver users on iOS can trigger this action by swiping left with three
  /// fingers. TalkBack users on Android can trigger this action by swiping
  /// right and then left in one motion path. On Android, [onScrollUp] and
  /// [onScrollLeft] share the same gesture. Therefore, only on of them should
  /// be provided.
  final VoidCallback? onScrollLeft;

  /// The handler for [SemanticsAction.scrollRight].
  ///
  /// This is the semantic equivalent of a user moving their finger across the
  /// screen from left to right. It should be recognized by controls that are
  /// horizontally scrollable.
  ///
  /// VoiceOver users on iOS can trigger this action by swiping right with three
  /// fingers. TalkBack users on Android can trigger this action by swiping
  /// left and then right in one motion path. On Android, [onScrollDown] and
  /// [onScrollRight] share the same gesture. Therefore, only on of them should
  /// be provided.
  final VoidCallback? onScrollRight;

  /// The handler for [SemanticsAction.scrollUp].
  ///
  /// This is the semantic equivalent of a user moving their finger across the
  /// screen from bottom to top. It should be recognized by controls that are
  /// vertically scrollable.
  ///
  /// VoiceOver users on iOS can trigger this action by swiping up with three
  /// fingers. TalkBack users on Android can trigger this action by swiping
  /// right and then left in one motion path. On Android, [onScrollUp] and
  /// [onScrollLeft] share the same gesture. Therefore, only on of them should
  /// be provided.
  final VoidCallback? onScrollUp;

  /// The handler for [SemanticsAction.scrollDown].
  ///
  /// This is the semantic equivalent of a user moving their finger across the
  /// screen from top to bottom. It should be recognized by controls that are
  /// vertically scrollable.
  ///
  /// VoiceOver users on iOS can trigger this action by swiping down with three
  /// fingers. TalkBack users on Android can trigger this action by swiping
  /// left and then right in one motion path. On Android, [onScrollDown] and
  /// [onScrollRight] share the same gesture. Therefore, only on of them should
  /// be provided.
  final VoidCallback? onScrollDown;

  /// The handler for [SemanticsAction.increase].
  ///
  /// This is a request to increase the value represented by the widget. For
  /// example, this action might be recognized by a slider control.
  ///
  /// If a [value] is set, [increasedValue] must also be provided and
  /// [onIncrease] must ensure that [value] will be set to [increasedValue].
  ///
  /// VoiceOver users on iOS can trigger this action by swiping up with one
  /// finger. TalkBack users on Android can trigger this action by pressing the
  /// volume up button.
  final VoidCallback? onIncrease;

  /// The handler for [SemanticsAction.decrease].
  ///
  /// This is a request to decrease the value represented by the widget. For
  /// example, this action might be recognized by a slider control.
  ///
  /// If a [value] is set, [decreasedValue] must also be provided and
  /// [onDecrease] must ensure that [value] will be set to [decreasedValue].
  ///
  /// VoiceOver users on iOS can trigger this action by swiping down with one
  /// finger. TalkBack users on Android can trigger this action by pressing the
  /// volume down button.
  final VoidCallback? onDecrease;

  /// The handler for [SemanticsAction.copy].
  ///
  /// This is a request to copy the current selection to the clipboard.
  ///
  /// TalkBack users on Android can trigger this action from the local context
  /// menu of a text field, for example.
  final VoidCallback? onCopy;

  /// The handler for [SemanticsAction.cut].
  ///
  /// This is a request to cut the current selection and place it in the
  /// clipboard.
  ///
  /// TalkBack users on Android can trigger this action from the local context
  /// menu of a text field, for example.
  final VoidCallback? onCut;

  /// The handler for [SemanticsAction.paste].
  ///
  /// This is a request to paste the current content of the clipboard.
  ///
  /// TalkBack users on Android can trigger this action from the local context
  /// menu of a text field, for example.
  final VoidCallback? onPaste;

  /// The handler for [SemanticsAction.moveCursorForwardByCharacter].
  ///
  /// This handler is invoked when the user wants to move the cursor in a
  /// text field forward by one character.
  ///
  /// TalkBack users can trigger this by pressing the volume up key while the
  /// input focus is in a text field.
  final MoveCursorHandler? onMoveCursorForwardByCharacter;

  /// The handler for [SemanticsAction.moveCursorBackwardByCharacter].
  ///
  /// This handler is invoked when the user wants to move the cursor in a
  /// text field backward by one character.
  ///
  /// TalkBack users can trigger this by pressing the volume down key while the
  /// input focus is in a text field.
  final MoveCursorHandler? onMoveCursorBackwardByCharacter;

  /// The handler for [SemanticsAction.moveCursorForwardByWord].
  ///
  /// This handler is invoked when the user wants to move the cursor in a
  /// text field backward by one word.
  ///
  /// TalkBack users can trigger this by pressing the volume down key while the
  /// input focus is in a text field.
  final MoveCursorHandler? onMoveCursorForwardByWord;

  /// The handler for [SemanticsAction.moveCursorBackwardByWord].
  ///
  /// This handler is invoked when the user wants to move the cursor in a
  /// text field backward by one word.
  ///
  /// TalkBack users can trigger this by pressing the volume down key while the
  /// input focus is in a text field.
  final MoveCursorHandler? onMoveCursorBackwardByWord;

  /// The handler for [SemanticsAction.setSelection].
  ///
  /// This handler is invoked when the user either wants to change the currently
  /// selected text in a text field or change the position of the cursor.
  ///
  /// TalkBack users can trigger this handler by selecting "Move cursor to
  /// beginning/end" or "Select all" from the local context menu.
  final SetSelectionHandler? onSetSelection;

  /// The handler for [SemanticsAction.setText].
  ///
  /// This handler is invoked when the user wants to replace the current text in
  /// the text field with a new text.
  ///
  /// Voice access users can trigger this handler by speaking `type <text>` to
  /// their Android devices.
  final SetTextHandler? onSetText;

  /// The handler for [SemanticsAction.didGainAccessibilityFocus].
  ///
  /// This handler is invoked when the node annotated with this handler gains
  /// the accessibility focus. The accessibility focus is the
  /// green (on Android with TalkBack) or black (on iOS with VoiceOver)
  /// rectangle shown on screen to indicate what element an accessibility
  /// user is currently interacting with.
  ///
  /// The accessibility focus is different from the input focus. The input focus
  /// is usually held by the element that currently responds to keyboard inputs.
  /// Accessibility focus and input focus can be held by two different nodes!
  ///
  /// See also:
  ///
  ///  * [onDidLoseAccessibilityFocus], which is invoked when the accessibility
  ///    focus is removed from the node.
  ///  * [onFocus], which is invoked when the assistive technology requests that
  ///    the input focus is gained by a widget.
  ///  * [FocusNode], [FocusScope], [FocusManager], which manage the input focus.
  final VoidCallback? onDidGainAccessibilityFocus;

  /// The handler for [SemanticsAction.didLoseAccessibilityFocus].
  ///
  /// This handler is invoked when the node annotated with this handler
  /// loses the accessibility focus. The accessibility focus is
  /// the green (on Android with TalkBack) or black (on iOS with VoiceOver)
  /// rectangle shown on screen to indicate what element an accessibility
  /// user is currently interacting with.
  ///
  /// The accessibility focus is different from the input focus. The input focus
  /// is usually held by the element that currently responds to keyboard inputs.
  /// Accessibility focus and input focus can be held by two different nodes!
  ///
  /// See also:
  ///
  ///  * [onDidGainAccessibilityFocus], which is invoked when the node gains
  ///    accessibility focus.
  ///  * [FocusNode], [FocusScope], [FocusManager], which manage the input focus.
  final VoidCallback? onDidLoseAccessibilityFocus;

  /// {@template flutter.semantics.SemanticsProperties.onFocus}
  /// The handler for [SemanticsAction.focus].
  ///
  /// This handler is invoked when the assistive technology requests that the
  /// focusable widget corresponding to this semantics node gain input focus.
  /// The [FocusNode] that manages the focus of the widget must gain focus. The
  /// widget must begin responding to relevant key events. For example:
  ///
  /// * Buttons must respond to tap/click events produced via keyboard shortcuts.
  /// * Text fields must become focused and editable, showing an on-screen
  ///   keyboard, if necessary.
  /// * Checkboxes, switches, and radio buttons must become toggleable using
  ///   keyboard shortcuts.
  ///
  /// Focus behavior is specific to the platform and to the assistive technology
  /// used. See the documentation of [SemanticsAction.focus] for more detail.
  ///
  /// See also:
  ///
  ///  * [onDidGainAccessibilityFocus], which is invoked when the node gains
  ///    accessibility focus.
  /// {@endtemplate}
  final VoidCallback? onFocus;

  /// The handler for [SemanticsAction.dismiss].
  ///
  /// This is a request to dismiss the currently focused node.
  ///
  /// TalkBack users on Android can trigger this action in the local context
  /// menu, and VoiceOver users on iOS can trigger this action with a standard
  /// gesture or menu option.
  final VoidCallback? onDismiss;

  /// A map from each supported [CustomSemanticsAction] to a provided handler.
  ///
  /// The handler associated with each custom action is called whenever a
  /// semantics action of type [SemanticsAction.customAction] is received. The
  /// provided argument will be an identifier used to retrieve an instance of
  /// a custom action which can then retrieve the correct handler from this map.
  ///
  /// See also:
  ///
  ///  * [CustomSemanticsAction], for an explanation of custom actions.
  final Map<CustomSemanticsAction, VoidCallback>? customSemanticsActions;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<bool>('checked', checked, defaultValue: null));
    properties.add(DiagnosticsProperty<bool>('mixed', mixed, defaultValue: null));
    properties.add(DiagnosticsProperty<bool>('expanded', expanded, defaultValue: null));
    properties.add(DiagnosticsProperty<bool>('selected', selected, defaultValue: null));
    properties.add(StringProperty('identifier', identifier, defaultValue: null));
    properties.add(StringProperty('label', label, defaultValue: null));
    properties.add(
      AttributedStringProperty('attributedLabel', attributedLabel, defaultValue: null),
    );
    properties.add(StringProperty('value', value, defaultValue: null));
    properties.add(
      AttributedStringProperty('attributedValue', attributedValue, defaultValue: null),
    );
    properties.add(StringProperty('increasedValue', value, defaultValue: null));
    properties.add(
      AttributedStringProperty(
        'attributedIncreasedValue',
        attributedIncreasedValue,
        defaultValue: null,
      ),
    );
    properties.add(StringProperty('decreasedValue', value, defaultValue: null));
    properties.add(
      AttributedStringProperty(
        'attributedDecreasedValue',
        attributedDecreasedValue,
        defaultValue: null,
      ),
    );
    properties.add(StringProperty('hint', hint, defaultValue: null));
    properties.add(AttributedStringProperty('attributedHint', attributedHint, defaultValue: null));
    properties.add(StringProperty('tooltip', tooltip, defaultValue: null));
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection, defaultValue: null));
    properties.add(DiagnosticsProperty<SemanticsSortKey>('sortKey', sortKey, defaultValue: null));
    properties.add(
      DiagnosticsProperty<SemanticsHintOverrides>(
        'hintOverrides',
        hintOverrides,
        defaultValue: null,
      ),
    );
  }

  @override
  String toStringShort() => objectRuntimeType(this, 'SemanticsProperties'); // the hashCode isn't important since we're immutable
}

/// In tests use this function to reset the counter used to generate
/// [SemanticsNode.id].
void debugResetSemanticsIdCounter() {
  SemanticsNode._lastIdentifier = 0;
}

/// A node that represents some semantic data.
///
/// The semantics tree is maintained during the semantics phase of the pipeline
/// (i.e., during [PipelineOwner.flushSemantics]), which happens after
/// compositing. The semantics tree is then uploaded into the engine for use
/// by assistive technology.
class SemanticsNode with DiagnosticableTreeMixin {
  /// Creates a semantic node.
  ///
  /// Each semantic node has a unique identifier that is assigned when the node
  /// is created.
  SemanticsNode({this.key, VoidCallback? showOnScreen})
    : _id = _generateNewId(),
      _showOnScreen = showOnScreen;

  /// Creates a semantic node to represent the root of the semantics tree.
  ///
  /// The root node is assigned an identifier of zero.
  SemanticsNode.root({this.key, VoidCallback? showOnScreen, required SemanticsOwner owner})
    : _id = 0,
      _showOnScreen = showOnScreen {
    attach(owner);
  }

  // The maximal semantic node identifier generated by the framework.
  //
  // The identifier range for semantic node IDs is split into 2, the least significant 16 bits are
  // reserved for framework generated IDs(generated with _generateNewId), and most significant 32
  // bits are reserved for engine generated IDs.
  static const int _maxFrameworkAccessibilityIdentifier = (1 << 16) - 1;

  static int _lastIdentifier = 0;
  static int _generateNewId() {
    _lastIdentifier = (_lastIdentifier + 1) % _maxFrameworkAccessibilityIdentifier;
    return _lastIdentifier;
  }

  /// Uniquely identifies this node in the list of sibling nodes.
  ///
  /// Keys are used during the construction of the semantics tree. They are not
  /// transferred to the engine.
  final Key? key;

  /// The unique identifier for this node.
  ///
  /// The root node has an id of zero. Other nodes are given a unique id
  /// when they are attached to a [SemanticsOwner]. If they are detached, their
  /// ids are invalid and should not be used.
  ///
  /// In rare circumstances, id may change if this node is detached and
  /// re-attached to the [SemanticsOwner]. This should only happen when the
  /// application has generated too many semantics nodes.
  int get id => _id;
  int _id;

  final VoidCallback? _showOnScreen;

  // GEOMETRY

  /// The transform from this node's coordinate system to its parent's coordinate system.
  ///
  /// By default, the transform is null, which represents the identity
  /// transformation (i.e., that this node has the same coordinate system as its
  /// parent).
  Matrix4? get transform => _transform;
  Matrix4? _transform;
  set transform(Matrix4? value) {
    if (!MatrixUtils.matrixEquals(_transform, value)) {
      _transform = value == null || MatrixUtils.isIdentity(value) ? null : value;
      _markDirty();
    }
  }

  /// The bounding box for this node in its coordinate system.
  Rect get rect => _rect;
  Rect _rect = Rect.zero;
  set rect(Rect value) {
    assert(value.isFinite, '$this (with $owner) tried to set a non-finite rect.');
    if (_rect != value) {
      _rect = value;
      _markDirty();
    }
  }

  /// The semantic clip from an ancestor that was applied to this node.
  ///
  /// Expressed in the coordinate system of the node. May be null if no clip has
  /// been applied.
  ///
  /// Descendant [SemanticsNode]s that are positioned outside of this rect will
  /// be excluded from the semantics tree. Descendant [SemanticsNode]s that are
  /// overlapping with this rect, but are outside of [parentPaintClipRect] will
  /// be included in the tree, but they will be marked as hidden because they
  /// are assumed to be not visible on screen.
  ///
  /// If this rect is null, all descendant [SemanticsNode]s outside of
  /// [parentPaintClipRect] will be excluded from the tree.
  ///
  /// If this rect is non-null it has to completely enclose
  /// [parentPaintClipRect]. If [parentPaintClipRect] is null this property is
  /// also null.
  Rect? parentSemanticsClipRect;

  /// The paint clip from an ancestor that was applied to this node.
  ///
  /// Expressed in the coordinate system of the node. May be null if no clip has
  /// been applied.
  ///
  /// Descendant [SemanticsNode]s that are positioned outside of this rect will
  /// either be excluded from the semantics tree (if they have no overlap with
  /// [parentSemanticsClipRect]) or they will be included and marked as hidden
  /// (if they are overlapping with [parentSemanticsClipRect]).
  ///
  /// This rect is completely enclosed by [parentSemanticsClipRect].
  ///
  /// If this rect is null [parentSemanticsClipRect] also has to be null.
  Rect? parentPaintClipRect;

  /// The elevation adjustment that the parent imposes on this node.
  ///
  /// The [elevation] property is relative to the elevation of the parent
  /// [SemanticsNode]. However, as [SemanticsConfiguration]s from various
  /// ascending [RenderObject]s are merged into each other to form that
  /// [SemanticsNode] the parent’s elevation may change. This requires an
  /// adjustment of the child’s relative elevation which is represented by this
  /// value.
  ///
  /// The value is rarely accessed directly. Instead, for most use cases the
  /// [elevation] value should be used, which includes this adjustment.
  ///
  /// See also:
  ///
  ///  * [elevation], the actual elevation of this [SemanticsNode].
  double? elevationAdjustment;

  /// The index of this node within the parent's list of semantic children.
  ///
  /// This includes all semantic nodes, not just those currently in the
  /// child list. For example, if a scrollable has five children but the first
  /// two are not visible (and thus not included in the list of children), then
  /// the index of the last node will still be 4.
  int? indexInParent;

  /// Whether the node is invisible.
  ///
  /// A node whose [rect] is outside of the bounds of the screen and hence not
  /// reachable for users is considered invisible if its semantic information
  /// is not merged into a (partially) visible parent as indicated by
  /// [isMergedIntoParent].
  ///
  /// An invisible node can be safely dropped from the semantic tree without
  /// losing semantic information that is relevant for describing the content
  /// currently shown on screen.
  bool get isInvisible => !isMergedIntoParent && rect.isEmpty;

  // MERGING

  /// Whether this node merges its semantic information into an ancestor node.
  ///
  /// This value indicates whether this node has any ancestors with
  /// [mergeAllDescendantsIntoThisNode] set to true.
  bool get isMergedIntoParent => parent != null && _isMergedIntoParent;
  bool _isMergedIntoParent = false;

  /// Whether the user can interact with this node in assistive technologies.
  ///
  /// This node can still receive accessibility focus even if this is true.
  /// Setting this to true prevents the user from activating pointer related
  /// [SemanticsAction]s, such as [SemanticsAction.tap] or
  /// [SemanticsAction.longPress].
  bool get areUserActionsBlocked => _areUserActionsBlocked;
  bool _areUserActionsBlocked = false;
  set areUserActionsBlocked(bool value) {
    if (_areUserActionsBlocked == value) {
      return;
    }
    _areUserActionsBlocked = value;
    _markDirty();
  }

  /// Whether this node is taking part in a merge of semantic information.
  ///
  /// This returns true if the node is either merged into an ancestor node or if
  /// decedent nodes are merged into this node.
  ///
  /// See also:
  ///
  ///  * [isMergedIntoParent]
  ///  * [mergeAllDescendantsIntoThisNode]
  bool get isPartOfNodeMerging => mergeAllDescendantsIntoThisNode || isMergedIntoParent;

  /// Whether this node and all of its descendants should be treated as one logical entity.
  bool get mergeAllDescendantsIntoThisNode => _mergeAllDescendantsIntoThisNode;
  bool _mergeAllDescendantsIntoThisNode = _kEmptyConfig.isMergingSemanticsOfDescendants;

  // CHILDREN

  /// Contains the children in inverse hit test order (i.e. paint order).
  List<SemanticsNode>? _children;

  /// A snapshot of `newChildren` passed to [_replaceChildren] that we keep in
  /// debug mode. It supports the assertion that user does not mutate the list
  /// of children.
  late List<SemanticsNode> _debugPreviousSnapshot;

  void _replaceChildren(List<SemanticsNode> newChildren) {
    assert(!newChildren.any((SemanticsNode child) => child == this));
    assert(() {
      final Set<SemanticsNode> seenChildren = <SemanticsNode>{};
      for (final SemanticsNode child in newChildren) {
        assert(seenChildren.add(child));
      } // check for duplicate adds
      return true;
    }());

    // The goal of this function is updating sawChange.
    if (_children != null) {
      for (final SemanticsNode child in _children!) {
        child._dead = true;
      }
    }
    for (final SemanticsNode child in newChildren) {
      child._dead = false;
    }
    bool sawChange = false;
    if (_children != null) {
      for (final SemanticsNode child in _children!) {
        if (child._dead) {
          if (child.parent == this) {
            // we might have already had our child stolen from us by
            // another node that is deeper in the tree.
            _dropChild(child);
          }
          sawChange = true;
        }
      }
    }
    for (final SemanticsNode child in newChildren) {
      if (child.parent != this) {
        if (child.parent != null) {
          // we're rebuilding the tree from the bottom up, so it's possible
          // that our child was, in the last pass, a child of one of our
          // ancestors. In that case, we drop the child eagerly here.
          // TODO(ianh): Find a way to assert that the same node didn't
          // actually appear in the tree in two places.
          child.parent?._dropChild(child);
        }
        assert(!child.attached);
        _adoptChild(child);
        sawChange = true;
      }
    }
    // Wait until the new children are adopted so isMergedIntoParent becomes
    // up-to-date.
    assert(() {
      if (identical(newChildren, _children)) {
        final List<DiagnosticsNode> mutationErrors = <DiagnosticsNode>[];
        if (newChildren.length != _debugPreviousSnapshot.length) {
          mutationErrors.add(
            ErrorDescription(
              "The list's length has changed from ${_debugPreviousSnapshot.length} "
              'to ${newChildren.length}.',
            ),
          );
        } else {
          for (int i = 0; i < newChildren.length; i++) {
            if (!identical(newChildren[i], _debugPreviousSnapshot[i])) {
              if (mutationErrors.isNotEmpty) {
                mutationErrors.add(ErrorSpacer());
              }
              mutationErrors.add(ErrorDescription('Child node at position $i was replaced:'));
              mutationErrors.add(
                _debugPreviousSnapshot[i].toDiagnosticsNode(
                  name: 'Previous child',
                  style: DiagnosticsTreeStyle.singleLine,
                ),
              );
              mutationErrors.add(
                newChildren[i].toDiagnosticsNode(
                  name: 'New child',
                  style: DiagnosticsTreeStyle.singleLine,
                ),
              );
            }
          }
        }
        if (mutationErrors.isNotEmpty) {
          throw FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary(
              'Failed to replace child semantics nodes because the list of `SemanticsNode`s was mutated.',
            ),
            ErrorHint(
              'Instead of mutating the existing list, create a new list containing the desired `SemanticsNode`s.',
            ),
            ErrorDescription('Error details:'),
            ...mutationErrors,
          ]);
        }
      }
      _debugPreviousSnapshot = List<SemanticsNode>.of(newChildren);

      SemanticsNode ancestor = this;
      while (ancestor.parent is SemanticsNode) {
        ancestor = ancestor.parent!;
      }
      assert(!newChildren.any((SemanticsNode child) => child == ancestor));
      return true;
    }());

    if (!sawChange && _children != null) {
      assert(newChildren.length == _children!.length);
      // Did the order change?
      for (int i = 0; i < _children!.length; i++) {
        if (_children![i].id != newChildren[i].id) {
          sawChange = true;
          break;
        }
      }
    }
    _children = newChildren;
    if (sawChange) {
      _markDirty();
    }
  }

  /// Whether this node has a non-zero number of children.
  bool get hasChildren => _children?.isNotEmpty ?? false;
  bool _dead = false;

  /// The number of children this node has.
  int get childrenCount => hasChildren ? _children!.length : 0;

  /// Visits the immediate children of this node.
  ///
  /// This function calls visitor for each immediate child until visitor returns
  /// false.
  void visitChildren(SemanticsNodeVisitor visitor) {
    if (_children != null) {
      for (final SemanticsNode child in _children!) {
        if (!visitor(child)) {
          return;
        }
      }
    }
  }

  /// Visit all the descendants of this node.
  ///
  /// This function calls visitor for each descendant in a pre-order traversal
  /// until visitor returns false. Returns true if all the visitor calls
  /// returned true, otherwise returns false.
  bool _visitDescendants(SemanticsNodeVisitor visitor) {
    if (_children != null) {
      for (final SemanticsNode child in _children!) {
        if (!visitor(child) || !child._visitDescendants(visitor)) {
          return false;
        }
      }
    }
    return true;
  }

  /// The owner for this node (null if unattached).
  ///
  /// The entire semantics tree that this node belongs to will have the same owner.
  SemanticsOwner? get owner => _owner;
  SemanticsOwner? _owner;

  /// Whether the semantics tree this node belongs to is attached to a [SemanticsOwner].
  ///
  /// This becomes true during the call to [attach].
  ///
  /// This becomes false during the call to [detach].
  bool get attached => _owner != null;

  /// The parent of this node in the semantics tree.
  ///
  /// The [parent] of the root node in the semantics tree is null.
  SemanticsNode? get parent => _parent;
  SemanticsNode? _parent;

  /// The depth of this node in the semantics tree.
  ///
  /// The depth of nodes in a tree monotonically increases as you traverse down
  /// the tree.  There's no guarantee regarding depth between siblings.
  ///
  /// The depth is used to ensure that nodes are processed in depth order.
  int get depth => _depth;
  int _depth = 0;

  void _redepthChild(SemanticsNode child) {
    assert(child.owner == owner);
    if (child._depth <= _depth) {
      child._depth = _depth + 1;
      child._redepthChildren();
    }
  }

  void _redepthChildren() {
    _children?.forEach(_redepthChild);
  }

  void _updateChildMergeFlagRecursively(SemanticsNode child) {
    assert(child.owner == owner);
    final bool childShouldMergeToParent = isPartOfNodeMerging;

    if (childShouldMergeToParent == child._isMergedIntoParent) {
      return;
    }

    child._isMergedIntoParent = childShouldMergeToParent;
    _markDirty();

    if (child.mergeAllDescendantsIntoThisNode) {
      // No need to update the descendants since `child` has the merge flag set.
    } else {
      child._updateChildrenMergeFlags();
    }
  }

  void _updateChildrenMergeFlags() {
    _children?.forEach(_updateChildMergeFlagRecursively);
  }

  void _adoptChild(SemanticsNode child) {
    assert(child._parent == null);
    assert(() {
      SemanticsNode node = this;
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
    _redepthChild(child);
    _updateChildMergeFlagRecursively(child);
  }

  void _dropChild(SemanticsNode child) {
    assert(child._parent == this);
    assert(child.attached == attached);
    child._parent = null;
    if (attached) {
      child.detach();
    }
  }

  /// Mark this node as attached to the given owner.
  @visibleForTesting
  void attach(SemanticsOwner owner) {
    assert(_owner == null);
    _owner = owner;
    while (owner._nodes.containsKey(id)) {
      // Ids may repeat if the Flutter has generated > 2^16 ids. We need to keep
      // regenerating the id until we found an id that is not used.
      _id = _generateNewId();
    }
    owner._nodes[id] = this;
    owner._detachedNodes.remove(this);
    if (_dirty) {
      _dirty = false;
      _markDirty();
    }
    if (_children != null) {
      for (final SemanticsNode child in _children!) {
        child.attach(owner);
      }
    }
  }

  /// Mark this node as detached from its owner.
  @visibleForTesting
  void detach() {
    assert(_owner != null);
    assert(owner!._nodes.containsKey(id));
    assert(!owner!._detachedNodes.contains(this));
    owner!._nodes.remove(id);
    owner!._detachedNodes.add(this);
    _owner = null;
    assert(parent == null || attached == parent!.attached);
    if (_children != null) {
      for (final SemanticsNode child in _children!) {
        // The list of children may be stale and may contain nodes that have
        // been assigned to a different parent.
        if (child.parent == this) {
          child.detach();
        }
      }
    }
    // The other side will have forgotten this node if we ever send
    // it again, so make sure to mark it dirty so that it'll get
    // sent if it is resurrected.
    _markDirty();
  }

  // DIRTY MANAGEMENT

  bool _dirty = false;
  void _markDirty() {
    if (_dirty) {
      return;
    }
    _dirty = true;
    if (attached) {
      assert(!owner!._detachedNodes.contains(this));
      owner!._dirtyNodes.add(this);
    }
  }

  bool _isDifferentFromCurrentSemanticAnnotation(SemanticsConfiguration config) {
    return _attributedLabel != config.attributedLabel ||
        _attributedHint != config.attributedHint ||
        _elevation != config.elevation ||
        _thickness != config.thickness ||
        _attributedValue != config.attributedValue ||
        _attributedIncreasedValue != config.attributedIncreasedValue ||
        _attributedDecreasedValue != config.attributedDecreasedValue ||
        _tooltip != config.tooltip ||
        _flags != config._flags ||
        _textDirection != config.textDirection ||
        _sortKey != config._sortKey ||
        _textSelection != config._textSelection ||
        _scrollPosition != config._scrollPosition ||
        _scrollExtentMax != config._scrollExtentMax ||
        _scrollExtentMin != config._scrollExtentMin ||
        _actionsAsBits != config._actionsAsBits ||
        indexInParent != config.indexInParent ||
        platformViewId != config.platformViewId ||
        _maxValueLength != config._maxValueLength ||
        _currentValueLength != config._currentValueLength ||
        _mergeAllDescendantsIntoThisNode != config.isMergingSemanticsOfDescendants ||
        _areUserActionsBlocked != config.isBlockingUserActions ||
        _headingLevel != config._headingLevel ||
        _linkUrl != config._linkUrl;
  }

  // TAGS, LABELS, ACTIONS

  Map<SemanticsAction, SemanticsActionHandler> _actions = _kEmptyConfig._actions;
  Map<CustomSemanticsAction, VoidCallback> _customSemanticsActions =
      _kEmptyConfig._customSemanticsActions;

  int get _effectiveActionsAsBits =>
      _areUserActionsBlocked ? _actionsAsBits & _kUnblockedUserActions : _actionsAsBits;
  int _actionsAsBits = _kEmptyConfig._actionsAsBits;

  /// The [SemanticsTag]s this node is tagged with.
  ///
  /// Tags are used during the construction of the semantics tree. They are not
  /// transferred to the engine.
  Set<SemanticsTag>? tags;

  /// Whether this node is tagged with `tag`.
  bool isTagged(SemanticsTag tag) => tags != null && tags!.contains(tag);

  int _flags = _kEmptyConfig._flags;

  /// Whether this node currently has a given [SemanticsFlag].
  bool hasFlag(SemanticsFlag flag) => _flags & flag.index != 0;

  /// {@macro flutter.semantics.SemanticsProperties.identifier}
  String get identifier => _identifier;
  String _identifier = _kEmptyConfig.identifier;

  /// A textual description of this node.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// This exposes the raw text of the [attributedLabel].
  String get label => _attributedLabel.string;

  /// A textual description of this node in [AttributedString] format.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also [label], which exposes just the raw text.
  AttributedString get attributedLabel => _attributedLabel;
  AttributedString _attributedLabel = _kEmptyConfig.attributedLabel;

  /// A textual description for the current value of the node.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// This exposes the raw text of the [attributedValue].
  String get value => _attributedValue.string;

  /// A textual description for the current value of the node in
  /// [AttributedString] format.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also [value], which exposes just the raw text.
  AttributedString get attributedValue => _attributedValue;
  AttributedString _attributedValue = _kEmptyConfig.attributedValue;

  /// The value that [value] will have after a [SemanticsAction.increase] action
  /// has been performed.
  ///
  /// This property is only valid if the [SemanticsAction.increase] action is
  /// available on this node.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// This exposes the raw text of the [attributedIncreasedValue].
  String get increasedValue => _attributedIncreasedValue.string;

  /// The value in [AttributedString] format that [value] or [attributedValue]
  /// will have after a [SemanticsAction.increase] action has been performed.
  ///
  /// This property is only valid if the [SemanticsAction.increase] action is
  /// available on this node.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also [increasedValue], which exposes just the raw text.
  AttributedString get attributedIncreasedValue => _attributedIncreasedValue;
  AttributedString _attributedIncreasedValue = _kEmptyConfig.attributedIncreasedValue;

  /// The value that [value] will have after a [SemanticsAction.decrease] action
  /// has been performed.
  ///
  /// This property is only valid if the [SemanticsAction.decrease] action is
  /// available on this node.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// This exposes the raw text of the [attributedDecreasedValue].
  String get decreasedValue => _attributedDecreasedValue.string;

  /// The value in [AttributedString] format that [value] or [attributedValue]
  /// will have after a [SemanticsAction.decrease] action has been performed.
  ///
  /// This property is only valid if the [SemanticsAction.decrease] action is
  /// available on this node.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also [decreasedValue], which exposes just the raw text.
  AttributedString get attributedDecreasedValue => _attributedDecreasedValue;
  AttributedString _attributedDecreasedValue = _kEmptyConfig.attributedDecreasedValue;

  /// A brief description of the result of performing an action on this node.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// This exposes the raw text of the [attributedHint].
  String get hint => _attributedHint.string;

  /// A brief description of the result of performing an action on this node
  /// in [AttributedString] format.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also [hint], which exposes just the raw text.
  AttributedString get attributedHint => _attributedHint;
  AttributedString _attributedHint = _kEmptyConfig.attributedHint;

  /// A textual description of the widget's tooltip.
  ///
  /// The reading direction is given by [textDirection].
  String get tooltip => _tooltip;
  String _tooltip = _kEmptyConfig.tooltip;

  /// The elevation along the z-axis at which the [rect] of this [SemanticsNode]
  /// is located above its parent.
  ///
  /// The value is relative to the parent's [elevation]. The sum of the
  /// [elevation]s of all ancestor node plus this value determines the absolute
  /// elevation of this [SemanticsNode].
  ///
  /// See also:
  ///
  ///  * [thickness], which describes how much space in z-direction this
  ///    [SemanticsNode] occupies starting at this [elevation].
  ///  * [elevationAdjustment], which has been used to calculate this value.
  double get elevation => _elevation;
  double _elevation = _kEmptyConfig.elevation;

  /// Describes how much space the [SemanticsNode] takes up along the z-axis.
  ///
  /// A [SemanticsNode] represents multiple [RenderObject]s, which can be
  /// located at various elevations in 3D. The [thickness] is the difference
  /// between the absolute elevations of the lowest and highest [RenderObject]
  /// represented by this [SemanticsNode]. In other words, the thickness
  /// describes how high the box is that this [SemanticsNode] occupies in three
  /// dimensional space. The two other dimensions are defined by [rect].
  ///
  /// {@tool snippet}
  /// The following code stacks three [PhysicalModel]s on top of each other
  /// separated by non-zero elevations.
  ///
  /// [PhysicalModel] C is elevated 10.0 above [PhysicalModel] B, which in turn
  /// is elevated 5.0 above [PhysicalModel] A. The side view of this
  /// constellation looks as follows:
  ///
  /// ![A diagram illustrating the elevations of three PhysicalModels and their
  /// corresponding SemanticsNodes.](https://flutter.github.io/assets-for-api-docs/assets/semantics/SemanticsNode.thickness.png)
  ///
  /// In this example the [RenderObject]s for [PhysicalModel] C and B share one
  /// [SemanticsNode] Y. Given the elevations of those [RenderObject]s, this
  /// [SemanticsNode] has a [thickness] of 10.0 and an elevation of 5.0 over
  /// its parent [SemanticsNode] X.
  /// ```dart
  /// PhysicalModel( // A
  ///   color: Colors.amber,
  ///   child: Semantics(
  ///     explicitChildNodes: true,
  ///     child: const PhysicalModel( // B
  ///       color: Colors.brown,
  ///       elevation: 5.0,
  ///       child: PhysicalModel( // C
  ///         color: Colors.cyan,
  ///         elevation: 10.0,
  ///         child: Placeholder(),
  ///       ),
  ///     ),
  ///   ),
  /// )
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [elevation], which describes the elevation of the box defined by
  ///    [thickness] and [rect] relative to the parent of this [SemanticsNode].
  double get thickness => _thickness;
  double _thickness = _kEmptyConfig.thickness;

  /// Provides hint values which override the default hints on supported
  /// platforms.
  SemanticsHintOverrides? get hintOverrides => _hintOverrides;
  SemanticsHintOverrides? _hintOverrides;

  /// The reading direction for [label], [value], [hint], [increasedValue], and
  /// [decreasedValue].
  TextDirection? get textDirection => _textDirection;
  TextDirection? _textDirection = _kEmptyConfig.textDirection;

  /// Determines the position of this node among its siblings in the traversal
  /// sort order.
  ///
  /// This is used to describe the order in which the semantic node should be
  /// traversed by the accessibility services on the platform (e.g. VoiceOver
  /// on iOS and TalkBack on Android).
  SemanticsSortKey? get sortKey => _sortKey;
  SemanticsSortKey? _sortKey;

  /// The currently selected text (or the position of the cursor) within [value]
  /// if this node represents a text field.
  TextSelection? get textSelection => _textSelection;
  TextSelection? _textSelection;

  /// If this node represents a text field, this indicates whether or not it's
  /// a multiline text field.
  bool? get isMultiline => _isMultiline;
  bool? _isMultiline;

  /// The total number of scrollable children that contribute to semantics.
  ///
  /// If the number of children are unknown or unbounded, this value will be
  /// null.
  int? get scrollChildCount => _scrollChildCount;
  int? _scrollChildCount;

  /// The index of the first visible semantic child of a scroll node.
  int? get scrollIndex => _scrollIndex;
  int? _scrollIndex;

  /// Indicates the current scrolling position in logical pixels if the node is
  /// scrollable.
  ///
  /// The properties [scrollExtentMin] and [scrollExtentMax] indicate the valid
  /// in-range values for this property. The value for [scrollPosition] may
  /// (temporarily) be outside that range, e.g. during an overscroll.
  ///
  /// See also:
  ///
  ///  * [ScrollPosition.pixels], from where this value is usually taken.
  double? get scrollPosition => _scrollPosition;
  double? _scrollPosition;

  /// Indicates the maximum in-range value for [scrollPosition] if the node is
  /// scrollable.
  ///
  /// This value may be infinity if the scroll is unbound.
  ///
  /// See also:
  ///
  ///  * [ScrollPosition.maxScrollExtent], from where this value is usually taken.
  double? get scrollExtentMax => _scrollExtentMax;
  double? _scrollExtentMax;

  /// Indicates the minimum in-range value for [scrollPosition] if the node is
  /// scrollable.
  ///
  /// This value may be infinity if the scroll is unbound.
  ///
  /// See also:
  ///
  ///  * [ScrollPosition.minScrollExtent] from where this value is usually taken.
  double? get scrollExtentMin => _scrollExtentMin;
  double? _scrollExtentMin;

  /// The id of the platform view, whose semantics nodes will be added as
  /// children to this node.
  ///
  /// If this value is non-null, the SemanticsNode must not have any children
  /// as those would be replaced by the semantics nodes of the referenced
  /// platform view.
  ///
  /// See also:
  ///
  ///  * [AndroidView], which is the platform view for Android.
  ///  * [UiKitView], which is the platform view for iOS.
  int? get platformViewId => _platformViewId;
  int? _platformViewId;

  /// The maximum number of characters that can be entered into an editable
  /// text field.
  ///
  /// For the purpose of this function a character is defined as one Unicode
  /// scalar value.
  ///
  /// This should only be set when [SemanticsFlag.isTextField] is set. Defaults
  /// to null, which means no limit is imposed on the text field.
  int? get maxValueLength => _maxValueLength;
  int? _maxValueLength;

  /// The current number of characters that have been entered into an editable
  /// text field.
  ///
  /// For the purpose of this function a character is defined as one Unicode
  /// scalar value.
  ///
  /// This should only be set when [SemanticsFlag.isTextField] is set. Must be
  /// set when [maxValueLength] is set.
  int? get currentValueLength => _currentValueLength;
  int? _currentValueLength;

  /// The level of the widget as a heading within the structural hierarchy
  /// of the screen. A value of 1 indicates the highest level of structural
  /// hierarchy. A value of 2 indicates the next level, and so on.
  int get headingLevel => _headingLevel;
  int _headingLevel = _kEmptyConfig._headingLevel;

  /// The URL that this node links to.
  Uri? get linkUrl => _linkUrl;
  Uri? _linkUrl = _kEmptyConfig._linkUrl;

  bool _canPerformAction(SemanticsAction action) => _actions.containsKey(action);

  static final SemanticsConfiguration _kEmptyConfig = SemanticsConfiguration();

  /// Reconfigures the properties of this object to describe the configuration
  /// provided in the `config` argument and the children listed in the
  /// `childrenInInversePaintOrder` argument.
  ///
  /// The arguments may be null; this represents an empty configuration (all
  /// values at their defaults, no children).
  ///
  /// No reference is kept to the [SemanticsConfiguration] object, but the child
  /// list is used as-is and should therefore not be changed after this call.
  void updateWith({
    required SemanticsConfiguration? config,
    List<SemanticsNode>? childrenInInversePaintOrder,
  }) {
    config ??= _kEmptyConfig;
    if (_isDifferentFromCurrentSemanticAnnotation(config)) {
      _markDirty();
    }

    assert(
      config.platformViewId == null ||
          childrenInInversePaintOrder == null ||
          childrenInInversePaintOrder.isEmpty,
      'SemanticsNodes with children must not specify a platformViewId.',
    );

    final bool mergeAllDescendantsIntoThisNodeValueChanged =
        _mergeAllDescendantsIntoThisNode != config.isMergingSemanticsOfDescendants;

    _identifier = config.identifier;
    _attributedLabel = config.attributedLabel;
    _attributedValue = config.attributedValue;
    _attributedIncreasedValue = config.attributedIncreasedValue;
    _attributedDecreasedValue = config.attributedDecreasedValue;
    _attributedHint = config.attributedHint;
    _tooltip = config.tooltip;
    _hintOverrides = config.hintOverrides;
    _elevation = config.elevation;
    _thickness = config.thickness;
    _flags = config._flags;
    _textDirection = config.textDirection;
    _sortKey = config.sortKey;
    _actions = Map<SemanticsAction, SemanticsActionHandler>.of(config._actions);
    _customSemanticsActions = Map<CustomSemanticsAction, VoidCallback>.of(
      config._customSemanticsActions,
    );
    _actionsAsBits = config._actionsAsBits;
    _textSelection = config._textSelection;
    _isMultiline = config.isMultiline;
    _scrollPosition = config._scrollPosition;
    _scrollExtentMax = config._scrollExtentMax;
    _scrollExtentMin = config._scrollExtentMin;
    _mergeAllDescendantsIntoThisNode = config.isMergingSemanticsOfDescendants;
    _scrollChildCount = config.scrollChildCount;
    _scrollIndex = config.scrollIndex;
    indexInParent = config.indexInParent;
    _platformViewId = config._platformViewId;
    _maxValueLength = config._maxValueLength;
    _currentValueLength = config._currentValueLength;
    _areUserActionsBlocked = config.isBlockingUserActions;
    _headingLevel = config._headingLevel;
    _linkUrl = config._linkUrl;
    _replaceChildren(childrenInInversePaintOrder ?? const <SemanticsNode>[]);

    if (mergeAllDescendantsIntoThisNodeValueChanged) {
      _updateChildrenMergeFlags();
    }

    assert(
      !_canPerformAction(SemanticsAction.increase) || (value == '') == (increasedValue == ''),
      'A SemanticsNode with action "increase" needs to be annotated with either both "value" and "increasedValue" or neither',
    );
    assert(
      !_canPerformAction(SemanticsAction.decrease) || (value == '') == (decreasedValue == ''),
      'A SemanticsNode with action "decrease" needs to be annotated with either both "value" and "decreasedValue" or neither',
    );
  }

  /// Returns a summary of the semantics for this node.
  ///
  /// If this node has [mergeAllDescendantsIntoThisNode], then the returned data
  /// includes the information from this node's descendants. Otherwise, the
  /// returned data matches the data on this node.
  SemanticsData getSemanticsData() {
    int flags = _flags;
    // Can't use _effectiveActionsAsBits here. The filtering of action bits
    // must be done after the merging the its descendants.
    int actions = _actionsAsBits;
    String identifier = _identifier;
    AttributedString attributedLabel = _attributedLabel;
    AttributedString attributedValue = _attributedValue;
    AttributedString attributedIncreasedValue = _attributedIncreasedValue;
    AttributedString attributedDecreasedValue = _attributedDecreasedValue;
    AttributedString attributedHint = _attributedHint;
    String tooltip = _tooltip;
    TextDirection? textDirection = _textDirection;
    Set<SemanticsTag>? mergedTags = tags == null ? null : Set<SemanticsTag>.of(tags!);
    TextSelection? textSelection = _textSelection;
    int? scrollChildCount = _scrollChildCount;
    int? scrollIndex = _scrollIndex;
    double? scrollPosition = _scrollPosition;
    double? scrollExtentMax = _scrollExtentMax;
    double? scrollExtentMin = _scrollExtentMin;
    int? platformViewId = _platformViewId;
    int? maxValueLength = _maxValueLength;
    int? currentValueLength = _currentValueLength;
    int headingLevel = _headingLevel;
    final double elevation = _elevation;
    double thickness = _thickness;
    Uri? linkUrl = _linkUrl;
    final Set<int> customSemanticsActionIds = <int>{};
    for (final CustomSemanticsAction action in _customSemanticsActions.keys) {
      customSemanticsActionIds.add(CustomSemanticsAction.getIdentifier(action));
    }
    if (hintOverrides != null) {
      if (hintOverrides!.onTapHint != null) {
        final CustomSemanticsAction action = CustomSemanticsAction.overridingAction(
          hint: hintOverrides!.onTapHint!,
          action: SemanticsAction.tap,
        );
        customSemanticsActionIds.add(CustomSemanticsAction.getIdentifier(action));
      }
      if (hintOverrides!.onLongPressHint != null) {
        final CustomSemanticsAction action = CustomSemanticsAction.overridingAction(
          hint: hintOverrides!.onLongPressHint!,
          action: SemanticsAction.longPress,
        );
        customSemanticsActionIds.add(CustomSemanticsAction.getIdentifier(action));
      }
    }

    if (mergeAllDescendantsIntoThisNode) {
      _visitDescendants((SemanticsNode node) {
        assert(node.isMergedIntoParent);
        flags |= node._flags;
        actions |= node._effectiveActionsAsBits;

        textDirection ??= node._textDirection;
        textSelection ??= node._textSelection;
        scrollChildCount ??= node._scrollChildCount;
        scrollIndex ??= node._scrollIndex;
        scrollPosition ??= node._scrollPosition;
        scrollExtentMax ??= node._scrollExtentMax;
        scrollExtentMin ??= node._scrollExtentMin;
        platformViewId ??= node._platformViewId;
        maxValueLength ??= node._maxValueLength;
        currentValueLength ??= node._currentValueLength;
        linkUrl ??= node._linkUrl;
        headingLevel = _mergeHeadingLevels(
          sourceLevel: node._headingLevel,
          targetLevel: headingLevel,
        );

        if (identifier == '') {
          identifier = node._identifier;
        }
        if (attributedValue.string == '') {
          attributedValue = node._attributedValue;
        }
        if (attributedIncreasedValue.string == '') {
          attributedIncreasedValue = node._attributedIncreasedValue;
        }
        if (attributedDecreasedValue.string == '') {
          attributedDecreasedValue = node._attributedDecreasedValue;
        }
        if (tooltip == '') {
          tooltip = node._tooltip;
        }
        if (node.tags != null) {
          mergedTags ??= <SemanticsTag>{};
          mergedTags!.addAll(node.tags!);
        }
        for (final CustomSemanticsAction action in _customSemanticsActions.keys) {
          customSemanticsActionIds.add(CustomSemanticsAction.getIdentifier(action));
        }
        if (node.hintOverrides != null) {
          if (node.hintOverrides!.onTapHint != null) {
            final CustomSemanticsAction action = CustomSemanticsAction.overridingAction(
              hint: node.hintOverrides!.onTapHint!,
              action: SemanticsAction.tap,
            );
            customSemanticsActionIds.add(CustomSemanticsAction.getIdentifier(action));
          }
          if (node.hintOverrides!.onLongPressHint != null) {
            final CustomSemanticsAction action = CustomSemanticsAction.overridingAction(
              hint: node.hintOverrides!.onLongPressHint!,
              action: SemanticsAction.longPress,
            );
            customSemanticsActionIds.add(CustomSemanticsAction.getIdentifier(action));
          }
        }
        attributedLabel = _concatAttributedString(
          thisAttributedString: attributedLabel,
          thisTextDirection: textDirection,
          otherAttributedString: node._attributedLabel,
          otherTextDirection: node._textDirection,
        );
        attributedHint = _concatAttributedString(
          thisAttributedString: attributedHint,
          thisTextDirection: textDirection,
          otherAttributedString: node._attributedHint,
          otherTextDirection: node._textDirection,
        );

        thickness = math.max(thickness, node._thickness + node._elevation);

        return true;
      });
    }

    return SemanticsData(
      flags: flags,
      actions: _areUserActionsBlocked ? actions & _kUnblockedUserActions : actions,
      identifier: identifier,
      attributedLabel: attributedLabel,
      attributedValue: attributedValue,
      attributedIncreasedValue: attributedIncreasedValue,
      attributedDecreasedValue: attributedDecreasedValue,
      attributedHint: attributedHint,
      tooltip: tooltip,
      textDirection: textDirection,
      rect: rect,
      transform: transform,
      elevation: elevation,
      thickness: thickness,
      tags: mergedTags,
      textSelection: textSelection,
      scrollChildCount: scrollChildCount,
      scrollIndex: scrollIndex,
      scrollPosition: scrollPosition,
      scrollExtentMax: scrollExtentMax,
      scrollExtentMin: scrollExtentMin,
      platformViewId: platformViewId,
      maxValueLength: maxValueLength,
      currentValueLength: currentValueLength,
      customSemanticsActionIds: customSemanticsActionIds.toList()..sort(),
      headingLevel: headingLevel,
      linkUrl: linkUrl,
    );
  }

  static Float64List _initIdentityTransform() {
    return Matrix4.identity().storage;
  }

  static final Int32List _kEmptyChildList = Int32List(0);
  static final Int32List _kEmptyCustomSemanticsActionsList = Int32List(0);
  static final Float64List _kIdentityTransform = _initIdentityTransform();

  void _addToUpdate(SemanticsUpdateBuilder builder, Set<int> customSemanticsActionIdsUpdate) {
    assert(_dirty);
    final SemanticsData data = getSemanticsData();
    final Int32List childrenInTraversalOrder;
    final Int32List childrenInHitTestOrder;
    if (!hasChildren || mergeAllDescendantsIntoThisNode) {
      childrenInTraversalOrder = _kEmptyChildList;
      childrenInHitTestOrder = _kEmptyChildList;
    } else {
      final int childCount = _children!.length;
      final List<SemanticsNode> sortedChildren = _childrenInTraversalOrder();
      childrenInTraversalOrder = Int32List(childCount);
      for (int i = 0; i < childCount; i += 1) {
        childrenInTraversalOrder[i] = sortedChildren[i].id;
      }
      // _children is sorted in paint order, so we invert it to get the hit test
      // order.
      childrenInHitTestOrder = Int32List(childCount);
      for (int i = childCount - 1; i >= 0; i -= 1) {
        childrenInHitTestOrder[i] = _children![childCount - i - 1].id;
      }
    }
    Int32List? customSemanticsActionIds;
    if (data.customSemanticsActionIds?.isNotEmpty ?? false) {
      customSemanticsActionIds = Int32List(data.customSemanticsActionIds!.length);
      for (int i = 0; i < data.customSemanticsActionIds!.length; i++) {
        customSemanticsActionIds[i] = data.customSemanticsActionIds![i];
        customSemanticsActionIdsUpdate.add(data.customSemanticsActionIds![i]);
      }
    }
    builder.updateNode(
      id: id,
      flags: data.flags,
      actions: data.actions,
      rect: data.rect,
      identifier: data.identifier,
      label: data.attributedLabel.string,
      labelAttributes: data.attributedLabel.attributes,
      value: data.attributedValue.string,
      valueAttributes: data.attributedValue.attributes,
      increasedValue: data.attributedIncreasedValue.string,
      increasedValueAttributes: data.attributedIncreasedValue.attributes,
      decreasedValue: data.attributedDecreasedValue.string,
      decreasedValueAttributes: data.attributedDecreasedValue.attributes,
      hint: data.attributedHint.string,
      hintAttributes: data.attributedHint.attributes,
      tooltip: data.tooltip,
      textDirection: data.textDirection,
      textSelectionBase: data.textSelection != null ? data.textSelection!.baseOffset : -1,
      textSelectionExtent: data.textSelection != null ? data.textSelection!.extentOffset : -1,
      platformViewId: data.platformViewId ?? -1,
      maxValueLength: data.maxValueLength ?? -1,
      currentValueLength: data.currentValueLength ?? -1,
      scrollChildren: data.scrollChildCount ?? 0,
      scrollIndex: data.scrollIndex ?? 0,
      scrollPosition: data.scrollPosition ?? double.nan,
      scrollExtentMax: data.scrollExtentMax ?? double.nan,
      scrollExtentMin: data.scrollExtentMin ?? double.nan,
      transform: data.transform?.storage ?? _kIdentityTransform,
      elevation: data.elevation,
      thickness: data.thickness,
      childrenInTraversalOrder: childrenInTraversalOrder,
      childrenInHitTestOrder: childrenInHitTestOrder,
      additionalActions: customSemanticsActionIds ?? _kEmptyCustomSemanticsActionsList,
      headingLevel: data.headingLevel,
      linkUrl: data.linkUrl?.toString() ?? '',
    );
    _dirty = false;
  }

  /// Builds a new list made of [_children] sorted in semantic traversal order.
  List<SemanticsNode> _childrenInTraversalOrder() {
    TextDirection? inheritedTextDirection = textDirection;
    SemanticsNode? ancestor = parent;
    while (inheritedTextDirection == null && ancestor != null) {
      inheritedTextDirection = ancestor.textDirection;
      ancestor = ancestor.parent;
    }

    List<SemanticsNode>? childrenInDefaultOrder;
    if (inheritedTextDirection != null) {
      childrenInDefaultOrder = _childrenInDefaultOrder(_children!, inheritedTextDirection);
    } else {
      // In the absence of text direction default to paint order.
      childrenInDefaultOrder = _children;
    }

    // List.sort does not guarantee stable sort order. Therefore, children are
    // first partitioned into groups that have compatible sort keys, i.e. keys
    // in the same group can be compared to each other. These groups stay in
    // the same place. Only children within the same group are sorted.
    final List<_TraversalSortNode> everythingSorted = <_TraversalSortNode>[];
    final List<_TraversalSortNode> sortNodes = <_TraversalSortNode>[];
    SemanticsSortKey? lastSortKey;
    for (int position = 0; position < childrenInDefaultOrder!.length; position += 1) {
      final SemanticsNode child = childrenInDefaultOrder[position];
      final SemanticsSortKey? sortKey = child.sortKey;
      lastSortKey = position > 0 ? childrenInDefaultOrder[position - 1].sortKey : null;
      final bool isCompatibleWithPreviousSortKey =
          position == 0 ||
          sortKey.runtimeType == lastSortKey.runtimeType &&
              (sortKey == null || sortKey.name == lastSortKey!.name);
      if (!isCompatibleWithPreviousSortKey && sortNodes.isNotEmpty) {
        // Do not sort groups with null sort keys. List.sort does not guarantee
        // a stable sort order.
        if (lastSortKey != null) {
          sortNodes.sort();
        }
        everythingSorted.addAll(sortNodes);
        sortNodes.clear();
      }

      sortNodes.add(_TraversalSortNode(node: child, sortKey: sortKey, position: position));
    }

    // Do not sort groups with null sort keys. List.sort does not guarantee
    // a stable sort order.
    if (lastSortKey != null) {
      sortNodes.sort();
    }
    everythingSorted.addAll(sortNodes);

    return everythingSorted
        .map<SemanticsNode>((_TraversalSortNode sortNode) => sortNode.node)
        .toList();
  }

  /// Sends a [SemanticsEvent] associated with this [SemanticsNode].
  ///
  /// Semantics events should be sent to inform interested parties (like
  /// the accessibility system of the operating system) about changes to the UI.
  void sendEvent(SemanticsEvent event) {
    if (!attached) {
      return;
    }
    SystemChannels.accessibility.send(event.toMap(nodeId: id));
  }

  bool _debugIsActionBlocked(SemanticsAction action) {
    bool result = false;
    assert(() {
      result = (_effectiveActionsAsBits & action.index) == 0;
      return true;
    }());
    return result;
  }

  @override
  String toStringShort() => '${objectRuntimeType(this, 'SemanticsNode')}#$id';

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    bool hideOwner = true;
    if (_dirty) {
      final bool inDirtyNodes = owner != null && owner!._dirtyNodes.contains(this);
      properties.add(
        FlagProperty('inDirtyNodes', value: inDirtyNodes, ifTrue: 'dirty', ifFalse: 'STALE'),
      );
      hideOwner = inDirtyNodes;
    }
    properties.add(
      DiagnosticsProperty<SemanticsOwner>(
        'owner',
        owner,
        level: hideOwner ? DiagnosticLevel.hidden : DiagnosticLevel.info,
      ),
    );
    properties.add(
      FlagProperty('isMergedIntoParent', value: isMergedIntoParent, ifTrue: 'merged up ⬆️'),
    );
    properties.add(
      FlagProperty(
        'mergeAllDescendantsIntoThisNode',
        value: mergeAllDescendantsIntoThisNode,
        ifTrue: 'merge boundary ⛔️',
      ),
    );
    final Offset? offset = transform != null ? MatrixUtils.getAsTranslation(transform!) : null;
    if (offset != null) {
      properties.add(DiagnosticsProperty<Rect>('rect', rect.shift(offset), showName: false));
    } else {
      final double? scale = transform != null ? MatrixUtils.getAsScale(transform!) : null;
      String? description;
      if (scale != null) {
        description = '$rect scaled by ${scale.toStringAsFixed(1)}x';
      } else if (transform != null && !MatrixUtils.isIdentity(transform!)) {
        final String matrix = transform
            .toString()
            .split('\n')
            .take(4)
            .map<String>((String line) => line.substring(4))
            .join('; ');
        description = '$rect with transform [$matrix]';
      }
      properties.add(
        DiagnosticsProperty<Rect>('rect', rect, description: description, showName: false),
      );
    }
    properties.add(
      IterableProperty<String>(
        'tags',
        tags?.map((SemanticsTag tag) => tag.name),
        defaultValue: null,
      ),
    );
    final List<String> actions =
        _actions.keys
            .map<String>(
              (SemanticsAction action) =>
                  '${action.name}${_debugIsActionBlocked(action) ? '🚫️' : ''}',
            )
            .toList()
          ..sort();
    final List<String?> customSemanticsActions =
        _customSemanticsActions.keys
            .map<String?>((CustomSemanticsAction action) => action.label)
            .toList();
    properties.add(IterableProperty<String>('actions', actions, ifEmpty: null));
    properties.add(
      IterableProperty<String?>('customActions', customSemanticsActions, ifEmpty: null),
    );
    final List<String> flags =
        SemanticsFlag.values
            .where((SemanticsFlag flag) => hasFlag(flag))
            .map((SemanticsFlag flag) => flag.name)
            .toList();
    properties.add(IterableProperty<String>('flags', flags, ifEmpty: null));
    properties.add(FlagProperty('isInvisible', value: isInvisible, ifTrue: 'invisible'));
    properties.add(
      FlagProperty('isHidden', value: hasFlag(SemanticsFlag.isHidden), ifTrue: 'HIDDEN'),
    );
    properties.add(StringProperty('identifier', _identifier, defaultValue: ''));
    properties.add(AttributedStringProperty('label', _attributedLabel));
    properties.add(AttributedStringProperty('value', _attributedValue));
    properties.add(AttributedStringProperty('increasedValue', _attributedIncreasedValue));
    properties.add(AttributedStringProperty('decreasedValue', _attributedDecreasedValue));
    properties.add(AttributedStringProperty('hint', _attributedHint));
    properties.add(StringProperty('tooltip', _tooltip, defaultValue: ''));
    properties.add(
      EnumProperty<TextDirection>('textDirection', _textDirection, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<SemanticsSortKey>('sortKey', sortKey, defaultValue: null));
    if (_textSelection?.isValid ?? false) {
      properties.add(
        MessageProperty('text selection', '[${_textSelection!.start}, ${_textSelection!.end}]'),
      );
    }
    properties.add(IntProperty('platformViewId', platformViewId, defaultValue: null));
    properties.add(IntProperty('maxValueLength', maxValueLength, defaultValue: null));
    properties.add(IntProperty('currentValueLength', currentValueLength, defaultValue: null));
    properties.add(IntProperty('scrollChildren', scrollChildCount, defaultValue: null));
    properties.add(IntProperty('scrollIndex', scrollIndex, defaultValue: null));
    properties.add(DoubleProperty('scrollExtentMin', scrollExtentMin, defaultValue: null));
    properties.add(DoubleProperty('scrollPosition', scrollPosition, defaultValue: null));
    properties.add(DoubleProperty('scrollExtentMax', scrollExtentMax, defaultValue: null));
    properties.add(IntProperty('indexInParent', indexInParent, defaultValue: null));
    properties.add(DoubleProperty('elevation', elevation, defaultValue: 0.0));
    properties.add(DoubleProperty('thickness', thickness, defaultValue: 0.0));
    properties.add(IntProperty('headingLevel', _headingLevel, defaultValue: 0));
  }

  /// Returns a string representation of this node and its descendants.
  ///
  /// The order in which the children of the [SemanticsNode] will be printed is
  /// controlled by the [childOrder] parameter.
  @override
  String toStringDeep({
    String prefixLineOne = '',
    String? prefixOtherLines,
    DiagnosticLevel minLevel = DiagnosticLevel.debug,
    DebugSemanticsDumpOrder childOrder = DebugSemanticsDumpOrder.traversalOrder,
    int wrapWidth = 65,
  }) {
    return toDiagnosticsNode(childOrder: childOrder).toStringDeep(
      prefixLineOne: prefixLineOne,
      prefixOtherLines: prefixOtherLines,
      minLevel: minLevel,
      wrapWidth: wrapWidth,
    );
  }

  @override
  DiagnosticsNode toDiagnosticsNode({
    String? name,
    DiagnosticsTreeStyle? style = DiagnosticsTreeStyle.sparse,
    DebugSemanticsDumpOrder childOrder = DebugSemanticsDumpOrder.traversalOrder,
  }) {
    return _SemanticsDiagnosticableNode(
      name: name,
      value: this,
      style: style,
      childOrder: childOrder,
    );
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren({
    DebugSemanticsDumpOrder childOrder = DebugSemanticsDumpOrder.inverseHitTest,
  }) {
    return debugListChildrenInOrder(childOrder)
        .map<DiagnosticsNode>(
          (SemanticsNode node) => node.toDiagnosticsNode(childOrder: childOrder),
        )
        .toList();
  }

  /// Returns the list of direct children of this node in the specified order.
  List<SemanticsNode> debugListChildrenInOrder(DebugSemanticsDumpOrder childOrder) {
    if (_children == null) {
      return const <SemanticsNode>[];
    }

    return switch (childOrder) {
      DebugSemanticsDumpOrder.inverseHitTest => _children!,
      DebugSemanticsDumpOrder.traversalOrder => _childrenInTraversalOrder(),
    };
  }
}

/// An edge of a box, such as top, bottom, left or right, used to compute
/// [SemanticsNode]s that overlap vertically or horizontally.
///
/// For computing horizontal overlap in an LTR setting we create two [_BoxEdge]
/// objects for each [SemanticsNode]: one representing the left edge (marked
/// with [isLeadingEdge] equal to true) and one for the right edge (with [isLeadingEdge]
/// equal to false). Similarly, for vertical overlap we also create two objects
/// for each [SemanticsNode], one for the top and one for the bottom edge.
class _BoxEdge implements Comparable<_BoxEdge> {
  _BoxEdge({required this.isLeadingEdge, required this.offset, required this.node})
    : assert(offset.isFinite);

  /// True if the edge comes before the seconds edge along the traversal
  /// direction, and false otherwise.
  ///
  /// This field is never null.
  ///
  /// For example, in LTR traversal the left edge's [isLeadingEdge] is set to true,
  /// the right edge's [isLeadingEdge] is set to false. When considering vertical
  /// ordering of boxes, the top edge is the start edge, and the bottom edge is
  /// the end edge.
  final bool isLeadingEdge;

  /// The offset from the start edge of the parent [SemanticsNode] in the
  /// direction of the traversal.
  final double offset;

  /// The node whom this edge belongs.
  final SemanticsNode node;

  @override
  int compareTo(_BoxEdge other) {
    return offset.compareTo(other.offset);
  }
}

/// A group of [nodes] that are disjoint vertically or horizontally from other
/// nodes that share the same [SemanticsNode] parent.
///
/// The [nodes] are sorted among each other separately from other nodes.
class _SemanticsSortGroup implements Comparable<_SemanticsSortGroup> {
  _SemanticsSortGroup({required this.startOffset, required this.textDirection});

  /// The offset from the start edge of the parent [SemanticsNode] in the
  /// direction of the traversal.
  ///
  /// This value is equal to the [_BoxEdge.offset] of the first node in the
  /// [nodes] list being considered.
  final double startOffset;

  final TextDirection textDirection;

  /// The nodes that are sorted among each other.
  final List<SemanticsNode> nodes = <SemanticsNode>[];

  @override
  int compareTo(_SemanticsSortGroup other) {
    return startOffset.compareTo(other.startOffset);
  }

  /// Sorts this group assuming that [nodes] belong to the same vertical group.
  ///
  /// This method breaks up this group into horizontal [_SemanticsSortGroup]s
  /// then sorts them using [sortedWithinKnot].
  List<SemanticsNode> sortedWithinVerticalGroup() {
    final List<_BoxEdge> edges = <_BoxEdge>[];
    for (final SemanticsNode child in nodes) {
      // Using a small delta to shrink child rects removes overlapping cases.
      final Rect childRect = child.rect.deflate(0.1);
      edges.add(
        _BoxEdge(
          isLeadingEdge: true,
          offset: _pointInParentCoordinates(child, childRect.topLeft).dx,
          node: child,
        ),
      );
      edges.add(
        _BoxEdge(
          isLeadingEdge: false,
          offset: _pointInParentCoordinates(child, childRect.bottomRight).dx,
          node: child,
        ),
      );
    }
    edges.sort();

    List<_SemanticsSortGroup> horizontalGroups = <_SemanticsSortGroup>[];
    _SemanticsSortGroup? group;
    int depth = 0;
    for (final _BoxEdge edge in edges) {
      if (edge.isLeadingEdge) {
        depth += 1;
        group ??= _SemanticsSortGroup(startOffset: edge.offset, textDirection: textDirection);
        group.nodes.add(edge.node);
      } else {
        depth -= 1;
      }
      if (depth == 0) {
        horizontalGroups.add(group!);
        group = null;
      }
    }
    horizontalGroups.sort();

    if (textDirection == TextDirection.rtl) {
      horizontalGroups = horizontalGroups.reversed.toList();
    }

    return horizontalGroups
        .expand((_SemanticsSortGroup group) => group.sortedWithinKnot())
        .toList();
  }

  /// Sorts [nodes] where nodes intersect both vertically and horizontally.
  ///
  /// In the special case when [nodes] contains one or less nodes, this method
  /// returns [nodes] unchanged.
  ///
  /// This method constructs a graph, where vertices are [SemanticsNode]s and
  /// edges are "traversed before" relation between pairs of nodes. The sort
  /// order is the topological sorting of the graph, with the original order of
  /// [nodes] used as the tie breaker.
  ///
  /// Whether a node is traversed before another node is determined by the
  /// vector that connects the two nodes' centers. If the vector "points to the
  /// right or down", defined as the [Offset.direction] being between `-pi/4`
  /// and `3*pi/4`), then the semantics node whose center is at the end of the
  /// vector is said to be traversed after.
  List<SemanticsNode> sortedWithinKnot() {
    if (nodes.length <= 1) {
      // Trivial knot. Nothing to do.
      return nodes;
    }
    final Map<int, SemanticsNode> nodeMap = <int, SemanticsNode>{};
    final Map<int, int> edges = <int, int>{};
    for (final SemanticsNode node in nodes) {
      nodeMap[node.id] = node;
      final Offset center = _pointInParentCoordinates(node, node.rect.center);
      for (final SemanticsNode nextNode in nodes) {
        if (identical(node, nextNode) || edges[nextNode.id] == node.id) {
          // Skip self or when we've already established that the next node
          // points to current node.
          continue;
        }

        final Offset nextCenter = _pointInParentCoordinates(nextNode, nextNode.rect.center);
        final Offset centerDelta = nextCenter - center;
        // When centers coincide, direction is 0.0.
        final double direction = centerDelta.direction;
        final bool isLtrAndForward =
            textDirection == TextDirection.ltr &&
            -math.pi / 4 < direction &&
            direction < 3 * math.pi / 4;
        final bool isRtlAndForward =
            textDirection == TextDirection.rtl &&
            (direction < -3 * math.pi / 4 || direction > 3 * math.pi / 4);
        if (isLtrAndForward || isRtlAndForward) {
          edges[node.id] = nextNode.id;
        }
      }
    }

    final List<int> sortedIds = <int>[];
    final Set<int> visitedIds = <int>{};
    final List<SemanticsNode> startNodes =
        nodes.toList()..sort((SemanticsNode a, SemanticsNode b) {
          final Offset aTopLeft = _pointInParentCoordinates(a, a.rect.topLeft);
          final Offset bTopLeft = _pointInParentCoordinates(b, b.rect.topLeft);
          final int verticalDiff = aTopLeft.dy.compareTo(bTopLeft.dy);
          if (verticalDiff != 0) {
            return -verticalDiff;
          }
          return -aTopLeft.dx.compareTo(bTopLeft.dx);
        });

    void search(int id) {
      if (visitedIds.contains(id)) {
        return;
      }
      visitedIds.add(id);
      if (edges.containsKey(id)) {
        search(edges[id]!);
      }
      sortedIds.add(id);
    }

    startNodes.map<int>((SemanticsNode node) => node.id).forEach(search);
    return sortedIds.map<SemanticsNode>((int id) => nodeMap[id]!).toList().reversed.toList();
  }
}

/// Converts `point` to the `node`'s parent's coordinate system.
Offset _pointInParentCoordinates(SemanticsNode node, Offset point) {
  if (node.transform == null) {
    return point;
  }
  final Vector3 vector = Vector3(point.dx, point.dy, 0.0);
  node.transform!.transform3(vector);
  return Offset(vector.x, vector.y);
}

/// Sorts `children` using the default sorting algorithm, and returns them as a
/// new list.
///
/// The algorithm first breaks up children into groups such that no two nodes
/// from different groups overlap vertically. These groups are sorted vertically
/// according to their [_SemanticsSortGroup.startOffset].
///
/// Within each group, the nodes are sorted using
/// [_SemanticsSortGroup.sortedWithinVerticalGroup].
///
/// For an illustration of the algorithm see http://bit.ly/flutter-default-traversal.
List<SemanticsNode> _childrenInDefaultOrder(
  List<SemanticsNode> children,
  TextDirection textDirection,
) {
  final List<_BoxEdge> edges = <_BoxEdge>[];
  for (final SemanticsNode child in children) {
    assert(child.rect.isFinite);
    // Using a small delta to shrink child rects removes overlapping cases.
    final Rect childRect = child.rect.deflate(0.1);
    edges.add(
      _BoxEdge(
        isLeadingEdge: true,
        offset: _pointInParentCoordinates(child, childRect.topLeft).dy,
        node: child,
      ),
    );
    edges.add(
      _BoxEdge(
        isLeadingEdge: false,
        offset: _pointInParentCoordinates(child, childRect.bottomRight).dy,
        node: child,
      ),
    );
  }
  edges.sort();

  final List<_SemanticsSortGroup> verticalGroups = <_SemanticsSortGroup>[];
  _SemanticsSortGroup? group;
  int depth = 0;
  for (final _BoxEdge edge in edges) {
    if (edge.isLeadingEdge) {
      depth += 1;
      group ??= _SemanticsSortGroup(startOffset: edge.offset, textDirection: textDirection);
      group.nodes.add(edge.node);
    } else {
      depth -= 1;
    }
    if (depth == 0) {
      verticalGroups.add(group!);
      group = null;
    }
  }
  verticalGroups.sort();

  return verticalGroups
      .expand((_SemanticsSortGroup group) => group.sortedWithinVerticalGroup())
      .toList();
}

/// The implementation of [Comparable] that implements the ordering of
/// [SemanticsNode]s in the accessibility traversal.
///
/// [SemanticsNode]s are sorted prior to sending them to the engine side.
///
/// This implementation considers a [node]'s [sortKey] and its position within
/// the list of its siblings. [sortKey] takes precedence over position.
class _TraversalSortNode implements Comparable<_TraversalSortNode> {
  _TraversalSortNode({required this.node, this.sortKey, required this.position});

  /// The node whose position this sort node determines.
  final SemanticsNode node;

  /// Determines the position of this node among its siblings.
  ///
  /// Sort keys take precedence over other attributes, such as
  /// [position].
  final SemanticsSortKey? sortKey;

  /// Position within the list of siblings as determined by the default sort
  /// order.
  final int position;

  @override
  int compareTo(_TraversalSortNode other) {
    if (sortKey == null || other.sortKey == null) {
      return position - other.position;
    }
    return sortKey!.compareTo(other.sortKey!);
  }
}

/// Owns [SemanticsNode] objects and notifies listeners of changes to the
/// render tree semantics.
///
/// To listen for semantic updates, call [SemanticsBinding.ensureSemantics] or
/// [PipelineOwner.ensureSemantics] to obtain a [SemanticsHandle]. This will
/// create a [SemanticsOwner] if necessary.
class SemanticsOwner extends ChangeNotifier {
  /// Creates a [SemanticsOwner] that manages zero or more [SemanticsNode] objects.
  SemanticsOwner({required this.onSemanticsUpdate}) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/semantics.dart',
        className: '$SemanticsOwner',
        object: this,
      );
    }
  }

  /// The [onSemanticsUpdate] callback is expected to dispatch [SemanticsUpdate]s
  /// to the [FlutterView] that is associated with this [PipelineOwner] and/or
  /// [SemanticsOwner].
  ///
  /// A [SemanticsOwner] calls [onSemanticsUpdate] during [sendSemanticsUpdate]
  /// after the [SemanticsUpdate] has been build, but before the [SemanticsOwner]'s
  /// listeners have been notified.
  final SemanticsUpdateCallback onSemanticsUpdate;
  final Set<SemanticsNode> _dirtyNodes = <SemanticsNode>{};
  final Map<int, SemanticsNode> _nodes = <int, SemanticsNode>{};
  final Set<SemanticsNode> _detachedNodes = <SemanticsNode>{};

  /// The root node of the semantics tree, if any.
  ///
  /// If the semantics tree is empty, returns null.
  SemanticsNode? get rootSemanticsNode => _nodes[0];

  @override
  void dispose() {
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    _dirtyNodes.clear();
    _nodes.clear();
    _detachedNodes.clear();
    super.dispose();
  }

  /// Update the semantics using [onSemanticsUpdate].
  void sendSemanticsUpdate() {
    // Once the tree is up-to-date, verify that every node is visible.
    assert(() {
      final List<SemanticsNode> invisibleNodes = <SemanticsNode>[];
      // Finds the invisible nodes in the tree rooted at `node` and adds them to
      // the invisibleNodes list. If a node is itself invisible, all its
      // descendants will be skipped.
      bool findInvisibleNodes(SemanticsNode node) {
        if (node.rect.isEmpty) {
          invisibleNodes.add(node);
        } else if (!node.mergeAllDescendantsIntoThisNode) {
          node.visitChildren(findInvisibleNodes);
        }
        return true;
      }

      final SemanticsNode? rootSemanticsNode = this.rootSemanticsNode;
      if (rootSemanticsNode != null) {
        // The root node is allowed to be invisible when it has no children.
        if (rootSemanticsNode.childrenCount > 0 && rootSemanticsNode.rect.isEmpty) {
          invisibleNodes.add(rootSemanticsNode);
        } else if (!rootSemanticsNode.mergeAllDescendantsIntoThisNode) {
          rootSemanticsNode.visitChildren(findInvisibleNodes);
        }
      }

      if (invisibleNodes.isEmpty) {
        return true;
      }

      List<DiagnosticsNode> nodeToMessage(SemanticsNode invisibleNode) {
        final SemanticsNode? parent = invisibleNode.parent;
        return <DiagnosticsNode>[
          invisibleNode.toDiagnosticsNode(style: DiagnosticsTreeStyle.errorProperty),
          parent?.toDiagnosticsNode(
                name: 'which was added as a child of',
                style: DiagnosticsTreeStyle.errorProperty,
              ) ??
              ErrorDescription('which was added as the root SemanticsNode'),
        ];
      }

      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('Invisible SemanticsNodes should not be added to the tree.'),
        ErrorDescription('The following invisible SemanticsNodes were added to the tree:'),
        ...invisibleNodes.expand(nodeToMessage),
        ErrorHint(
          'An invisible SemanticsNode is one whose rect is not on screen hence not reachable for users, '
          'and its semantic information is not merged into a visible parent.',
        ),
        ErrorHint(
          'An invisible SemanticsNode makes the accessibility experience confusing, '
          'as it does not provide any visual indication when the user selects it '
          'via accessibility technologies.',
        ),
        ErrorHint(
          'Consider removing the above invisible SemanticsNodes if they were added by your '
          'RenderObject.assembleSemanticsNode implementation, or filing a bug on GitHub:\n'
          '  https://github.com/flutter/flutter/issues/new?template=2_bug.yml',
        ),
      ]);
    }());

    if (_dirtyNodes.isEmpty) {
      return;
    }
    final Set<int> customSemanticsActionIds = <int>{};
    final List<SemanticsNode> visitedNodes = <SemanticsNode>[];
    while (_dirtyNodes.isNotEmpty) {
      final List<SemanticsNode> localDirtyNodes =
          _dirtyNodes.where((SemanticsNode node) => !_detachedNodes.contains(node)).toList();
      _dirtyNodes.clear();
      _detachedNodes.clear();
      localDirtyNodes.sort((SemanticsNode a, SemanticsNode b) => a.depth - b.depth);
      visitedNodes.addAll(localDirtyNodes);
      for (final SemanticsNode node in localDirtyNodes) {
        assert(node._dirty);
        assert(node.parent == null || !node.parent!.isPartOfNodeMerging || node.isMergedIntoParent);
        if (node.isPartOfNodeMerging) {
          assert(node.mergeAllDescendantsIntoThisNode || node.parent != null);
          // If child node is merged into its parent, make sure the parent is marked as dirty
          if (node.parent != null && node.parent!.isPartOfNodeMerging) {
            node.parent!._markDirty(); // this can add the node to the dirty list
            node._dirty = false; // Do not send update for this node, as it's now part of its parent
          }
        }
      }
    }
    visitedNodes.sort((SemanticsNode a, SemanticsNode b) => a.depth - b.depth);
    final SemanticsUpdateBuilder builder = SemanticsBinding.instance.createSemanticsUpdateBuilder();
    for (final SemanticsNode node in visitedNodes) {
      assert(node.parent?._dirty != true); // could be null (no parent) or false (not dirty)
      // The _serialize() method marks the node as not dirty, and
      // recurses through the tree to do a deep serialization of all
      // contiguous dirty nodes. This means that when we return here,
      // it's quite possible that subsequent nodes are no longer
      // dirty. We skip these here.
      // We also skip any nodes that were reset and subsequently
      // dropped entirely (RenderObject.markNeedsSemanticsUpdate()
      // calls reset() on its SemanticsNode if onlyChanges isn't set,
      // which happens e.g. when the node is no longer contributing
      // semantics).
      if (node._dirty && node.attached) {
        node._addToUpdate(builder, customSemanticsActionIds);
      }
    }
    _dirtyNodes.clear();
    for (final int actionId in customSemanticsActionIds) {
      final CustomSemanticsAction action = CustomSemanticsAction.getAction(actionId)!;
      builder.updateCustomAction(
        id: actionId,
        label: action.label,
        hint: action.hint,
        overrideId: action.action?.index ?? -1,
      );
    }
    onSemanticsUpdate(builder.build());
    notifyListeners();
  }

  SemanticsActionHandler? _getSemanticsActionHandlerForId(int id, SemanticsAction action) {
    SemanticsNode? result = _nodes[id];
    if (result != null && result.isPartOfNodeMerging && !result._canPerformAction(action)) {
      result._visitDescendants((SemanticsNode node) {
        if (node._canPerformAction(action)) {
          result = node;
          return false; // found node, abort walk
        }
        return true; // continue walk
      });
    }
    if (result == null || !result!._canPerformAction(action)) {
      return null;
    }
    return result!._actions[action];
  }

  /// Asks the [SemanticsNode] with the given id to perform the given action.
  ///
  /// If the [SemanticsNode] has not indicated that it can perform the action,
  /// this function does nothing.
  ///
  /// If the given `action` requires arguments they need to be passed in via
  /// the `args` parameter.
  void performAction(int id, SemanticsAction action, [Object? args]) {
    final SemanticsActionHandler? handler = _getSemanticsActionHandlerForId(id, action);
    if (handler != null) {
      handler(args);
      return;
    }

    // Default actions if no [handler] was provided.
    if (action == SemanticsAction.showOnScreen && _nodes[id]?._showOnScreen != null) {
      _nodes[id]!._showOnScreen!();
    }
  }

  SemanticsActionHandler? _getSemanticsActionHandlerForPosition(
    SemanticsNode node,
    Offset position,
    SemanticsAction action,
  ) {
    if (node.transform != null) {
      final Matrix4 inverse = Matrix4.identity();
      if (inverse.copyInverse(node.transform!) == 0.0) {
        return null;
      }
      position = MatrixUtils.transformPoint(inverse, position);
    }
    if (!node.rect.contains(position)) {
      return null;
    }
    if (node.mergeAllDescendantsIntoThisNode) {
      SemanticsNode? result;
      node._visitDescendants((SemanticsNode child) {
        if (child._canPerformAction(action)) {
          result = child;
          return false;
        }
        return true;
      });
      return result?._actions[action];
    }
    if (node.hasChildren) {
      for (final SemanticsNode child in node._children!.reversed) {
        final SemanticsActionHandler? handler = _getSemanticsActionHandlerForPosition(
          child,
          position,
          action,
        );
        if (handler != null) {
          return handler;
        }
      }
    }
    return node._actions[action];
  }

  /// Asks the [SemanticsNode] at the given position to perform the given action.
  ///
  /// If the [SemanticsNode] has not indicated that it can perform the action,
  /// this function does nothing.
  ///
  /// If the given `action` requires arguments they need to be passed in via
  /// the `args` parameter.
  void performActionAt(Offset position, SemanticsAction action, [Object? args]) {
    final SemanticsNode? node = rootSemanticsNode;
    if (node == null) {
      return;
    }
    final SemanticsActionHandler? handler = _getSemanticsActionHandlerForPosition(
      node,
      position,
      action,
    );
    if (handler != null) {
      handler(args);
    }
  }

  @override
  String toString() => describeIdentity(this);
}

/// Describes the semantic information associated with the owning
/// [RenderObject].
///
/// The information provided in the configuration is used to generate the
/// semantics tree.
class SemanticsConfiguration {
  // SEMANTIC BOUNDARY BEHAVIOR

  /// Whether the [RenderObject] owner of this configuration wants to own its
  /// own [SemanticsNode].
  ///
  /// When set to true semantic information associated with the [RenderObject]
  /// owner of this configuration or any of its descendants will not leak into
  /// parents. The [SemanticsNode] generated out of this configuration will
  /// act as a boundary.
  ///
  /// Whether descendants of the owning [RenderObject] can add their semantic
  /// information to the [SemanticsNode] introduced by this configuration
  /// is controlled by [explicitChildNodes].
  ///
  /// This has to be true if [isMergingSemanticsOfDescendants] is also true.
  bool get isSemanticBoundary => _isSemanticBoundary;
  bool _isSemanticBoundary = false;
  set isSemanticBoundary(bool value) {
    assert(!isMergingSemanticsOfDescendants || value);
    _isSemanticBoundary = value;
  }

  /// Whether to block pointer related user actions for the rendering subtree.
  ///
  /// Setting this to true will prevent users from interacting with the
  /// rendering object produces this semantics configuration and its subtree
  /// through pointer-related [SemanticsAction]s in assistive technologies.
  ///
  /// The [SemanticsNode] created from this semantics configuration is still
  /// focusable by assistive technologies. Only pointer-related
  /// [SemanticsAction]s, such as [SemanticsAction.tap] or its friends, are
  /// blocked.
  ///
  /// If this semantics configuration is merged into a parent semantics node,
  /// only the [SemanticsAction]s from this rendering object and the rendering
  /// objects in the subtree are blocked.
  bool isBlockingUserActions = false;

  /// Whether the configuration forces all children of the owning [RenderObject]
  /// that want to contribute semantic information to the semantics tree to do
  /// so in the form of explicit [SemanticsNode]s.
  ///
  /// When set to false children of the owning [RenderObject] are allowed to
  /// annotate [SemanticsNode]s of their parent with the semantic information
  /// they want to contribute to the semantic tree.
  /// When set to true the only way for children of the owning [RenderObject]
  /// to contribute semantic information to the semantic tree is to introduce
  /// new explicit [SemanticsNode]s to the tree.
  ///
  /// This setting is often used in combination with [isSemanticBoundary] to
  /// create semantic boundaries that are either writable or not for children.
  bool explicitChildNodes = false;

  /// Whether the owning [RenderObject] makes other [RenderObject]s previously
  /// painted within the same semantic boundary unreachable for accessibility
  /// purposes.
  ///
  /// If set to true, the semantic information for all siblings and cousins of
  /// this node, that are earlier in a depth-first pre-order traversal, are
  /// dropped from the semantics tree up until a semantic boundary (as defined
  /// by [isSemanticBoundary]) is reached.
  ///
  /// If [isSemanticBoundary] and [isBlockingSemanticsOfPreviouslyPaintedNodes]
  /// is set on the same node, all previously painted siblings and cousins up
  /// until the next ancestor that is a semantic boundary are dropped.
  ///
  /// Paint order as established by [RenderObject.visitChildrenForSemantics] is
  /// used to determine if a node is previous to this one.
  bool isBlockingSemanticsOfPreviouslyPaintedNodes = false;

  // SEMANTIC ANNOTATIONS
  // These will end up on [SemanticsNode]s generated from
  // [SemanticsConfiguration]s.

  /// Whether this configuration is empty.
  ///
  /// An empty configuration doesn't contain any semantic information that it
  /// wants to contribute to the semantics tree.
  bool get hasBeenAnnotated => _hasBeenAnnotated;
  bool _hasBeenAnnotated = false;

  /// The actions (with associated action handlers) that this configuration
  /// would like to contribute to the semantics tree.
  ///
  /// See also:
  ///
  ///  * [_addAction] to add an action.
  final Map<SemanticsAction, SemanticsActionHandler> _actions =
      <SemanticsAction, SemanticsActionHandler>{};

  int get _effectiveActionsAsBits =>
      isBlockingUserActions ? _actionsAsBits & _kUnblockedUserActions : _actionsAsBits;
  int _actionsAsBits = 0;

  /// Adds an `action` to the semantics tree.
  ///
  /// The provided `handler` is called to respond to the user triggered
  /// `action`.
  void _addAction(SemanticsAction action, SemanticsActionHandler handler) {
    _actions[action] = handler;
    _actionsAsBits |= action.index;
    _hasBeenAnnotated = true;
  }

  /// Adds an `action` to the semantics tree, whose `handler` does not expect
  /// any arguments.
  ///
  /// The provided `handler` is called to respond to the user triggered
  /// `action`.
  void _addArgumentlessAction(SemanticsAction action, VoidCallback handler) {
    _addAction(action, (Object? args) {
      assert(args == null);
      handler();
    });
  }

  /// The handler for [SemanticsAction.tap].
  ///
  /// This is the semantic equivalent of a user briefly tapping the screen with
  /// the finger without moving it. For example, a button should implement this
  /// action.
  ///
  /// VoiceOver users on iOS and TalkBack users on Android can trigger this
  /// action by double-tapping the screen while an element is focused.
  ///
  /// On Android prior to Android Oreo a double-tap on the screen while an
  /// element with an [onTap] handler is focused will not call the registered
  /// handler. Instead, Android will simulate a pointer down and up event at the
  /// center of the focused element. Those pointer events will get dispatched
  /// just like a regular tap with TalkBack disabled would: The events will get
  /// processed by any [GestureDetector] listening for gestures in the center of
  /// the focused element. Therefore, to ensure that [onTap] handlers work
  /// properly on Android versions prior to Oreo, a [GestureDetector] with an
  /// onTap handler should always be wrapping an element that defines a
  /// semantic [onTap] handler. By default a [GestureDetector] will register its
  /// own semantic [onTap] handler that follows this principle.
  VoidCallback? get onTap => _onTap;
  VoidCallback? _onTap;
  set onTap(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.tap, value!);
    _onTap = value;
  }

  /// The handler for [SemanticsAction.longPress].
  ///
  /// This is the semantic equivalent of a user pressing and holding the screen
  /// with the finger for a few seconds without moving it.
  ///
  /// VoiceOver users on iOS and TalkBack users on Android can trigger this
  /// action by double-tapping the screen without lifting the finger after the
  /// second tap.
  VoidCallback? get onLongPress => _onLongPress;
  VoidCallback? _onLongPress;
  set onLongPress(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.longPress, value!);
    _onLongPress = value;
  }

  /// The handler for [SemanticsAction.scrollLeft].
  ///
  /// This is the semantic equivalent of a user moving their finger across the
  /// screen from right to left. It should be recognized by controls that are
  /// horizontally scrollable.
  ///
  /// VoiceOver users on iOS can trigger this action by swiping left with three
  /// fingers. TalkBack users on Android can trigger this action by swiping
  /// right and then left in one motion path. On Android, [onScrollUp] and
  /// [onScrollLeft] share the same gesture. Therefore, only on of them should
  /// be provided.
  VoidCallback? get onScrollLeft => _onScrollLeft;
  VoidCallback? _onScrollLeft;
  set onScrollLeft(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.scrollLeft, value!);
    _onScrollLeft = value;
  }

  /// The handler for [SemanticsAction.dismiss].
  ///
  /// This is a request to dismiss the currently focused node.
  ///
  /// TalkBack users on Android can trigger this action in the local context
  /// menu, and VoiceOver users on iOS can trigger this action with a standard
  /// gesture or menu option.
  VoidCallback? get onDismiss => _onDismiss;
  VoidCallback? _onDismiss;
  set onDismiss(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.dismiss, value!);
    _onDismiss = value;
  }

  /// The handler for [SemanticsAction.scrollRight].
  ///
  /// This is the semantic equivalent of a user moving their finger across the
  /// screen from left to right. It should be recognized by controls that are
  /// horizontally scrollable.
  ///
  /// VoiceOver users on iOS can trigger this action by swiping right with three
  /// fingers. TalkBack users on Android can trigger this action by swiping
  /// left and then right in one motion path. On Android, [onScrollDown] and
  /// [onScrollRight] share the same gesture. Therefore, only on of them should
  /// be provided.
  VoidCallback? get onScrollRight => _onScrollRight;
  VoidCallback? _onScrollRight;
  set onScrollRight(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.scrollRight, value!);
    _onScrollRight = value;
  }

  /// The handler for [SemanticsAction.scrollUp].
  ///
  /// This is the semantic equivalent of a user moving their finger across the
  /// screen from bottom to top. It should be recognized by controls that are
  /// vertically scrollable.
  ///
  /// VoiceOver users on iOS can trigger this action by swiping up with three
  /// fingers. TalkBack users on Android can trigger this action by swiping
  /// right and then left in one motion path. On Android, [onScrollUp] and
  /// [onScrollLeft] share the same gesture. Therefore, only on of them should
  /// be provided.
  VoidCallback? get onScrollUp => _onScrollUp;
  VoidCallback? _onScrollUp;
  set onScrollUp(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.scrollUp, value!);
    _onScrollUp = value;
  }

  /// The handler for [SemanticsAction.scrollDown].
  ///
  /// This is the semantic equivalent of a user moving their finger across the
  /// screen from top to bottom. It should be recognized by controls that are
  /// vertically scrollable.
  ///
  /// VoiceOver users on iOS can trigger this action by swiping down with three
  /// fingers. TalkBack users on Android can trigger this action by swiping
  /// left and then right in one motion path. On Android, [onScrollDown] and
  /// [onScrollRight] share the same gesture. Therefore, only on of them should
  /// be provided.
  VoidCallback? get onScrollDown => _onScrollDown;
  VoidCallback? _onScrollDown;
  set onScrollDown(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.scrollDown, value!);
    _onScrollDown = value;
  }

  /// The handler for [SemanticsAction.scrollToOffset].
  ///
  /// This handler is only called on iOS by UIKit, when the iOS focus engine
  /// switches its focus to an item too close to a scrollable edge of a
  /// scrollable container, to make sure the focused item is always fully
  /// visible.
  ///
  /// The callback, if not `null`, should typically set the scroll offset of
  /// the associated scrollable container to the given `targetOffset` without
  /// animation as it is already animated by the caller: the iOS focus engine
  /// invokes [onScrollToOffset] every frame during the scroll animation with
  /// animated scroll offset values.
  ScrollToOffsetHandler? get onScrollToOffset => _onScrollToOffset;
  ScrollToOffsetHandler? _onScrollToOffset;
  set onScrollToOffset(ScrollToOffsetHandler? value) {
    assert(value != null);
    _addAction(SemanticsAction.scrollToOffset, (Object? args) {
      final Float64List list = args! as Float64List;
      value!(Offset(list[0], list[1]));
    });
    _onScrollToOffset = value;
  }

  /// The handler for [SemanticsAction.increase].
  ///
  /// This is a request to increase the value represented by the widget. For
  /// example, this action might be recognized by a slider control.
  ///
  /// If [value] is set, [increasedValue] must also be provided and
  /// [onIncrease] must ensure that [value] will be set to [increasedValue].
  ///
  /// VoiceOver users on iOS can trigger this action by swiping up with one
  /// finger. TalkBack users on Android can trigger this action by pressing the
  /// volume up button.
  VoidCallback? get onIncrease => _onIncrease;
  VoidCallback? _onIncrease;
  set onIncrease(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.increase, value!);
    _onIncrease = value;
  }

  /// The handler for [SemanticsAction.decrease].
  ///
  /// This is a request to decrease the value represented by the widget. For
  /// example, this action might be recognized by a slider control.
  ///
  /// If [value] is set, [decreasedValue] must also be provided and
  /// [onDecrease] must ensure that [value] will be set to [decreasedValue].
  ///
  /// VoiceOver users on iOS can trigger this action by swiping down with one
  /// finger. TalkBack users on Android can trigger this action by pressing the
  /// volume down button.
  VoidCallback? get onDecrease => _onDecrease;
  VoidCallback? _onDecrease;
  set onDecrease(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.decrease, value!);
    _onDecrease = value;
  }

  /// The handler for [SemanticsAction.copy].
  ///
  /// This is a request to copy the current selection to the clipboard.
  ///
  /// TalkBack users on Android can trigger this action from the local context
  /// menu of a text field, for example.
  VoidCallback? get onCopy => _onCopy;
  VoidCallback? _onCopy;
  set onCopy(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.copy, value!);
    _onCopy = value;
  }

  /// The handler for [SemanticsAction.cut].
  ///
  /// This is a request to cut the current selection and place it in the
  /// clipboard.
  ///
  /// TalkBack users on Android can trigger this action from the local context
  /// menu of a text field, for example.
  VoidCallback? get onCut => _onCut;
  VoidCallback? _onCut;
  set onCut(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.cut, value!);
    _onCut = value;
  }

  /// The handler for [SemanticsAction.paste].
  ///
  /// This is a request to paste the current content of the clipboard.
  ///
  /// TalkBack users on Android can trigger this action from the local context
  /// menu of a text field, for example.
  VoidCallback? get onPaste => _onPaste;
  VoidCallback? _onPaste;
  set onPaste(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.paste, value!);
    _onPaste = value;
  }

  /// The handler for [SemanticsAction.showOnScreen].
  ///
  /// A request to fully show the semantics node on screen. For example, this
  /// action might be send to a node in a scrollable list that is partially off
  /// screen to bring it on screen.
  ///
  /// For elements in a scrollable list the framework provides a default
  /// implementation for this action and it is not advised to provide a
  /// custom one via this setter.
  VoidCallback? get onShowOnScreen => _onShowOnScreen;
  VoidCallback? _onShowOnScreen;
  set onShowOnScreen(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.showOnScreen, value!);
    _onShowOnScreen = value;
  }

  /// The handler for [SemanticsAction.moveCursorForwardByCharacter].
  ///
  /// This handler is invoked when the user wants to move the cursor in a
  /// text field forward by one character.
  ///
  /// TalkBack users can trigger this by pressing the volume up key while the
  /// input focus is in a text field.
  MoveCursorHandler? get onMoveCursorForwardByCharacter => _onMoveCursorForwardByCharacter;
  MoveCursorHandler? _onMoveCursorForwardByCharacter;
  set onMoveCursorForwardByCharacter(MoveCursorHandler? value) {
    assert(value != null);
    _addAction(SemanticsAction.moveCursorForwardByCharacter, (Object? args) {
      final bool extendSelection = args! as bool;
      value!(extendSelection);
    });
    _onMoveCursorForwardByCharacter = value;
  }

  /// The handler for [SemanticsAction.moveCursorBackwardByCharacter].
  ///
  /// This handler is invoked when the user wants to move the cursor in a
  /// text field backward by one character.
  ///
  /// TalkBack users can trigger this by pressing the volume down key while the
  /// input focus is in a text field.
  MoveCursorHandler? get onMoveCursorBackwardByCharacter => _onMoveCursorBackwardByCharacter;
  MoveCursorHandler? _onMoveCursorBackwardByCharacter;
  set onMoveCursorBackwardByCharacter(MoveCursorHandler? value) {
    assert(value != null);
    _addAction(SemanticsAction.moveCursorBackwardByCharacter, (Object? args) {
      final bool extendSelection = args! as bool;
      value!(extendSelection);
    });
    _onMoveCursorBackwardByCharacter = value;
  }

  /// The handler for [SemanticsAction.moveCursorForwardByWord].
  ///
  /// This handler is invoked when the user wants to move the cursor in a
  /// text field backward by one word.
  ///
  /// TalkBack users can trigger this by pressing the volume down key while the
  /// input focus is in a text field.
  MoveCursorHandler? get onMoveCursorForwardByWord => _onMoveCursorForwardByWord;
  MoveCursorHandler? _onMoveCursorForwardByWord;
  set onMoveCursorForwardByWord(MoveCursorHandler? value) {
    assert(value != null);
    _addAction(SemanticsAction.moveCursorForwardByWord, (Object? args) {
      final bool extendSelection = args! as bool;
      value!(extendSelection);
    });
    _onMoveCursorForwardByCharacter = value;
  }

  /// The handler for [SemanticsAction.moveCursorBackwardByWord].
  ///
  /// This handler is invoked when the user wants to move the cursor in a
  /// text field backward by one word.
  ///
  /// TalkBack users can trigger this by pressing the volume down key while the
  /// input focus is in a text field.
  MoveCursorHandler? get onMoveCursorBackwardByWord => _onMoveCursorBackwardByWord;
  MoveCursorHandler? _onMoveCursorBackwardByWord;
  set onMoveCursorBackwardByWord(MoveCursorHandler? value) {
    assert(value != null);
    _addAction(SemanticsAction.moveCursorBackwardByWord, (Object? args) {
      final bool extendSelection = args! as bool;
      value!(extendSelection);
    });
    _onMoveCursorBackwardByCharacter = value;
  }

  /// The handler for [SemanticsAction.setSelection].
  ///
  /// This handler is invoked when the user either wants to change the currently
  /// selected text in a text field or change the position of the cursor.
  ///
  /// TalkBack users can trigger this handler by selecting "Move cursor to
  /// beginning/end" or "Select all" from the local context menu.
  SetSelectionHandler? get onSetSelection => _onSetSelection;
  SetSelectionHandler? _onSetSelection;
  set onSetSelection(SetSelectionHandler? value) {
    assert(value != null);
    _addAction(SemanticsAction.setSelection, (Object? args) {
      assert(args != null && args is Map);
      final Map<String, int> selection = (args! as Map<dynamic, dynamic>).cast<String, int>();
      assert(selection['base'] != null && selection['extent'] != null);
      value!(TextSelection(baseOffset: selection['base']!, extentOffset: selection['extent']!));
    });
    _onSetSelection = value;
  }

  /// The handler for [SemanticsAction.setText].
  ///
  /// This handler is invoked when the user wants to replace the current text in
  /// the text field with a new text.
  ///
  /// Voice access users can trigger this handler by speaking `type <text>` to
  /// their Android devices.
  SetTextHandler? get onSetText => _onSetText;
  SetTextHandler? _onSetText;
  set onSetText(SetTextHandler? value) {
    assert(value != null);
    _addAction(SemanticsAction.setText, (Object? args) {
      assert(args != null && args is String);
      final String text = args! as String;
      value!(text);
    });
    _onSetText = value;
  }

  /// The handler for [SemanticsAction.didGainAccessibilityFocus].
  ///
  /// This handler is invoked when the node annotated with this handler gains
  /// the accessibility focus. The accessibility focus is the
  /// green (on Android with TalkBack) or black (on iOS with VoiceOver)
  /// rectangle shown on screen to indicate what element an accessibility
  /// user is currently interacting with.
  ///
  /// The accessibility focus is different from the input focus. The input focus
  /// is usually held by the element that currently responds to keyboard inputs.
  /// Accessibility focus and input focus can be held by two different nodes!
  ///
  /// See also:
  ///
  ///  * [onDidLoseAccessibilityFocus], which is invoked when the accessibility
  ///    focus is removed from the node.
  ///  * [FocusNode], [FocusScope], [FocusManager], which manage the input focus.
  VoidCallback? get onDidGainAccessibilityFocus => _onDidGainAccessibilityFocus;
  VoidCallback? _onDidGainAccessibilityFocus;
  set onDidGainAccessibilityFocus(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.didGainAccessibilityFocus, value!);
    _onDidGainAccessibilityFocus = value;
  }

  /// The handler for [SemanticsAction.didLoseAccessibilityFocus].
  ///
  /// This handler is invoked when the node annotated with this handler
  /// loses the accessibility focus. The accessibility focus is
  /// the green (on Android with TalkBack) or black (on iOS with VoiceOver)
  /// rectangle shown on screen to indicate what element an accessibility
  /// user is currently interacting with.
  ///
  /// The accessibility focus is different from the input focus. The input focus
  /// is usually held by the element that currently responds to keyboard inputs.
  /// Accessibility focus and input focus can be held by two different nodes!
  ///
  /// See also:
  ///
  ///  * [onDidGainAccessibilityFocus], which is invoked when the node gains
  ///    accessibility focus.
  ///  * [FocusNode], [FocusScope], [FocusManager], which manage the input focus.
  VoidCallback? get onDidLoseAccessibilityFocus => _onDidLoseAccessibilityFocus;
  VoidCallback? _onDidLoseAccessibilityFocus;
  set onDidLoseAccessibilityFocus(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.didLoseAccessibilityFocus, value!);
    _onDidLoseAccessibilityFocus = value;
  }

  /// {@macro flutter.semantics.SemanticsProperties.onFocus}
  VoidCallback? get onFocus => _onFocus;
  VoidCallback? _onFocus;
  set onFocus(VoidCallback? value) {
    _addArgumentlessAction(SemanticsAction.focus, value!);
    _onFocus = value;
  }

  /// A delegate that decides how to handle [SemanticsConfiguration]s produced
  /// in the widget subtree.
  ///
  /// The [SemanticsConfiguration]s are produced by rendering objects in the
  /// subtree and want to merge up to their parent. This delegate can decide
  /// which of these should be merged together to form sibling SemanticsNodes and
  /// which of them should be merged upwards into the parent SemanticsNode.
  ///
  /// The input list of [SemanticsConfiguration]s can be empty if the rendering
  /// object of this semantics configuration is a leaf node or child rendering
  /// objects do not contribute to the semantics.
  ChildSemanticsConfigurationsDelegate? get childConfigurationsDelegate =>
      _childConfigurationsDelegate;
  ChildSemanticsConfigurationsDelegate? _childConfigurationsDelegate;
  set childConfigurationsDelegate(ChildSemanticsConfigurationsDelegate? value) {
    assert(value != null);
    _childConfigurationsDelegate = value;
    // Setting the childConfigsDelegate does not annotate any meaningful
    // semantics information of the config.
  }

  /// Returns the action handler registered for [action] or null if none was
  /// registered.
  SemanticsActionHandler? getActionHandler(SemanticsAction action) => _actions[action];

  /// Determines the position of this node among its siblings in the traversal
  /// sort order.
  ///
  /// This is used to describe the order in which the semantic node should be
  /// traversed by the accessibility services on the platform (e.g. VoiceOver
  /// on iOS and TalkBack on Android).
  ///
  /// Whether this sort key has an effect on the [SemanticsNode] sort order is
  /// subject to how this configuration is used. For example, the [absorb]
  /// method may decide to not use this key when it combines multiple
  /// [SemanticsConfiguration] objects.
  SemanticsSortKey? get sortKey => _sortKey;
  SemanticsSortKey? _sortKey;
  set sortKey(SemanticsSortKey? value) {
    assert(value != null);
    _sortKey = value;
    _hasBeenAnnotated = true;
  }

  /// The index of this node within the parent's list of semantic children.
  ///
  /// This includes all semantic nodes, not just those currently in the
  /// child list. For example, if a scrollable has five children but the first
  /// two are not visible (and thus not included in the list of children), then
  /// the index of the last node will still be 4.
  int? get indexInParent => _indexInParent;
  int? _indexInParent;
  set indexInParent(int? value) {
    _indexInParent = value;
    _hasBeenAnnotated = true;
  }

  /// The total number of scrollable children that contribute to semantics.
  ///
  /// If the number of children are unknown or unbounded, this value will be
  /// null.
  int? get scrollChildCount => _scrollChildCount;
  int? _scrollChildCount;
  set scrollChildCount(int? value) {
    if (value == scrollChildCount) {
      return;
    }
    _scrollChildCount = value;
    _hasBeenAnnotated = true;
  }

  /// The index of the first visible scrollable child that contributes to
  /// semantics.
  int? get scrollIndex => _scrollIndex;
  int? _scrollIndex;
  set scrollIndex(int? value) {
    if (value == scrollIndex) {
      return;
    }
    _scrollIndex = value;
    _hasBeenAnnotated = true;
  }

  /// The id of the platform view, whose semantics nodes will be added as
  /// children to this node.
  int? get platformViewId => _platformViewId;
  int? _platformViewId;
  set platformViewId(int? value) {
    if (value == platformViewId) {
      return;
    }
    _platformViewId = value;
    _hasBeenAnnotated = true;
  }

  /// The maximum number of characters that can be entered into an editable
  /// text field.
  ///
  /// For the purpose of this function a character is defined as one Unicode
  /// scalar value.
  ///
  /// This should only be set when [isTextField] is true. Defaults to null,
  /// which means no limit is imposed on the text field.
  int? get maxValueLength => _maxValueLength;
  int? _maxValueLength;
  set maxValueLength(int? value) {
    if (value == maxValueLength) {
      return;
    }
    _maxValueLength = value;
    _hasBeenAnnotated = true;
  }

  /// The current number of characters that have been entered into an editable
  /// text field.
  ///
  /// For the purpose of this function a character is defined as one Unicode
  /// scalar value.
  ///
  /// This should only be set when [isTextField] is true. Must be set when
  /// [maxValueLength] is set.
  int? get currentValueLength => _currentValueLength;
  int? _currentValueLength;
  set currentValueLength(int? value) {
    if (value == currentValueLength) {
      return;
    }
    _currentValueLength = value;
    _hasBeenAnnotated = true;
  }

  /// Whether the semantic information provided by the owning [RenderObject] and
  /// all of its descendants should be treated as one logical entity.
  ///
  /// If set to true, the descendants of the owning [RenderObject]'s
  /// [SemanticsNode] will merge their semantic information into the
  /// [SemanticsNode] representing the owning [RenderObject].
  ///
  /// Setting this to true requires that [isSemanticBoundary] is also true.
  bool get isMergingSemanticsOfDescendants => _isMergingSemanticsOfDescendants;
  bool _isMergingSemanticsOfDescendants = false;
  set isMergingSemanticsOfDescendants(bool value) {
    assert(isSemanticBoundary);
    _isMergingSemanticsOfDescendants = value;
    _hasBeenAnnotated = true;
  }

  /// The handlers for each supported [CustomSemanticsAction].
  ///
  /// Whenever a custom accessibility action is added to a node, the action
  /// [SemanticsAction.customAction] is automatically added. A handler is
  /// created which uses the passed argument to lookup the custom action
  /// handler from this map and invoke it, if present.
  Map<CustomSemanticsAction, VoidCallback> get customSemanticsActions => _customSemanticsActions;
  Map<CustomSemanticsAction, VoidCallback> _customSemanticsActions =
      <CustomSemanticsAction, VoidCallback>{};
  set customSemanticsActions(Map<CustomSemanticsAction, VoidCallback> value) {
    _hasBeenAnnotated = true;
    _actionsAsBits |= SemanticsAction.customAction.index;
    _customSemanticsActions = value;
    _actions[SemanticsAction.customAction] = _onCustomSemanticsAction;
  }

  void _onCustomSemanticsAction(Object? args) {
    final CustomSemanticsAction? action = CustomSemanticsAction.getAction(args! as int);
    if (action == null) {
      return;
    }
    final VoidCallback? callback = _customSemanticsActions[action];
    if (callback != null) {
      callback();
    }
  }

  /// {@macro flutter.semantics.SemanticsProperties.identifier}
  String get identifier => _identifier;
  String _identifier = '';
  set identifier(String identifier) {
    _identifier = identifier;
    _hasBeenAnnotated = true;
  }

  /// A textual description of the owning [RenderObject].
  ///
  /// Setting this attribute will override the [attributedLabel].
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also:
  ///
  ///  * [attributedLabel], which is the [AttributedString] of this property.
  String get label => _attributedLabel.string;
  set label(String label) {
    _attributedLabel = AttributedString(label);
    _hasBeenAnnotated = true;
  }

  /// A textual description of the owning [RenderObject] in [AttributedString]
  /// format.
  ///
  /// On iOS this is used for the `accessibilityAttributedLabel` property
  /// defined in the `UIAccessibility` Protocol. On Android it is concatenated
  /// together with [attributedValue] and [attributedHint] in the following
  /// order: [attributedValue], [attributedLabel], [attributedHint]. The
  /// concatenated value is then used as the `Text` description.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also:
  ///
  ///  * [label], which is the raw text of this property.
  AttributedString get attributedLabel => _attributedLabel;
  AttributedString _attributedLabel = AttributedString('');
  set attributedLabel(AttributedString attributedLabel) {
    _attributedLabel = attributedLabel;
    _hasBeenAnnotated = true;
  }

  /// A textual description for the current value of the owning [RenderObject].
  ///
  /// Setting this attribute will override the [attributedValue].
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also:
  ///
  ///  * [attributedValue], which is the [AttributedString] of this property.
  ///  * [increasedValue] and [attributedIncreasedValue], which describe what
  ///    [value] will be after performing [SemanticsAction.increase].
  ///  * [decreasedValue] and [attributedDecreasedValue], which describe what
  ///    [value] will be after performing [SemanticsAction.decrease].
  String get value => _attributedValue.string;
  set value(String value) {
    _attributedValue = AttributedString(value);
    _hasBeenAnnotated = true;
  }

  /// A textual description for the current value of the owning [RenderObject]
  /// in [AttributedString] format.
  ///
  /// On iOS this is used for the `accessibilityAttributedValue` property
  /// defined in the `UIAccessibility` Protocol. On Android it is concatenated
  /// together with [attributedLabel] and [attributedHint] in the following
  /// order: [attributedValue], [attributedLabel], [attributedHint]. The
  /// concatenated value is then used as the `Text` description.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also:
  ///
  ///  * [value], which is the raw text of this property.
  ///  * [attributedIncreasedValue], which describes what [value] will be after
  ///    performing [SemanticsAction.increase].
  ///  * [attributedDecreasedValue], which describes what [value] will be after
  ///    performing [SemanticsAction.decrease].
  AttributedString get attributedValue => _attributedValue;
  AttributedString _attributedValue = AttributedString('');
  set attributedValue(AttributedString attributedValue) {
    _attributedValue = attributedValue;
    _hasBeenAnnotated = true;
  }

  /// The value that [value] will have after performing a
  /// [SemanticsAction.increase] action.
  ///
  /// Setting this attribute will override the [attributedIncreasedValue].
  ///
  /// One of the [attributedIncreasedValue] or [increasedValue] must be set if
  /// a handler for [SemanticsAction.increase] is provided and one of the
  /// [value] or [attributedValue] is set.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also:
  ///
  ///  * [attributedIncreasedValue], which is the [AttributedString] of this property.
  String get increasedValue => _attributedIncreasedValue.string;
  set increasedValue(String increasedValue) {
    _attributedIncreasedValue = AttributedString(increasedValue);
    _hasBeenAnnotated = true;
  }

  /// The value that [value] will have after performing a
  /// [SemanticsAction.increase] action in [AttributedString] format.
  ///
  /// One of the [attributedIncreasedValue] or [increasedValue] must be set if
  /// a handler for [SemanticsAction.increase] is provided and one of the
  /// [value] or [attributedValue] is set.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also:
  ///
  ///  * [increasedValue], which is the raw text of this property.
  AttributedString get attributedIncreasedValue => _attributedIncreasedValue;
  AttributedString _attributedIncreasedValue = AttributedString('');
  set attributedIncreasedValue(AttributedString attributedIncreasedValue) {
    _attributedIncreasedValue = attributedIncreasedValue;
    _hasBeenAnnotated = true;
  }

  /// The value that [value] will have after performing a
  /// [SemanticsAction.decrease] action.
  ///
  /// Setting this attribute will override the [attributedDecreasedValue].
  ///
  /// One of the [attributedDecreasedValue] or [decreasedValue] must be set if
  /// a handler for [SemanticsAction.decrease] is provided and one of the
  /// [value] or [attributedValue] is set.
  ///
  /// The reading direction is given by [textDirection].
  ///
  ///  * [attributedDecreasedValue], which is the [AttributedString] of this property.
  String get decreasedValue => _attributedDecreasedValue.string;
  set decreasedValue(String decreasedValue) {
    _attributedDecreasedValue = AttributedString(decreasedValue);
    _hasBeenAnnotated = true;
  }

  /// The value that [value] will have after performing a
  /// [SemanticsAction.decrease] action in [AttributedString] format.
  ///
  /// One of the [attributedDecreasedValue] or [decreasedValue] must be set if
  /// a handler for [SemanticsAction.decrease] is provided and one of the
  /// [value] or [attributedValue] is set.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also:
  ///
  ///  * [decreasedValue], which is the raw text of this property.
  AttributedString get attributedDecreasedValue => _attributedDecreasedValue;
  AttributedString _attributedDecreasedValue = AttributedString('');
  set attributedDecreasedValue(AttributedString attributedDecreasedValue) {
    _attributedDecreasedValue = attributedDecreasedValue;
    _hasBeenAnnotated = true;
  }

  /// A brief description of the result of performing an action on this node.
  ///
  /// Setting this attribute will override the [attributedHint].
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also:
  ///
  ///  * [attributedHint], which is the [AttributedString] of this property.
  String get hint => _attributedHint.string;
  set hint(String hint) {
    _attributedHint = AttributedString(hint);
    _hasBeenAnnotated = true;
  }

  /// A brief description of the result of performing an action on this node in
  /// [AttributedString] format.
  ///
  /// On iOS this is used for the `accessibilityAttributedHint` property
  /// defined in the `UIAccessibility` Protocol. On Android it is concatenated
  /// together with [attributedLabel] and [attributedValue] in the following
  /// order: [attributedValue], [attributedLabel], [attributedHint]. The
  /// concatenated value is then used as the `Text` description.
  ///
  /// The reading direction is given by [textDirection].
  ///
  /// See also:
  ///
  ///  * [hint], which is the raw text of this property.
  AttributedString get attributedHint => _attributedHint;
  AttributedString _attributedHint = AttributedString('');
  set attributedHint(AttributedString attributedHint) {
    _attributedHint = attributedHint;
    _hasBeenAnnotated = true;
  }

  /// A textual description of the widget's tooltip.
  ///
  /// The reading direction is given by [textDirection].
  String get tooltip => _tooltip;
  String _tooltip = '';
  set tooltip(String tooltip) {
    _tooltip = tooltip;
    _hasBeenAnnotated = true;
  }

  /// Provides hint values which override the default hints on supported
  /// platforms.
  SemanticsHintOverrides? get hintOverrides => _hintOverrides;
  SemanticsHintOverrides? _hintOverrides;
  set hintOverrides(SemanticsHintOverrides? value) {
    if (value == null) {
      return;
    }
    _hintOverrides = value;
    _hasBeenAnnotated = true;
  }

  /// The elevation in z-direction at which the owning [RenderObject] is
  /// located relative to its parent.
  double get elevation => _elevation;
  double _elevation = 0.0;
  set elevation(double value) {
    assert(value >= 0.0);
    if (value == _elevation) {
      return;
    }
    _elevation = value;
    _hasBeenAnnotated = true;
  }

  /// The extend that the owning [RenderObject] occupies in z-direction starting
  /// at [elevation].
  ///
  /// It's extremely rare to set this value directly. Instead, it is calculated
  /// implicitly when other [SemanticsConfiguration]s are merged into this one
  /// via [absorb].
  double get thickness => _thickness;
  double _thickness = 0.0;
  set thickness(double value) {
    assert(value >= 0.0);
    if (value == _thickness) {
      return;
    }
    _thickness = value;
    _hasBeenAnnotated = true;
  }

  /// Whether the semantics node is the root of a subtree for which values
  /// should be announced.
  ///
  /// See also:
  ///
  ///  * [SemanticsFlag.scopesRoute], for a full description of route scoping.
  bool get scopesRoute => _hasFlag(SemanticsFlag.scopesRoute);
  set scopesRoute(bool value) {
    _setFlag(SemanticsFlag.scopesRoute, value);
  }

  /// Whether the semantics node contains the label of a route.
  ///
  /// See also:
  ///
  ///  * [SemanticsFlag.namesRoute], for a full description of route naming.
  bool get namesRoute => _hasFlag(SemanticsFlag.namesRoute);
  set namesRoute(bool value) {
    _setFlag(SemanticsFlag.namesRoute, value);
  }

  /// Whether the semantics node represents an image.
  bool get isImage => _hasFlag(SemanticsFlag.isImage);
  set isImage(bool value) {
    _setFlag(SemanticsFlag.isImage, value);
  }

  /// Whether the semantics node is a live region.
  ///
  /// A live region indicates that updates to semantics node are important.
  /// Platforms may use this information to make polite announcements to the
  /// user to inform them of updates to this node.
  ///
  /// An example of a live region is a [SnackBar] widget. On Android and iOS,
  /// live region causes a polite announcement to be generated automatically,
  /// even if the widget does not have accessibility focus. This announcement
  /// may not be spoken if the OS accessibility services are already
  /// announcing something else, such as reading the label of a focused widget
  /// or providing a system announcement.
  ///
  /// See also:
  ///
  ///  * [SemanticsFlag.isLiveRegion], the semantics flag that this setting controls.
  bool get liveRegion => _hasFlag(SemanticsFlag.isLiveRegion);
  set liveRegion(bool value) {
    _setFlag(SemanticsFlag.isLiveRegion, value);
  }

  /// The reading direction for the text in [label], [value], [hint],
  /// [increasedValue], and [decreasedValue].
  TextDirection? get textDirection => _textDirection;
  TextDirection? _textDirection;
  set textDirection(TextDirection? textDirection) {
    _textDirection = textDirection;
    _hasBeenAnnotated = true;
  }

  /// Whether the owning [RenderObject] is selected (true) or not (false).
  ///
  /// This is different from having accessibility focus. The element that is
  /// accessibility focused may or may not be selected; e.g. a [ListTile] can have
  /// accessibility focus but have its [ListTile.selected] property set to false,
  /// in which case it will not be flagged as selected.
  bool get isSelected => _hasFlag(SemanticsFlag.isSelected);
  set isSelected(bool value) {
    _setFlag(SemanticsFlag.hasSelectedState, true);
    _setFlag(SemanticsFlag.isSelected, value);
  }

  /// If this node has Boolean state that can be controlled by the user, whether
  /// that state is expanded or collapsed, corresponding to true and false, respectively.
  ///
  /// Do not call the setter for this field if the owning [RenderObject] doesn't
  /// have expanded/collapsed state that can be controlled by the user.
  ///
  /// The getter returns null if the owning [RenderObject] does not have
  /// expanded/collapsed state.
  bool? get isExpanded =>
      _hasFlag(SemanticsFlag.hasExpandedState) ? _hasFlag(SemanticsFlag.isExpanded) : null;
  set isExpanded(bool? value) {
    _setFlag(SemanticsFlag.hasExpandedState, true);
    _setFlag(SemanticsFlag.isExpanded, value!);
  }

  /// Whether the owning [RenderObject] is currently enabled.
  ///
  /// A disabled object does not respond to user interactions. Only objects that
  /// usually respond to user interactions, but which currently do not (like a
  /// disabled button) should be marked as disabled.
  ///
  /// The setter should not be called for objects (like static text) that never
  /// respond to user interactions.
  ///
  /// The getter will return null if the owning [RenderObject] doesn't support
  /// the concept of being enabled/disabled.
  ///
  /// This property does not control whether semantics are enabled. If you wish to
  /// disable semantics for a particular widget, you should use an [ExcludeSemantics]
  /// widget.
  bool? get isEnabled =>
      _hasFlag(SemanticsFlag.hasEnabledState) ? _hasFlag(SemanticsFlag.isEnabled) : null;
  set isEnabled(bool? value) {
    _setFlag(SemanticsFlag.hasEnabledState, true);
    _setFlag(SemanticsFlag.isEnabled, value!);
  }

  /// If this node has Boolean state that can be controlled by the user, whether
  /// that state is checked or unchecked, corresponding to true and false,
  /// respectively.
  ///
  /// Do not call the setter for this field if the owning [RenderObject] doesn't
  /// have checked/unchecked state that can be controlled by the user.
  ///
  /// The getter returns null if the owning [RenderObject] does not have
  /// checked/unchecked state.
  bool? get isChecked =>
      _hasFlag(SemanticsFlag.hasCheckedState) ? _hasFlag(SemanticsFlag.isChecked) : null;
  set isChecked(bool? value) {
    assert(value != true || isCheckStateMixed != true);
    _setFlag(SemanticsFlag.hasCheckedState, true);
    _setFlag(SemanticsFlag.isChecked, value!);
  }

  /// If this node has tristate that can be controlled by the user, whether
  /// that state is in its mixed state.
  ///
  /// Do not call the setter for this field if the owning [RenderObject] doesn't
  /// have checked/unchecked state that can be controlled by the user.
  ///
  /// The getter returns null if the owning [RenderObject] does not have
  /// mixed checked state.
  bool? get isCheckStateMixed =>
      _hasFlag(SemanticsFlag.hasCheckedState) ? _hasFlag(SemanticsFlag.isCheckStateMixed) : null;
  set isCheckStateMixed(bool? value) {
    assert(value != true || isChecked != true);
    _setFlag(SemanticsFlag.hasCheckedState, true);
    _setFlag(SemanticsFlag.isCheckStateMixed, value!);
  }

  /// If this node has Boolean state that can be controlled by the user, whether
  /// that state is on or off, corresponding to true and false, respectively.
  ///
  /// Do not call the setter for this field if the owning [RenderObject] doesn't
  /// have on/off state that can be controlled by the user.
  ///
  /// The getter returns null if the owning [RenderObject] does not have
  /// on/off state.
  bool? get isToggled =>
      _hasFlag(SemanticsFlag.hasToggledState) ? _hasFlag(SemanticsFlag.isToggled) : null;
  set isToggled(bool? value) {
    _setFlag(SemanticsFlag.hasToggledState, true);
    _setFlag(SemanticsFlag.isToggled, value!);
  }

  /// Whether the owning RenderObject corresponds to UI that allows the user to
  /// pick one of several mutually exclusive options.
  ///
  /// For example, a [Radio] button is in a mutually exclusive group because
  /// only one radio button in that group can be marked as [isChecked].
  bool get isInMutuallyExclusiveGroup => _hasFlag(SemanticsFlag.isInMutuallyExclusiveGroup);
  set isInMutuallyExclusiveGroup(bool value) {
    _setFlag(SemanticsFlag.isInMutuallyExclusiveGroup, value);
  }

  /// Whether the owning [RenderObject] can hold the input focus.
  bool get isFocusable => _hasFlag(SemanticsFlag.isFocusable);
  set isFocusable(bool value) {
    _setFlag(SemanticsFlag.isFocusable, value);
  }

  /// Whether the owning [RenderObject] currently holds the input focus.
  bool get isFocused => _hasFlag(SemanticsFlag.isFocused);
  set isFocused(bool value) {
    _setFlag(SemanticsFlag.isFocused, value);
  }

  /// Whether the owning [RenderObject] is a button (true) or not (false).
  bool get isButton => _hasFlag(SemanticsFlag.isButton);
  set isButton(bool value) {
    _setFlag(SemanticsFlag.isButton, value);
  }

  /// Whether the owning [RenderObject] is a link (true) or not (false).
  bool get isLink => _hasFlag(SemanticsFlag.isLink);
  set isLink(bool value) {
    _setFlag(SemanticsFlag.isLink, value);
  }

  /// The URL that the owning [RenderObject] links to.
  Uri? get linkUrl => _linkUrl;
  Uri? _linkUrl;

  set linkUrl(Uri? value) {
    if (value == _linkUrl) {
      return;
    }
    _linkUrl = value;
    _hasBeenAnnotated = true;
  }

  /// Whether the owning [RenderObject] is a header (true) or not (false).
  bool get isHeader => _hasFlag(SemanticsFlag.isHeader);
  set isHeader(bool value) {
    _setFlag(SemanticsFlag.isHeader, value);
  }

  /// Indicates the heading level in the document structure.
  ///
  /// This is only used for web semantics, and is ignored on other platforms.
  int get headingLevel => _headingLevel;
  int _headingLevel = 0;

  set headingLevel(int value) {
    assert(value >= 0 && value <= 6);
    if (value == headingLevel) {
      return;
    }
    _headingLevel = value;
    _hasBeenAnnotated = true;
  }

  /// Whether the owning [RenderObject] is a slider (true) or not (false).
  bool get isSlider => _hasFlag(SemanticsFlag.isSlider);
  set isSlider(bool value) {
    _setFlag(SemanticsFlag.isSlider, value);
  }

  /// Whether the owning [RenderObject] is a keyboard key (true) or not
  //(false).
  bool get isKeyboardKey => _hasFlag(SemanticsFlag.isKeyboardKey);
  set isKeyboardKey(bool value) {
    _setFlag(SemanticsFlag.isKeyboardKey, value);
  }

  /// Whether the owning [RenderObject] is considered hidden.
  ///
  /// Hidden elements are currently not visible on screen. They may be covered
  /// by other elements or positioned outside of the visible area of a viewport.
  ///
  /// Hidden elements cannot gain accessibility focus though regular touch. The
  /// only way they can be focused is by moving the focus to them via linear
  /// navigation.
  ///
  /// Platforms are free to completely ignore hidden elements and new platforms
  /// are encouraged to do so.
  ///
  /// Instead of marking an element as hidden it should usually be excluded from
  /// the semantics tree altogether. Hidden elements are only included in the
  /// semantics tree to work around platform limitations and they are mainly
  /// used to implement accessibility scrolling on iOS.
  bool get isHidden => _hasFlag(SemanticsFlag.isHidden);
  set isHidden(bool value) {
    _setFlag(SemanticsFlag.isHidden, value);
  }

  /// Whether the owning [RenderObject] is a text field.
  bool get isTextField => _hasFlag(SemanticsFlag.isTextField);
  set isTextField(bool value) {
    _setFlag(SemanticsFlag.isTextField, value);
  }

  /// Whether the owning [RenderObject] is read only.
  ///
  /// Only applicable when [isTextField] is true.
  bool get isReadOnly => _hasFlag(SemanticsFlag.isReadOnly);
  set isReadOnly(bool value) {
    _setFlag(SemanticsFlag.isReadOnly, value);
  }

  /// Whether [value] should be obscured.
  ///
  /// This option is usually set in combination with [isTextField] to indicate
  /// that the text field contains a password (or other sensitive information).
  /// Doing so instructs screen readers to not read out [value].
  bool get isObscured => _hasFlag(SemanticsFlag.isObscured);
  set isObscured(bool value) {
    _setFlag(SemanticsFlag.isObscured, value);
  }

  /// Whether the text field is multiline.
  ///
  /// This option is usually set in combination with [isTextField] to indicate
  /// that the text field is configured to be multiline.
  bool get isMultiline => _hasFlag(SemanticsFlag.isMultiline);
  set isMultiline(bool value) {
    _setFlag(SemanticsFlag.isMultiline, value);
  }

  /// Whether the platform can scroll the semantics node when the user attempts
  /// to move focus to an offscreen child.
  ///
  /// For example, a [ListView] widget has implicit scrolling so that users can
  /// easily move to the next visible set of children. A [TabBar] widget does
  /// not have implicit scrolling, so that users can navigate into the tab
  /// body when reaching the end of the tab bar.
  bool get hasImplicitScrolling => _hasFlag(SemanticsFlag.hasImplicitScrolling);
  set hasImplicitScrolling(bool value) {
    _setFlag(SemanticsFlag.hasImplicitScrolling, value);
  }

  /// The currently selected text (or the position of the cursor) within
  /// [value] if this node represents a text field.
  TextSelection? get textSelection => _textSelection;
  TextSelection? _textSelection;
  set textSelection(TextSelection? value) {
    assert(value != null);
    _textSelection = value;
    _hasBeenAnnotated = true;
  }

  /// Indicates the current scrolling position in logical pixels if the node is
  /// scrollable.
  ///
  /// The properties [scrollExtentMin] and [scrollExtentMax] indicate the valid
  /// in-range values for this property. The value for [scrollPosition] may
  /// (temporarily) be outside that range, e.g. during an overscroll.
  ///
  /// See also:
  ///
  ///  * [ScrollPosition.pixels], from where this value is usually taken.
  double? get scrollPosition => _scrollPosition;
  double? _scrollPosition;
  set scrollPosition(double? value) {
    assert(value != null);
    _scrollPosition = value;
    _hasBeenAnnotated = true;
  }

  /// Indicates the maximum in-range value for [scrollPosition] if the node is
  /// scrollable.
  ///
  /// This value may be infinity if the scroll is unbound.
  ///
  /// See also:
  ///
  ///  * [ScrollPosition.maxScrollExtent], from where this value is usually taken.
  double? get scrollExtentMax => _scrollExtentMax;
  double? _scrollExtentMax;
  set scrollExtentMax(double? value) {
    assert(value != null);
    _scrollExtentMax = value;
    _hasBeenAnnotated = true;
  }

  /// Indicates the minimum in-range value for [scrollPosition] if the node is
  /// scrollable.
  ///
  /// This value may be infinity if the scroll is unbound.
  ///
  /// See also:
  ///
  ///  * [ScrollPosition.minScrollExtent], from where this value is usually taken.
  double? get scrollExtentMin => _scrollExtentMin;
  double? _scrollExtentMin;
  set scrollExtentMin(double? value) {
    assert(value != null);
    _scrollExtentMin = value;
    _hasBeenAnnotated = true;
  }

  // TAGS

  /// The set of tags that this configuration wants to add to all child
  /// [SemanticsNode]s.
  ///
  /// See also:
  ///
  ///  * [addTagForChildren] to add a tag and for more information about their
  ///    usage.
  Iterable<SemanticsTag>? get tagsForChildren => _tagsForChildren;

  /// Whether this configuration will tag the child semantics nodes with a
  /// given [SemanticsTag].
  bool tagsChildrenWith(SemanticsTag tag) => _tagsForChildren?.contains(tag) ?? false;

  Set<SemanticsTag>? _tagsForChildren;

  /// Specifies a [SemanticsTag] that this configuration wants to apply to all
  /// child [SemanticsNode]s.
  ///
  /// The tag is added to all [SemanticsNode] that pass through the
  /// [RenderObject] owning this configuration while looking to be attached to a
  /// parent [SemanticsNode].
  ///
  /// Tags are used to communicate to a parent [SemanticsNode] that a child
  /// [SemanticsNode] was passed through a particular [RenderObject]. The parent
  /// can use this information to determine the shape of the semantics tree.
  ///
  /// See also:
  ///
  ///  * [RenderViewport.excludeFromScrolling] for an example of
  ///    how tags are used.
  void addTagForChildren(SemanticsTag tag) {
    _tagsForChildren ??= <SemanticsTag>{};
    _tagsForChildren!.add(tag);
  }

  // INTERNAL FLAG MANAGEMENT

  int _flags = 0;
  void _setFlag(SemanticsFlag flag, bool value) {
    if (value) {
      _flags |= flag.index;
    } else {
      _flags &= ~flag.index;
    }
    _hasBeenAnnotated = true;
  }

  bool _hasFlag(SemanticsFlag flag) => (_flags & flag.index) != 0;

  // CONFIGURATION COMBINATION LOGIC

  /// Whether this configuration is compatible with the provided `other`
  /// configuration.
  ///
  /// Two configurations are said to be compatible if they can be added to the
  /// same [SemanticsNode] without losing any semantics information.
  bool isCompatibleWith(SemanticsConfiguration? other) {
    if (other == null || !other.hasBeenAnnotated || !hasBeenAnnotated) {
      return true;
    }
    if (_actionsAsBits & other._actionsAsBits != 0) {
      return false;
    }
    if ((_flags & other._flags) != 0) {
      return false;
    }
    if (_platformViewId != null && other._platformViewId != null) {
      return false;
    }
    if (_maxValueLength != null && other._maxValueLength != null) {
      return false;
    }
    if (_currentValueLength != null && other._currentValueLength != null) {
      return false;
    }
    if (_attributedValue.string.isNotEmpty && other._attributedValue.string.isNotEmpty) {
      return false;
    }
    return true;
  }

  /// Absorb the semantic information from `child` into this configuration.
  ///
  /// This adds the semantic information of both configurations and saves the
  /// result in this configuration.
  ///
  /// The [RenderObject] owning the `child` configuration must be a descendant
  /// of the [RenderObject] that owns this configuration.
  ///
  /// Only configurations that have [explicitChildNodes] set to false can
  /// absorb other configurations and it is recommended to only absorb compatible
  /// configurations as determined by [isCompatibleWith].
  void absorb(SemanticsConfiguration child) {
    assert(!explicitChildNodes);

    if (!child.hasBeenAnnotated) {
      return;
    }
    if (child.isBlockingUserActions) {
      child._actions.forEach((SemanticsAction key, SemanticsActionHandler value) {
        if (_kUnblockedUserActions & key.index > 0) {
          _actions[key] = value;
        }
      });
    } else {
      _actions.addAll(child._actions);
    }
    _actionsAsBits |= child._effectiveActionsAsBits;
    _customSemanticsActions.addAll(child._customSemanticsActions);
    _flags |= child._flags;
    _textSelection ??= child._textSelection;
    _scrollPosition ??= child._scrollPosition;
    _scrollExtentMax ??= child._scrollExtentMax;
    _scrollExtentMin ??= child._scrollExtentMin;
    _hintOverrides ??= child._hintOverrides;
    _indexInParent ??= child.indexInParent;
    _scrollIndex ??= child._scrollIndex;
    _scrollChildCount ??= child._scrollChildCount;
    _platformViewId ??= child._platformViewId;
    _maxValueLength ??= child._maxValueLength;
    _currentValueLength ??= child._currentValueLength;

    _headingLevel = _mergeHeadingLevels(
      sourceLevel: child._headingLevel,
      targetLevel: _headingLevel,
    );

    textDirection ??= child.textDirection;
    _sortKey ??= child._sortKey;
    if (_identifier == '') {
      _identifier = child._identifier;
    }
    _attributedLabel = _concatAttributedString(
      thisAttributedString: _attributedLabel,
      thisTextDirection: textDirection,
      otherAttributedString: child._attributedLabel,
      otherTextDirection: child.textDirection,
    );
    if (_attributedValue.string == '') {
      _attributedValue = child._attributedValue;
    }
    if (_attributedIncreasedValue.string == '') {
      _attributedIncreasedValue = child._attributedIncreasedValue;
    }
    if (_attributedDecreasedValue.string == '') {
      _attributedDecreasedValue = child._attributedDecreasedValue;
    }
    _attributedHint = _concatAttributedString(
      thisAttributedString: _attributedHint,
      thisTextDirection: textDirection,
      otherAttributedString: child._attributedHint,
      otherTextDirection: child.textDirection,
    );
    if (_tooltip == '') {
      _tooltip = child._tooltip;
    }

    _thickness = math.max(_thickness, child._thickness + child._elevation);

    _hasBeenAnnotated = _hasBeenAnnotated || child._hasBeenAnnotated;
  }

  /// Returns an exact copy of this configuration.
  SemanticsConfiguration copy() {
    return SemanticsConfiguration()
      .._isSemanticBoundary = _isSemanticBoundary
      ..explicitChildNodes = explicitChildNodes
      ..isBlockingSemanticsOfPreviouslyPaintedNodes = isBlockingSemanticsOfPreviouslyPaintedNodes
      .._hasBeenAnnotated = _hasBeenAnnotated
      .._isMergingSemanticsOfDescendants = _isMergingSemanticsOfDescendants
      .._textDirection = _textDirection
      .._sortKey = _sortKey
      .._identifier = _identifier
      .._attributedLabel = _attributedLabel
      .._attributedIncreasedValue = _attributedIncreasedValue
      .._attributedValue = _attributedValue
      .._attributedDecreasedValue = _attributedDecreasedValue
      .._attributedHint = _attributedHint
      .._hintOverrides = _hintOverrides
      .._tooltip = _tooltip
      .._elevation = _elevation
      .._thickness = _thickness
      .._flags = _flags
      .._tagsForChildren = _tagsForChildren
      .._textSelection = _textSelection
      .._scrollPosition = _scrollPosition
      .._scrollExtentMax = _scrollExtentMax
      .._scrollExtentMin = _scrollExtentMin
      .._actionsAsBits = _actionsAsBits
      .._indexInParent = indexInParent
      .._scrollIndex = _scrollIndex
      .._scrollChildCount = _scrollChildCount
      .._platformViewId = _platformViewId
      .._maxValueLength = _maxValueLength
      .._currentValueLength = _currentValueLength
      .._actions.addAll(_actions)
      .._customSemanticsActions.addAll(_customSemanticsActions)
      ..isBlockingUserActions = isBlockingUserActions
      .._headingLevel = _headingLevel
      .._linkUrl = _linkUrl;
  }
}

/// Used by [debugDumpSemanticsTree] to specify the order in which child nodes
/// are printed.
enum DebugSemanticsDumpOrder {
  /// Print nodes in inverse hit test order.
  ///
  /// In inverse hit test order, the last child of a [SemanticsNode] will be
  /// asked first if it wants to respond to a user's interaction, followed by
  /// the second last, etc. until a taker is found.
  inverseHitTest,

  /// Print nodes in semantic traversal order.
  ///
  /// This is the order in which a user would navigate the UI using the "next"
  /// and "previous" gestures.
  traversalOrder,
}

AttributedString _concatAttributedString({
  required AttributedString thisAttributedString,
  required AttributedString otherAttributedString,
  required TextDirection? thisTextDirection,
  required TextDirection? otherTextDirection,
}) {
  if (otherAttributedString.string.isEmpty) {
    return thisAttributedString;
  }
  if (thisTextDirection != otherTextDirection && otherTextDirection != null) {
    final AttributedString directionEmbedding = switch (otherTextDirection) {
      TextDirection.rtl => AttributedString(Unicode.RLE),
      TextDirection.ltr => AttributedString(Unicode.LRE),
    };
    otherAttributedString =
        directionEmbedding + otherAttributedString + AttributedString(Unicode.PDF);
  }
  if (thisAttributedString.string.isEmpty) {
    return otherAttributedString;
  }

  return thisAttributedString + AttributedString('\n') + otherAttributedString;
}

/// Base class for all sort keys for [SemanticsProperties.sortKey] accessibility
/// traversal order sorting.
///
/// Sort keys are sorted by [name], then by the comparison that the subclass
/// implements. If [SemanticsProperties.sortKey] is specified, sort keys within
/// the same semantic group must all be of the same type.
///
/// Keys with no [name] are compared to other keys with no [name], and will
/// be traversed before those with a [name].
///
/// If no sort key is applied to a semantics node, then it will be ordered using
/// a platform dependent default algorithm.
///
/// See also:
///
///  * [OrdinalSortKey] for a sort key that sorts using an ordinal.
abstract class SemanticsSortKey with Diagnosticable implements Comparable<SemanticsSortKey> {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const SemanticsSortKey({this.name});

  /// An optional name that will group this sort key with other sort keys of the
  /// same [name].
  ///
  /// Sort keys must have the same `runtimeType` when compared.
  ///
  /// Keys with no [name] are compared to other keys with no [name], and will
  /// be traversed before those with a [name].
  final String? name;

  @override
  int compareTo(SemanticsSortKey other) {
    // Sort by name first and then subclass ordering.
    assert(
      runtimeType == other.runtimeType,
      'Semantics sort keys can only be compared to other sort keys of the same type.',
    );

    // Defer to the subclass implementation for ordering only if the names are
    // identical (or both null).
    if (name == other.name) {
      return doCompare(other);
    }

    // Keys that don't have a name are sorted together and come before those with
    // a name.
    if (name == null && other.name != null) {
      return -1;
    } else if (name != null && other.name == null) {
      return 1;
    }

    return name!.compareTo(other.name!);
  }

  /// The implementation of [compareTo].
  ///
  /// The argument is guaranteed to be of the same type as this object and have
  /// the same [name].
  ///
  /// The method should return a negative number if this object comes earlier in
  /// the sort order than the argument; and a positive number if it comes later
  /// in the sort order. Returning zero causes the system to use default sort
  /// order.
  @protected
  int doCompare(covariant SemanticsSortKey other);

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(StringProperty('name', name, defaultValue: null));
  }
}

/// A [SemanticsSortKey] that sorts based on the `double` value it is
/// given.
///
/// The [OrdinalSortKey] compares itself with other [OrdinalSortKey]s
/// to sort based on the order it is given.
///
/// [OrdinalSortKey]s are sorted by the optional [name], then by their [order].
/// If [SemanticsProperties.sortKey] is a [OrdinalSortKey], then all the other
/// specified sort keys in the same semantics group must also be
/// [OrdinalSortKey]s.
///
/// Keys with no [name] are compared to other keys with no [name], and will
/// be traversed before those with a [name].
///
/// The ordinal value [order] is typically a whole number, though it can be
/// fractional, e.g. in order to fit between two other consecutive whole
/// numbers. The value must be finite (it cannot be [double.nan],
/// [double.infinity], or [double.negativeInfinity]).
class OrdinalSortKey extends SemanticsSortKey {
  /// Creates a const semantics sort key that uses a [double] as its key value.
  ///
  /// The [order] must be a finite number.
  const OrdinalSortKey(this.order, {super.name})
    : assert(order > double.negativeInfinity),
      assert(order < double.infinity);

  /// Determines the placement of this key in a sequence of keys that defines
  /// the order in which this node is traversed by the platform's accessibility
  /// services.
  ///
  /// Lower values will be traversed first. Keys with the same [name] will be
  /// grouped together and sorted by name first, and then sorted by [order].
  final double order;

  @override
  int doCompare(OrdinalSortKey other) {
    if (other.order == order) {
      return 0;
    }
    return order.compareTo(other.order);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('order', order, defaultValue: null));
  }
}

/// Picks the most accurate heading level when two nodes, with potentially
/// different heading levels, are merged.
///
/// Argument [sourceLevel] is the heading level of the source node that is being
/// merged into a target node, which has heading level [targetLevel].
///
/// If the target node is not a heading, the the source heading level is used.
/// Otherwise, the target heading level is used irrespective of the source
/// heading level.
int _mergeHeadingLevels({required int sourceLevel, required int targetLevel}) {
  return targetLevel == 0 ? sourceLevel : targetLevel;
}
