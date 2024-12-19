// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
///
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'placeholder_span.dart';
library;

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/services.dart';

import 'basic_types.dart';
import 'text_painter.dart';
import 'text_scaler.dart';
import 'text_span.dart';
import 'text_style.dart';

// Examples can assume:
// late InlineSpan myInlineSpan;

/// The signature of [InlineSpanAttributes.remove].
///
/// The type information is intentionally kept obscure because it is used as a
/// sentinel value.
typedef RemoveInlineSpanAttribute = _PoorMansBottomType;

/// Mutable wrapper of an integer that can be passed by reference to track a
/// value across a recursive stack.
class Accumulator {
  /// [Accumulator] may be initialized with a specified value, otherwise, it will
  /// initialize to zero.
  Accumulator([this._value = 0]);

  /// The integer stored in this [Accumulator].
  int get value => _value;
  int _value;

  /// Increases the [value] by the `addend`.
  void increment(int addend) {
    assert(addend >= 0);
    _value += addend;
  }
}
/// Called on each span as [InlineSpan.visitChildren] walks the [InlineSpan] tree.
///
/// Returns true when the walk should continue, and false to stop visiting further
/// [InlineSpan]s.
typedef InlineSpanVisitor = bool Function(InlineSpan span);

/// The textual and semantic label information for an [InlineSpan].
///
/// For [PlaceholderSpan]s, [InlineSpanSemanticsInformation.placeholder] is used by default.
///
/// See also:
///
///  * [InlineSpan.getSemanticsInformation]
@immutable
class InlineSpanSemanticsInformation {
  /// Constructs an object that holds the text and semantics label values of an
  /// [InlineSpan].
  ///
  /// Use [InlineSpanSemanticsInformation.placeholder] instead of directly setting
  /// [isPlaceholder].
  const InlineSpanSemanticsInformation(
    this.text, {
    this.isPlaceholder = false,
    this.semanticsLabel,
    this.stringAttributes = const <ui.StringAttribute>[],
    this.recognizer,
  }) : assert(!isPlaceholder || (text == '\uFFFC' && semanticsLabel == null && recognizer == null)),
       requiresOwnNode = isPlaceholder || recognizer != null;

  /// The text info for a [PlaceholderSpan].
  static const InlineSpanSemanticsInformation placeholder = InlineSpanSemanticsInformation('\uFFFC', isPlaceholder: true);

  /// The text value, if any. For [PlaceholderSpan]s, this will be the unicode
  /// placeholder value.
  final String text;

  /// The semanticsLabel, if any.
  final String? semanticsLabel;

  /// The gesture recognizer, if any, for this span.
  final GestureRecognizer? recognizer;

  /// Whether this is for a placeholder span.
  final bool isPlaceholder;

  /// True if this configuration should get its own semantics node.
  ///
  /// This will be the case of the [recognizer] is not null, of if
  /// [isPlaceholder] is true.
  final bool requiresOwnNode;

  /// The string attributes attached to this semantics information
  final List<ui.StringAttribute> stringAttributes;

  @override
  bool operator ==(Object other) {
    return other is InlineSpanSemanticsInformation
        && other.text == text
        && other.semanticsLabel == semanticsLabel
        && other.recognizer == recognizer
        && other.isPlaceholder == isPlaceholder
        && listEquals<ui.StringAttribute>(other.stringAttributes, stringAttributes);
  }

  @override
  int get hashCode => Object.hash(text, semanticsLabel, recognizer, isPlaceholder);

  @override
  String toString() => '${objectRuntimeType(this, 'InlineSpanSemanticsInformation')}{text: $text, semanticsLabel: $semanticsLabel, recognizer: $recognizer}';
}

/// Combines _semanticsInfo entries where permissible.
///
/// Consecutive inline spans can be combined if their
/// [InlineSpanSemanticsInformation.requiresOwnNode] return false.
List<InlineSpanSemanticsInformation> combineSemanticsInfo(List<InlineSpanSemanticsInformation> infoList) {
  final List<InlineSpanSemanticsInformation> combined = <InlineSpanSemanticsInformation>[];
  String workingText = '';
  String workingLabel = '';
  List<ui.StringAttribute> workingAttributes = <ui.StringAttribute>[];
  for (final InlineSpanSemanticsInformation info in infoList) {
    if (info.requiresOwnNode) {
      combined.add(InlineSpanSemanticsInformation(
        workingText,
        semanticsLabel: workingLabel,
        stringAttributes: workingAttributes,
      ));
      workingText = '';
      workingLabel = '';
      workingAttributes = <ui.StringAttribute>[];
      combined.add(info);
    } else {
      workingText += info.text;
      final String effectiveLabel = info.semanticsLabel ?? info.text;
      for (final ui.StringAttribute infoAttribute in info.stringAttributes) {
        workingAttributes.add(
          infoAttribute.copy(
            range: TextRange(
              start: infoAttribute.range.start + workingLabel.length,
              end: infoAttribute.range.end + workingLabel.length,
            ),
          ),
        );
      }
      workingLabel += effectiveLabel;

    }
  }
  combined.add(InlineSpanSemanticsInformation(
    workingText,
    semanticsLabel: workingLabel,
    stringAttributes: workingAttributes,
  ));
  return combined;
}

/// An immutable span of inline content which forms part of a paragraph.
///
///  * The subclass [TextSpan] specifies text and may contain child [InlineSpan]s.
///  * The subclass [PlaceholderSpan] represents a placeholder that may be
///    filled with non-text content. [PlaceholderSpan] itself defines a
///    [PlaceholderAlignment] and a [TextBaseline]. To be useful,
///    [PlaceholderSpan] must be extended to define content. An instance of
///    this is the [WidgetSpan] class in the widgets library.
///  * The subclass [WidgetSpan] specifies embedded inline widgets.
///
/// {@tool snippet}
///
/// This example shows a tree of [InlineSpan]s that make a query asking for a
/// name with a [TextField] embedded inline.
///
/// ```dart
/// Text.rich(
///   TextSpan(
///     text: 'My name is ',
///     style: const TextStyle(color: Colors.black),
///     children: <InlineSpan>[
///       WidgetSpan(
///         alignment: PlaceholderAlignment.baseline,
///         baseline: TextBaseline.alphabetic,
///         child: ConstrainedBox(
///           constraints: const BoxConstraints(maxWidth: 100),
///           child: const TextField(),
///         )
///       ),
///       const TextSpan(
///         text: '.',
///       ),
///     ],
///   ),
/// )
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [Text], a widget for showing uniformly-styled text.
///  * [RichText], a widget for finer control of text rendering.
///  * [TextPainter], a class for painting [InlineSpan] objects on a [Canvas].
@immutable
abstract class InlineSpan extends DiagnosticableTree {
  /// Creates an [InlineSpan] with the given values.
  const InlineSpan({
    this.style,
  });

  /// The [TextStyle] to apply to this span.
  ///
  /// The [style] is also applied to any child spans when this is an instance
  /// of [TextSpan].
  final TextStyle? style;

  /// Apply the properties of this object to the given [ParagraphBuilder], from
  /// which a [Paragraph] can be obtained.
  ///
  /// The `textScaler` parameter specifies a [TextScaler] that the text and
  /// placeholders will be scaled by. The scaling is performed before layout,
  /// so the text will be laid out with the scaled glyphs and placeholders.
  ///
  /// The `dimensions` parameter specifies the sizes of the placeholders.
  /// Each [PlaceholderSpan] must be paired with a [PlaceholderDimensions]
  /// in the same order as defined in the [InlineSpan] tree.
  ///
  /// [Paragraph] objects can be drawn on [Canvas] objects.
  void build(ui.ParagraphBuilder builder, {
    TextScaler textScaler = TextScaler.noScaling,
    List<PlaceholderDimensions>? dimensions,
  });

  /// Walks this [InlineSpan] and any descendants in pre-order and calls `visitor`
  /// for each span that has content.
  ///
  /// When `visitor` returns true, the walk will continue. When `visitor` returns
  /// false, then the walk will end.
  ///
  /// See also:
  ///
  ///  * [visitDirectChildren], which preforms `build`-order traversal on the
  ///    immediate children of this [InlineSpan], regardless of whether they
  ///    have content.
  bool visitChildren(InlineSpanVisitor visitor);

  /// Calls `visitor` for each immediate child of this [InlineSpan].
  ///
  /// The immediate children are visited in the same order they are added to
  /// a [ui.ParagraphBuilder] in the [build] method, which is also the logical
  /// order of the child [InlineSpan]s in the text.
  ///
  /// The traversal stops when all immediate children are visited, or when the
  /// `visitor` callback returns `false` on an immediate child. This method
  /// itself returns a `bool` indicating whether the visitor callback returned
  /// `true` on all immediate children.
  ///
  /// See also:
  ///
  ///  * [visitChildren], which performs preorder traversal on this [InlineSpan]
  ///    if it has content, and all its descendants with content.
  bool visitDirectChildren(InlineSpanVisitor visitor);

  /// Returns the [InlineSpan] that contains the given position in the text.
  InlineSpan? getSpanForPosition(TextPosition position) {
    assert(debugAssertIsValid());
    final Accumulator offset = Accumulator();
    InlineSpan? result;
    visitChildren((InlineSpan span) {
      result = span.getSpanForPositionVisitor(position, offset);
      return result == null;
    });
    return result;
  }

  /// Performs the check at each [InlineSpan] for if the `position` falls within the range
  /// of the span and returns the span if it does.
  ///
  /// The `offset` parameter tracks the current index offset in the text buffer formed
  /// if the contents of the [InlineSpan] tree were concatenated together starting
  /// from the root [InlineSpan].
  ///
  /// This method should not be directly called. Use [getSpanForPosition] instead.
  @protected
  InlineSpan? getSpanForPositionVisitor(TextPosition position, Accumulator offset);

  /// Flattens the [InlineSpan] tree into a single string.
  ///
  /// Styles are not honored in this process. If `includeSemanticsLabels` is
  /// true, then the text returned will include the [TextSpan.semanticsLabel]s
  /// instead of the text contents for [TextSpan]s.
  ///
  /// When `includePlaceholders` is true, [PlaceholderSpan]s in the tree will be
  /// represented as a 0xFFFC 'object replacement character'.
  String toPlainText({bool includeSemanticsLabels = true, bool includePlaceholders = true}) {
    final StringBuffer buffer = StringBuffer();
    computeToPlainText(buffer, includeSemanticsLabels: includeSemanticsLabels, includePlaceholders: includePlaceholders);
    return buffer.toString();
  }

  /// Flattens the [InlineSpan] tree to a list of
  /// [InlineSpanSemanticsInformation] objects.
  ///
  /// [PlaceholderSpan]s in the tree will be represented with a
  /// [InlineSpanSemanticsInformation.placeholder] value.
  List<InlineSpanSemanticsInformation> getSemanticsInformation() {
    final List<InlineSpanSemanticsInformation> collector = <InlineSpanSemanticsInformation>[];
    computeSemanticsInformation(collector);
    return collector;
  }

  /// Walks the [InlineSpan] tree and accumulates a list of
  /// [InlineSpanSemanticsInformation] objects.
  ///
  /// This method should not be directly called. Use
  /// [getSemanticsInformation] instead.
  ///
  /// [PlaceholderSpan]s in the tree will be represented with a
  /// [InlineSpanSemanticsInformation.placeholder] value.
  @protected
  void computeSemanticsInformation(List<InlineSpanSemanticsInformation> collector);

  /// Walks the [InlineSpan] tree and writes the plain text representation to `buffer`.
  ///
  /// This method should not be directly called. Use [toPlainText] instead.
  ///
  /// Styles are not honored in this process. If `includeSemanticsLabels` is
  /// true, then the text returned will include the [TextSpan.semanticsLabel]s
  /// instead of the text contents for [TextSpan]s.
  ///
  /// When `includePlaceholders` is true, [PlaceholderSpan]s in the tree will be
  /// represented as a 0xFFFC 'object replacement character'.
  ///
  /// The plain-text representation of this [InlineSpan] is written into the `buffer`.
  /// This method will then recursively call [computeToPlainText] on its children
  /// [InlineSpan]s if available.
  @protected
  void computeToPlainText(StringBuffer buffer, {bool includeSemanticsLabels = true, bool includePlaceholders = true});

  /// Returns the UTF-16 code unit at the given `index` in the flattened string.
  ///
  /// This only accounts for the [TextSpan.text] values and ignores [PlaceholderSpan]s.
  ///
  /// Returns null if the `index` is out of bounds.
  int? codeUnitAt(int index) {
    if (index < 0) {
      return null;
    }
    final Accumulator offset = Accumulator();
    int? result;
    visitChildren((InlineSpan span) {
      result = span.codeUnitAtVisitor(index, offset);
      return result == null;
    });
    return result;
  }

  /// Performs the check at each [InlineSpan] for if the `index` falls within the range
  /// of the span and returns the corresponding code unit. Returns null otherwise.
  ///
  /// The `offset` parameter tracks the current index offset in the text buffer formed
  /// if the contents of the [InlineSpan] tree were concatenated together starting
  /// from the root [InlineSpan].
  ///
  /// This method should not be directly called. Use [codeUnitAt] instead.
  @protected
  int? codeUnitAtVisitor(int index, Accumulator offset);

  /// Returns an [InlineSpan] by applying `newAttributes` to a [TextRange] within
  /// this span.
  ///
  /// The `textRange` argument is the range, in relation to the start of the root
  /// span, to which the `newAttributes` should be applied.
  ///
  /// The `offset` argument is a mutable offset. When this implementation is
  /// invoked, its value is set to the offset from the start of the root span to
  /// the span of this [InlineSpan]. The implementation is responsible for
  /// advancing the offset by this [InlineSpan]'s length in UTF16 code units,
  /// such that the `offset` points to the end (enclusive) of this [InlineSpan]
  /// when this method returns.
  ///
  /// This method is typically for overriding only and should not be called
  /// directly. Use [updateAttributes] instead.
  @protected
  InlineSpan updateAttributesAtOffset(covariant InlineSpanAttributes newAttributes, TextRange textRange, Accumulator offset);

  /// Creates a new [InlineSpan] from this [InlineSpan] by applying
  /// `newAttributes` to the given [TextRange] within this span.
  ///
  /// This method calls [updateAttributesAtOffset] and overriding is rarely needed.
  @useResult
  InlineSpan updateAttributes(covariant InlineSpanAttributes newAttributes, TextRange textRange) => updateAttributesAtOffset(newAttributes, textRange, Accumulator());

  /// In debug mode, throws an exception if the object is not in a
  /// valid configuration. Otherwise, returns true.
  ///
  /// This is intended to be used as follows:
  ///
  /// ```dart
  /// assert(myInlineSpan.debugAssertIsValid());
  /// ```
  bool debugAssertIsValid() => true;

  /// Describe the difference between this span and another, in terms of
  /// how much damage it will make to the rendering. The comparison is deep.
  ///
  /// Comparing [InlineSpan] objects of different types, for example, comparing
  /// a [TextSpan] to a [WidgetSpan], always results in [RenderComparison.layout].
  ///
  /// See also:
  ///
  ///  * [TextStyle.compareTo], which does the same thing for [TextStyle]s.
  RenderComparison compareTo(InlineSpan other);

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is InlineSpan
        && other.style == style;
  }

  @override
  int get hashCode => style.hashCode;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.defaultDiagnosticsTreeStyle = DiagnosticsTreeStyle.whitespace;
    style?.debugFillProperties(properties);
  }
}

/// A text attribute set that can be used to update the attributes of an
/// [InlineSpan] subclass.
///
/// All attributes are nullable, and default to `null`. Attributes set to `null`
/// in an [InlineSpanAttributes] will not be updated.
///
/// See also:
///
///  * [InlineSpan.updateAttributes], which takes an [InlineSpanAttributes], and
///    updates a given [TextRange] within the receiver [InlineSpan] using the
///    [InlineSpanAttributes].
class InlineSpanAttributes {
  /// Creates an [InlineSpanAttributes].
  const InlineSpanAttributes({
    this.fontFamilies,
    this.locale,
    this.fontSize,
    this.fontWeight,
    this.fontStyle,
    this.fontFeatures,
    this.fontVariations,
    this.height,
    this.leadingDistribution,
    this.textBaseline,
    this.wordSpacing,
    this.letterSpacing,

    this.color,
    this.foreground,
    this.backgroundColor,
    this.background,
    this.shadows,
    this.underline,
    this.overline,
    this.lineThrough,
    this.decorationColor,
    this.decorationStyle,
    this.decorationThickness,

    this.recognizer,
    this.mouseCursor,
    this.onEnter,
    this.onExit,
    this.spellOut,
  }) : assert(color == null || foreground == null),
       assert(backgroundColor == null || background == null);

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.fontFamily] if
  /// set to non-null.
  final List<String>? fontFamilies;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.locale] and
  /// [TextSpan.locale] if set to non-null.
  final ui.Locale? locale;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.fontSize] if set
  /// to non-null.
  final double? fontSize;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.fontSize] if set
  /// to non-null.
  final ui.FontWeight? fontWeight;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.fontSize] if set
  /// to non-null.
  final ui.FontStyle? fontStyle;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.fontFeatures]
  /// if set to non-null.
  final List<ui.FontFeature>? fontFeatures;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.fontVariations]
  /// if set to non-null.
  final List<ui.FontVariation>? fontVariations;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.height] if set
  /// to non-null.
  ///
  /// Setting this attribute to [kTextHeightNone] unsets the [TextStyle.height]
  /// multiplier, and restores the font's natural ascent and descent.
  final double? height;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.leadingDistribution]
  /// if set to non-null.
  final ui.TextLeadingDistribution? leadingDistribution;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.textBaseline]
  /// if set to non-null.
  final ui.TextBaseline? textBaseline;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.wordSpacing]
  /// if set to non-null.
  final double? wordSpacing;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.letterSpacing]
  /// if set to non-null.
  final double? letterSpacing;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.color] if set to
  /// non-null.
  ///
  /// Must be null if [foreground] is set to a non-null value.
  final ui.Color? color;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.foreground] if
  /// set to non-null.
  ///
  /// Must be null if [color] is set to a non-null value.
  final ui.Paint? foreground;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.backgroundColor]
  /// if set to non-null.
  ///
  /// Must be null if [background] is set to a non-null value.
  final ui.Color? backgroundColor;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.background] if
  /// set to non-null.
  ///
  /// Must be null if [backgroundColor] is set to a non-null value.
  final ui.Paint? background;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.shadows] if set
  /// to non-null.
  final List<ui.Shadow>? shadows;

  /// An attribute which overwrites the [TextDecoration.underline] aspect of
  /// [TextStyle.decoration] on an [InlineSpan], if set to non-null.
  ///
  /// Setting this value to true applies underline and setting it to false
  /// disables underline.
  final bool? underline;

  /// An attribute which overwrites the [TextDecoration.overline] aspect of
  /// [TextStyle.decoration] on an [InlineSpan], if set to non-null.
  ///
  /// Setting this value to true applies overline and setting it to false
  /// disables overline.
  final bool? overline;

  /// An attribute which overwrites the [TextDecoration.lineThrough] aspect of
  /// [TextStyle.decoration] on an [InlineSpan], if set to non-null.
  ///
  /// Setting this value to true applies lineThrough and setting it to false
  /// disables lineThrough.
  final bool? lineThrough;

  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.decorationColor]
  /// if set to non-null.
  final ui.Color? decorationColor;
  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.decorationStyle]
  /// if set to non-null.
  final ui.TextDecorationStyle? decorationStyle;
  /// An attribute which overwrites an [InlineSpan]'s [TextStyle.decorationThickness]
  /// if set to non-null.
  final double? decorationThickness;

  /// An attribute which overwrites [TextSpan.recognizer] if set to non-null.
  ///
  /// When a recognizer is specified, the [InlineSpan.updateAttributes] method
  /// replaces existing recognizers (if any) within the given range with the
  /// specified recognizer.
  ///
  /// When this value is set to [remove], the [InlineSpan.updateAttributes]
  /// method sets [TextSpan.recognizer] to null in the given range of the returned
  /// [TextSpan].
  final GestureRecognizer? recognizer;

  /// An attribute which overwrites [TextSpan.mouseCursor] if set to non-null.
  final MouseCursor? mouseCursor;

  /// An attribute which overwrites [TextSpan.onEnter] if set to non-null.
  ///
  /// [TextSpan.onEnter] can not be set from a non-null value to null using this
  /// attribute. Considering setting [TextSpan.onEnter] to a function that does
  /// nothing if you want to unset the callback from a [TextSpan].
  final PointerEnterEventListener? onEnter;

  /// An attribute which overwrites [TextSpan.onExit] if set to non-null.
  ///
  /// [TextSpan.onExit] can not be set from a non-null value to null using this
  /// attribute. Considering setting [TextSpan.onExit] to a function that does
  /// nothing if you want to unset the callback from a [TextSpan].
  final PointerExitEventListener? onExit;

  /// An attribute which overwrites [TextSpan.spellOut] if set to non-null.
  final bool? spellOut;

  /// A sentinel values that can be used to unset the [TextSpan.recognizer] from
  /// an [TextSpan].
  ///
  /// This is useful when the value `null` already has a different meaning (other
  /// than "unset") in the context. In [InlineSpanAttributes], a null value means
  /// an attribute should not be updated. However, [TextSpan.recognizer] uses
  /// `null` to indicate there's no recognizer. The sentinel value introduces an
  /// additional state that indicates any existing [TextSpan.recognizer] should
  /// be unset in the new span.
  ///
  /// The only supported operation on this singleton object is the identity
  /// comparison function `identical`, performing any other operation on this
  /// object may crash the program.
  ///
  /// [InlineSpan.updateAttributes] does not mutate the target
  /// [InlineSpan]. Rather, when [recognizer] is set to [remove], that method
  /// returns a new [InlineSpan] with [recognizer] set to null.
  static const RemoveInlineSpanAttribute remove = _PoorMansBottomType._();

  /// A convenience method, similar to [TextStyle.merge], that merges the
  /// `textStyle` argument with the [TextStyle] related attributes in this
  /// [InlineSpanAttributes] object.
  @useResult
  TextStyle? updateTextStyle(TextStyle? textStyle) {
    final bool hasNoUpdate = fontFamilies == null
      && locale == null
      && fontSize == null
      && fontWeight == null
      && fontStyle == null
      && fontFeatures == null
      && fontVariations == null
      && height == null
      && leadingDistribution == null
      && textBaseline == null
      && wordSpacing == null
      && letterSpacing == null
      && color == null
      && foreground == null
      && background == null
      && backgroundColor == null
      && shadows == null
      && underline == null
      && overline == null
      && lineThrough == null
      && decorationColor == null
      && decorationStyle == null
      && decorationThickness == null;

    if (hasNoUpdate) {
      return textStyle;
    }
    final (String? fontFamily, List<String>? fallback) = switch (fontFamilies) {
      null => (null, null),
      [] => ('', const <String>[]),
      [final String fontFamily, ...final List<String> fallback] => (fontFamily, fallback)
    };
    final ui.TextDecoration? decoration = underline == null && overline == null && lineThrough == null
      ? null
      : ui.TextDecoration.combine(<ui.TextDecoration>[
          if (underline ?? textStyle?.decoration?.contains(ui.TextDecoration.underline) ?? false) ui.TextDecoration.underline,
          if (overline ?? textStyle?.decoration?.contains(ui.TextDecoration.overline) ?? false) ui.TextDecoration.overline,
          if (lineThrough ?? textStyle?.decoration?.contains(ui.TextDecoration.lineThrough) ?? false) ui.TextDecoration.lineThrough,
        ]);
    return (textStyle ?? const TextStyle()).copyWith(
      fontFamily: fontFamily,
      fontFamilyFallback: fallback,
      locale: locale,
      fontSize: fontSize,
      fontWeight: fontWeight,
      fontStyle: fontStyle,
      fontFeatures: fontFeatures,
      fontVariations: fontVariations,
      height: height,
      leadingDistribution: leadingDistribution,
      textBaseline: textBaseline,
      wordSpacing: wordSpacing,
      letterSpacing: letterSpacing,
      color: color,
      foreground: foreground,
      backgroundColor: backgroundColor,
      background: background,
      shadows: shadows,
      decoration: decoration,
      decorationColor: decorationColor,
      decorationStyle: decorationStyle,
      decorationThickness: decorationThickness,
    );
  }
}

final class _PoorMansBottomType with DiagnosticableTreeMixin implements GestureRecognizer {
  const _PoorMansBottomType._();
  @override
  Never noSuchMethod(Invocation invocation) => throw NoSuchMethodError.withInvocation(this, invocation);
}
