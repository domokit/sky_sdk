// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'editable_text.dart';
/// @docImport 'text.dart';
library;

import 'dart:ui' as ui show ParagraphBuilder, PlaceholderAlignment;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'framework.dart';

// Examples can assume:
// late WidgetSpan myWidgetSpan;

/// An immutable widget that is embedded inline within text.
///
/// The [child] property is the widget that will be embedded. Children are
/// constrained by the width of the paragraph.
///
/// The [child] property may contain its own [Widget] children (if applicable),
/// including [Text] and [RichText] widgets which may include additional
/// [WidgetSpan]s. Child [Text] and [RichText] widgets will be laid out
/// independently and occupy a rectangular space in the parent text layout.
///
/// [WidgetSpan]s will be ignored when passed into a [TextPainter] directly.
/// To properly layout and paint the [child] widget, [WidgetSpan] should be
/// passed into a [Text.rich] widget.
///
/// {@tool snippet}
///
/// A card with `Hello World!` embedded inline within a TextSpan tree.
///
/// ```dart
/// const Text.rich(
///   TextSpan(
///     children: <InlineSpan>[
///       TextSpan(text: 'Flutter is'),
///       WidgetSpan(
///         child: SizedBox(
///           width: 120,
///           height: 50,
///           child: Card(
///             child: Center(
///               child: Text('Hello World!')
///             )
///           ),
///         )
///       ),
///       TextSpan(text: 'the best!'),
///     ],
///   )
/// )
/// ```
/// {@end-tool}
///
/// [WidgetSpan] contributes the semantics of the [WidgetSpan.child] to the
/// semantics tree.
///
/// See also:
///
///  * [TextSpan], a node that represents text in an [InlineSpan] tree.
///  * [Text], a widget for showing uniformly-styled text.
///  * [RichText], a widget for finer control of text rendering.
///  * [TextPainter], a class for painting [InlineSpan] objects on a [Canvas].
@immutable
class WidgetSpan extends PlaceholderSpan {
  /// Creates a [WidgetSpan] with the given values.
  ///
  /// [WidgetSpan] is a leaf node in the [InlineSpan] tree. Child widgets are
  /// constrained by the width of the paragraph they occupy. Child widget
  /// heights are unconstrained, and may cause the text to overflow and be
  /// ellipsized/truncated.
  ///
  /// A [TextStyle] may be provided with the [style] property, but only the
  /// decoration, foreground, background, and spacing options will be used.
  const WidgetSpan({required this.child, super.alignment, super.baseline, super.style})
    : assert(
        baseline != null ||
            !(identical(alignment, ui.PlaceholderAlignment.aboveBaseline) ||
                identical(alignment, ui.PlaceholderAlignment.belowBaseline) ||
                identical(alignment, ui.PlaceholderAlignment.baseline)),
      );

  /// Helper function for extracting [WidgetSpan]s in preorder, from the given
  /// [InlineSpan] as a list of widgets.
  ///
  /// The `textScaler` is the scaling strategy for scaling the content.
  ///
  /// This function is used by [EditableText] and [RichText] so calling it
  /// directly is rarely necessary.
  static List<Widget> extractFromInlineSpan(InlineSpan span, TextScaler textScaler) {
    final List<Widget> widgets = <Widget>[];
    // _kEngineDefaultFontSize is the default font size to use when none of the
    // ancestor spans specifies one.
    final List<double> fontSizeStack = <double>[kDefaultFontSize];
    int index = 0;
    // This assumes an InlineSpan tree's logical order is equivalent to preorder.
    bool visitSubtree(InlineSpan span) {
      final double? fontSizeToPush = switch (span.style?.fontSize) {
        final double size when size != fontSizeStack.last => size,
        _ => null,
      };
      if (fontSizeToPush != null) {
        fontSizeStack.add(fontSizeToPush);
      }
      if (span is WidgetSpan) {
        final double fontSize = fontSizeStack.last;
        final double textScaleFactor = fontSize == 0 ? 0 : textScaler.scale(fontSize) / fontSize;
        widgets.add(
          _WidgetSpanParentData(
            span: span,
            child: Semantics(
              tagForChildren: PlaceholderSpanIndexSemanticsTag(index++),
              child: _AutoScaleInlineWidget(
                span: span,
                textScaleFactor: textScaleFactor,
                child: span.child,
              ),
            ),
          ),
        );
      }
      assert(
        span is WidgetSpan || span is! PlaceholderSpan,
        '$span is a PlaceholderSpan but not a WidgetSpan subclass. This is currently not supported.',
      );
      span.visitDirectChildren(visitSubtree);
      if (fontSizeToPush != null) {
        final double poppedFontSize = fontSizeStack.removeLast();
        assert(fontSizeStack.isNotEmpty);
        assert(poppedFontSize == fontSizeToPush);
      }
      return true;
    }

    visitSubtree(span);
    return widgets;
  }

  /// The widget to embed inline within text.
  final Widget child;

  /// Adds a placeholder box to the paragraph builder if a size has been
  /// calculated for the widget.
  ///
  /// Sizes are provided through `dimensions`, which should contain a 1:1
  /// in-order mapping of widget to laid-out dimensions. If no such dimension
  /// is provided, the widget will be skipped.
  ///
  /// The `textScaler` will be applied to the laid-out size of the widget.
  @override
  void build(
    ui.ParagraphBuilder builder, {
    TextScaler textScaler = TextScaler.noScaling,
    List<PlaceholderDimensions>? dimensions,
  }) {
    assert(debugAssertIsValid());
    assert(dimensions != null);
    final bool hasStyle = style != null;
    if (hasStyle) {
      builder.pushStyle(style!.getTextStyle(textScaler: textScaler));
    }
    assert(builder.placeholderCount < dimensions!.length);
    final PlaceholderDimensions currentDimensions = dimensions![builder.placeholderCount];
    builder.addPlaceholder(
      currentDimensions.size.width,
      currentDimensions.size.height,
      alignment,
      baseline: currentDimensions.baseline,
      baselineOffset: currentDimensions.baselineOffset,
    );
    if (hasStyle) {
      builder.pop();
    }
  }

  /// Calls `visitor` on this [WidgetSpan]. There are no children spans to walk.
  @override
  bool visitChildren(InlineSpanVisitor visitor) => visitor(this);

  @override
  bool visitDirectChildren(InlineSpanVisitor visitor) => true;

  @override
  InlineSpan? getSpanForPositionVisitor(TextPosition position, Accumulator offset) {
    if (position.offset == offset.value) {
      return this;
    }
    offset.increment(1);
    return null;
  }

  @override
  int? codeUnitAtVisitor(int index, Accumulator offset) {
    final int localOffset = index - offset.value;
    assert(localOffset >= 0);
    offset.increment(1);
    return localOffset == 0 ? PlaceholderSpan.placeholderCodeUnit : null;
  }

  @override
  RenderComparison compareTo(InlineSpan other) {
    if (identical(this, other)) {
      return RenderComparison.identical;
    }
    if (other.runtimeType != runtimeType) {
      return RenderComparison.layout;
    }
    if ((style == null) != (other.style == null)) {
      return RenderComparison.layout;
    }
    final WidgetSpan typedOther = other as WidgetSpan;
    if (child != typedOther.child || alignment != typedOther.alignment) {
      return RenderComparison.layout;
    }
    RenderComparison result = RenderComparison.identical;
    if (style != null) {
      final RenderComparison candidate = style!.compareTo(other.style!);
      if (candidate.index > result.index) {
        result = candidate;
      }
      if (result == RenderComparison.layout) {
        return result;
      }
    }
    return result;
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    if (super != other) {
      return false;
    }
    return other is WidgetSpan &&
        other.child == child &&
        other.alignment == alignment &&
        other.baseline == baseline;
  }

  @override
  int get hashCode => Object.hash(super.hashCode, child, alignment, baseline);

  /// Returns the text span that contains the given position in the text.
  @override
  InlineSpan? getSpanForPosition(TextPosition position) {
    assert(debugAssertIsValid());
    return null;
  }

  /// In debug mode, throws an exception if the object is not in a
  /// valid configuration. Otherwise, returns true.
  ///
  /// This is intended to be used as follows:
  ///
  /// ```dart
  /// assert(myWidgetSpan.debugAssertIsValid());
  /// ```
  @override
  bool debugAssertIsValid() {
    // WidgetSpans are always valid as asserts prevent invalid WidgetSpans
    // from being constructed.
    return true;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Widget>('widget', child));
  }
}

// A ParentDataWidget that sets TextParentData.span.
class _WidgetSpanParentData extends ParentDataWidget<TextParentData> {
  const _WidgetSpanParentData({required this.span, required super.child});

  final WidgetSpan span;

  @override
  void applyParentData(RenderObject renderObject) {
    final TextParentData parentData = renderObject.parentData! as TextParentData;
    parentData.span = span;
  }

  @override
  Type get debugTypicalAncestorWidgetClass => RichText;
}

// A RenderObjectWidget that automatically applies text scaling on inline
// widgets.
//
// TODO(LongCatIsLooong): this shouldn't happen automatically, at least there
// should be a way to opt out: https://github.com/flutter/flutter/issues/126962
class _AutoScaleInlineWidget extends SingleChildRenderObjectWidget {
  const _AutoScaleInlineWidget({
    required this.span,
    required this.textScaleFactor,
    required super.child,
  });

  final WidgetSpan span;
  final double textScaleFactor;

  @override
  _RenderScaledInlineWidget createRenderObject(BuildContext context) {
    return _RenderScaledInlineWidget(span.alignment, span.baseline, textScaleFactor);
  }

  @override
  void updateRenderObject(BuildContext context, _RenderScaledInlineWidget renderObject) {
    renderObject
      ..alignment = span.alignment
      ..baseline = span.baseline
      ..scale = textScaleFactor;
  }
}

class _RenderScaledInlineWidget extends RenderBox with RenderObjectWithChildMixin<RenderBox> {
  _RenderScaledInlineWidget(this._alignment, this._baseline, this._scale);

  double get scale => _scale;
  double _scale;
  set scale(double value) {
    if (value == _scale) {
      return;
    }
    assert(value > 0);
    assert(value.isFinite);
    _scale = value;
    markNeedsLayout();
  }

  ui.PlaceholderAlignment get alignment => _alignment;
  ui.PlaceholderAlignment _alignment;
  set alignment(ui.PlaceholderAlignment value) {
    if (_alignment == value) {
      return;
    }
    _alignment = value;
    markNeedsLayout();
  }

  TextBaseline? get baseline => _baseline;
  TextBaseline? _baseline;
  set baseline(TextBaseline? value) {
    if (value == _baseline) {
      return;
    }
    _baseline = value;
    markNeedsLayout();
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return (child?.getMaxIntrinsicHeight(width / scale) ?? 0.0) * scale;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return (child?.getMaxIntrinsicWidth(height / scale) ?? 0.0) * scale;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return (child?.getMinIntrinsicHeight(width / scale) ?? 0.0) * scale;
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    return (child?.getMinIntrinsicWidth(height / scale) ?? 0.0) * scale;
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    return switch (child?.getDistanceToActualBaseline(baseline)) {
      null => super.computeDistanceToActualBaseline(baseline),
      final double childBaseline => scale * childBaseline,
    };
  }

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    final double? distance = child?.getDryBaseline(
      BoxConstraints(maxWidth: constraints.maxWidth / scale),
      baseline,
    );
    return distance == null ? null : scale * distance;
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    assert(!constraints.hasBoundedHeight);
    final Size unscaledSize =
        child?.getDryLayout(BoxConstraints(maxWidth: constraints.maxWidth / scale)) ?? Size.zero;
    return constraints.constrain(unscaledSize * scale);
  }

  @override
  void performLayout() {
    final RenderBox? child = this.child;
    if (child == null) {
      return;
    }
    assert(!constraints.hasBoundedHeight);
    // Only constrain the width to the maximum width of the paragraph.
    // Leave height unconstrained, which will overflow if expanded past.
    child.layout(BoxConstraints(maxWidth: constraints.maxWidth / scale), parentUsesSize: true);
    size = constraints.constrain(child.size * scale);
  }

  @override
  void applyPaintTransform(RenderBox child, Matrix4 transform) {
    transform.scale(scale, scale);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    final RenderBox? child = this.child;
    if (child == null) {
      layer = null;
      return;
    }
    if (scale == 1.0) {
      context.paintChild(child, offset);
      layer = null;
      return;
    }
    layer = context.pushTransform(
      needsCompositing,
      offset,
      Matrix4.diagonal3Values(scale, scale, 1.0),
      (PaintingContext context, Offset offset) => context.paintChild(child, offset),
      oldLayer: layer as TransformLayer?,
    );
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    final RenderBox? child = this.child;
    if (child == null) {
      return false;
    }
    return result.addWithPaintTransform(
      transform: Matrix4.diagonal3Values(scale, scale, 1.0),
      position: position,
      hitTest:
          (BoxHitTestResult result, Offset transformedOffset) =>
              child.hitTest(result, position: transformedOffset),
    );
  }
}
