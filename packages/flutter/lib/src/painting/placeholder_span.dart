// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:ui' as ui show PlaceholderAlignment;

import 'package:flutter/foundation.dart';

import 'basic_types.dart';
import 'inline_span.dart';
import 'text_painter.dart';
import 'text_span.dart';
import 'text_style.dart';

/// An immutable placeholder that is embedded inline within text.
///
/// [PlaceholderSpan] represents a placeholder that acts as a stand-in for other
/// content. A [PlaceholderSpan] by itself does not contain useful information
/// to change a [TextSpan]. [WidgetSpan] from the widgets library extends
/// [PlaceholderSpan] and may be used instead to specify a widget as the contents
/// of the placeholder.
///
/// Flutter widgets such as [TextField], [Text] and [RichText] do not recognize
/// [PlaceholderSpan] subclasses other than [WidgetSpan]. **Consider
/// implementing the [WidgetSpan] interface instead of the [Placeholder]
/// interface.**
///
///
/// See also:
///
///  * [WidgetSpan], a leaf node that represents an embedded inline widget.
///  * [TextSpan], a node that represents text in a [TextSpan] tree.
///  * [Text], a widget for showing uniformly-styled text.
///  * [RichText], a widget for finer control of text rendering.
///  * [TextPainter], a class for painting [TextSpan] objects on a [Canvas].
abstract class PlaceholderSpan extends InlineSpan {
  /// Creates a [PlaceholderSpan] with the given values.
  ///
  /// A [TextStyle] may be provided with the [style] property, but only the
  /// decoration, foreground, background, and spacing options will be used.
  const PlaceholderSpan({
    this.alignment = ui.PlaceholderAlignment.bottom,
    this.baseline,
    super.style,
  });

  /// The unicode character to represent a placeholder.
  static const int placeholderCodeUnit = 0xFFFC;

  /// How the placeholder aligns vertically with the text.
  ///
  /// See [ui.PlaceholderAlignment] for details on each mode.
  final ui.PlaceholderAlignment alignment;

  /// The [TextBaseline] to align against when using [ui.PlaceholderAlignment.baseline],
  /// [ui.PlaceholderAlignment.aboveBaseline], and [ui.PlaceholderAlignment.belowBaseline].
  ///
  /// This is ignored when using other alignment modes.
  final TextBaseline? baseline;

  /// [PlaceholderSpan]s are flattened to a `0xFFFC` object replacement character in the
  /// plain text representation when `includePlaceholders` is true.
  @override
  void computeToPlainText(
    StringBuffer buffer, {
    bool includeSemanticsLabels = true,
    bool includePlaceholders = true,
  }) {
    if (includePlaceholders) {
      buffer.writeCharCode(placeholderCodeUnit);
    }
  }

  @override
  void computeSemanticsInformation(List<InlineSpanSemanticsInformation> collector) {
    collector.add(InlineSpanSemanticsInformation.placeholder);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);

    properties.add(
      EnumProperty<ui.PlaceholderAlignment>('alignment', alignment, defaultValue: null),
    );
    properties.add(EnumProperty<TextBaseline>('baseline', baseline, defaultValue: null));
  }

  @override
  bool debugAssertIsValid() {
    assert(false, 'Consider implementing the WidgetSpan interface instead.');
    return super.debugAssertIsValid();
  }
}
