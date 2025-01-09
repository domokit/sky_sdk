// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../html/bitmap_canvas.dart';
import '../html/painting.dart';
import 'canvas_paragraph.dart';
import 'layout_fragmenter.dart';
import 'paragraph.dart';

/// Responsible for painting a [CanvasParagraph] on a [BitmapCanvas].
class TextPaintService {
  TextPaintService(this.paragraph);

  final CanvasParagraph paragraph;

  void paint(BitmapCanvas canvas, ui.Offset offset) {
    // Loop through all the lines, for each line, loop through all fragments and
    // paint them. The fragment objects have enough information to be painted
    // individually.
    final List<ParagraphLine> lines = paragraph.lines;

    for (final ParagraphLine line in lines) {
      for (final LayoutFragment fragment in line.fragments) {
        _paintBackground(canvas, offset, fragment);
        _paintText(canvas, offset, line, fragment);
      }
    }
  }

  void _paintBackground(BitmapCanvas canvas, ui.Offset offset, LayoutFragment fragment) {
    if (fragment.isPlaceholder) {
      return;
    }

    // Paint the background of the box, if the span has a background.
    final SurfacePaint? background = fragment.style.background as SurfacePaint?;
    if (background != null) {
      final ui.Rect rect = fragment.toPaintingTextBox().toRect();
      if (!rect.isEmpty) {
        canvas.drawRect(rect.shift(offset), background.paintData);
      }
    }
  }

  void _paintText(
    BitmapCanvas canvas,
    ui.Offset offset,
    ParagraphLine line,
    LayoutFragment fragment,
  ) {
    // There's no text to paint in placeholder spans.
    if (fragment.isPlaceholder) {
      return;
    }

    // Don't paint the text for space-only boxes. This is just an
    // optimization, it doesn't have any effect on the output.
    if (fragment.isSpaceOnly) {
      return;
    }

    _prepareCanvasForFragment(canvas, fragment);
    final double fragmentX =
        fragment.textDirection! == ui.TextDirection.ltr ? fragment.left : fragment.right;

    final double x = offset.dx + line.left + fragmentX;
    final double y = offset.dy + line.baseline;

    final EngineTextStyle style = fragment.style;

    final String text = fragment.getText(paragraph);
    canvas.drawText(text, x, y, style: style.foreground?.style, shadows: style.shadows);

    canvas.tearDownPaint();
  }

  void _prepareCanvasForFragment(BitmapCanvas canvas, LayoutFragment fragment) {
    final EngineTextStyle style = fragment.style;

    final SurfacePaint? paint;
    final ui.Paint? foreground = style.foreground;
    if (foreground != null) {
      paint = foreground as SurfacePaint;
    } else {
      paint = ui.Paint() as SurfacePaint;
      if (style.color != null) {
        paint.color = style.color!;
      }
    }

    canvas.setCssFont(style.cssFontString, fragment.textDirection!);
    canvas.setUpPaint(paint.paintData, null);
  }
}
