// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';

import 'recorder.dart';

/// Creates a [PageView] that uses a font style that can't be rendered
/// using canvas (switching to DOM).
///
/// Since the whole page uses a CustomPainter this is a good representation
/// for apps that have pictures with large number of painting commands.
class BenchPageViewScrollLineThrough extends WidgetRecorder {
  BenchPageViewScrollLineThrough() : super(name: benchmarkName);

  static const String benchmarkName = 'bench_page_view_scroll_line_through';

  @override
  Widget createWidget() =>
      const MaterialApp(title: 'PageView Scroll LineThrough Benchmark', home: _MyScrollContainer());
}

class _MyScrollContainer extends StatefulWidget {
  const _MyScrollContainer();

  @override
  State<_MyScrollContainer> createState() => _MyScrollContainerState();
}

class _MyScrollContainerState extends State<_MyScrollContainer> {
  static const Duration stepDuration = Duration(milliseconds: 500);

  late PageController pageController;
  final _CustomPainter _painter = _CustomPainter('aa');
  int pageNumber = 0;

  @override
  void initState() {
    super.initState();

    pageController = PageController();

    // Without the timer the animation doesn't begin.
    Timer.run(() async {
      while (pageNumber < 25) {
        await pageController.animateToPage(
          pageNumber % 5,
          duration: stepDuration,
          curve: Curves.easeInOut,
        );
        pageNumber++;
      }
    });
  }

  @override
  void dispose() {
    pageController.dispose();
    _painter._textPainter.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return PageView.builder(
      controller: pageController,
      itemBuilder: (BuildContext context, int position) {
        return CustomPaint(painter: _painter, size: const Size(300, 500));
      },
    );
  }
}

class _CustomPainter extends CustomPainter {
  _CustomPainter(this.text);

  final String text;
  final Paint _linePainter = Paint();
  final TextPainter _textPainter = TextPainter();
  static const double lineWidth = 0.5;

  @override
  void paint(Canvas canvas, Size size) {
    canvas.clipRect(Rect.fromLTWH(0, 0, size.width, size.height));
    double xPosition, yPosition;
    final double width = size.width / 7;
    final double height = size.height / 6;
    xPosition = 0;
    const double viewPadding = 5;
    const double circlePadding = 4;
    yPosition = viewPadding;
    _textPainter.textDirection = TextDirection.ltr;
    _textPainter.textWidthBasis = TextWidthBasis.longestLine;
    _textPainter.textScaler = TextScaler.noScaling;
    const TextStyle textStyle = TextStyle(
      color: Colors.black87,
      fontSize: 13,
      fontFamily: 'Roboto',
    );

    _linePainter.isAntiAlias = true;
    for (int i = 0; i < 42; i++) {
      _linePainter.color = Colors.white;

      TextStyle temp = textStyle;
      if (i % 7 == 0) {
        temp = textStyle.copyWith(decoration: TextDecoration.lineThrough);
      }

      final TextSpan span = TextSpan(text: text, style: temp);

      _textPainter.text = span;

      _textPainter.layout(maxWidth: width);
      _linePainter.style = PaintingStyle.fill;
      canvas.drawRect(
        Rect.fromLTWH(xPosition, yPosition - viewPadding, width, height),
        _linePainter,
      );

      _textPainter.paint(
        canvas,
        Offset(xPosition + (width / 2 - _textPainter.width / 2), yPosition + circlePadding),
      );
      xPosition += width;
      if (xPosition.round() >= size.width.round()) {
        xPosition = 0;
        yPosition += height;
      }
    }

    _drawVerticalAndHorizontalLines(canvas, size, yPosition, xPosition, height, width);
  }

  void _drawVerticalAndHorizontalLines(
    Canvas canvas,
    Size size,
    double yPosition,
    double xPosition,
    double height,
    double width,
  ) {
    yPosition = height;
    _linePainter.strokeWidth = lineWidth;
    _linePainter.color = Colors.grey;
    canvas.drawLine(const Offset(0, lineWidth), Offset(size.width, lineWidth), _linePainter);
    for (int i = 0; i < 6; i++) {
      canvas.drawLine(Offset(0, yPosition), Offset(size.width, yPosition), _linePainter);
      yPosition += height;
    }

    canvas.drawLine(
      Offset(0, size.height - lineWidth),
      Offset(size.width, size.height - lineWidth),
      _linePainter,
    );
    xPosition = width;
    canvas.drawLine(const Offset(lineWidth, 0), Offset(lineWidth, size.height), _linePainter);
    for (int i = 0; i < 6; i++) {
      canvas.drawLine(Offset(xPosition, 0), Offset(xPosition, size.height), _linePainter);
      xPosition += width;
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) {
    return true;
  }
}
