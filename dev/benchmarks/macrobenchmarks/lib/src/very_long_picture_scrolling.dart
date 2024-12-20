// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';
import 'dart:typed_data';
import 'dart:ui';
import 'package:flutter/material.dart';

// Adapted from test case submitted in
// https://github.com/flutter/flutter/issues/92366
// Converted to use fixed data rather than reading a waveform file
class VeryLongPictureScrollingPerf extends StatefulWidget {
  const VeryLongPictureScrollingPerf({super.key});

  @override
  State createState() => VeryLongPictureScrollingPerfState();
}

class VeryLongPictureScrollingPerfState extends State<VeryLongPictureScrollingPerf> {
  bool consolidate = false;
  bool useList = false;
  Int16List waveData = loadGraph();

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        actions: <Widget>[
          Row(
            children: <Widget>[
              const Text('list:'),
              Checkbox(
                value: useList,
                onChanged:
                    (bool? value) => setState(() {
                      useList = value!;
                    }),
              ),
            ],
          ),
          Row(
            children: <Widget>[
              const Text('consolidate:'),
              Checkbox(
                value: consolidate,
                onChanged:
                    (bool? value) => setState(() {
                      consolidate = value!;
                    }),
              ),
            ],
          ),
        ],
      ),
      backgroundColor: Colors.transparent,
      body: SizedBox(
        width: MediaQuery.of(context).size.width,
        height: MediaQuery.of(context).size.height,
        child:
            useList
                ? ListView.builder(
                  key: const ValueKey<String>('vlp_list_view_scrollable'),
                  scrollDirection: Axis.horizontal,
                  clipBehavior: Clip.none,
                  itemCount: (waveData.length / 200).ceil(),
                  itemExtent: 100,
                  itemBuilder:
                      (BuildContext context, int index) => CustomPaint(
                        painter: PaintSomeTest(
                          waveData: waveData,
                          from: index * 200,
                          to: min((index + 1) * 200, waveData.length - 1),
                        ),
                      ),
                )
                : SingleChildScrollView(
                  key: const ValueKey<String>('vlp_single_child_scrollable'),
                  scrollDirection: Axis.horizontal,
                  child: SizedBox(
                    width: MediaQuery.of(context).size.width * 20,
                    height: MediaQuery.of(context).size.height,
                    child: RepaintBoundary(
                      child: CustomPaint(
                        isComplex: true,
                        painter: PaintTest(consolidate: consolidate, waveData: waveData),
                      ),
                    ),
                  ),
                ),
      ),
    );
  }
}

class PaintTest extends CustomPainter {
  const PaintTest({required this.consolidate, required this.waveData});

  final bool consolidate;
  final Int16List waveData;

  @override
  void paint(Canvas canvas, Size size) {
    final double halfHeight = size.height / 2;
    double x = 0;
    const double strokeSize = .5;
    const double zoomFactor = .5;

    final Paint paintPos =
        Paint()
          ..color = Colors.pink
          ..strokeWidth = strokeSize
          ..isAntiAlias = false
          ..style = PaintingStyle.stroke;

    final Paint paintNeg =
        Paint()
          ..color = Colors.pink
          ..strokeWidth = strokeSize
          ..isAntiAlias = false
          ..style = PaintingStyle.stroke;

    final Paint paintZero =
        Paint()
          ..color = Colors.green
          ..strokeWidth = strokeSize
          ..isAntiAlias = false
          ..style = PaintingStyle.stroke;

    int index = 0;
    Paint? listPaint;
    final Float32List offsets = Float32List(consolidate ? waveData.length * 4 : 4);
    int used = 0;
    for (index = 0; index < waveData.length; index++) {
      final (Paint curPaint, Offset p1) = switch (waveData[index]) {
        < 0 => (paintPos, Offset(x, halfHeight * (1 - waveData[index] / 32768))),
        > 0 => (paintNeg, Offset(x, halfHeight * (1 - waveData[index] / 32767))),
        _ => (paintZero, Offset(x, halfHeight + 1)),
      };
      final Offset p0 = Offset(x, halfHeight);
      if (consolidate) {
        if (listPaint != null && listPaint != curPaint) {
          canvas.drawRawPoints(PointMode.lines, offsets.sublist(0, used), listPaint);
          used = 0;
        }
        listPaint = curPaint;
        offsets[used++] = p0.dx;
        offsets[used++] = p0.dy;
        offsets[used++] = p1.dx;
        offsets[used++] = p1.dy;
      } else {
        canvas.drawLine(p0, p1, curPaint);
      }
      x += zoomFactor;
    }
    if (consolidate && used > 0) {
      canvas.drawRawPoints(PointMode.lines, offsets.sublist(0, used), listPaint!);
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) {
    return oldDelegate is! PaintTest ||
        oldDelegate.consolidate != consolidate ||
        oldDelegate.waveData != waveData;
  }
}

class PaintSomeTest extends CustomPainter {
  const PaintSomeTest({required this.waveData, int? from, int? to})
    : from = from ?? 0,
      to = to ?? waveData.length;

  final Int16List waveData;
  final int from;
  final int to;

  @override
  void paint(Canvas canvas, Size size) {
    final double halfHeight = size.height / 2;
    double x = 0;
    const double strokeSize = .5;
    const double zoomFactor = .5;

    final Paint paintPos =
        Paint()
          ..color = Colors.pink
          ..strokeWidth = strokeSize
          ..isAntiAlias = false
          ..style = PaintingStyle.stroke;

    final Paint paintNeg =
        Paint()
          ..color = Colors.pink
          ..strokeWidth = strokeSize
          ..isAntiAlias = false
          ..style = PaintingStyle.stroke;

    final Paint paintZero =
        Paint()
          ..color = Colors.green
          ..strokeWidth = strokeSize
          ..isAntiAlias = false
          ..style = PaintingStyle.stroke;

    for (int index = from; index <= to; index++) {
      final (Paint curPaint, Offset p1) = switch (waveData[index]) {
        < 0 => (paintPos, Offset(x, halfHeight * (1 - waveData[index] / 32768))),
        > 0 => (paintNeg, Offset(x, halfHeight * (1 - waveData[index] / 32767))),
        _ => (paintZero, Offset(x, halfHeight + 1)),
      };
      final Offset p0 = Offset(x, halfHeight);
      canvas.drawLine(p0, p1, curPaint);
      x += zoomFactor;
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) {
    return oldDelegate is! PaintSomeTest ||
        oldDelegate.waveData != waveData ||
        oldDelegate.from != from ||
        oldDelegate.to != to;
  }
}

Int16List loadGraph() {
  final Int16List waveData = Int16List(350000);
  final Random r = Random(0x42);
  for (int i = 0; i < waveData.length; i++) {
    waveData[i] = r.nextInt(32768) - 16384;
  }
  return waveData;
}
