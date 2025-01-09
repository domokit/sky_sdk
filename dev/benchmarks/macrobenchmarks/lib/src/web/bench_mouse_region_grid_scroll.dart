// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui';

import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'recorder.dart';

/// Creates a grid of mouse regions, then continuously scrolls them up and down.
///
/// Measures our ability to render mouse regions.
class BenchMouseRegionGridScroll extends WidgetRecorder {
  BenchMouseRegionGridScroll() : super(name: benchmarkName);

  static const String benchmarkName = 'bench_mouse_region_grid_scroll';

  final _Tester _tester = _Tester();

  // Use a non-trivial border to force Web to switch painter
  Border _getBorder(int columnIndex, int rowIndex) {
    const BorderSide defaultBorderSide = BorderSide();

    return Border(
      left: columnIndex == 0 ? defaultBorderSide : BorderSide.none,
      top: rowIndex == 0 ? defaultBorderSide : BorderSide.none,
      right: defaultBorderSide,
      bottom: defaultBorderSide,
    );
  }

  bool started = false;

  @override
  void frameDidDraw() {
    if (!started) {
      started = true;
      SchedulerBinding.instance.addPostFrameCallback((Duration timeStamp) async {
        _tester.start();
        registerDidStop(_tester.stop);
      });
    }
    super.frameDidDraw();
  }

  @override
  Widget createWidget() {
    const int rowsCount = 60;
    const int columnsCount = 20;
    const double containerSize = 20;
    return Directionality(
      textDirection: TextDirection.ltr,
      child: Align(
        alignment: Alignment.topLeft,
        child: SizedBox(
          width: 400,
          height: 400,
          child: ListView.builder(
            itemCount: rowsCount,
            cacheExtent: rowsCount * containerSize,
            physics: const ClampingScrollPhysics(),
            itemBuilder:
                (BuildContext context, int rowIndex) => Row(
                  children: List<Widget>.generate(
                    columnsCount,
                    (int columnIndex) => MouseRegion(
                      onEnter: (_) {},
                      child: Container(
                        decoration: BoxDecoration(
                          border: _getBorder(columnIndex, rowIndex),
                          color: Color.fromARGB(255, rowIndex * 20 % 256, 127, 127),
                        ),
                        width: containerSize,
                        height: containerSize,
                      ),
                    ),
                  ),
                ),
          ),
        ),
      ),
    );
  }
}

abstract final class _UntilNextFrame {
  static Completer<void>? _completer;

  static Future<void> wait() {
    if (_UntilNextFrame._completer == null) {
      _UntilNextFrame._completer = Completer<void>();
      SchedulerBinding.instance.addPostFrameCallback((_) {
        _UntilNextFrame._completer!.complete();
        _UntilNextFrame._completer = null;
      });
    }
    return _UntilNextFrame._completer!.future;
  }
}

class _Tester {
  static const int scrollFrequency = 60;
  static const Offset dragStartLocation = Offset(200, 200);
  static const Offset dragUpOffset = Offset(0, 200);
  static const Offset dragDownOffset = Offset(0, -200);
  static const Duration dragDuration = Duration(milliseconds: 200);

  bool _stopped = false;

  TestGesture get gesture {
    return _gesture ??= TestGesture(
      dispatcher: (PointerEvent event) async {
        RendererBinding.instance.handlePointerEvent(event);
      },
      kind: PointerDeviceKind.mouse,
    );
  }

  TestGesture? _gesture;

  Duration currentTime = Duration.zero;

  Future<void> _scroll(Offset start, Offset offset, Duration duration) async {
    final int durationMs = duration.inMilliseconds;
    final Duration fullFrameDuration = const Duration(seconds: 1) ~/ scrollFrequency;
    final int frameDurationMs = fullFrameDuration.inMilliseconds;

    final int fullFrames = duration.inMilliseconds ~/ frameDurationMs;
    final Offset fullFrameOffset = offset * (frameDurationMs.toDouble() / durationMs);

    final Duration finalFrameDuration = duration - fullFrameDuration * fullFrames;
    final Offset finalFrameOffset = offset - fullFrameOffset * fullFrames.toDouble();

    await gesture.down(start, timeStamp: currentTime);

    for (int frame = 0; frame < fullFrames; frame += 1) {
      currentTime += fullFrameDuration;
      await gesture.moveBy(fullFrameOffset, timeStamp: currentTime);
      await _UntilNextFrame.wait();
    }

    if (finalFrameOffset != Offset.zero) {
      currentTime += finalFrameDuration;
      await gesture.moveBy(finalFrameOffset, timeStamp: currentTime);
      await _UntilNextFrame.wait();
    }

    await gesture.up(timeStamp: currentTime);
  }

  Future<void> start() async {
    await Future<void>.delayed(Duration.zero);
    while (!_stopped) {
      await _scroll(dragStartLocation, dragUpOffset, dragDuration);
      await _scroll(dragStartLocation, dragDownOffset, dragDuration);
    }
  }

  void stop() {
    _stopped = true;
  }
}
