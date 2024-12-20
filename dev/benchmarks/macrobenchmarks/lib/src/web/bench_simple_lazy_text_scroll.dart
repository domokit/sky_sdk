// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/widgets.dart';

import 'recorder.dart';
import 'test_data.dart';

/// Creates several list views containing text items, then continuously scrolls
/// them up and down.
///
/// Measures our ability to lazily render virtually infinitely big content.
class BenchSimpleLazyTextScroll extends WidgetRecorder {
  BenchSimpleLazyTextScroll() : super(name: benchmarkName);

  static const String benchmarkName = 'bench_simple_lazy_text_scroll';

  @override
  Widget createWidget() {
    return const Directionality(
      textDirection: TextDirection.ltr,
      child: Row(
        children: <Widget>[
          Flexible(
            child: _TestScrollingWidget(
              initialScrollOffset: 0,
              scrollDistance: 300,
              scrollDuration: Duration(seconds: 1),
            ),
          ),
          Flexible(
            child: _TestScrollingWidget(
              initialScrollOffset: 1000,
              scrollDistance: 500,
              scrollDuration: Duration(milliseconds: 1500),
            ),
          ),
          Flexible(
            child: _TestScrollingWidget(
              initialScrollOffset: 2000,
              scrollDistance: 700,
              scrollDuration: Duration(milliseconds: 2000),
            ),
          ),
        ],
      ),
    );
  }
}

class _TestScrollingWidget extends StatefulWidget {
  const _TestScrollingWidget({
    required this.initialScrollOffset,
    required this.scrollDistance,
    required this.scrollDuration,
  });

  final double initialScrollOffset;
  final double scrollDistance;
  final Duration scrollDuration;

  @override
  State<StatefulWidget> createState() {
    return _TestScrollingWidgetState();
  }
}

class _TestScrollingWidgetState extends State<_TestScrollingWidget> {
  late ScrollController scrollController;

  @override
  void initState() {
    super.initState();

    scrollController = ScrollController(initialScrollOffset: widget.initialScrollOffset);

    // Without the timer the animation doesn't begin.
    Timer.run(() async {
      bool forward = true;
      while (true) {
        await scrollController.animateTo(
          forward ? widget.initialScrollOffset + widget.scrollDistance : widget.initialScrollOffset,
          curve: Curves.linear,
          duration: widget.scrollDuration,
        );
        forward = !forward;
      }
    });
  }

  @override
  void dispose() {
    scrollController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return ListView.builder(
      controller: scrollController,
      itemCount: 10000,
      itemBuilder: (BuildContext context, int index) {
        return Text(lipsum[index % lipsum.length]);
      },
    );
  }
}
