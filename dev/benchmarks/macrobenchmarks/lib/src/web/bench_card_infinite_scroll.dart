// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';

import 'recorder.dart';
import 'test_data.dart';

/// Creates an infinite list of Material cards and scrolls it.
class BenchCardInfiniteScroll extends WidgetRecorder {
  BenchCardInfiniteScroll.forward()
    : initialOffset = 0.0,
      finalOffset = 30000.0,
      super(name: benchmarkName);

  BenchCardInfiniteScroll.backward()
    : initialOffset = 30000.0,
      finalOffset = 0.0,
      super(name: benchmarkNameBackward);

  static const String benchmarkName = 'bench_card_infinite_scroll';
  static const String benchmarkNameBackward = 'bench_card_infinite_scroll_backward';

  final double initialOffset;
  final double finalOffset;

  @override
  Widget createWidget() => MaterialApp(
    title: 'Infinite Card Scroll Benchmark',
    home: _InfiniteScrollCards(initialOffset, finalOffset),
  );
}

class _InfiniteScrollCards extends StatefulWidget {
  const _InfiniteScrollCards(this.initialOffset, this.finalOffset);

  final double initialOffset;
  final double finalOffset;

  @override
  State<_InfiniteScrollCards> createState() => _InfiniteScrollCardsState();
}

class _InfiniteScrollCardsState extends State<_InfiniteScrollCards> {
  static const Duration stepDuration = Duration(seconds: 20);

  late ScrollController scrollController;
  late double offset;

  @override
  void initState() {
    super.initState();

    offset = widget.initialOffset;

    scrollController = ScrollController(initialScrollOffset: offset);

    // Without the timer the animation doesn't begin.
    Timer.run(() async {
      await scrollController.animateTo(
        widget.finalOffset,
        curve: Curves.linear,
        duration: stepDuration,
      );
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
      itemExtent: 100.0,
      itemBuilder: (BuildContext context, int index) {
        return SizedBox(
          height: 100.0,
          child: Card(
            elevation: 16.0,
            child: Text('${lipsum[index % lipsum.length]} $index', textAlign: TextAlign.center),
          ),
        );
      },
    );
  }
}
