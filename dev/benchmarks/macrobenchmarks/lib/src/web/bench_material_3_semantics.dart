// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/semantics.dart';

import 'material3.dart';
import 'recorder.dart';

/// Measures the cost of semantics when constructing screens containing
/// Material 3 widgets.
class BenchMaterial3Semantics extends WidgetBuildRecorder {
  BenchMaterial3Semantics() : super(name: benchmarkName);

  static const String benchmarkName = 'bench_material3_semantics';

  @override
  Future<void> setUpAll() async {
    FlutterTimeline.debugCollectionEnabled = true;
    super.setUpAll();
    SemanticsBinding.instance.ensureSemantics();
  }

  @override
  Future<void> tearDownAll() async {
    FlutterTimeline.debugReset();
  }

  @override
  void frameDidDraw() {
    // Only record frames that show the widget. Frames that remove the widget
    // are not interesting.
    if (showWidget) {
      final AggregatedTimings timings = FlutterTimeline.debugCollect();
      final AggregatedTimedBlock semanticsBlock = timings.getAggregated('SEMANTICS');
      final AggregatedTimedBlock getFragmentBlock = timings.getAggregated('Semantics.GetFragment');
      final AggregatedTimedBlock compileChildrenBlock = timings.getAggregated(
        'Semantics.compileChildren',
      );
      profile!.addTimedBlock(semanticsBlock, reported: true);
      profile!.addTimedBlock(getFragmentBlock, reported: true);
      profile!.addTimedBlock(compileChildrenBlock, reported: true);
    }

    super.frameDidDraw();
    FlutterTimeline.debugReset();
  }

  @override
  Widget createWidget() {
    return const SingleColumnMaterial3Components();
  }
}

/// Measures the cost of semantics when scrolling screens containing Material 3
/// widgets.
///
/// The implementation uses a ListView that jumps the scroll position between
/// 0 and 1 every frame. Such a small delta is not enough for lazy rendering to
/// add/remove widgets, but its enough to trigger the framework to recompute
/// some of the semantics.
///
/// The expected output numbers of this benchmarks should be very small as
/// scrolling a list view should be a matter of shifting some widgets and
/// updating the projected clip imposed by the viewport. As of June 2023, the
/// numbers are not great. Semantics consumes >50% of frame time.
class BenchMaterial3ScrollSemantics extends WidgetRecorder {
  BenchMaterial3ScrollSemantics() : super(name: benchmarkName);

  static const String benchmarkName = 'bench_material3_scroll_semantics';

  @override
  Future<void> setUpAll() async {
    FlutterTimeline.debugCollectionEnabled = true;
    super.setUpAll();
    SemanticsBinding.instance.ensureSemantics();
  }

  @override
  Future<void> tearDownAll() async {
    FlutterTimeline.debugReset();
  }

  @override
  void frameDidDraw() {
    final AggregatedTimings timings = FlutterTimeline.debugCollect();
    final AggregatedTimedBlock semanticsBlock = timings.getAggregated('SEMANTICS');
    final AggregatedTimedBlock getFragmentBlock = timings.getAggregated('Semantics.GetFragment');
    final AggregatedTimedBlock compileChildrenBlock = timings.getAggregated(
      'Semantics.compileChildren',
    );
    profile!.addTimedBlock(semanticsBlock, reported: true);
    profile!.addTimedBlock(getFragmentBlock, reported: true);
    profile!.addTimedBlock(compileChildrenBlock, reported: true);

    super.frameDidDraw();
    FlutterTimeline.debugReset();
  }

  @override
  Widget createWidget() => _ScrollTest();
}

class _ScrollTest extends StatefulWidget {
  @override
  State<_ScrollTest> createState() => _ScrollTestState();
}

class _ScrollTestState extends State<_ScrollTest> with SingleTickerProviderStateMixin {
  late final Ticker ticker;
  late final ScrollController scrollController;

  @override
  void initState() {
    super.initState();

    scrollController = ScrollController();

    bool forward = true;

    // A one-off timer is necessary to allow the framework to measure the
    // available scroll extents before the scroll controller can be exercised
    // to change the scroll position.
    Timer.run(() {
      ticker = createTicker((_) {
        scrollController.jumpTo(forward ? 1 : 0);
        forward = !forward;
      });
      ticker.start();
    });
  }

  @override
  void dispose() {
    ticker.dispose();
    scrollController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return SingleColumnMaterial3Components(scrollController: scrollController);
  }
}
