// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';

import 'recorder.dart';

/// Creates a [Wrap] inside a ListView.
///
/// Tests large number of DOM nodes since image breaks up large canvas.
class BenchWrapBoxScroll extends WidgetRecorder {
  BenchWrapBoxScroll() : super(name: benchmarkName);

  static const String benchmarkName = 'bench_wrapbox_scroll';

  @override
  Widget createWidget() {
    return MaterialApp(
      theme: ThemeData(primarySwatch: Colors.blue),
      title: 'WrapBox Scroll Benchmark',
      home: const Scaffold(body: MyHomePage()),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key});

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  late ScrollController scrollController;
  int block = 0;
  static const Duration stepDuration = Duration(milliseconds: 500);
  static const double stepDistance = 400;

  @override
  void initState() {
    super.initState();

    scrollController = ScrollController();

    // Without the timer the animation doesn't begin.
    Timer.run(() async {
      while (block < 25) {
        await scrollController.animateTo(
          (block % 5) * stepDistance,
          duration: stepDuration,
          curve: Curves.easeInOut,
        );
        block++;
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
    return ListView(
      controller: scrollController,
      children: <Widget>[
        Wrap(
          children: <Widget>[
            for (int i = 0; i < 30; i++)
              FractionallySizedBox(widthFactor: 0.2, child: ProductPreview(i)), //need case1
            for (int i = 0; i < 30; i++) ProductPreview(i), //need case2
          ],
        ),
      ],
    );
  }
}

class ProductPreview extends StatelessWidget {
  const ProductPreview(this.previewIndex, {super.key});

  final int previewIndex;

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      behavior: HitTestBehavior.translucent,
      onTap: () => print('tap'),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Container(
            margin: const EdgeInsets.all(23),
            padding: const EdgeInsets.all(18),
            decoration: const BoxDecoration(color: Color(0xfff9f9f9), shape: BoxShape.circle),
            child: Image.network('assets/assets/Icon-192.png', width: 100, height: 100),
          ),
          const Text('title'),
          const SizedBox(height: 14),
          Wrap(
            alignment: WrapAlignment.center,
            children: <Widget>[
              ProductOption(optionText: '$previewIndex: option1'),
              ProductOption(optionText: '$previewIndex: option2'),
              ProductOption(optionText: '$previewIndex: option3'),
              ProductOption(optionText: '$previewIndex: option4'),
              ProductOption(optionText: '$previewIndex: option5'),
            ],
          ),
        ],
      ),
    );
  }
}

class ProductOption extends StatelessWidget {
  const ProductOption({super.key, required this.optionText});

  final String optionText;

  @override
  Widget build(BuildContext context) {
    return Container(
      constraints: const BoxConstraints(minWidth: 56),
      margin: const EdgeInsets.all(2),
      padding: const EdgeInsets.symmetric(horizontal: 11, vertical: 5),
      decoration: BoxDecoration(
        border: Border.all(color: const Color(0xffebebeb)),
        borderRadius: const BorderRadius.all(Radius.circular(15)),
      ),
      child: Text(
        optionText,
        maxLines: 1,
        textAlign: TextAlign.center,
        overflow: TextOverflow.ellipsis,
      ),
    );
  }
}
