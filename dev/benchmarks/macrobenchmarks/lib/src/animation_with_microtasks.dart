// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class AnimationWithMicrotasks extends StatefulWidget {
  const AnimationWithMicrotasks({super.key});

  @override
  State<AnimationWithMicrotasks> createState() => _AnimationWithMicrotasksState();
}

class _AnimationWithMicrotasksState extends State<AnimationWithMicrotasks> {
  final _ChunkedWork work = _ChunkedWork();

  @override
  void initState() {
    super.initState();
    work.start();
  }

  @override
  void dispose() {
    work.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return const Scaffold(
      backgroundColor: Colors.grey,
      body: Center(child: SizedBox(width: 200, height: 100, child: LinearProgressIndicator())),
    );
  }
}

class _ChunkedWork {
  bool _canceled = false;

  Future<void> start() async {
    // Run 100 pieces of synchronous work.
    // Chunked up to allow frames to be drawn.
    for (int i = 0; i < 100; ++i) {
      _chunkedSynchronousWork();
    }
  }

  void cancel() {
    _canceled = true;
  }

  Future<void> _chunkedSynchronousWork() async {
    while (!_canceled) {
      // Yield to the event loop to let engine draw frames.
      await Future<void>.delayed(Duration.zero);

      // Perform synchronous computation for 1 ms.
      _syncComputationFor(const Duration(milliseconds: 1));
    }
  }

  void _syncComputationFor(Duration duration) {
    final Stopwatch sw = Stopwatch()..start();
    while (!_canceled && sw.elapsed < duration) {}
  }
}
