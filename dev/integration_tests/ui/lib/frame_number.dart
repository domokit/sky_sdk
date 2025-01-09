// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math';

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_driver/driver_extension.dart';

/// This application shows empty screen until first frame timings are acquired.
void main() {
  enableFlutterDriverExtension();

  final Completer<List<FrameTiming>> completer = Completer<List<FrameTiming>>();
  SchedulerBinding.instance.addTimingsCallback((List<FrameTiming> timings) {
    completer.complete(timings);
  });

  runApp(
    Directionality(
      textDirection: TextDirection.ltr,
      child: _FirstFrameTimings(completer: completer),
    ),
  );
}

class _FirstFrameTimings extends StatefulWidget {
  const _FirstFrameTimings({required this.completer});

  final Completer<List<FrameTiming>> completer;

  @override
  _FirstFrameTimingsState createState() => _FirstFrameTimingsState();
}

class _FirstFrameTimingsState extends State<_FirstFrameTimings> {
  int? _minFrameNumber;

  @override
  Widget build(BuildContext context) {
    widget.completer.future.then(_setMinFrameNumber);
    if (_minFrameNumber != null) {
      return Text(_minFrameNumber.toString(), key: const Key('minFrameNumber'));
    } else {
      return const Text('Waiting...');
    }
  }

  void _setMinFrameNumber(List<FrameTiming> timings) {
    final int minFrameNumber = timings.map((FrameTiming timing) => timing.frameNumber).reduce(min);
    setState(() {
      _minFrameNumber = minFrameNumber;
    });
  }
}
