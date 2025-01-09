// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_driver/driver_extension.dart';

void main() {
  enableFlutterDriverExtension();
  debugPrint('Application starting...');
  runApp(const MyApp());
}

class MyApp extends StatefulWidget {
  const MyApp({super.key});

  @override
  State createState() => MyAppState();
}

const MethodChannel channel = MethodChannel('texture');

enum FrameState { initial, slow, afterSlow, fast, afterFast }

class MyAppState extends State<MyApp> with SingleTickerProviderStateMixin {
  int _widgetBuilds = 0;
  FrameState _state = FrameState.initial;
  String _summary = '';
  IconData? _icon;
  double _flutterFrameRate = 0;

  Future<void> _summarizeStats() async {
    final double? framesProduced = await channel.invokeMethod('getProducedFrameRate');
    final double? framesConsumed = await channel.invokeMethod('getConsumedFrameRate');
    _summary = '''
Produced: ${framesProduced?.toStringAsFixed(1)}fps
Consumed: ${framesConsumed?.toStringAsFixed(1)}fps
Widget builds: $_widgetBuilds''';
  }

  Future<void> _nextState() async {
    switch (_state) {
      case FrameState.initial:
        debugPrint('Starting .5x speed test...');
        _widgetBuilds = 0;
        _summary = 'Producing texture frames at .5x speed...';
        _state = FrameState.slow;
        _icon = Icons.stop;
        channel.invokeMethod<void>('start', _flutterFrameRate ~/ 2);
      case FrameState.slow:
        debugPrint('Stopping .5x speed test...');
        await channel.invokeMethod<void>('stop');
        await _summarizeStats();
        _icon = Icons.fast_forward;
        _state = FrameState.afterSlow;
      case FrameState.afterSlow:
        debugPrint('Starting 2x speed test...');
        _widgetBuilds = 0;
        _summary = 'Producing texture frames at 2x speed...';
        _state = FrameState.fast;
        _icon = Icons.stop;
        channel.invokeMethod<void>('start', (_flutterFrameRate * 2).toInt());
      case FrameState.fast:
        debugPrint('Stopping 2x speed test...');
        await channel.invokeMethod<void>('stop');
        await _summarizeStats();
        _state = FrameState.afterFast;
        _icon = Icons.replay;
      case FrameState.afterFast:
        debugPrint('Test complete.');
        _summary = 'Press play to start again';
        _state = FrameState.initial;
        _icon = Icons.play_arrow;
    }
    setState(() {});
  }

  @override
  void initState() {
    super.initState();
    _calibrate();
  }

  static const int calibrationTickCount = 600;

  /// Measures Flutter's frame rate.
  Future<void> _calibrate() async {
    debugPrint('Awaiting calm (3 second pause)...');
    await Future<void>.delayed(const Duration(milliseconds: 3000));
    debugPrint('Calibrating...');
    late DateTime startTime;
    int tickCount = 0;
    Ticker? ticker;
    ticker = createTicker((Duration time) {
      tickCount += 1;
      if (tickCount == calibrationTickCount) {
        // about 10 seconds
        final Duration elapsed = DateTime.now().difference(startTime);
        ticker?.stop();
        ticker?.dispose();
        setState(() {
          _flutterFrameRate = tickCount * 1000 / elapsed.inMilliseconds;
          debugPrint('Calibrated: frame rate ${_flutterFrameRate.toStringAsFixed(1)}fps.');
          _summary = '''
Flutter frame rate is ${_flutterFrameRate.toStringAsFixed(1)}fps.
Press play to produce texture frames.''';
          _icon = Icons.play_arrow;
          _state = FrameState.initial;
        });
      } else {
        if ((tickCount % (calibrationTickCount ~/ 20)) == 0) {
          debugPrint('Calibrating... ${(100.0 * tickCount / calibrationTickCount).floor()}%');
        }
      }
    });
    ticker.start();
    startTime = DateTime.now();
    setState(() {
      _summary = 'Calibrating...';
      _icon = null;
    });
  }

  @override
  Widget build(BuildContext context) {
    _widgetBuilds += 1;
    return MaterialApp(
      home: Scaffold(
        body: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              const SizedBox(width: 300.0, height: 200.0, child: Texture(textureId: 0)),
              Container(
                width: 300.0,
                height: 60.0,
                color: Colors.grey,
                child: Center(child: Text(_summary, key: const ValueKey<String>('summary'))),
              ),
            ],
          ),
        ),
        floatingActionButton:
            _icon == null
                ? null
                : FloatingActionButton(
                  key: const ValueKey<String>('fab'),
                  onPressed: _nextState,
                  child: Icon(_icon),
                ),
      ),
    );
  }
}
