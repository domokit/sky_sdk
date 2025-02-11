// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';

void main() {
  runApp(const PlatformViewApp());
}

class PlatformViewApp extends StatefulWidget {
  const PlatformViewApp({super.key});

  @override
  PlatformViewAppState createState() => PlatformViewAppState();
}

class PlatformViewAppState extends State<PlatformViewApp> {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData.light(),
      title: 'Advanced Layout',
      home: const PlatformViewLayout(),
    );
  }
}

class PlatformViewLayout extends StatelessWidget {
  const PlatformViewLayout({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Platform View Scrolling Layout')),
      body: ListView.builder(
        key: const Key('platform-views-scroll'), // This key is used by the driver test.
        itemCount: 200,
        itemBuilder: (BuildContext context, int index) {
          return Padding(
            padding: const EdgeInsets.all(5.0),
            child: Material(
              elevation: (index % 5 + 1).toDouble(),
              color: Colors.white,
              child: const Stack(children: <Widget>[_AndroidPlatformView(), RotationContainer()]),
            ),
          );
        },
      ),
    );
  }
}

final class _AndroidPlatformView extends StatelessWidget {
  const _AndroidPlatformView();

  static const String viewType = 'benchmarks/platform_views_layout/DummyPlatformView';

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: 400,
      height: 200,
      child: PlatformViewLink(
        viewType: viewType,
        surfaceFactory: (BuildContext context, PlatformViewController controller) {
          return AndroidViewSurface(
            controller: controller as AndroidViewController,
            gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
            hitTestBehavior: PlatformViewHitTestBehavior.opaque,
          );
        },
        onCreatePlatformView: (PlatformViewCreationParams params) {
          return PlatformViewsService.initHybridAndroidView(
              id: params.id,
              viewType: viewType,
              layoutDirection: TextDirection.ltr,
              creationParamsCodec: const StandardMessageCodec(),
            )
            ..addOnPlatformViewCreatedListener(params.onPlatformViewCreated)
            ..create();
        },
      ),
    );
  }
}

class RotationContainer extends StatefulWidget {
  const RotationContainer({super.key});

  @override
  State<RotationContainer> createState() => _RotationContainerState();
}

class _RotationContainerState extends State<RotationContainer> with SingleTickerProviderStateMixin {
  late AnimationController _rotationController;

  @override
  void initState() {
    super.initState();
    _rotationController = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 1),
      value: 1,
    );
    _rotationController.repeat();
  }

  @override
  void dispose() {
    _rotationController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return RotationTransition(
      turns: Tween<double>(begin: 0.0, end: 1.0).animate(_rotationController),
      child: Container(color: Colors.purple, width: 50.0, height: 50.0),
    );
  }
}
