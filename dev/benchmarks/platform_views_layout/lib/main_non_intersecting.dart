// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter/material.dart';

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
              child:
                  index.isEven
                      ? CustomPaint(painter: ExpensivePainter(), size: const Size(400, 200))
                      : const DummyPlatformView(),
            ),
          );
        },
      ),
    );
  }
}

class DummyPlatformView extends StatelessWidget {
  const DummyPlatformView({super.key});

  @override
  Widget build(BuildContext context) {
    const String viewType = 'benchmarks/platform_views_layout/DummyPlatformView';
    late Widget nativeView;
    if (Platform.isIOS) {
      nativeView = const UiKitView(viewType: viewType);
    } else if (Platform.isAndroid) {
      nativeView = const AndroidView(viewType: viewType);
    } else {
      assert(false, 'Invalid platform');
    }
    return Container(color: Colors.purple, height: 200.0, child: nativeView);
  }
}

class ExpensivePainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final double boxWidth = size.width / 50;
    final double boxHeight = size.height / 50;
    for (int i = 0; i < 50; i++) {
      for (int j = 0; j < 50; j++) {
        final Rect rect = Rect.fromLTWH(i * boxWidth, j * boxHeight, boxWidth, boxHeight);
        canvas.drawRect(
          rect,
          Paint()
            ..style = PaintingStyle.fill
            ..color = Colors.red,
        );
      }
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return false;
  }
}
