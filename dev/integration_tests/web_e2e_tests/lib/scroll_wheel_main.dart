// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'package:web/web.dart' as web;

void main() => runApp(const MyApp());

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Scroll Wheel Test',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        fontFamily: 'RobotoMono',
        visualDensity: VisualDensity.adaptivePlatformDensity,
      ),
      home: const MyHomePage(title: 'Flutter Scroll Wheel Test'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});

  final String title;

  @override
  State createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(widget.title)),
      body: ListView.builder(
        itemCount: 1000,
        itemBuilder:
            (BuildContext context, int index) => Padding(
              padding: const EdgeInsets.all(20),
              child: Container(
                height: 100,
                color: Colors.lightBlue,
                child: Center(child: Text('Item $index')),
              ),
            ),
      ),
      floatingActionButton: FloatingActionButton.extended(
        key: const Key('scroll-button'),
        onPressed: () {
          const int centerX = 100; //html.window.innerWidth ~/ 2;
          const int centerY = 100; //html.window.innerHeight ~/ 2;
          dispatchMouseWheelEvent(centerX, centerY, DeltaMode.kLine, 0, 1);
          dispatchMouseWheelEvent(centerX, centerY, DeltaMode.kLine, 0, 1);
          dispatchMouseWheelEvent(centerX, centerY, DeltaMode.kLine, 0, 1);
          dispatchMouseWheelEvent(centerX, centerY, DeltaMode.kLine, 0, 1);
          dispatchMouseWheelEvent(centerX, centerY, DeltaMode.kLine, 0, 1);
        },
        label: const Text('Scroll'),
        icon: const Icon(Icons.thumb_up),
      ),
    );
  }
}

abstract class DeltaMode {
  static const int kLine = 0x01;
}

void dispatchMouseWheelEvent(int mouseX, int mouseY, int deltaMode, double deltaX, double deltaY) {
  final web.EventTarget target = web.document.elementFromPoint(mouseX, mouseY)!;

  target.dispatchEvent(
    web.MouseEvent(
      'mouseover',
      web.MouseEventInit(screenX: mouseX, screenY: mouseY, clientX: mouseX, clientY: mouseY),
    ),
  );

  target.dispatchEvent(
    web.MouseEvent(
      'mousemove',
      web.MouseEventInit(screenX: mouseX, screenY: mouseY, clientX: mouseX, clientY: mouseY),
    ),
  );

  target.dispatchEvent(
    web.WheelEvent(
      'wheel',
      web.WheelEventInit(deltaMode: deltaMode, deltaX: deltaX, deltaY: deltaY),
    ),
  );
}
