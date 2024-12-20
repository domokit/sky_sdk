// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/semantics.dart';
import 'package:google_fonts/google_fonts.dart';

void main() => runApp(const MyApp(Colors.blue));

@pragma('vm:entry-point')
void topMain() => runApp(const MyApp(Colors.green));

@pragma('vm:entry-point')
void bottomMain() => runApp(const MyApp(Colors.purple));

class MyApp extends StatelessWidget {
  const MyApp(this.color, {super.key});

  final Color color;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(primarySwatch: color as MaterialColor),
      home: const MyHomePage(title: 'Flutter Demo Home Page'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, this.title});
  final String? title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class Sky extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final Rect rect = Offset.zero & size;
    const RadialGradient gradient = RadialGradient(
      center: Alignment(0.7, -0.6),
      radius: 0.2,
      colors: <Color>[Color(0xFFFFFF00), Color(0xFF0099FF)],
      stops: <double>[0.4, 1.0],
    );
    canvas.drawRect(rect, Paint()..shader = gradient.createShader(rect));
  }

  @override
  SemanticsBuilderCallback get semanticsBuilder {
    return (Size size) {
      // Annotate a rectangle containing the picture of the sun
      // with the label "Sun". When text to speech feature is enabled on the
      // device, a user will be able to locate the sun on this picture by
      // touch.
      Rect rect = Offset.zero & size;
      final double width = size.shortestSide * 0.4;
      rect = const Alignment(0.8, -0.9).inscribe(Size(width, width), rect);
      return <CustomPainterSemantics>[
        CustomPainterSemantics(
          rect: rect,
          properties: const SemanticsProperties(label: 'Sun', textDirection: TextDirection.ltr),
        ),
      ];
    };
  }

  // Since this Sky painter has no fields, it always paints
  // the same thing and semantics information is the same.
  // Therefore we return false here. If we had fields (set
  // from the constructor) then we would return true if any
  // of them differed from the same fields on the oldDelegate.
  @override
  bool shouldRepaint(Sky oldDelegate) => false;
  @override
  bool shouldRebuildSemantics(Sky oldDelegate) => false;
}

class _MyHomePageState extends State<MyHomePage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(widget.title ?? '')),
      body: Center(
        child: SingleChildScrollView(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              Text('You have pushed the button this many times:', style: GoogleFonts.lato()),
              Text('0', style: Theme.of(context).textTheme.headlineMedium),
              TextButton(onPressed: () {}, child: const Text('Add')),
              TextButton(onPressed: () {}, child: const Text('Next')),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceAround,
                children: <Widget>[
                  const Icon(
                    Icons.favorite,
                    color: Colors.pink,
                    size: 24.0,
                    semanticLabel: 'Text to announce in accessibility modes',
                  ),
                  const Icon(Icons.audiotrack, color: Colors.green, size: 30.0),
                  const Icon(Icons.beach_access, color: Colors.blue, size: 36.0),
                  const Icon(Icons.zoom_out, color: Colors.amber, size: 36.0),
                  const Icon(Icons.money, color: Colors.lightGreen, size: 36.0),
                  const Icon(Icons.bug_report, color: Colors.teal, size: 36.0),
                  Container(
                    width: 36.0,
                    height: 36.0,
                    decoration: const BoxDecoration(
                      gradient: LinearGradient(
                        begin: Alignment.topLeft,
                        end: Alignment(0.8, 0.0), // 10% of the width, so there are ten blinds.
                        colors: <Color>[Color(0xffee0000), Color(0xffeeee00)], // red to yellow
                        tileMode: TileMode.repeated, // repeats the gradient over the canvas
                      ),
                    ),
                  ),
                ],
              ),
              CustomPaint(painter: Sky(), size: const Size(200.0, 36.0)),
            ],
          ),
        ),
      ),
    );
  }
}
