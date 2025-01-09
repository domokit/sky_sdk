// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [AppBar.shape].

void main() => runApp(const AppBarExampleApp());

class AppBarExampleApp extends StatelessWidget {
  const AppBarExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: AppBarExample());
  }
}

class AppBarExample extends StatelessWidget {
  const AppBarExample({super.key});

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;

    return Scaffold(
      appBar: AppBar(
        shape: const CustomAppBarShape(),
        backgroundColor: colorScheme.primaryContainer,
        title: const Text('AppBar Sample'),
        bottom: PreferredSize(
          preferredSize: const Size.fromHeight(64.0),
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16.0),
            child: TextField(
              decoration: InputDecoration(
                border: OutlineInputBorder(borderSide: BorderSide(color: colorScheme.primary)),
                enabledBorder: OutlineInputBorder(
                  borderSide: BorderSide(color: colorScheme.onPrimaryContainer),
                ),
                filled: true,
                hintText: 'Enter a search term',
                fillColor: colorScheme.surface,
                prefixIcon: Icon(Icons.search_rounded, color: colorScheme.primary),
                suffixIcon: Icon(Icons.tune_rounded, color: colorScheme.primary),
              ),
            ),
          ),
        ),
      ),
      body: ListView.builder(
        padding: const EdgeInsets.only(top: 45.0),
        itemCount: 20,
        itemBuilder: (BuildContext context, int index) {
          return ListTile(title: Text('Item $index'));
        },
      ),
    );
  }
}

class CustomAppBarShape extends OutlinedBorder {
  // Implementing the constructor allows the CustomAppBarShape to be
  // properly compared when calling the `identical` method.
  const CustomAppBarShape({super.side});

  Path _getPath(Rect rect) {
    final Path path = Path();
    final Size size = Size(rect.width, rect.height * 1.5);

    final double p0 = size.height * 0.75;
    path.lineTo(0.0, p0);

    final Offset controlPoint = Offset(size.width * 0.4, size.height);
    final Offset endPoint = Offset(size.width, size.height / 2);
    path.quadraticBezierTo(controlPoint.dx, controlPoint.dy, endPoint.dx, endPoint.dy);

    path.lineTo(size.width, 0.0);
    path.close();

    return path;
  }

  @override
  Path getOuterPath(Rect rect, {TextDirection? textDirection}) {
    return _getPath(rect.inflate(side.width));
  }

  @override
  Path getInnerPath(Rect rect, {TextDirection? textDirection}) {
    return _getPath(rect);
  }

  @override
  void paint(Canvas canvas, Rect rect, {TextDirection? textDirection}) {
    if (rect.isEmpty) {
      return;
    }
    canvas.drawPath(getOuterPath(rect, textDirection: textDirection), side.toPaint());
  }

  @override
  ShapeBorder scale(double t) {
    return CustomAppBarShape(side: side.scale(t));
  }

  @override
  OutlinedBorder copyWith({BorderSide? side}) {
    return CustomAppBarShape(side: side ?? this.side);
  }

  // The lerpFrom method is necessary for the CustomAppBarShape to be
  // properly animated when changing the AppBar shape and when
  // the AppBar is rebuilt.
  @override
  ShapeBorder? lerpFrom(ShapeBorder? a, double t) {
    if (a is CustomAppBarShape) {
      return CustomAppBarShape(side: BorderSide.lerp(a.side, side, t));
    }
    return super.lerpFrom(a, t);
  }

  // The lerpTo method is necessary for the CustomAppBarShape to be
  // properly animated when changing the AppBar shape and when
  // the AppBar is rebuilt.
  @override
  ShapeBorder? lerpTo(ShapeBorder? b, double t) {
    if (b is CustomAppBarShape) {
      return CustomAppBarShape(side: BorderSide.lerp(b.side, side, t));
    }
    return super.lerpTo(b, t);
  }
}
