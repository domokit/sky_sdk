// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/material.dart';

Future<ui.Image> loadImage(String asset) async {
  final ui.ImmutableBuffer buffer = await ui.ImmutableBuffer.fromAsset(asset);
  final ui.Codec codec = await PaintingBinding.instance.instantiateImageCodecWithSize(buffer);
  final ui.FrameInfo frameInfo = await codec.getNextFrame();
  codec.dispose();
  return frameInfo.image;
}

class DrawVerticesPage extends StatefulWidget {
  const DrawVerticesPage({super.key});

  @override
  State<DrawVerticesPage> createState() => _DrawVerticesPageState();
}

class _DrawVerticesPageState extends State<DrawVerticesPage> with SingleTickerProviderStateMixin {
  late final AnimationController controller;
  double tick = 0.0;
  ui.Image? image;

  @override
  void initState() {
    super.initState();
    loadImage('packages/flutter_gallery_assets/food/butternut_squash_soup.png').then((
      ui.Image pending,
    ) {
      setState(() {
        image = pending;
      });
    });
    controller = AnimationController(vsync: this, duration: const Duration(hours: 1));
    controller.addListener(() {
      setState(() {
        tick += 1;
      });
    });
    controller.forward(from: 0);
  }

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (image == null) {
      return const Placeholder();
    }
    return CustomPaint(
      size: const Size(500, 500),
      painter: VerticesPainter(tick, image!),
      child: Container(),
    );
  }
}

class VerticesPainter extends CustomPainter {
  VerticesPainter(this.tick, this.image);

  final double tick;
  final ui.Image image;

  @override
  void paint(Canvas canvas, Size size) {
    canvas.translate(0, tick);
    final ui.Vertices vertices = ui.Vertices(
      VertexMode.triangles,
      const <Offset>[
        Offset.zero,
        Offset(0, 250),
        Offset(250, 0),
        Offset(0, 250),
        Offset(250, 0),
        Offset(250, 250),
      ],
      textureCoordinates: <Offset>[
        Offset.zero,
        Offset(0, image.height.toDouble()),
        Offset(image.width.toDouble(), 0),
        Offset(0, image.height.toDouble()),
        Offset(image.width.toDouble(), 0),
        Offset(image.width.toDouble(), image.height.toDouble()),
      ],
      colors: <Color>[Colors.red, Colors.blue, Colors.green, Colors.red, Colors.blue, Colors.green],
    );
    canvas.drawVertices(
      vertices,
      BlendMode.plus,
      Paint()
        ..shader = ImageShader(image, TileMode.clamp, TileMode.clamp, Matrix4.identity().storage),
    );
    canvas.translate(250, 0);
    canvas.drawVertices(
      vertices,
      BlendMode.plus,
      Paint()
        ..shader = ImageShader(image, TileMode.clamp, TileMode.clamp, Matrix4.identity().storage),
    );
    canvas.translate(0, 250);
    canvas.drawVertices(
      vertices,
      BlendMode.plus,
      Paint()
        ..shader = ImageShader(image, TileMode.clamp, TileMode.clamp, Matrix4.identity().storage),
    );
    canvas.translate(-250, 0);
    canvas.drawVertices(
      vertices,
      BlendMode.plus,
      Paint()
        ..shader = ImageShader(image, TileMode.clamp, TileMode.clamp, Matrix4.identity().storage),
    );
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true;
  }
}
