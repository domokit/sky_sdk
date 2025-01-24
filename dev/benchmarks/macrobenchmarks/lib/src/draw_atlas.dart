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

class DrawAtlasPage extends StatefulWidget {
  const DrawAtlasPage({super.key});

  @override
  State<DrawAtlasPage> createState() => _DrawAtlasPageState();
}

class _DrawAtlasPageState extends State<DrawAtlasPage> with SingleTickerProviderStateMixin {
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
    canvas.drawAtlas(
      image,
      <RSTransform>[
        RSTransform.fromComponents(
          rotation: 0,
          scale: 1,
          anchorX: 0,
          anchorY: 0,
          translateX: 0,
          translateY: 0,
        ),
      ],
      <Rect>[Rect.fromLTWH(0, 0, image.width.toDouble(), image.height.toDouble())],
      <Color>[Colors.red],
      BlendMode.plus,
      null,
      Paint(),
    );
    canvas.drawAtlas(
      image,
      <RSTransform>[
        RSTransform.fromComponents(
          rotation: 0,
          scale: 1,
          anchorX: 0,
          anchorY: 0,
          translateX: 250,
          translateY: 0,
        ),
      ],
      <Rect>[Rect.fromLTWH(0, 0, image.width.toDouble(), image.height.toDouble())],
      <Color>[Colors.green],
      BlendMode.plus,
      null,
      Paint(),
    );
    canvas.drawAtlas(
      image,
      <RSTransform>[
        RSTransform.fromComponents(
          rotation: 0,
          scale: 1,
          anchorX: 0,
          anchorY: 0,
          translateX: 0,
          translateY: 250,
        ),
      ],
      <Rect>[Rect.fromLTWH(0, 0, image.width.toDouble(), image.height.toDouble())],
      <Color>[Colors.blue],
      BlendMode.plus,
      null,
      Paint(),
    );
    canvas.drawAtlas(
      image,
      <RSTransform>[
        RSTransform.fromComponents(
          rotation: 0,
          scale: 1,
          anchorX: 0,
          anchorY: 0,
          translateX: 250,
          translateY: 250,
        ),
      ],
      <Rect>[Rect.fromLTWH(0, 0, image.width.toDouble(), image.height.toDouble())],
      <Color>[Colors.yellow],
      BlendMode.plus,
      null,
      Paint(),
    );
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true;
  }
}
