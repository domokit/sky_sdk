// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class ColorFilterWithUnstableChildPage extends StatefulWidget {
  const ColorFilterWithUnstableChildPage({super.key});

  @override
  State<StatefulWidget> createState() => _ColorFilterWithUnstableChildPageState();
}

class _ColorFilterWithUnstableChildPageState extends State<ColorFilterWithUnstableChildPage>
    with SingleTickerProviderStateMixin {
  late Animation<double> _offsetY;
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(vsync: this, duration: const Duration(seconds: 2));
    _offsetY = Tween<double>(begin: 0, end: -1000.0).animate(_controller);
    _controller.repeat();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: _offsetY,
      builder: (BuildContext context, Widget? child) {
        return Stack(
          children: List<Widget>.generate(
            50,
            (int i) => Positioned(
              left: 0,
              top: (200 * i).toDouble() + _offsetY.value,
              child: ColorFiltered(
                colorFilter: ColorFilter.mode(Colors.green[300]!, BlendMode.luminosity),
                child: RepaintBoundary(
                  child: Container(
                    // Slightly change width to invalidate raster cache.
                    width: 1000 - (_offsetY.value / 100),
                    height: 100,
                    color: Colors.red,
                  ),
                ),
              ),
            ),
          ),
        );
      },
    );
  }
}
