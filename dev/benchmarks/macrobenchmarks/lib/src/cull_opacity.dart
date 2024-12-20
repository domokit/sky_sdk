// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class CullOpacityPage extends StatefulWidget {
  const CullOpacityPage({super.key});

  @override
  State<StatefulWidget> createState() => _CullOpacityPageState();
}

class _CullOpacityPageState extends State<CullOpacityPage> with SingleTickerProviderStateMixin {
  late Animation<double> _offsetY;
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();

    _controller = AnimationController(vsync: this, duration: const Duration(seconds: 2));
    // Animations are typically implemented using the AnimatedBuilder widget.
    // This code uses a manual listener for historical reasons and will remain
    // in order to preserve compatibility with the history of measurements for
    // this benchmark.
    _offsetY = Tween<double>(begin: 0, end: -1000.0).animate(_controller)..addListener(() {
      setState(() {});
    });
    _controller.repeat();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: List<Widget>.generate(
        50,
        (int i) => Positioned(
          left: 0,
          top: (200 * i).toDouble() + _offsetY.value,
          child: Opacity(
            opacity: 0.5,
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
  }
}
