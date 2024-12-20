// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'package:flutter/material.dart';
import 'picture_cache.dart';

class ShaderMaskCachePage extends StatefulWidget {
  const ShaderMaskCachePage({super.key});
  @override
  State<ShaderMaskCachePage> createState() => _ShaderMaskCachePageState();
}

class _ShaderMaskCachePageState extends State<ShaderMaskCachePage> with TickerProviderStateMixin {
  final ScrollController _controller = ScrollController();

  @override
  void initState() {
    super.initState();
    _controller.addListener(() {
      if (_controller.offset < 10) {
        _controller.animateTo(
          100,
          duration: const Duration(milliseconds: 1000),
          curve: Curves.ease,
        );
      } else if (_controller.offset > 90) {
        _controller.animateTo(0, duration: const Duration(milliseconds: 1000), curve: Curves.ease);
      }
    });
    Timer(const Duration(milliseconds: 500), () {
      _controller.animateTo(100, duration: const Duration(milliseconds: 1000), curve: Curves.ease);
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.lightBlue,
      body: ListView(
        controller: _controller,
        children: <Widget>[
          const SizedBox(height: 100),
          buildShaderMask(0),
          const SizedBox(height: 10),
          buildShaderMask(1),
          const SizedBox(height: 1000),
        ],
      ),
    );
  }

  Widget buildShaderMask(int index) {
    return ShaderMask(
      shaderCallback: (Rect bounds) {
        return const RadialGradient(
          center: Alignment.topLeft,
          radius: 1.0,
          colors: <Color>[Colors.yellow, Colors.red],
          tileMode: TileMode.mirror,
        ).createShader(bounds);
      },
      child: Container(
        clipBehavior: Clip.antiAlias,
        decoration: const BoxDecoration(
          boxShadow: <BoxShadow>[BoxShadow(color: Colors.white, blurRadius: 5.0)],
        ),
        child: ListItem(index: index),
      ),
    );
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }
}
