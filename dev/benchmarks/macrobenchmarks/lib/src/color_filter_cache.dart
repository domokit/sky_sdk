// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'package:flutter/material.dart';

class ColorFilterCachePage extends StatefulWidget {
  const ColorFilterCachePage({super.key});
  @override
  State<ColorFilterCachePage> createState() => _ColorFilterCachePageState();
}

class _ColorFilterCachePageState extends State<ColorFilterCachePage> with TickerProviderStateMixin {
  final ScrollController _controller = ScrollController();
  @override
  void initState() {
    super.initState();
    _controller.addListener(() {
      if (_controller.offset < 20) {
        _controller.animateTo(
          150,
          duration: const Duration(milliseconds: 1000),
          curve: Curves.ease,
        );
      } else if (_controller.offset > 130) {
        _controller.animateTo(0, duration: const Duration(milliseconds: 1000), curve: Curves.ease);
      }
    });
    Timer(const Duration(milliseconds: 1000), () {
      _controller.animateTo(150, duration: const Duration(milliseconds: 1000), curve: Curves.ease);
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.lightBlue,
      body: ListView(
        controller: _controller,
        children: <Widget>[
          const SizedBox(height: 150),
          ColorFiltered(
            colorFilter: ColorFilter.mode(Colors.green[300]!, BlendMode.luminosity),
            child: Container(
              clipBehavior: Clip.antiAlias,
              decoration: const BoxDecoration(
                boxShadow: <BoxShadow>[BoxShadow(color: Colors.red, blurRadius: 5.0)],
                color: Colors.blue,
                backgroundBlendMode: BlendMode.luminosity,
              ),
              child: Column(
                children: <Widget>[
                  const Text('Color Filter Cache Pref Test'),
                  Image.asset(
                    'food/butternut_squash_soup.png',
                    package: 'flutter_gallery_assets',
                    fit: BoxFit.cover,
                    width: 330,
                    height: 210,
                  ),
                  const Text('Color Filter Cache Pref Test'),
                ],
              ),
            ),
          ),
          const SizedBox(height: 1000),
        ],
      ),
    );
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }
}
