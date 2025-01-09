// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'picture_cache.dart';

class ClipperCachePage extends StatefulWidget {
  const ClipperCachePage({super.key});
  @override
  State<ClipperCachePage> createState() => _ClipperCachePageState();
}

class _ClipperCachePageState extends State<ClipperCachePage> with TickerProviderStateMixin {
  final double _animateOffset = 100;
  final ScrollController _controller = ScrollController();
  final bool _isComplex = true;
  late double _topMargin;

  @override
  void initState() {
    super.initState();
    _controller.addListener(() {
      if (_controller.offset < 10) {
        _controller.animateTo(
          _animateOffset,
          duration: const Duration(milliseconds: 1000),
          curve: Curves.ease,
        );
      } else if (_controller.offset > _animateOffset - 10) {
        _controller.animateTo(0, duration: const Duration(milliseconds: 1000), curve: Curves.ease);
      }
    });
    Timer(const Duration(milliseconds: 500), () {
      _controller.animateTo(
        _animateOffset,
        duration: const Duration(milliseconds: 1000),
        curve: Curves.ease,
      );
    });
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();

    const double itemHeight = 140;
    final FlutterView view = View.of(context);
    _topMargin = (view.physicalSize.height / view.devicePixelRatio - itemHeight * 3) / 2;
    if (_topMargin < 0) {
      _topMargin = 0;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.blue,
      body: ListView(
        controller: _controller,
        children: <Widget>[
          SizedBox(height: _topMargin),
          ClipPath(clipBehavior: Clip.antiAliasWithSaveLayer, child: _makeChild(0, _isComplex)),
          ClipRect(clipBehavior: Clip.antiAliasWithSaveLayer, child: _makeChild(1, _isComplex)),
          ClipRRect(clipBehavior: Clip.antiAliasWithSaveLayer, child: _makeChild(2, _isComplex)),
          const SizedBox(height: 1000),
        ],
      ),
    );
  }

  Widget _makeChild(int itemIndex, bool complex) {
    final BoxDecoration decoration = BoxDecoration(
      color: Colors.white70,
      boxShadow: const <BoxShadow>[BoxShadow(blurRadius: 5.0)],
      borderRadius: BorderRadius.circular(5.0),
    );
    return RepaintBoundary(
      child: Container(
        margin: const EdgeInsets.fromLTRB(10, 5, 10, 5),
        decoration: complex ? decoration : null,
        child: ListItem(index: itemIndex),
      ),
    );
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }
}
