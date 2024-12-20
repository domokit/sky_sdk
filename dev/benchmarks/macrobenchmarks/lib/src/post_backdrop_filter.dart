// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';

class PostBackdropFilterPage extends StatefulWidget {
  const PostBackdropFilterPage({super.key});

  @override
  State<PostBackdropFilterPage> createState() => _PostBackdropFilterPageState();
}

class _PostBackdropFilterPageState extends State<PostBackdropFilterPage>
    with TickerProviderStateMixin {
  bool _includeBackdropFilter = false;
  late AnimationController animation;

  @override
  void initState() {
    super.initState();
    animation = AnimationController(vsync: this, duration: const Duration(seconds: 1));
  }

  @override
  void dispose() {
    animation.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    Widget getConditionalBackdrop() {
      if (_includeBackdropFilter) {
        return Column(
          children: <Widget>[
            const SizedBox(height: 20),
            ClipRect(
              child: BackdropFilter(
                filter: ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0),
                child: const Text('BackdropFilter'),
              ),
            ),
            const SizedBox(height: 20),
          ],
        );
      } else {
        return const SizedBox(height: 20);
      }
    }

    return Scaffold(
      backgroundColor: Colors.grey,
      body: Stack(
        children: <Widget>[
          Text('0' * 10000, style: const TextStyle(color: Colors.yellow)),
          Column(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: <Widget>[
              Expanded(
                child: RepaintBoundary(
                  child: Center(
                    child: AnimatedBuilder(
                      animation: animation,
                      builder: (BuildContext c, Widget? w) {
                        final int val = (animation.value * 255).round();
                        return Container(
                          width: 50,
                          height: 50,
                          color: Color.fromARGB(255, val, val, val),
                        );
                      },
                    ),
                  ),
                ),
              ),
              getConditionalBackdrop(),
              RepaintBoundary(
                child: ColoredBox(
                  color: Colors.white,
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[
                      const Text('Include BackdropFilter:'),
                      Checkbox(
                        key: const Key('bdf-checkbox'), // this key is used by the driver test
                        value: _includeBackdropFilter,
                        onChanged:
                            (bool? v) => setState(() {
                              _includeBackdropFilter = v ?? false;
                            }),
                      ),
                      MaterialButton(
                        key: const Key('bdf-animate'), // this key is used by the driver test
                        child: const Text('Animate'),
                        onPressed:
                            () => setState(() {
                              animation.repeat();
                            }),
                      ),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}
