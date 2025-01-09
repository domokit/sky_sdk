// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';

class BackdropFilterPage extends StatefulWidget {
  const BackdropFilterPage({super.key});

  @override
  State<BackdropFilterPage> createState() => _BackdropFilterPageState();
}

class _BackdropFilterPageState extends State<BackdropFilterPage> with TickerProviderStateMixin {
  bool _blurGroup = false;
  bool _blurTexts = true;
  late AnimationController animation;

  @override
  void initState() {
    super.initState();
    animation = AnimationController(vsync: this, duration: const Duration(seconds: 1));
    animation.repeat();
  }

  @override
  void dispose() {
    animation.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    Widget addBlur(Widget child, bool shouldBlur) {
      if (shouldBlur) {
        return ClipRect(
          child: BackdropFilter(filter: ImageFilter.blur(sigmaX: 5, sigmaY: 5), child: child),
        );
      } else {
        return child;
      }
    }

    final Widget txt = addBlur(
      Container(padding: const EdgeInsets.all(5), child: const Text('txt')),
      _blurTexts,
    );

    Widget col(Widget w, int numRows) {
      return Column(children: List<Widget>.generate(numRows, (int i) => w));
    }

    Widget grid(Widget w, int numRows, int numCols) {
      return Row(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: List<Widget>.generate(numCols, (int i) => col(w, numRows)),
      );
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
              const SizedBox(height: 20),
              RepaintBoundary(child: addBlur(grid(txt, 17, 5), _blurGroup)),
              const SizedBox(height: 20),
              ColoredBox(
                color: Colors.white,
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: <Widget>[
                    const Text('Backdrop per txt:'),
                    Checkbox(
                      value: _blurTexts,
                      onChanged:
                          (bool? v) => setState(() {
                            _blurTexts = v ?? false;
                          }),
                    ),
                    const SizedBox(width: 10),
                    const Text('Backdrop grid:'),
                    Checkbox(
                      value: _blurGroup,
                      onChanged:
                          (bool? v) => setState(() {
                            _blurGroup = v ?? false;
                          }),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}
