// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;
import 'package:flutter/material.dart';

// Various tests to verify that animated image filtered layers do not
// dirty children even without explicit repaint boundaries. These intentionally use
// text to ensure we don't measure the opacity peephole case.
class AnimatedBlurBackdropFilter extends StatefulWidget {
  const AnimatedBlurBackdropFilter({super.key});

  @override
  State<AnimatedBlurBackdropFilter> createState() => _AnimatedBlurBackdropFilterState();
}

class _AnimatedBlurBackdropFilterState extends State<AnimatedBlurBackdropFilter>
    with SingleTickerProviderStateMixin {
  late final AnimationController controller = AnimationController(
    vsync: this,
    duration: const Duration(milliseconds: 5000),
  );
  late final Animation<double> animation = controller.drive(Tween<double>(begin: 0.0, end: 1.0));
  ui.ImageFilter imageFilter = ui.ImageFilter.blur();

  @override
  void initState() {
    super.initState();
    controller.repeat();
    animation.addListener(() {
      setState(() {
        imageFilter = ui.ImageFilter.blur(
          sigmaX: animation.value * 16,
          sigmaY: animation.value * 16,
          tileMode: TileMode.decal,
        );
      });
    });
  }

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: Stack(
          children: <Widget>[
            ListView(
              children: <Widget>[
                for (int i = 0; i < 30; i++)
                  Center(
                    child: Transform.scale(scale: 1.01, child: const ModeratelyComplexWidget()),
                  ),
              ],
            ),
            BackdropFilter(filter: imageFilter, child: const SizedBox.expand()),
          ],
        ),
      ),
    );
  }
}

class ModeratelyComplexWidget extends StatelessWidget {
  const ModeratelyComplexWidget({super.key});

  @override
  Widget build(BuildContext context) {
    return const Material(
      elevation: 10,
      clipBehavior: Clip.hardEdge,
      child: ListTile(
        leading: Icon(Icons.abc, size: 24),
        title: DecoratedBox(
          decoration: BoxDecoration(color: Colors.red),
          child: Text('Hello World'),
        ),
        trailing: FlutterLogo(),
      ),
    );
  }
}
