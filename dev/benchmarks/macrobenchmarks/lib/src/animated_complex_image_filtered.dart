// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;
import 'package:flutter/material.dart';

// Various tests to verify that animated image filtered layers do not
// dirty children even without explicit repaint boundaries. These intentionally use
// text to ensure we don't measure the opacity peephole case.
class AnimatedComplexImageFiltered extends StatefulWidget {
  const AnimatedComplexImageFiltered({super.key});

  @override
  State<AnimatedComplexImageFiltered> createState() => _AnimatedComplexImageFilteredState();
}

class _AnimatedComplexImageFilteredState extends State<AnimatedComplexImageFiltered>
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
        imageFilter = ui.ImageFilter.blur(sigmaX: animation.value * 5, sigmaY: animation.value * 5);
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
        body: ListView(
          children: <Widget>[
            for (int i = 0; i < 20; i++)
              ImageFiltered(
                imageFilter: imageFilter,
                child: Center(
                  child: Transform.scale(scale: 1.01, child: const ModeratelyComplexWidget()),
                ),
              ),
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
