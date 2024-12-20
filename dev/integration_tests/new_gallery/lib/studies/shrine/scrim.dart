// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class Scrim extends StatelessWidget {
  const Scrim({super.key, required this.controller});

  final AnimationController controller;

  @override
  Widget build(BuildContext context) {
    return ExcludeSemantics(
      child: AnimatedBuilder(
        animation: controller,
        builder: (BuildContext context, Widget? child) {
          final Widget scrimRectangle = ColoredBox(
            color: Color.fromRGBO(0xFF, 0xF0, 0xEA, controller.value * 0.87),
            child: SizedBox.fromSize(size: MediaQuery.sizeOf(context)),
          );

          switch (controller.status) {
            case AnimationStatus.completed:
              return MouseRegion(
                cursor: SystemMouseCursors.click,
                child: GestureDetector(onTap: controller.reverse, child: scrimRectangle),
              );
            case AnimationStatus.dismissed:
              return IgnorePointer(child: scrimRectangle);
            case AnimationStatus.forward || AnimationStatus.reverse:
              return scrimRectangle;
          }
        },
      ),
    );
  }
}
