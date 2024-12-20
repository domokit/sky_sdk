// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ImageFiltered avoids repainting child as it animates', (WidgetTester tester) async {
    RenderTestObject.paintCount = 0;
    await tester.pumpWidget(
      ColoredBox(
        color: Colors.red,
        child: ImageFiltered(
          imageFilter: ImageFilter.blur(sigmaX: 5, sigmaY: 5),
          child: const TestWidget(),
        ),
      ),
    );

    expect(RenderTestObject.paintCount, 1);

    await tester.pumpWidget(
      ColoredBox(
        color: Colors.red,
        child: ImageFiltered(
          imageFilter: ImageFilter.blur(sigmaX: 6, sigmaY: 6),
          child: const TestWidget(),
        ),
      ),
    );

    expect(RenderTestObject.paintCount, 1);
  });
}

class TestWidget extends SingleChildRenderObjectWidget {
  const TestWidget({super.key, super.child});

  @override
  RenderObject createRenderObject(BuildContext context) {
    return RenderTestObject();
  }
}

class RenderTestObject extends RenderProxyBox {
  static int paintCount = 0;

  @override
  void paint(PaintingContext context, Offset offset) {
    paintCount += 1;
    super.paint(context, offset);
  }
}
