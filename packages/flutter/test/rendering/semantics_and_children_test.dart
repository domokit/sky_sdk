// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

int countSemanticsChildren(RenderObject object) {
  int count = 0;
  object.visitChildrenForSemantics((RenderObject child) {
    count += 1;
  });
  return count;
}

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('RenderOpacity and children and semantics', () {
    final RenderOpacity box = RenderOpacity(
      child: RenderParagraph(const TextSpan(), textDirection: TextDirection.ltr),
    );
    expect(countSemanticsChildren(box), 1);
    box.opacity = 0.5;
    expect(countSemanticsChildren(box), 1);
    box.opacity = 0.25;
    expect(countSemanticsChildren(box), 1);
    box.opacity = 0.125;
    expect(countSemanticsChildren(box), 1);
    box.opacity = 0.0;
    expect(countSemanticsChildren(box), 0);
    box.opacity = 0.125;
    expect(countSemanticsChildren(box), 1);
    box.opacity = 0.0;
    expect(countSemanticsChildren(box), 0);
  });

  test('RenderOpacity and children and semantics', () {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    final RenderAnimatedOpacity box = RenderAnimatedOpacity(
      opacity: controller,
      child: RenderParagraph(const TextSpan(), textDirection: TextDirection.ltr),
    );
    expect(countSemanticsChildren(box), 0); // controller defaults to 0.0
    controller.value = 0.2; // has no effect, box isn't subscribed yet
    expect(countSemanticsChildren(box), 0);
    controller.value = 1.0; // ditto
    expect(countSemanticsChildren(box), 0); // alpha is still 0
    layout(box); // this causes the box to attach, which makes it subscribe
    expect(countSemanticsChildren(box), 1);
    controller.value = 1.0;
    expect(countSemanticsChildren(box), 1);
    controller.value = 0.5;
    expect(countSemanticsChildren(box), 1);
    controller.value = 0.25;
    expect(countSemanticsChildren(box), 1);
    controller.value = 0.125;
    expect(countSemanticsChildren(box), 1);
    controller.value = 0.0;
    expect(countSemanticsChildren(box), 0);
    controller.value = 0.125;
    expect(countSemanticsChildren(box), 1);
    controller.value = 0.0;
    expect(countSemanticsChildren(box), 0);
  });
}
