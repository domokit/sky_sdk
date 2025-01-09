// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Properly constraints the physical size', (WidgetTester tester) async {
    final FlutterViewSpy view =
        FlutterViewSpy(view: tester.view)
          ..physicalConstraints = ViewConstraints.tight(const Size(1008.0, 2198.0))
          ..devicePixelRatio = 1.912500023841858;

    await tester.pumpWidget(wrapWithView: false, View(view: view, child: const SizedBox()));

    expect(view.sizes.single, const Size(1008.0, 2198.0));
  });
}

class FlutterViewSpy extends TestFlutterView {
  FlutterViewSpy({required TestFlutterView super.view})
    : super(platformDispatcher: view.platformDispatcher, display: view.display);

  List<Size?> sizes = <Size?>[];

  @override
  void render(Scene scene, {Size? size}) {
    sizes.add(size);
  }
}
