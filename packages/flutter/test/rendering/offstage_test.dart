// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('offstage', () {
    RenderBox child;
    bool painted = false;
    // incoming constraints are tight 800x600
    final RenderBox root = RenderPositionedBox(
      child: RenderConstrainedBox(
        additionalConstraints: const BoxConstraints.tightFor(width: 800.0),
        child: RenderOffstage(
          child: RenderCustomPaint(
            painter: TestCallbackPainter(
              onPaint: () {
                painted = true;
              },
            ),
            child:
                child = RenderConstrainedBox(
                  additionalConstraints: const BoxConstraints.tightFor(height: 10.0, width: 10.0),
                ),
          ),
        ),
      ),
    );
    expect(child.hasSize, isFalse);
    expect(painted, isFalse);
    layout(root, phase: EnginePhase.paint);
    expect(child.hasSize, isTrue);
    expect(painted, isFalse);
    expect(child.size, equals(const Size(800.0, 10.0)));
  });
}
