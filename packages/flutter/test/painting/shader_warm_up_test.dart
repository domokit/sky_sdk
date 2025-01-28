// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

Future<void> main() async {
  test('ShaderWarmUp', () {
    final FakeShaderWarmUp shaderWarmUp = FakeShaderWarmUp();
    PaintingBinding.shaderWarmUp = shaderWarmUp;
    debugCaptureShaderWarmUpImage = expectAsync1((ui.Image image) => true);
    WidgetsFlutterBinding.ensureInitialized();
    expect(shaderWarmUp.ranWarmUp, true);
  });
}

class FakeShaderWarmUp extends ShaderWarmUp {
  bool ranWarmUp = false;

  @override
  Future<bool> warmUpOnCanvas(ui.Canvas canvas) {
    ranWarmUp = true;
    return Future<bool>.delayed(Duration.zero, () => true);
  }
}
