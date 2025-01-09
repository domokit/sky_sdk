// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('RenderConstrainedBox getters and setters', () {
    final RenderConstrainedBox box = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints.tightFor(height: 10.0),
    );
    expect(box.additionalConstraints, const BoxConstraints(minHeight: 10.0, maxHeight: 10.0));
    box.additionalConstraints = const BoxConstraints.tightFor(width: 10.0);
    expect(box.additionalConstraints, const BoxConstraints(minWidth: 10.0, maxWidth: 10.0));
  });

  test('RenderLimitedBox getters and setters', () {
    final RenderLimitedBox box = RenderLimitedBox();
    expect(box.maxWidth, double.infinity);
    expect(box.maxHeight, double.infinity);
    box.maxWidth = 0.0;
    box.maxHeight = 1.0;
    expect(box.maxHeight, 1.0);
    expect(box.maxWidth, 0.0);
  });

  test('RenderAspectRatio getters and setters', () {
    final RenderAspectRatio box = RenderAspectRatio(aspectRatio: 1.0);
    expect(box.aspectRatio, 1.0);
    box.aspectRatio = 0.2;
    expect(box.aspectRatio, 0.2);
    box.aspectRatio = 1.2;
    expect(box.aspectRatio, 1.2);
  });

  test('RenderIntrinsicWidth getters and setters', () {
    final RenderIntrinsicWidth box = RenderIntrinsicWidth();
    expect(box.stepWidth, isNull);
    box.stepWidth = 10.0;
    expect(box.stepWidth, 10.0);
    expect(box.stepHeight, isNull);
    box.stepHeight = 10.0;
    expect(box.stepHeight, 10.0);
  });

  test('RenderOpacity getters and setters', () {
    final RenderOpacity box = RenderOpacity();
    expect(box.opacity, 1.0);
    box.opacity = 0.0;
    expect(box.opacity, 0.0);
  });

  test('RenderShaderMask getters and setters', () {
    Shader callback1(Rect bounds) {
      assert(false); // The test should not call this.
      const LinearGradient gradient = LinearGradient(colors: <Color>[Colors.red]);
      return gradient.createShader(Rect.zero);
    }

    Shader callback2(Rect bounds) {
      assert(false); // The test should not call this.
      const LinearGradient gradient = LinearGradient(colors: <Color>[Colors.blue]);
      return gradient.createShader(Rect.zero);
    }

    final RenderShaderMask box = RenderShaderMask(shaderCallback: callback1);
    expect(box.shaderCallback, equals(callback1));
    box.shaderCallback = callback2;
    expect(box.shaderCallback, equals(callback2));
    expect(box.blendMode, BlendMode.modulate);
    box.blendMode = BlendMode.colorBurn;
    expect(box.blendMode, BlendMode.colorBurn);
  });
}
