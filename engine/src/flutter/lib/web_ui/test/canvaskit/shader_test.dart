// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'common.dart';
import 'test_data.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('CanvasKit shaders', () {
    setUpCanvasKitTest();

    test('Sweep gradient', () {
      final CkGradientSweep gradient =
          ui.Gradient.sweep(ui.Offset.zero, testColors) as CkGradientSweep;
      expect(gradient.getSkShader(ui.FilterQuality.none), isNotNull);
    });

    test('Linear gradient', () {
      final CkGradientLinear gradient =
          ui.Gradient.linear(ui.Offset.zero, const ui.Offset(0, 1), testColors) as CkGradientLinear;
      expect(gradient.getSkShader(ui.FilterQuality.none), isNotNull);
    });

    test('Radial gradient', () {
      final CkGradientRadial gradient =
          ui.Gradient.radial(ui.Offset.zero, 10, testColors) as CkGradientRadial;
      expect(gradient.getSkShader(ui.FilterQuality.none), isNotNull);
    });

    test('Conical gradient', () {
      final CkGradientConical gradient =
          ui.Gradient.radial(
                ui.Offset.zero,
                10,
                testColors,
                null,
                ui.TileMode.clamp,
                null,
                const ui.Offset(10, 10),
                40,
              )
              as CkGradientConical;
      expect(gradient.getSkShader(ui.FilterQuality.none), isNotNull);
    });

    test('Image shader initialize/dispose cycle', () {
      final SkImage skImage =
          canvasKit.MakeAnimatedImageFromEncoded(kTransparentImage)!.makeImageAtCurrentFrame();
      final CkImage image = CkImage(skImage);
      final CkImageShader imageShader =
          ui.ImageShader(
                image,
                ui.TileMode.clamp,
                ui.TileMode.repeated,
                Float64List.fromList(Matrix4.diagonal3Values(1, 2, 3).storage),
              )
              as CkImageShader;
      expect(imageShader, isA<CkImageShader>());

      final UniqueRef<SkShader> ref = imageShader.ref!;
      expect(imageShader.debugDisposed, false);
      expect(imageShader.getSkShader(ui.FilterQuality.none), same(ref.nativeObject));
      expect(ref.isDisposed, false);
      expect(image.debugDisposed, false);
      imageShader.dispose();
      expect(imageShader.debugDisposed, true);
      expect(ref.isDisposed, true);
      expect(imageShader.ref, isNull);
      expect(image.debugDisposed, true);
    });

    test('Image shader withQuality', () {
      final SkImage skImage =
          canvasKit.MakeAnimatedImageFromEncoded(kTransparentImage)!.makeImageAtCurrentFrame();
      final CkImage image = CkImage(skImage);
      final CkImageShader imageShader =
          ui.ImageShader(
                image,
                ui.TileMode.clamp,
                ui.TileMode.repeated,
                Float64List.fromList(Matrix4.diagonal3Values(1, 2, 3).storage),
              )
              as CkImageShader;
      expect(imageShader, isA<CkImageShader>());

      final UniqueRef<SkShader> ref1 = imageShader.ref!;
      expect(imageShader.getSkShader(ui.FilterQuality.none), same(ref1.nativeObject));

      // Request the same quality as the default quality (none).
      expect(imageShader.getSkShader(ui.FilterQuality.none), isNotNull);
      final UniqueRef<SkShader> ref2 = imageShader.ref!;
      expect(ref1, same(ref2));
      expect(ref1.isDisposed, false);
      expect(image.debugDisposed, false);

      // Change quality to medium.
      expect(imageShader.getSkShader(ui.FilterQuality.medium), isNotNull);
      final UniqueRef<SkShader> ref3 = imageShader.ref!;
      expect(ref1, isNot(same(ref3)));
      expect(
        ref1.isDisposed,
        true,
        reason: 'The previous reference must be released to avoid a memory leak',
      );
      expect(image.debugDisposed, false);
      expect(imageShader.ref!.nativeObject, same(ref3.nativeObject));

      // Ask for medium again.
      expect(imageShader.getSkShader(ui.FilterQuality.medium), isNotNull);
      final UniqueRef<SkShader> ref4 = imageShader.ref!;
      expect(ref4, same(ref3));
      expect(ref3.isDisposed, false);
      expect(image.debugDisposed, false);
      expect(imageShader.ref!.nativeObject, same(ref4.nativeObject));

      // Done with the shader.
      imageShader.dispose();
      expect(imageShader.debugDisposed, true);
      expect(ref4.isDisposed, true);
      expect(imageShader.ref, isNull);
      expect(image.debugDisposed, true);
    });
  });
}

const List<ui.Color> testColors = <ui.Color>[ui.Color(0xFFFFFF00), ui.Color(0xFFFFFFFF)];
