// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

const String kApiDocsLink =
    'See "Types with special considerations" at https://api.flutter.dev/flutter/animation/Tween-class.html for more information.';

void main() {
  test(
    'throws flutter error when tweening types that do not fully satisfy tween requirements - Object',
    () {
      final Tween<Object> objectTween = Tween<Object>(begin: Object(), end: Object());

      expect(
        () => objectTween.transform(0.1),
        throwsA(
          isA<FlutterError>().having(
            (FlutterError error) =>
                error.diagnostics.map((DiagnosticsNode node) => node.toString()),
            'diagnostics',
            <String>[
              'Cannot lerp between "Instance of \'Object\'" and "Instance of \'Object\'".',
              'The type Object might not fully implement `+`, `-`, and/or `*`. $kApiDocsLink',
              'There may be a dedicated "ObjectTween" for this type, or you may need to create one.',
            ],
          ),
        ),
      );
    },
  );

  test(
    'throws flutter error when tweening types that do not fully satisfy tween requirements - Color',
    () {
      final Tween<Color> colorTween = Tween<Color>(
        begin: const Color(0xFF000000),
        end: const Color(0xFFFFFFFF),
      );

      expect(
        () => colorTween.transform(0.1),
        throwsA(
          isA<FlutterError>().having(
            (FlutterError error) =>
                error.diagnostics.map((DiagnosticsNode node) => node.toString()),
            'diagnostics',
            <String>[
              'Cannot lerp between "${const Color(0xff000000)}" and "${const Color(0xffffffff)}".',
              'The type Color might not fully implement `+`, `-`, and/or `*`. $kApiDocsLink',
              'To lerp colors, consider ColorTween instead.',
            ],
          ),
        ),
      );
    },
  );

  test(
    'throws flutter error when tweening types that do not fully satisfy tween requirements - Rect',
    () {
      final Tween<Rect> rectTween = Tween<Rect>(
        begin: const Rect.fromLTWH(0, 0, 10, 10),
        end: const Rect.fromLTWH(2, 2, 2, 2),
      );

      expect(
        () => rectTween.transform(0.1),
        throwsA(
          isA<FlutterError>().having(
            (FlutterError error) =>
                error.diagnostics.map((DiagnosticsNode node) => node.toString()),
            'diagnostics',
            <String>[
              'Cannot lerp between "Rect.fromLTRB(0.0, 0.0, 10.0, 10.0)" and "Rect.fromLTRB(2.0, 2.0, 4.0, 4.0)".',
              'The type Rect might not fully implement `+`, `-`, and/or `*`. $kApiDocsLink',
              'To lerp rects, consider RectTween instead.',
            ],
          ),
        ),
      );
    },
  );

  test(
    'throws flutter error when tweening types that do not fully satisfy tween requirements - int',
    () {
      final Tween<int> colorTween = Tween<int>(begin: 0, end: 1);

      expect(
        () => colorTween.transform(0.1),
        throwsA(
          isA<FlutterError>().having(
            (FlutterError error) =>
                error.diagnostics.map((DiagnosticsNode node) => node.toString()),
            'diagnostics',
            <String>[
              'Cannot lerp between "0" and "1".',
              'The type int returned a double after multiplication with a double value. $kApiDocsLink',
              'To lerp int values, consider IntTween or StepTween instead.',
            ],
          ),
        ),
      );
    },
  );

  test('Can chain tweens', () {
    final Tween<double> tween = Tween<double>(begin: 0.30, end: 0.50);
    expect(tween, hasOneLineDescription);
    final Animatable<double> chain = tween.chain(Tween<double>(begin: 0.50, end: 1.0));
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    expect(chain.evaluate(controller), 0.40);
    expect(chain, hasOneLineDescription);
  });

  test('Can animate tweens', () {
    final Tween<double> tween = Tween<double>(begin: 0.30, end: 0.50);
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    final Animation<double> animation = tween.animate(controller);
    controller.value = 0.50;
    expect(animation.value, 0.40);
    expect(animation, hasOneLineDescription);
    expect(animation.toStringDetails(), hasOneLineDescription);
  });

  test('Can drive tweens', () {
    final Tween<double> tween = Tween<double>(begin: 0.30, end: 0.50);
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    final Animation<double> animation = controller.drive(tween);
    controller.value = 0.50;
    expect(animation.value, 0.40);
    expect(animation, hasOneLineDescription);
    expect(animation.toStringDetails(), hasOneLineDescription);
  });

  test('BorderTween nullable test', () {
    BorderTween tween = BorderTween();
    expect(tween.lerp(0.0), null);
    expect(tween.lerp(1.0), null);

    tween = BorderTween(end: const Border(top: BorderSide()));
    expect(tween.lerp(0.0), const Border());
    expect(tween.lerp(0.5), const Border(top: BorderSide(width: 0.5)));
    expect(tween.lerp(1.0), const Border(top: BorderSide()));
  });

  test('SizeTween', () {
    final SizeTween tween = SizeTween(begin: Size.zero, end: const Size(20.0, 30.0));
    expect(tween.lerp(0.5), equals(const Size(10.0, 15.0)));
    expect(tween, hasOneLineDescription);
  });

  test('IntTween', () {
    final IntTween tween = IntTween(begin: 5, end: 9);
    expect(tween.lerp(0.5), 7);
    expect(tween.lerp(0.7), 8);
  });

  test('RectTween', () {
    const Rect a = Rect.fromLTWH(5.0, 3.0, 7.0, 11.0);
    const Rect b = Rect.fromLTWH(8.0, 12.0, 14.0, 18.0);
    final RectTween tween = RectTween(begin: a, end: b);
    expect(tween.lerp(0.5), equals(Rect.lerp(a, b, 0.5)));
    expect(tween, hasOneLineDescription);
  });

  test('Matrix4Tween', () {
    final Matrix4 a = Matrix4.identity();
    final Matrix4 b =
        a.clone()
          ..translate(6.0, -8.0)
          ..scale(0.5, 1.0, 5.0);
    final Matrix4Tween tween = Matrix4Tween(begin: a, end: b);
    expect(tween.lerp(0.0), equals(a));
    expect(tween.lerp(1.0), equals(b));
    expect(
      tween.lerp(0.5),
      equals(
        a.clone()
          ..translate(3.0, -4.0)
          ..scale(0.75, 1.0, 3.0),
      ),
    );
    final Matrix4 c = a.clone()..rotateZ(1.0);
    final Matrix4Tween rotationTween = Matrix4Tween(begin: a, end: c);
    expect(rotationTween.lerp(0.0), equals(a));
    expect(rotationTween.lerp(1.0), equals(c));
    expect(rotationTween.lerp(0.5).absoluteError(a.clone()..rotateZ(0.5)), moreOrLessEquals(0.0));
  });

  test('ConstantTween', () {
    final ConstantTween<double> tween = ConstantTween<double>(100.0);
    expect(tween.begin, 100.0);
    expect(tween.end, 100.0);
    expect(tween.lerp(0.0), 100.0);
    expect(tween.lerp(0.5), 100.0);
    expect(tween.lerp(1.0), 100.0);
  });

  test('ReverseTween', () {
    final ReverseTween<int> tween = ReverseTween<int>(IntTween(begin: 5, end: 9));
    expect(tween.lerp(0.5), 7);
    expect(tween.lerp(0.7), 6);
  });

  test('ColorTween', () {
    final ColorTween tween = ColorTween(
      begin: const Color(0xff000000),
      end: const Color(0xffffffff),
    );
    expect(tween.lerp(0.0), isSameColorAs(const Color(0xff000000)));
    expect(tween.lerp(0.5), isSameColorAs(const Color(0xff7f7f7f)));
    expect(tween.lerp(0.7), isSameColorAs(const Color(0xffb2b2b2)));
    expect(tween.lerp(1.0), isSameColorAs(const Color(0xffffffff)));
  });

  test('StepTween', () {
    final StepTween tween = StepTween(begin: 5, end: 9);
    expect(tween.lerp(0.5), 7);
    expect(tween.lerp(0.7), 7);
  });

  test('CurveTween', () {
    final CurveTween tween = CurveTween(curve: Curves.easeIn);
    expect(tween.transform(0.0), 0.0);
    expect(tween.transform(0.5), 0.31640625);
    expect(tween.transform(1.0), 1.0);
  });

  test('BorderRadiusTween nullable test', () {
    final BorderRadiusTween tween = BorderRadiusTween();
    expect(tween.transform(0.0), null);
    expect(tween.transform(1.0), null);
    expect(tween.lerp(0.0), null);
  });
}
