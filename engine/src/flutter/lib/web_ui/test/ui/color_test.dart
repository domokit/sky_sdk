// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart';

import '../common/test_initialization.dart';

class _ColorMatcher extends Matcher {
  _ColorMatcher(this._target, this._threshold);

  final Color _target;
  final double _threshold;

  @override
  Description describe(Description description) {
    return description.add('matches color "$_target" with threshold "$_threshold".');
  }

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    return item is Color &&
        item.colorSpace == _target.colorSpace &&
        (item.a - _target.a).abs() <= _threshold &&
        (item.r - _target.r).abs() <= _threshold &&
        (item.g - _target.g).abs() <= _threshold &&
        (item.b - _target.b).abs() <= _threshold;
  }
}

Matcher isSameColorAs(Color color, {double threshold = 0.004}) {
  return _ColorMatcher(color, threshold);
}

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

class NotAColor extends Color {
  const NotAColor(super.value);
}

Future<void> testMain() async {
  setUpUnitTests();

  test('color accessors should work', () {
    const Color foo = Color(0x12345678);
    expect(foo.alpha, equals(0x12));
    expect(foo.red, equals(0x34));
    expect(foo.green, equals(0x56));
    expect(foo.blue, equals(0x78));
  });

  test('paint set to black', () {
    const Color c = Color(0x00000000);
    final Paint p = Paint();
    p.color = c;
    expect(c.toString(), equals('${const Color(0x00000000)}'));
  });

  test('color created with out of bounds value', () {
    const Color c = Color(0x100 << 24);
    final Paint p = Paint();
    p.color = c;
  });

  test('color created with wildly out of bounds value', () {
    const Color c = Color(1 << 1000000);
    final Paint p = Paint();
    p.color = c;
  });

  test('two colors are only == if they have the same runtime type', () {
    expect(const Color(123), equals(const Color(123)));
    expect(const Color(123), equals(const Color(123)));
    expect(const Color(123), isNot(equals(const Color(321))));
    expect(const Color(123), isNot(equals(const NotAColor(123))));
    expect(const NotAColor(123), isNot(equals(const Color(123))));
    expect(const NotAColor(123), equals(const NotAColor(123)));
  });

  test('Color.lerp', () {
    expect(
      Color.lerp(const Color(0x00000000), const Color(0xFFFFFFFF), 0.0),
      const Color(0x00000000),
    );
    expect(
      Color.lerp(const Color(0x00000000), const Color(0xFFFFFFFF), 0.5),
      isSameColorAs(const Color(0x7F7F7F7F)),
    );
    expect(
      Color.lerp(const Color(0x00000000), const Color(0xFFFFFFFF), 1.0),
      const Color(0xFFFFFFFF),
    );
    expect(
      Color.lerp(const Color(0x00000000), const Color(0xFFFFFFFF), -0.1),
      const Color(0x00000000),
    );
    expect(
      Color.lerp(const Color(0x00000000), const Color(0xFFFFFFFF), 1.1),
      const Color(0xFFFFFFFF),
    );
  });

  test('Color.alphaBlend', () {
    expect(
      Color.alphaBlend(const Color(0x00000000), const Color(0x00000000)),
      const Color(0x00000000),
    );
    expect(
      Color.alphaBlend(const Color(0x00000000), const Color(0xFFFFFFFF)),
      const Color(0xFFFFFFFF),
    );
    expect(
      Color.alphaBlend(const Color(0xFFFFFFFF), const Color(0x00000000)),
      const Color(0xFFFFFFFF),
    );
    expect(
      Color.alphaBlend(const Color(0xFFFFFFFF), const Color(0xFFFFFFFF)),
      const Color(0xFFFFFFFF),
    );
    expect(
      Color.alphaBlend(const Color(0x80FFFFFF), const Color(0xFF000000)),
      const Color(0xFF808080),
    );
    expect(
      Color.alphaBlend(const Color(0x80808080), const Color(0xFFFFFFFF)),
      isSameColorAs(const Color(0xFFBFBFBF)),
    );
    expect(
      Color.alphaBlend(const Color(0x80808080), const Color(0xFF000000)),
      isSameColorAs(const Color(0xFF404040)),
    );
    expect(
      Color.alphaBlend(const Color(0x01020304), const Color(0xFF000000)),
      isSameColorAs(const Color(0xFF000000)),
    );
    expect(
      Color.alphaBlend(const Color(0x11223344), const Color(0xFF000000)),
      isSameColorAs(const Color(0xFF020304)),
    );
    expect(
      Color.alphaBlend(const Color(0x11223344), const Color(0x80000000)),
      isSameColorAs(const Color(0x88040608)),
    );
  });

  test('compute gray luminance', () {
    // Each color component is at 20%.
    const Color lightGray = Color(0xFF333333);
    // Relative luminance's formula is just the linearized color value for gray.
    // ((0.2 + 0.055) / 1.055) ^ 2.4.
    expect(lightGray.computeLuminance(), equals(0.033104766570885055));
  });

  test('compute color luminance', () {
    const Color brightRed = Color(0xFFFF3B30);
    // 0.2126 * ((1.0 + 0.055) / 1.055) ^ 2.4 +
    // 0.7152 * ((0.23137254902 +0.055) / 1.055) ^ 2.4 +
    // 0.0722 * ((0.18823529411 + 0.055) / 1.055) ^ 2.4
    expect(brightRed.computeLuminance(), equals(0.24601329637099723));
  });

  // Regression test for https://github.com/flutter/flutter/issues/41257
  // CupertinoDynamicColor was overriding base class and calling super(0).
  test('subclass of Color can override value', () {
    const DynamicColorClass color = DynamicColorClass(0xF0E0D0C0);
    expect(color.value, 0xF0E0D0C0);
    // Call base class member, make sure it uses overridden value.
    expect(color.red, 0xE0);
  });

  test('Paint converts Color subclasses to plain Color', () {
    const DynamicColorClass color = DynamicColorClass(0xF0E0D0C0);
    final Paint paint = Paint()..color = color;
    expect(paint.color.runtimeType, Color);
  });
}

class DynamicColorClass extends Color {
  const DynamicColorClass(int newValue) : _newValue = newValue, super(0);

  final int _newValue;

  @override
  int get value => _newValue;
}
