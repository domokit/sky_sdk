// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  // Here and below, see: https://github.com/dart-lang/sdk/issues/26980
  const FlutterLogoDecoration start = FlutterLogoDecoration(
    textColor: Color(0xFFD4F144),
    style: FlutterLogoStyle.stacked,
    margin: EdgeInsets.all(10.0),
  );

  const FlutterLogoDecoration end = FlutterLogoDecoration(
    textColor: Color(0xFF81D4FA),
    style: FlutterLogoStyle.stacked,
    margin: EdgeInsets.all(10.0),
  );

  test('FlutterLogoDecoration lerp from null to null is null', () {
    final FlutterLogoDecoration? logo = FlutterLogoDecoration.lerp(null, null, 0.5);
    expect(logo, isNull);
  });

  test('FlutterLogoDecoration.lerp identical a,b', () {
    expect(FlutterLogoDecoration.lerp(null, null, 0), null);
    const FlutterLogoDecoration logo = FlutterLogoDecoration();
    expect(identical(FlutterLogoDecoration.lerp(logo, logo, 0.5), logo), true);
  });

  test('FlutterLogoDecoration lerp from non-null to null lerps margin', () {
    final FlutterLogoDecoration logo = FlutterLogoDecoration.lerp(start, null, 0.4)!;
    expect(logo.textColor, start.textColor);
    expect(logo.style, start.style);
    expect(logo.margin, start.margin * 0.4);
  });

  test('FlutterLogoDecoration lerp from null to non-null lerps margin', () {
    final FlutterLogoDecoration logo = FlutterLogoDecoration.lerp(null, end, 0.6)!;
    expect(logo.textColor, end.textColor);
    expect(logo.style, end.style);
    expect(logo.margin, end.margin * 0.6);
  });

  test('FlutterLogoDecoration lerps colors and margins', () {
    final FlutterLogoDecoration logo = FlutterLogoDecoration.lerp(start, end, 0.5)!;
    expect(logo.textColor, Color.lerp(start.textColor, end.textColor, 0.5));
    expect(logo.margin, EdgeInsets.lerp(start.margin, end.margin, 0.5));
  });

  test('FlutterLogoDecoration.lerpFrom and FlutterLogoDecoration.lerpTo', () {
    expect(Decoration.lerp(start, const BoxDecoration(), 0.0), start);
    expect(Decoration.lerp(start, const BoxDecoration(), 1.0), const BoxDecoration());
    expect(Decoration.lerp(const BoxDecoration(), end, 0.0), const BoxDecoration());
    expect(Decoration.lerp(const BoxDecoration(), end, 1.0), end);
  });

  test('FlutterLogoDecoration lerp changes styles at 0.5', () {
    FlutterLogoDecoration logo = FlutterLogoDecoration.lerp(start, end, 0.4)!;
    expect(logo.style, start.style);

    logo = FlutterLogoDecoration.lerp(start, end, 0.5)!;
    expect(logo.style, end.style);
  });

  test('FlutterLogoDecoration toString', () {
    expect(
      start.toString(),
      equals('FlutterLogoDecoration(textColor: ${const Color(0xffd4f144)}, style: stacked)'),
    );
    expect(
      FlutterLogoDecoration.lerp(null, end, 0.5).toString(),
      equals(
        'FlutterLogoDecoration(textColor: ${const Color(0xff81d4fa)}, style: stacked, transition -1.0:0.5)',
      ),
    );
  });

  testWidgets('Flutter Logo golden test', (WidgetTester tester) async {
    final Key logo = UniqueKey();
    await tester.pumpWidget(Container(key: logo, decoration: const FlutterLogoDecoration()));

    await expectLater(find.byKey(logo), matchesGoldenFile('flutter_logo.png'));
  });
}
