// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('WidgetSpan codeUnitAt', () {
    const InlineSpan span = WidgetSpan(child: SizedBox());
    expect(span.codeUnitAt(-1), isNull);
    expect(span.codeUnitAt(0), PlaceholderSpan.placeholderCodeUnit);
    expect(span.codeUnitAt(1), isNull);
    expect(span.codeUnitAt(2), isNull);

    const InlineSpan nestedSpan = TextSpan(text: 'AAA', children: <InlineSpan>[span, span]);
    expect(nestedSpan.codeUnitAt(-1), isNull);
    expect(nestedSpan.codeUnitAt(0), 65);
    expect(nestedSpan.codeUnitAt(1), 65);
    expect(nestedSpan.codeUnitAt(2), 65);
    expect(nestedSpan.codeUnitAt(3), PlaceholderSpan.placeholderCodeUnit);
    expect(nestedSpan.codeUnitAt(4), PlaceholderSpan.placeholderCodeUnit);
    expect(nestedSpan.codeUnitAt(5), isNull);
  });

  test('WidgetSpan.extractFromInlineSpan applies the correct scaling factor', () {
    const WidgetSpan a = WidgetSpan(child: SizedBox(), style: TextStyle(fontSize: 0));
    const WidgetSpan b = WidgetSpan(child: SizedBox(), style: TextStyle(fontSize: 10));
    const WidgetSpan c = WidgetSpan(child: SizedBox());
    const WidgetSpan d = WidgetSpan(child: SizedBox(), style: TextStyle(letterSpacing: 999));

    const TextSpan span = TextSpan(
      children: <InlineSpan>[
        a, // fontSize = 0.
        TextSpan(
          children: <InlineSpan>[
            b, // fontSize = 10.
            c, // fontSize = 20.
          ],
          style: TextStyle(fontSize: 20),
        ),
        d, // fontSize = 14.
      ],
    );

    double effectiveTextScaleFactorFromWidget(Widget widget) {
      final Semantics child = (widget as ProxyWidget).child as Semantics;
      final dynamic grandChild = child.child;
      final double textScaleFactor =
          grandChild.textScaleFactor as double; // ignore: avoid_dynamic_calls
      return textScaleFactor;
    }

    final List<double> textScaleFactors =
        WidgetSpan.extractFromInlineSpan(
          span,
          const _QuadraticScaler(),
        ).map(effectiveTextScaleFactorFromWidget).toList();

    expect(textScaleFactors, <double>[
      0, // a
      10, // b
      20, // c
      14, // d
    ]);
  });
}

class _QuadraticScaler extends TextScaler {
  const _QuadraticScaler();

  @override
  double scale(double fontSize) => fontSize * fontSize;

  @override
  double get textScaleFactor => throw UnimplementedError();
}
