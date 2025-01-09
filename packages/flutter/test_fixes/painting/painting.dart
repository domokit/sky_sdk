// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/painting.dart';

void main() {
  // Change made in https://github.com/flutter/flutter/pull/121152
  final EdgeInsets insets = EdgeInsets.fromWindowPadding(ViewPadding.zero, 3.0);

  // Change made in https://github.com/flutter/flutter/pull/128522
  const TextStyle textStyle =
      TextStyle()
        ..getTextStyle(textScaleFactor: math.min(_kTextScaleFactor, 1.0))
        ..getTextStyle();

  TextPainter(text: inlineSpan);
  TextPainter(textScaleFactor: someValue);

  TextPainter.computeWidth(textScaleFactor: textScaleFactor);
  TextPainter.computeMaxIntrinsicWidth(textScaleFactor: textScaleFactor);

  Color color = Color.from(alpha: 1, red: 0, green: 1, blue: 0);
  print(color.opacity);
  color = color.withOpacity(0.55);
}
