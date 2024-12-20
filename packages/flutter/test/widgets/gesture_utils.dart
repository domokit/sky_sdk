// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

Future<void> scrollAt(
  Offset position,
  WidgetTester tester, [
  Offset offset = const Offset(0.0, 20.0),
]) {
  final TestPointer testPointer = TestPointer(1, PointerDeviceKind.mouse);
  // Create a hover event so that |testPointer| has a location when generating the scroll.
  testPointer.hover(position);
  return tester.sendEventToBinding(testPointer.scroll(offset));
}
