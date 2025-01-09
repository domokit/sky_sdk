// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Rendering Error', (WidgetTester tester) async {
    // Assets can load with its package name.
    await tester.pumpWidget(Image.asset('icon/test.png', width: 54, height: 54, fit: BoxFit.none));
  });
}
