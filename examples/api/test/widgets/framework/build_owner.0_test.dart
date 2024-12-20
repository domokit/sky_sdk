// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/framework/build_owner.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('BuildOwnerExample displays the measured size', (WidgetTester tester) async {
    await tester.pumpWidget(const example.BuildOwnerExample());

    expect(find.text('Size(640.0, 480.0)'), findsOne);
  });

  test('The size of the widget is measured', () {
    expect(example.measureWidget(const SizedBox(width: 234, height: 567)), const Size(234, 567));
  });
}
