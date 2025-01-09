// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart' as ui;

import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(setUpTestViewDimensions: false);

  // Previously the logic that set the effective font family would forget the
  // original value and would print incorrect value in toString.
  test('TextStyle remembers original fontFamily value', () {
    final ui.TextStyle style1 = ui.TextStyle();
    expect(style1.toString(), contains('fontFamily: unspecified'));

    final ui.TextStyle style2 = ui.TextStyle(fontFamily: 'Hello');
    expect(style2.toString(), contains('fontFamily: Hello'));
  });

  test('ParagraphStyle remembers original fontFamily value', () {
    final ui.ParagraphStyle style1 = ui.ParagraphStyle();
    expect(style1.toString(), contains('fontFamily: unspecified'));

    final ui.ParagraphStyle style2 = ui.ParagraphStyle(fontFamily: 'Hello');
    expect(style2.toString(), contains('fontFamily: Hello'));
  });
}
