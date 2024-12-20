// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  test('importing platformViewRegistry from dart:ui is deprecated', () {
    final void Function(String) oldPrintWarning = printWarning;

    final List<String> warnings = <String>[];
    printWarning = (String message) {
      warnings.add(message);
    };

    // ignore: unnecessary_statements
    ui_web.platformViewRegistry;
    expect(warnings, isEmpty);

    // ignore: unnecessary_statements
    ui.platformViewRegistry;
    expect(warnings, hasLength(1));
    expect(warnings.single, contains('platformViewRegistry'));
    expect(warnings.single, contains('deprecated'));
    expect(warnings.single, contains('dart:ui_web'));

    printWarning = oldPrintWarning;
  });
}
