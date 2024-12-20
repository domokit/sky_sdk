// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  final List<String> warnings = <String>[];
  late void Function(String) oldPrintWarning;

  setUpAll(() async {
    oldPrintWarning = printWarning;
    printWarning = (String warning) {
      warnings.add(warning);
    };
  });

  tearDownAll(() {
    printWarning = oldPrintWarning;
  });

  test('Emit a warning when the HTML Renderer was picked.', () {
    final Renderer chosenRenderer = renderer;

    expect(chosenRenderer, isA<HtmlRenderer>());
    expect(
      warnings,
      contains(contains('See: https://docs.flutter.dev/to/web-html-renderer-deprecation')),
    );
  });
}
