// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('capabilities are set properly in HTML mode', (WidgetTester tester) async {
    await tester.pumpAndSettle();
    expect(isCanvasKit, false);
    expect(isSkwasm, false);
    expect(isSkiaWeb, false);
  });
}
