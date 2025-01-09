// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/button/cupertino_button.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Has 4 CupertinoButton variants', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoButtonApp());

    expect(find.byType(CupertinoButton), findsNWidgets(4));
    expect(
      find.ancestor(of: find.text('Enabled'), matching: find.byType(CupertinoButton)),
      findsNWidgets(2),
    );
    expect(
      find.ancestor(of: find.text('Disabled'), matching: find.byType(CupertinoButton)),
      findsNWidgets(2),
    );
  });
}
