// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('positions itself at the anchor', (WidgetTester tester) async {
    // An arbitrary point on the screen to position at.
    const Offset anchor = Offset(30.0, 40.0);

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: DesktopTextSelectionToolbar(
            anchor: anchor,
            children: <Widget>[
              DesktopTextSelectionToolbarButton(child: const Text('Tap me'), onPressed: () {}),
            ],
          ),
        ),
      ),
    );

    expect(tester.getTopLeft(find.byType(DesktopTextSelectionToolbarButton)), anchor);
  });
}
