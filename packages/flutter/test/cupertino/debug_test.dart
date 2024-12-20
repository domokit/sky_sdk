// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('debugCheckHasCupertinoLocalizations throws', (WidgetTester tester) async {
    final GlobalKey noLocalizationsAvailable = GlobalKey();
    final GlobalKey localizationsAvailable = GlobalKey();

    await tester.pumpWidget(
      Container(
        key: noLocalizationsAvailable,
        child: CupertinoApp(home: Container(key: localizationsAvailable)),
      ),
    );

    expect(
      () => debugCheckHasCupertinoLocalizations(noLocalizationsAvailable.currentContext!),
      throwsA(
        isAssertionError.having(
          (AssertionError e) => e.message,
          'message',
          contains('No CupertinoLocalizations found'),
        ),
      ),
    );

    expect(debugCheckHasCupertinoLocalizations(localizationsAvailable.currentContext!), isTrue);
  });
}
