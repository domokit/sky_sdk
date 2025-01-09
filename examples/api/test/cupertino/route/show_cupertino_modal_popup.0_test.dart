// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/route/show_cupertino_modal_popup.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Tap on button displays cupertino modal dialog', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ModalPopupApp());

    final Finder actionOne = find.text('Action One');
    expect(actionOne, findsNothing);

    await tester.tap(find.byType(CupertinoButton));
    await tester.pumpAndSettle();
    expect(actionOne, findsOneWidget);

    await tester.tap(find.text('Action One'));
    await tester.pumpAndSettle();
    expect(actionOne, findsNothing);
  });
}
