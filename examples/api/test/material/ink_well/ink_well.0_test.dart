// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/ink_well/ink_well.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets(
    'Clicking on InkWell changes the Size of 50x50 AnimatedContainer to 100x100 and vice versa',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.InkWellExampleApp());
      expect(find.widgetWithText(AppBar, 'InkWell Sample'), findsOneWidget);
      final Finder inkWell = find.byType(InkWell);
      final InkWell inkWellWidget = tester.widget<InkWell>(inkWell);
      final Finder animatedContainer = find.byType(AnimatedContainer);
      AnimatedContainer animatedContainerWidget = tester.widget<AnimatedContainer>(
        animatedContainer,
      );
      expect(inkWell, findsOneWidget);
      expect(inkWellWidget.onTap.runtimeType, VoidCallback);
      expect(animatedContainerWidget.constraints?.minWidth, 50);
      expect(animatedContainerWidget.constraints?.minHeight, 50);
      await tester.tap(inkWell);
      await tester.pumpAndSettle();
      animatedContainerWidget = tester.widget<AnimatedContainer>(animatedContainer);
      expect(animatedContainerWidget.constraints?.minWidth, 100);
      expect(animatedContainerWidget.constraints?.minHeight, 100);
      await tester.tap(inkWell);
      await tester.pumpAndSettle();
      animatedContainerWidget = tester.widget<AnimatedContainer>(animatedContainer);
      expect(animatedContainerWidget.constraints?.minWidth, 50);
      expect(animatedContainerWidget.constraints?.minHeight, 50);
    },
  );
}
