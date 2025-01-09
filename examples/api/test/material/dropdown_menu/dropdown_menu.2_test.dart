// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/material/dropdown_menu/dropdown_menu.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('DropdownMenu cursor behavior', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DropdownMenuApp());

    Finder textFieldFinder(int index) {
      return find.byType(TextField).at(index);
    }

    // Hover over the "enabled and requestFocusOnTap set to true" text field.
    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.moveTo(tester.getCenter(textFieldFinder(0)));

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Hover over the "enabled and requestFocusOnTap set to false" text field.
    await gesture.moveTo(tester.getCenter(textFieldFinder(1)));

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Hover over the "disabled and requestFocusOnTap set to true" text field.
    await gesture.moveTo(tester.getCenter(textFieldFinder(2)));

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    // Hover over the "disabled and requestFocusOnTap set to false" text field.
    await gesture.moveTo(tester.getCenter(textFieldFinder(3)));

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
  });
}
