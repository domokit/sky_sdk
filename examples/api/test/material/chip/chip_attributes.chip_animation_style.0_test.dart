// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/chip/chip_attributes.chip_animation_style.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ChipAnimationStyle.enableAnimation overrides chip enable animation', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ChipAnimationStyleExampleApp());

    final RenderBox materialBox = tester.firstRenderObject<RenderBox>(
      find.descendant(
        of: find.widgetWithText(RawChip, 'Enabled'),
        matching: find.byType(CustomPaint),
      ),
    );

    expect(materialBox, paints..rrect(color: const Color(0xffffc107)));

    await tester.tap(find.widgetWithText(ElevatedButton, 'Disable'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500)); // Advance enable animation by 500ms.

    expect(materialBox, paints..rrect(color: const Color(0x1f882f2b)));

    await tester.pump(const Duration(milliseconds: 500)); // Advance enable animation by 500ms.

    expect(materialBox, paints..rrect(color: const Color(0x1ff44336)));

    await tester.tap(find.widgetWithText(ElevatedButton, 'Enable'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 1500)); // Advance enable animation by 1500ms.

    expect(materialBox, paints..rrect(color: const Color(0xfffbd980)));

    await tester.pump(const Duration(milliseconds: 1500)); // Advance enable animation by 1500ms.

    expect(materialBox, paints..rrect(color: const Color(0xffffc107)));
  });

  testWidgets('ChipAnimationStyle.selectAnimation overrides chip select animation', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ChipAnimationStyleExampleApp());

    final RenderBox materialBox = tester.firstRenderObject<RenderBox>(
      find.descendant(
        of: find.widgetWithText(RawChip, 'Unselected'),
        matching: find.byType(CustomPaint),
      ),
    );

    expect(materialBox, paints..rrect(color: const Color(0xffffc107)));

    await tester.tap(find.widgetWithText(ElevatedButton, 'Select'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 1500)); // Advance select animation by 1500ms.

    expect(materialBox, paints..rrect(color: const Color(0xff4da6f4)));

    await tester.pump(const Duration(milliseconds: 1500)); // Advance select animation by 1500ms.

    expect(materialBox, paints..rrect(color: const Color(0xff2196f3)));

    await tester.tap(find.widgetWithText(ElevatedButton, 'Unselect'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500)); // Advance select animation by 500ms.

    expect(materialBox, paints..rrect(color: const Color(0xfff8e7c3)));

    await tester.pump(const Duration(milliseconds: 500)); // Advance select animation by 500ms.

    expect(materialBox, paints..rrect(color: const Color(0xffffc107)));
  });

  testWidgets('ChipAnimationStyle.avatarDrawerAnimation overrides chip checkmark animation', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ChipAnimationStyleExampleApp());

    expect(tester.getSize(find.widgetWithText(RawChip, 'Checked')).width, closeTo(152.6, 0.1));

    await tester.tap(find.widgetWithText(ElevatedButton, 'Hide checkmark'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500)); // Advance avatar animation by 500ms.

    expect(tester.getSize(find.widgetWithText(RawChip, 'Unchecked')).width, closeTo(160.9, 0.1));

    await tester.pump(const Duration(milliseconds: 500)); // Advance avatar animation by 500ms.

    expect(tester.getSize(find.widgetWithText(RawChip, 'Unchecked')).width, closeTo(160.9, 0.1));

    await tester.tap(find.widgetWithText(ElevatedButton, 'Show checkmark'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // Advance avatar animation by 1sec.

    expect(tester.getSize(find.widgetWithText(RawChip, 'Checked')).width, closeTo(132.7, 0.1));

    await tester.pump(const Duration(seconds: 1)); // Advance avatar animation by 1sec.

    expect(tester.getSize(find.widgetWithText(RawChip, 'Checked')).width, closeTo(152.6, 0.1));
  });

  testWidgets('ChipAnimationStyle.deleteDrawerAnimation overrides chip delete icon animation', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ChipAnimationStyleExampleApp());

    expect(tester.getSize(find.widgetWithText(RawChip, 'Deletable')).width, closeTo(180.9, 0.1));

    await tester.tap(find.widgetWithText(ElevatedButton, 'Hide delete icon'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500)); // Advance delete icon animation by 500ms.

    expect(tester.getSize(find.widgetWithText(RawChip, 'Undeletable')).width, closeTo(204.6, 0.1));

    await tester.pump(const Duration(milliseconds: 500)); // Advance delete icon animation by 500ms.

    expect(tester.getSize(find.widgetWithText(RawChip, 'Undeletable')).width, closeTo(189.1, 0.1));

    await tester.tap(find.widgetWithText(ElevatedButton, 'Show delete icon'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // Advance delete icon animation by 1sec.

    expect(tester.getSize(find.widgetWithText(RawChip, 'Deletable')).width, closeTo(176.4, 0.1));

    await tester.pump(const Duration(seconds: 1)); // Advance delete icon animation by 1sec.

    expect(tester.getSize(find.widgetWithText(RawChip, 'Deletable')).width, closeTo(180.9, 0.1));
  });
}
