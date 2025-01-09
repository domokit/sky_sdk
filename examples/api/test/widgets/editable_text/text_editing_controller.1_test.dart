// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/editable_text/text_editing_controller.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Initial selection is collapsed at offset 0', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TextEditingControllerExampleApp());

    final EditableText editableText = tester.widget(find.byType(EditableText));
    final TextEditingController controller = editableText.controller;

    expect(controller.text, 'Flutter');
    expect(controller.selection, const TextSelection.collapsed(offset: 0));
  });
}
