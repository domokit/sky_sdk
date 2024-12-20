// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/editable_text/editable_text.on_content_inserted.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Image.memory displays inserted content', (WidgetTester tester) async {
    await tester.pumpWidget(const example.KeyboardInsertedContentApp());

    expect(find.text('Keyboard Inserted Content Sample'), findsOneWidget);

    await tester.tap(find.byType(EditableText));
    await tester.enterText(find.byType(EditableText), 'test');
    await tester.idle();

    const String uri = 'content://com.google.android.inputmethod.latin.fileprovider/test.png';
    const List<int> kBlueSquarePng = <int>[
      0x89,
      0x50,
      0x4e,
      0x47,
      0x0d,
      0x0a,
      0x1a,
      0x0a,
      0x00,
      0x00,
      0x00,
      0x0d,
      0x49,
      0x48,
      0x44,
      0x52,
      0x00,
      0x00,
      0x00,
      0x32,
      0x00,
      0x00,
      0x00,
      0x32,
      0x08,
      0x06,
      0x00,
      0x00,
      0x00,
      0x1e,
      0x3f,
      0x88,
      0xb1,
      0x00,
      0x00,
      0x00,
      0x48,
      0x49,
      0x44,
      0x41,
      0x54,
      0x78,
      0xda,
      0xed,
      0xcf,
      0x31,
      0x0d,
      0x00,
      0x30,
      0x08,
      0x00,
      0xb0,
      0x61,
      0x63,
      0x2f,
      0xfe,
      0x2d,
      0x61,
      0x05,
      0x34,
      0xf0,
      0x92,
      0xd6,
      0x41,
      0x23,
      0x7f,
      0xf5,
      0x3b,
      0x20,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x44,
      0x36,
      0x06,
      0x03,
      0x6e,
      0x69,
      0x47,
      0x12,
      0x8e,
      0xea,
      0xaa,
      0x00,
      0x00,
      0x00,
      0x00,
      0x49,
      0x45,
      0x4e,
      0x44,
      0xae,
      0x42,
      0x60,
      0x82,
    ];
    final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
      'args': <dynamic>[
        -1,
        'TextInputAction.commitContent',
        jsonDecode('{"mimeType": "image/png", "data": $kBlueSquarePng, "uri": "$uri"}'),
      ],
      'method': 'TextInputClient.performAction',
    });

    try {
      await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/textinput',
        messageBytes,
        (ByteData? _) {},
      );
    } catch (_) {}

    await tester.pumpAndSettle();
    expect(find.byType(Image), findsOneWidget);
  });
}
