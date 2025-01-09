// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/clipboard_utils.dart';

void main() {
  final MockClipboard mockClipboard = MockClipboard();
  TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
    SystemChannels.platform,
    mockClipboard.handleMethodCall,
  );

  test('Clipboard.getData returns text', () async {
    mockClipboard.clipboardData = <String, dynamic>{'text': 'Hello world'};

    final ClipboardData? data = await Clipboard.getData(Clipboard.kTextPlain);

    expect(data, isNotNull);
    expect(data!.text, equals('Hello world'));
  });

  test('Clipboard.getData returns null', () async {
    mockClipboard.clipboardData = null;

    final ClipboardData? data = await Clipboard.getData(Clipboard.kTextPlain);

    expect(data, isNull);
  });

  test('Clipboard.getData throws if text is missing', () async {
    mockClipboard.clipboardData = <String, dynamic>{};

    expect(() => Clipboard.getData(Clipboard.kTextPlain), throwsA(isA<TypeError>()));
  });

  test('Clipboard.getData throws if text is null', () async {
    mockClipboard.clipboardData = <String, dynamic>{'text': null};

    expect(() => Clipboard.getData(Clipboard.kTextPlain), throwsA(isA<TypeError>()));
  });

  test('Clipboard.setData sets text', () async {
    await Clipboard.setData(const ClipboardData(text: 'Hello world'));

    expect(mockClipboard.clipboardData, <String, dynamic>{'text': 'Hello world'});
  });
}
