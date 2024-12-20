// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/semantics.dart';
import 'package:flutter/services.dart' show SystemChannels;
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  test('Semantic announcement', () async {
    final List<Map<dynamic, dynamic>> log = <Map<dynamic, dynamic>>[];

    Future<dynamic> handleMessage(dynamic mockMessage) async {
      final Map<dynamic, dynamic> message = mockMessage as Map<dynamic, dynamic>;
      log.add(message);
    }

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
        .setMockDecodedMessageHandler<dynamic>(SystemChannels.accessibility, handleMessage);

    await SemanticsService.announce('announcement 1', TextDirection.ltr);
    await SemanticsService.announce(
      'announcement 2',
      TextDirection.rtl,
      assertiveness: Assertiveness.assertive,
    );
    expect(
      log,
      equals(<Map<String, dynamic>>[
        <String, dynamic>{
          'type': 'announce',
          'data': <String, dynamic>{'message': 'announcement 1', 'textDirection': 1},
        },
        <String, dynamic>{
          'type': 'announce',
          'data': <String, dynamic>{
            'message': 'announcement 2',
            'textDirection': 0,
            'assertiveness': 1,
          },
        },
      ]),
    );
  });
}
