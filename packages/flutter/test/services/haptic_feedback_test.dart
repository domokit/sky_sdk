// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  test('Haptic feedback control test', () async {
    final List<MethodCall> log = <MethodCall>[];

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        log.add(methodCall);
        return null;
      },
    );

    await HapticFeedback.vibrate();

    expect(log, hasLength(1));
    expect(log.single, isMethodCall('HapticFeedback.vibrate', arguments: null));
  });

  test('Haptic feedback variation tests', () async {
    Future<void> callAndVerifyHapticFunction(
      Future<void> Function() hapticFunction,
      String platformMethodArgument,
    ) async {
      final List<MethodCall> log = <MethodCall>[];

      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        (MethodCall methodCall) async {
          log.add(methodCall);
          return null;
        },
      );

      await hapticFunction();
      expect(log, hasLength(1));
      expect(log.last, isMethodCall('HapticFeedback.vibrate', arguments: platformMethodArgument));
    }

    await callAndVerifyHapticFunction(HapticFeedback.lightImpact, 'HapticFeedbackType.lightImpact');
    await callAndVerifyHapticFunction(
      HapticFeedback.mediumImpact,
      'HapticFeedbackType.mediumImpact',
    );
    await callAndVerifyHapticFunction(HapticFeedback.heavyImpact, 'HapticFeedbackType.heavyImpact');
    await callAndVerifyHapticFunction(
      HapticFeedback.selectionClick,
      'HapticFeedbackType.selectionClick',
    );
  });
}
