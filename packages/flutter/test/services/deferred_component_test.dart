// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  test('installDeferredComponent test', () async {
    final List<MethodCall> log = <MethodCall>[];

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.deferredComponent,
      (MethodCall methodCall) async {
        log.add(methodCall);
        return null;
      },
    );

    await DeferredComponent.installDeferredComponent(componentName: 'testComponentName');

    expect(log, hasLength(1));
    expect(
      log.single,
      isMethodCall(
        'installDeferredComponent',
        arguments: <String, dynamic>{'loadingUnitId': -1, 'componentName': 'testComponentName'},
      ),
    );
  });

  test('uninstallDeferredComponent test', () async {
    final List<MethodCall> log = <MethodCall>[];

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.deferredComponent,
      (MethodCall methodCall) async {
        log.add(methodCall);
        return null;
      },
    );

    await DeferredComponent.uninstallDeferredComponent(componentName: 'testComponentName');

    expect(log, hasLength(1));
    expect(
      log.single,
      isMethodCall(
        'uninstallDeferredComponent',
        arguments: <String, dynamic>{'loadingUnitId': -1, 'componentName': 'testComponentName'},
      ),
    );
  });
}
