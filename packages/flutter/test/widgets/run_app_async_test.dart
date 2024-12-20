// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:fake_async/fake_async.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

// This test is very fragile and bypasses some zone-related checks.
// It is written this way to verify some invariants that would otherwise
// be difficult to check.
// Do not use this test as a guide for writing good Flutter code.

class TestBinding extends WidgetsFlutterBinding {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
  }

  @override
  bool debugCheckZone(String entryPoint) {
    return true;
  }

  static TestBinding get instance => BindingBase.checkInstance(_instance);
  static TestBinding? _instance;

  static TestBinding ensureInitialized() {
    if (TestBinding._instance == null) {
      TestBinding();
    }
    return TestBinding.instance;
  }
}

void main() {
  setUp(() {
    TestBinding.ensureInitialized();
    WidgetsBinding.instance.resetEpoch();
  });

  test('WidgetBinding build rendering tree and warm up frame back to back', () {
    final FakeAsync fakeAsync = FakeAsync();
    fakeAsync.run((FakeAsync async) {
      runApp(const MaterialApp(home: Material(child: Text('test'))));
      // Rendering tree is not built synchronously.
      expect(WidgetsBinding.instance.rootElement, isNull);
      fakeAsync.flushTimers();
      expect(WidgetsBinding.instance.rootElement, isNotNull);
    });
  });
}
