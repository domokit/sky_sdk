// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter_test/flutter_test.dart';

void main() {
  test('SemanticsHandle ensureSemantics calls setSemanticsTreeEnabled', () async {
    final SemanticsTestBinding binding = SemanticsTestBinding();
    expect(binding.platformDispatcher.semanticsTreeEnabled, isFalse);
    final SemanticsHandle handle = binding.ensureSemantics();
    expect(binding.platformDispatcher.semanticsTreeEnabled, isTrue);
    handle.dispose();
    expect(binding.platformDispatcher.semanticsTreeEnabled, isFalse);
  });
}

class SemanticsTestBinding extends AutomatedTestWidgetsFlutterBinding {
  @override
  TestPlatformDispatcherSpy get platformDispatcher => _platformDispatcherSpy;
  static final TestPlatformDispatcherSpy _platformDispatcherSpy = TestPlatformDispatcherSpy(
    platformDispatcher: PlatformDispatcher.instance,
  );
}

class TestPlatformDispatcherSpy extends TestPlatformDispatcher {
  TestPlatformDispatcherSpy({required super.platformDispatcher});
  bool semanticsTreeEnabled = false;
  @override
  void setSemanticsTreeEnabled(bool enabled) {
    semanticsTreeEnabled = enabled;
  }
}
