// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../convert.dart';

import 'test_device.dart';
import 'watcher.dart';

/// Prints JSON events when running a test in --machine mode.
class EventPrinter extends TestWatcher {
  EventPrinter({required StringSink out, TestWatcher? parent}) : _out = out, _parent = parent;

  final StringSink _out;
  final TestWatcher? _parent;

  @override
  void handleStartedDevice(Uri? vmServiceUri) {
    _sendEvent('test.startedProcess', <String, dynamic>{
      'vmServiceUri': vmServiceUri?.toString(),
      // TODO(bkonyi): remove references to Observatory
      // See https://github.com/flutter/flutter/issues/121271
      'observatoryUri': vmServiceUri?.toString(),
    });
    _parent?.handleStartedDevice(vmServiceUri);
  }

  @override
  Future<void> handleTestCrashed(TestDevice testDevice) async {
    return _parent?.handleTestCrashed(testDevice);
  }

  @override
  Future<void> handleTestTimedOut(TestDevice testDevice) async {
    return _parent?.handleTestTimedOut(testDevice);
  }

  @override
  Future<void> handleFinishedTest(TestDevice testDevice) async {
    return _parent?.handleFinishedTest(testDevice);
  }

  void _sendEvent(String name, [dynamic params]) {
    final Map<String, dynamic> map = <String, dynamic>{'event': name};
    if (params != null) {
      map['params'] = params;
    }
    _send(map);
  }

  void _send(Map<String, dynamic> command) {
    final String encoded = json.encode(command, toEncodable: _jsonEncodeObject);
    _out.writeln('\n[$encoded]');
  }

  dynamic _jsonEncodeObject(dynamic object) {
    if (object is Uri) {
      return object.toString();
    }
    return object;
  }
}
