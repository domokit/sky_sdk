// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';
import 'dart:js_interop_unsafe';

import 'package:flutter_driver/src/extension/_extension_web.dart';
import 'package:flutter_test/flutter_test.dart';

@JS('window')
external JSObject get _window;

void main() {
  group('test web_extension', () {
    late Future<Map<String, dynamic>> Function(Map<String, String>) call;

    setUp(() {
      call = (Map<String, String> args) async {
        return Future<Map<String, dynamic>>.value(args);
      };
    });

    test('web_extension should register a function', () {
      expect(() => registerWebServiceExtension(call), returnsNormally);

      expect(_window.hasProperty(r'$flutterDriver'.toJS).toDart, true);
      expect(_window.getProperty(r'$flutterDriver'.toJS), isNotNull);

      expect(_window.hasProperty(r'$flutterDriverResult'.toJS).toDart, true);
      expect(_window.getProperty(r'$flutterDriverResult'.toJS), isNull);
    });
  });
}
