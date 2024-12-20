// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:js_interop';
import 'dart:js_interop_unsafe';

@JS('window')
external JSObject get _window;

/// The web implementation of [registerWebServiceExtension].
///
/// Registers Web Service Extension for Flutter Web application.
///
/// window.$flutterDriver will be called by Flutter Web Driver to process
/// Flutter Command.
void registerWebServiceExtension(Future<Map<String, dynamic>> Function(Map<String, String>) call) {
  // Define the result variable because packages/flutter_driver/lib/src/driver/web_driver.dart
  // checks for this value to become non-null when waiting for the result. If this value is
  // undefined at the time of the check, WebDriver throws an exception.
  _window.setProperty(r'$flutterDriverResult'.toJS, null);

  _window.setProperty(
    r'$flutterDriver'.toJS,
    (JSAny message) {
      final Map<String, String> params = Map<String, String>.from(
        jsonDecode((message as JSString).toDart) as Map<String, dynamic>,
      );
      call(params).then((Map<String, dynamic> result) {
        _window.setProperty(r'$flutterDriverResult'.toJS, json.encode(result).toJS);
      });
    }.toJS,
  );
}
