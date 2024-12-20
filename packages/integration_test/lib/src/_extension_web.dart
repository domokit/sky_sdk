// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:js_interop';
import 'dart:js_interop_unsafe';

@JS('window')
external JSObject get _window;

/// The web implementation of [registerWebServiceExtension].
///
/// Adds a hidden `window.$flutterDriver` JavaScript function that's called by
/// `FlutterWebDriver` to fulfill FlutterDriver commands.
///
/// See also:
///
///  * `_extension_io.dart`, which has the dart:io implementation
void registerWebServiceExtension(
  Future<Map<String, dynamic>> Function(Map<String, String>) callback,
) {
  // Define the result variable because packages/flutter_driver/lib/src/driver/web_driver.dart
  // checks for this value to become non-null when waiting for the result. If this value is
  // undefined at the time of the check, WebDriver throws an exception.
  _window.setProperty(r'$flutterDriverResult'.toJS, null);

  _window.setProperty(
    r'$flutterDriver'.toJS,
    (JSAny message) {
      (() async {
        try {
          final Map<String, dynamic> messageJson =
              jsonDecode((message as JSString).toDart) as Map<String, dynamic>;
          final Map<String, String> params = messageJson.cast<String, String>();
          final Map<String, dynamic> result = await callback(params);
          _window.setProperty(r'$flutterDriverResult'.toJS, json.encode(result).toJS);
        } catch (error, stackTrace) {
          // Encode the error in the same format the FlutterDriver extension uses.
          // See //packages/flutter_driver/lib/src/extension/extension.dart
          _window.setProperty(
            r'$flutterDriverResult'.toJS,
            json.encode(<String, dynamic>{'isError': true, 'response': '$error\n$stackTrace'}).toJS,
          );
        }
      })();
    }.toJS,
  );
}
