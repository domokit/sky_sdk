// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:web/web.dart' as web;

// Attempt to load a file that is hosted in the applications's `web/` directory.
Future<void> main() async {
  try {
    final web.Response response =
        await web.window.fetch('/example'.toJS, web.RequestInit(method: 'GET')).toDart;
    final String body = (await response.text().toDart).toDart;
    if (body == 'This is an Example') {
      print('--- TEST SUCCEEDED ---');
    } else {
      print('--- TEST FAILED ---');
    }
  } catch (err) {
    print(err);
    print('--- TEST FAILED ---');
  }
}
