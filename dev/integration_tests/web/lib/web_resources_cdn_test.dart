// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:web/web.dart' as web;

Future<void> main() async {
  if (await testFetchResources()) {
    print('--- TEST SUCCEEDED ---');
  } else {
    print('--- TEST FAILED ---');
  }
}

// Attempt to load CanvasKit resources hosted on gstatic.
Future<bool> testFetchResources() async {
  const String engineVersion = String.fromEnvironment('TEST_FLUTTER_ENGINE_VERSION');
  if (engineVersion.isEmpty) {
    return false;
  }
  try {
    final web.Response response =
        await web.window
            .fetch(
              'https://www.gstatic.com/flutter-canvaskit/$engineVersion/canvaskit.js'.toJS,
              web.RequestInit(method: 'GET'),
            )
            .toDart;
    if (!response.ok) {
      return false;
    }
  } catch (err) {
    print(err);
    return false;
  }
  try {
    final web.Response response =
        await web.window
            .fetch(
              'https://www.gstatic.com/flutter-canvaskit/$engineVersion/canvaskit.wasm'.toJS,
              web.RequestInit(method: 'GET'),
            )
            .toDart;
    if (!response.ok) {
      return false;
    }
  } catch (err) {
    print(err);
    return false;
  }
  return true;
}
