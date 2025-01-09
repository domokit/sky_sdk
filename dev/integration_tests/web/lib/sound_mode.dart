// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// @dart = 2.12

import 'dart:js_interop';

import 'package:web/web.dart' as web;

// Verify that web applications can be run in sound mode.
void main() async {
  const bool isWeak = <int?>[] is List<int>;
  String output;
  if (isWeak) {
    output = '--- TEST FAILED ---';
  } else {
    output = '--- TEST SUCCEEDED ---';
  }
  await web.window
      .fetch(
          '/test-result'.toJS,
          web.RequestInit(
            method: 'POST',
            body: output.toJS,
          ))
      .toDart;
  print(output);
}
