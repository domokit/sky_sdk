// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:flutter/foundation.dart';
import 'package:web/web.dart' as web;

Future<void> main() async {
  bool executedAssert = false;
  assert(() {
    executedAssert = true;
    return true;
  }());

  final StringBuffer output = StringBuffer();
  if (executedAssert == kDebugMode) {
    output.write('--- TEST SUCCEEDED ---');
  } else {
    output.write('--- TEST FAILED ---');
  }

  await web.window
      .fetch('/test-result'.toJS, web.RequestInit(method: 'POST', body: '$output'.toJS))
      .toDart;
  print(output);
}
