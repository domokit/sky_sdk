// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:fuchsia.builtin';

String? OnEchoString(String? str) {
  print('Got echo string: $str');
  return str;
}

void main(List<String> args) {
  receiveEchoStringCallback = OnEchoString;
}
