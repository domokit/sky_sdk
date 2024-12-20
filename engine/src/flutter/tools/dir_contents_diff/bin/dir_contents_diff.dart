// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' show exitCode;

import 'package:dir_contents_diff/dir_contents_diff.dart';

void main(List<String> args) {
  exitCode = run(args);
}
