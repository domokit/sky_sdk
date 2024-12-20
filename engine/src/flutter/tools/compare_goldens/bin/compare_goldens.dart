// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' show exitCode;
import 'package:compare_goldens/compare_goldens.dart' as compare_goldens;

void main(List<String> args) {
  exitCode = compare_goldens.run(args);
}
