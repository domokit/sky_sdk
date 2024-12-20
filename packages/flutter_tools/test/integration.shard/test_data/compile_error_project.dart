// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'project.dart';

class CompileErrorProject extends Project {
  @override
  final String pubspec = '''
  name: test
  environment:
    sdk: ^3.7.0-0

  dependencies:
    flutter:
      sdk: flutter
  ''';

  @override
  final String main = r'''
  import 'dart:async';

  import 'package:flutter/material.dart';

  Future<void> main() async {
    this code does not compile
  }
  ''';
}
