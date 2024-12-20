// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'project.dart';

class TestProject extends Project {
  @override
  final String pubspec = '''
  name: test
  environment:
    sdk: ^3.7.0-0

  dependencies:
    flutter:
      sdk: flutter
  dev_dependencies:
    flutter_test:
      sdk: flutter
  ''';

  @override
  final String main = r'''
int foo(int bar) {
  return bar + 2;
}
  ''';

  @override
  final String test = r'''
  import 'package:flutter_test/flutter_test.dart';
  import 'package:test/main.dart';

  void main() {
    testWidgets('it can test', (WidgetTester tester) async {
      expect(foo(2), 4);
    });
  }
''';
}
