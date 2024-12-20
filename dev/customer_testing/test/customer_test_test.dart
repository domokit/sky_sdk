// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:customer_testing/customer_test.dart';
import 'package:file/file.dart';
import 'package:file/memory.dart';

import 'common.dart';

void main() {
  test('constructs expected model', () async {
    const String registryContent = '''
contact=abc@gmail.com
fetch=git clone https://github.com/flutter/cocoon.git tests
fetch=git -C tests checkout abc123
setup=flutter --version
setup.windows=flutter doctor
setup.posix=flutter -h
setup.linux=flutter analyze -h
setup.macos=flutter build -h
update=.
# Runs flutter analyze, flutter test, and builds web platform
test.posix=./test_utilities/bin/flutter_test_runner.sh app_flutter
test.posix=./test_utilities/bin/flutter_test_runner.sh repo_dashboard
test.windows=.\test_utilities\bin\flutter_test_runner.bat repo_dashboard
    ''';
    final File registryFile = MemoryFileSystem().file('flutter_cocoon.test')
      ..writeAsStringSync(registryContent);

    final CustomerTest test = CustomerTest(registryFile);
    expect(test.contacts, containsAll(<String>['abc@gmail.com']));
    expect(
      test.fetch,
      containsAllInOrder(<String>[
        'git clone https://github.com/flutter/cocoon.git tests',
        'git -C tests checkout abc123',
      ]),
    );
    expect(test.setup.first, 'flutter --version');
    if (Platform.isLinux || Platform.isMacOS) {
      expect(test.setup.length, 3);
      expect(test.setup[1], 'flutter -h');
      expect(test.setup[2], Platform.isLinux ? 'flutter analyze -h' : 'flutter build -h');
      expect(
        test.tests,
        containsAllInOrder(<String>[
          './test_utilities/bin/flutter_test_runner.sh app_flutter',
          './test_utilities/bin/flutter_test_runner.sh repo_dashboard',
        ]),
      );
    } else if (Platform.isWindows) {
      expect(test.setup.length, 2);
      expect(test.setup[1], 'flutter doctor');
      expect(
        test.tests,
        containsAllInOrder(<String>['.\test_utilities\bin\flutter_test_runner.bat repo_dashboard']),
      );
    }
  });

  test('throws exception when unknown field is passed', () async {
    const String registryContent = '''
contact=abc@gmail.com
update=.
fetch=git clone https://github.com/flutter/cocoon.git tests
fetch=git -C tests checkout abc123
test.posix=./test_utilities/bin/flutter_test_runner.sh app_flutter
test.windows=.\test_utilities\bin\flutter_test_runner.bat repo_dashboard
unknownfield=super not cool
    ''';
    final File registryFile = MemoryFileSystem().file('abc.test')
      ..writeAsStringSync(registryContent);

    expect(() => CustomerTest(registryFile), throwsFormatException);
  });

  test('throws exception when no tests given', () async {
    const String registryContent = '''
contact=abc@gmail.com
update=.
fetch=git clone https://github.com/flutter/cocoon.git tests
''';
    final File registryFile = MemoryFileSystem().file('abc.test')
      ..writeAsStringSync(registryContent);

    expect(() => CustomerTest(registryFile), throwsFormatException);
  });

  test('throws exception when only one fetch instruction given', () async {
    const String registryContent = '''
contact=abc@gmail.com
update=.
fetch=git clone https://github.com/flutter/cocoon.git tests
test.posix=./test_utilities/bin/flutter_test_runner.sh app_flutter
test.windows=.\test_utilities\bin\flutter_test_runner.bat repo_dashboard
    ''';
    final File registryFile = MemoryFileSystem().file('abc.test')
      ..writeAsStringSync(registryContent);

    expect(() => CustomerTest(registryFile), throwsFormatException);
  });

  test('throws exception when no contacts given', () async {
    const String registryContent = '''
update=.
fetch=git clone https://github.com/flutter/cocoon.git tests
test.posix=./test_utilities/bin/flutter_test_runner.sh app_flutter
test.windows=.\test_utilities\bin\flutter_test_runner.bat repo_dashboard
    ''';
    final File registryFile = MemoryFileSystem().file('abc.test')
      ..writeAsStringSync(registryContent);

    expect(() => CustomerTest(registryFile), throwsFormatException);
  });
}
