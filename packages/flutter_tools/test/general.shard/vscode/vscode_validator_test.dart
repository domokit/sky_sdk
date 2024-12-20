// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/vscode/vscode.dart';
import 'package:flutter_tools/src/vscode/vscode_validator.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';

void main() {
  testWithoutContext('VsCode search locations on windows supports an empty environment', () {
    final FileSystem fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
    final Platform platform = FakePlatform(
      operatingSystem: 'windows',
      environment: <String, String>{},
    );

    expect(VsCode.allInstalled(fileSystem, platform, FakeProcessManager.any()), isEmpty);
  });

  group(VsCodeValidator, () {
    testUsingContext(
      'Warns if VS Code version could not be found',
      () async {
        final VsCodeValidator validator = VsCodeValidator(_FakeVsCode());
        final ValidationResult result = await validator.validate();
        expect(
          result.messages,
          contains(const ValidationMessage.error('Unable to determine VS Code version.')),
        );
        expect(result.statusInfo, 'version unknown');
      },
      overrides: <Type, Generator>{UserMessages: () => UserMessages()},
    );
  });
}

class _FakeVsCode extends Fake implements VsCode {
  @override
  Iterable<ValidationMessage> get validationMessages => <ValidationMessage>[];

  @override
  String get productName => 'VS Code';

  @override
  Version? get version => null;
}
