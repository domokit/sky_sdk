// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_studio.dart';
import 'package:flutter_tools/src/android/android_studio_validator.dart';
import 'package:flutter_tools/src/base/config.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';

const String home = '/home/me';

final Platform linuxPlatform = FakePlatform(environment: <String, String>{'HOME': home});

void main() {
  late FileSystem fileSystem;
  late FakeProcessManager fakeProcessManager;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    fakeProcessManager = FakeProcessManager.empty();
  });

  group(NoAndroidStudioValidator, () {
    testWithoutContext('shows Android Studio as "not available" when not available.', () async {
      final Config config = Config.test();
      final NoAndroidStudioValidator validator = NoAndroidStudioValidator(
        config: config,
        platform: linuxPlatform,
        userMessages: UserMessages(),
      );

      expect((await validator.validate()).type, equals(ValidationType.notAvailable));
    });
  });

  group(AndroidStudioValidator, () {
    testUsingContext(
      'gives doctor error on java crash',
      () async {
        fakeProcessManager.addCommand(
          const FakeCommand(
            command: <String>['/opt/android-studio-with-cheese-5.0/jre/bin/java', '-version'],
            exception: ProcessException('java', <String>['-version']),
          ),
        );
        const String installPath = '/opt/android-studio-with-cheese-5.0';
        const String studioHome = '$home/.AndroidStudioWithCheese5.0';
        const String homeFile = '$studioHome/system/.home';
        globals.fs.directory(installPath).createSync(recursive: true);
        globals.fs.file(homeFile).createSync(recursive: true);
        globals.fs.file(homeFile).writeAsStringSync(installPath);

        // This checks that running the validator doesn't throw an unhandled
        // exception and that the ProcessException makes it into the error
        // message list.
        for (final DoctorValidator validator in AndroidStudioValidator.allValidators(
          globals.config,
          globals.platform,
          globals.fs,
          globals.userMessages,
        )) {
          final ValidationResult result = await validator.validate();
          expect(
            result.messages.where((ValidationMessage message) {
              return message.isError && message.message.contains('ProcessException');
            }).isNotEmpty,
            true,
          );
        }
        expect(fakeProcessManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => fakeProcessManager,
        Platform: () => linuxPlatform,
        FileSystemUtils: () => FileSystemUtils(fileSystem: fileSystem, platform: linuxPlatform),
      },
    );

    testWithoutContext('warns if version of Android Studio could not be determined', () async {
      final AndroidStudio studio = _FakeAndroidStudio();
      final AndroidStudioValidator validator = AndroidStudioValidator(
        studio,
        fileSystem: fileSystem,
        userMessages: UserMessages(),
      );
      final ValidationResult result = await validator.validate();
      expect(
        result.messages,
        contains(const ValidationMessage.error('Unable to determine Android Studio version.')),
      );
      expect(result.statusInfo, 'version unknown');
    });
  });
}

class _FakeAndroidStudio extends Fake implements AndroidStudio {
  @override
  List<String> get validationMessages => <String>[];
  @override
  bool get isValid => true;
  @override
  String? get pluginsPath => null;
  @override
  String get directory => 'android-studio';
  @override
  Version? get version => null;
  @override
  String get javaPath => 'android-studio/jbr/bin/java';
}
