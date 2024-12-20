// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/windows/visual_studio.dart';
import 'package:flutter_tools/src/windows/visual_studio_validator.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';

void main() {
  group('Visual Studio validation', () {
    late FakeVisualStudio fakeVisualStudio;
    final UserMessages userMessages = UserMessages();

    setUp(() {
      fakeVisualStudio = FakeVisualStudio();
    });

    // Assigns default values for a complete VS installation with necessary components.
    void configureMockVisualStudioAsInstalled() {
      fakeVisualStudio.isPrerelease = false;
      fakeVisualStudio.isRebootRequired = false;
      fakeVisualStudio.fullVersion = '16.2';
      fakeVisualStudio.displayName = 'Visual Studio Community 2019';
      fakeVisualStudio.windows10SDKVersion = '10.0.18362.0';
    }

    // Assigns default values for a complete VS installation that is too old.
    void configureMockVisualStudioAsTooOld() {
      fakeVisualStudio.isAtLeastMinimumVersion = false;
      fakeVisualStudio.isPrerelease = false;
      fakeVisualStudio.isRebootRequired = false;
      fakeVisualStudio.fullVersion = '15.1';
      fakeVisualStudio.displayName = 'Visual Studio Community 2017';
      fakeVisualStudio.windows10SDKVersion = '10.0.17763.0';
    }

    // Assigns default values for a missing VS installation.
    void configureMockVisualStudioAsNotInstalled() {
      fakeVisualStudio.isInstalled = false;
      fakeVisualStudio.isAtLeastMinimumVersion = false;
      fakeVisualStudio.isPrerelease = false;
      fakeVisualStudio.isComplete = false;
      fakeVisualStudio.isLaunchable = false;
      fakeVisualStudio.isRebootRequired = false;
      fakeVisualStudio.hasNecessaryComponents = false;
      fakeVisualStudio.windows10SDKVersion = null;
    }

    testWithoutContext('Emits a message when Visual Studio is a pre-release version', () async {
      final VisualStudioValidator validator = VisualStudioValidator(
        userMessages: userMessages,
        visualStudio: fakeVisualStudio,
      );
      configureMockVisualStudioAsInstalled();
      fakeVisualStudio.isPrerelease = true;

      final ValidationResult result = await validator.validate();
      const ValidationMessage expectedMessage = ValidationMessage(
        'The current Visual Studio installation is a pre-release version. '
        'It may not be supported by Flutter yet.',
      );

      expect(result.messages, contains(expectedMessage));
    });

    testWithoutContext(
      'Emits a partial status when Visual Studio installation is incomplete',
      () async {
        final VisualStudioValidator validator = VisualStudioValidator(
          userMessages: userMessages,
          visualStudio: fakeVisualStudio,
        );
        configureMockVisualStudioAsInstalled();
        fakeVisualStudio.isComplete = false;

        final ValidationResult result = await validator.validate();
        const ValidationMessage expectedMessage = ValidationMessage.error(
          'The current Visual Studio installation is incomplete.\n'
          'Please use Visual Studio Installer to complete the installation or reinstall Visual Studio.',
        );

        expect(result.messages, contains(expectedMessage));
        expect(result.type, ValidationType.partial);
      },
    );

    testWithoutContext(
      'Emits a partial status when Visual Studio installation needs rebooting',
      () async {
        final VisualStudioValidator validator = VisualStudioValidator(
          userMessages: userMessages,
          visualStudio: fakeVisualStudio,
        );
        configureMockVisualStudioAsInstalled();
        fakeVisualStudio.isRebootRequired = true;

        final ValidationResult result = await validator.validate();
        const ValidationMessage expectedMessage = ValidationMessage.error(
          'Visual Studio requires a reboot of your system to complete installation.',
        );

        expect(result.messages, contains(expectedMessage));
        expect(result.type, ValidationType.partial);
      },
    );

    testWithoutContext(
      'Emits a partial status when Visual Studio installation is not launchable',
      () async {
        final VisualStudioValidator validator = VisualStudioValidator(
          userMessages: userMessages,
          visualStudio: fakeVisualStudio,
        );
        configureMockVisualStudioAsInstalled();
        fakeVisualStudio.isLaunchable = false;

        final ValidationResult result = await validator.validate();
        const ValidationMessage expectedMessage = ValidationMessage.error(
          'The current Visual Studio installation is not launchable. Please reinstall Visual Studio.',
        );

        expect(result.messages, contains(expectedMessage));
        expect(result.type, ValidationType.partial);
      },
    );

    testWithoutContext('Emits partial status when Visual Studio is installed but too old', () async {
      final VisualStudioValidator validator = VisualStudioValidator(
        userMessages: userMessages,
        visualStudio: fakeVisualStudio,
      );
      configureMockVisualStudioAsTooOld();

      final ValidationResult result = await validator.validate();
      const ValidationMessage expectedMessage = ValidationMessage.error(
        'Visual Studio 2019 or later is required.\n'
        'Download at https://visualstudio.microsoft.com/downloads/.\n'
        'Please install the "Desktop development" workload, including all of its default components',
      );

      expect(result.messages, contains(expectedMessage));
      expect(result.type, ValidationType.partial);
    });

    testWithoutContext(
      'Emits partial status when Visual Studio is installed without necessary components',
      () async {
        final VisualStudioValidator validator = VisualStudioValidator(
          userMessages: userMessages,
          visualStudio: fakeVisualStudio,
        );
        configureMockVisualStudioAsInstalled();
        fakeVisualStudio.hasNecessaryComponents = false;
        final ValidationResult result = await validator.validate();

        expect(result.type, ValidationType.partial);
      },
    );

    testWithoutContext(
      'Emits partial status when Visual Studio is installed but the SDK cannot be found',
      () async {
        final VisualStudioValidator validator = VisualStudioValidator(
          userMessages: userMessages,
          visualStudio: fakeVisualStudio,
        );
        configureMockVisualStudioAsInstalled();
        fakeVisualStudio.windows10SDKVersion = null;
        final ValidationResult result = await validator.validate();

        expect(result.type, ValidationType.partial);
      },
    );

    testWithoutContext(
      'Emits installed status when Visual Studio is installed with necessary components',
      () async {
        final VisualStudioValidator validator = VisualStudioValidator(
          userMessages: userMessages,
          visualStudio: fakeVisualStudio,
        );
        configureMockVisualStudioAsInstalled();

        final ValidationResult result = await validator.validate();
        const ValidationMessage expectedDisplayNameMessage = ValidationMessage(
          'Visual Studio Community 2019 version 16.2',
        );

        expect(result.messages, contains(expectedDisplayNameMessage));
        expect(result.type, ValidationType.success);
      },
    );

    testWithoutContext('Emits missing status when Visual Studio is not installed', () async {
      final VisualStudioValidator validator = VisualStudioValidator(
        userMessages: userMessages,
        visualStudio: fakeVisualStudio,
      );
      configureMockVisualStudioAsNotInstalled();

      final ValidationResult result = await validator.validate();
      const ValidationMessage expectedMessage = ValidationMessage.error(
        'Visual Studio not installed; this is necessary to develop Windows apps.\n'
        'Download at https://visualstudio.microsoft.com/downloads/.\n'
        'Please install the "Desktop development" workload, including all of its default components',
      );

      expect(result.messages, contains(expectedMessage));
      expect(result.type, ValidationType.missing);
    });
  });
}

class FakeVisualStudio extends Fake implements VisualStudio {
  @override
  final String installLocation = 'bogus';

  @override
  final String displayVersion = 'version';

  @override
  final String minimumVersionDescription = '2019';

  @override
  List<String> necessaryComponentDescriptions() => <String>['A', 'B'];

  @override
  bool isInstalled = true;

  @override
  bool isAtLeastMinimumVersion = true;

  @override
  bool isPrerelease = true;

  @override
  bool isComplete = true;

  @override
  bool isLaunchable = true;

  @override
  bool isRebootRequired = true;

  @override
  bool hasNecessaryComponents = true;

  @override
  String? fullVersion;

  @override
  String? displayName;

  String? windows10SDKVersion;

  @override
  String? getWindows10SDKVersion() => windows10SDKVersion;

  @override
  String get workloadDescription => 'Desktop development';
}
