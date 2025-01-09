// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:fake_async/fake_async.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_studio_validator.dart';
import 'package:flutter_tools/src/android/android_workflow.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/base/time.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/custom_devices/custom_device_workflow.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/doctor.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:flutter_tools/src/version.dart';
import 'package:flutter_tools/src/vscode/vscode.dart';
import 'package:flutter_tools/src/vscode/vscode_validator.dart';
import 'package:flutter_tools/src/web/workflow.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';

void main() {
  late BufferLogger logger;
  late FakeProcessManager fakeProcessManager;
  late MemoryFileSystem fs;

  setUp(() {
    logger = BufferLogger.test();
    fakeProcessManager = FakeProcessManager.empty();
    fs = MemoryFileSystem.test();
  });

  testWithoutContext('ValidationMessage equality and hashCode includes contextUrl', () {
    const ValidationMessage messageA = ValidationMessage('ab', contextUrl: 'a');
    const ValidationMessage messageB = ValidationMessage('ab', contextUrl: 'b');

    expect(messageB, isNot(messageA));
    expect(messageB.hashCode, isNot(messageA.hashCode));
    expect(messageA, isNot(messageB));
    expect(messageA.hashCode, isNot(messageB.hashCode));
  });

  group('doctor', () {
    testUsingContext('vs code validator when both installed', () async {
      final ValidationResult result =
          await VsCodeValidatorTestTargets.installedWithExtension.validate();
      expect(result.type, ValidationType.success);
      expect(result.statusInfo, 'version 1.2.3');
      expect(result.messages, hasLength(2));

      ValidationMessage message = result.messages.firstWhere(
        (ValidationMessage m) => m.message.startsWith('VS Code '),
      );
      expect(message.message, 'VS Code at ${VsCodeValidatorTestTargets.validInstall}');

      message = result.messages.firstWhere(
        (ValidationMessage m) => m.message.startsWith('Flutter '),
      );
      expect(message.message, 'Flutter extension version 4.5.6');
      expect(message.isError, isFalse);
    });

    testUsingContext('No IDE Validator includes expected installation messages', () async {
      final ValidationResult result = await NoIdeValidator().validate();
      expect(result.type, ValidationType.notAvailable);

      expect(
        result.messages.map((ValidationMessage vm) => vm.message),
        UserMessages().noIdeInstallationInfo,
      );
    });

    testUsingContext('vs code validator when 64bit installed', () async {
      expect(
        VsCodeValidatorTestTargets.installedWithExtension64bit.title,
        'VS Code, 64-bit edition',
      );
      final ValidationResult result =
          await VsCodeValidatorTestTargets.installedWithExtension64bit.validate();
      expect(result.type, ValidationType.success);
      expect(result.statusInfo, 'version 1.2.3');
      expect(result.messages, hasLength(2));

      ValidationMessage message = result.messages.firstWhere(
        (ValidationMessage m) => m.message.startsWith('VS Code '),
      );
      expect(message.message, 'VS Code at ${VsCodeValidatorTestTargets.validInstall}');

      message = result.messages.firstWhere(
        (ValidationMessage m) => m.message.startsWith('Flutter '),
      );
      expect(message.message, 'Flutter extension version 4.5.6');
    });

    testUsingContext('vs code validator when extension missing', () async {
      final ValidationResult result =
          await VsCodeValidatorTestTargets.installedWithoutExtension.validate();
      expect(result.type, ValidationType.success);
      expect(result.statusInfo, 'version 1.2.3');
      expect(result.messages, hasLength(2));

      ValidationMessage message = result.messages.firstWhere(
        (ValidationMessage m) => m.message.startsWith('VS Code '),
      );
      expect(message.message, 'VS Code at ${VsCodeValidatorTestTargets.validInstall}');

      message = result.messages.firstWhere(
        (ValidationMessage m) => m.message.startsWith('Flutter '),
      );
      expect(message.message, startsWith('Flutter extension can be installed from'));
      expect(
        message.contextUrl,
        'https://marketplace.visualstudio.com/items?itemName=Dart-Code.flutter',
      );
      expect(message.isError, false);
    });

    group('device validator', () {
      testWithoutContext('no devices', () async {
        final FakeDeviceManager deviceManager = FakeDeviceManager();
        final DeviceValidator deviceValidator = DeviceValidator(
          deviceManager: deviceManager,
          userMessages: UserMessages(),
        );
        final ValidationResult result = await deviceValidator.validate();
        expect(result.type, ValidationType.notAvailable);
        expect(result.messages, const <ValidationMessage>[
          ValidationMessage.hint('No devices available'),
        ]);
        expect(result.statusInfo, isNull);
      });

      testWithoutContext('diagnostic message', () async {
        final FakeDeviceManager deviceManager =
            FakeDeviceManager()..diagnostics = <String>['Device locked'];

        final DeviceValidator deviceValidator = DeviceValidator(
          deviceManager: deviceManager,
          userMessages: UserMessages(),
        );
        final ValidationResult result = await deviceValidator.validate();
        expect(result.type, ValidationType.notAvailable);
        expect(result.messages, const <ValidationMessage>[ValidationMessage.hint('Device locked')]);
        expect(result.statusInfo, isNull);
      });

      testWithoutContext('diagnostic message and devices', () async {
        final FakeDevice device = FakeDevice();
        final FakeDeviceManager deviceManager =
            FakeDeviceManager()
              ..devices = <Device>[device]
              ..diagnostics = <String>['Device locked'];

        final DeviceValidator deviceValidator = DeviceValidator(
          deviceManager: deviceManager,
          userMessages: UserMessages(),
        );
        final ValidationResult result = await deviceValidator.validate();
        expect(result.type, ValidationType.success);
        expect(result.messages, const <ValidationMessage>[
          ValidationMessage('name (mobile) • device-id • android • 1.2.3'),
          ValidationMessage.hint('Device locked'),
        ]);
        expect(result.statusInfo, '1 available');
      });
    });
  });

  group('doctor with overridden validators', () {
    testUsingContext(
      'validate non-verbose output format for run without issues',
      () async {
        final Doctor doctor = Doctor(logger: logger, clock: const SystemClock());
        expect(await doctor.diagnose(verbose: false), isTrue);
        expect(
          logger.statusText,
          equals(
            'Doctor summary (to see all details, run flutter doctor -v):\n'
            '[✓] Passing Validator (with statusInfo)\n'
            '[✓] Another Passing Validator (with statusInfo)\n'
            '[✓] Providing validators is fun (with statusInfo)\n'
            '\n'
            '• No issues found!\n',
          ),
        );
      },
      overrides: <Type, Generator>{
        AnsiTerminal: () => FakeTerminal(),
        DoctorValidatorsProvider: () => FakeDoctorValidatorsProvider(),
      },
    );
  });

  group('doctor usage params', () {
    late TestUsage testUsage;

    setUp(() {
      testUsage = TestUsage();
    });

    testUsingContext(
      'contains installed',
      () async {
        final Doctor doctor = Doctor(logger: logger, clock: const SystemClock());
        await doctor.diagnose(verbose: false);

        expect(testUsage.events.length, 3);
        expect(
          testUsage.events,
          contains(const TestUsageEvent('doctor-result', 'PassingValidator', label: 'installed')),
        );
      },
      overrides: <Type, Generator>{
        DoctorValidatorsProvider: () => FakeDoctorValidatorsProvider(),
        Usage: () => testUsage,
      },
    );

    testUsingContext('contains installed and partial', () async {
      await FakePassingDoctor(logger).diagnose(verbose: false);

      expect(
        testUsage.events,
        unorderedEquals(<TestUsageEvent>[
          const TestUsageEvent('doctor-result', 'PassingValidator', label: 'installed'),
          const TestUsageEvent('doctor-result', 'PassingValidator', label: 'installed'),
          const TestUsageEvent('doctor-result', 'PartialValidatorWithHintsOnly', label: 'partial'),
          const TestUsageEvent('doctor-result', 'PartialValidatorWithErrors', label: 'partial'),
        ]),
      );
    }, overrides: <Type, Generator>{Usage: () => testUsage});

    testUsingContext('contains installed, missing and partial', () async {
      await FakeDoctor(logger).diagnose(verbose: false);

      expect(
        testUsage.events,
        unorderedEquals(<TestUsageEvent>[
          const TestUsageEvent('doctor-result', 'PassingValidator', label: 'installed'),
          const TestUsageEvent('doctor-result', 'MissingValidator', label: 'missing'),
          const TestUsageEvent('doctor-result', 'NotAvailableValidator', label: 'notAvailable'),
          const TestUsageEvent('doctor-result', 'PartialValidatorWithHintsOnly', label: 'partial'),
          const TestUsageEvent('doctor-result', 'PartialValidatorWithErrors', label: 'partial'),
        ]),
      );
    }, overrides: <Type, Generator>{Usage: () => testUsage});

    testUsingContext(
      'events for grouped validators are properly decomposed',
      () async {
        await FakeGroupedDoctor(logger).diagnose(verbose: false);

        expect(
          testUsage.events,
          unorderedEquals(<TestUsageEvent>[
            const TestUsageEvent('doctor-result', 'PassingGroupedValidator', label: 'installed'),
            const TestUsageEvent('doctor-result', 'PassingGroupedValidator', label: 'installed'),
            const TestUsageEvent('doctor-result', 'PassingGroupedValidator', label: 'installed'),
            const TestUsageEvent('doctor-result', 'MissingGroupedValidator', label: 'missing'),
          ]),
        );
      },
      overrides: <Type, Generator>{Usage: () => testUsage},
    );

    testUsingContext('sending events can be skipped', () async {
      await FakePassingDoctor(logger).diagnose(verbose: false, sendEvent: false);

      expect(testUsage.events, isEmpty);
    }, overrides: <Type, Generator>{Usage: () => testUsage});
  });

  group('doctor with fake validators', () {
    testUsingContext(
      'validate non-verbose output format for run without issues',
      () async {
        expect(await FakeQuietDoctor(logger).diagnose(verbose: false), isTrue);
        expect(
          logger.statusText,
          equals(
            'Doctor summary (to see all details, run flutter doctor -v):\n'
            '[✓] Passing Validator (with statusInfo)\n'
            '[✓] Another Passing Validator (with statusInfo)\n'
            '[✓] Validators are fun (with statusInfo)\n'
            '[✓] Four score and seven validators ago (with statusInfo)\n'
            '\n'
            '• No issues found!\n',
          ),
        );
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate non-verbose output format for run with crash',
      () async {
        expect(await FakeCrashingDoctor(logger).diagnose(verbose: false), isFalse);
        expect(
          logger.statusText,
          equals(
            'Doctor summary (to see all details, run flutter doctor -v):\n'
            '[✓] Passing Validator (with statusInfo)\n'
            '[✓] Another Passing Validator (with statusInfo)\n'
            '[☠] Crashing validator (the doctor check crashed)\n'
            '    ✗ Due to an error, the doctor check did not complete. If the error message below is not helpful, '
            'please let us know about this issue at https://github.com/flutter/flutter/issues.\n'
            '    ✗ Bad state: fatal error\n'
            '[✓] Validators are fun (with statusInfo)\n'
            '[✓] Four score and seven validators ago (with statusInfo)\n'
            '\n'
            '! Doctor found issues in 1 category.\n',
          ),
        );
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext('validate verbose output format contains trace for run with crash', () async {
      expect(await FakeCrashingDoctor(logger).diagnose(), isFalse);
      expect(logger.statusText, contains('#0      CrashingValidator.validate'));
    });

    testUsingContext(
      'validate tool exit when exceeding timeout',
      () async {
        FakeAsync().run<void>((FakeAsync time) {
          final Doctor doctor = FakeAsyncStuckDoctor(logger);
          doctor.diagnose(verbose: false);
          time.elapse(const Duration(minutes: 5));
          time.flushMicrotasks();
        });

        expect(
          logger.statusText,
          contains('Stuck validator that never completes exceeded maximum allowed duration of '),
        );
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate non-verbose output format for run with an async crash',
      () async {
        final Completer<void> completer = Completer<void>();
        await FakeAsync().run((FakeAsync time) {
          unawaited(
            FakeAsyncCrashingDoctor(time, logger).diagnose(verbose: false).then((bool r) {
              expect(r, isFalse);
              completer.complete();
            }),
          );
          time.elapse(const Duration(seconds: 1));
          time.flushMicrotasks();
          return completer.future;
        });
        expect(
          logger.statusText,
          equals(
            'Doctor summary (to see all details, run flutter doctor -v):\n'
            '[✓] Passing Validator (with statusInfo)\n'
            '[✓] Another Passing Validator (with statusInfo)\n'
            '[☠] Async crashing validator (the doctor check crashed)\n'
            '    ✗ Due to an error, the doctor check did not complete. If the error message below is not helpful, '
            'please let us know about this issue at https://github.com/flutter/flutter/issues.\n'
            '    ✗ Bad state: fatal error\n'
            '[✓] Validators are fun (with statusInfo)\n'
            '[✓] Four score and seven validators ago (with statusInfo)\n'
            '\n'
            '! Doctor found issues in 1 category.\n',
          ),
        );
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate non-verbose output format when only one category fails',
      () async {
        expect(await FakeSinglePassingDoctor(logger).diagnose(verbose: false), isTrue);
        expect(
          logger.statusText,
          equals(
            'Doctor summary (to see all details, run flutter doctor -v):\n'
            '[!] Partial Validator with only a Hint\n'
            '    ! There is a hint here\n'
            '\n'
            '! Doctor found issues in 1 category.\n',
          ),
        );
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate non-verbose output format for a passing run',
      () async {
        expect(await FakePassingDoctor(logger).diagnose(verbose: false), isTrue);
        expect(
          logger.statusText,
          equals(
            'Doctor summary (to see all details, run flutter doctor -v):\n'
            '[✓] Passing Validator (with statusInfo)\n'
            '[!] Partial Validator with only a Hint\n'
            '    ! There is a hint here\n'
            '[!] Partial Validator with Errors\n'
            '    ✗ An error message indicating partial installation\n'
            '    ! Maybe a hint will help the user\n'
            '[✓] Another Passing Validator (with statusInfo)\n'
            '\n'
            '! Doctor found issues in 2 categories.\n',
          ),
        );
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate non-verbose output format',
      () async {
        expect(await FakeDoctor(logger).diagnose(verbose: false), isFalse);
        expect(
          logger.statusText,
          equals(
            'Doctor summary (to see all details, run flutter doctor -v):\n'
            '[✓] Passing Validator (with statusInfo)\n'
            '[✗] Missing Validator\n'
            '    ✗ A useful error message\n'
            '    ! A hint message\n'
            '[!] Not Available Validator\n'
            '    ✗ A useful error message\n'
            '    ! A hint message\n'
            '[!] Partial Validator with only a Hint\n'
            '    ! There is a hint here\n'
            '[!] Partial Validator with Errors\n'
            '    ✗ An error message indicating partial installation\n'
            '    ! Maybe a hint will help the user\n'
            '\n'
            '! Doctor found issues in 4 categories.\n',
          ),
        );
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext('validate verbose output format', () async {
      expect(await FakeDoctor(logger).diagnose(), isFalse);
      expect(
        logger.statusText,
        equals(
          '[✓] Passing Validator (with statusInfo)\n'
          '    • A helpful message\n'
          '    • A second, somewhat longer helpful message\n'
          '\n'
          '[✗] Missing Validator\n'
          '    ✗ A useful error message\n'
          '    • A message that is not an error\n'
          '    ! A hint message\n'
          '\n'
          '[!] Not Available Validator\n'
          '    ✗ A useful error message\n'
          '    • A message that is not an error\n'
          '    ! A hint message\n'
          '\n'
          '[!] Partial Validator with only a Hint\n'
          '    ! There is a hint here\n'
          '    • But there is no error\n'
          '\n'
          '[!] Partial Validator with Errors\n'
          '    ✗ An error message indicating partial installation\n'
          '    ! Maybe a hint will help the user\n'
          '    • An extra message with some verbose details\n'
          '\n'
          '! Doctor found issues in 4 categories.\n',
        ),
      );
    }, overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()});

    testUsingContext('validate PII can be hidden', () async {
      expect(await FakePiiDoctor(logger).diagnose(showPii: false), isTrue);
      expect(
        logger.statusText,
        equals(
          '[✓] PII Validator\n'
          '    • Does not contain PII\n'
          '\n'
          '• No issues found!\n',
        ),
      );
      logger.clear();
      // PII shown.
      expect(await FakePiiDoctor(logger).diagnose(), isTrue);
      expect(
        logger.statusText,
        equals(
          '[✓] PII Validator\n'
          '    • Contains PII path/to/username\n'
          '\n'
          '• No issues found!\n',
        ),
      );
    }, overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()});
  });

  group('doctor diagnosis wrapper', () {
    late TestUsage testUsage;
    late BufferLogger logger;

    setUp(() {
      testUsage = TestUsage();
      logger = BufferLogger.test();
    });

    testUsingContext(
      'PII separated, events only sent once',
      () async {
        final Doctor fakeDoctor = FakePiiDoctor(logger);
        final DoctorText doctorText = DoctorText(logger, doctor: fakeDoctor);
        const String expectedPiiText =
            '[✓] PII Validator\n'
            '    • Contains PII path/to/username\n'
            '\n'
            '• No issues found!\n';
        const String expectedPiiStrippedText =
            '[✓] PII Validator\n'
            '    • Does not contain PII\n'
            '\n'
            '• No issues found!\n';

        // Run each multiple times to make sure the logger buffer is being cleared,
        // and that events are only sent once.
        expect(await doctorText.text, expectedPiiText);
        expect(await doctorText.text, expectedPiiText);

        expect(await doctorText.piiStrippedText, expectedPiiStrippedText);
        expect(await doctorText.piiStrippedText, expectedPiiStrippedText);

        // Only one event sent.
        expect(testUsage.events, <TestUsageEvent>[
          const TestUsageEvent('doctor-result', 'PiiValidator', label: 'installed'),
        ]);
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal(), Usage: () => testUsage},
    );

    testUsingContext(
      'without PII has same text and PII-stripped text',
      () async {
        final Doctor fakeDoctor = FakePassingDoctor(logger);
        final DoctorText doctorText = DoctorText(logger, doctor: fakeDoctor);
        final String piiText = await doctorText.text;
        expect(piiText, isNotEmpty);
        expect(piiText, await doctorText.piiStrippedText);
      },
      overrides: <Type, Generator>{Usage: () => testUsage},
    );
  });

  testUsingContext(
    'validate non-verbose output wrapping',
    () async {
      final BufferLogger wrapLogger = BufferLogger.test(
        outputPreferences: OutputPreferences(wrapText: true, wrapColumn: 30),
      );
      expect(await FakeDoctor(wrapLogger).diagnose(verbose: false), isFalse);
      expect(
        wrapLogger.statusText,
        equals(
          'Doctor summary (to see all\n'
          'details, run flutter doctor\n'
          '-v):\n'
          '[✓] Passing Validator (with\n'
          '    statusInfo)\n'
          '[✗] Missing Validator\n'
          '    ✗ A useful error message\n'
          '    ! A hint message\n'
          '[!] Not Available Validator\n'
          '    ✗ A useful error message\n'
          '    ! A hint message\n'
          '[!] Partial Validator with\n'
          '    only a Hint\n'
          '    ! There is a hint here\n'
          '[!] Partial Validator with\n'
          '    Errors\n'
          '    ✗ An error message\n'
          '      indicating partial\n'
          '      installation\n'
          '    ! Maybe a hint will help\n'
          '      the user\n'
          '\n'
          '! Doctor found issues in 4\n'
          '  categories.\n',
        ),
      );
    },
    overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
  );

  testUsingContext('validate verbose output wrapping', () async {
    final BufferLogger wrapLogger = BufferLogger.test(
      outputPreferences: OutputPreferences(wrapText: true, wrapColumn: 30),
    );
    expect(await FakeDoctor(wrapLogger).diagnose(), isFalse);
    expect(
      wrapLogger.statusText,
      equals(
        '[✓] Passing Validator (with\n'
        '    statusInfo)\n'
        '    • A helpful message\n'
        '    • A second, somewhat\n'
        '      longer helpful message\n'
        '\n'
        '[✗] Missing Validator\n'
        '    ✗ A useful error message\n'
        '    • A message that is not an\n'
        '      error\n'
        '    ! A hint message\n'
        '\n'
        '[!] Not Available Validator\n'
        '    ✗ A useful error message\n'
        '    • A message that is not an\n'
        '      error\n'
        '    ! A hint message\n'
        '\n'
        '[!] Partial Validator with\n'
        '    only a Hint\n'
        '    ! There is a hint here\n'
        '    • But there is no error\n'
        '\n'
        '[!] Partial Validator with\n'
        '    Errors\n'
        '    ✗ An error message\n'
        '      indicating partial\n'
        '      installation\n'
        '    ! Maybe a hint will help\n'
        '      the user\n'
        '    • An extra message with\n'
        '      some verbose details\n'
        '\n'
        '! Doctor found issues in 4\n'
        '  categories.\n',
      ),
    );
  }, overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()});

  group('doctor with grouped validators', () {
    testUsingContext(
      'validate diagnose combines validator output',
      () async {
        expect(await FakeGroupedDoctor(logger).diagnose(), isTrue);
        expect(
          logger.statusText,
          equals(
            '[✓] Category 1\n'
            '    • A helpful message\n'
            '    • A helpful message\n'
            '\n'
            '[!] Category 2\n'
            '    • A helpful message\n'
            '    ✗ A useful error message\n'
            '\n'
            '! Doctor found issues in 1 category.\n',
          ),
        );
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate merging assigns statusInfo and title',
      () async {
        // There are two subvalidators. Only the second contains statusInfo.
        expect(await FakeGroupedDoctorWithStatus(logger).diagnose(), isTrue);
        expect(
          logger.statusText,
          equals(
            '[✓] First validator title (A status message)\n'
            '    • A helpful message\n'
            '    • A different message\n'
            '\n'
            '• No issues found!\n',
          ),
        );
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );
  });

  group('grouped validator merging results', () {
    final PassingGroupedValidator installed = PassingGroupedValidator('Category');
    final PartialGroupedValidator partial = PartialGroupedValidator('Category');
    final MissingGroupedValidator missing = MissingGroupedValidator('Category');

    testUsingContext(
      'validate installed + installed = installed',
      () async {
        expect(await FakeSmallGroupDoctor(logger, installed, installed).diagnose(), isTrue);
        expect(logger.statusText, startsWith('[✓]'));
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate installed + partial = partial',
      () async {
        expect(await FakeSmallGroupDoctor(logger, installed, partial).diagnose(), isTrue);
        expect(logger.statusText, startsWith('[!]'));
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate installed + missing = partial',
      () async {
        expect(await FakeSmallGroupDoctor(logger, installed, missing).diagnose(), isTrue);
        expect(logger.statusText, startsWith('[!]'));
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate partial + installed = partial',
      () async {
        expect(await FakeSmallGroupDoctor(logger, partial, installed).diagnose(), isTrue);
        expect(logger.statusText, startsWith('[!]'));
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate partial + partial = partial',
      () async {
        expect(await FakeSmallGroupDoctor(logger, partial, partial).diagnose(), isTrue);
        expect(logger.statusText, startsWith('[!]'));
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate partial + missing = partial',
      () async {
        expect(await FakeSmallGroupDoctor(logger, partial, missing).diagnose(), isTrue);
        expect(logger.statusText, startsWith('[!]'));
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate missing + installed = partial',
      () async {
        expect(await FakeSmallGroupDoctor(logger, missing, installed).diagnose(), isTrue);
        expect(logger.statusText, startsWith('[!]'));
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate missing + partial = partial',
      () async {
        expect(await FakeSmallGroupDoctor(logger, missing, partial).diagnose(), isTrue);
        expect(logger.statusText, startsWith('[!]'));
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );

    testUsingContext(
      'validate missing + missing = missing',
      () async {
        expect(await FakeSmallGroupDoctor(logger, missing, missing).diagnose(), isFalse);
        expect(logger.statusText, startsWith('[✗]'));
      },
      overrides: <Type, Generator>{AnsiTerminal: () => FakeTerminal()},
    );
  });

  testUsingContext(
    'WebWorkflow is a part of validator workflows if enabled',
    () async {
      final List<Workflow> workflows =
          DoctorValidatorsProvider.test(
            featureFlags: TestFeatureFlags(isWebEnabled: true),
            platform: FakePlatform(),
          ).workflows;
      expect(workflows, contains(isA<WebWorkflow>()));
    },
    overrides: <Type, Generator>{
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => fakeProcessManager,
    },
  );

  testUsingContext(
    'CustomDevicesWorkflow is a part of validator workflows if enabled',
    () async {
      final List<Workflow> workflows =
          DoctorValidatorsProvider.test(
            featureFlags: TestFeatureFlags(areCustomDevicesEnabled: true),
            platform: FakePlatform(),
          ).workflows;
      expect(workflows, contains(isA<CustomDeviceWorkflow>()));
    },
    overrides: <Type, Generator>{FileSystem: () => fs, ProcessManager: () => fakeProcessManager},
  );

  group('FlutterValidator', () {
    late FakeFlutterVersion initialVersion;
    late FakeFlutterVersion secondVersion;
    late TestFeatureFlags featureFlags;

    setUp(() {
      secondVersion = FakeFlutterVersion(frameworkRevisionShort: '222');
      initialVersion = FakeFlutterVersion(
        frameworkRevisionShort: '111',
        nextFlutterVersion: secondVersion,
      );
      featureFlags = TestFeatureFlags();
    });

    testUsingContext(
      'FlutterValidator fetches tags and gets fresh version',
      () async {
        final Directory devtoolsDir = fs.directory(
          '/path/to/flutter/bin/cache/dart-sdk/bin/resources/devtools',
        )..createSync(recursive: true);
        fs.directory('/path/to/flutter/bin/cache/artifacts').createSync(recursive: true);
        devtoolsDir.childFile('version.json').writeAsStringSync('{"version": "123"}');
        fakeProcessManager.addCommands(const <FakeCommand>[
          FakeCommand(command: <String>['which', 'java']),
        ]);
        final List<DoctorValidator> validators =
            DoctorValidatorsProvider.test(
              featureFlags: featureFlags,
              platform: FakePlatform(),
            ).validators;
        final FlutterValidator flutterValidator = validators.whereType<FlutterValidator>().first;
        final ValidationResult result = await flutterValidator.validate();
        expect(
          result.messages.map((ValidationMessage msg) => msg.message),
          contains(contains('Framework revision 222')),
        );
      },
      overrides: <Type, Generator>{
        Cache:
            () => Cache.test(
              rootOverride: fs.directory('/path/to/flutter'),
              fileSystem: fs,
              processManager: fakeProcessManager,
            ),
        FileSystem: () => fs,
        FlutterVersion: () => initialVersion,
        Platform: () => FakePlatform(),
        ProcessManager: () => fakeProcessManager,
        TestFeatureFlags: () => featureFlags,
      },
    );
  });
  testUsingContext(
    'If android workflow is disabled, AndroidStudio validator is not included',
    () {
      final DoctorValidatorsProvider provider = DoctorValidatorsProvider.test(
        featureFlags: TestFeatureFlags(isAndroidEnabled: false),
      );
      expect(provider.validators, isNot(contains(isA<AndroidStudioValidator>())));
      expect(provider.validators, isNot(contains(isA<NoAndroidStudioValidator>())));
    },
    overrides: <Type, Generator>{
      AndroidWorkflow: () => FakeAndroidWorkflow(appliesToHostPlatform: false),
    },
  );

  group('Doctor events with unified_analytics', () {
    late FakeAnalytics fakeAnalytics;
    final FakeFlutterVersion fakeFlutterVersion = FakeFlutterVersion();
    final DateTime fakeDate = DateTime(1995, 3, 3);
    final SystemClock fakeSystemClock = SystemClock.fixed(fakeDate);

    setUp(() {
      fakeAnalytics = getInitializedFakeAnalyticsInstance(
        fakeFlutterVersion: fakeFlutterVersion,
        fs: fs,
      );
    });

    testUsingContext(
      'ensure fake is being used and initialized',
      () {
        expect(fakeAnalytics.sentEvents.length, 0);
        expect(fakeAnalytics.okToSend, true);
      },
      overrides: <Type, Generator>{Analytics: () => fakeAnalytics},
    );

    testUsingContext(
      'contains installed',
      () async {
        final Doctor doctor = Doctor(
          logger: logger,
          clock: fakeSystemClock,
          analytics: fakeAnalytics,
        );
        await doctor.diagnose(verbose: false);

        expect(fakeAnalytics.sentEvents.length, 3);

        // The event that should have been fired off during the doctor invocation
        final Event eventToFind = Event.doctorValidatorResult(
          validatorName: 'Passing Validator',
          result: 'installed',
          partOfGroupedValidator: false,
          doctorInvocationId: DateTime(1995, 3, 3).millisecondsSinceEpoch,
          statusInfo: 'with statusInfo',
        );
        expect(fakeAnalytics.sentEvents, contains(eventToFind));
      },
      overrides: <Type, Generator>{DoctorValidatorsProvider: () => FakeDoctorValidatorsProvider()},
    );

    testUsingContext(
      'contains installed and partial',
      () async {
        await FakePassingDoctor(logger, clock: fakeSystemClock).diagnose(verbose: false);

        expect(fakeAnalytics.sentEvents, hasLength(4));
        expect(
          fakeAnalytics.sentEvents,
          unorderedEquals(<Event>[
            Event.doctorValidatorResult(
              validatorName: 'Passing Validator',
              result: 'installed',
              partOfGroupedValidator: false,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
              statusInfo: 'with statusInfo',
            ),
            Event.doctorValidatorResult(
              validatorName: 'Partial Validator with only a Hint',
              result: 'partial',
              partOfGroupedValidator: false,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
            ),
            Event.doctorValidatorResult(
              validatorName: 'Partial Validator with Errors',
              result: 'partial',
              partOfGroupedValidator: false,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
            ),
            Event.doctorValidatorResult(
              validatorName: 'Another Passing Validator',
              result: 'installed',
              partOfGroupedValidator: false,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
              statusInfo: 'with statusInfo',
            ),
          ]),
        );
      },
      overrides: <Type, Generator>{
        DoctorValidatorsProvider: () => FakeDoctorValidatorsProvider(),
        Analytics: () => fakeAnalytics,
      },
    );

    testUsingContext(
      'contains installed, missing and partial',
      () async {
        await FakeDoctor(logger, clock: fakeSystemClock).diagnose(verbose: false);

        expect(fakeAnalytics.sentEvents, hasLength(5));
        expect(
          fakeAnalytics.sentEvents,
          unorderedEquals(<Event>[
            Event.doctorValidatorResult(
              validatorName: 'Passing Validator',
              result: 'installed',
              partOfGroupedValidator: false,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
              statusInfo: 'with statusInfo',
            ),
            Event.doctorValidatorResult(
              validatorName: 'Missing Validator',
              result: 'missing',
              partOfGroupedValidator: false,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
            ),
            Event.doctorValidatorResult(
              validatorName: 'Not Available Validator',
              result: 'notAvailable',
              partOfGroupedValidator: false,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
            ),
            Event.doctorValidatorResult(
              validatorName: 'Partial Validator with only a Hint',
              result: 'partial',
              partOfGroupedValidator: false,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
            ),
            Event.doctorValidatorResult(
              validatorName: 'Partial Validator with Errors',
              result: 'partial',
              partOfGroupedValidator: false,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
            ),
          ]),
        );
      },
      overrides: <Type, Generator>{
        DoctorValidatorsProvider: () => FakeDoctorValidatorsProvider(),
        Analytics: () => fakeAnalytics,
      },
    );

    testUsingContext(
      'events for grouped validators are properly decomposed',
      () async {
        await FakeGroupedDoctor(logger, clock: fakeSystemClock).diagnose(verbose: false);

        expect(fakeAnalytics.sentEvents, hasLength(4));
        expect(
          fakeAnalytics.sentEvents,
          unorderedEquals(<Event>[
            Event.doctorValidatorResult(
              validatorName: 'Category 1',
              result: 'installed',
              partOfGroupedValidator: true,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
            ),
            Event.doctorValidatorResult(
              validatorName: 'Category 1',
              result: 'installed',
              partOfGroupedValidator: true,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
            ),
            Event.doctorValidatorResult(
              validatorName: 'Category 2',
              result: 'installed',
              partOfGroupedValidator: true,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
            ),
            Event.doctorValidatorResult(
              validatorName: 'Category 2',
              result: 'missing',
              partOfGroupedValidator: true,
              doctorInvocationId: fakeDate.millisecondsSinceEpoch,
            ),
          ]),
        );
      },
      overrides: <Type, Generator>{
        DoctorValidatorsProvider: () => FakeDoctorValidatorsProvider(),
        Analytics: () => fakeAnalytics,
      },
    );

    testUsingContext(
      'grouped validator subresult and subvalidators different lengths',
      () async {
        final FakeGroupedDoctorWithCrash fakeDoctor = FakeGroupedDoctorWithCrash(
          logger,
          clock: fakeSystemClock,
        );
        await fakeDoctor.diagnose(verbose: false);

        expect(fakeDoctor.validators, hasLength(1));
        expect(fakeDoctor.validators.first.runtimeType == FakeGroupedValidatorWithCrash, true);
        expect(fakeAnalytics.sentEvents, hasLength(0));

        // Attempt to send a random event to ensure that the
        // analytics package is still working, despite not sending
        // above (as expected)
        final Event testEvent = Event.analyticsCollectionEnabled(status: true);
        fakeAnalytics.send(testEvent);
        expect(fakeAnalytics.sentEvents, hasLength(1));
        expect(fakeAnalytics.sentEvents, contains(testEvent));
      },
      overrides: <Type, Generator>{Analytics: () => fakeAnalytics},
    );

    testUsingContext('sending events can be skipped', () async {
      await FakePassingDoctor(logger).diagnose(verbose: false, sendEvent: false);
      expect(fakeAnalytics.sentEvents, isEmpty);
    }, overrides: <Type, Generator>{Analytics: () => fakeAnalytics});
  });
}

class FakeAndroidWorkflow extends Fake implements AndroidWorkflow {
  FakeAndroidWorkflow({this.canListDevices = true, this.appliesToHostPlatform = true});

  @override
  final bool canListDevices;

  @override
  final bool appliesToHostPlatform;
}

class PassingValidator extends DoctorValidator {
  PassingValidator(super.title);

  @override
  Future<ValidationResult> validate() async {
    const List<ValidationMessage> messages = <ValidationMessage>[
      ValidationMessage('A helpful message'),
      ValidationMessage('A second, somewhat longer helpful message'),
    ];
    return const ValidationResult(ValidationType.success, messages, statusInfo: 'with statusInfo');
  }
}

class PiiValidator extends DoctorValidator {
  PiiValidator() : super('PII Validator');

  @override
  Future<ValidationResult> validate() async {
    const List<ValidationMessage> messages = <ValidationMessage>[
      ValidationMessage(
        'Contains PII path/to/username',
        piiStrippedMessage: 'Does not contain PII',
      ),
    ];
    return const ValidationResult(ValidationType.success, messages);
  }
}

class MissingValidator extends DoctorValidator {
  MissingValidator() : super('Missing Validator');

  @override
  Future<ValidationResult> validate() async {
    const List<ValidationMessage> messages = <ValidationMessage>[
      ValidationMessage.error('A useful error message'),
      ValidationMessage('A message that is not an error'),
      ValidationMessage.hint('A hint message'),
    ];
    return const ValidationResult(ValidationType.missing, messages);
  }
}

class NotAvailableValidator extends DoctorValidator {
  NotAvailableValidator() : super('Not Available Validator');

  @override
  Future<ValidationResult> validate() async {
    const List<ValidationMessage> messages = <ValidationMessage>[
      ValidationMessage.error('A useful error message'),
      ValidationMessage('A message that is not an error'),
      ValidationMessage.hint('A hint message'),
    ];
    return const ValidationResult(ValidationType.notAvailable, messages);
  }
}

class StuckValidator extends DoctorValidator {
  StuckValidator() : super('Stuck validator that never completes');

  @override
  Future<ValidationResult> validate() {
    final Completer<ValidationResult> completer = Completer<ValidationResult>();

    // This future will never complete
    return completer.future;
  }
}

class PartialValidatorWithErrors extends DoctorValidator {
  PartialValidatorWithErrors() : super('Partial Validator with Errors');

  @override
  Future<ValidationResult> validate() async {
    const List<ValidationMessage> messages = <ValidationMessage>[
      ValidationMessage.error('An error message indicating partial installation'),
      ValidationMessage.hint('Maybe a hint will help the user'),
      ValidationMessage('An extra message with some verbose details'),
    ];
    return const ValidationResult(ValidationType.partial, messages);
  }
}

class PartialValidatorWithHintsOnly extends DoctorValidator {
  PartialValidatorWithHintsOnly() : super('Partial Validator with only a Hint');

  @override
  Future<ValidationResult> validate() async {
    const List<ValidationMessage> messages = <ValidationMessage>[
      ValidationMessage.hint('There is a hint here'),
      ValidationMessage('But there is no error'),
    ];
    return const ValidationResult(ValidationType.partial, messages);
  }
}

class CrashingValidator extends DoctorValidator {
  CrashingValidator() : super('Crashing validator');

  @override
  Future<ValidationResult> validate() async {
    throw StateError('fatal error');
  }
}

class AsyncCrashingValidator extends DoctorValidator {
  AsyncCrashingValidator(this._time) : super('Async crashing validator');

  final FakeAsync _time;

  @override
  Future<ValidationResult> validate() {
    const Duration delay = Duration(seconds: 1);
    final Future<ValidationResult> result = Future<ValidationResult>.delayed(
      delay,
      () => throw StateError('fatal error'),
    );
    _time.elapse(const Duration(seconds: 1));
    _time.flushMicrotasks();
    return result;
  }
}

/// A doctor that fails with a missing [ValidationResult].
class FakeDoctor extends Doctor {
  FakeDoctor(Logger logger, {super.clock = const SystemClock()}) : super(logger: logger);

  @override
  late final List<DoctorValidator> validators = <DoctorValidator>[
    PassingValidator('Passing Validator'),
    MissingValidator(),
    NotAvailableValidator(),
    PartialValidatorWithHintsOnly(),
    PartialValidatorWithErrors(),
  ];
}

/// A doctor that should pass, but still has issues in some categories.
class FakePassingDoctor extends Doctor {
  FakePassingDoctor(Logger logger, {super.clock = const SystemClock()}) : super(logger: logger);

  @override
  late final List<DoctorValidator> validators = <DoctorValidator>[
    PassingValidator('Passing Validator'),
    PartialValidatorWithHintsOnly(),
    PartialValidatorWithErrors(),
    PassingValidator('Another Passing Validator'),
  ];
}

/// A doctor that should pass, but still has 1 issue to test the singular of
/// categories.
class FakeSinglePassingDoctor extends Doctor {
  FakeSinglePassingDoctor(Logger logger, {super.clock = const SystemClock()})
    : super(logger: logger);

  @override
  late final List<DoctorValidator> validators = <DoctorValidator>[PartialValidatorWithHintsOnly()];
}

/// A doctor that passes and has no issues anywhere.
class FakeQuietDoctor extends Doctor {
  FakeQuietDoctor(Logger logger, {super.clock = const SystemClock()}) : super(logger: logger);

  @override
  late final List<DoctorValidator> validators = <DoctorValidator>[
    PassingValidator('Passing Validator'),
    PassingValidator('Another Passing Validator'),
    PassingValidator('Validators are fun'),
    PassingValidator('Four score and seven validators ago'),
  ];
}

/// A doctor that passes and contains PII that can be hidden.
class FakePiiDoctor extends Doctor {
  FakePiiDoctor(Logger logger, {super.clock = const SystemClock()}) : super(logger: logger);

  @override
  late final List<DoctorValidator> validators = <DoctorValidator>[PiiValidator()];
}

/// A doctor with a validator that throws an exception.
class FakeCrashingDoctor extends Doctor {
  FakeCrashingDoctor(Logger logger, {super.clock = const SystemClock()}) : super(logger: logger);

  @override
  late final List<DoctorValidator> validators = <DoctorValidator>[
    PassingValidator('Passing Validator'),
    PassingValidator('Another Passing Validator'),
    CrashingValidator(),
    PassingValidator('Validators are fun'),
    PassingValidator('Four score and seven validators ago'),
  ];
}

/// A doctor with a validator that will never finish.
class FakeAsyncStuckDoctor extends Doctor {
  FakeAsyncStuckDoctor(Logger logger, {super.clock = const SystemClock()}) : super(logger: logger);

  @override
  late final List<DoctorValidator> validators = <DoctorValidator>[
    PassingValidator('Passing Validator'),
    PassingValidator('Another Passing Validator'),
    StuckValidator(),
    PassingValidator('Validators are fun'),
    PassingValidator('Four score and seven validators ago'),
  ];
}

/// A doctor with a validator that throws an exception.
class FakeAsyncCrashingDoctor extends Doctor {
  FakeAsyncCrashingDoctor(this._time, Logger logger, {super.clock = const SystemClock()})
    : super(logger: logger);

  final FakeAsync _time;

  @override
  late final List<DoctorValidator> validators = <DoctorValidator>[
    PassingValidator('Passing Validator'),
    PassingValidator('Another Passing Validator'),
    AsyncCrashingValidator(_time),
    PassingValidator('Validators are fun'),
    PassingValidator('Four score and seven validators ago'),
  ];
}

/// A DoctorValidatorsProvider that overrides the default validators without
/// overriding the doctor.
class FakeDoctorValidatorsProvider implements DoctorValidatorsProvider {
  @override
  List<DoctorValidator> get validators => <DoctorValidator>[
    PassingValidator('Passing Validator'),
    PassingValidator('Another Passing Validator'),
    PassingValidator('Providing validators is fun'),
  ];

  @override
  List<Workflow> get workflows => <Workflow>[];
}

class PassingGroupedValidator extends DoctorValidator {
  PassingGroupedValidator(super.title);

  @override
  Future<ValidationResult> validate() async {
    const List<ValidationMessage> messages = <ValidationMessage>[
      ValidationMessage('A helpful message'),
    ];
    return const ValidationResult(ValidationType.success, messages);
  }
}

class MissingGroupedValidator extends DoctorValidator {
  MissingGroupedValidator(super.title);

  @override
  Future<ValidationResult> validate() async {
    const List<ValidationMessage> messages = <ValidationMessage>[
      ValidationMessage.error('A useful error message'),
    ];
    return const ValidationResult(ValidationType.missing, messages);
  }
}

class PartialGroupedValidator extends DoctorValidator {
  PartialGroupedValidator(super.title);

  @override
  Future<ValidationResult> validate() async {
    const List<ValidationMessage> messages = <ValidationMessage>[
      ValidationMessage.error('An error message for partial installation'),
    ];
    return const ValidationResult(ValidationType.partial, messages);
  }
}

class PassingGroupedValidatorWithStatus extends DoctorValidator {
  PassingGroupedValidatorWithStatus(super.title);

  @override
  Future<ValidationResult> validate() async {
    const List<ValidationMessage> messages = <ValidationMessage>[
      ValidationMessage('A different message'),
    ];
    return const ValidationResult(ValidationType.success, messages, statusInfo: 'A status message');
  }
}

/// A doctor that has two groups of two validators each.
class FakeGroupedDoctor extends Doctor {
  FakeGroupedDoctor(Logger logger, {super.clock = const SystemClock()}) : super(logger: logger);

  @override
  late final List<DoctorValidator> validators = <DoctorValidator>[
    GroupedValidator(<DoctorValidator>[
      PassingGroupedValidator('Category 1'),
      PassingGroupedValidator('Category 1'),
    ]),
    GroupedValidator(<DoctorValidator>[
      PassingGroupedValidator('Category 2'),
      MissingGroupedValidator('Category 2'),
    ]),
  ];
}

/// Fake grouped doctor that is intended to be used with [FakeGroupedValidatorWithCrash].
class FakeGroupedDoctorWithCrash extends Doctor {
  FakeGroupedDoctorWithCrash(Logger logger, {super.clock = const SystemClock()})
    : super(logger: logger);

  @override
  late final List<DoctorValidator> validators = <DoctorValidator>[
    FakeGroupedValidatorWithCrash(<DoctorValidator>[
      PassingGroupedValidator('Category 1'),
      PassingGroupedValidator('Category 1'),
    ]),
  ];
}

/// This extended grouped validator will have a list of sub validators
/// provided in the constructor, but it will have no [subResults] in the
/// list which simulates what happens if a validator crashes.
///
/// Usually, the grouped validators have 2 lists, a [subValidators] and
/// a [subResults] list, and if nothing crashes, those 2 lists will have the
/// same length. This fake is simulating what happens when the validators
/// crash and results in no results getting returned.
class FakeGroupedValidatorWithCrash extends GroupedValidator {
  FakeGroupedValidatorWithCrash(super.subValidators);

  @override
  List<ValidationResult> get subResults => <ValidationResult>[];
}

class FakeGroupedDoctorWithStatus extends Doctor {
  FakeGroupedDoctorWithStatus(Logger logger, {super.clock = const SystemClock()})
    : super(logger: logger);

  @override
  late final List<DoctorValidator> validators = <DoctorValidator>[
    GroupedValidator(<DoctorValidator>[
      PassingGroupedValidator('First validator title'),
      PassingGroupedValidatorWithStatus('Second validator title'),
    ]),
  ];
}

/// A doctor that takes any two validators. Used to check behavior when
/// merging ValidationTypes (installed, missing, partial).
class FakeSmallGroupDoctor extends Doctor {
  FakeSmallGroupDoctor(
    Logger logger,
    DoctorValidator val1,
    DoctorValidator val2, {
    super.clock = const SystemClock(),
  }) : validators = <DoctorValidator>[
         GroupedValidator(<DoctorValidator>[val1, val2]),
       ],
       super(logger: logger);

  @override
  final List<DoctorValidator> validators;
}

class VsCodeValidatorTestTargets extends VsCodeValidator {
  VsCodeValidatorTestTargets._(
    String installDirectory,
    String extensionDirectory, {
    String? edition,
  }) : super(
         VsCode.fromDirectory(
           installDirectory,
           extensionDirectory,
           edition: edition,
           fileSystem: globals.fs,
         ),
       );

  static VsCodeValidatorTestTargets get installedWithExtension =>
      VsCodeValidatorTestTargets._(validInstall, validExtensions);

  static VsCodeValidatorTestTargets get installedWithExtension64bit =>
      VsCodeValidatorTestTargets._(validInstall, validExtensions, edition: '64-bit edition');

  static VsCodeValidatorTestTargets get installedWithoutExtension =>
      VsCodeValidatorTestTargets._(validInstall, missingExtensions);

  static final String validInstall = globals.fs.path.join('test', 'data', 'vscode', 'application');
  static final String validExtensions = globals.fs.path.join(
    'test',
    'data',
    'vscode',
    'extensions',
  );
  static final String missingExtensions = globals.fs.path.join(
    'test',
    'data',
    'vscode',
    'notExtensions',
  );
}

class FakeDeviceManager extends Fake implements DeviceManager {
  List<String> diagnostics = <String>[];
  List<Device> devices = <Device>[];

  @override
  Future<List<Device>> getAllDevices({DeviceDiscoveryFilter? filter}) async => devices;

  @override
  Future<List<Device>> refreshAllDevices({
    Duration? timeout,
    DeviceDiscoveryFilter? filter,
  }) async => devices;

  @override
  Future<List<String>> getDeviceDiagnostics() async => diagnostics;
}

class FakeDevice extends Fake implements Device {
  @override
  String get name => 'name';

  @override
  String get id => 'device-id';

  @override
  Category get category => Category.mobile;

  @override
  bool isSupported() => true;

  @override
  Future<bool> get isLocalEmulator async => false;

  @override
  Future<String> get targetPlatformDisplayName async => 'android';

  @override
  Future<String> get sdkNameAndVersion async => '1.2.3';

  @override
  Future<TargetPlatform> get targetPlatform => Future<TargetPlatform>.value(TargetPlatform.android);
}

class FakeTerminal extends Fake implements AnsiTerminal {
  @override
  final bool supportsColor = false;

  @override
  bool get isCliAnimationEnabled => supportsColor;
}
