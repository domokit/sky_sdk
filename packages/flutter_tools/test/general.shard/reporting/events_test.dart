// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:package_config/package_config.dart';

import '../../src/common.dart';

void main() {
  testWithoutContext('DoctorResultEvent sends usage event for each sub validator', () async {
    final TestUsage usage = TestUsage();
    final GroupedValidator groupedValidator = FakeGroupedValidator(<DoctorValidator>[
      FakeDoctorValidator('a'),
      FakeDoctorValidator('b'),
      FakeDoctorValidator('c'),
    ]);
    final ValidationResult result = await groupedValidator.validate();

    final DoctorResultEvent doctorResultEvent = DoctorResultEvent(
      validator: groupedValidator,
      result: result,
      flutterUsage: usage,
    );

    expect(doctorResultEvent.send, returnsNormally);
    expect(usage.events.length, 3);
    expect(
      usage.events,
      contains(const TestUsageEvent('doctor-result', 'FakeDoctorValidator', label: 'crash')),
    );
  });

  testWithoutContext('DoctorResultEvent does not crash if a synthetic crash result was used instead'
      ' of validation. This happens when a grouped validator throws an exception, causing subResults to never '
      ' be instantiated.', () async {
    final TestUsage usage = TestUsage();
    final GroupedValidator groupedValidator = FakeGroupedValidator(<DoctorValidator>[
      FakeDoctorValidator('a'),
      FakeDoctorValidator('b'),
      FakeDoctorValidator('c'),
    ]);
    final ValidationResult result = ValidationResult.crash(Object());

    final DoctorResultEvent doctorResultEvent = DoctorResultEvent(
      validator: groupedValidator,
      result: result,
      flutterUsage: usage,
    );

    expect(doctorResultEvent.send, returnsNormally);

    expect(usage.events.length, 1);
    expect(
      usage.events,
      contains(const TestUsageEvent('doctor-result', 'FakeGroupedValidator', label: 'crash')),
    );
  });

  testWithoutContext('Reports null safe analytics events', () {
    final TestUsage usage = TestUsage();
    final PackageConfig packageConfig = PackageConfig(<Package>[
      Package('foo', Uri.parse('file:///foo/'), languageVersion: LanguageVersion(2, 12)),
      Package('bar', Uri.parse('file:///fizz/'), languageVersion: LanguageVersion(2, 1)),
      Package('baz', Uri.parse('file:///bar/'), languageVersion: LanguageVersion(2, 2)),
    ]);

    NullSafetyAnalysisEvent(packageConfig, NullSafetyMode.sound, 'foo', usage).send();

    expect(
      usage.events,
      unorderedEquals(<TestUsageEvent>[
        const TestUsageEvent(
          NullSafetyAnalysisEvent.kNullSafetyCategory,
          'runtime-mode',
          label: 'NullSafetyMode.sound',
        ),
        TestUsageEvent(
          NullSafetyAnalysisEvent.kNullSafetyCategory,
          'stats',
          parameters: CustomDimensions.fromMap(<String, String>{'cd49': '1', 'cd50': '3'}),
        ),
        const TestUsageEvent(
          NullSafetyAnalysisEvent.kNullSafetyCategory,
          'language-version',
          label: '2.12',
        ),
      ]),
    );
  });

  testWithoutContext('Does not crash if main package is missing', () {
    final TestUsage usage = TestUsage();
    final PackageConfig packageConfig = PackageConfig(<Package>[
      Package('foo', Uri.parse('file:///foo/lib/'), languageVersion: LanguageVersion(2, 12)),
      Package('bar', Uri.parse('file:///fizz/lib/'), languageVersion: LanguageVersion(2, 1)),
      Package('baz', Uri.parse('file:///bar/lib/'), languageVersion: LanguageVersion(2, 2)),
    ]);

    NullSafetyAnalysisEvent(
      packageConfig,
      NullSafetyMode.sound,
      'something-unrelated',
      usage,
    ).send();

    expect(
      usage.events,
      unorderedEquals(<TestUsageEvent>[
        const TestUsageEvent(
          NullSafetyAnalysisEvent.kNullSafetyCategory,
          'runtime-mode',
          label: 'NullSafetyMode.sound',
        ),
        TestUsageEvent(
          NullSafetyAnalysisEvent.kNullSafetyCategory,
          'stats',
          parameters: CustomDimensions.fromMap(<String, String>{'cd49': '1', 'cd50': '3'}),
        ),
      ]),
    );
  });

  testWithoutContext('a null language version is treated as unmigrated', () {
    final TestUsage usage = TestUsage();
    final PackageConfig packageConfig = PackageConfig(<Package>[
      Package('foo', Uri.parse('file:///foo/lib/')),
    ]);

    NullSafetyAnalysisEvent(
      packageConfig,
      NullSafetyMode.sound,
      'something-unrelated',
      usage,
    ).send();

    expect(
      usage.events,
      unorderedEquals(<TestUsageEvent>[
        const TestUsageEvent(
          NullSafetyAnalysisEvent.kNullSafetyCategory,
          'runtime-mode',
          label: 'NullSafetyMode.sound',
        ),
        TestUsageEvent(
          NullSafetyAnalysisEvent.kNullSafetyCategory,
          'stats',
          parameters: CustomDimensions.fromMap(<String, String>{'cd49': '0', 'cd50': '1'}),
        ),
      ]),
    );
  });
}

class FakeGroupedValidator extends GroupedValidator {
  FakeGroupedValidator(super.subValidators);
}

class FakeDoctorValidator extends DoctorValidator {
  FakeDoctorValidator(super.title);

  @override
  Future<ValidationResult> validateImpl() async {
    return ValidationResult.crash(Object());
  }
}
