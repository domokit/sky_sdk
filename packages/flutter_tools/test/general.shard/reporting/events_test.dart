// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';

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
