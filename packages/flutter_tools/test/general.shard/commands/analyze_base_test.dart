// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/commands/analyze_base.dart';

import '../../src/common.dart';

void main() {
  testWithoutContext('AnalyzeBase message formatting with zero issues', () async {
    final String message = AnalyzeBase.generateErrorsMessage(issueCount: 0, seconds: '10');

    expect(message, 'No issues found! (ran in 10s)');
  });

  testWithoutContext('AnalyzeBase message formatting with one issue', () async {
    final String message = AnalyzeBase.generateErrorsMessage(issueCount: 1, seconds: '10');

    expect(message, '1 issue found. (ran in 10s)');
  });

  testWithoutContext('AnalyzeBase message formatting with N issues', () async {
    final String message = AnalyzeBase.generateErrorsMessage(issueCount: 10, seconds: '10');

    expect(message, '10 issues found. (ran in 10s)');
  });

  testWithoutContext('AnalyzeBase message with analyze files', () async {
    final String message = AnalyzeBase.generateErrorsMessage(
      issueCount: 0,
      seconds: '10',
      files: 10,
    );

    expect(message, 'No issues found! • analyzed 10 files (ran in 10s)');
  });

  testWithoutContext('AnalyzeBase message with positive issue diff', () async {
    final String message = AnalyzeBase.generateErrorsMessage(
      issueCount: 1,
      seconds: '10',
      issueDiff: 1,
    );

    expect(message, '1 issue found. (1 new) (ran in 10s)');
  });

  testWithoutContext('AnalyzeBase message with negative issue diff', () async {
    final String message = AnalyzeBase.generateErrorsMessage(
      issueCount: 0,
      seconds: '10',
      issueDiff: -1,
    );

    expect(message, 'No issues found! (1 fixed) (ran in 10s)');
  });
}
