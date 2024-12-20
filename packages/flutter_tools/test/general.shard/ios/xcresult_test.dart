// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/ios/xcresult.dart';
import 'package:flutter_tools/src/macos/xcode.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';
import 'xcresult_test_data.dart';

void main() {
  // Creates a FakeCommand for the xcresult get call to build the app
  // in the given configuration.
  FakeCommand setUpFakeXCResultGetCommand({
    required String stdout,
    required String tempResultPath,
    required Xcode xcode,
    int exitCode = 0,
    String stderr = '',
    bool useLegacyFlag = true,
  }) {
    return FakeCommand(
      command: <String>[
        ...xcode.xcrunCommand(),
        'xcresulttool',
        'get',
        if (useLegacyFlag) '--legacy',
        '--path',
        tempResultPath,
        '--format',
        'json',
      ],
      stdout: stdout,
      stderr: stderr,
      exitCode: exitCode,
      onRun: (_) {},
    );
  }

  const FakeCommand kWhichSysctlCommand = FakeCommand(command: <String>['which', 'sysctl']);

  const FakeCommand kx64CheckCommand = FakeCommand(
    command: <String>['sysctl', 'hw.optional.arm64'],
    exitCode: 1,
  );

  XCResultGenerator setupGenerator({
    required String resultJson,
    int exitCode = 0,
    String stderr = '',
    Version? xcodeVersion = const Version.withText(16, 0, 0, '16.0'),
    bool useLegacyFlag = true,
  }) {
    final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
      kWhichSysctlCommand,
      kx64CheckCommand,
    ]);
    final Xcode xcode = Xcode.test(
      processManager: fakeProcessManager,
      xcodeProjectInterpreter: XcodeProjectInterpreter.test(
        processManager: fakeProcessManager,
        version: xcodeVersion,
      ),
    );
    fakeProcessManager.addCommands(<FakeCommand>[
      setUpFakeXCResultGetCommand(
        stdout: resultJson,
        tempResultPath: _tempResultPath,
        xcode: xcode,
        exitCode: exitCode,
        stderr: stderr,
        useLegacyFlag: useLegacyFlag,
      ),
    ]);
    final ProcessUtils processUtils = ProcessUtils(
      processManager: fakeProcessManager,
      logger: BufferLogger.test(),
    );
    return XCResultGenerator(resultPath: _tempResultPath, xcode: xcode, processUtils: processUtils);
  }

  testWithoutContext('correctly parse sample result json when there are issues.', () async {
    final XCResultGenerator generator = setupGenerator(resultJson: kSampleResultJsonWithIssues);
    final XCResult result = await generator.generate();
    expect(result.issues.length, 2);
    expect(result.issues.first.type, XCResultIssueType.error);
    expect(result.issues.first.subType, 'Semantic Issue');
    expect(result.issues.first.message, "Use of undeclared identifier 'asdas'");
    expect(
      result.issues.first.location,
      '/Users/m/Projects/test_create/ios/Runner/AppDelegate.m:7:56',
    );
    expect(result.issues.last.type, XCResultIssueType.warning);
    expect(result.issues.last.subType, 'Warning');
    expect(
      result.issues.last.message,
      "The iOS deployment target 'IPHONEOS_DEPLOYMENT_TARGET' is set to 8.0, but the range of supported deployment target versions is 9.0 to 14.0.99.",
    );
    expect(result.parseSuccess, isTrue);
    expect(result.parsingErrorMessage, isNull);
  });

  testWithoutContext(
    'correctly parse sample result json when there are issues but invalid url.',
    () async {
      final XCResultGenerator generator = setupGenerator(
        resultJson: kSampleResultJsonWithIssuesAndInvalidUrl,
      );
      final XCResult result = await generator.generate();
      expect(result.issues.length, 2);
      expect(result.issues.first.type, XCResultIssueType.error);
      expect(result.issues.first.subType, 'Semantic Issue');
      expect(result.issues.first.message, "Use of undeclared identifier 'asdas'");
      expect(result.issues.first.location, isNull);
      expect(
        result.issues.first.warnings.first,
        '(XCResult) The `url` exists but it was failed to be parsed. url: 3:00',
      );
      expect(result.issues.last.type, XCResultIssueType.warning);
      expect(result.issues.last.subType, 'Warning');
      expect(
        result.issues.last.message,
        "The iOS deployment target 'IPHONEOS_DEPLOYMENT_TARGET' is set to 8.0, but the range of supported deployment target versions is 9.0 to 14.0.99.",
      );
      expect(result.parseSuccess, isTrue);
      expect(result.parsingErrorMessage, isNull);
    },
  );

  testWithoutContext('correctly parse sample result json and discard all warnings', () async {
    final XCResultGenerator generator = setupGenerator(resultJson: kSampleResultJsonWithIssues);
    final XCResultIssueDiscarder discarder = XCResultIssueDiscarder(
      typeMatcher: XCResultIssueType.warning,
    );
    final XCResult result = await generator.generate(
      issueDiscarders: <XCResultIssueDiscarder>[discarder],
    );
    expect(result.issues.length, 1);
    expect(result.issues.first.type, XCResultIssueType.error);
    expect(result.issues.first.subType, 'Semantic Issue');
    expect(result.issues.first.message, "Use of undeclared identifier 'asdas'");
    expect(
      result.issues.first.location,
      '/Users/m/Projects/test_create/ios/Runner/AppDelegate.m:7:56',
    );
    expect(result.parseSuccess, isTrue);
    expect(result.parsingErrorMessage, isNull);
  });

  testWithoutContext('correctly parse sample result json and discard base on subType', () async {
    final XCResultGenerator generator = setupGenerator(resultJson: kSampleResultJsonWithIssues);
    final XCResultIssueDiscarder discarder = XCResultIssueDiscarder(
      subTypeMatcher: RegExp(r'^Warning$'),
    );
    final XCResult result = await generator.generate(
      issueDiscarders: <XCResultIssueDiscarder>[discarder],
    );
    expect(result.issues.length, 1);
    expect(result.issues.first.type, XCResultIssueType.error);
    expect(result.issues.first.subType, 'Semantic Issue');
    expect(result.issues.first.message, "Use of undeclared identifier 'asdas'");
    expect(
      result.issues.first.location,
      '/Users/m/Projects/test_create/ios/Runner/AppDelegate.m:7:56',
    );
    expect(result.parseSuccess, isTrue);
    expect(result.parsingErrorMessage, isNull);
  });

  testWithoutContext('correctly parse sample result json and discard base on message', () async {
    final XCResultGenerator generator = setupGenerator(resultJson: kSampleResultJsonWithIssues);
    final XCResultIssueDiscarder discarder = XCResultIssueDiscarder(
      messageMatcher: RegExp(
        r"^The iOS deployment target 'IPHONEOS_DEPLOYMENT_TARGET' is set to 8.0, but the range of supported deployment target versions is 9.0 to 14.0.99.$",
      ),
    );
    final XCResult result = await generator.generate(
      issueDiscarders: <XCResultIssueDiscarder>[discarder],
    );
    expect(result.issues.length, 1);
    expect(result.issues.first.type, XCResultIssueType.error);
    expect(result.issues.first.subType, 'Semantic Issue');
    expect(result.issues.first.message, "Use of undeclared identifier 'asdas'");
    expect(
      result.issues.first.location,
      '/Users/m/Projects/test_create/ios/Runner/AppDelegate.m:7:56',
    );
    expect(result.parseSuccess, isTrue);
    expect(result.parsingErrorMessage, isNull);
  });

  testWithoutContext('correctly parse sample result json and discard base on location', () async {
    final XCResultGenerator generator = setupGenerator(resultJson: kSampleResultJsonWithIssues);
    final XCResultIssueDiscarder discarder = XCResultIssueDiscarder(
      locationMatcher: RegExp(r'/Users/m/Projects/test_create/ios/Runner/AppDelegate.m'),
    );
    final XCResult result = await generator.generate(
      issueDiscarders: <XCResultIssueDiscarder>[discarder],
    );
    expect(result.issues.length, 1);
    expect(result.issues.first.type, XCResultIssueType.warning);
    expect(result.issues.first.subType, 'Warning');
    expect(
      result.issues.first.message,
      "The iOS deployment target 'IPHONEOS_DEPLOYMENT_TARGET' is set to 8.0, but the range of supported deployment target versions is 9.0 to 14.0.99.",
    );
    expect(result.parseSuccess, isTrue);
    expect(result.parsingErrorMessage, isNull);
  });

  testWithoutContext('correctly parse sample result json with multiple discarders.', () async {
    final XCResultGenerator generator = setupGenerator(resultJson: kSampleResultJsonWithIssues);
    final XCResultIssueDiscarder discardWarnings = XCResultIssueDiscarder(
      typeMatcher: XCResultIssueType.warning,
    );
    final XCResultIssueDiscarder discardSemanticIssues = XCResultIssueDiscarder(
      subTypeMatcher: RegExp(r'^Semantic Issue$'),
    );
    final XCResult result = await generator.generate(
      issueDiscarders: <XCResultIssueDiscarder>[discardWarnings, discardSemanticIssues],
    );
    expect(result.issues, isEmpty);
    expect(result.parseSuccess, isTrue);
    expect(result.parsingErrorMessage, isNull);
  });

  testWithoutContext('correctly parse sample result json when no issues.', () async {
    final XCResultGenerator generator = setupGenerator(resultJson: kSampleResultJsonNoIssues);
    final XCResult result = await generator.generate();
    expect(result.issues.length, 0);
    expect(result.parseSuccess, isTrue);
    expect(result.parsingErrorMessage, isNull);
  });

  testWithoutContext('correctly parse sample result json with action issues.', () async {
    final XCResultGenerator generator = setupGenerator(
      resultJson: kSampleResultJsonWithActionIssues,
    );
    final XCResultIssueDiscarder discarder = XCResultIssueDiscarder(
      typeMatcher: XCResultIssueType.warning,
    );
    final XCResult result = await generator.generate(
      issueDiscarders: <XCResultIssueDiscarder>[discarder],
    );
    expect(result.issues.length, 1);
    expect(result.issues.first.type, XCResultIssueType.error);
    expect(result.issues.first.subType, 'Uncategorized');
    expect(
      result.issues.first.message,
      contains('Unable to find a destination matching the provided destination specifier'),
    );
    expect(result.parseSuccess, isTrue);
    expect(result.parsingErrorMessage, isNull);
  });

  testWithoutContext('correctly parse sample result on < Xcode 16.', () async {
    final XCResultGenerator generator = setupGenerator(
      resultJson: kSampleResultJsonNoIssues,
      xcodeVersion: Version(15, 0, 0),
      useLegacyFlag: false,
    );
    final XCResult result = await generator.generate();
    expect(result.issues.length, 0);
    expect(result.parseSuccess, isTrue);
    expect(result.parsingErrorMessage, isNull);
  });

  testWithoutContext(
    'error: `xcresulttool get` process fail should return an `XCResult` with stderr as `parsingErrorMessage`.',
    () async {
      const String fakeStderr = 'Fake: fail to parse result json.';
      final XCResultGenerator generator = setupGenerator(
        resultJson: '',
        exitCode: 1,
        stderr: fakeStderr,
      );

      final XCResult result = await generator.generate();
      expect(result.issues.length, 0);
      expect(result.parseSuccess, false);
      expect(result.parsingErrorMessage, fakeStderr);
    },
  );

  testWithoutContext('error: `xcresulttool get` no stdout', () async {
    final XCResultGenerator generator = setupGenerator(resultJson: '');

    final XCResult result = await generator.generate();
    expect(result.issues.length, 0);
    expect(result.parseSuccess, false);
    expect(result.parsingErrorMessage, 'xcresult parser: Unrecognized top level json format.');
  });

  testWithoutContext('error: wrong top level json format.', () async {
    final XCResultGenerator generator = setupGenerator(resultJson: '[]');

    final XCResult result = await generator.generate();
    expect(result.issues.length, 0);
    expect(result.parseSuccess, false);
    expect(result.parsingErrorMessage, 'xcresult parser: Unrecognized top level json format.');
  });

  testWithoutContext('error: fail to parse issue map', () async {
    final XCResultGenerator generator = setupGenerator(resultJson: '{}');

    final XCResult result = await generator.generate();
    expect(result.issues.length, 0);
    expect(result.parseSuccess, false);
    expect(result.parsingErrorMessage, 'xcresult parser: Failed to parse the issues map.');
  });

  testWithoutContext('error: invalid issue map', () async {
    final XCResultGenerator generator = setupGenerator(
      resultJson: kSampleResultJsonInvalidIssuesMap,
    );

    final XCResult result = await generator.generate();
    expect(result.issues.length, 0);
    expect(result.parseSuccess, false);
    expect(result.parsingErrorMessage, 'xcresult parser: Failed to parse the issues map.');
  });
}

const String _tempResultPath = 'temp';
