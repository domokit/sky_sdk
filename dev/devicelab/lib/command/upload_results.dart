// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';

import '../framework/cocoon.dart';
import '../framework/metrics_center.dart';

class UploadResultsCommand extends Command<void> {
  UploadResultsCommand() {
    argParser.addOption('results-file', help: 'Test results JSON to upload to Cocoon.');
    argParser.addOption(
      'service-account-token-file',
      help: 'Authentication token for uploading results.',
    );
    argParser.addOption(
      'test-flaky',
      help: 'Flag to show whether the test is flaky: "True" or "False"',
    );
    argParser.addOption(
      'git-branch',
      help:
          '[Flutter infrastructure] Git branch of the current commit. LUCI\n'
          'checkouts run in detached HEAD state, so the branch must be passed.',
    );
    argParser.addOption(
      'luci-builder',
      help: '[Flutter infrastructure] Name of the LUCI builder being run on.',
    );
    argParser.addOption(
      'task-name',
      help: '[Flutter infrastructure] Name of the task being run on.',
    );
    argParser.addOption(
      'benchmark-tags',
      help: '[Flutter infrastructure] Benchmark tags to surface on Skia Perf',
    );
    argParser.addOption('test-status', help: 'Test status: Succeeded|Failed');
    argParser.addOption('commit-time', help: 'Commit time in UNIX timestamp');
    argParser.addOption(
      'builder-bucket',
      help: '[Flutter infrastructure] Luci builder bucket the test is running in.',
    );
  }

  @override
  String get name => 'upload-metrics';

  @override
  String get description => '[Flutter infrastructure] Upload results data to Cocoon/Skia Perf';

  @override
  Future<void> run() async {
    final String? resultsPath = argResults!['results-file'] as String?;
    final String? serviceAccountTokenFile = argResults!['service-account-token-file'] as String?;
    final String? testFlakyStatus = argResults!['test-flaky'] as String?;
    final String? gitBranch = argResults!['git-branch'] as String?;
    final String? builderName = argResults!['luci-builder'] as String?;
    final String? testStatus = argResults!['test-status'] as String?;
    final String? commitTime = argResults!['commit-time'] as String?;
    final String? taskName = argResults!['task-name'] as String?;
    final String? benchmarkTags = argResults!['benchmark-tags'] as String?;
    final String? builderBucket = argResults!['builder-bucket'] as String?;

    // Upload metrics to skia perf from test runner when `resultsPath` is specified.
    if (resultsPath != null) {
      await uploadToSkiaPerf(resultsPath, commitTime, taskName, benchmarkTags);
      print('Successfully uploaded metrics to skia perf');
    }

    final Cocoon cocoon = Cocoon(serviceAccountTokenPath: serviceAccountTokenFile);
    return cocoon.sendTaskStatus(
      resultsPath: resultsPath,
      isTestFlaky: testFlakyStatus == 'True',
      gitBranch: gitBranch,
      builderName: builderName,
      testStatus: testStatus,
      builderBucket: builderBucket,
    );
  }
}
