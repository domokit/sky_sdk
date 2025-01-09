// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:file/memory.dart';
import 'package:platform/platform.dart';

import '../../../packages/flutter_tools/test/src/fake_process_manager.dart';
import '../post_process_docs.dart';
import 'common.dart';

void main() async {
  group('getBranch', () {
    const String branchName = 'stable';
    test('getBranchName does not call git if env LUCI_BRANCH provided', () async {
      final Platform platform = FakePlatform(
        environment: <String, String>{'LUCI_BRANCH': branchName},
      );
      final ProcessManager processManager = FakeProcessManager.empty();
      final String calculatedBranchName = await getBranchName(
        platform: platform,
        processManager: processManager,
      );
      expect(calculatedBranchName, branchName);
    });

    test('getBranchName calls git if env LUCI_BRANCH not provided', () async {
      final Platform platform = FakePlatform(environment: <String, String>{});

      final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['git', 'status', '-b', '--porcelain'],
          stdout: '## $branchName',
        ),
      ]);

      final String calculatedBranchName = await getBranchName(
        platform: platform,
        processManager: processManager,
      );
      expect(calculatedBranchName, branchName);
      expect(processManager, hasNoRemainingExpectations);
    });
    test('getBranchName calls git if env LUCI_BRANCH is empty', () async {
      final Platform platform = FakePlatform(environment: <String, String>{'LUCI_BRANCH': ''});

      final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['git', 'status', '-b', '--porcelain'],
          stdout: '## $branchName',
        ),
      ]);
      final String calculatedBranchName = await getBranchName(
        platform: platform,
        processManager: processManager,
      );
      expect(calculatedBranchName, branchName);
      expect(processManager, hasNoRemainingExpectations);
    });
  });

  group('gitRevision', () {
    test('Return short format', () async {
      const String commitHash = 'e65f01793938e13cac2d321b9fcdc7939f9b2ea6';
      final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: commitHash),
      ]);
      final String revision = await gitRevision(processManager: processManager);
      expect(processManager, hasNoRemainingExpectations);
      expect(revision, commitHash.substring(0, 10));
    });

    test('Return full length', () async {
      const String commitHash = 'e65f01793938e13cac2d321b9fcdc7939f9b2ea6';
      final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(command: <String>['git', 'rev-parse', 'HEAD'], stdout: commitHash),
      ]);
      final String revision = await gitRevision(fullLength: true, processManager: processManager);
      expect(processManager, hasNoRemainingExpectations);
      expect(revision, commitHash);
    });
  });

  group('runProcessWithValidation', () {
    test('With no error', () async {
      const List<String> command = <String>['git', 'rev-parse', 'HEAD'];
      final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(command: command),
      ]);
      await runProcessWithValidations(command, '', processManager: processManager, verbose: false);
      expect(processManager, hasNoRemainingExpectations);
    });

    test('With error', () async {
      const List<String> command = <String>['git', 'rev-parse', 'HEAD'];
      final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(command: command, exitCode: 1),
      ]);
      try {
        await runProcessWithValidations(
          command,
          '',
          processManager: processManager,
          verbose: false,
        );
        throw Exception('Exception was not thrown');
      } on CommandException catch (e) {
        expect(e, isA<Exception>());
      }
    });
  });

  group('generateFooter', () {
    test('generated correctly', () async {
      const String expectedContent = '''
(function() {
  var span = document.querySelector('footer>span');
  if (span) {
    span.innerText = 'Flutter 3.0.0 • 2022-09-22 14:09 • abcdef • stable';
  }
  var sourceLink = document.querySelector('a.source-link');
  if (sourceLink) {
    sourceLink.href = sourceLink.href.replace('/master/', '/abcdef/');
  }
})();
''';
      final MemoryFileSystem fs = MemoryFileSystem();
      final File footerFile = fs.file('/a/b/c/footer.js')..createSync(recursive: true);
      await createFooter(
        footerFile,
        '3.0.0',
        timestampParam: '2022-09-22 14:09',
        branchParam: 'stable',
        revisionParam: 'abcdef',
      );
      final String content = await footerFile.readAsString();
      expect(content, expectedContent);
    });
  });
}
