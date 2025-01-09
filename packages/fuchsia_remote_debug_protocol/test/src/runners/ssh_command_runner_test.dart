// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' show ProcessResult, systemEncoding;

import 'package:fuchsia_remote_debug_protocol/src/runners/ssh_command_runner.dart';
import 'package:process/process.dart';
import 'package:test/fake.dart';

import 'package:test/test.dart';

void main() {
  group('SshCommandRunner.constructors', () {
    test('throws exception with invalid address', () async {
      SshCommandRunner newCommandRunner() {
        return SshCommandRunner(address: 'sillyaddress.what');
      }

      expect(newCommandRunner, throwsArgumentError);
    });

    test('throws exception from injection constructor with invalid addr', () async {
      SshCommandRunner newCommandRunner() {
        return SshCommandRunner.withProcessManager(
          const LocalProcessManager(),
          address: '192.168.1.1.1',
        );
      }

      expect(newCommandRunner, throwsArgumentError);
    });
  });

  group('SshCommandRunner.run', () {
    late FakeProcessManager fakeProcessManager;
    SshCommandRunner runner;

    setUp(() {
      fakeProcessManager = FakeProcessManager();
    });

    test('verify interface is appended to ipv6 address', () async {
      const String ipV6Addr = 'fe80::8eae:4cff:fef4:9247';
      const String interface = 'eno1';
      runner = SshCommandRunner.withProcessManager(
        fakeProcessManager,
        address: ipV6Addr,
        interface: interface,
        sshConfigPath: '/whatever',
      );
      fakeProcessManager.fakeResult = ProcessResult(23, 0, 'somestuff', null);
      await runner.run('ls /whatever');
      expect(fakeProcessManager.runCommands.single, contains('$ipV6Addr%$interface'));
    });

    test('verify no percentage symbol is added when no ipv6 interface', () async {
      const String ipV6Addr = 'fe80::8eae:4cff:fef4:9247';
      runner = SshCommandRunner.withProcessManager(fakeProcessManager, address: ipV6Addr);
      fakeProcessManager.fakeResult = ProcessResult(23, 0, 'somestuff', null);
      await runner.run('ls /whatever');
      expect(fakeProcessManager.runCommands.single, contains(ipV6Addr));
    });

    test('verify commands are split into multiple lines', () async {
      const String addr = '192.168.1.1';
      runner = SshCommandRunner.withProcessManager(fakeProcessManager, address: addr);
      fakeProcessManager.fakeResult = ProcessResult(23, 0, '''
          this
          has
          four
          lines''', null);
      final List<String> result = await runner.run('oihaw');
      expect(result, hasLength(4));
    });

    test('verify exception on nonzero process result exit code', () async {
      const String addr = '192.168.1.1';
      runner = SshCommandRunner.withProcessManager(fakeProcessManager, address: addr);
      fakeProcessManager.fakeResult = ProcessResult(23, 1, 'whatever', null);
      Future<void> failingFunction() async {
        await runner.run('oihaw');
      }

      expect(failingFunction, throwsA(isA<SshCommandError>()));
    });

    test('verify correct args with config', () async {
      const String addr = 'fe80::8eae:4cff:fef4:9247';
      const String config = '/this/that/this/and/uh';
      runner = SshCommandRunner.withProcessManager(
        fakeProcessManager,
        address: addr,
        sshConfigPath: config,
      );
      fakeProcessManager.fakeResult = ProcessResult(23, 0, 'somestuff', null);
      await runner.run('ls /whatever');
      final List<String?> passedCommand = fakeProcessManager.runCommands.single as List<String?>;
      expect(passedCommand, contains('-F'));
      final int indexOfFlag = passedCommand.indexOf('-F');
      final String? passedConfig = passedCommand[indexOfFlag + 1];
      expect(passedConfig, config);
    });

    test('verify config is excluded correctly', () async {
      const String addr = 'fe80::8eae:4cff:fef4:9247';
      runner = SshCommandRunner.withProcessManager(fakeProcessManager, address: addr);
      fakeProcessManager.fakeResult = ProcessResult(23, 0, 'somestuff', null);
      await runner.run('ls /whatever');
      final List<String?> passedCommand = fakeProcessManager.runCommands.single as List<String?>;
      final int indexOfFlag = passedCommand.indexOf('-F');
      expect(indexOfFlag, equals(-1));
    });
  });
}

class FakeProcessManager extends Fake implements ProcessManager {
  ProcessResult? fakeResult;

  List<List<dynamic>> runCommands = <List<dynamic>>[];

  @override
  Future<ProcessResult> run(
    List<dynamic> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    Encoding? stdoutEncoding = systemEncoding,
    Encoding? stderrEncoding = systemEncoding,
  }) async {
    runCommands.add(command);
    return fakeResult!;
  }
}
