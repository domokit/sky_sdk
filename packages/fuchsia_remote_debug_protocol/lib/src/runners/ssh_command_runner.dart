// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' show ProcessResult;

import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../common/logging.dart';
import '../common/network.dart';

/// An error raised when a command fails to run within the [SshCommandRunner].
///
/// This occurs for both connection failures, and for failure to
/// run the command on the remote device. This error is raised when the
/// subprocess running the SSH command returns a nonzero exit code.
class SshCommandError extends Error {
  /// Basic constructor outlining the reason for the SSH command failure through
  /// the message string.
  SshCommandError(this.message);

  /// The reason for the command failure.
  final String message;

  @override
  String toString() {
    return '$SshCommandError: $message\n${super.stackTrace}';
  }
}

/// Runs commands remotely on a Fuchsia device.
///
/// Requires a Fuchsia root and build type (to load the ssh config),
/// and the address of the Fuchsia device.
class SshCommandRunner {
  /// Instantiates the command runner, pointing to an `address` as well as
  /// an optional SSH config file path.
  ///
  /// If the SSH config path is supplied as an empty string, behavior is
  /// undefined.
  ///
  /// [ArgumentError] is thrown in the event that `address` is neither valid
  /// IPv4 nor IPv6. When connecting to a link local address (`fe80::` is
  /// usually at the start of the address), an interface should be supplied.
  SshCommandRunner({required this.address, this.interface = '', this.sshConfigPath})
    : _processManager = const LocalProcessManager() {
    validateAddress(address);
  }

  /// Private constructor for dependency injection of the process manager.
  @visibleForTesting
  SshCommandRunner.withProcessManager(
    this._processManager, {
    required this.address,
    this.interface = '',
    this.sshConfigPath,
  }) {
    validateAddress(address);
  }

  final Logger _log = Logger('SshCommandRunner');

  final ProcessManager _processManager;

  /// The IPv4 address to access the Fuchsia machine over SSH.
  final String address;

  /// The path to the SSH config (optional).
  final String? sshConfigPath;

  /// The name of the machine's network interface (for use with IPv6
  /// connections. Ignored otherwise).
  final String interface;

  /// Runs a command on a Fuchsia device through an SSH tunnel.
  ///
  /// If the subprocess creating the SSH tunnel returns a nonzero exit status,
  /// then an [SshCommandError] is raised.
  Future<List<String>> run(String command) async {
    final List<String> args = <String>[
      'ssh',
      if (sshConfigPath != null) ...<String>['-F', sshConfigPath!],
      if (isIpV6Address(address)) ...<String>[
        '-6',
        if (interface.isEmpty) address else '$address%$interface',
      ] else
        address,
      command,
    ];
    _log.fine('Running command through SSH: ${args.join(' ')}');
    final ProcessResult result = await _processManager.run(args);
    if (result.exitCode != 0) {
      throw SshCommandError(
        'Command failed: $command\nstdout: ${result.stdout}\nstderr: ${result.stderr}',
      );
    }
    _log.fine('SSH command stdout in brackets:[${result.stdout}]');
    return (result.stdout as String).split('\n');
  }
}
