// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/runner/flutter_command.dart';

typedef CommandFunction = Future<FlutterCommandResult> Function();

class DummyFlutterCommand extends FlutterCommand {
  DummyFlutterCommand({
    this.shouldUpdateCache = false,
    this.noUsagePath = false,
    this.name = 'dummy',
    this.commandFunction,
    this.packagesPath,
    this.fileSystemScheme,
    this.fileSystemRoots = const <String>[],
  });

  final bool noUsagePath;
  final CommandFunction? commandFunction;

  @override
  final bool shouldUpdateCache;

  @override
  String get description => 'does nothing';

  @override
  Future<String?> get usagePath async => noUsagePath ? null : super.usagePath;

  @override
  final String name;

  @override
  Future<FlutterCommandResult> runCommand() async {
    return commandFunction == null ? FlutterCommandResult.fail() : await commandFunction!();
  }

  @override
  final String? packagesPath;

  @override
  final String? fileSystemScheme;

  @override
  final List<String> fileSystemRoots;
}
