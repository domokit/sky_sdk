// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import '../debug_adapters/server.dart';
import '../globals.dart' as globals;
import '../runner/flutter_command.dart';

/// This command will start up a Debug Adapter that communicates using the Debug Adapter Protocol (DAP).
///
/// This is for use by editors and IDEs that have DAP clients to launch and
/// debug Flutter apps/tests. It extends the standard Dart DAP implementation
/// from DDS with Flutter-specific functionality (such as Hot Restart).
///
/// The server is intended to be single-use. It should live only for the
/// duration of a single debug session in the editor, and terminate when the
/// user stops debugging. If a user starts multiple debug sessions
/// simultaneously it is expected that the editor will start multiple debug
/// adapters.
///
/// The DAP specification can be found at
/// https://microsoft.github.io/debug-adapter-protocol/.
class DebugAdapterCommand extends FlutterCommand {
  DebugAdapterCommand({bool verboseHelp = false}) : hidden = !verboseHelp {
    usesIpv6Flag(verboseHelp: verboseHelp);
    addDdsOptions(verboseHelp: verboseHelp);
    argParser.addFlag(
      'test',
      help:
          'Whether to use the "flutter test" debug adapter to run tests'
          ' and emit custom events for test progress/results.',
    );
  }

  @override
  final String name = 'debug-adapter';

  @override
  List<String> get aliases => const <String>['debug_adapter'];

  @override
  final String description =
      'Run a Debug Adapter Protocol (DAP) server to communicate with the Flutter tool.';

  @override
  final String category = FlutterCommandCategory.tools;

  @override
  final bool hidden;

  @override
  Future<FlutterCommandResult> runCommand() async {
    final DapServer server = DapServer(
      globals.stdio.stdin,
      globals.stdio.stdout.nonBlocking,
      fileSystem: globals.fs,
      platform: globals.platform,
      ipv6: ipv6 ?? false,
      enableDds: enableDds,
      test: boolArg('test'),
      onError: (Object? e) {
        globals.printError(
          'Input could not be parsed as a Debug Adapter Protocol message.\n'
          'The "flutter debug-adapter" command is intended for use by tooling '
          'that communicates using the Debug Adapter Protocol.\n\n'
          '$e',
        );
      },
    );

    await server.channel.closed;

    return FlutterCommandResult.success();
  }
}
