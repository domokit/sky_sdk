// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/signals.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/commands/attach.dart';
import 'package:flutter_tools/src/commands/build.dart';
import 'package:flutter_tools/src/commands/build_aar.dart';
import 'package:flutter_tools/src/commands/build_apk.dart';
import 'package:flutter_tools/src/commands/build_appbundle.dart';
import 'package:flutter_tools/src/commands/build_ios.dart';
import 'package:flutter_tools/src/commands/build_ios_framework.dart';
import 'package:flutter_tools/src/commands/build_linux.dart';
import 'package:flutter_tools/src/commands/build_macos.dart';
import 'package:flutter_tools/src/commands/build_web.dart';
import 'package:flutter_tools/src/commands/build_windows.dart';
import 'package:flutter_tools/src/runner/flutter_command.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';
import '../../src/test_build_system.dart';

class FakeTerminal extends Fake implements AnsiTerminal {
  FakeTerminal({this.stdinHasTerminal = true});

  @override
  final bool stdinHasTerminal;
}

class FakeProcessInfo extends Fake implements ProcessInfo {
  @override
  int maxRss = 0;
}

void main() {
  testUsingContext('All build commands support null safety options', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final Platform platform = FakePlatform();
    final BufferLogger logger = BufferLogger.test();
    final List<FlutterCommand> commands = <FlutterCommand>[
      BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      ),
      BuildLinuxCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      ),
      BuildMacosCommand(logger: BufferLogger.test(), verboseHelp: false),
      BuildWebCommand(fileSystem: fileSystem, logger: BufferLogger.test(), verboseHelp: false),
      BuildApkCommand(logger: BufferLogger.test()),
      BuildIOSCommand(logger: BufferLogger.test(), verboseHelp: false),
      BuildIOSArchiveCommand(logger: BufferLogger.test(), verboseHelp: false),
      BuildAppBundleCommand(logger: BufferLogger.test()),
      BuildAarCommand(
        logger: BufferLogger.test(),
        androidSdk: FakeAndroidSdk(),
        fileSystem: fileSystem,
        verboseHelp: false,
      ),
      BuildIOSFrameworkCommand(
        logger: BufferLogger.test(),
        verboseHelp: false,
        buildSystem: FlutterBuildSystem(fileSystem: fileSystem, platform: platform, logger: logger),
      ),
      AttachCommand(
        stdio: FakeStdio(),
        logger: logger,
        terminal: FakeTerminal(),
        signals: Signals.test(),
        platform: platform,
        processInfo: FakeProcessInfo(),
        fileSystem: MemoryFileSystem.test(),
      ),
    ];

    for (final FlutterCommand command in commands) {
      final ArgResults results = command.argParser.parse(<String>[
        '--sound-null-safety',
        '--enable-experiment=non-nullable',
      ]);

      expect(results.wasParsed('sound-null-safety'), true);
      expect(results.wasParsed('enable-experiment'), true);
    }
  });

  testUsingContext('BuildSubCommand displays current null safety mode', () async {
    const BuildInfo unsound = BuildInfo(
      BuildMode.debug,
      '',
      nullSafetyMode: NullSafetyMode.unsound,
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );

    final BufferLogger logger = BufferLogger.test();
    FakeBuildSubCommand(logger).test(unsound);
    expect(logger.statusText, contains('Building without sound null safety ⚠️'));
  });

  testUsingContext('Include only supported sub commands', () {
    final BufferLogger logger = BufferLogger.test();
    final ProcessUtils processUtils = ProcessUtils(
      logger: logger,
      processManager: FakeProcessManager.empty(),
    );
    final MemoryFileSystem fs = MemoryFileSystem.test();
    final BuildCommand command = BuildCommand(
      artifacts: Artifacts.test(fileSystem: fs),
      androidSdk: FakeAndroidSdk(),
      buildSystem: TestBuildSystem.all(BuildResult(success: true)),
      fileSystem: fs,
      logger: logger,
      processUtils: processUtils,
      osUtils: FakeOperatingSystemUtils(),
    );
    for (final Command<void> x in command.subcommands.values) {
      expect((x as BuildSubCommand).supported, isTrue);
    }
  });
}

class FakeBuildSubCommand extends BuildSubCommand {
  FakeBuildSubCommand(Logger logger) : super(logger: logger, verboseHelp: false);

  @override
  String get description => throw UnimplementedError();

  @override
  String get name => throw UnimplementedError();

  void test(BuildInfo buildInfo) {
    displayNullSafetyMode(buildInfo);
  }

  @override
  Future<FlutterCommandResult> runCommand() {
    throw UnimplementedError();
  }
}
