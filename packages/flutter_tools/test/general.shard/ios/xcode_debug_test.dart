// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/ios/xcode_debug.dart';
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/macos/xcode.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';

void main() {
  group('Debug project through Xcode', () {
    late MemoryFileSystem fileSystem;
    late BufferLogger logger;
    late FakeProcessManager fakeProcessManager;

    const String flutterRoot = '/path/to/flutter';
    const String pathToXcodeAutomationScript =
        '$flutterRoot/packages/flutter_tools/bin/xcode_debug.js';

    setUp(() {
      fileSystem = MemoryFileSystem.test();
      logger = BufferLogger.test();
      fakeProcessManager = FakeProcessManager.empty();
    });

    group('debugApp', () {
      const String pathToXcodeApp = '/Applications/Xcode.app';
      const String deviceId = '0000001234';

      late Xcode xcode;
      late Directory xcodeproj;
      late Directory xcworkspace;
      late XcodeDebugProject project;

      setUp(() {
        xcode = setupXcode(
          fakeProcessManager: fakeProcessManager,
          fileSystem: fileSystem,
          flutterRoot: flutterRoot,
        );

        xcodeproj = fileSystem.directory('Runner.xcodeproj');
        xcworkspace = fileSystem.directory('Runner.xcworkspace');
        project = XcodeDebugProject(
          scheme: 'Runner',
          xcodeProject: xcodeproj,
          xcodeWorkspace: xcworkspace,
          hostAppProjectName: 'Runner',
        );
      });

      testWithoutContext(
        'succeeds in opening and debugging with launch options, expectedConfigurationBuildDir, and verbose logging',
        () async {
          fakeProcessManager.addCommands(<FakeCommand>[
            FakeCommand(
              command: <String>[
                'xcrun',
                'osascript',
                '-l',
                'JavaScript',
                pathToXcodeAutomationScript,
                'check-workspace-opened',
                '--xcode-path',
                pathToXcodeApp,
                '--project-path',
                project.xcodeProject.path,
                '--workspace-path',
                project.xcodeWorkspace.path,
                '--verbose',
              ],
              stdout: '''
  {"status":false,"errorMessage":"Xcode is not running","debugResult":null}
  ''',
            ),
            FakeCommand(
              command: <String>['open', '-a', pathToXcodeApp, '-g', '-j', '-F', xcworkspace.path],
            ),
            FakeCommand(
              command: <String>[
                'xcrun',
                'osascript',
                '-l',
                'JavaScript',
                pathToXcodeAutomationScript,
                'debug',
                '--xcode-path',
                pathToXcodeApp,
                '--project-path',
                project.xcodeProject.path,
                '--workspace-path',
                project.xcodeWorkspace.path,
                '--project-name',
                project.hostAppProjectName,
                '--expected-configuration-build-dir',
                '/build/ios/iphoneos',
                '--device-id',
                deviceId,
                '--scheme',
                project.scheme,
                '--skip-building',
                '--launch-args',
                r'["--enable-dart-profiling","--trace-allowlist=\"foo,bar\""]',
                '--verbose',
              ],
              stdout: '''
  {"status":true,"errorMessage":null,"debugResult":{"completed":false,"status":"running","errorMessage":null}}
  ''',
            ),
          ]);

          final XcodeDebug xcodeDebug = XcodeDebug(
            logger: logger,
            processManager: fakeProcessManager,
            xcode: xcode,
            fileSystem: fileSystem,
          );

          project = XcodeDebugProject(
            scheme: 'Runner',
            xcodeProject: xcodeproj,
            xcodeWorkspace: xcworkspace,
            hostAppProjectName: 'Runner',
            expectedConfigurationBuildDir: '/build/ios/iphoneos',
            verboseLogging: true,
          );

          final bool status = await xcodeDebug.debugApp(
            project: project,
            deviceId: deviceId,
            launchArguments: <String>['--enable-dart-profiling', '--trace-allowlist="foo,bar"'],
          );

          expect(logger.errorText, isEmpty);
          expect(logger.traceText, contains('Error checking if project opened in Xcode'));
          expect(fakeProcessManager, hasNoRemainingExpectations);
          expect(xcodeDebug.startDebugActionProcess, isNull);
          expect(status, true);
        },
      );

      testWithoutContext(
        'succeeds in opening and debugging without launch options, expectedConfigurationBuildDir, and verbose logging',
        () async {
          fakeProcessManager.addCommands(<FakeCommand>[
            FakeCommand(
              command: <String>[
                'xcrun',
                'osascript',
                '-l',
                'JavaScript',
                pathToXcodeAutomationScript,
                'check-workspace-opened',
                '--xcode-path',
                pathToXcodeApp,
                '--project-path',
                project.xcodeProject.path,
                '--workspace-path',
                project.xcodeWorkspace.path,
              ],
              stdout: '''
  {"status":false,"errorMessage":"Xcode is not running","debugResult":null}
  ''',
            ),
            FakeCommand(
              command: <String>['open', '-a', pathToXcodeApp, '-g', '-j', '-F', xcworkspace.path],
            ),
            FakeCommand(
              command: <String>[
                'xcrun',
                'osascript',
                '-l',
                'JavaScript',
                pathToXcodeAutomationScript,
                'debug',
                '--xcode-path',
                pathToXcodeApp,
                '--project-path',
                project.xcodeProject.path,
                '--workspace-path',
                project.xcodeWorkspace.path,
                '--project-name',
                project.hostAppProjectName,
                '--device-id',
                deviceId,
                '--scheme',
                project.scheme,
                '--skip-building',
                '--launch-args',
                '[]',
              ],
              stdout: '''
  {"status":true,"errorMessage":null,"debugResult":{"completed":false,"status":"running","errorMessage":null}}
  ''',
            ),
          ]);

          final XcodeDebug xcodeDebug = XcodeDebug(
            logger: logger,
            processManager: fakeProcessManager,
            xcode: xcode,
            fileSystem: fileSystem,
          );

          final bool status = await xcodeDebug.debugApp(
            project: project,
            deviceId: deviceId,
            launchArguments: <String>[],
          );

          expect(logger.errorText, isEmpty);
          expect(logger.traceText, contains('Error checking if project opened in Xcode'));
          expect(fakeProcessManager, hasNoRemainingExpectations);
          expect(xcodeDebug.startDebugActionProcess, isNull);
          expect(status, true);
        },
      );

      testWithoutContext('fails if project fails to open', () async {
        fakeProcessManager.addCommands(<FakeCommand>[
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'check-workspace-opened',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
            ],
            stdout: '''
  {"status":false,"errorMessage":"Xcode is not running","debugResult":null}
  ''',
          ),
          FakeCommand(
            command: <String>['open', '-a', pathToXcodeApp, '-g', '-j', '-F', xcworkspace.path],
            exception: ProcessException('open', <String>[
              '-a',
              '/non_existent_path',
              '-g',
              '-j',
              '-F',
              xcworkspace.path,
            ], 'The application /non_existent_path cannot be opened for an unexpected reason'),
          ),
        ]);

        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final bool status = await xcodeDebug.debugApp(
          project: project,
          deviceId: deviceId,
          launchArguments: <String>['--enable-dart-profiling', '--trace-allowlist="foo,bar"'],
        );

        expect(
          logger.errorText,
          contains('The application /non_existent_path cannot be opened for an unexpected reason'),
        );
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(status, false);
      });

      testWithoutContext('fails if osascript errors', () async {
        fakeProcessManager.addCommands(<FakeCommand>[
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'check-workspace-opened',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
            ],
            stdout: '''
  {"status":true,"errorMessage":"","debugResult":null}
  ''',
          ),
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'debug',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
              '--project-name',
              project.hostAppProjectName,
              '--device-id',
              deviceId,
              '--scheme',
              project.scheme,
              '--skip-building',
              '--launch-args',
              r'["--enable-dart-profiling","--trace-allowlist=\"foo,bar\""]',
            ],
            exitCode: 1,
            stderr:
                "/flutter/packages/flutter_tools/bin/xcode_debug.js: execution error: Error: ReferenceError: Can't find variable: y (-2700)",
          ),
        ]);

        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final bool status = await xcodeDebug.debugApp(
          project: project,
          deviceId: deviceId,
          launchArguments: <String>['--enable-dart-profiling', '--trace-allowlist="foo,bar"'],
        );

        expect(logger.errorText, contains('Error executing osascript'));
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(status, false);
      });

      testWithoutContext('fails if osascript output returns false status', () async {
        fakeProcessManager.addCommands(<FakeCommand>[
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'check-workspace-opened',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
            ],
            stdout: '''
  {"status":true,"errorMessage":null,"debugResult":null}
  ''',
          ),
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'debug',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
              '--project-name',
              project.hostAppProjectName,
              '--device-id',
              deviceId,
              '--scheme',
              project.scheme,
              '--skip-building',
              '--launch-args',
              r'["--enable-dart-profiling","--trace-allowlist=\"foo,bar\""]',
            ],
            stdout: '''
  {"status":false,"errorMessage":"Unable to find target device.","debugResult":null}
  ''',
          ),
        ]);

        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final bool status = await xcodeDebug.debugApp(
          project: project,
          deviceId: deviceId,
          launchArguments: <String>['--enable-dart-profiling', '--trace-allowlist="foo,bar"'],
        );

        expect(logger.errorText, contains('Error starting debug session in Xcode'));
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(status, false);
      });

      testWithoutContext('fails if missing debug results', () async {
        fakeProcessManager.addCommands(<FakeCommand>[
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'check-workspace-opened',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
            ],
            stdout: '''
  {"status":true,"errorMessage":null,"debugResult":null}
  ''',
          ),
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'debug',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
              '--project-name',
              project.hostAppProjectName,
              '--device-id',
              deviceId,
              '--scheme',
              project.scheme,
              '--skip-building',
              '--launch-args',
              r'["--enable-dart-profiling","--trace-allowlist=\"foo,bar\""]',
            ],
            stdout: '''
  {"status":true,"errorMessage":null,"debugResult":null}
  ''',
          ),
        ]);

        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final bool status = await xcodeDebug.debugApp(
          project: project,
          deviceId: deviceId,
          launchArguments: <String>['--enable-dart-profiling', '--trace-allowlist="foo,bar"'],
        );

        expect(logger.errorText, contains('Unable to get debug results from response'));
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(status, false);
      });

      testWithoutContext('fails if debug results status is not running', () async {
        fakeProcessManager.addCommands(<FakeCommand>[
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'check-workspace-opened',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
            ],
            stdout: '''
  {"status":true,"errorMessage":null,"debugResult":null}
  ''',
          ),
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'debug',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
              '--project-name',
              project.hostAppProjectName,
              '--device-id',
              deviceId,
              '--scheme',
              project.scheme,
              '--skip-building',
              '--launch-args',
              r'["--enable-dart-profiling","--trace-allowlist=\"foo,bar\""]',
            ],
            stdout: '''
  {"status":true,"errorMessage":null,"debugResult":{"completed":false,"status":"not yet started","errorMessage":null}}
  ''',
          ),
        ]);

        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final bool status = await xcodeDebug.debugApp(
          project: project,
          deviceId: deviceId,
          launchArguments: <String>['--enable-dart-profiling', '--trace-allowlist="foo,bar"'],
        );

        expect(logger.errorText, contains('Unexpected debug results'));
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(status, false);
      });
    });

    group('parse script response', () {
      testWithoutContext('fails if osascript output returns non-json output', () async {
        final Xcode xcode = setupXcode(
          fakeProcessManager: FakeProcessManager.any(),
          fileSystem: fileSystem,
          flutterRoot: flutterRoot,
        );
        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final XcodeAutomationScriptResponse? response = xcodeDebug.parseScriptResponse('not json');

        expect(logger.errorText, contains('osascript returned non-JSON response'));
        expect(response, isNull);
      });

      testWithoutContext('fails if osascript output returns unexpected json', () async {
        final Xcode xcode = setupXcode(
          fakeProcessManager: FakeProcessManager.any(),
          fileSystem: fileSystem,
          flutterRoot: flutterRoot,
        );
        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final XcodeAutomationScriptResponse? response = xcodeDebug.parseScriptResponse('[]');

        expect(logger.errorText, contains('osascript returned unexpected JSON response'));
        expect(response, isNull);
      });

      testWithoutContext('fails if osascript output is missing status field', () async {
        final Xcode xcode = setupXcode(
          fakeProcessManager: FakeProcessManager.any(),
          fileSystem: fileSystem,
          flutterRoot: flutterRoot,
        );
        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final XcodeAutomationScriptResponse? response = xcodeDebug.parseScriptResponse('{}');

        expect(logger.errorText, contains('osascript returned unexpected JSON response'));
        expect(response, isNull);
      });
    });

    group('exit', () {
      const String pathToXcodeApp = '/Applications/Xcode.app';

      late Directory projectDirectory;
      late Directory xcodeproj;
      late Directory xcworkspace;

      setUp(() {
        projectDirectory = fileSystem.directory('FlutterApp');
        xcodeproj = projectDirectory.childDirectory('Runner.xcodeproj');
        xcworkspace = projectDirectory.childDirectory('Runner.xcworkspace');
      });

      testWithoutContext('exits when waiting for debug session to start', () async {
        final Xcode xcode = setupXcode(
          fakeProcessManager: fakeProcessManager,
          fileSystem: fileSystem,
          flutterRoot: flutterRoot,
        );
        final XcodeDebugProject project = XcodeDebugProject(
          scheme: 'Runner',
          xcodeProject: xcodeproj,
          xcodeWorkspace: xcworkspace,
          hostAppProjectName: 'Runner',
        );
        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );
        fakeProcessManager.addCommands(<FakeCommand>[
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'stop',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
            ],
            stdout: '''
  {"status":true,"errorMessage":null,"debugResult":null}
  ''',
          ),
        ]);

        xcodeDebug.startDebugActionProcess = FakeProcess();
        xcodeDebug.currentDebuggingProject = project;

        expect(xcodeDebug.startDebugActionProcess, isNotNull);
        expect(xcodeDebug.currentDebuggingProject, isNotNull);

        final bool exitStatus = await xcodeDebug.exit();

        expect((xcodeDebug.startDebugActionProcess! as FakeProcess).killed, isTrue);
        expect(xcodeDebug.currentDebuggingProject, isNull);
        expect(logger.errorText, isEmpty);
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(exitStatus, isTrue);
      });

      testWithoutContext('exits and deletes temporary directory', () async {
        final Xcode xcode = setupXcode(
          fakeProcessManager: fakeProcessManager,
          fileSystem: fileSystem,
          flutterRoot: flutterRoot,
        );
        xcodeproj.createSync(recursive: true);
        xcworkspace.createSync(recursive: true);

        final XcodeDebugProject project = XcodeDebugProject(
          scheme: 'Runner',
          xcodeProject: xcodeproj,
          xcodeWorkspace: xcworkspace,
          hostAppProjectName: 'Runner',
          isTemporaryProject: true,
        );

        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );
        fakeProcessManager.addCommands(<FakeCommand>[
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'stop',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
              '--close-window',
            ],
            stdout: '''
  {"status":true,"errorMessage":null,"debugResult":null}
  ''',
          ),
        ]);

        xcodeDebug.startDebugActionProcess = FakeProcess();
        xcodeDebug.currentDebuggingProject = project;

        expect(xcodeDebug.startDebugActionProcess, isNotNull);
        expect(xcodeDebug.currentDebuggingProject, isNotNull);
        expect(projectDirectory.existsSync(), isTrue);
        expect(xcodeproj.existsSync(), isTrue);
        expect(xcworkspace.existsSync(), isTrue);

        final bool status = await xcodeDebug.exit(skipDelay: true);

        expect((xcodeDebug.startDebugActionProcess! as FakeProcess).killed, isTrue);
        expect(xcodeDebug.currentDebuggingProject, isNull);
        expect(projectDirectory.existsSync(), isFalse);
        expect(xcodeproj.existsSync(), isFalse);
        expect(xcworkspace.existsSync(), isFalse);
        expect(logger.errorText, isEmpty);
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(status, isTrue);
      });

      testWithoutContext(
        'prints error message when deleting temporary directory that is nonexistent',
        () async {
          final Xcode xcode = setupXcode(
            fakeProcessManager: fakeProcessManager,
            fileSystem: fileSystem,
            flutterRoot: flutterRoot,
          );
          final XcodeDebugProject project = XcodeDebugProject(
            scheme: 'Runner',
            xcodeProject: xcodeproj,
            xcodeWorkspace: xcworkspace,
            hostAppProjectName: 'Runner',
            isTemporaryProject: true,
          );
          final XcodeDebug xcodeDebug = XcodeDebug(
            logger: logger,
            processManager: fakeProcessManager,
            xcode: xcode,
            fileSystem: fileSystem,
          );

          fakeProcessManager.addCommands(<FakeCommand>[
            FakeCommand(
              command: <String>[
                'xcrun',
                'osascript',
                '-l',
                'JavaScript',
                pathToXcodeAutomationScript,
                'stop',
                '--xcode-path',
                pathToXcodeApp,
                '--project-path',
                project.xcodeProject.path,
                '--workspace-path',
                project.xcodeWorkspace.path,
                '--close-window',
              ],
              stdout: '''
  {"status":true,"errorMessage":null,"debugResult":null}
  ''',
            ),
          ]);

          xcodeDebug.startDebugActionProcess = FakeProcess();
          xcodeDebug.currentDebuggingProject = project;

          expect(xcodeDebug.startDebugActionProcess, isNotNull);
          expect(xcodeDebug.currentDebuggingProject, isNotNull);
          expect(projectDirectory.existsSync(), isFalse);
          expect(xcodeproj.existsSync(), isFalse);
          expect(xcworkspace.existsSync(), isFalse);

          final bool status = await xcodeDebug.exit(skipDelay: true);

          expect((xcodeDebug.startDebugActionProcess! as FakeProcess).killed, isTrue);
          expect(xcodeDebug.currentDebuggingProject, isNull);
          expect(projectDirectory.existsSync(), isFalse);
          expect(xcodeproj.existsSync(), isFalse);
          expect(xcworkspace.existsSync(), isFalse);
          expect(logger.errorText, contains('Failed to delete temporary Xcode project'));
          expect(fakeProcessManager, hasNoRemainingExpectations);
          expect(status, isTrue);
        },
      );

      testWithoutContext('kill Xcode when force exit', () async {
        final Xcode xcode = setupXcode(
          fakeProcessManager: FakeProcessManager.any(),
          fileSystem: fileSystem,
          flutterRoot: flutterRoot,
        );
        final XcodeDebugProject project = XcodeDebugProject(
          scheme: 'Runner',
          xcodeProject: xcodeproj,
          xcodeWorkspace: xcworkspace,
          hostAppProjectName: 'Runner',
        );
        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );
        fakeProcessManager.addCommands(<FakeCommand>[
          const FakeCommand(command: <String>['killall', '-9', 'Xcode']),
        ]);

        xcodeDebug.startDebugActionProcess = FakeProcess();
        xcodeDebug.currentDebuggingProject = project;

        expect(xcodeDebug.startDebugActionProcess, isNotNull);
        expect(xcodeDebug.currentDebuggingProject, isNotNull);

        final bool exitStatus = await xcodeDebug.exit(force: true);

        expect((xcodeDebug.startDebugActionProcess! as FakeProcess).killed, isTrue);
        expect(xcodeDebug.currentDebuggingProject, isNull);
        expect(logger.errorText, isEmpty);
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(exitStatus, isTrue);
      });

      testWithoutContext(
        'does not crash when deleting temporary directory that is nonexistent when force exiting',
        () async {
          final Xcode xcode = setupXcode(
            fakeProcessManager: FakeProcessManager.any(),
            fileSystem: fileSystem,
            flutterRoot: flutterRoot,
          );
          final XcodeDebugProject project = XcodeDebugProject(
            scheme: 'Runner',
            xcodeProject: xcodeproj,
            xcodeWorkspace: xcworkspace,
            hostAppProjectName: 'Runner',
            isTemporaryProject: true,
          );
          final XcodeDebug xcodeDebug = XcodeDebug(
            logger: logger,
            processManager: FakeProcessManager.any(),
            xcode: xcode,
            fileSystem: fileSystem,
          );

          xcodeDebug.startDebugActionProcess = FakeProcess();
          xcodeDebug.currentDebuggingProject = project;

          expect(xcodeDebug.startDebugActionProcess, isNotNull);
          expect(xcodeDebug.currentDebuggingProject, isNotNull);
          expect(projectDirectory.existsSync(), isFalse);
          expect(xcodeproj.existsSync(), isFalse);
          expect(xcworkspace.existsSync(), isFalse);

          final bool status = await xcodeDebug.exit(force: true);

          expect((xcodeDebug.startDebugActionProcess! as FakeProcess).killed, isTrue);
          expect(xcodeDebug.currentDebuggingProject, isNull);
          expect(projectDirectory.existsSync(), isFalse);
          expect(xcodeproj.existsSync(), isFalse);
          expect(xcworkspace.existsSync(), isFalse);
          expect(logger.errorText, isEmpty);
          expect(fakeProcessManager, hasNoRemainingExpectations);
          expect(status, isTrue);
        },
      );
    });

    group('stop app', () {
      const String pathToXcodeApp = '/Applications/Xcode.app';

      late Xcode xcode;
      late Directory xcodeproj;
      late Directory xcworkspace;
      late XcodeDebugProject project;

      setUp(() {
        xcode = setupXcode(
          fakeProcessManager: fakeProcessManager,
          fileSystem: fileSystem,
          flutterRoot: flutterRoot,
        );
        xcodeproj = fileSystem.directory('Runner.xcodeproj');
        xcworkspace = fileSystem.directory('Runner.xcworkspace');
        project = XcodeDebugProject(
          scheme: 'Runner',
          xcodeProject: xcodeproj,
          xcodeWorkspace: xcworkspace,
          hostAppProjectName: 'Runner',
        );
      });

      testWithoutContext('succeeds with all optional flags', () async {
        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );
        fakeProcessManager.addCommands(<FakeCommand>[
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'stop',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
              '--close-window',
              '--prompt-to-save',
            ],
            stdout: '''
  {"status":true,"errorMessage":null,"debugResult":null}
  ''',
          ),
        ]);

        final bool status = await xcodeDebug.stopDebuggingApp(
          project: project,
          closeXcode: true,
          promptToSaveOnClose: true,
        );

        expect(logger.errorText, isEmpty);
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(status, isTrue);
      });

      testWithoutContext('fails if osascript output returns false status', () async {
        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );
        fakeProcessManager.addCommands(<FakeCommand>[
          FakeCommand(
            command: <String>[
              'xcrun',
              'osascript',
              '-l',
              'JavaScript',
              pathToXcodeAutomationScript,
              'stop',
              '--xcode-path',
              pathToXcodeApp,
              '--project-path',
              project.xcodeProject.path,
              '--workspace-path',
              project.xcodeWorkspace.path,
              '--close-window',
              '--prompt-to-save',
            ],
            stdout: '''
  {"status":false,"errorMessage":"Failed to stop app","debugResult":null}
  ''',
          ),
        ]);

        final bool status = await xcodeDebug.stopDebuggingApp(
          project: project,
          closeXcode: true,
          promptToSaveOnClose: true,
        );

        expect(logger.errorText, contains('Error stopping app in Xcode'));
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(status, isFalse);
      });
    });

    group('ensureXcodeDebuggerLaunchAction', () {
      late Xcode xcode;

      setUp(() {
        xcode = setupXcode(
          fakeProcessManager: fakeProcessManager,
          fileSystem: fileSystem,
          flutterRoot: flutterRoot,
        );
      });

      testWithoutContext('succeeds', () async {
        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final File schemeFile = fileSystem.file(
          'ios/Runner.xcodeproj/xcshareddata/xcschemes/Runner.xcscheme',
        );
        schemeFile.createSync(recursive: true);
        schemeFile.writeAsStringSync(validSchemeXml);

        xcodeDebug.ensureXcodeDebuggerLaunchAction(schemeFile);
        expect(logger.errorText, isEmpty);
      });

      testWithoutContext('prints error if scheme file not found', () async {
        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final File schemeFile = fileSystem.file(
          'ios/Runner.xcodeproj/xcshareddata/xcschemes/Runner.xcscheme',
        );

        xcodeDebug.ensureXcodeDebuggerLaunchAction(schemeFile);
        expect(logger.errorText.contains('Failed to find'), isTrue);
      });

      testWithoutContext('throws error if launch action is missing debugger info', () async {
        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final File schemeFile = fileSystem.file(
          'ios/Runner.xcodeproj/xcshareddata/xcschemes/Runner.xcscheme',
        );
        schemeFile.createSync(recursive: true);
        schemeFile.writeAsStringSync(disabledDebugExecutableSchemeXml);

        expect(
          () => xcodeDebug.ensureXcodeDebuggerLaunchAction(schemeFile),
          throwsToolExit(message: 'Your Xcode project is not setup to start a debugger.'),
        );
      });

      testWithoutContext('prints error if unable to find launch action', () async {
        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final File schemeFile = fileSystem.file(
          'ios/Runner.xcodeproj/xcshareddata/xcschemes/Runner.xcscheme',
        );
        schemeFile.createSync(recursive: true);
        schemeFile.writeAsStringSync('<?xml version="1.0" encoding="UTF-8"?><Scheme></Scheme>');

        xcodeDebug.ensureXcodeDebuggerLaunchAction(schemeFile);
        expect(logger.errorText.contains('Failed to find LaunchAction for the Scheme'), isTrue);
      });

      testWithoutContext('prints error if invalid xml', () async {
        final XcodeDebug xcodeDebug = XcodeDebug(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fileSystem,
        );

        final File schemeFile = fileSystem.file(
          'ios/Runner.xcodeproj/xcshareddata/xcschemes/Runner.xcscheme',
        );
        schemeFile.createSync(recursive: true);
        schemeFile.writeAsStringSync('<?xml version="1.0" encoding="UTF-8"?><Scheme>');

        xcodeDebug.ensureXcodeDebuggerLaunchAction(schemeFile);
        expect(logger.errorText.contains('Failed to parse'), isTrue);
      });
    });
  });

  group('Debug project through Xcode with app bundle', () {
    late BufferLogger logger;
    late FakeProcessManager fakeProcessManager;
    late MemoryFileSystem fileSystem;

    const String flutterRoot = '/path/to/flutter';

    setUp(() {
      logger = BufferLogger.test();
      fakeProcessManager = FakeProcessManager.empty();
      fileSystem = MemoryFileSystem.test();
    });

    testUsingContext('creates temporary xcode project', () async {
      final Xcode xcode = setupXcode(
        fakeProcessManager: fakeProcessManager,
        fileSystem: fileSystem,
        flutterRoot: flutterRoot,
      );

      final XcodeDebug xcodeDebug = XcodeDebug(
        logger: logger,
        processManager: fakeProcessManager,
        xcode: xcode,
        fileSystem: globals.fs,
      );

      final Directory projectDirectory = globals.fs.systemTempDirectory.createTempSync(
        'flutter_empty_xcode.',
      );

      try {
        final XcodeDebugProject project = await xcodeDebug.createXcodeProjectWithCustomBundle(
          '/path/to/bundle',
          templateRenderer: globals.templateRenderer,
          projectDestination: projectDirectory,
        );

        final File schemeFile = projectDirectory
            .childDirectory('Runner.xcodeproj')
            .childDirectory('xcshareddata')
            .childDirectory('xcschemes')
            .childFile('Runner.xcscheme');

        expect(project.scheme, 'Runner');
        expect(project.xcodeProject.existsSync(), isTrue);
        expect(project.xcodeWorkspace.existsSync(), isTrue);
        expect(project.isTemporaryProject, isTrue);
        expect(projectDirectory.childDirectory('Runner.xcodeproj').existsSync(), isTrue);
        expect(projectDirectory.childDirectory('Runner.xcworkspace').existsSync(), isTrue);
        expect(schemeFile.existsSync(), isTrue);
        expect(schemeFile.readAsStringSync(), contains('FilePath = "/path/to/bundle"'));
      } catch (err) {
        // ignore: avoid_catches_without_on_clauses
        fail(err.toString());
      } finally {
        projectDirectory.deleteSync(recursive: true);
      }
    });
  });
}

Xcode setupXcode({
  required FakeProcessManager fakeProcessManager,
  required FileSystem fileSystem,
  required String flutterRoot,
  bool xcodeSelect = true,
}) {
  fakeProcessManager.addCommand(
    const FakeCommand(
      command: <String>['/usr/bin/xcode-select', '--print-path'],
      stdout: '/Applications/Xcode.app/Contents/Developer',
    ),
  );

  fileSystem
      .file('$flutterRoot/packages/flutter_tools/bin/xcode_debug.js')
      .createSync(recursive: true);

  final XcodeProjectInterpreter xcodeProjectInterpreter = XcodeProjectInterpreter.test(
    processManager: FakeProcessManager.any(),
    version: Version(14, 0, 0),
  );

  return Xcode.test(
    processManager: fakeProcessManager,
    xcodeProjectInterpreter: xcodeProjectInterpreter,
    fileSystem: fileSystem,
    flutterRoot: flutterRoot,
  );
}

class FakeProcess extends Fake implements Process {
  bool killed = false;

  @override
  bool kill([io.ProcessSignal signal = io.ProcessSignal.sigterm]) {
    killed = true;
    return true;
  }
}

const String validSchemeXml = '''
<?xml version="1.0" encoding="UTF-8"?>
<Scheme
   LastUpgradeVersion = "1510"
   version = "1.3">
   <BuildAction
      parallelizeBuildables = "YES"
      buildImplicitDependencies = "YES">
   </BuildAction>
   <TestAction
      buildConfiguration = "Debug"
      selectedDebuggerIdentifier = "Xcode.DebuggerFoundation.Debugger.LLDB"
      selectedLauncherIdentifier = "Xcode.DebuggerFoundation.Launcher.LLDB"
      shouldUseLaunchSchemeArgsEnv = "YES">
   </TestAction>
   <LaunchAction
      buildConfiguration = "Debug"
      selectedDebuggerIdentifier = "Xcode.DebuggerFoundation.Debugger.LLDB"
      selectedLauncherIdentifier = "Xcode.DebuggerFoundation.Launcher.LLDB"
      launchStyle = "0"
      useCustomWorkingDirectory = "NO"
      ignoresPersistentStateOnLaunch = "NO"
      debugDocumentVersioning = "YES"
      debugServiceExtension = "internal"
      allowLocationSimulation = "YES">
   </LaunchAction>
   <ProfileAction
      buildConfiguration = "Profile"
      shouldUseLaunchSchemeArgsEnv = "YES"
      savedToolIdentifier = ""
      useCustomWorkingDirectory = "NO"
      debugDocumentVersioning = "YES">
   </ProfileAction>
   <AnalyzeAction
      buildConfiguration = "Debug">
   </AnalyzeAction>
   <ArchiveAction
      buildConfiguration = "Release"
      revealArchiveInOrganizer = "YES">
   </ArchiveAction>
</Scheme>
''';

const String disabledDebugExecutableSchemeXml = '''
<?xml version="1.0" encoding="UTF-8"?>
<Scheme
   LastUpgradeVersion = "1510"
   version = "1.3">
   <BuildAction
      parallelizeBuildables = "YES"
      buildImplicitDependencies = "YES">
   </BuildAction>
   <TestAction
      buildConfiguration = "Debug"
      selectedDebuggerIdentifier = "Xcode.DebuggerFoundation.Debugger.LLDB"
      selectedLauncherIdentifier = "Xcode.DebuggerFoundation.Launcher.LLDB"
      shouldUseLaunchSchemeArgsEnv = "YES">
   </TestAction>
   <LaunchAction
      buildConfiguration = "Debug"
      selectedDebuggerIdentifier = ""
      selectedLauncherIdentifier = "Xcode.IDEFoundation.Launcher.PosixSpawn"
      launchStyle = "0"
      useCustomWorkingDirectory = "NO"
      ignoresPersistentStateOnLaunch = "NO"
      debugDocumentVersioning = "YES"
      debugServiceExtension = "internal"
      allowLocationSimulation = "YES">
   </LaunchAction>
   <ProfileAction
      buildConfiguration = "Profile"
      shouldUseLaunchSchemeArgsEnv = "YES"
      savedToolIdentifier = ""
      useCustomWorkingDirectory = "NO"
      debugDocumentVersioning = "YES">
   </ProfileAction>
   <AnalyzeAction
      buildConfiguration = "Debug">
   </AnalyzeAction>
   <ArchiveAction
      buildConfiguration = "Release"
      revealArchiveInOrganizer = "YES">
   </ArchiveAction>
</Scheme>
''';
