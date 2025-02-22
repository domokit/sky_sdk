// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/io.dart' show ProcessException;
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/ios/core_devices.dart';
import 'package:flutter_tools/src/ios/devices.dart';
import 'package:flutter_tools/src/ios/iproxy.dart';
import 'package:flutter_tools/src/ios/xcode_debug.dart';
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/macos/xcdevice.dart';
import 'package:flutter_tools/src/macos/xcode.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

void main() {
  late BufferLogger logger;

  setUp(() {
    logger = BufferLogger.test();
  });

  group('FakeProcessManager', () {
    late FakeProcessManager fakeProcessManager;

    setUp(() {
      fakeProcessManager = FakeProcessManager.empty();
    });

    group('Xcode', () {
      late FakeXcodeProjectInterpreter xcodeProjectInterpreter;

      setUp(() {
        xcodeProjectInterpreter = FakeXcodeProjectInterpreter();
      });

      testWithoutContext('isInstalledAndMeetsVersionCheck is false when not macOS', () {
        final Xcode xcode = Xcode.test(
          platform: FakePlatform(operatingSystem: 'windows'),
          processManager: fakeProcessManager,
          xcodeProjectInterpreter: xcodeProjectInterpreter,
        );

        expect(xcode.isInstalledAndMeetsVersionCheck, isFalse);
      });

      testWithoutContext('isSimctlInstalled is true when simctl list succeeds', () {
        fakeProcessManager.addCommand(
          const FakeCommand(command: <String>['xcrun', 'simctl', 'list', 'devices', 'booted']),
        );
        final Xcode xcode = Xcode.test(
          processManager: fakeProcessManager,
          xcodeProjectInterpreter: xcodeProjectInterpreter,
        );

        expect(xcode.isSimctlInstalled, isTrue);
        expect(fakeProcessManager, hasNoRemainingExpectations);
      });

      testWithoutContext('isSimctlInstalled is false when simctl list fails', () {
        fakeProcessManager.addCommand(
          const FakeCommand(
            command: <String>['xcrun', 'simctl', 'list', 'devices', 'booted'],
            stderr: 'failed to run',
            exitCode: 1,
          ),
        );
        final Xcode xcode = Xcode.test(
          processManager: fakeProcessManager,
          xcodeProjectInterpreter: xcodeProjectInterpreter,
          logger: logger,
        );

        expect(xcode.isSimctlInstalled, isFalse);
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.statusText, isEmpty);
        expect(logger.errorText, isEmpty);
      });

      testWithoutContext(
        'isSimctlInstalled is false when simctl list throws process exception',
        () {
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['xcrun', 'simctl', 'list', 'devices', 'booted'],
              exception: ProcessException('xcrun', <String>['simctl']),
            ),
          );
          final Xcode xcode = Xcode.test(
            processManager: fakeProcessManager,
            xcodeProjectInterpreter: xcodeProjectInterpreter,
            logger: logger,
          );

          expect(xcode.isSimctlInstalled, isFalse);
          expect(fakeProcessManager, hasNoRemainingExpectations);
          expect(logger.statusText, isEmpty);
          expect(logger.errorText, isEmpty);
          expect(logger.traceText, contains('ProcessException'));
        },
      );

      group('isDevicectlInstalled', () {
        testWithoutContext('is true when Xcode is 15+ and devicectl succeeds', () {
          fakeProcessManager.addCommand(
            const FakeCommand(command: <String>['xcrun', 'devicectl', '--version']),
          );
          xcodeProjectInterpreter.version = Version(15, 0, 0);
          final Xcode xcode = Xcode.test(
            processManager: fakeProcessManager,
            xcodeProjectInterpreter: xcodeProjectInterpreter,
          );

          expect(xcode.isDevicectlInstalled, isTrue);
          expect(fakeProcessManager, hasNoRemainingExpectations);
        });

        testWithoutContext('is false when devicectl fails', () {
          fakeProcessManager.addCommand(
            const FakeCommand(command: <String>['xcrun', 'devicectl', '--version'], exitCode: 1),
          );
          xcodeProjectInterpreter.version = Version(15, 0, 0);
          final Xcode xcode = Xcode.test(
            processManager: fakeProcessManager,
            xcodeProjectInterpreter: xcodeProjectInterpreter,
            logger: logger,
          );

          expect(xcode.isDevicectlInstalled, isFalse);
          expect(fakeProcessManager, hasNoRemainingExpectations);
          expect(logger.statusText, isEmpty);
          expect(logger.errorText, isEmpty);
        });

        testWithoutContext('is false when devicectl throws process exception', () {
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['xcrun', 'devicectl', '--version'],
              exitCode: 1,
              exception: ProcessException('xcrun', <String>['devicectl']),
            ),
          );
          xcodeProjectInterpreter.version = Version(15, 0, 0);
          final Xcode xcode = Xcode.test(
            processManager: fakeProcessManager,
            xcodeProjectInterpreter: xcodeProjectInterpreter,
            logger: logger,
          );

          expect(xcode.isDevicectlInstalled, isFalse);
          expect(fakeProcessManager, hasNoRemainingExpectations);
          expect(logger.statusText, isEmpty);
          expect(logger.errorText, isEmpty);
          expect(logger.traceText, contains('ProcessException'));
        });

        testWithoutContext('is false when Xcode is less than 15', () {
          xcodeProjectInterpreter.version = Version(14, 0, 0);
          final Xcode xcode = Xcode.test(
            processManager: fakeProcessManager,
            xcodeProjectInterpreter: xcodeProjectInterpreter,
          );

          expect(xcode.isDevicectlInstalled, isFalse);
          expect(fakeProcessManager, hasNoRemainingExpectations);
        });
      });

      group('pathToXcodeApp', () {
        late UserMessages userMessages;

        setUp(() {
          userMessages = UserMessages();
        });

        testWithoutContext('parses correctly', () {
          final Xcode xcode = Xcode.test(
            processManager: fakeProcessManager,
            xcodeProjectInterpreter: xcodeProjectInterpreter,
          );

          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['/usr/bin/xcode-select', '--print-path'],
              stdout: '/Applications/Xcode.app/Contents/Developer',
            ),
          );

          expect(xcode.xcodeAppPath, '/Applications/Xcode.app');
          expect(fakeProcessManager, hasNoRemainingExpectations);
        });

        testWithoutContext('throws error if not found', () {
          final Xcode xcode = Xcode.test(
            processManager: FakeProcessManager.any(),
            xcodeProjectInterpreter: xcodeProjectInterpreter,
          );

          expect(() => xcode.xcodeAppPath, throwsToolExit(message: userMessages.xcodeMissing));
        });

        testWithoutContext('throws error with unexpected outcome', () {
          final Xcode xcode = Xcode.test(
            processManager: fakeProcessManager,
            xcodeProjectInterpreter: xcodeProjectInterpreter,
          );

          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['/usr/bin/xcode-select', '--print-path'],
              stdout: '/Library/Developer/CommandLineTools',
            ),
          );

          expect(() => xcode.xcodeAppPath, throwsToolExit(message: userMessages.xcodeMissing));
          expect(fakeProcessManager, hasNoRemainingExpectations);
        });
      });

      group('pathToXcodeAutomationScript', () {
        const String flutterRoot = '/path/to/flutter';

        late MemoryFileSystem fileSystem;

        setUp(() {
          fileSystem = MemoryFileSystem.test();
        });

        testWithoutContext('returns path when file is found', () {
          final Xcode xcode = Xcode.test(
            processManager: fakeProcessManager,
            xcodeProjectInterpreter: xcodeProjectInterpreter,
            fileSystem: fileSystem,
            flutterRoot: flutterRoot,
          );

          fileSystem
              .file('$flutterRoot/packages/flutter_tools/bin/xcode_debug.js')
              .createSync(recursive: true);

          expect(
            xcode.xcodeAutomationScriptPath,
            '$flutterRoot/packages/flutter_tools/bin/xcode_debug.js',
          );
        });

        testWithoutContext('throws error when not found', () {
          final Xcode xcode = Xcode.test(
            processManager: fakeProcessManager,
            xcodeProjectInterpreter: xcodeProjectInterpreter,
            fileSystem: fileSystem,
            flutterRoot: flutterRoot,
          );

          expect(() => xcode.xcodeAutomationScriptPath, throwsToolExit());
        });
      });

      group('macOS', () {
        late Xcode xcode;
        late BufferLogger logger;

        setUp(() {
          xcodeProjectInterpreter = FakeXcodeProjectInterpreter();
          logger = BufferLogger.test();
          xcode = Xcode.test(
            processManager: fakeProcessManager,
            xcodeProjectInterpreter: xcodeProjectInterpreter,
            logger: logger,
          );
        });

        testWithoutContext('xcodeSelectPath returns path when xcode-select is installed', () {
          const String xcodePath = '/Applications/Xcode8.0.app/Contents/Developer';
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['/usr/bin/xcode-select', '--print-path'],
              stdout: xcodePath,
            ),
          );

          expect(xcode.xcodeSelectPath, xcodePath);
          expect(fakeProcessManager, hasNoRemainingExpectations);
        });

        testWithoutContext('xcodeSelectPath returns null when xcode-select is not installed', () {
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['/usr/bin/xcode-select', '--print-path'],
              exception: ProcessException('/usr/bin/xcode-select', <String>['--print-path']),
            ),
          );

          expect(xcode.xcodeSelectPath, isNull);
          expect(fakeProcessManager, hasNoRemainingExpectations);

          fakeProcessManager.addCommand(
            FakeCommand(
              command: const <String>['/usr/bin/xcode-select', '--print-path'],
              exception: ArgumentError(
                'Invalid argument(s): Cannot find executable for /usr/bin/xcode-select',
              ),
            ),
          );

          expect(xcode.xcodeSelectPath, isNull);
          expect(fakeProcessManager, hasNoRemainingExpectations);
        });

        testWithoutContext('version checks fail when version is less than minimum', () {
          xcodeProjectInterpreter.isInstalled = true;
          xcodeProjectInterpreter.version = Version(9, null, null);

          expect(xcode.isRequiredVersionSatisfactory, isFalse);
          expect(xcode.isRecommendedVersionSatisfactory, isFalse);
        });

        testWithoutContext('version checks fail when xcodebuild tools are not installed', () {
          xcodeProjectInterpreter.isInstalled = false;

          expect(xcode.isRequiredVersionSatisfactory, isFalse);
          expect(xcode.isRecommendedVersionSatisfactory, isFalse);
        });

        testWithoutContext(
          'version checks pass when version meets minimum but not recommended',
          () {
            xcodeProjectInterpreter.isInstalled = true;
            xcodeProjectInterpreter.version = Version(14, null, null);

            expect(xcode.isRequiredVersionSatisfactory, isTrue);
            expect(xcode.isRecommendedVersionSatisfactory, isFalse);
          },
        );

        testWithoutContext('version checks pass when major version exceeds minimum', () {
          xcodeProjectInterpreter.isInstalled = true;
          xcodeProjectInterpreter.version = Version(15, 0, 0);

          expect(xcode.isRequiredVersionSatisfactory, isTrue);
        });

        testWithoutContext('version checks pass when minor version exceeds minimum', () {
          xcodeProjectInterpreter.isInstalled = true;
          xcodeProjectInterpreter.version = Version(14, 3, 0);

          expect(xcode.isRequiredVersionSatisfactory, isTrue);
        });

        testWithoutContext('version checks pass when patch version exceeds minimum', () {
          xcodeProjectInterpreter.isInstalled = true;
          xcodeProjectInterpreter.version = Version(14, 0, 2);

          expect(xcode.isRequiredVersionSatisfactory, isTrue);
        });

        testWithoutContext('version checks pass when major version exceeds recommendation', () {
          xcodeProjectInterpreter.isInstalled = true;
          xcodeProjectInterpreter.version = Version(16, 0, 0);

          expect(xcode.isRequiredVersionSatisfactory, isTrue);
          expect(xcode.isRecommendedVersionSatisfactory, isTrue);
        });

        testWithoutContext('version checks pass when minor version exceeds recommendation', () {
          xcodeProjectInterpreter.isInstalled = true;
          xcodeProjectInterpreter.version = Version(15, 3, 0);

          expect(xcode.isRequiredVersionSatisfactory, isTrue);
          expect(xcode.isRecommendedVersionSatisfactory, isTrue);
        });

        testWithoutContext('version checks pass when patch version exceeds recommendation', () {
          xcodeProjectInterpreter.isInstalled = true;
          xcodeProjectInterpreter.version = Version(15, 0, 2);

          expect(xcode.isRequiredVersionSatisfactory, isTrue);
          expect(xcode.isRecommendedVersionSatisfactory, isTrue);
        });

        testWithoutContext('isInstalledAndMeetsVersionCheck is false when not installed', () {
          xcodeProjectInterpreter.isInstalled = false;

          expect(xcode.isInstalledAndMeetsVersionCheck, isFalse);
          expect(fakeProcessManager, hasNoRemainingExpectations);
        });

        testWithoutContext(
          'isInstalledAndMeetsVersionCheck is false when version not satisfied',
          () {
            xcodeProjectInterpreter.isInstalled = true;
            xcodeProjectInterpreter.version = Version(10, 2, 0);

            expect(xcode.isInstalledAndMeetsVersionCheck, isFalse);
            expect(fakeProcessManager, hasNoRemainingExpectations);
          },
        );

        testWithoutContext(
          'isInstalledAndMeetsVersionCheck is true when macOS and installed and version is satisfied',
          () {
            xcodeProjectInterpreter.isInstalled = true;
            xcodeProjectInterpreter.version = Version(14, null, null);

            expect(xcode.isInstalledAndMeetsVersionCheck, isTrue);
            expect(fakeProcessManager, hasNoRemainingExpectations);
          },
        );

        testWithoutContext(
          'eulaSigned is false when clang output indicates EULA not yet accepted',
          () {
            fakeProcessManager.addCommands(const <FakeCommand>[
              FakeCommand(
                command: <String>['xcrun', 'clang'],
                exitCode: 1,
                stderr: 'Xcode EULA has not been accepted.\nLaunch Xcode and accept the license.',
              ),
            ]);

            expect(xcode.eulaSigned, isFalse);
            expect(fakeProcessManager, hasNoRemainingExpectations);
          },
        );

        testWithoutContext('eulaSigned is false when clang is not installed', () {
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['xcrun', 'clang'],
              exception: ProcessException('xcrun', <String>['clang']),
            ),
          );

          expect(xcode.eulaSigned, isFalse);
        });

        testWithoutContext(
          'eulaSigned is true when clang output indicates EULA has been accepted',
          () {
            fakeProcessManager.addCommands(const <FakeCommand>[
              FakeCommand(
                command: <String>['xcrun', 'clang'],
                exitCode: 1,
                stderr: 'clang: error: no input files',
              ),
            ]);
            expect(xcode.eulaSigned, isTrue);
            expect(fakeProcessManager, hasNoRemainingExpectations);
          },
        );

        testWithoutContext('SDK name', () {
          expect(getSDKNameForIOSEnvironmentType(EnvironmentType.physical), 'iphoneos');
          expect(getSDKNameForIOSEnvironmentType(EnvironmentType.simulator), 'iphonesimulator');
        });

        group('SDK location', () {
          const String sdkroot =
              'Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Developer/SDKs/iPhoneOS13.2.sdk';

          testWithoutContext('--show-sdk-path iphoneos', () async {
            fakeProcessManager.addCommand(
              const FakeCommand(
                command: <String>['xcrun', '--sdk', 'iphoneos', '--show-sdk-path'],
                stdout: sdkroot,
              ),
            );

            expect(await xcode.sdkLocation(EnvironmentType.physical), sdkroot);
            expect(fakeProcessManager, hasNoRemainingExpectations);
          });

          testWithoutContext('--show-sdk-path fails', () async {
            fakeProcessManager.addCommand(
              const FakeCommand(
                command: <String>['xcrun', '--sdk', 'iphoneos', '--show-sdk-path'],
                exitCode: 1,
                stderr: 'xcrun: error:',
              ),
            );

            expect(
              () async => xcode.sdkLocation(EnvironmentType.physical),
              throwsToolExit(message: 'Could not find SDK location'),
            );
            expect(fakeProcessManager, hasNoRemainingExpectations);
          });
        });

        group('SDK Platform Version', () {
          testWithoutContext('--show-sdk-platform-version iphonesimulator', () async {
            fakeProcessManager.addCommand(
              const FakeCommand(
                command: <String>[
                  'xcrun',
                  '--sdk',
                  'iphonesimulator',
                  '--show-sdk-platform-version',
                ],
                stdout: '16.4',
              ),
            );

            expect(await xcode.sdkPlatformVersion(EnvironmentType.simulator), Version(16, 4, null));
            expect(fakeProcessManager, hasNoRemainingExpectations);
          });

          testWithoutContext(
            '--show-sdk-platform-version iphonesimulator with leading and trailing new line',
            () async {
              fakeProcessManager.addCommand(
                const FakeCommand(
                  command: <String>[
                    'xcrun',
                    '--sdk',
                    'iphonesimulator',
                    '--show-sdk-platform-version',
                  ],
                  stdout: '\n16.4\n',
                ),
              );

              expect(
                await xcode.sdkPlatformVersion(EnvironmentType.simulator),
                Version(16, 4, null),
              );
              expect(fakeProcessManager, hasNoRemainingExpectations);
            },
          );

          testWithoutContext(
            '--show-sdk-platform-version returns version followed by text',
            () async {
              fakeProcessManager.addCommand(
                const FakeCommand(
                  command: <String>[
                    'xcrun',
                    '--sdk',
                    'iphonesimulator',
                    '--show-sdk-platform-version',
                  ],
                  stdout: '13.2 (a) 12344',
                ),
              );

              expect(
                await xcode.sdkPlatformVersion(EnvironmentType.simulator),
                Version(13, 2, null, text: '13.2 (a) 12344'),
              );
              expect(fakeProcessManager, hasNoRemainingExpectations);
            },
          );

          testWithoutContext('--show-sdk-platform-version returns something unexpected', () async {
            fakeProcessManager.addCommand(
              const FakeCommand(
                command: <String>[
                  'xcrun',
                  '--sdk',
                  'iphonesimulator',
                  '--show-sdk-platform-version',
                ],
                stdout: 'bogus',
              ),
            );

            expect(await xcode.sdkPlatformVersion(EnvironmentType.simulator), null);
            expect(fakeProcessManager, hasNoRemainingExpectations);
          });

          testWithoutContext('--show-sdk-platform-version fails', () async {
            fakeProcessManager.addCommand(
              const FakeCommand(
                command: <String>[
                  'xcrun',
                  '--sdk',
                  'iphonesimulator',
                  '--show-sdk-platform-version',
                ],
                exitCode: 1,
                stderr: 'xcrun: error:',
              ),
            );
            expect(await xcode.sdkPlatformVersion(EnvironmentType.simulator), null);
            expect(fakeProcessManager, hasNoRemainingExpectations);
            expect(logger.errorText, contains('Could not find SDK Platform Version'));
          });
        });
      });
    });

    group('xcdevice not installed', () {
      late XCDevice xcdevice;
      late Xcode xcode;
      late MemoryFileSystem fileSystem;

      setUp(() {
        xcode = Xcode.test(
          processManager: FakeProcessManager.any(),
          xcodeProjectInterpreter: XcodeProjectInterpreter.test(
            processManager: FakeProcessManager.any(),
            version: null, // Not installed.
          ),
        );
        fileSystem = MemoryFileSystem.test();
        xcdevice = XCDevice(
          processManager: fakeProcessManager,
          logger: logger,
          xcode: xcode,
          platform: FakePlatform(operatingSystem: 'macos'),
          artifacts: Artifacts.test(),
          cache: Cache.test(processManager: FakeProcessManager.any()),
          iproxy: IProxy.test(logger: logger, processManager: fakeProcessManager),
          fileSystem: fileSystem,
          coreDeviceControl: FakeIOSCoreDeviceControl(),
          xcodeDebug: FakeXcodeDebug(),
          analytics: const NoOpAnalytics(),
          shutdownHooks: FakeShutdownHooks(),
        );
      });

      testWithoutContext('Xcode not installed', () async {
        expect(xcode.isInstalled, false);

        expect(xcdevice.isInstalled, false);
        expect(xcdevice.observedDeviceEvents(), isNull);
        expect(
          logger.traceText,
          contains("Xcode not found. Run 'flutter doctor' for more information."),
        );
        expect(await xcdevice.getAvailableIOSDevices(), isEmpty);
        expect(await xcdevice.getDiagnostics(), isEmpty);
      });
    });

    testWithoutContext('shutdown hooks disposes xcdevice observers', () async {
      final ShutdownHooks shutdownHooks = ShutdownHooks();

      final XCDevice xcdevice = XCDevice(
        processManager: FakeProcessManager.any(),
        logger: logger,
        xcode: Xcode.test(processManager: FakeProcessManager.any()),
        platform: FakePlatform(operatingSystem: 'macos'),
        artifacts: Artifacts.test(),
        cache: Cache.test(processManager: FakeProcessManager.any()),
        iproxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
        fileSystem: MemoryFileSystem.test(),
        coreDeviceControl: FakeIOSCoreDeviceControl(),
        xcodeDebug: FakeXcodeDebug(),
        analytics: const NoOpAnalytics(),
        shutdownHooks: shutdownHooks,
      );

      expect(shutdownHooks.registeredHooks, hasLength(1));
      final Completer<void> doneCompleter = Completer<void>();
      xcdevice.observedDeviceEvents()!.listen(
        null,
        onDone: () {
          doneCompleter.complete();
        },
      );
      await doneCompleter.future;
      xcdevice.dispose();
      expect(logger.traceText, contains('xcdevice observe --usb exited with code 0'));
      expect(logger.traceText, contains('xcdevice observe --wifi exited with code 0'));
    });

    group('xcdevice', () {
      late XCDevice xcdevice;
      late Xcode xcode;
      late MemoryFileSystem fileSystem;
      late FakeAnalytics fakeAnalytics;
      late FakeIOSCoreDeviceControl coreDeviceControl;

      setUp(() {
        xcode = Xcode.test(processManager: FakeProcessManager.any());
        fileSystem = MemoryFileSystem.test();
        coreDeviceControl = FakeIOSCoreDeviceControl();
        fakeAnalytics = getInitializedFakeAnalyticsInstance(
          fs: fileSystem,
          fakeFlutterVersion: FakeFlutterVersion(),
        );
        xcdevice = XCDevice(
          processManager: fakeProcessManager,
          logger: logger,
          xcode: xcode,
          platform: FakePlatform(operatingSystem: 'macos'),
          artifacts: Artifacts.test(),
          cache: Cache.test(processManager: FakeProcessManager.any()),
          iproxy: IProxy.test(logger: logger, processManager: fakeProcessManager),
          fileSystem: fileSystem,
          coreDeviceControl: coreDeviceControl,
          xcodeDebug: FakeXcodeDebug(),
          analytics: fakeAnalytics,
          shutdownHooks: FakeShutdownHooks(),
        );
      });

      group('observe device events', () {
        testUsingContext('relays events', () async {
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>[
                'script',
                '-t',
                '0',
                '/dev/null',
                'xcrun',
                'xcdevice',
                'observe',
                '--usb',
              ],
              stdout:
                  'Listening for all devices, on USB.\n'
                  'Attach: d83d5bc53967baa0ee18626ba87b6254b2ab5418\n'
                  'Attach: 00008027-00192736010F802E\n'
                  'Detach: d83d5bc53967baa0ee18626ba87b6254b2ab5418',
              stderr: 'Some usb error',
            ),
          );

          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>[
                'script',
                '-t',
                '0',
                '/dev/null',
                'xcrun',
                'xcdevice',
                'observe',
                '--wifi',
              ],
              stdout:
                  'Listening for all devices, on WiFi.\n'
                  'Attach: 00000001-0000000000000000\n'
                  'Detach: 00000001-0000000000000000',
              stderr: 'Some wifi error',
            ),
          );

          final Completer<void> attach1 = Completer<void>();
          final Completer<void> attach2 = Completer<void>();
          final Completer<void> detach1 = Completer<void>();
          final Completer<void> attach3 = Completer<void>();
          final Completer<void> detach2 = Completer<void>();

          // Attach: d83d5bc53967baa0ee18626ba87b6254b2ab5418
          // Attach: 00008027-00192736010F802E
          // Detach: d83d5bc53967baa0ee18626ba87b6254b2ab5418
          xcdevice.observedDeviceEvents()!.listen((XCDeviceEventNotification event) {
            if (event.eventType == XCDeviceEvent.attach) {
              if (event.deviceIdentifier == 'd83d5bc53967baa0ee18626ba87b6254b2ab5418') {
                attach1.complete();
              } else if (event.deviceIdentifier == '00008027-00192736010F802E') {
                attach2.complete();
              }
              if (event.deviceIdentifier == '00000001-0000000000000000') {
                attach3.complete();
              }
            } else if (event.eventType == XCDeviceEvent.detach) {
              if (event.deviceIdentifier == 'd83d5bc53967baa0ee18626ba87b6254b2ab5418') {
                detach1.complete();
              }
              if (event.deviceIdentifier == '00000001-0000000000000000') {
                detach2.complete();
              }
            } else {
              fail('Unexpected event');
            }
          });
          await attach1.future;
          await attach2.future;
          await detach1.future;
          await attach3.future;
          await detach2.future;
          expect(logger.errorText, contains('xcdevice observe --usb: Some usb error'));
          expect(logger.errorText, contains('xcdevice observe --wifi: Some wifi error'));
        });

        testUsingContext('handles exit code', () async {
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>[
                'script',
                '-t',
                '0',
                '/dev/null',
                'xcrun',
                'xcdevice',
                'observe',
                '--usb',
              ],
            ),
          );
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>[
                'script',
                '-t',
                '0',
                '/dev/null',
                'xcrun',
                'xcdevice',
                'observe',
                '--wifi',
              ],
              exitCode: 1,
            ),
          );

          final Completer<void> doneCompleter = Completer<void>();
          xcdevice.observedDeviceEvents()!.listen(
            null,
            onDone: () {
              doneCompleter.complete();
            },
          );
          await doneCompleter.future;
          expect(logger.traceText, contains('xcdevice observe --usb exited with code 0'));
          expect(logger.traceText, contains('xcdevice observe --wifi exited with code 0'));
        });
      });

      group('wait device events', () {
        testUsingContext('relays events', () async {
          const String deviceId = '00000001-0000000000000000';

          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>[
                'script',
                '-t',
                '0',
                '/dev/null',
                'xcrun',
                'xcdevice',
                'wait',
                '--usb',
                deviceId,
              ],
              stdout: 'Waiting for $deviceId to appear, on USB.\n',
            ),
          );
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>[
                'script',
                '-t',
                '0',
                '/dev/null',
                'xcrun',
                'xcdevice',
                'wait',
                '--wifi',
                deviceId,
              ],
              stdout:
                  'Waiting for $deviceId to appear, on WiFi.\n'
                  'Attach: 00000001-0000000000000000\n',
            ),
          );

          // Attach: 00000001-0000000000000000

          final XCDeviceEventNotification? event = await xcdevice.waitForDeviceToConnect(deviceId);

          expect(event?.deviceIdentifier, deviceId);
          expect(event?.eventInterface, XCDeviceEventInterface.wifi);
          expect(event?.eventType, XCDeviceEvent.attach);
        });

        testUsingContext('handles exit code', () async {
          const String deviceId = '00000001-0000000000000000';

          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>[
                'script',
                '-t',
                '0',
                '/dev/null',
                'xcrun',
                'xcdevice',
                'wait',
                '--usb',
                deviceId,
              ],
              exitCode: 1,
              stderr: 'Some error',
            ),
          );
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>[
                'script',
                '-t',
                '0',
                '/dev/null',
                'xcrun',
                'xcdevice',
                'wait',
                '--wifi',
                deviceId,
              ],
            ),
          );

          final XCDeviceEventNotification? event = await xcdevice.waitForDeviceToConnect(deviceId);

          expect(event, isNull);
          expect(logger.errorText, contains('xcdevice wait --usb: Some error'));
          expect(logger.traceText, contains('xcdevice wait --usb exited with code 0'));
          expect(logger.traceText, contains('xcdevice wait --wifi exited with code 0'));
          expect(xcdevice.waitStreamController?.isClosed, isTrue);
        });

        testUsingContext('handles cancel', () async {
          const String deviceId = '00000001-0000000000000000';

          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>[
                'script',
                '-t',
                '0',
                '/dev/null',
                'xcrun',
                'xcdevice',
                'wait',
                '--usb',
                deviceId,
              ],
            ),
          );
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>[
                'script',
                '-t',
                '0',
                '/dev/null',
                'xcrun',
                'xcdevice',
                'wait',
                '--wifi',
                deviceId,
              ],
            ),
          );

          final Future<XCDeviceEventNotification?> futureEvent = xcdevice.waitForDeviceToConnect(
            deviceId,
          );
          xcdevice.cancelWaitForDeviceToConnect();
          final XCDeviceEventNotification? event = await futureEvent;

          expect(event, isNull);
          expect(logger.traceText, contains('xcdevice wait --usb exited with code 0'));
          expect(logger.traceText, contains('xcdevice wait --wifi exited with code 0'));
          expect(xcdevice.waitStreamController?.isClosed, isTrue);
        });
      });

      group('available devices', () {
        final FakePlatform macPlatform = FakePlatform(operatingSystem: 'macos');
        testUsingContext(
          'returns devices',
          () async {
            const String devicesOutput = '''
[
  {
    "simulator" : true,
    "operatingSystemVersion" : "13.3 (17K446)",
    "available" : true,
    "platform" : "com.apple.platform.appletvsimulator",
    "modelCode" : "AppleTV5,3",
    "identifier" : "CBB5E1ED-2172-446E-B4E7-F2B5823DBBA6",
    "architecture" : "x86_64",
    "modelName" : "Apple TV",
    "name" : "Apple TV"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "00008027-00192736010F802E",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "An iPhone (Space Gray)"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "10.1 (14C54)",
    "interface" : "usb",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPad11,4",
    "identifier" : "98206e7a4afd4aedaff06e687594e089dede3c44",
    "architecture" : "armv7",
    "modelName" : "iPad Air 3rd Gen",
    "name" : "iPad 1"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "10.1 (14C54)",
    "interface" : "network",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPad11,4",
    "identifier" : "234234234234234234345445687594e089dede3c44",
    "architecture" : "arm64",
    "modelName" : "iPad Air 3rd Gen",
    "name" : "A networked iPad"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "10.1 (14C54)",
    "interface" : "usb",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPad11,4",
    "identifier" : "f577a7903cc54959be2e34bc4f7f80b7009efcf4",
    "architecture" : "BOGUS",
    "modelName" : "iPad Air 3rd Gen",
    "name" : "iPad 2"
  },
  {
    "simulator" : true,
    "operatingSystemVersion" : "6.1.1 (17S445)",
    "available" : true,
    "platform" : "com.apple.platform.watchsimulator",
    "modelCode" : "Watch5,4",
    "identifier" : "2D74FB11-88A0-44D0-B81E-C0C142B1C94A",
    "architecture" : "i386",
    "modelName" : "Apple Watch Series 5 - 44mm",
    "name" : "Apple Watch Series 5 - 44mm"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "c4ca6f7a53027d1b7e4972e28478e7a28e2faee2",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "iPhone",
    "error" : {
      "code" : -9,
      "failureReason" : "",
      "description" : "iPhone is not paired with your computer.",
      "domain" : "com.apple.platform.iphoneos"
    }
  }
]
''';

            fakeProcessManager.addCommand(
              const FakeCommand(
                command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
                stdout: devicesOutput,
              ),
            );
            final List<IOSDevice> devices = await xcdevice.getAvailableIOSDevices();
            expect(devices, hasLength(5));
            expect(devices[0].id, '00008027-00192736010F802E');
            expect(devices[0].name, 'An iPhone (Space Gray)');
            expect(await devices[0].sdkNameAndVersion, 'iOS 13.3 17C54');
            expect(devices[0].cpuArchitecture, DarwinArch.arm64);
            expect(devices[0].connectionInterface, DeviceConnectionInterface.attached);
            expect(devices[0].isConnected, true);

            expect(devices[1].id, '98206e7a4afd4aedaff06e687594e089dede3c44');
            expect(devices[1].name, 'iPad 1');
            expect(await devices[1].sdkNameAndVersion, 'iOS 10.1 14C54');
            expect(devices[1].cpuArchitecture, DarwinArch.armv7);
            expect(devices[1].connectionInterface, DeviceConnectionInterface.attached);
            expect(devices[1].isConnected, true);

            expect(devices[2].id, '234234234234234234345445687594e089dede3c44');
            expect(devices[2].name, 'A networked iPad');
            expect(await devices[2].sdkNameAndVersion, 'iOS 10.1 14C54');
            expect(
              devices[2].cpuArchitecture,
              DarwinArch.arm64,
            ); // Defaults to arm64 for unknown architecture.
            expect(devices[2].connectionInterface, DeviceConnectionInterface.wireless);
            expect(devices[2].isConnected, true);

            expect(devices[3].id, 'f577a7903cc54959be2e34bc4f7f80b7009efcf4');
            expect(devices[3].name, 'iPad 2');
            expect(await devices[3].sdkNameAndVersion, 'iOS 10.1 14C54');
            expect(
              devices[3].cpuArchitecture,
              DarwinArch.arm64,
            ); // Defaults to arm64 for unknown architecture.
            expect(devices[3].connectionInterface, DeviceConnectionInterface.attached);
            expect(devices[3].isConnected, true);

            expect(devices[4].id, 'c4ca6f7a53027d1b7e4972e28478e7a28e2faee2');
            expect(devices[4].name, 'iPhone');
            expect(await devices[4].sdkNameAndVersion, 'iOS 13.3 17C54');
            expect(devices[4].cpuArchitecture, DarwinArch.arm64);
            expect(devices[4].connectionInterface, DeviceConnectionInterface.attached);
            expect(devices[4].isConnected, false);

            expect(fakeProcessManager, hasNoRemainingExpectations);
          },
          overrides: <Type, Generator>{
            Platform: () => macPlatform,
            Artifacts: () => Artifacts.test(),
          },
        );

        testWithoutContext('available devices xcdevice fails', () async {
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
              exception: ProcessException('xcrun', <String>['xcdevice', 'list', '--timeout', '2']),
            ),
          );

          expect(await xcdevice.getAvailableIOSDevices(), isEmpty);
        });

        testWithoutContext('uses timeout', () async {
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '20'],
              stdout: '[]',
            ),
          );
          await xcdevice.getAvailableIOSDevices(timeout: const Duration(seconds: 20));
          expect(fakeProcessManager, hasNoRemainingExpectations);
        });

        testUsingContext(
          'ignores "Preparing debugger support for iPhone" error',
          () async {
            const String devicesOutput = '''
[
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "43ad2fda7991b34fe1acbda82f9e2fd3d6ddc9f7",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "iPhone",
    "error" : {
      "code" : -10,
      "failureReason" : "",
      "description" : "iPhone is busy: Preparing debugger support for iPhone",
      "recoverySuggestion" : "Xcode will continue when iPhone is finished.",
      "domain" : "com.apple.platform.iphoneos"
    }
  }
]
''';

            fakeProcessManager.addCommand(
              const FakeCommand(
                command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
                stdout: devicesOutput,
              ),
            );
            final List<IOSDevice> devices = await xcdevice.getAvailableIOSDevices();
            expect(devices, hasLength(1));
            expect(devices[0].id, '43ad2fda7991b34fe1acbda82f9e2fd3d6ddc9f7');
            expect(fakeProcessManager, hasNoRemainingExpectations);
          },
          overrides: <Type, Generator>{
            Platform: () => macPlatform,
            Artifacts: () => Artifacts.test(),
          },
        );

        testUsingContext(
          'handles unknown architectures',
          () async {
            const String devicesOutput = '''
[
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "d83d5bc53967baa0ee18626ba87b6254b2ab5418",
    "architecture" : "armv7x",
    "modelName" : "iPad 3 BOGUS",
    "name" : "iPad"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "43ad2fda7991b34fe1acbda82f9e2fd3d6ddc9f7",
    "architecture" : "BOGUS",
    "modelName" : "Future iPad",
    "name" : "iPad"
  }
]
''';

            fakeProcessManager.addCommand(
              const FakeCommand(
                command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
                stdout: devicesOutput,
              ),
            );
            final List<IOSDevice> devices = await xcdevice.getAvailableIOSDevices();
            expect(devices[0].cpuArchitecture, DarwinArch.armv7);
            expect(devices[1].cpuArchitecture, DarwinArch.arm64);
            expect(fakeProcessManager, hasNoRemainingExpectations);
          },
          overrides: <Type, Generator>{
            Platform: () => macPlatform,
            Artifacts: () => Artifacts.test(),
          },
        );

        testUsingContext('Sdk Version is parsed correctly', () async {
          const String devicesOutput = '''
[
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "00008027-00192736010F802E",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "An iPhone (Space Gray)"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "10.1",
    "interface" : "usb",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPad11,4",
    "identifier" : "234234234234234234345445687594e089dede3c44",
    "architecture" : "arm64",
    "modelName" : "iPad Air 3rd Gen",
    "name" : "A networked iPad"
  },
  {
    "simulator" : false,
    "interface" : "usb",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPad11,4",
    "identifier" : "f577a7903cc54959be2e34bc4f7f80b7009efcf4",
    "architecture" : "BOGUS",
    "modelName" : "iPad Air 3rd Gen",
    "name" : "iPad 2"
  }
]
''';
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
              stdout: devicesOutput,
            ),
          );

          final List<IOSDevice> devices = await xcdevice.getAvailableIOSDevices();
          expect(await devices[0].sdkNameAndVersion, 'iOS 13.3 17C54');
          expect(await devices[1].sdkNameAndVersion, 'iOS 10.1');
          expect(await devices[2].sdkNameAndVersion, 'iOS unknown version');
        }, overrides: <Type, Generator>{Platform: () => macPlatform});

        testUsingContext(
          'use connected entry when filtering out duplicates',
          () async {
            const String devicesOutput = '''
[
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "c4ca6f7a53027d1b7e4972e28478e7a28e2faee2",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "iPhone"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "c4ca6f7a53027d1b7e4972e28478e7a28e2faee2",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "iPhone",
    "error" : {
      "code" : -13,
      "failureReason" : "",
      "description" : "iPhone iPad is not connected",
      "recoverySuggestion" : "Xcode will continue when iPhone is connected and unlocked.",
      "domain" : "com.apple.platform.iphoneos"
    }
  }
]
''';

            fakeProcessManager.addCommand(
              const FakeCommand(
                command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
                stdout: devicesOutput,
              ),
            );
            final List<IOSDevice> devices = await xcdevice.getAvailableIOSDevices();
            expect(devices, hasLength(1));

            expect(devices[0].id, 'c4ca6f7a53027d1b7e4972e28478e7a28e2faee2');
            expect(devices[0].name, 'iPhone');
            expect(await devices[0].sdkNameAndVersion, 'iOS 13.3 17C54');
            expect(devices[0].cpuArchitecture, DarwinArch.arm64);
            expect(devices[0].connectionInterface, DeviceConnectionInterface.attached);
            expect(devices[0].isConnected, true);

            expect(fakeProcessManager, hasNoRemainingExpectations);
          },
          overrides: <Type, Generator>{
            Platform: () => macPlatform,
            Artifacts: () => Artifacts.test(),
          },
        );

        testUsingContext(
          'use entry with sdk when filtering out duplicates',
          () async {
            const String devicesOutput = '''
[
  {
    "simulator" : false,
    "interface" : "usb",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "c4ca6f7a53027d1b7e4972e28478e7a28e2faee2",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "iPhone_1",
    "error" : {
      "code" : -13,
      "failureReason" : "",
      "description" : "iPhone iPad is not connected",
      "recoverySuggestion" : "Xcode will continue when iPhone is connected and unlocked.",
      "domain" : "com.apple.platform.iphoneos"
    }
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "c4ca6f7a53027d1b7e4972e28478e7a28e2faee2",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "iPhone_2",
    "error" : {
      "code" : -13,
      "failureReason" : "",
      "description" : "iPhone iPad is not connected",
      "recoverySuggestion" : "Xcode will continue when iPhone is connected and unlocked.",
      "domain" : "com.apple.platform.iphoneos"
    }
  }
]
''';

            fakeProcessManager.addCommand(
              const FakeCommand(
                command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
                stdout: devicesOutput,
              ),
            );
            final List<IOSDevice> devices = await xcdevice.getAvailableIOSDevices();
            expect(devices, hasLength(1));

            expect(devices[0].id, 'c4ca6f7a53027d1b7e4972e28478e7a28e2faee2');
            expect(devices[0].name, 'iPhone_2');
            expect(await devices[0].sdkNameAndVersion, 'iOS 13.3 17C54');
            expect(devices[0].cpuArchitecture, DarwinArch.arm64);
            expect(devices[0].connectionInterface, DeviceConnectionInterface.attached);
            expect(devices[0].isConnected, false);

            expect(fakeProcessManager, hasNoRemainingExpectations);
          },
          overrides: <Type, Generator>{
            Platform: () => macPlatform,
            Artifacts: () => Artifacts.test(),
          },
        );

        testUsingContext(
          'use entry with higher sdk when filtering out duplicates',
          () async {
            const String devicesOutput = '''
[
  {
    "simulator" : false,
    "operatingSystemVersion" : "14.3 (17C54)",
    "interface" : "usb",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "c4ca6f7a53027d1b7e4972e28478e7a28e2faee2",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "iPhone_1",
    "error" : {
      "code" : -13,
      "failureReason" : "",
      "description" : "iPhone iPad is not connected",
      "recoverySuggestion" : "Xcode will continue when iPhone is connected and unlocked.",
      "domain" : "com.apple.platform.iphoneos"
    }
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "c4ca6f7a53027d1b7e4972e28478e7a28e2faee2",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "iPhone_2",
    "error" : {
      "code" : -13,
      "failureReason" : "",
      "description" : "iPhone iPad is not connected",
      "recoverySuggestion" : "Xcode will continue when iPhone is connected and unlocked.",
      "domain" : "com.apple.platform.iphoneos"
    }
  }
]
''';

            fakeProcessManager.addCommand(
              const FakeCommand(
                command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
                stdout: devicesOutput,
              ),
            );
            final List<IOSDevice> devices = await xcdevice.getAvailableIOSDevices();
            expect(devices, hasLength(1));

            expect(devices[0].id, 'c4ca6f7a53027d1b7e4972e28478e7a28e2faee2');
            expect(devices[0].name, 'iPhone_1');
            expect(await devices[0].sdkNameAndVersion, 'iOS 14.3 17C54');
            expect(devices[0].cpuArchitecture, DarwinArch.arm64);
            expect(devices[0].connectionInterface, DeviceConnectionInterface.attached);
            expect(devices[0].isConnected, false);

            expect(fakeProcessManager, hasNoRemainingExpectations);
          },
          overrides: <Type, Generator>{
            Platform: () => macPlatform,
            Artifacts: () => Artifacts.test(),
          },
        );

        testUsingContext('handles bad output', () async {
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
              stdout: 'Something bad happened, not JSON',
            ),
          );

          final List<IOSDevice> devices = await xcdevice.getAvailableIOSDevices();
          expect(devices, isEmpty);
          expect(logger.errorText, contains('xcdevice returned non-JSON response'));
        }, overrides: <Type, Generator>{Platform: () => macPlatform});

        group('with CoreDevices', () {
          testUsingContext(
            'returns devices with corresponding CoreDevices',
            () async {
              const String devicesOutput = '''
[
  {
    "simulator" : true,
    "operatingSystemVersion" : "13.3 (17K446)",
    "available" : true,
    "platform" : "com.apple.platform.appletvsimulator",
    "modelCode" : "AppleTV5,3",
    "identifier" : "CBB5E1ED-2172-446E-B4E7-F2B5823DBBA6",
    "architecture" : "x86_64",
    "modelName" : "Apple TV",
    "name" : "Apple TV"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "00008027-00192736010F802E",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "An iPhone (Space Gray)"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "10.1 (14C54)",
    "interface" : "usb",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPad11,4",
    "identifier" : "98206e7a4afd4aedaff06e687594e089dede3c44",
    "architecture" : "armv7",
    "modelName" : "iPad Air 3rd Gen",
    "name" : "iPad 1"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "10.1 (14C54)",
    "interface" : "network",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPad11,4",
    "identifier" : "234234234234234234345445687594e089dede3c44",
    "architecture" : "arm64",
    "modelName" : "iPad Air 3rd Gen",
    "name" : "A networked iPad"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "10.1 (14C54)",
    "interface" : "usb",
    "available" : true,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPad11,4",
    "identifier" : "f577a7903cc54959be2e34bc4f7f80b7009efcf4",
    "architecture" : "BOGUS",
    "modelName" : "iPad Air 3rd Gen",
    "name" : "iPad 2"
  },
  {
    "simulator" : true,
    "operatingSystemVersion" : "6.1.1 (17S445)",
    "available" : true,
    "platform" : "com.apple.platform.watchsimulator",
    "modelCode" : "Watch5,4",
    "identifier" : "2D74FB11-88A0-44D0-B81E-C0C142B1C94A",
    "architecture" : "i386",
    "modelName" : "Apple Watch Series 5 - 44mm",
    "name" : "Apple Watch Series 5 - 44mm"
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "c4ca6f7a53027d1b7e4972e28478e7a28e2faee2",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "iPhone",
    "error" : {
      "code" : -9,
      "failureReason" : "",
      "description" : "iPhone is not paired with your computer.",
      "domain" : "com.apple.platform.iphoneos"
    }
  }
]
''';
              coreDeviceControl.devices.addAll(<FakeIOSCoreDevice>[
                FakeIOSCoreDevice(
                  udid: '00008027-00192736010F802E',
                  connectionInterface: DeviceConnectionInterface.wireless,
                  developerModeStatus: 'enabled',
                ),
                FakeIOSCoreDevice(
                  connectionInterface: DeviceConnectionInterface.wireless,
                  developerModeStatus: 'enabled',
                ),
                FakeIOSCoreDevice(
                  udid: '234234234234234234345445687594e089dede3c44',
                  connectionInterface: DeviceConnectionInterface.attached,
                ),
                FakeIOSCoreDevice(
                  udid: 'f577a7903cc54959be2e34bc4f7f80b7009efcf4',
                  connectionInterface: DeviceConnectionInterface.attached,
                  developerModeStatus: 'disabled',
                ),
              ]);

              fakeProcessManager.addCommand(
                const FakeCommand(
                  command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
                  stdout: devicesOutput,
                ),
              );

              final List<IOSDevice> devices = await xcdevice.getAvailableIOSDevices();
              expect(devices, hasLength(5));
              expect(devices[0].id, '00008027-00192736010F802E');
              expect(devices[0].name, 'An iPhone (Space Gray)');
              expect(await devices[0].sdkNameAndVersion, 'iOS 13.3 17C54');
              expect(devices[0].cpuArchitecture, DarwinArch.arm64);
              expect(devices[0].connectionInterface, DeviceConnectionInterface.wireless);
              expect(devices[0].isConnected, true);
              expect(devices[0].devModeEnabled, true);

              expect(devices[1].id, '98206e7a4afd4aedaff06e687594e089dede3c44');
              expect(devices[1].name, 'iPad 1');
              expect(await devices[1].sdkNameAndVersion, 'iOS 10.1 14C54');
              expect(devices[1].cpuArchitecture, DarwinArch.armv7);
              expect(devices[1].connectionInterface, DeviceConnectionInterface.attached);
              expect(devices[1].isConnected, true);
              expect(devices[1].devModeEnabled, true);

              expect(devices[2].id, '234234234234234234345445687594e089dede3c44');
              expect(devices[2].name, 'A networked iPad');
              expect(await devices[2].sdkNameAndVersion, 'iOS 10.1 14C54');
              expect(
                devices[2].cpuArchitecture,
                DarwinArch.arm64,
              ); // Defaults to arm64 for unknown architecture.
              expect(devices[2].connectionInterface, DeviceConnectionInterface.attached);
              expect(devices[2].isConnected, true);
              expect(devices[2].devModeEnabled, false);

              expect(devices[3].id, 'f577a7903cc54959be2e34bc4f7f80b7009efcf4');
              expect(devices[3].name, 'iPad 2');
              expect(await devices[3].sdkNameAndVersion, 'iOS 10.1 14C54');
              expect(
                devices[3].cpuArchitecture,
                DarwinArch.arm64,
              ); // Defaults to arm64 for unknown architecture.
              expect(devices[3].connectionInterface, DeviceConnectionInterface.attached);
              expect(devices[3].isConnected, true);
              expect(devices[3].devModeEnabled, false);

              expect(devices[4].id, 'c4ca6f7a53027d1b7e4972e28478e7a28e2faee2');
              expect(devices[4].name, 'iPhone');
              expect(await devices[4].sdkNameAndVersion, 'iOS 13.3 17C54');
              expect(devices[4].cpuArchitecture, DarwinArch.arm64);
              expect(devices[4].connectionInterface, DeviceConnectionInterface.attached);
              expect(devices[4].isConnected, false);
              expect(devices[4].devModeEnabled, true);

              expect(fakeProcessManager, hasNoRemainingExpectations);

              expect(
                fakeAnalytics.sentEvents,
                contains(Event.appleUsageEvent(workflow: 'device', parameter: 'ios-trust-failure')),
              );
            },
            overrides: <Type, Generator>{
              Platform: () => macPlatform,
              Artifacts: () => Artifacts.test(),
            },
          );
        });
      });

      group('diagnostics', () {
        final FakePlatform macPlatform = FakePlatform(operatingSystem: 'macos');
        testUsingContext('uses cache', () async {
          const String devicesOutput = '''
[
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "network",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "d83d5bc53967baa0ee18626ba87b6254b2ab5418",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "error" : {
      "code" : -13,
      "failureReason" : "",
      "domain" : "com.apple.platform.iphoneos"
    }
  }
]
''';

          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
              stdout: devicesOutput,
            ),
          );

          await xcdevice.getAvailableIOSDevices();
          final List<String> errors = await xcdevice.getDiagnostics();
          expect(errors, hasLength(1));
          expect(fakeProcessManager, hasNoRemainingExpectations);
        }, overrides: <Type, Generator>{Platform: () => macPlatform});

        testWithoutContext('diagnostics xcdevice fails', () async {
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
              exception: ProcessException('xcrun', <String>['xcdevice', 'list', '--timeout', '2']),
            ),
          );

          expect(await xcdevice.getDiagnostics(), isEmpty);
        });

        testUsingContext('returns error message', () async {
          const String devicesOutput = '''
[
   {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "98206e7a4afd4aedaff06e687594e089dede3c44",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "An iPhone (Space Gray)",
    "error" : {
      "code" : -9,
      "failureReason" : "",
      "underlyingErrors" : [
        {
          "code" : 5,
          "failureReason" : "allowsSecureServices: 1. isConnected: 0. Platform: <DVTPlatform:0x7f804ce32880:'com.apple.platform.iphoneos':<DVTFilePath:0x7f804ce32800:'/Users/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform'>>. DTDKDeviceIdentifierIsIDID: 0",
          "description" : "📱<DVTiOSDevice (0x7f801f190450), iPhone, iPhone, 13.3 (17C54), d83d5bc53967baa0ee18626ba87b6254b2ab5418> -- Failed _shouldMakeReadyForDevelopment check even though device is not locked by passcode.",
          "recoverySuggestion" : "",
          "domain" : "com.apple.platform.iphoneos"
        }
      ],
      "description" : "iPhone is not paired with your computer.",
      "recoverySuggestion" : "To use iPhone with Xcode, unlock it and choose to trust this computer when prompted.",
      "domain" : "com.apple.platform.iphoneos"
    }
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "d83d5bc53967baa0ee18626ba87b6254b2ab5418",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "iPhone",
    "error" : {
      "failureReason" : "",
      "description" : "iPhone is not paired with your computer",
      "domain" : "com.apple.platform.iphoneos"
    }
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "network",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "d83d5bc53967baa0ee18626ba87b6254b2ab5418",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "error" : {
      "code" : -13,
      "failureReason" : "",
      "domain" : "com.apple.platform.iphoneos"
    }
  },
  {
    "simulator" : false,
    "operatingSystemVersion" : "13.3 (17C54)",
    "interface" : "usb",
    "available" : false,
    "platform" : "com.apple.platform.iphoneos",
    "modelCode" : "iPhone8,1",
    "identifier" : "43ad2fda7991b34fe1acbda82f9e2fd3d6ddc9f7",
    "architecture" : "arm64",
    "modelName" : "iPhone 6s",
    "name" : "iPhone",
    "error" : {
      "code" : -10,
      "failureReason" : "",
      "description" : "iPhone is busy: Preparing debugger support for iPhone",
      "recoverySuggestion" : "Xcode will continue when iPhone is finished.",
      "domain" : "com.apple.platform.iphoneos"
    }
  },
  {
    "modelCode" : "iPad8,5",
    "simulator" : false,
    "modelName" : "iPad Pro (12.9-inch) (3rd generation)",
    "error" : {
      "code" : -13,
      "failureReason" : "",
      "underlyingErrors" : [
        {
          "code" : 4,
          "failureReason" : "",
          "description" : "iPad is locked.",
          "recoverySuggestion" : "To use iPad with Xcode, unlock it.",
          "domain" : "DVTDeviceIneligibilityErrorDomain"
        }
      ],
      "description" : "iPad is not connected",
      "recoverySuggestion" : "Xcode will continue when iPad is connected.",
      "domain" : "com.apple.platform.iphoneos"
    },
    "operatingSystemVersion" : "15.6 (19G5027e)",
    "identifier" : "00008027-0019253601068123",
    "platform" : "com.apple.platform.iphoneos",
    "architecture" : "arm64e",
    "interface" : "usb",
    "available" : false,
    "name" : "iPad",
    "modelUTI" : "com.apple.ipad-pro-12point9-1"
  }
]
''';

          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>['xcrun', 'xcdevice', 'list', '--timeout', '2'],
              stdout: devicesOutput,
            ),
          );

          final List<String> errors = await xcdevice.getDiagnostics();
          expect(errors, hasLength(4));

          expect(
            errors[0],
            'Error: iPhone is not paired with your computer. To use iPhone with Xcode, unlock it and choose to trust this computer when prompted. (code -9)',
          );
          expect(errors[1], 'Error: iPhone is not paired with your computer.');
          expect(errors[2], 'Error: Xcode pairing error. (code -13)');
          expect(
            errors[3],
            'Error: iPhone is busy: Preparing debugger support for iPhone. Xcode will continue when iPhone is finished. (code -10)',
          );
          expect(errors, isNot(contains('Xcode will continue')));
          expect(fakeProcessManager, hasNoRemainingExpectations);
        }, overrides: <Type, Generator>{Platform: () => macPlatform});
      });
    });
  });
}

class FakeXcodeProjectInterpreter extends Fake implements XcodeProjectInterpreter {
  @override
  Version version = Version(0, 0, 0);

  @override
  bool isInstalled = false;

  @override
  List<String> xcrunCommand() => <String>['xcrun'];
}

class FakeXcodeDebug extends Fake implements XcodeDebug {}

class FakeShutdownHooks extends Fake implements ShutdownHooks {
  @override
  void addShutdownHook(ShutdownHook shutdownHook) {}
}

class FakeIOSCoreDeviceControl extends Fake implements IOSCoreDeviceControl {
  List<FakeIOSCoreDevice> devices = <FakeIOSCoreDevice>[];

  @override
  Future<List<IOSCoreDevice>> getCoreDevices({Duration timeout = Duration.zero}) async {
    return devices;
  }
}

class FakeIOSCoreDevice extends Fake implements IOSCoreDevice {
  FakeIOSCoreDevice({this.udid, this.connectionInterface, this.developerModeStatus});

  final String? developerModeStatus;

  @override
  final String? udid;

  @override
  final DeviceConnectionInterface? connectionInterface;

  @override
  IOSCoreDeviceProperties? get deviceProperties =>
      FakeIOSCoreDeviceProperties(developerModeStatus: developerModeStatus);
}

class FakeIOSCoreDeviceProperties extends Fake implements IOSCoreDeviceProperties {
  FakeIOSCoreDeviceProperties({required this.developerModeStatus});

  @override
  final String? developerModeStatus;
}
