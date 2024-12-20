// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/dds.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/native_assets.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/test/flutter_tester_device.dart';
import 'package:flutter_tools/src/test/font_config_manager.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:stream_channel/stream_channel.dart';
import 'package:test/fake.dart';

import '../src/context.dart';
import '../src/fake_process_manager.dart';
import '../src/fake_vm_services.dart';
import '../src/fakes.dart';

void main() {
  late FakePlatform platform;
  late FileSystem fileSystem;
  late FakeProcessManager processManager;
  late FlutterTesterTestDevice device;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    // Not Windows.
    platform = FakePlatform(environment: <String, String>{});
    processManager = FakeProcessManager.any();
  });

  FlutterTesterTestDevice createDevice({
    List<String> dartEntrypointArgs = const <String>[],
    bool enableVmService = false,
    bool enableImpeller = false,
    FlutterProject? flutterProject,
  }) => TestFlutterTesterDevice(
    platform: platform,
    fileSystem: fileSystem,
    processManager: processManager,
    enableVmService: enableVmService,
    dartEntrypointArgs: dartEntrypointArgs,
    enableImpeller: enableImpeller,
    flutterProject: flutterProject,
  );

  testUsingContext(
    'Missing dir error caught for FontConfigManger.dispose',
    () async {
      final FontConfigManager fontConfigManager = FontConfigManager();

      final Directory fontsDirectory = fileSystem.file(fontConfigManager.fontConfigFile).parent;
      fontsDirectory.deleteSync(recursive: true);

      await fontConfigManager.dispose();
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'Flutter tester passes through impeller config and environment variables.',
    () async {
      processManager = FakeProcessManager.list(<FakeCommand>[]);
      device = createDevice(enableImpeller: true);
      processManager.addCommand(
        FakeCommand(
          command: const <String>[
            '/',
            '--disable-vm-service',
            '--ipv6',
            '--enable-checked-mode',
            '--verify-entry-points',
            '--enable-impeller',
            '--enable-dart-profiling',
            '--non-interactive',
            '--use-test-fonts',
            '--disable-asset-fonts',
            '--packages=.dart_tool/package_config.json',
            'example.dill',
          ],
          environment: <String, String>{
            'FLUTTER_TEST': 'true',
            'FONTCONFIG_FILE': device.fontConfigManager.fontConfigFile.path,
            'SERVER_PORT': '0',
            'APP_NAME': '',
            'FLUTTER_TEST_IMPELLER': 'true',
          },
        ),
      );

      await device.start('example.dill');

      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'The PATH environment variable contains native assets build dir on Windows',
    () async {
      platform = FakePlatform(
        environment: <String, String>{'PATH': r'C:\existing\path'},
        operatingSystem: 'windows',
      );
      processManager = FakeProcessManager.list(<FakeCommand>[]);
      final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
      device = createDevice(enableImpeller: true, flutterProject: project);
      processManager.addCommand(
        FakeCommand(
          command: const <String>[
            '/',
            '--disable-vm-service',
            '--ipv6',
            '--enable-checked-mode',
            '--verify-entry-points',
            '--enable-impeller',
            '--enable-dart-profiling',
            '--non-interactive',
            '--use-test-fonts',
            '--disable-asset-fonts',
            '--packages=.dart_tool/package_config.json',
            'example.dill',
          ],
          environment: <String, String>{
            'FLUTTER_TEST': 'true',
            'FONTCONFIG_FILE': device.fontConfigManager.fontConfigFile.path,
            'SERVER_PORT': '0',
            'APP_NAME': '',
            'FLUTTER_TEST_IMPELLER': 'true',
            'PATH':
                '${device.nativeAssetsBuilder!.windowsBuildDirectory(project)};${platform.environment['PATH']}',
          },
        ),
      );

      await device.start('example.dill');

      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  group('The FLUTTER_TEST environment variable is passed to the test process', () {
    setUp(() {
      processManager = FakeProcessManager.list(<FakeCommand>[]);
      device = createDevice();

      fileSystem.file('.dart_tool/package_config.json')
        ..createSync(recursive: true)
        ..writeAsStringSync('{"configVersion":2,"packages":[]}');
    });

    FakeCommand flutterTestCommand(String expectedFlutterTestValue) {
      return FakeCommand(
        command: const <String>[
          '/',
          '--disable-vm-service',
          '--ipv6',
          '--enable-checked-mode',
          '--verify-entry-points',
          '--enable-software-rendering',
          '--skia-deterministic-rendering',
          '--enable-dart-profiling',
          '--non-interactive',
          '--use-test-fonts',
          '--disable-asset-fonts',
          '--packages=.dart_tool/package_config.json',
          'example.dill',
        ],
        environment: <String, String>{
          'FLUTTER_TEST': expectedFlutterTestValue,
          'FONTCONFIG_FILE': device.fontConfigManager.fontConfigFile.path,
          'SERVER_PORT': '0',
          'APP_NAME': '',
        },
      );
    }

    testUsingContext('as true when not originally set', () async {
      processManager.addCommand(flutterTestCommand('true'));

      await device.start('example.dill');
      expect(processManager, hasNoRemainingExpectations);
    });

    testUsingContext('as true when set to true', () async {
      platform.environment = <String, String>{'FLUTTER_TEST': 'true'};
      processManager.addCommand(flutterTestCommand('true'));

      await device.start('example.dill');
      expect(processManager, hasNoRemainingExpectations);
    });

    testUsingContext('as false when set to false', () async {
      platform.environment = <String, String>{'FLUTTER_TEST': 'false'};
      processManager.addCommand(flutterTestCommand('false'));

      await device.start('example.dill');
      expect(processManager, hasNoRemainingExpectations);
    });

    testUsingContext('unchanged when set', () async {
      platform.environment = <String, String>{'FLUTTER_TEST': 'neither true nor false'};
      processManager.addCommand(flutterTestCommand('neither true nor false'));

      await device.start('example.dill');
      expect(processManager, hasNoRemainingExpectations);
    });
  });

  group('Dart Entrypoint Args', () {
    setUp(() {
      processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>[
            '/',
            '--disable-vm-service',
            '--ipv6',
            '--enable-checked-mode',
            '--verify-entry-points',
            '--enable-software-rendering',
            '--skia-deterministic-rendering',
            '--enable-dart-profiling',
            '--non-interactive',
            '--use-test-fonts',
            '--disable-asset-fonts',
            '--packages=.dart_tool/package_config.json',
            '--foo',
            '--bar',
            'example.dill',
          ],
          stdout: 'success',
          stderr: 'failure',
        ),
      ]);
      device = createDevice(dartEntrypointArgs: <String>['--foo', '--bar']);
    });

    testUsingContext('Can pass additional arguments to tester binary', () async {
      await device.start('example.dill');

      expect(processManager, hasNoRemainingExpectations);
    });
  });

  group('DDS', () {
    late DDSLauncherCallback originalDdsLauncher;
    setUp(() {
      processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>[
            '/',
            '--vm-service-port=0',
            '--ipv6',
            '--enable-checked-mode',
            '--verify-entry-points',
            '--enable-software-rendering',
            '--skia-deterministic-rendering',
            '--enable-dart-profiling',
            '--non-interactive',
            '--use-test-fonts',
            '--disable-asset-fonts',
            '--packages=.dart_tool/package_config.json',
            'example.dill',
          ],
          stdout: 'The Dart VM service is listening on http://localhost:1234',
          stderr: 'failure',
        ),
      ]);
      device = createDevice(enableVmService: true);
      originalDdsLauncher = ddsLauncherCallback;
      ddsLauncherCallback = ({
        required Uri remoteVmServiceUri,
        Uri? serviceUri,
        bool enableAuthCodes = true,
        bool serveDevTools = false,
        Uri? devToolsServerAddress,
        bool enableServicePortFallback = false,
        List<String> cachedUserTags = const <String>[],
        String? dartExecutable,
        String? google3WorkspaceRoot,
      }) async {
        return FakeDartDevelopmentServiceLauncher(uri: Uri.parse('http://localhost:1234'));
      };
    });

    tearDown(() {
      ddsLauncherCallback = originalDdsLauncher;
    });

    testUsingContext(
      'skips setting VM Service port and uses the input port for DDS instead',
      () async {
        await device.start('example.dill');
        await device.vmServiceUri;

        final Uri? uri = await (device as TestFlutterTesterDevice).vmServiceUri;
        expect(uri!.port, 1234);
      },
    );
  });
}

/// A Flutter Tester device.
///
/// Uses a mock HttpServer. We don't want to bind random ports in our CI hosts.
class TestFlutterTesterDevice extends FlutterTesterTestDevice {
  TestFlutterTesterDevice({
    required super.platform,
    required super.fileSystem,
    required super.processManager,
    required super.enableVmService,
    required List<String> dartEntrypointArgs,
    required bool enableImpeller,
    super.flutterProject,
  }) : super(
         id: 999,
         shellPath: '/',
         logger: BufferLogger.test(),
         debuggingOptions: DebuggingOptions.enabled(
           const BuildInfo(
             BuildMode.debug,
             '',
             treeShakeIcons: false,
             packageConfigPath: '.dart_tool/package_config.json',
           ),
           hostVmServicePort: 1234,
           dartEntrypointArgs: dartEntrypointArgs,
           enableImpeller: enableImpeller ? ImpellerStatus.enabled : ImpellerStatus.platformDefault,
         ),
         machine: false,
         host: InternetAddress.loopbackIPv6,
         testAssetDirectory: null,
         icudtlPath: null,
         compileExpression: null,
         fontConfigManager: FontConfigManager(),
         nativeAssetsBuilder: FakeNativeAssetsBuilder(),
       );

  @override
  Future<FlutterVmService> connectToVmServiceImpl(
    Uri httpUri, {
    CompileExpression? compileExpression,
    required Logger logger,
  }) async {
    return FakeVmServiceHost(
      requests: <VmServiceExpectation>[const FakeVmServiceRequest(method: '_serveObservatory')],
    ).vmService;
  }

  @override
  Future<HttpServer> bind(InternetAddress? host, int port) async => FakeHttpServer();

  @override
  Future<StreamChannel<String>> get remoteChannel async =>
      StreamChannelController<String>().foreign;
}

class FakeHttpServer extends Fake implements HttpServer {
  @override
  int get port => 0;
}

class FakeNativeAssetsBuilder extends Fake implements TestCompilerNativeAssetsBuilder {
  @override
  String windowsBuildDirectory(FlutterProject project) => r'C:\native_assets\path';
}
