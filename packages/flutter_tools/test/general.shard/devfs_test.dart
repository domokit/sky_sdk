// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io' as io show Process, ProcessSignal;
import 'dart:typed_data';

import 'package:args/args.dart';
import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/asset.dart';
import 'package:flutter_tools/src/base/config.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/tools/shader_compiler.dart';
import 'package:flutter_tools/src/compile.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';
import 'package:package_config/package_config.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import '../src/common.dart';
import '../src/context.dart';
import '../src/fake_http_client.dart';
import '../src/fake_process_manager.dart';
import '../src/fake_vm_services.dart';
import '../src/fakes.dart';
import '../src/logging_logger.dart';

final FakeVmServiceRequest createDevFSRequest = FakeVmServiceRequest(
  method: '_createDevFS',
  args: <String, Object>{'fsName': 'test'},
  jsonResponse: <String, Object>{'uri': Uri.parse('test').toString()},
);

FakeVmServiceRequest failingCreateDevFSRequest = FakeVmServiceRequest(
  method: '_createDevFS',
  args: <String, Object>{'fsName': 'test'},
  error: FakeRPCError(code: vm_service.RPCErrorKind.kServiceDisappeared.code),
);

FakeVmServiceRequest failingDeleteDevFSRequest = FakeVmServiceRequest(
  method: '_deleteDevFS',
  args: <String, dynamic>{'fsName': 'test'},
  error: FakeRPCError(code: vm_service.RPCErrorKind.kServiceDisappeared.code),
);

void main() {
  testWithoutContext('DevFSByteContent', () {
    final DevFSByteContent content = DevFSByteContent(<int>[4, 5, 6]);

    expect(content.bytes, orderedEquals(<int>[4, 5, 6]));
    expect(content.isModified, isTrue);
    expect(content.isModified, isFalse);
  });

  testWithoutContext('DevFSStringContent', () {
    final DevFSStringContent content = DevFSStringContent('some string');

    expect(content.string, 'some string');
    expect(content.bytes, orderedEquals(utf8.encode('some string')));
    expect(content.isModified, isTrue);
    expect(content.isModified, isFalse);
  });

  testWithoutContext('DevFSFileContent', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File file = fileSystem.file('foo.txt');
    final DevFSFileContent content = DevFSFileContent(file);
    expect(content.isModified, isFalse);
    expect(content.isModified, isFalse);

    file.parent.createSync(recursive: true);
    file.writeAsBytesSync(<int>[1, 2, 3], flush: true);

    final DateTime fiveSecondsAgo = file.statSync().modified.subtract(const Duration(seconds: 5));
    expect(content.isModifiedAfter(fiveSecondsAgo), isTrue);
    expect(content.isModifiedAfter(fiveSecondsAgo), isTrue);

    file.writeAsBytesSync(<int>[2, 3, 4], flush: true);

    expect(content.isModified, isTrue);
    expect(content.isModified, isFalse);
    expect(await content.contentsAsBytes(), <int>[2, 3, 4]);

    expect(content.isModified, isFalse);
    expect(content.isModified, isFalse);

    file.deleteSync();
    expect(content.isModified, isTrue);
    expect(content.isModified, isFalse);
    expect(content.isModified, isFalse);
  });

  testWithoutContext('DevFSStringCompressingBytesContent', () {
    final DevFSStringCompressingBytesContent content = DevFSStringCompressingBytesContent(
      'uncompressed string',
    );

    expect(content.equals('uncompressed string'), isTrue);
    expect(content.bytes, isNotNull);
    expect(content.isModified, isTrue);
    expect(content.isModified, isFalse);
  });

  testWithoutContext(
    'DevFS create throws a DevFSException when vmservice disconnects unexpectedly',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final OperatingSystemUtils osUtils = FakeOperatingSystemUtils();
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[failingCreateDevFSRequest],
        httpAddress: Uri.parse('http://localhost'),
      );

      final DevFS devFS = DevFS(
        fakeVmServiceHost.vmService,
        'test',
        fileSystem.currentDirectory,
        osUtils: osUtils,
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        httpClient: FakeHttpClient.any(),
        processManager: FakeProcessManager.empty(),
        artifacts: Artifacts.test(),
        buildMode: BuildMode.debug,
      );
      expect(() async => devFS.create(), throwsA(isA<DevFSException>()));
    },
  );

  testWithoutContext('DevFS destroy is resilient to vmservice disconnection', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final OperatingSystemUtils osUtils = FakeOperatingSystemUtils();
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[createDevFSRequest, failingDeleteDevFSRequest],
      httpAddress: Uri.parse('http://localhost'),
    );

    final DevFS devFS = DevFS(
      fakeVmServiceHost.vmService,
      'test',
      fileSystem.currentDirectory,
      osUtils: osUtils,
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
      httpClient: FakeHttpClient.any(),
      processManager: FakeProcessManager.empty(),
      artifacts: Artifacts.test(),
      buildMode: BuildMode.debug,
    );

    expect(await devFS.create(), isNotNull);
    await devFS.destroy(); // Testing that this does not throw.
  });

  testWithoutContext('DevFS retries uploads when connection reset by peer', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final OperatingSystemUtils osUtils = OperatingSystemUtils(
      fileSystem: fileSystem,
      platform: FakePlatform(),
      logger: BufferLogger.test(),
      processManager: FakeProcessManager.any(),
    );
    final FakeResidentCompiler residentCompiler = FakeResidentCompiler();
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[createDevFSRequest],
      httpAddress: Uri.parse('http://localhost'),
    );
    residentCompiler.onRecompile = (Uri mainUri, List<Uri>? invalidatedFiles) async {
      fileSystem.file('lib/foo.dill')
        ..createSync(recursive: true)
        ..writeAsBytesSync(<int>[1, 2, 3, 4, 5]);
      return const CompilerOutput('lib/foo.dill', 0, <Uri>[]);
    };

    /// This output can change based on the host platform.
    final List<List<int>> expectedEncoded =
        await osUtils.gzipLevel1Stream(Stream<List<int>>.value(<int>[1, 2, 3, 4, 5])).toList();

    final DevFS devFS = DevFS(
      fakeVmServiceHost.vmService,
      'test',
      fileSystem.currentDirectory,
      osUtils: osUtils,
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
      httpClient: FakeHttpClient.list(<FakeRequest>[
        FakeRequest(
          Uri.parse('http://localhost'),
          method: HttpMethod.put,
          responseError: const OSError('Connection Reset by peer'),
        ),
        FakeRequest(
          Uri.parse('http://localhost'),
          method: HttpMethod.put,
          responseError: const OSError('Connection Reset by peer'),
        ),
        FakeRequest(
          Uri.parse('http://localhost'),
          method: HttpMethod.put,
          responseError: const OSError('Connection Reset by peer'),
        ),
        FakeRequest(
          Uri.parse('http://localhost'),
          method: HttpMethod.put,
          responseError: const OSError('Connection Reset by peer'),
        ),
        FakeRequest(
          Uri.parse('http://localhost'),
          method: HttpMethod.put,
          responseError: const OSError('Connection Reset by peer'),
        ),
        // This is the value of `<int>[1, 2, 3, 4, 5]` run through `osUtils.gzipLevel1Stream`.
        FakeRequest(
          Uri.parse('http://localhost'),
          method: HttpMethod.put,
          body: <int>[for (final List<int> chunk in expectedEncoded) ...chunk],
        ),
      ]),
      uploadRetryThrottle: Duration.zero,
      processManager: FakeProcessManager.empty(),
      artifacts: Artifacts.test(),
      buildMode: BuildMode.debug,
    );
    await devFS.create();

    final UpdateFSReport report = await devFS.update(
      mainUri: Uri.parse('lib/foo.txt'),
      dillOutputPath: 'lib/foo.dill',
      generator: residentCompiler,
      pathToReload: 'lib/foo.txt.dill',
      trackWidgetCreation: false,
      invalidatedFiles: <Uri>[],
      packageConfig: PackageConfig.empty,
      shaderCompiler: const FakeShaderCompiler(),
    );

    expect(report.syncedBytes, 5);
    expect(report.success, isTrue);
  });

  testWithoutContext('DevFS reports unsuccessful compile when errors are returned', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[createDevFSRequest],
      httpAddress: Uri.parse('http://localhost'),
    );

    final DevFS devFS = DevFS(
      fakeVmServiceHost.vmService,
      'test',
      fileSystem.currentDirectory,
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
      osUtils: FakeOperatingSystemUtils(),
      httpClient: FakeHttpClient.any(),
      processManager: FakeProcessManager.empty(),
      artifacts: Artifacts.test(),
      buildMode: BuildMode.debug,
    );

    await devFS.create();
    final DateTime? previousCompile = devFS.lastCompiled;

    final FakeResidentCompiler residentCompiler = FakeResidentCompiler();
    residentCompiler.onRecompile = (Uri mainUri, List<Uri>? invalidatedFiles) async {
      return const CompilerOutput('lib/foo.dill', 2, <Uri>[]);
    };

    final UpdateFSReport report = await devFS.update(
      mainUri: Uri.parse('lib/foo.txt'),
      generator: residentCompiler,
      dillOutputPath: 'lib/foo.dill',
      pathToReload: 'lib/foo.txt.dill',
      trackWidgetCreation: false,
      invalidatedFiles: <Uri>[],
      packageConfig: PackageConfig.empty,
      shaderCompiler: const FakeShaderCompiler(),
    );

    expect(report.success, false);
    expect(devFS.lastCompiled, previousCompile);
  });

  testWithoutContext(
    'DevFS correctly updates last compiled time when compilation does not fail',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[createDevFSRequest],
        httpAddress: Uri.parse('http://localhost'),
      );

      final DevFS devFS = DevFS(
        fakeVmServiceHost.vmService,
        'test',
        fileSystem.currentDirectory,
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        osUtils: FakeOperatingSystemUtils(),
        httpClient: FakeHttpClient.any(),
        processManager: FakeProcessManager.empty(),
        artifacts: Artifacts.test(),
        buildMode: BuildMode.debug,
      );

      await devFS.create();
      final DateTime? previousCompile = devFS.lastCompiled;

      final FakeResidentCompiler residentCompiler = FakeResidentCompiler();
      residentCompiler.onRecompile = (Uri mainUri, List<Uri>? invalidatedFiles) async {
        fileSystem.file('lib/foo.txt.dill').createSync(recursive: true);
        return const CompilerOutput('lib/foo.txt.dill', 0, <Uri>[]);
      };

      final UpdateFSReport report = await devFS.update(
        mainUri: Uri.parse('lib/main.dart'),
        generator: residentCompiler,
        dillOutputPath: 'lib/foo.dill',
        pathToReload: 'lib/foo.txt.dill',
        trackWidgetCreation: false,
        invalidatedFiles: <Uri>[],
        packageConfig: PackageConfig.empty,
        shaderCompiler: const FakeShaderCompiler(),
      );

      expect(report.success, true);
      expect(devFS.lastCompiled, isNot(previousCompile));
    },
  );

  testWithoutContext('DevFS can reset compilation time', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[createDevFSRequest],
    );
    final LocalDevFSWriter localDevFSWriter = LocalDevFSWriter(fileSystem: fileSystem);
    fileSystem.directory('test').createSync();

    final DevFS devFS = DevFS(
      fakeVmServiceHost.vmService,
      'test',
      fileSystem.currentDirectory,
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
      osUtils: FakeOperatingSystemUtils(),
      httpClient: HttpClient(),
      processManager: FakeProcessManager.empty(),
      artifacts: Artifacts.test(),
      buildMode: BuildMode.debug,
    );

    await devFS.create();
    final DateTime? previousCompile = devFS.lastCompiled;

    final FakeResidentCompiler residentCompiler = FakeResidentCompiler();
    residentCompiler.onRecompile = (Uri mainUri, List<Uri>? invalidatedFiles) async {
      fileSystem.file('lib/foo.txt.dill').createSync(recursive: true);
      return const CompilerOutput('lib/foo.txt.dill', 0, <Uri>[]);
    };

    final UpdateFSReport report = await devFS.update(
      mainUri: Uri.parse('lib/main.dart'),
      generator: residentCompiler,
      dillOutputPath: 'lib/foo.dill',
      pathToReload: 'lib/foo.txt.dill',
      trackWidgetCreation: false,
      invalidatedFiles: <Uri>[],
      packageConfig: PackageConfig.empty,
      devFSWriter: localDevFSWriter,
      shaderCompiler: const FakeShaderCompiler(),
    );

    expect(report.success, true);
    expect(devFS.lastCompiled, isNot(previousCompile));

    devFS.resetLastCompiled();
    expect(devFS.lastCompiled, previousCompile);

    // Does not reset to report compile time.
    devFS.resetLastCompiled();
    expect(devFS.lastCompiled, previousCompile);
  });

  testWithoutContext('DevFS uses provided DevFSWriter instead of default HTTP writer', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final FakeDevFSWriter writer = FakeDevFSWriter();
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[createDevFSRequest],
    );

    final DevFS devFS = DevFS(
      fakeVmServiceHost.vmService,
      'test',
      fileSystem.currentDirectory,
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
      osUtils: FakeOperatingSystemUtils(),
      httpClient: FakeHttpClient.any(),
      processManager: FakeProcessManager.empty(),
      artifacts: Artifacts.test(),
      buildMode: BuildMode.debug,
    );

    await devFS.create();

    final FakeResidentCompiler residentCompiler = FakeResidentCompiler();
    residentCompiler.onRecompile = (Uri mainUri, List<Uri>? invalidatedFiles) async {
      fileSystem.file('example').createSync();
      return const CompilerOutput('lib/foo.txt.dill', 0, <Uri>[]);
    };

    expect(writer.written, false);

    final UpdateFSReport report = await devFS.update(
      mainUri: Uri.parse('lib/main.dart'),
      generator: residentCompiler,
      dillOutputPath: 'lib/foo.dill',
      pathToReload: 'lib/foo.txt.dill',
      trackWidgetCreation: false,
      invalidatedFiles: <Uri>[],
      packageConfig: PackageConfig.empty,
      devFSWriter: writer,
      shaderCompiler: const FakeShaderCompiler(),
    );

    expect(report.success, true);
    expect(writer.written, true);
  });

  testWithoutContext('Local DevFSWriter can copy and write files', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File file = fileSystem.file('foo_bar')..writeAsStringSync('goodbye');
    final LocalDevFSWriter writer = LocalDevFSWriter(fileSystem: fileSystem);

    await writer.write(<Uri, DevFSContent>{
      Uri.parse('hello'): DevFSStringContent('hello'),
      Uri.parse('goodbye'): DevFSFileContent(file),
    }, Uri.parse('/foo/bar/devfs/'));

    expect(fileSystem.file('/foo/bar/devfs/hello'), exists);
    expect(fileSystem.file('/foo/bar/devfs/hello').readAsStringSync(), 'hello');
    expect(fileSystem.file('/foo/bar/devfs/goodbye'), exists);
    expect(fileSystem.file('/foo/bar/devfs/goodbye').readAsStringSync(), 'goodbye');
  });

  testWithoutContext('Local DevFSWriter turns FileSystemException into DevFSException', () async {
    final FileExceptionHandler handler = FileExceptionHandler();
    final FileSystem fileSystem = MemoryFileSystem.test(opHandle: handler.opHandle);
    final LocalDevFSWriter writer = LocalDevFSWriter(fileSystem: fileSystem);
    final File file = fileSystem.file('foo');
    handler.addError(file, FileSystemOp.read, const FileSystemException('foo'));

    await expectLater(
      () async => writer.write(<Uri, DevFSContent>{
        Uri.parse('goodbye'): DevFSFileContent(file),
      }, Uri.parse('/foo/bar/devfs/')),
      throwsA(isA<DevFSException>()),
    );
  });

  testWithoutContext('DevFS correctly records the elapsed time', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    // final FakeDevFSWriter writer = FakeDevFSWriter();
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[createDevFSRequest],
      httpAddress: Uri.parse('http://localhost'),
    );

    final DevFS devFS = DevFS(
      fakeVmServiceHost.vmService,
      'test',
      fileSystem.currentDirectory,
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
      osUtils: FakeOperatingSystemUtils(),
      httpClient: FakeHttpClient.any(),
      stopwatchFactory: FakeStopwatchFactory(
        stopwatches: <String, Stopwatch>{
          'compile': FakeStopwatch()..elapsed = const Duration(seconds: 3),
          'transfer': FakeStopwatch()..elapsed = const Duration(seconds: 5),
        },
      ),
      processManager: FakeProcessManager.empty(),
      artifacts: Artifacts.test(),
      buildMode: BuildMode.debug,
    );

    await devFS.create();

    final FakeResidentCompiler residentCompiler = FakeResidentCompiler();
    residentCompiler.onRecompile = (Uri mainUri, List<Uri>? invalidatedFiles) async {
      fileSystem.file('lib/foo.txt.dill').createSync(recursive: true);
      return const CompilerOutput('lib/foo.txt.dill', 0, <Uri>[]);
    };

    final UpdateFSReport report = await devFS.update(
      mainUri: Uri.parse('lib/main.dart'),
      generator: residentCompiler,
      dillOutputPath: 'lib/foo.dill',
      pathToReload: 'lib/foo.txt.dill',
      trackWidgetCreation: false,
      invalidatedFiles: <Uri>[],
      packageConfig: PackageConfig.empty,
      shaderCompiler: const FakeShaderCompiler(),
    );

    expect(report.success, true);
    expect(report.compileDuration, const Duration(seconds: 3));
    expect(report.transferDuration, const Duration(seconds: 5));
  });

  testUsingContext('DevFS actually starts compile before processing bundle', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[createDevFSRequest],
      httpAddress: Uri.parse('http://localhost'),
    );

    final LoggingLogger logger = LoggingLogger();

    final DevFS devFS = DevFS(
      fakeVmServiceHost.vmService,
      'test',
      fileSystem.currentDirectory,
      fileSystem: fileSystem,
      logger: logger,
      osUtils: FakeOperatingSystemUtils(),
      httpClient: FakeHttpClient.any(),
      processManager: FakeProcessManager.empty(),
      artifacts: Artifacts.test(),
      buildMode: BuildMode.debug,
    );

    await devFS.create();

    final MemoryIOSink frontendServerStdIn = MemoryIOSink();
    Stream<List<int>> frontendServerStdOut() async* {
      int processed = 0;
      while (true) {
        while (frontendServerStdIn.writes.length == processed) {
          await Future<dynamic>.delayed(const Duration(milliseconds: 5));
        }

        String? boundaryKey;
        while (processed < frontendServerStdIn.writes.length) {
          final List<int> data = frontendServerStdIn.writes[processed];
          final String stringData = utf8.decode(data);
          if (stringData.startsWith('compile ')) {
            yield utf8.encode('result abc1\nline1\nline2\nabc1\nabc1 lib/foo.txt.dill 0\n');
          } else if (stringData.startsWith('recompile ')) {
            final String line = stringData.split('\n').first;
            final int spaceDelim = line.lastIndexOf(' ');
            boundaryKey = line.substring(spaceDelim + 1);
          } else if (boundaryKey != null && stringData.startsWith(boundaryKey)) {
            yield utf8.encode('result abc2\nline1\nline2\nabc2\nabc2 lib/foo.txt.dill 0\n');
          } else {
            throw Exception('Saw $data ($stringData)');
          }
          processed++;
        }
      }
    }

    Stream<List<int>> frontendServerStdErr() async* {
      // Output nothing on stderr.
    }

    final AnsweringFakeProcessManager fakeProcessManager = AnsweringFakeProcessManager(
      frontendServerStdOut(),
      frontendServerStdErr(),
      frontendServerStdIn,
    );
    final StdoutHandler generatorStdoutHandler = StdoutHandler(
      logger: testLogger,
      fileSystem: fileSystem,
    );

    final DefaultResidentCompiler residentCompiler = DefaultResidentCompiler(
      'sdkroot',
      buildMode: BuildMode.debug,
      logger: logger,
      processManager: fakeProcessManager,
      artifacts: Artifacts.test(),
      platform: FakePlatform(),
      fileSystem: fileSystem,
      stdoutHandler: generatorStdoutHandler,
    );

    fileSystem.file('lib/foo.txt.dill').createSync(recursive: true);

    final UpdateFSReport report1 = await devFS.update(
      mainUri: Uri.parse('lib/main.dart'),
      generator: residentCompiler,
      dillOutputPath: 'lib/foo.dill',
      pathToReload: 'lib/foo.txt.dill',
      trackWidgetCreation: false,
      invalidatedFiles: <Uri>[],
      packageConfig: PackageConfig.empty,
      bundle: FakeBundle(),
      shaderCompiler: const FakeShaderCompiler(),
    );
    expect(report1.success, true);
    logger.messages.clear();

    final UpdateFSReport report2 = await devFS.update(
      mainUri: Uri.parse('lib/main.dart'),
      generator: residentCompiler,
      dillOutputPath: 'lib/foo.dill',
      pathToReload: 'lib/foo.txt.dill',
      trackWidgetCreation: false,
      invalidatedFiles: <Uri>[],
      packageConfig: PackageConfig.empty,
      bundle: FakeBundle(),
      shaderCompiler: const FakeShaderCompiler(),
    );
    expect(report2.success, true);

    final int processingBundleIndex = logger.messages.indexOf('Processing bundle.');
    final int bundleProcessingDoneIndex = logger.messages.indexOf('Bundle processing done.');
    final int compileLibMainIndex = logger.messages.indexWhere(
      (String element) => element.startsWith('<- recompile lib/main.dart '),
    );
    expect(processingBundleIndex, greaterThanOrEqualTo(0));
    expect(bundleProcessingDoneIndex, greaterThanOrEqualTo(0));
    expect(compileLibMainIndex, greaterThanOrEqualTo(0));
    expect(bundleProcessingDoneIndex, greaterThan(compileLibMainIndex));
  });

  group('Shader compilation', () {
    testWithoutContext('DevFS recompiles shaders', () async {
      final MemoryFileSystem fileSystem = MemoryFileSystem.test();
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[createDevFSRequest],
        httpAddress: Uri.parse('http://localhost'),
      );
      final BufferLogger logger = BufferLogger.test();
      final DevFS devFS = DevFS(
        fakeVmServiceHost.vmService,
        'test',
        fileSystem.currentDirectory,
        fileSystem: fileSystem,
        logger: logger,
        osUtils: FakeOperatingSystemUtils(),
        httpClient: FakeHttpClient.any(),
        config: Config.test(),
        processManager: FakeProcessManager.empty(),
        artifacts: Artifacts.test(),
        buildMode: BuildMode.debug,
      );

      await devFS.create();

      final FakeResidentCompiler residentCompiler =
          FakeResidentCompiler()
            ..onRecompile = (Uri mainUri, List<Uri>? invalidatedFiles) async {
              fileSystem.file('lib/foo.dill')
                ..createSync(recursive: true)
                ..writeAsBytesSync(<int>[1, 2, 3, 4, 5]);
              return const CompilerOutput('lib/foo.dill', 0, <Uri>[]);
            };
      final FakeBundle bundle =
          FakeBundle()
            ..entries['foo.frag'] = AssetBundleEntry(
              DevFSByteContent(<int>[1, 2, 3, 4]),
              kind: AssetKind.shader,
              transformers: const <AssetTransformerEntry>[],
            )
            ..entries['not.frag'] = AssetBundleEntry(
              DevFSByteContent(<int>[1, 2, 3, 4]),
              kind: AssetKind.regular,
              transformers: const <AssetTransformerEntry>[],
            );

      final UpdateFSReport report = await devFS.update(
        mainUri: Uri.parse('lib/main.dart'),
        generator: residentCompiler,
        dillOutputPath: 'lib/foo.dill',
        pathToReload: 'lib/foo.txt.dill',
        trackWidgetCreation: false,
        invalidatedFiles: <Uri>[],
        packageConfig: PackageConfig.empty,
        shaderCompiler: const FakeShaderCompiler(),
        bundle: bundle,
      );

      expect(report.success, true);
      expect(devFS.shaderPathsToEvict, <String>{'foo.frag'});
      expect(devFS.assetPathsToEvict, <String>{'not.frag'});
    });

    testWithoutContext('DevFS tracks when FontManifest is updated', () async {
      final MemoryFileSystem fileSystem = MemoryFileSystem.test();
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[createDevFSRequest],
        httpAddress: Uri.parse('http://localhost'),
      );
      final BufferLogger logger = BufferLogger.test();
      final DevFS devFS = DevFS(
        fakeVmServiceHost.vmService,
        'test',
        fileSystem.currentDirectory,
        fileSystem: fileSystem,
        logger: logger,
        osUtils: FakeOperatingSystemUtils(),
        httpClient: FakeHttpClient.any(),
        config: Config.test(),
        processManager: FakeProcessManager.empty(),
        artifacts: Artifacts.test(),
        buildMode: BuildMode.debug,
      );

      await devFS.create();

      expect(devFS.didUpdateFontManifest, false);

      final FakeResidentCompiler residentCompiler =
          FakeResidentCompiler()
            ..onRecompile = (Uri mainUri, List<Uri>? invalidatedFiles) async {
              fileSystem.file('lib/foo.dill')
                ..createSync(recursive: true)
                ..writeAsBytesSync(<int>[1, 2, 3, 4, 5]);
              return const CompilerOutput('lib/foo.dill', 0, <Uri>[]);
            };
      final FakeBundle bundle =
          FakeBundle()
            ..entries['FontManifest.json'] = AssetBundleEntry(
              DevFSByteContent(<int>[1, 2, 3, 4]),
              kind: AssetKind.regular,
              transformers: const <AssetTransformerEntry>[],
            );

      final UpdateFSReport report = await devFS.update(
        mainUri: Uri.parse('lib/main.dart'),
        generator: residentCompiler,
        dillOutputPath: 'lib/foo.dill',
        pathToReload: 'lib/foo.txt.dill',
        trackWidgetCreation: false,
        invalidatedFiles: <Uri>[],
        packageConfig: PackageConfig.empty,
        shaderCompiler: const FakeShaderCompiler(),
        bundle: bundle,
      );

      expect(report.success, true);
      expect(devFS.shaderPathsToEvict, <String>{});
      expect(devFS.assetPathsToEvict, <String>{'FontManifest.json'});
      expect(devFS.didUpdateFontManifest, true);
    });
  });

  group('Asset transformation', () {
    testWithoutContext('DevFS re-transforms assets with transformers during update', () async {
      final MemoryFileSystem fileSystem = MemoryFileSystem.test();
      final Artifacts artifacts = Artifacts.test();
      final FakeDevFSWriter devFSWriter = FakeDevFSWriter();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <Pattern>[
            artifacts.getArtifactPath(Artifact.engineDartBinary),
            'run',
            'increment',
            '--input=/.tmp_rand0/rand0/retransformerInput-asset.txt-transformOutput0.txt',
            '--output=/.tmp_rand0/rand0/retransformerInput-asset.txt-transformOutput1.txt',
          ],
          onRun: (List<String> command) {
            final ArgResults argParseResults = (ArgParser()
                  ..addOption('input', mandatory: true)
                  ..addOption('output', mandatory: true))
                .parse(command);

            final File inputFile = fileSystem.file(argParseResults['input']);
            final File outputFile = fileSystem.file(argParseResults['output']);

            expect(inputFile, exists);
            outputFile
              ..createSync()
              ..writeAsBytesSync(
                Uint8List.fromList(inputFile.readAsBytesSync().map((int b) => b + 1).toList()),
              );
          },
        ),
      ]);

      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[createDevFSRequest],
        httpAddress: Uri.parse('http://localhost'),
      );
      final BufferLogger logger = BufferLogger.test();
      final DevFS devFS = DevFS(
        fakeVmServiceHost.vmService,
        'test',
        fileSystem.currentDirectory,
        fileSystem: fileSystem,
        logger: logger,
        osUtils: FakeOperatingSystemUtils(),
        httpClient: FakeHttpClient.any(),
        config: Config.test(),
        processManager: processManager,
        artifacts: artifacts,
        buildMode: BuildMode.debug,
      );

      await devFS.create();

      final FakeResidentCompiler residentCompiler =
          FakeResidentCompiler()
            ..onRecompile = (Uri mainUri, List<Uri>? invalidatedFiles) async {
              fileSystem.file('lib/foo.dill')
                ..createSync(recursive: true)
                ..writeAsBytesSync(<int>[1, 2, 3, 4, 5]);
              return const CompilerOutput('lib/foo.dill', 0, <Uri>[]);
            };

      final FakeBundle bundle =
          FakeBundle()
            ..entries['asset.txt'] = AssetBundleEntry(
              DevFSByteContent(<int>[1, 2, 3, 4]),
              kind: AssetKind.regular,
              transformers: const <AssetTransformerEntry>[
                AssetTransformerEntry(package: 'increment', args: <String>[]),
              ],
            );

      final UpdateFSReport report = await devFS.update(
        mainUri: Uri.parse('lib/main.dart'),
        generator: residentCompiler,
        dillOutputPath: 'lib/foo.dill',
        pathToReload: 'lib/foo.txt.dill',
        trackWidgetCreation: false,
        invalidatedFiles: <Uri>[],
        packageConfig: PackageConfig.empty,
        devFSWriter: devFSWriter,
        shaderCompiler: const FakeShaderCompiler(),
        bundle: bundle,
      );

      expect(processManager, hasNoRemainingExpectations);
      expect(report.success, true);
      expect(devFSWriter.entries, isNotNull);
      final Uri assetUri = Uri(path: 'build/flutter_assets/asset.txt');
      expect(devFSWriter.entries, contains(assetUri));
      expect(
        await devFSWriter.entries![assetUri]!.contentsAsBytes(),
        containsAllInOrder(<int>[2, 3, 4, 5]),
      );
    });

    testWithoutContext('DevFS reports failure when asset transformation fails', () async {
      final MemoryFileSystem fileSystem = MemoryFileSystem.test();
      final Artifacts artifacts = Artifacts.test();
      final FakeDevFSWriter devFSWriter = FakeDevFSWriter();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <Pattern>[
            artifacts.getArtifactPath(Artifact.engineDartBinary),
            'run',
            'increment',
            '--input=/.tmp_rand0/rand0/retransformerInput-asset.txt-transformOutput0.txt',
            '--output=/.tmp_rand0/rand0/retransformerInput-asset.txt-transformOutput1.txt',
          ],
          exitCode: 1,
        ),
      ]);

      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[createDevFSRequest],
        httpAddress: Uri.parse('http://localhost'),
      );
      final BufferLogger logger = BufferLogger.test();
      final DevFS devFS = DevFS(
        fakeVmServiceHost.vmService,
        'test',
        fileSystem.currentDirectory,
        fileSystem: fileSystem,
        logger: logger,
        osUtils: FakeOperatingSystemUtils(),
        httpClient: FakeHttpClient.any(),
        config: Config.test(),
        processManager: processManager,
        artifacts: artifacts,
        buildMode: BuildMode.debug,
      );

      await devFS.create();

      final FakeResidentCompiler residentCompiler =
          FakeResidentCompiler()
            ..onRecompile = (Uri mainUri, List<Uri>? invalidatedFiles) async {
              fileSystem.file('lib/foo.dill')
                ..createSync(recursive: true)
                ..writeAsBytesSync(<int>[1, 2, 3, 4, 5]);
              return const CompilerOutput('lib/foo.dill', 0, <Uri>[]);
            };

      final FakeBundle bundle =
          FakeBundle()
            ..entries['asset.txt'] = AssetBundleEntry(
              DevFSByteContent(<int>[1, 2, 3, 4]),
              kind: AssetKind.regular,
              transformers: const <AssetTransformerEntry>[
                AssetTransformerEntry(package: 'increment', args: <String>[]),
              ],
            );

      final UpdateFSReport report = await devFS.update(
        mainUri: Uri.parse('lib/main.dart'),
        generator: residentCompiler,
        dillOutputPath: 'lib/foo.dill',
        pathToReload: 'lib/foo.txt.dill',
        trackWidgetCreation: false,
        invalidatedFiles: <Uri>[],
        packageConfig: PackageConfig.empty,
        devFSWriter: devFSWriter,
        shaderCompiler: const FakeShaderCompiler(),
        bundle: bundle,
      );

      expect(processManager, hasNoRemainingExpectations);
      expect(
        report.success,
        false,
        reason: 'DevFS update should fail since asset transformation failed.',
      );
      expect(
        devFSWriter.entries,
        isNull,
        reason: 'DevFS should not have written anything since the update failed.',
      );
      expect(
        logger.errorText,
        'Transformer process terminated with non-zero exit code: 1\n'
        'Transformer package: increment\n'
        'Full command: Artifact.engineDartBinary run increment --input=/.tmp_rand0/rand0/retransformerInput-asset.txt-transformOutput0.txt --output=/.tmp_rand0/rand0/retransformerInput-asset.txt-transformOutput1.txt\n'
        'stdout:\n'
        '\n'
        'stderr:\n'
        '\n',
      );
    });
  });
}

class FakeResidentCompiler extends Fake implements ResidentCompiler {
  Future<CompilerOutput> Function(Uri mainUri, List<Uri>? invalidatedFiles)? onRecompile;

  @override
  Future<CompilerOutput> recompile(
    Uri mainUri,
    List<Uri>? invalidatedFiles, {
    String? outputPath,
    PackageConfig? packageConfig,
    String? projectRootPath,
    FileSystem? fs,
    bool suppressErrors = false,
    bool checkDartPluginRegistry = false,
    File? dartPluginRegistrant,
    Uri? nativeAssetsYaml,
    bool recompileRestart = false,
  }) {
    return onRecompile?.call(mainUri, invalidatedFiles) ??
        Future<CompilerOutput>.value(const CompilerOutput('', 1, <Uri>[]));
  }
}

class FakeDevFSWriter implements DevFSWriter {
  bool written = false;
  Map<Uri, DevFSContent>? entries;

  @override
  Future<void> write(Map<Uri, DevFSContent> entries, Uri baseUri, DevFSWriter parent) async {
    written = true;
    this.entries = entries;
  }
}

class FakeBundle extends AssetBundle {
  @override
  List<File> get additionalDependencies => <File>[];

  @override
  Future<int> build({
    String manifestPath = defaultManifestPath,
    String? assetDirPath,
    String? packageConfigPath,
    bool deferredComponentsEnabled = false,
    TargetPlatform? targetPlatform,
    String? flavor,
  }) async {
    return 0;
  }

  @override
  Map<String, Map<String, AssetBundleEntry>> get deferredComponentsEntries =>
      <String, Map<String, AssetBundleEntry>>{};

  @override
  final Map<String, AssetBundleEntry> entries = <String, AssetBundleEntry>{};

  @override
  List<File> get inputFiles => <File>[];

  @override
  bool needsBuild({String manifestPath = defaultManifestPath}) {
    return true;
  }

  @override
  bool wasBuiltOnce() {
    return false;
  }
}

class AnsweringFakeProcessManager implements ProcessManager {
  AnsweringFakeProcessManager(this.stdout, this.stderr, this.stdin);

  final Stream<List<int>> stdout;
  final Stream<List<int>> stderr;
  final IOSink stdin;

  @override
  bool canRun(dynamic executable, {String? workingDirectory}) {
    return true;
  }

  @override
  bool killPid(int pid, [io.ProcessSignal signal = io.ProcessSignal.sigterm]) {
    return true;
  }

  @override
  Future<ProcessResult> run(
    List<Object> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    Encoding? stdoutEncoding = systemEncoding,
    Encoding? stderrEncoding = systemEncoding,
  }) async {
    throw UnimplementedError();
  }

  @override
  ProcessResult runSync(
    List<Object> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    Encoding? stdoutEncoding = systemEncoding,
    Encoding? stderrEncoding = systemEncoding,
  }) {
    throw UnimplementedError();
  }

  @override
  Future<Process> start(
    List<Object> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    ProcessStartMode mode = ProcessStartMode.normal,
  }) async {
    return AnsweringFakeProcess(stdout, stderr, stdin);
  }
}

class AnsweringFakeProcess implements io.Process {
  AnsweringFakeProcess(this.stdout, this.stderr, this.stdin);

  @override
  final Stream<List<int>> stdout;
  @override
  final Stream<List<int>> stderr;
  @override
  final IOSink stdin;

  @override
  Future<int> get exitCode async => 0;

  @override
  bool kill([io.ProcessSignal signal = io.ProcessSignal.sigterm]) {
    return true;
  }

  @override
  int get pid => 42;
}

class FakeShaderCompiler implements DevelopmentShaderCompiler {
  const FakeShaderCompiler();

  @override
  void configureCompiler(TargetPlatform? platform) {}

  @override
  Future<DevFSContent> recompileShader(DevFSContent inputShader) async {
    return DevFSByteContent(await inputShader.contentsAsBytes());
  }
}
