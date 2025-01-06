// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:package_config/package_config.dart';

import '../artifacts.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../build_info.dart';
import '../cache.dart';
import '../compile.dart';
import '../convert.dart';
import '../device.dart';
import '../globals.dart' as globals;
import '../native_assets.dart';
import '../project.dart';
import '../web/chrome.dart';
import '../web/memory_fs.dart';
import 'flutter_platform.dart' as loader;
import 'flutter_web_platform.dart';
import 'font_config_manager.dart';
import 'test_config.dart';
import 'test_time_recorder.dart';
import 'test_wrapper.dart';
import 'watcher.dart';
import 'web_test_compiler.dart';

/// A class that abstracts launching the test process from the test runner.
abstract class FlutterTestRunner {
  const factory FlutterTestRunner() = _FlutterTestRunnerImpl;

  /// Runs tests using package:test and the Flutter engine.
  Future<int> runTests(
    TestWrapper testWrapper,
    List<Uri> testFiles, {
    required DebuggingOptions debuggingOptions,
    List<String> names = const <String>[],
    List<String> plainNames = const <String>[],
    String? tags,
    String? excludeTags,
    bool enableVmService = false,
    bool machine = false,
    String? precompiledDillPath,
    Map<String, String>? precompiledDillFiles,
    bool updateGoldens = false,
    TestWatcher? watcher,
    required int? concurrency,
    String? testAssetDirectory,
    FlutterProject? flutterProject,
    String? icudtlPath,
    Directory? coverageDirectory,
    bool web = false,
    String? randomSeed,
    String? reporter,
    String? fileReporter,
    String? timeout,
    bool failFast = false,
    bool runSkipped = false,
    int? shardIndex,
    int? totalShards,
    Device? integrationTestDevice,
    String? integrationTestUserIdentifier,
    TestTimeRecorder? testTimeRecorder,
    TestCompilerNativeAssetsBuilder? nativeAssetsBuilder,
    required BuildInfo buildInfo,
  });

  /// Runs tests using the experimental strategy of spawning each test in a
  /// separate lightweight Engine.
  Future<int> runTestsBySpawningLightweightEngines(
    List<Uri> testFiles, {
    required DebuggingOptions debuggingOptions,
    List<String> names = const <String>[],
    List<String> plainNames = const <String>[],
    String? tags,
    String? excludeTags,
    bool machine = false,
    bool updateGoldens = false,
    required int? concurrency,
    String? testAssetDirectory,
    FlutterProject? flutterProject,
    String? icudtlPath,
    String? randomSeed,
    String? reporter,
    String? fileReporter,
    String? timeout,
    bool failFast = false,
    bool runSkipped = false,
    int? shardIndex,
    int? totalShards,
    TestTimeRecorder? testTimeRecorder,
    TestCompilerNativeAssetsBuilder? nativeAssetsBuilder,
  });
}

class _FlutterTestRunnerImpl implements FlutterTestRunner {
  const _FlutterTestRunnerImpl();

  @override
  Future<int> runTests(
    TestWrapper testWrapper,
    List<Uri> testFiles, {
    required DebuggingOptions debuggingOptions,
    List<String> names = const <String>[],
    List<String> plainNames = const <String>[],
    String? tags,
    String? excludeTags,
    bool enableVmService = false,
    bool machine = false,
    String? precompiledDillPath,
    Map<String, String>? precompiledDillFiles,
    bool updateGoldens = false,
    TestWatcher? watcher,
    required int? concurrency,
    String? testAssetDirectory,
    FlutterProject? flutterProject,
    String? icudtlPath,
    Directory? coverageDirectory,
    bool web = false,
    String? randomSeed,
    String? reporter,
    String? fileReporter,
    String? timeout,
    bool failFast = false,
    bool runSkipped = false,
    int? shardIndex,
    int? totalShards,
    Device? integrationTestDevice,
    String? integrationTestUserIdentifier,
    TestTimeRecorder? testTimeRecorder,
    TestCompilerNativeAssetsBuilder? nativeAssetsBuilder,
    required BuildInfo buildInfo,
  }) async {
    // Configure package:test to use the Flutter engine for child processes.
    final String flutterTesterBinPath = globals.artifacts!.getArtifactPath(Artifact.flutterTester);

    // Compute the command-line arguments for package:test.
    final List<String> testArgs = <String>[
      if (!globals.terminal.supportsColor) '--no-color',
      if (debuggingOptions.startPaused) '--pause-after-load',
      if (machine) ...<String>['-r', 'json'] else if (reporter != null) ...<String>['-r', reporter],
      if (fileReporter != null) '--file-reporter=$fileReporter',
      if (timeout != null) ...<String>['--timeout', timeout],
      if (concurrency != null) '--concurrency=$concurrency',
      for (final String name in names) ...<String>['--name', name],
      for (final String plainName in plainNames) ...<String>['--plain-name', plainName],
      if (randomSeed != null) '--test-randomize-ordering-seed=$randomSeed',
      if (tags != null) ...<String>['--tags', tags],
      if (excludeTags != null) ...<String>['--exclude-tags', excludeTags],
      if (failFast) '--fail-fast',
      if (runSkipped) '--run-skipped',
      if (totalShards != null) '--total-shards=$totalShards',
      if (shardIndex != null) '--shard-index=$shardIndex',
      '--chain-stack-traces',
    ];

    if (web) {
      final String tempBuildDir =
          globals.fs.systemTempDirectory.createTempSync('flutter_test.').absolute.uri.toFilePath();
      final WebMemoryFS result = await WebTestCompiler(
        logger: globals.logger,
        fileSystem: globals.fs,
        platform: globals.platform,
        artifacts: globals.artifacts!,
        processManager: globals.processManager,
        config: globals.config,
      ).initialize(
        projectDirectory: flutterProject!.directory,
        testOutputDir: tempBuildDir,
        testFiles: testFiles.map((Uri uri) => uri.toFilePath()).toList(),
        buildInfo: debuggingOptions.buildInfo,
        webRenderer: debuggingOptions.webRenderer,
        useWasm: debuggingOptions.webUseWasm,
      );
      testArgs
        ..add('--platform=chrome')
        ..add('--')
        ..addAll(testFiles.map((Uri uri) => uri.toString()));
      testWrapper.registerPlatformPlugin(<Runtime>[Runtime.chrome], () {
        return FlutterWebPlatform.start(
          flutterProject.directory.path,
          updateGoldens: updateGoldens,
          flutterTesterBinPath: flutterTesterBinPath,
          flutterProject: flutterProject,
          pauseAfterLoad: debuggingOptions.startPaused,
          nullAssertions: debuggingOptions.nullAssertions,
          buildInfo: debuggingOptions.buildInfo,
          webMemoryFS: result,
          logger: globals.logger,
          fileSystem: globals.fs,
          buildDirectory: globals.fs.directory(tempBuildDir),
          artifacts: globals.artifacts,
          processManager: globals.processManager,
          chromiumLauncher: ChromiumLauncher(
            fileSystem: globals.fs,
            platform: globals.platform,
            processManager: globals.processManager,
            operatingSystemUtils: globals.os,
            browserFinder: findChromeExecutable,
            logger: globals.logger,
          ),
          testTimeRecorder: testTimeRecorder,
          webRenderer: debuggingOptions.webRenderer,
          useWasm: debuggingOptions.webUseWasm,
        );
      });
      await testWrapper.main(testArgs);
      return exitCode;
    }

    testArgs
      ..add('--')
      ..addAll(testFiles.map((Uri uri) => uri.toString()));

    final InternetAddressType serverType =
        debuggingOptions.ipv6 ? InternetAddressType.IPv6 : InternetAddressType.IPv4;

    final loader.FlutterPlatform platform = loader.installHook(
      testWrapper: testWrapper,
      shellPath: flutterTesterBinPath,
      debuggingOptions: debuggingOptions,
      watcher: watcher,
      enableVmService: enableVmService,
      machine: machine,
      serverType: serverType,
      precompiledDillPath: precompiledDillPath,
      precompiledDillFiles: precompiledDillFiles,
      updateGoldens: updateGoldens,
      testAssetDirectory: testAssetDirectory,
      projectRootDirectory: globals.fs.currentDirectory.uri,
      flutterProject: flutterProject,
      icudtlPath: icudtlPath,
      integrationTestDevice: integrationTestDevice,
      integrationTestUserIdentifier: integrationTestUserIdentifier,
      testTimeRecorder: testTimeRecorder,
      nativeAssetsBuilder: nativeAssetsBuilder,
      buildInfo: buildInfo,
      fileSystem: globals.fs,
      logger: globals.logger,
      processManager: globals.processManager,
    );

    try {
      globals.printTrace('running test package with arguments: $testArgs');
      await testWrapper.main(testArgs);

      // test.main() sets dart:io's exitCode global.
      globals.printTrace('test package returned with exit code $exitCode');

      return exitCode;
    } finally {
      await platform.close();
    }
  }

  // To compile root_test_isolate_spawner.dart and
  // child_test_isolate_spawner.dart successfully, we will need to pass a
  // package_config.json to the frontend server that contains the
  // union of package:test_core, package:ffi, and all the dependencies of the
  // project under test. This function generates such a package_config.json.
  static Future<void> _generateIsolateSpawningTesterPackageConfig({
    required FlutterProject flutterProject,
    required File isolateSpawningTesterPackageConfigFile,
  }) async {
    final File projectPackageConfigFile = globals.fs
        .directory(flutterProject.directory.path)
        .childDirectory('.dart_tool')
        .childFile('package_config.json');
    final PackageConfig projectPackageConfig = PackageConfig.parseBytes(
      projectPackageConfigFile.readAsBytesSync(),
      projectPackageConfigFile.uri,
    );

    // The flutter_tools package_config.json is guaranteed to include
    // package:ffi and package:test_core.
    final File flutterToolsPackageConfigFile = globals.fs
        .directory(globals.fs.path.join(Cache.flutterRoot!, 'packages', 'flutter_tools'))
        .childDirectory('.dart_tool')
        .childFile('package_config.json');
    final PackageConfig flutterToolsPackageConfig = PackageConfig.parseBytes(
      flutterToolsPackageConfigFile.readAsBytesSync(),
      flutterToolsPackageConfigFile.uri,
    );

    final List<Package> mergedPackages = <Package>[...projectPackageConfig.packages];
    final Set<String> projectPackageNames = Set<String>.from(
      mergedPackages.map((Package p) => p.name),
    );
    for (final Package p in flutterToolsPackageConfig.packages) {
      if (!projectPackageNames.contains(p.name)) {
        mergedPackages.add(p);
      }
    }

    final PackageConfig mergedPackageConfig = PackageConfig(mergedPackages);
    final StringBuffer buffer = StringBuffer();
    PackageConfig.writeString(mergedPackageConfig, buffer);
    isolateSpawningTesterPackageConfigFile.writeAsStringSync(buffer.toString());
  }

  static void _generateChildTestIsolateSpawnerSourceFile(
    List<Uri> paths, {
    required List<String> packageTestArgs,
    required bool autoUpdateGoldenFiles,
    required File childTestIsolateSpawnerSourceFile,
    required File childTestIsolateSpawnerDillFile,
  }) {
    final Map<String, String> testConfigPaths = <String, String>{};

    final StringBuffer buffer = StringBuffer();
    buffer.writeln('''
import 'dart:ffi';
import 'dart:isolate';
import 'dart:ui';

import 'package:ffi/ffi.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:stream_channel/isolate_channel.dart';
import 'package:test_api/backend.dart'; // flutter_ignore: test_api_import
''');

    String pathToImport(String path) {
      assert(path.endsWith('.dart'));
      return path
          .replaceAll('.', '_')
          .replaceAll(':', '_')
          .replaceAll('/', '_')
          .replaceAll(r'\', '_')
          .replaceRange(path.length - '.dart'.length, null, '');
    }

    final Map<String, String> testImports = <String, String>{};
    final Set<String> seenTestConfigPaths = <String>{};
    for (final Uri path in paths) {
      final String sanitizedPath =
          !path.path.endsWith('?') ? path.path : path.path.substring(0, path.path.length - 1);
      final String sanitizedImport = pathToImport(sanitizedPath);
      buffer.writeln("import '$sanitizedPath' as $sanitizedImport;");
      testImports[sanitizedPath] = sanitizedImport;
      final File? testConfigFile = findTestConfigFile(
        globals.fs.file(
          globals.platform.isWindows
              ? sanitizedPath.replaceAll('/', r'\').replaceFirst(r'\', '')
              : sanitizedPath,
        ),
        globals.logger,
      );
      if (testConfigFile != null) {
        final String sanitizedTestConfigImport = pathToImport(testConfigFile.path);
        testConfigPaths[sanitizedImport] = sanitizedTestConfigImport;
        if (seenTestConfigPaths.add(testConfigFile.path)) {
          buffer.writeln(
            "import '${Uri.file(testConfigFile.path, windows: true)}' as $sanitizedTestConfigImport;",
          );
        }
      }
    }
    buffer.writeln();

    buffer.writeln('const List<String> packageTestArgs = <String>[');
    for (final String arg in packageTestArgs) {
      buffer.writeln("  '$arg',");
    }
    buffer.writeln('];');
    buffer.writeln();

    buffer.writeln('const List<String> testPaths = <String>[');
    for (final Uri path in paths) {
      buffer.writeln("  '$path',");
    }
    buffer.writeln('];');
    buffer.writeln();

    buffer.writeln(r'''
@Native<Void Function(Pointer<Utf8>, Pointer<Utf8>)>(symbol: 'Spawn')
external void _spawn(Pointer<Utf8> entrypoint, Pointer<Utf8> route);

void spawn({required SendPort port, String entrypoint = 'main', String route = '/'}) {
  assert(
    entrypoint != 'main' || route != '/',
    'Spawn should not be used to spawn main with the default route name',
  );
  IsolateNameServer.registerPortWithName(port, route);
  _spawn(entrypoint.toNativeUtf8(), route.toNativeUtf8());
}
''');

    buffer.write('''
/// Runs on a spawned isolate.
void createChannelAndConnect(String path, String name, Function testMain) {
  goldenFileComparator = LocalFileComparator(Uri.parse(path));
  autoUpdateGoldenFiles = $autoUpdateGoldenFiles;
  final IsolateChannel<dynamic> channel = IsolateChannel<dynamic>.connectSend(
    IsolateNameServer.lookupPortByName(name)!,
  );
  channel.pipe(RemoteListener.start(() => testMain));
}

@pragma('vm:entry-point')
void testMain() {
  final String route = PlatformDispatcher.instance.defaultRouteName;
  switch (route) {
''');

    for (final MapEntry<String, String> kvp in testImports.entries) {
      final String importName = kvp.value;
      final String path = kvp.key;
      final String? testConfigImport = testConfigPaths[importName];
      if (testConfigImport != null) {
        buffer.writeln("    case '$importName':");
        buffer.writeln(
          "      createChannelAndConnect('$path', route, () => $testConfigImport.testExecutable($importName.main));",
        );
      } else {
        buffer.writeln("    case '$importName':");
        buffer.writeln("      createChannelAndConnect('$path', route, $importName.main);");
      }
    }

    buffer.write(r'''
  }
}

@pragma('vm:entry-point')
void main([dynamic sendPort]) {
  if (sendPort is SendPort) {
    final ReceivePort receivePort = ReceivePort();
    receivePort.listen((dynamic msg) {
      switch (msg as List<dynamic>) {
        case ['spawn', final SendPort port, final String entrypoint, final String route]:
          spawn(port: port, entrypoint: entrypoint, route: route);
        case ['close']:
          receivePort.close();
      }
    });

    sendPort.send(<Object>[receivePort.sendPort, packageTestArgs, testPaths]);
  }
}
''');

    childTestIsolateSpawnerSourceFile.writeAsStringSync(buffer.toString());
  }

  static void _generateRootTestIsolateSpawnerSourceFile({
    required File childTestIsolateSpawnerSourceFile,
    required File childTestIsolateSpawnerDillFile,
    required File rootTestIsolateSpawnerSourceFile,
  }) {
    final StringBuffer buffer = StringBuffer();
    buffer.writeln('''
import 'dart:async';
import 'dart:ffi';
import 'dart:io' show exit, exitCode; // flutter_ignore: dart_io_import
import 'dart:isolate';
import 'dart:ui';

import 'package:ffi/ffi.dart';
import 'package:stream_channel/isolate_channel.dart';
import 'package:stream_channel/stream_channel.dart';
import 'package:test_core/src/executable.dart' as test; // ignore: implementation_imports
import 'package:test_core/src/platform.dart'; // ignore: implementation_imports

@Native<Handle Function(Pointer<Utf8>)>(symbol: 'LoadLibraryFromKernel')
external Object _loadLibraryFromKernel(Pointer<Utf8> path);

@Native<Handle Function(Pointer<Utf8>, Pointer<Utf8>)>(symbol: 'LookupEntryPoint')
external Object _lookupEntryPoint(Pointer<Utf8> library, Pointer<Utf8> name);

late final List<String> packageTestArgs;
late final List<String> testPaths;

/// Runs on the main isolate.
Future<void> registerPluginAndRun() {
  final SpawnPlugin platform = SpawnPlugin();
  registerPlatformPlugin(
    <Runtime>[Runtime.vm],
    () {
      return platform;
    },
  );
  return test.main(<String>[...packageTestArgs, '--', ...testPaths]);
}

late final Isolate rootTestIsolate;
late final SendPort commandPort;
bool readyToRun = false;
final Completer<void> readyToRunSignal = Completer<void>();

Future<void> spawn({
  required SendPort port,
  String entrypoint = 'main',
  String route = '/',
}) async {
  if (!readyToRun) {
    await readyToRunSignal.future;
  }

  commandPort.send(<Object>['spawn', port, entrypoint, route]);
}

@pragma('vm:entry-point')
void main() async {
  final String route = PlatformDispatcher.instance.defaultRouteName;

  if (route == '/') {
    final ReceivePort port = ReceivePort();

    port.listen((dynamic message) {
      final [SendPort sendPort, List<String> args, List<String> paths] = message as List<dynamic>;

      commandPort = sendPort;
      packageTestArgs = args;
      testPaths = paths;
      readyToRun = true;
      readyToRunSignal.complete();
    });

    rootTestIsolate = await Isolate.spawn(
      _loadLibraryFromKernel(
          r'${childTestIsolateSpawnerDillFile.absolute.path}'
              .toNativeUtf8()) as void Function(SendPort),
      port.sendPort,
    );

    await readyToRunSignal.future;
    port.close(); // Not expecting anything else.
    await registerPluginAndRun();
    // The [test.main] call in [registerPluginAndRun] sets dart:io's [exitCode]
    // global.
    exit(exitCode);
  } else {
    (_lookupEntryPoint(
        r'file://${childTestIsolateSpawnerSourceFile.absolute.uri.toFilePath(windows: false)}'
            .toNativeUtf8(),
        'testMain'.toNativeUtf8()) as void Function())();
  }
}
''');

    buffer.write(r'''
String pathToImport(String path) {
  assert(path.endsWith('.dart'));
  return path
      .replaceRange(path.length - '.dart'.length, null, '')
      .replaceAll('.', '_')
      .replaceAll(':', '_')
      .replaceAll('/', '_')
      .replaceAll(r'\', '_');
}

class SpawnPlugin extends PlatformPlugin {
  SpawnPlugin();

  final Map<String, IsolateChannel<dynamic>> _channels = <String, IsolateChannel<dynamic>>{};

  Future<void> launchIsolate(String path) async {
    final String name = pathToImport(path);
    final ReceivePort port = ReceivePort();
    _channels[name] = IsolateChannel<dynamic>.connectReceive(port);
    await spawn(port: port.sendPort, route: name);
  }

  @override
  Future<void> close() async {
    commandPort.send(<String>['close']);
  }
''');

    buffer.write('''
  @override
  Future<RunnerSuite> load(
    String path,
    SuitePlatform platform,
    SuiteConfiguration suiteConfig,
    Object message,
  ) async {
    final String correctedPath = ${globals.platform.isWindows ? r'"/$path"' : 'path'};
    await launchIsolate(correctedPath);

    final StreamChannel<dynamic> channel = _channels[pathToImport(correctedPath)]!;
    final RunnerSuiteController controller = deserializeSuite(correctedPath, platform,
        suiteConfig, const PluginEnvironment(), channel, message);
    return controller.suite;
  }
}
''');

    rootTestIsolateSpawnerSourceFile.writeAsStringSync(buffer.toString());
  }

  static Future<void> _compileFile({
    required DebuggingOptions debuggingOptions,
    required File packageConfigFile,
    required PackageConfig packageConfig,
    required File sourceFile,
    required File outputDillFile,
    required TestTimeRecorder? testTimeRecorder,
    Uri? nativeAssetsYaml,
  }) async {
    globals.printTrace('Compiling ${sourceFile.absolute.uri}');
    final Stopwatch compilerTime = Stopwatch()..start();
    final Stopwatch? testTimeRecorderStopwatch = testTimeRecorder?.start(TestTimePhases.Compile);

    final ResidentCompiler residentCompiler = ResidentCompiler(
      globals.artifacts!.getArtifactPath(Artifact.flutterPatchedSdkPath),
      artifacts: globals.artifacts!,
      logger: globals.logger,
      processManager: globals.processManager,
      buildMode: debuggingOptions.buildInfo.mode,
      trackWidgetCreation: debuggingOptions.buildInfo.trackWidgetCreation,
      dartDefines: debuggingOptions.buildInfo.dartDefines,
      packagesPath: packageConfigFile.path,
      frontendServerStarterPath: debuggingOptions.buildInfo.frontendServerStarterPath,
      extraFrontEndOptions: debuggingOptions.buildInfo.extraFrontEndOptions,
      platform: globals.platform,
      testCompilation: true,
      fileSystem: globals.fs,
      fileSystemRoots: debuggingOptions.buildInfo.fileSystemRoots,
      fileSystemScheme: debuggingOptions.buildInfo.fileSystemScheme,
    );

    await residentCompiler.recompile(
      sourceFile.absolute.uri,
      null,
      outputPath: outputDillFile.absolute.path,
      packageConfig: packageConfig,
      fs: globals.fs,
      nativeAssetsYaml: nativeAssetsYaml,
    );
    residentCompiler.accept();

    globals.printTrace(
      'Compiling ${sourceFile.absolute.uri} took ${compilerTime.elapsedMilliseconds}ms',
    );
    testTimeRecorder?.stop(TestTimePhases.Compile, testTimeRecorderStopwatch!);
  }

  @override
  Future<int> runTestsBySpawningLightweightEngines(
    List<Uri> testFiles, {
    required DebuggingOptions debuggingOptions,
    List<String> names = const <String>[],
    List<String> plainNames = const <String>[],
    String? tags,
    String? excludeTags,
    bool machine = false,
    bool updateGoldens = false,
    required int? concurrency,
    String? testAssetDirectory,
    FlutterProject? flutterProject,
    String? icudtlPath,
    String? randomSeed,
    String? reporter,
    String? fileReporter,
    String? timeout,
    bool failFast = false,
    bool runSkipped = false,
    int? shardIndex,
    int? totalShards,
    TestTimeRecorder? testTimeRecorder,
    TestCompilerNativeAssetsBuilder? nativeAssetsBuilder,
  }) async {
    assert(testFiles.length > 1);

    final Directory buildDirectory = globals.fs.directory(
      globals.fs.path.join(flutterProject!.directory.path, getBuildDirectory()),
    );
    final Directory isolateSpawningTesterDirectory = buildDirectory.childDirectory(
      'isolate_spawning_tester',
    );
    isolateSpawningTesterDirectory.createSync();

    final File isolateSpawningTesterPackageConfigFile = isolateSpawningTesterDirectory
        .childDirectory('.dart_tool')
        .childFile('package_config.json');
    isolateSpawningTesterPackageConfigFile.createSync(recursive: true);
    await _generateIsolateSpawningTesterPackageConfig(
      flutterProject: flutterProject,
      isolateSpawningTesterPackageConfigFile: isolateSpawningTesterPackageConfigFile,
    );
    final PackageConfig isolateSpawningTesterPackageConfig = PackageConfig.parseBytes(
      isolateSpawningTesterPackageConfigFile.readAsBytesSync(),
      isolateSpawningTesterPackageConfigFile.uri,
    );

    final File childTestIsolateSpawnerSourceFile = isolateSpawningTesterDirectory.childFile(
      'child_test_isolate_spawner.dart',
    );
    final File rootTestIsolateSpawnerSourceFile = isolateSpawningTesterDirectory.childFile(
      'root_test_isolate_spawner.dart',
    );
    final File childTestIsolateSpawnerDillFile = isolateSpawningTesterDirectory.childFile(
      'child_test_isolate_spawner.dill',
    );
    final File rootTestIsolateSpawnerDillFile = isolateSpawningTesterDirectory.childFile(
      'root_test_isolate_spawner.dill',
    );

    // Compute the command-line arguments for package:test.
    final List<String> packageTestArgs = <String>[
      if (!globals.terminal.supportsColor) '--no-color',
      if (machine) ...<String>['-r', 'json'] else if (reporter != null) ...<String>['-r', reporter],
      if (fileReporter != null) '--file-reporter=$fileReporter',
      if (timeout != null) ...<String>['--timeout', timeout],
      if (concurrency != null) '--concurrency=$concurrency',
      for (final String name in names) ...<String>['--name', name],
      for (final String plainName in plainNames) ...<String>['--plain-name', plainName],
      if (randomSeed != null) '--test-randomize-ordering-seed=$randomSeed',
      if (tags != null) ...<String>['--tags', tags],
      if (excludeTags != null) ...<String>['--exclude-tags', excludeTags],
      if (failFast) '--fail-fast',
      if (runSkipped) '--run-skipped',
      if (totalShards != null) '--total-shards=$totalShards',
      if (shardIndex != null) '--shard-index=$shardIndex',
      '--chain-stack-traces',
    ];

    _generateChildTestIsolateSpawnerSourceFile(
      testFiles,
      packageTestArgs: packageTestArgs,
      autoUpdateGoldenFiles: updateGoldens,
      childTestIsolateSpawnerSourceFile: childTestIsolateSpawnerSourceFile,
      childTestIsolateSpawnerDillFile: childTestIsolateSpawnerDillFile,
    );

    _generateRootTestIsolateSpawnerSourceFile(
      childTestIsolateSpawnerSourceFile: childTestIsolateSpawnerSourceFile,
      childTestIsolateSpawnerDillFile: childTestIsolateSpawnerDillFile,
      rootTestIsolateSpawnerSourceFile: rootTestIsolateSpawnerSourceFile,
    );

    await _compileFile(
      debuggingOptions: debuggingOptions,
      packageConfigFile: isolateSpawningTesterPackageConfigFile,
      packageConfig: isolateSpawningTesterPackageConfig,
      sourceFile: childTestIsolateSpawnerSourceFile,
      outputDillFile: childTestIsolateSpawnerDillFile,
      testTimeRecorder: testTimeRecorder,
    );

    await _compileFile(
      debuggingOptions: debuggingOptions,
      packageConfigFile: isolateSpawningTesterPackageConfigFile,
      packageConfig: isolateSpawningTesterPackageConfig,
      sourceFile: rootTestIsolateSpawnerSourceFile,
      outputDillFile: rootTestIsolateSpawnerDillFile,
      testTimeRecorder: testTimeRecorder,
    );

    final List<String> command = <String>[
      globals.artifacts!.getArtifactPath(Artifact.flutterTester),
      '--disable-vm-service',
      if (icudtlPath != null) '--icu-data-file-path=$icudtlPath',
      '--enable-checked-mode',
      '--verify-entry-points',
      '--enable-software-rendering',
      '--skia-deterministic-rendering',
      if (debuggingOptions.enableDartProfiling) '--enable-dart-profiling',
      '--non-interactive',
      '--use-test-fonts',
      '--disable-asset-fonts',
      '--packages=${debuggingOptions.buildInfo.packageConfigPath}',
      if (testAssetDirectory != null) '--flutter-assets-dir=$testAssetDirectory',
      if (debuggingOptions.nullAssertions) '--dart-flags=--null_assertions',
      ...debuggingOptions.dartEntrypointArgs,
      rootTestIsolateSpawnerDillFile.absolute.path,
    ];

    // If the FLUTTER_TEST environment variable has been set, then pass it on
    // for package:flutter_test to handle the value.
    //
    // If FLUTTER_TEST has not been set, assume from this context that this
    // call was invoked by the command 'flutter test'.
    final String flutterTest =
        globals.platform.environment.containsKey('FLUTTER_TEST')
            ? globals.platform.environment['FLUTTER_TEST']!
            : 'true';
    final Map<String, String> environment = <String, String>{
      'FLUTTER_TEST': flutterTest,
      'FONTCONFIG_FILE': FontConfigManager().fontConfigFile.path,
      'APP_NAME': flutterProject.manifest.appName,
      if (testAssetDirectory != null) 'UNIT_TEST_ASSETS': testAssetDirectory,
      if (nativeAssetsBuilder != null && globals.platform.isWindows)
        'PATH':
            '${nativeAssetsBuilder.windowsBuildDirectory(flutterProject)};${globals.platform.environment['PATH']}',
    };

    globals.logger.printTrace(
      'Starting flutter_tester process with command=$command, environment=$environment',
    );
    final Stopwatch? testTimeRecorderStopwatch = testTimeRecorder?.start(TestTimePhases.Run);
    final Process process = await globals.processManager.start(command, environment: environment);
    globals.logger.printTrace('Started flutter_tester process at pid ${process.pid}');

    for (final Stream<List<int>> stream in <Stream<List<int>>>[process.stderr, process.stdout]) {
      stream.transform<String>(utf8.decoder).listen(globals.stdio.stdoutWrite);
    }

    return process.exitCode.then((int exitCode) {
      testTimeRecorder?.stop(TestTimePhases.Run, testTimeRecorderStopwatch!);
      globals.logger.printTrace(
        'flutter_tester process at pid ${process.pid} exited with code=$exitCode',
      );
      return exitCode;
    });
  }
}
