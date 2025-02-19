// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:package_config/package_config.dart';
import 'package:package_config/package_config_types.dart';
import 'package:process/process.dart';

import '../artifacts.dart';
import '../base/common.dart';
import '../base/config.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../build_info.dart';
import '../bundle.dart';
import '../cache.dart';
import '../compile.dart';
import '../dart/language_version.dart';
import '../web/bootstrap.dart';
import '../web/compile.dart';
import '../web/memory_fs.dart';
import 'test_config.dart';

/// A web compiler for the test runner.
class WebTestCompiler {
  WebTestCompiler({
    required FileSystem fileSystem,
    required Logger logger,
    required Artifacts artifacts,
    required Platform platform,
    required ProcessManager processManager,
    required Config config,
  }) : _logger = logger,
       _fileSystem = fileSystem,
       _artifacts = artifacts,
       _platform = platform,
       _processManager = processManager,
       _config = config;

  final Logger _logger;
  final FileSystem _fileSystem;
  final Artifacts _artifacts;
  final Platform _platform;
  final ProcessManager _processManager;
  final Config _config;

  Future<File> _generateTestEntrypoint({
    required List<String> testFiles,
    required Directory projectDirectory,
    required Directory outputDirectory,
    required LanguageVersion languageVersion,
  }) async {
    final List<WebTestInfo> testInfos =
        testFiles.map((String testFilePath) {
          final List<String> relativeTestSegments = _fileSystem.path.split(
            _fileSystem.path.relative(
              testFilePath,
              from: projectDirectory.childDirectory('test').path,
            ),
          );

          final File? testConfigFile = findTestConfigFile(_fileSystem.file(testFilePath), _logger);
          String? testConfigPath;
          if (testConfigFile != null) {
            testConfigPath = _fileSystem.path
                .split(
                  _fileSystem.path.relative(
                    testConfigFile.path,
                    from: projectDirectory.childDirectory('test').path,
                  ),
                )
                .join('/');
          }
          return (
            entryPoint: relativeTestSegments.join('/'),
            configFile: testConfigPath,
            goldensUri: Uri.file(testFilePath),
          );
        }).toList();
    return _fileSystem.file(_fileSystem.path.join(outputDirectory.path, 'main.dart'))
      ..createSync(recursive: true)
      ..writeAsStringSync(
        generateTestEntrypoint(testInfos: testInfos, languageVersion: languageVersion),
      );
  }

  Future<WebMemoryFS> initialize({
    required Directory projectDirectory,
    required String testOutputDir,
    required List<String> testFiles,
    required BuildInfo buildInfo,
    required WebRendererMode webRenderer,
    required bool useWasm,
  }) async {
    return useWasm
        ? _compileWasm(
          projectDirectory: projectDirectory,
          testOutputDir: testOutputDir,
          testFiles: testFiles,
          buildInfo: buildInfo,
          webRenderer: webRenderer,
        )
        : _compileJS(
          projectDirectory: projectDirectory,
          testOutputDir: testOutputDir,
          testFiles: testFiles,
          buildInfo: buildInfo,
          webRenderer: webRenderer,
        );
  }

  Future<WebMemoryFS> _compileJS({
    required Directory projectDirectory,
    required String testOutputDir,
    required List<String> testFiles,
    required BuildInfo buildInfo,
    required WebRendererMode webRenderer,
  }) async {
    final LanguageVersion languageVersion = currentLanguageVersion(_fileSystem, Cache.flutterRoot!);
    final String platformDillPath = _fileSystem.path.join(
      _artifacts.getHostArtifact(HostArtifact.webPlatformKernelFolder).path,
      'ddc_outline.dill',
    );

    final Directory outputDirectory = _fileSystem.directory(testOutputDir)
      ..createSync(recursive: true);
    final File testFile = await _generateTestEntrypoint(
      testFiles: testFiles,
      projectDirectory: projectDirectory,
      outputDirectory: outputDirectory,
      languageVersion: languageVersion,
    );

    final String cachedKernelPath = getDefaultCachedKernelPath(
      trackWidgetCreation: buildInfo.trackWidgetCreation,
      dartDefines: buildInfo.dartDefines,
      extraFrontEndOptions: buildInfo.extraFrontEndOptions,
      fileSystem: _fileSystem,
      config: _config,
    );
    final List<String> dartDefines = webRenderer.updateDartDefines(buildInfo.dartDefines);
    final ResidentCompiler residentCompiler = ResidentCompiler(
      _artifacts.getHostArtifact(HostArtifact.flutterWebSdk).path,
      buildMode: buildInfo.mode,
      trackWidgetCreation: buildInfo.trackWidgetCreation,
      fileSystemRoots: <String>[projectDirectory.childDirectory('test').path, testOutputDir],
      // Override the filesystem scheme so that the frontend_server can find
      // the generated entrypoint code.
      fileSystemScheme: 'org-dartlang-app',
      initializeFromDill: cachedKernelPath,
      targetModel: TargetModel.dartdevc,
      extraFrontEndOptions: buildInfo.extraFrontEndOptions,
      platformDill: _fileSystem.file(platformDillPath).absolute.uri.toString(),
      dartDefines: dartDefines,
      librariesSpec:
          _artifacts.getHostArtifact(HostArtifact.flutterWebLibrariesJson).uri.toString(),
      packagesPath: buildInfo.packageConfigPath,
      artifacts: _artifacts,
      processManager: _processManager,
      logger: _logger,
      platform: _platform,
      fileSystem: _fileSystem,
    );

    final CompilerOutput? output = await residentCompiler.recompile(
      Uri.parse('org-dartlang-app:///${testFile.basename}'),
      <Uri>[],
      outputPath: outputDirectory.childFile('out').path,
      packageConfig: buildInfo.packageConfig,
      fs: _fileSystem,
      projectRootPath: projectDirectory.absolute.path,
    );
    if (output == null || output.errorCount > 0) {
      throwToolExit('Failed to compile');
    }
    // Cache the output kernel file to speed up subsequent compiles.
    _fileSystem.file(cachedKernelPath).parent.createSync(recursive: true);
    _fileSystem.file(output.outputFilename).copySync(cachedKernelPath);

    final File codeFile = outputDirectory.childFile('${output.outputFilename}.sources');
    final File manifestFile = outputDirectory.childFile('${output.outputFilename}.json');
    final File sourcemapFile = outputDirectory.childFile('${output.outputFilename}.map');
    final File metadataFile = outputDirectory.childFile('${output.outputFilename}.metadata');

    return WebMemoryFS()..write(codeFile, manifestFile, sourcemapFile, metadataFile);
  }

  Future<WebMemoryFS> _compileWasm({
    required Directory projectDirectory,
    required String testOutputDir,
    required List<String> testFiles,
    required BuildInfo buildInfo,
    required WebRendererMode webRenderer,
  }) async {
    final Directory outputDirectory = _fileSystem.directory(testOutputDir)
      ..createSync(recursive: true);
    final File testFile = await _generateTestEntrypoint(
      testFiles: testFiles,
      projectDirectory: projectDirectory,
      outputDirectory: outputDirectory,
      languageVersion: currentLanguageVersion(_fileSystem, Cache.flutterRoot!),
    );

    final String platformBinariesPath =
        _artifacts.getHostArtifact(HostArtifact.webPlatformKernelFolder).path;
    final String platformFilePath = _fileSystem.path.join(
      platformBinariesPath,
      'dart2wasm_platform.dill',
    );
    final List<String> dartDefines = webRenderer.updateDartDefines(buildInfo.dartDefines);
    final File outputWasmFile = outputDirectory.childFile('main.dart.wasm');

    final List<String> compilationArgs = <String>[
      _artifacts.getArtifactPath(
        Artifact.engineDartBinary,
        platform: TargetPlatform.web_javascript,
      ),
      'compile',
      'wasm',
      '--packages=.dart_tool/package_config.json',
      '--extra-compiler-option=--platform=$platformFilePath',
      '--extra-compiler-option=--multi-root-scheme=org-dartlang-app',
      '--extra-compiler-option=--multi-root=${projectDirectory.childDirectory('test').path}',
      '--extra-compiler-option=--multi-root=${outputDirectory.path}',
      '--extra-compiler-option=--enable-asserts',
      '--extra-compiler-option=--no-inlining',
      if (webRenderer == WebRendererMode.skwasm) ...<String>[
        '--extra-compiler-option=--import-shared-memory',
        '--extra-compiler-option=--shared-memory-max-pages=32768',
      ],
      ...buildInfo.extraFrontEndOptions,
      for (final String dartDefine in dartDefines) '-D$dartDefine',

      '-O0',
      '-o',
      outputWasmFile.path,
      testFile.path, // dartfile
    ];

    final ProcessUtils processUtils = ProcessUtils(
      logger: _logger,
      processManager: _processManager,
    );

    await processUtils.stream(compilationArgs);

    return WebMemoryFS();
  }
}
