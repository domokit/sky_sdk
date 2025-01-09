// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:package_config/package_config.dart';
import 'package:process/process.dart';
import 'package:usage/uuid/uuid.dart';

import 'artifacts.dart';
import 'base/common.dart';
import 'base/file_system.dart';
import 'base/io.dart';
import 'base/logger.dart';
import 'base/platform.dart';
import 'build_info.dart';
import 'convert.dart';

/// Opt-in changes to the dart compilers.
const List<String> kDartCompilerExperiments = <String>[];

/// The target model describes the set of core libraries that are available within
/// the SDK.
class TargetModel {
  /// Parse a [TargetModel] from a raw string.
  ///
  /// Throws an exception if passed a value other than 'flutter',
  /// 'flutter_runner', 'vm', or 'dartdevc'.
  factory TargetModel(String rawValue) {
    return switch (rawValue) {
      'flutter' => flutter,
      'flutter_runner' => flutterRunner,
      'vm' => vm,
      'dartdevc' => dartdevc,
      _ => throw Exception('Unexpected target model $rawValue'),
    };
  }

  const TargetModel._(this._value);

  /// The Flutter patched Dart SDK.
  static const TargetModel flutter = TargetModel._('flutter');

  /// The Fuchsia patched SDK.
  static const TargetModel flutterRunner = TargetModel._('flutter_runner');

  /// The Dart VM.
  static const TargetModel vm = TargetModel._('vm');

  /// The development compiler for JavaScript.
  static const TargetModel dartdevc = TargetModel._('dartdevc');

  final String _value;

  @override
  String toString() => _value;
}

class CompilerOutput {
  const CompilerOutput(
    this.outputFilename,
    this.errorCount,
    this.sources, {
    this.expressionData,
    this.errorMessage,
  });

  final String outputFilename;
  final int errorCount;
  final List<Uri> sources;

  /// This field is only non-null for expression compilation requests.
  final Uint8List? expressionData;

  /// This field is only non-null when a compilation error was encountered.
  final String? errorMessage;
}

enum StdoutState { CollectDiagnostic, CollectDependencies }

/// Handles stdin/stdout communication with the frontend server.
class StdoutHandler {
  StdoutHandler({required Logger logger, required FileSystem fileSystem})
    : _logger = logger,
      _fileSystem = fileSystem {
    reset();
  }

  final Logger _logger;
  final FileSystem _fileSystem;

  String? boundaryKey;
  StdoutState state = StdoutState.CollectDiagnostic;
  Completer<CompilerOutput?>? compilerOutput;
  final List<Uri> sources = <Uri>[];

  bool _suppressCompilerMessages = false;
  bool _expectSources = true;
  bool _readFile = false;
  StringBuffer _errorBuffer = StringBuffer();

  void handler(String message) {
    const String kResultPrefix = 'result ';
    if (boundaryKey == null && message.startsWith(kResultPrefix)) {
      boundaryKey = message.substring(kResultPrefix.length);
      return;
    }
    final String? messageBoundaryKey = boundaryKey;
    if (messageBoundaryKey != null && message.startsWith(messageBoundaryKey)) {
      if (_expectSources) {
        if (state == StdoutState.CollectDiagnostic) {
          state = StdoutState.CollectDependencies;
          return;
        }
      }
      if (message.length <= messageBoundaryKey.length) {
        compilerOutput?.complete();
        return;
      }
      final int spaceDelimiter = message.lastIndexOf(' ');
      final String fileName = message.substring(messageBoundaryKey.length + 1, spaceDelimiter);
      final int errorCount = int.parse(message.substring(spaceDelimiter + 1).trim());
      Uint8List? expressionData;
      if (_readFile) {
        expressionData = _fileSystem.file(fileName).readAsBytesSync();
      }
      final CompilerOutput output = CompilerOutput(
        fileName,
        errorCount,
        sources,
        expressionData: expressionData,
        errorMessage: _errorBuffer.isNotEmpty ? _errorBuffer.toString() : null,
      );
      compilerOutput?.complete(output);
      return;
    }
    switch (state) {
      case StdoutState.CollectDiagnostic when _suppressCompilerMessages:
        _logger.printTrace(message);
        _errorBuffer.writeln(message);
      case StdoutState.CollectDiagnostic:
        _logger.printError(message);
        _errorBuffer.writeln(message);
      case StdoutState.CollectDependencies:
        switch (message[0]) {
          case '+':
            sources.add(Uri.parse(message.substring(1)));
          case '-':
            sources.remove(Uri.parse(message.substring(1)));
          default:
            _logger.printTrace('Unexpected prefix for $message uri - ignoring');
        }
    }
  }

  // This is needed to get ready to process next compilation result output,
  // with its own boundary key and new completer.
  void reset({
    bool suppressCompilerMessages = false,
    bool expectSources = true,
    bool readFile = false,
  }) {
    boundaryKey = null;
    compilerOutput = Completer<CompilerOutput?>();
    _suppressCompilerMessages = suppressCompilerMessages;
    _expectSources = expectSources;
    _readFile = readFile;
    state = StdoutState.CollectDiagnostic;
    _errorBuffer = StringBuffer();
  }
}

/// List the preconfigured build options for a given build mode.
List<String> buildModeOptions(BuildMode mode, List<String> dartDefines) => switch (mode) {
  BuildMode.debug => <String>[
    // These checks allow the CLI to override the value of this define for unit
    // testing the framework.
    if (!dartDefines.any((String define) => define.startsWith('dart.vm.profile')))
      '-Ddart.vm.profile=false',
    if (!dartDefines.any((String define) => define.startsWith('dart.vm.product')))
      '-Ddart.vm.product=false',
    '--enable-asserts',
  ],
  BuildMode.profile => <String>[
    // These checks allow the CLI to override the value of this define for
    // benchmarks with most timeline traces disabled.
    if (!dartDefines.any((String define) => define.startsWith('dart.vm.profile')))
      '-Ddart.vm.profile=true',
    if (!dartDefines.any((String define) => define.startsWith('dart.vm.product')))
      '-Ddart.vm.product=false',
    '--delete-tostring-package-uri=dart:ui',
    '--delete-tostring-package-uri=package:flutter',
    ...kDartCompilerExperiments,
  ],
  BuildMode.release => <String>[
    '-Ddart.vm.profile=false',
    '-Ddart.vm.product=true',
    '--delete-tostring-package-uri=dart:ui',
    '--delete-tostring-package-uri=package:flutter',
    ...kDartCompilerExperiments,
  ],
  _ => throw Exception('Unknown BuildMode: $mode'),
};

/// A compiler interface for producing single (non-incremental) kernel files.
class KernelCompiler {
  KernelCompiler({
    required FileSystem fileSystem,
    required Logger logger,
    required ProcessManager processManager,
    required Artifacts artifacts,
    required List<String> fileSystemRoots,
    String? fileSystemScheme,
    @visibleForTesting StdoutHandler? stdoutHandler,
  }) : _logger = logger,
       _fileSystem = fileSystem,
       _artifacts = artifacts,
       _processManager = processManager,
       _fileSystemScheme = fileSystemScheme,
       _fileSystemRoots = fileSystemRoots,
       _stdoutHandler = stdoutHandler ?? StdoutHandler(logger: logger, fileSystem: fileSystem);

  final FileSystem _fileSystem;
  final Artifacts _artifacts;
  final ProcessManager _processManager;
  final Logger _logger;
  final String? _fileSystemScheme;
  final List<String> _fileSystemRoots;
  final StdoutHandler _stdoutHandler;

  Future<CompilerOutput?> compile({
    required String sdkRoot,
    String? mainPath,
    String? outputFilePath,
    String? depFilePath,
    TargetModel targetModel = TargetModel.flutter,
    bool linkPlatformKernelIn = false,
    bool aot = false,
    String? frontendServerStarterPath,
    List<String>? extraFrontEndOptions,
    List<String>? fileSystemRoots,
    String? fileSystemScheme,
    String? initializeFromDill,
    String? platformDill,
    Directory? buildDir,
    String? targetOS,
    bool checkDartPluginRegistry = false,
    required String? packagesPath,
    required BuildMode buildMode,
    required bool trackWidgetCreation,
    required List<String> dartDefines,
    required PackageConfig packageConfig,
    String? nativeAssets,
  }) async {
    final TargetPlatform? platform =
        targetModel == TargetModel.dartdevc ? TargetPlatform.web_javascript : null;
    // This is a URI, not a file path, so the forward slash is correct even on Windows.
    if (!sdkRoot.endsWith('/')) {
      sdkRoot = '$sdkRoot/';
    }
    String? mainUri;
    if (mainPath != null) {
      final File mainFile = _fileSystem.file(mainPath);
      final Uri mainFileUri = mainFile.uri;
      if (packagesPath != null) {
        mainUri = packageConfig.toPackageUri(mainFileUri)?.toString();
      }
      mainUri ??= toMultiRootPath(
        mainFileUri,
        _fileSystemScheme,
        _fileSystemRoots,
        _fileSystem.path.separator == r'\',
      );
    }
    if (outputFilePath != null && !_fileSystem.isFileSync(outputFilePath)) {
      _fileSystem.file(outputFilePath).createSync(recursive: true);
    }

    // Check if there's a Dart plugin registrant.
    // This is contained in the file `dart_plugin_registrant.dart` under `.dart_tools/flutter_build/`.
    final File? dartPluginRegistrant =
        checkDartPluginRegistry ? buildDir?.parent.childFile('dart_plugin_registrant.dart') : null;

    String? dartPluginRegistrantUri;
    if (dartPluginRegistrant != null && dartPluginRegistrant.existsSync()) {
      final Uri dartPluginRegistrantFileUri = dartPluginRegistrant.uri;
      dartPluginRegistrantUri =
          packageConfig.toPackageUri(dartPluginRegistrantFileUri)?.toString() ??
          toMultiRootPath(
            dartPluginRegistrantFileUri,
            _fileSystemScheme,
            _fileSystemRoots,
            _fileSystem.path.separator == r'\',
          );
    }

    final List<String> commandToStartFrontendServer;
    if (frontendServerStarterPath != null && frontendServerStarterPath.isNotEmpty) {
      final String engineDartPath = _artifacts.getArtifactPath(
        Artifact.engineDartBinary,
        platform: platform,
      );
      if (!_processManager.canRun(engineDartPath)) {
        throwToolExit('Unable to find Dart binary at $engineDartPath');
      }
      commandToStartFrontendServer = <String>[engineDartPath, frontendServerStarterPath];
    } else {
      final String engineDartAotRuntimePath = _artifacts.getArtifactPath(
        Artifact.engineDartAotRuntime,
        platform: platform,
      );
      if (!_processManager.canRun(engineDartAotRuntimePath)) {
        throwToolExit('Unable to find dartaotruntime binary at $engineDartAotRuntimePath');
      }
      commandToStartFrontendServer = <String>[
        engineDartAotRuntimePath,
        _artifacts.getArtifactPath(
          Artifact.frontendServerSnapshotForEngineDartSdk,
          platform: platform,
        ),
      ];
    }

    final List<String> command =
        commandToStartFrontendServer +
        <String>[
          '--sdk-root',
          sdkRoot,
          '--target=$targetModel',
          '--no-print-incremental-dependencies',
          for (final Object dartDefine in dartDefines) '-D$dartDefine',
          ...buildModeOptions(buildMode, dartDefines),
          if (trackWidgetCreation) '--track-widget-creation',
          if (!linkPlatformKernelIn) '--no-link-platform',
          if (aot) ...<String>[
            '--aot',
            '--tfa',
            // The --target-os flag only makes sense for whole program compilation.
            if (targetOS != null) ...<String>['--target-os', targetOS],
          ],
          if (packagesPath != null) ...<String>['--packages', packagesPath],
          if (outputFilePath != null) ...<String>['--output-dill', outputFilePath],
          if (depFilePath != null &&
              (fileSystemRoots == null || fileSystemRoots.isEmpty)) ...<String>[
            '--depfile',
            depFilePath,
          ],
          if (fileSystemRoots != null)
            for (final String root in fileSystemRoots) ...<String>['--filesystem-root', root],
          if (fileSystemScheme != null) ...<String>['--filesystem-scheme', fileSystemScheme],
          if (initializeFromDill != null) ...<String>[
            '--incremental',
            '--initialize-from-dill',
            initializeFromDill,
          ],
          if (platformDill != null) ...<String>['--platform', platformDill],
          if (dartPluginRegistrantUri != null) ...<String>[
            '--source',
            dartPluginRegistrantUri,
            '--source',
            'package:flutter/src/dart_plugin_registrant.dart',
            '-Dflutter.dart_plugin_registrant=$dartPluginRegistrantUri',
          ],
          if (nativeAssets != null) ...<String>['--native-assets', nativeAssets],
          // See: https://github.com/flutter/flutter/issues/103994
          '--verbosity=error',
          ...?extraFrontEndOptions,
          if (mainUri != null) mainUri else '--native-assets-only',
        ];

    _logger.printTrace(command.join(' '));
    final Process server = await _processManager.start(command);

    server.stderr.transform<String>(utf8.decoder).listen(_logger.printError);
    server.stdout
        .transform<String>(utf8.decoder)
        .transform<String>(const LineSplitter())
        .listen(_stdoutHandler.handler);
    final int exitCode = await server.exitCode;
    if (exitCode == 0) {
      return _stdoutHandler.compilerOutput?.future;
    }
    return null;
  }
}

/// Class that allows to serialize compilation requests to the compiler.
abstract class _CompilationRequest {
  _CompilationRequest(this.completer);

  Completer<CompilerOutput?> completer;

  Future<CompilerOutput?> _run(DefaultResidentCompiler compiler);

  Future<void> run(DefaultResidentCompiler compiler) async {
    completer.complete(await _run(compiler));
  }
}

class _RecompileRequest extends _CompilationRequest {
  _RecompileRequest(
    super.completer,
    this.mainUri,
    this.invalidatedFiles,
    this.outputPath,
    this.packageConfig,
    this.suppressErrors, {
    this.additionalSourceUri,
    this.nativeAssetsYamlUri,
  });

  Uri mainUri;
  List<Uri>? invalidatedFiles;
  String outputPath;
  PackageConfig packageConfig;
  bool suppressErrors;
  final Uri? additionalSourceUri;
  final Uri? nativeAssetsYamlUri;

  @override
  Future<CompilerOutput?> _run(DefaultResidentCompiler compiler) async => compiler._recompile(this);
}

class _CompileExpressionRequest extends _CompilationRequest {
  _CompileExpressionRequest(
    super.completer,
    this.expression,
    this.definitions,
    this.definitionTypes,
    this.typeDefinitions,
    this.typeBounds,
    this.typeDefaults,
    this.libraryUri,
    this.klass,
    this.method,
    this.isStatic,
  );

  String expression;
  List<String>? definitions;
  List<String>? definitionTypes;
  List<String>? typeDefinitions;
  List<String>? typeBounds;
  List<String>? typeDefaults;
  String? libraryUri;
  String? klass;
  String? method;
  bool isStatic;

  @override
  Future<CompilerOutput?> _run(DefaultResidentCompiler compiler) async =>
      compiler._compileExpression(this);
}

class _CompileExpressionToJsRequest extends _CompilationRequest {
  _CompileExpressionToJsRequest(
    super.completer,
    this.libraryUri,
    this.line,
    this.column,
    this.jsModules,
    this.jsFrameValues,
    this.moduleName,
    this.expression,
  );

  final String? libraryUri;
  final int line;
  final int column;
  final Map<String, String>? jsModules;
  final Map<String, String>? jsFrameValues;
  final String? moduleName;
  final String? expression;

  @override
  Future<CompilerOutput?> _run(DefaultResidentCompiler compiler) async =>
      compiler._compileExpressionToJs(this);
}

class _RejectRequest extends _CompilationRequest {
  _RejectRequest(super.completer);

  @override
  Future<CompilerOutput?> _run(DefaultResidentCompiler compiler) async => compiler._reject();
}

/// Wrapper around incremental frontend server compiler, that communicates with
/// server via stdin/stdout.
///
/// The wrapper is intended to stay resident in memory as user changes, reloads,
/// restarts the Flutter app.
abstract class ResidentCompiler {
  factory ResidentCompiler(
    String sdkRoot, {
    required BuildMode buildMode,
    required Logger logger,
    required ProcessManager processManager,
    required Artifacts artifacts,
    required Platform platform,
    required FileSystem fileSystem,
    bool testCompilation,
    bool trackWidgetCreation,
    String packagesPath,
    List<String> fileSystemRoots,
    String? fileSystemScheme,
    String initializeFromDill,
    bool assumeInitializeFromDillUpToDate,
    TargetModel targetModel,
    bool unsafePackageSerialization,
    String? frontendServerStarterPath,
    List<String> extraFrontEndOptions,
    String platformDill,
    List<String>? dartDefines,
    String librariesSpec,
  }) = DefaultResidentCompiler;

  // TODO(zanderso): find a better way to configure additional file system
  // roots from the runner.
  // See: https://github.com/flutter/flutter/issues/50494
  void addFileSystemRoot(String root);

  /// If invoked for the first time, it compiles Dart script identified by
  /// [mainPath], [invalidatedFiles] list is ignored.
  /// On successive runs [invalidatedFiles] indicates which files need to be
  /// recompiled. If [mainPath] is [null], previously used [mainPath] entry
  /// point that is used for recompilation.
  /// Binary file name is returned if compilation was successful, otherwise
  /// null is returned.
  ///
  /// If [checkDartPluginRegistry] is true, it is the caller's responsibility
  /// to ensure that the generated registrant file has been updated such that
  /// it is wrapping [mainUri].
  Future<CompilerOutput?> recompile(
    Uri mainUri,
    List<Uri>? invalidatedFiles, {
    required String outputPath,
    required PackageConfig packageConfig,
    required FileSystem fs,
    String? projectRootPath,
    bool suppressErrors = false,
    bool checkDartPluginRegistry = false,
    File? dartPluginRegistrant,
    Uri? nativeAssetsYaml,
  });

  Future<CompilerOutput?> compileExpression(
    String expression,
    List<String>? definitions,
    List<String>? definitionTypes,
    List<String>? typeDefinitions,
    List<String>? typeBounds,
    List<String>? typeDefaults,
    String? libraryUri,
    String? klass,
    String? method,
    bool isStatic,
  );

  /// Compiles [expression] in [libraryUri] at [line]:[column] to JavaScript
  /// in [moduleName].
  ///
  /// Values listed in [jsFrameValues] are substituted for their names in the
  /// [expression].
  ///
  /// Ensures that all [jsModules] are loaded and accessible inside the
  /// expression.
  ///
  /// Example values of parameters:
  /// [moduleName] is of the form '/packages/hello_world_main.dart'
  /// [jsFrameValues] is a map from js variable name to its primitive value
  /// or another variable name, for example
  /// { 'x': '1', 'y': 'y', 'o': 'null' }
  /// [jsModules] is a map from variable name to the module name, where
  /// variable name is the name originally used in JavaScript to contain the
  /// module object, for example:
  /// { 'dart':'dart_sdk', 'main': '/packages/hello_world_main.dart' }
  /// Returns a [CompilerOutput] including the name of the file containing the
  /// compilation result and a number of errors.
  Future<CompilerOutput?> compileExpressionToJs(
    String libraryUri,
    int line,
    int column,
    Map<String, String> jsModules,
    Map<String, String> jsFrameValues,
    String moduleName,
    String expression,
  );

  /// Should be invoked when results of compilation are accepted by the client.
  ///
  /// Either [accept] or [reject] should be called after every [recompile] call.
  void accept();

  /// Should be invoked when results of compilation are rejected by the client.
  ///
  /// Either [accept] or [reject] should be called after every [recompile] call.
  Future<CompilerOutput?> reject();

  /// Should be invoked when frontend server compiler should forget what was
  /// accepted previously so that next call to [recompile] produces complete
  /// kernel file.
  void reset();

  Future<Object> shutdown();
}

@visibleForTesting
class DefaultResidentCompiler implements ResidentCompiler {
  DefaultResidentCompiler(
    String sdkRoot, {
    required this.buildMode,
    required Logger logger,
    required ProcessManager processManager,
    required this.artifacts,
    required Platform platform,
    required FileSystem fileSystem,
    this.testCompilation = false,
    this.trackWidgetCreation = true,
    this.packagesPath,
    List<String> fileSystemRoots = const <String>[],
    this.fileSystemScheme,
    this.initializeFromDill,
    this.assumeInitializeFromDillUpToDate = false,
    this.targetModel = TargetModel.flutter,
    this.unsafePackageSerialization = false,
    this.frontendServerStarterPath,
    this.extraFrontEndOptions,
    this.platformDill,
    List<String>? dartDefines,
    this.librariesSpec,
    @visibleForTesting StdoutHandler? stdoutHandler,
  }) : _logger = logger,
       _processManager = processManager,
       _stdoutHandler = stdoutHandler ?? StdoutHandler(logger: logger, fileSystem: fileSystem),
       _platform = platform,
       dartDefines = dartDefines ?? const <String>[],
       // This is a URI, not a file path, so the forward slash is correct even on Windows.
       sdkRoot = sdkRoot.endsWith('/') ? sdkRoot : '$sdkRoot/',
       // Make a copy, we might need to modify it later.
       fileSystemRoots = List<String>.from(fileSystemRoots);

  final Logger _logger;
  final ProcessManager _processManager;
  final Artifacts artifacts;
  final Platform _platform;

  final bool testCompilation;
  final BuildMode buildMode;
  final bool trackWidgetCreation;
  final String? packagesPath;
  final TargetModel targetModel;
  final List<String> fileSystemRoots;
  final String? fileSystemScheme;
  final String? initializeFromDill;
  final bool assumeInitializeFromDillUpToDate;
  final bool unsafePackageSerialization;
  final String? frontendServerStarterPath;
  final List<String>? extraFrontEndOptions;
  final List<String> dartDefines;
  final String? librariesSpec;

  @override
  void addFileSystemRoot(String root) {
    fileSystemRoots.add(root);
  }

  /// The path to the root of the Dart SDK used to compile.
  ///
  /// This is used to resolve the [platformDill].
  final String sdkRoot;

  /// The path to the platform dill file.
  ///
  /// This does not need to be provided for the normal Flutter workflow.
  final String? platformDill;

  Process? _server;
  final StdoutHandler _stdoutHandler;
  bool _compileRequestNeedsConfirmation = false;

  final StreamController<_CompilationRequest> _controller = StreamController<_CompilationRequest>();

  @override
  Future<CompilerOutput?> recompile(
    Uri mainUri,
    List<Uri>? invalidatedFiles, {
    required String outputPath,
    required PackageConfig packageConfig,
    bool suppressErrors = false,
    bool checkDartPluginRegistry = false,
    File? dartPluginRegistrant,
    String? projectRootPath,
    FileSystem? fs,
    Uri? nativeAssetsYaml,
  }) async {
    if (!_controller.hasListener) {
      _controller.stream.listen(_handleCompilationRequest);
    }
    Uri? additionalSourceUri;
    // `dart_plugin_registrant.dart` contains the Dart plugin registry.
    if (checkDartPluginRegistry &&
        dartPluginRegistrant != null &&
        dartPluginRegistrant.existsSync()) {
      additionalSourceUri = dartPluginRegistrant.uri;
    }
    final Completer<CompilerOutput?> completer = Completer<CompilerOutput?>();
    _controller.add(
      _RecompileRequest(
        completer,
        mainUri,
        invalidatedFiles,
        outputPath,
        packageConfig,
        suppressErrors,
        additionalSourceUri: additionalSourceUri,
        nativeAssetsYamlUri: nativeAssetsYaml,
      ),
    );
    return completer.future;
  }

  Future<CompilerOutput?> _recompile(_RecompileRequest request) async {
    _stdoutHandler.reset();
    _compileRequestNeedsConfirmation = true;
    _stdoutHandler._suppressCompilerMessages = request.suppressErrors;

    final String mainUri =
        request.packageConfig.toPackageUri(request.mainUri)?.toString() ??
        toMultiRootPath(request.mainUri, fileSystemScheme, fileSystemRoots, _platform.isWindows);

    String? additionalSourceUri;
    if (request.additionalSourceUri != null) {
      additionalSourceUri =
          request.packageConfig.toPackageUri(request.additionalSourceUri!)?.toString() ??
          toMultiRootPath(
            request.additionalSourceUri!,
            fileSystemScheme,
            fileSystemRoots,
            _platform.isWindows,
          );
    }

    final String? nativeAssets = request.nativeAssetsYamlUri?.toString();
    final Process? server = _server;
    if (server == null) {
      return _compile(
        mainUri,
        request.outputPath,
        additionalSourceUri: additionalSourceUri,
        nativeAssetsUri: nativeAssets,
      );
    }
    final String inputKey = Uuid().generateV4();

    if (nativeAssets != null && nativeAssets.isNotEmpty) {
      server.stdin.writeln('native-assets $nativeAssets');
      _logger.printTrace('<- native-assets $nativeAssets');
    }
    server.stdin.writeln('recompile $mainUri $inputKey');
    _logger.printTrace('<- recompile $mainUri $inputKey');
    final List<Uri>? invalidatedFiles = request.invalidatedFiles;
    if (invalidatedFiles != null) {
      for (final Uri fileUri in invalidatedFiles) {
        String message;
        if (fileUri.scheme == 'package') {
          message = fileUri.toString();
        } else {
          message =
              request.packageConfig.toPackageUri(fileUri)?.toString() ??
              toMultiRootPath(fileUri, fileSystemScheme, fileSystemRoots, _platform.isWindows);
        }
        server.stdin.writeln(message);
        _logger.printTrace(message);
      }
    }
    server.stdin.writeln(inputKey);
    _logger.printTrace('<- $inputKey');

    return _stdoutHandler.compilerOutput?.future;
  }

  final List<_CompilationRequest> _compilationQueue = <_CompilationRequest>[];

  Future<void> _handleCompilationRequest(_CompilationRequest request) async {
    final bool isEmpty = _compilationQueue.isEmpty;
    _compilationQueue.add(request);
    // Only trigger processing if queue was empty - i.e. no other requests
    // are currently being processed. This effectively enforces "one
    // compilation request at a time".
    if (isEmpty) {
      while (_compilationQueue.isNotEmpty) {
        final _CompilationRequest request = _compilationQueue.first;
        await request.run(this);
        _compilationQueue.removeAt(0);
      }
    }
  }

  Future<CompilerOutput?> _compile(
    String scriptUri,
    String? outputPath, {
    String? additionalSourceUri,
    String? nativeAssetsUri,
  }) async {
    final TargetPlatform? platform =
        (targetModel == TargetModel.dartdevc) ? TargetPlatform.web_javascript : null;
    late final List<String> commandToStartFrontendServer;
    if (frontendServerStarterPath != null && frontendServerStarterPath!.isNotEmpty) {
      commandToStartFrontendServer = <String>[
        artifacts.getArtifactPath(Artifact.engineDartBinary, platform: platform),
        frontendServerStarterPath!,
      ];
    } else {
      commandToStartFrontendServer = <String>[
        artifacts.getArtifactPath(Artifact.engineDartAotRuntime, platform: platform),
        artifacts.getArtifactPath(
          Artifact.frontendServerSnapshotForEngineDartSdk,
          platform: platform,
        ),
      ];
    }

    final List<String> command =
        commandToStartFrontendServer +
        <String>[
          '--sdk-root',
          sdkRoot,
          '--incremental',
          if (testCompilation) '--no-print-incremental-dependencies',
          '--target=$targetModel',
          // TODO(annagrin): remove once this becomes the default behavior
          // in the frontend_server.
          // https://github.com/flutter/flutter/issues/59902
          '--experimental-emit-debug-metadata',
          for (final Object dartDefine in dartDefines) '-D$dartDefine',
          if (outputPath != null) ...<String>['--output-dill', outputPath],
          // If we have a platform dill, we don't need to pass the libraries spec,
          // since the information is embedded in the .dill file.
          if (librariesSpec != null && platformDill == null) ...<String>[
            '--libraries-spec',
            librariesSpec!,
          ],
          if (packagesPath != null) ...<String>['--packages', packagesPath!],
          ...buildModeOptions(buildMode, dartDefines),
          if (trackWidgetCreation) '--track-widget-creation',
          for (final String root in fileSystemRoots) ...<String>['--filesystem-root', root],
          if (fileSystemScheme != null) ...<String>['--filesystem-scheme', fileSystemScheme!],
          if (initializeFromDill != null) ...<String>[
            '--initialize-from-dill',
            initializeFromDill!,
          ],
          if (assumeInitializeFromDillUpToDate) '--assume-initialize-from-dill-up-to-date',
          if (additionalSourceUri != null) ...<String>[
            '--source',
            additionalSourceUri,
            '--source',
            'package:flutter/src/dart_plugin_registrant.dart',
            '-Dflutter.dart_plugin_registrant=$additionalSourceUri',
          ],
          if (nativeAssetsUri != null) ...<String>['--native-assets', nativeAssetsUri],
          if (platformDill != null) ...<String>['--platform', platformDill!],
          if (unsafePackageSerialization) '--unsafe-package-serialization',
          // See: https://github.com/flutter/flutter/issues/103994
          '--verbosity=error',
          ...?extraFrontEndOptions,
        ];
    _logger.printTrace(command.join(' '));
    _server = await _processManager.start(command);
    _server?.stdout
        .transform<String>(utf8.decoder)
        .transform<String>(const LineSplitter())
        .listen(
          _stdoutHandler.handler,
          onDone: () {
            // when outputFilename future is not completed, but stdout is closed
            // process has died unexpectedly.
            if (_stdoutHandler.compilerOutput?.isCompleted == false) {
              _stdoutHandler.compilerOutput?.complete();
              throwToolExit('the Dart compiler exited unexpectedly.');
            }
          },
        );

    _server?.stderr
        .transform<String>(utf8.decoder)
        .transform<String>(const LineSplitter())
        .listen(_logger.printError);

    unawaited(
      _server?.exitCode.then((int code) {
        if (code != 0) {
          throwToolExit('the Dart compiler exited unexpectedly.');
        }
      }),
    );

    if (nativeAssetsUri != null && nativeAssetsUri.isNotEmpty) {
      _server?.stdin.writeln('native-assets $nativeAssetsUri');
      _logger.printTrace('<- native-assets $nativeAssetsUri');
    }

    _server?.stdin.writeln('compile $scriptUri');
    _logger.printTrace('<- compile $scriptUri');

    return _stdoutHandler.compilerOutput?.future;
  }

  @override
  Future<CompilerOutput?> compileExpression(
    String expression,
    List<String>? definitions,
    List<String>? definitionTypes,
    List<String>? typeDefinitions,
    List<String>? typeBounds,
    List<String>? typeDefaults,
    String? libraryUri,
    String? klass,
    String? method,
    bool isStatic,
  ) async {
    if (!_controller.hasListener) {
      _controller.stream.listen(_handleCompilationRequest);
    }

    final Completer<CompilerOutput?> completer = Completer<CompilerOutput?>();
    final _CompileExpressionRequest request = _CompileExpressionRequest(
      completer,
      expression,
      definitions,
      definitionTypes,
      typeDefinitions,
      typeBounds,
      typeDefaults,
      libraryUri,
      klass,
      method,
      isStatic,
    );
    _controller.add(request);
    return completer.future;
  }

  Future<CompilerOutput?> _compileExpression(_CompileExpressionRequest request) async {
    _stdoutHandler.reset(suppressCompilerMessages: true, expectSources: false, readFile: true);

    // 'compile-expression' should be invoked after compiler has been started,
    // program was compiled.
    final Process? server = _server;
    if (server == null) {
      return null;
    }

    final String inputKey = Uuid().generateV4();
    server.stdin
      ..writeln('compile-expression $inputKey')
      ..writeln(request.expression);
    request.definitions?.forEach(server.stdin.writeln);
    server.stdin.writeln(inputKey);
    request.definitionTypes?.forEach(server.stdin.writeln);
    server.stdin.writeln(inputKey);
    request.typeDefinitions?.forEach(server.stdin.writeln);
    server.stdin.writeln(inputKey);
    request.typeBounds?.forEach(server.stdin.writeln);
    server.stdin.writeln(inputKey);
    request.typeDefaults?.forEach(server.stdin.writeln);
    server.stdin
      ..writeln(inputKey)
      ..writeln(request.libraryUri ?? '')
      ..writeln(request.klass ?? '')
      ..writeln(request.method ?? '')
      ..writeln(request.isStatic);

    return _stdoutHandler.compilerOutput?.future;
  }

  @override
  Future<CompilerOutput?> compileExpressionToJs(
    String libraryUri,
    int line,
    int column,
    Map<String, String> jsModules,
    Map<String, String> jsFrameValues,
    String moduleName,
    String expression,
  ) {
    if (!_controller.hasListener) {
      _controller.stream.listen(_handleCompilationRequest);
    }

    final Completer<CompilerOutput?> completer = Completer<CompilerOutput?>();
    _controller.add(
      _CompileExpressionToJsRequest(
        completer,
        libraryUri,
        line,
        column,
        jsModules,
        jsFrameValues,
        moduleName,
        expression,
      ),
    );
    return completer.future;
  }

  Future<CompilerOutput?> _compileExpressionToJs(_CompileExpressionToJsRequest request) async {
    _stdoutHandler.reset(suppressCompilerMessages: true, expectSources: false);

    // 'compile-expression-to-js' should be invoked after compiler has been started,
    // program was compiled.
    final Process? server = _server;
    if (server == null) {
      return null;
    }

    final String inputKey = Uuid().generateV4();
    server.stdin
      ..writeln('compile-expression-to-js $inputKey')
      ..writeln(request.libraryUri ?? '')
      ..writeln(request.line)
      ..writeln(request.column);
    request.jsModules?.forEach((String k, String v) {
      server.stdin.writeln('$k:$v');
    });
    server.stdin.writeln(inputKey);
    request.jsFrameValues?.forEach((String k, String v) {
      server.stdin.writeln('$k:$v');
    });
    server.stdin
      ..writeln(inputKey)
      ..writeln(request.moduleName ?? '')
      ..writeln(request.expression ?? '');

    return _stdoutHandler.compilerOutput?.future;
  }

  @override
  void accept() {
    if (_compileRequestNeedsConfirmation) {
      _server?.stdin.writeln('accept');
      _logger.printTrace('<- accept');
    }
    _compileRequestNeedsConfirmation = false;
  }

  @override
  Future<CompilerOutput?> reject() {
    if (!_controller.hasListener) {
      _controller.stream.listen(_handleCompilationRequest);
    }

    final Completer<CompilerOutput?> completer = Completer<CompilerOutput?>();
    _controller.add(_RejectRequest(completer));
    return completer.future;
  }

  Future<CompilerOutput?> _reject() async {
    if (!_compileRequestNeedsConfirmation) {
      return Future<CompilerOutput?>.value();
    }
    _stdoutHandler.reset(expectSources: false);
    _server?.stdin.writeln('reject');
    _logger.printTrace('<- reject');
    _compileRequestNeedsConfirmation = false;
    return _stdoutHandler.compilerOutput?.future;
  }

  @override
  void reset() {
    _server?.stdin.writeln('reset');
    _logger.printTrace('<- reset');
  }

  @override
  Future<Object> shutdown() async {
    // Server was never successfully created.
    final Process? server = _server;
    if (server == null) {
      return 0;
    }
    _logger.printTrace('killing pid ${server.pid}');
    server.kill();
    return server.exitCode;
  }
}

/// Convert a file URI into a multi-root scheme URI if provided, otherwise
/// return unmodified.
@visibleForTesting
String toMultiRootPath(Uri fileUri, String? scheme, List<String> fileSystemRoots, bool windows) {
  if (scheme == null || fileSystemRoots.isEmpty || fileUri.scheme != 'file') {
    return fileUri.toString();
  }
  final String filePath = fileUri.toFilePath(windows: windows);
  for (final String fileSystemRoot in fileSystemRoots) {
    if (filePath.startsWith(fileSystemRoot)) {
      return '$scheme://${filePath.substring(fileSystemRoot.length)}';
    }
  }
  return fileUri.toString();
}
