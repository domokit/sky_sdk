// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:async/async.dart';
import 'package:http_multi_server/http_multi_server.dart';
import 'package:mime/mime.dart' as mime;
import 'package:package_config/package_config.dart';
import 'package:pool/pool.dart';
import 'package:process/process.dart';
import 'package:shelf/shelf.dart' as shelf;
import 'package:shelf/shelf_io.dart' as shelf_io;
import 'package:shelf_web_socket/shelf_web_socket.dart';
import 'package:stream_channel/stream_channel.dart';
import 'package:test_core/src/platform.dart'; // ignore: implementation_imports
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:webkit_inspection_protocol/webkit_inspection_protocol.dart' hide StackTrace;

import '../artifacts.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../build_info.dart';
import '../cache.dart';
import '../convert.dart';
import '../dart/package_map.dart';
import '../project.dart';
import '../web/bootstrap.dart';
import '../web/chrome.dart';
import '../web/compile.dart';
import '../web/memory_fs.dart';
import 'test_compiler.dart';
import 'test_golden_comparator.dart';
import 'test_time_recorder.dart';

shelf.Handler createDirectoryHandler(Directory directory, {required bool crossOriginIsolated}) {
  final mime.MimeTypeResolver resolver = mime.MimeTypeResolver();
  final FileSystem fileSystem = directory.fileSystem;
  return (shelf.Request request) async {
    String uriPath = request.requestedUri.path;

    // Strip any leading slashes
    if (uriPath.startsWith('/')) {
      uriPath = uriPath.substring(1);
    }
    final String filePath = fileSystem.path.join(directory.path, uriPath);
    final File file = fileSystem.file(filePath);
    if (!file.existsSync()) {
      return shelf.Response.notFound('Not Found');
    }
    final String? contentType = resolver.lookup(file.path);
    final bool needsCrossOriginIsolated = crossOriginIsolated && uriPath.endsWith('.html');
    return shelf.Response.ok(
      file.openRead(),
      headers: <String, String>{
        if (contentType != null) 'Content-Type': contentType,
        if (needsCrossOriginIsolated) ...<String, String>{
          'Cross-Origin-Opener-Policy': 'same-origin',
          'Cross-Origin-Embedder-Policy': 'credentialless',
        },
      },
    );
  };
}

class FlutterWebPlatform extends PlatformPlugin {
  FlutterWebPlatform._(
    this._server,
    this._config,
    this._root, {
    this.nullAssertions,
    required this.updateGoldens,
    required this.buildInfo,
    required this.webMemoryFS,
    required FlutterProject flutterProject,
    required String flutterTesterBinPath,
    required FileSystem fileSystem,
    required Directory buildDirectory,
    required File testDartJs,
    required File testHostDartJs,
    required ChromiumLauncher chromiumLauncher,
    required Logger logger,
    required Artifacts? artifacts,
    required ProcessManager processManager,
    required this.webRenderer,
    required this.useWasm,
    TestTimeRecorder? testTimeRecorder,
  }) : _fileSystem = fileSystem,
       _buildDirectory = buildDirectory,
       _testDartJs = testDartJs,
       _testHostDartJs = testHostDartJs,
       _chromiumLauncher = chromiumLauncher,
       _logger = logger,
       _artifacts = artifacts {
    final shelf.Cascade cascade = shelf.Cascade()
        .add(_webSocketHandler.handler)
        .add(
          createDirectoryHandler(
            fileSystem.directory(
              fileSystem.path.join(Cache.flutterRoot!, 'packages', 'flutter_tools'),
            ),
            crossOriginIsolated: webRenderer == WebRendererMode.skwasm,
          ),
        )
        .add(_handleStaticArtifact)
        .add(_localCanvasKitHandler)
        .add(_goldenFileHandler)
        .add(_wrapperHandler)
        .add(_handleTestRequest)
        .add(
          createDirectoryHandler(
            fileSystem.directory(fileSystem.path.join(fileSystem.currentDirectory.path, 'test')),
            crossOriginIsolated: webRenderer == WebRendererMode.skwasm,
          ),
        )
        .add(_packageFilesHandler);
    _server.mount(cascade.handler);
    _testGoldenComparator = TestGoldenComparator(
      compilerFactory:
          () => TestCompiler(buildInfo, flutterProject, testTimeRecorder: testTimeRecorder),
      flutterTesterBinPath: flutterTesterBinPath,
      fileSystem: _fileSystem,
      logger: _logger,
      processManager: processManager,
      environment: <String, String>{
        // Chrome is the only supported browser currently.
        'FLUTTER_TEST_BROWSER': 'chrome',
        'FLUTTER_WEB_RENDERER': webRenderer.name,
      },
    );
  }

  final WebMemoryFS webMemoryFS;
  final BuildInfo buildInfo;
  final FileSystem _fileSystem;
  final Directory _buildDirectory;
  final File _testDartJs;
  final File _testHostDartJs;
  final ChromiumLauncher _chromiumLauncher;
  final Logger _logger;
  final Artifacts? _artifacts;
  final bool updateGoldens;
  final bool? nullAssertions;
  final OneOffHandler _webSocketHandler = OneOffHandler();
  final AsyncMemoizer<void> _closeMemo = AsyncMemoizer<void>();
  final String _root;
  final WebRendererMode webRenderer;
  final bool useWasm;

  /// Allows only one test suite (typically one test file) to be loaded and run
  /// at any given point in time. Loading more than one file at a time is known
  /// to lead to flaky tests.
  final Pool _suiteLock = Pool(1);

  BrowserManager? _browserManager;
  late TestGoldenComparator _testGoldenComparator;

  static Future<shelf.Server> defaultServerFactory() async {
    return shelf_io.IOServer(await HttpMultiServer.loopback(0));
  }

  static Future<FlutterWebPlatform> start(
    String root, {
    bool updateGoldens = false,
    bool pauseAfterLoad = false,
    bool nullAssertions = false,
    required FlutterProject flutterProject,
    required String flutterTesterBinPath,
    required BuildInfo buildInfo,
    required WebMemoryFS webMemoryFS,
    required FileSystem fileSystem,
    required Directory buildDirectory,
    required Logger logger,
    required ChromiumLauncher chromiumLauncher,
    required Artifacts? artifacts,
    required ProcessManager processManager,
    required WebRendererMode webRenderer,
    required bool useWasm,
    TestTimeRecorder? testTimeRecorder,
    Uri? testPackageUri,
    Future<shelf.Server> Function() serverFactory = defaultServerFactory,
  }) async {
    final shelf.Server server = await serverFactory();
    if (testPackageUri == null) {
      final PackageConfig packageConfig = await currentPackageConfig();
      testPackageUri = packageConfig['test']!.packageUriRoot;
    }
    final File testDartJs = fileSystem.file(
      fileSystem.path.join(testPackageUri.toFilePath(), 'dart.js'),
    );
    final File testHostDartJs = fileSystem.file(
      fileSystem.path.join(
        testPackageUri.toFilePath(),
        'src',
        'runner',
        'browser',
        'static',
        'host.dart.js',
      ),
    );
    return FlutterWebPlatform._(
      server,
      Configuration.current.change(pauseAfterLoad: pauseAfterLoad),
      root,
      flutterProject: flutterProject,
      flutterTesterBinPath: flutterTesterBinPath,
      updateGoldens: updateGoldens,
      buildInfo: buildInfo,
      webMemoryFS: webMemoryFS,
      testDartJs: testDartJs,
      testHostDartJs: testHostDartJs,
      fileSystem: fileSystem,
      buildDirectory: buildDirectory,
      chromiumLauncher: chromiumLauncher,
      artifacts: artifacts,
      logger: logger,
      nullAssertions: nullAssertions,
      processManager: processManager,
      webRenderer: webRenderer,
      useWasm: useWasm,
      testTimeRecorder: testTimeRecorder,
    );
  }

  bool get _closed => _closeMemo.hasRun;

  NullSafetyMode get _nullSafetyMode {
    return buildInfo.nullSafetyMode == NullSafetyMode.sound
        ? NullSafetyMode.sound
        : NullSafetyMode.unsound;
  }

  final Configuration _config;
  final shelf.Server _server;
  Uri get url => _server.url;

  /// The ahem text file.
  File get _ahem => _fileSystem.file(
    _fileSystem.path.join(Cache.flutterRoot!, 'packages', 'flutter_tools', 'static', 'Ahem.ttf'),
  );

  /// The require js binary.
  File get _requireJs => _fileSystem.file(
    _fileSystem.path.join(
      _artifacts!.getArtifactPath(
        Artifact.engineDartSdkPath,
        platform: TargetPlatform.web_javascript,
      ),
      'lib',
      'dev_compiler',
      'amd',
      'require.js',
    ),
  );

  /// The ddc module loader js binary.
  File get _ddcModuleLoaderJs => _fileSystem.file(
    _fileSystem.path.join(
      _artifacts!.getArtifactPath(
        Artifact.engineDartSdkPath,
        platform: TargetPlatform.web_javascript,
      ),
      'lib',
      'dev_compiler',
      'ddc',
      'ddc_module_loader.js',
    ),
  );

  /// The ddc to dart stack trace mapper.
  File get _stackTraceMapper => _fileSystem.file(
    _fileSystem.path.join(
      _artifacts!.getArtifactPath(
        Artifact.engineDartSdkPath,
        platform: TargetPlatform.web_javascript,
      ),
      'lib',
      'dev_compiler',
      'web',
      'dart_stack_trace_mapper.js',
    ),
  );

  File get _flutterJs => _fileSystem.file(
    _fileSystem.path.join(
      _artifacts!.getHostArtifact(HostArtifact.flutterJsDirectory).path,
      'flutter.js',
    ),
  );

  File get _dartSdk {
    final Map<WebRendererMode, Map<NullSafetyMode, HostArtifact>> dartSdkArtifactMap =
        buildInfo.ddcModuleFormat == DdcModuleFormat.ddc
            ? kDdcDartSdkJsArtifactMap
            : kAmdDartSdkJsArtifactMap;
    return _fileSystem.file(
      _artifacts!.getHostArtifact(dartSdkArtifactMap[webRenderer]![_nullSafetyMode]!),
    );
  }

  File get _dartSdkSourcemaps {
    final Map<WebRendererMode, Map<NullSafetyMode, HostArtifact>> dartSdkArtifactMap =
        buildInfo.ddcModuleFormat == DdcModuleFormat.ddc
            ? kDdcDartSdkJsMapArtifactMap
            : kAmdDartSdkJsMapArtifactMap;
    return _fileSystem.file(
      _artifacts!.getHostArtifact(dartSdkArtifactMap[webRenderer]![_nullSafetyMode]!),
    );
  }

  File _canvasKitFile(String relativePath) {
    final String canvasKitPath = _fileSystem.path.join(
      _artifacts!.getHostArtifact(HostArtifact.flutterWebSdk).path,
      'canvaskit',
    );
    final File canvasKitFile = _fileSystem.file(_fileSystem.path.join(canvasKitPath, relativePath));
    return canvasKitFile;
  }

  Future<shelf.Response> _handleTestRequest(shelf.Request request) async {
    if (request.url.path.endsWith('main.dart.browser_test.dart.js')) {
      return shelf.Response.ok(
        generateTestBootstrapFileContents(
          '/main.dart.bootstrap.js',
          'require.js',
          'dart_stack_trace_mapper.js',
        ),
        headers: <String, String>{HttpHeaders.contentTypeHeader: 'text/javascript'},
      );
    }
    if (request.url.path.endsWith('main.dart.bootstrap.js')) {
      return shelf.Response.ok(
        generateMainModule(
          nullAssertions: nullAssertions!,
          nativeNullAssertions: true,
          bootstrapModule: 'main.dart.bootstrap',
          entrypoint: '/main.dart.js',
        ),
        headers: <String, String>{HttpHeaders.contentTypeHeader: 'text/javascript'},
      );
    }
    if (request.url.path.endsWith('.dart.js')) {
      final String path = request.url.path.split('.dart.js')[0];
      return shelf.Response.ok(
        webMemoryFS.files['$path.dart.lib.js'],
        headers: <String, String>{HttpHeaders.contentTypeHeader: 'text/javascript'},
      );
    }
    if (request.url.path.endsWith('.lib.js.map')) {
      return shelf.Response.ok(
        webMemoryFS.sourcemaps[request.url.path],
        headers: <String, String>{HttpHeaders.contentTypeHeader: 'text/plain'},
      );
    }
    return shelf.Response.notFound('');
  }

  Future<shelf.Response> _handleStaticArtifact(shelf.Request request) async {
    if (request.requestedUri.path.contains('require.js')) {
      return shelf.Response.ok(
        _requireJs.openRead(),
        headers: <String, String>{'Content-Type': 'text/javascript'},
      );
    } else if (request.requestedUri.path.contains('ddc_module_loader.js')) {
      return shelf.Response.ok(
        _ddcModuleLoaderJs.openRead(),
        headers: <String, String>{'Content-Type': 'text/javascript'},
      );
    } else if (request.requestedUri.path.contains('ahem.ttf')) {
      return shelf.Response.ok(_ahem.openRead());
    } else if (request.requestedUri.path.contains('dart_sdk.js')) {
      return shelf.Response.ok(
        _dartSdk.openRead(),
        headers: <String, String>{'Content-Type': 'text/javascript'},
      );
    } else if (request.requestedUri.path.contains('dart_sdk.js.map')) {
      return shelf.Response.ok(
        _dartSdkSourcemaps.openRead(),
        headers: <String, String>{'Content-Type': 'text/javascript'},
      );
    } else if (request.requestedUri.path.contains('dart_stack_trace_mapper.js')) {
      return shelf.Response.ok(
        _stackTraceMapper.openRead(),
        headers: <String, String>{'Content-Type': 'text/javascript'},
      );
    } else if (request.requestedUri.path.contains('static/dart.js')) {
      return shelf.Response.ok(
        _testDartJs.openRead(),
        headers: <String, String>{'Content-Type': 'text/javascript'},
      );
    } else if (request.requestedUri.path.contains('host.dart.js')) {
      return shelf.Response.ok(
        _testHostDartJs.openRead(),
        headers: <String, String>{'Content-Type': 'text/javascript'},
      );
    } else if (request.requestedUri.path.contains('flutter.js')) {
      return shelf.Response.ok(
        _flutterJs.openRead(),
        headers: <String, String>{'Content-Type': 'text/javascript'},
      );
    } else if (request.requestedUri.path.contains('main.dart.mjs')) {
      return shelf.Response.ok(
        _buildDirectory.childFile('main.dart.mjs').openRead(),
        headers: <String, String>{'Content-Type': 'text/javascript'},
      );
    } else if (request.requestedUri.path.contains('main.dart.wasm')) {
      return shelf.Response.ok(
        _buildDirectory.childFile('main.dart.wasm').openRead(),
        headers: <String, String>{'Content-Type': 'application/wasm'},
      );
    } else {
      return shelf.Response.notFound('Not Found');
    }
  }

  FutureOr<shelf.Response> _packageFilesHandler(shelf.Request request) async {
    if (request.requestedUri.pathSegments.first == 'packages') {
      final Uri? fileUri = buildInfo.packageConfig.resolve(
        Uri(scheme: 'package', pathSegments: request.requestedUri.pathSegments.skip(1)),
      );
      if (fileUri != null) {
        final String dirname = _fileSystem.path.dirname(fileUri.toFilePath());
        final String basename = _fileSystem.path.basename(fileUri.toFilePath());
        final shelf.Handler handler = createDirectoryHandler(
          _fileSystem.directory(dirname),
          crossOriginIsolated: webRenderer == WebRendererMode.skwasm,
        );
        final shelf.Request modifiedRequest = shelf.Request(
          request.method,
          request.requestedUri.replace(path: basename),
          protocolVersion: request.protocolVersion,
          headers: request.headers,
          handlerPath: request.handlerPath,
          url: request.url.replace(path: basename),
          encoding: request.encoding,
          context: request.context,
        );
        return handler(modifiedRequest);
      }
    }
    return shelf.Response.notFound('Not Found');
  }

  Future<shelf.Response> _goldenFileHandler(shelf.Request request) async {
    if (request.url.path.contains('flutter_goldens')) {
      final Map<String, Object?> body =
          json.decode(await request.readAsString()) as Map<String, Object?>;
      final Uri goldenKey = Uri.parse(body['key']! as String);
      final Uri testUri = Uri.parse(body['testUri']! as String);
      final num? width = body['width'] as num?;
      final num? height = body['height'] as num?;
      Uint8List bytes;

      if (body.containsKey('bytes')) {
        bytes = base64.decode(body['bytes']! as String);
      } else {
        // TODO(hterkelsen): Do not use browser screenshots for testing on the
        // web once we transition off the HTML renderer. See:
        // https://github.com/flutter/flutter/issues/135700
        try {
          final ChromeTab chromeTab =
              (await getChromeTabGuarded(_browserManager!._browser.chromeConnection, (
                ChromeTab tab,
              ) {
                return tab.url.contains(_browserManager!._browser.url!);
              }))!;
          final WipConnection connection = await chromeTab.connect();
          final WipResponse response = await connection.sendCommand(
            'Page.captureScreenshot',
            <String, Object>{
              // Clip the screenshot to include only the element.
              // Prior to taking a screenshot, we are calling `window.render()` in
              // `_matchers_web.dart` to only render the element on screen. That
              // will make sure that the element will always be displayed on the
              // origin of the screen.
              'clip': <String, Object>{
                'x': 0.0,
                'y': 0.0,
                'width': width!.toDouble(),
                'height': height!.toDouble(),
                'scale': 1.0,
              },
            },
          );
          bytes = base64.decode(response.result!['data'] as String);
        } on WipError catch (ex) {
          _logger.printError('Caught WIPError: $ex');
          return shelf.Response.ok('WIP error: $ex');
        } on FormatException catch (ex) {
          _logger.printError('Caught FormatException: $ex');
          return shelf.Response.ok('Caught exception: $ex');
        }
      }
      if (updateGoldens) {
        return switch (await _testGoldenComparator.update(testUri, bytes, goldenKey)) {
          TestGoldenUpdateDone() => shelf.Response.ok('true'),
          TestGoldenUpdateError(error: final String error) => shelf.Response.ok(error),
        };
      } else {
        return switch (await _testGoldenComparator.compare(testUri, bytes, goldenKey)) {
          TestGoldenComparisonDone(matched: final bool matched) => shelf.Response.ok('$matched'),
          TestGoldenComparisonError(error: final String error) => shelf.Response.ok(error),
        };
      }
    } else {
      return shelf.Response.notFound('Not Found');
    }
  }

  /// Serves a local build of CanvasKit, replacing the CDN build, which can
  /// cause test flakiness due to reliance on network.
  shelf.Response _localCanvasKitHandler(shelf.Request request) {
    final String fullPath = _fileSystem.path.fromUri(request.url);
    if (!fullPath.startsWith('canvaskit/')) {
      return shelf.Response.notFound('Not a CanvasKit file request');
    }

    final String relativePath = fullPath.replaceFirst('canvaskit/', '');
    final String extension = _fileSystem.path.extension(relativePath);
    String contentType;
    switch (extension) {
      case '.js':
        contentType = 'text/javascript';
      case '.wasm':
        contentType = 'application/wasm';
      default:
        final String error = 'Failed to determine Content-Type for "${request.url.path}".';
        _logger.printError(error);
        return shelf.Response.internalServerError(body: error);
    }

    final File canvasKitFile = _canvasKitFile(relativePath);
    return shelf.Response.ok(
      canvasKitFile.openRead(),
      headers: <String, Object>{HttpHeaders.contentTypeHeader: contentType},
    );
  }

  String _makeBuildConfigString() {
    return useWasm
        ? '''
      {
        compileTarget: "dart2wasm",
        renderer: "${webRenderer.name}",
        mainWasmPath: "main.dart.wasm",
        jsSupportRuntimePath: "main.dart.mjs",
      }
'''
        : '''
      {
        compileTarget: "dartdevc",
        renderer: "${webRenderer.name}",
        mainJsPath: "main.dart.browser_test.dart.js",
      }
''';
  }

  // A handler that serves wrapper files used to bootstrap tests.
  shelf.Response _wrapperHandler(shelf.Request request) {
    final String path = _fileSystem.path.fromUri(request.url);
    if (path.endsWith('.html')) {
      final String test = '${_fileSystem.path.withoutExtension(path)}.dart';
      return shelf.Response.ok(
        '''
        <!DOCTYPE html>
        <html>
        <head>
          <title>${htmlEscape.convert(test)} Test</title>
          <script src="flutter.js"></script>
          <script>
            _flutter.buildConfig = {
              builds: [
                ${_makeBuildConfigString()}
              ]
            }
            window.testSelector = "$test";
            _flutter.loader.load({
              config: {
                canvasKitBaseUrl: "/canvaskit/",
              }
            });
          </script>
        </head>
        </html>
      ''',
        headers: <String, String>{
          'Content-Type': 'text/html',
          if (webRenderer == WebRendererMode.skwasm) ...<String, String>{
            'Cross-Origin-Opener-Policy': 'same-origin',
            'Cross-Origin-Embedder-Policy': 'credentialless',
          },
        },
      );
    }
    return shelf.Response.notFound('Not found.');
  }

  @override
  Future<RunnerSuite> load(
    String path,
    SuitePlatform platform,
    SuiteConfiguration suiteConfig,
    Object message,
  ) async {
    if (_closed) {
      throw StateError('Load called on a closed FlutterWebPlatform');
    }

    final String pathFromTest = _fileSystem.path.relative(
      path,
      from: _fileSystem.path.join(_root, 'test'),
    );
    final Uri suiteUrl = url.resolveUri(
      _fileSystem.path.toUri('${_fileSystem.path.withoutExtension(pathFromTest)}.html'),
    );
    final String relativePath = _fileSystem.path.relative(
      _fileSystem.path.normalize(path),
      from: _fileSystem.currentDirectory.path,
    );
    if (_logger.isVerbose) {
      _logger.printTrace('Loading test suite $relativePath.');
    }

    final PoolResource lockResource = await _suiteLock.request();

    final Runtime browser = platform.runtime;
    try {
      _browserManager = await _launchBrowser(browser);
    } on Error catch (_) {
      await _suiteLock.close();
      rethrow;
    }

    if (_closed) {
      throw StateError('Load called on a closed FlutterWebPlatform');
    }

    if (_logger.isVerbose) {
      _logger.printTrace('Running test suite $relativePath.');
    }

    final RunnerSuite suite = await _browserManager!.load(
      relativePath,
      suiteUrl,
      suiteConfig,
      message,
      onDone: () async {
        await _browserManager!.close();
        _browserManager = null;
        lockResource.release();
        if (_logger.isVerbose) {
          _logger.printTrace('Test suite $relativePath finished.');
        }
      },
    );

    if (_closed) {
      throw StateError('Load called on a closed FlutterWebPlatform');
    }

    return suite;
  }

  /// Returns the [BrowserManager] for [runtime], which should be a browser.
  ///
  /// If no browser manager is running yet, starts one.
  Future<BrowserManager> _launchBrowser(Runtime browser) {
    if (_browserManager != null) {
      throw StateError('Another browser is currently running.');
    }

    final Completer<WebSocketChannel> completer = Completer<WebSocketChannel>.sync();
    final String path = _webSocketHandler.create(webSocketHandler(completer.complete));
    final Uri webSocketUrl = url.replace(scheme: 'ws').resolve(path);
    final Uri hostUrl = url
        .resolve('static/index.html')
        .replace(
          queryParameters: <String, String>{
            'managerUrl': webSocketUrl.toString(),
            'debug': _config.pauseAfterLoad.toString(),
          },
        );

    _logger.printTrace('Serving tests at $hostUrl');

    return BrowserManager.start(
      _chromiumLauncher,
      browser,
      hostUrl,
      completer.future,
      headless: !_config.pauseAfterLoad,
    );
  }

  @override
  Future<void> closeEphemeral() async {
    if (_browserManager != null) {
      await _browserManager!.close();
    }
  }

  @override
  Future<void> close() => _closeMemo.runOnce(() async {
    await Future.wait<void>(<Future<dynamic>>[
      if (_browserManager != null) _browserManager!.close(),
      _server.close(),
      _testGoldenComparator.close(),
    ]);
  });
}

class OneOffHandler {
  /// A map from URL paths to handlers.
  final Map<String, shelf.Handler> _handlers = <String, shelf.Handler>{};

  /// The counter of handlers that have been activated.
  int _counter = 0;

  /// The actual [shelf.Handler] that dispatches requests.
  shelf.Handler get handler => _onRequest;

  /// Creates a new one-off handler that forwards to [handler].
  ///
  /// Returns a string that's the URL path for hitting this handler, relative to
  /// the URL for the one-off handler itself.
  ///
  /// [handler] will be unmounted as soon as it receives a request.
  String create(shelf.Handler handler) {
    final String path = _counter.toString();
    _handlers[path] = handler;
    _counter++;
    return path;
  }

  /// Dispatches [request] to the appropriate handler.
  FutureOr<shelf.Response> _onRequest(shelf.Request request) {
    final List<String> components = request.url.path.split('/');
    if (components.isEmpty) {
      return shelf.Response.notFound(null);
    }
    final String path = components.removeAt(0);
    final FutureOr<shelf.Response> Function(shelf.Request)? handler = _handlers.remove(path);
    if (handler == null) {
      return shelf.Response.notFound(null);
    }
    return handler(request.change(path: path));
  }
}

class BrowserManager {
  /// Creates a new BrowserManager that communicates with [browser] over
  /// [webSocket].
  BrowserManager._(this._browser, this._runtime, WebSocketChannel webSocket) {
    // The duration should be short enough that the debugging console is open as
    // soon as the user is done setting breakpoints, but long enough that a test
    // doing a lot of synchronous work doesn't trigger a false positive.
    //
    // Start this canceled because we don't want it to start ticking until we
    // get some response from the iframe.
    _timer = RestartableTimer(const Duration(seconds: 3), () {
      for (final RunnerSuiteController controller in _controllers) {
        controller.setDebugging(true);
      }
    })..cancel();

    // Whenever we get a message, no matter which child channel it's for, we know
    // the browser is still running code which means the user isn't debugging.
    _channel = MultiChannel<dynamic>(
      webSocket.cast<String>().transform(jsonDocument).changeStream((Stream<Object?> stream) {
        return stream.map((Object? message) {
          if (!_closed) {
            _timer.reset();
          }
          for (final RunnerSuiteController controller in _controllers) {
            controller.setDebugging(false);
          }

          return message;
        });
      }),
    );

    _environment = _loadBrowserEnvironment();
    _channel.stream.listen(_onMessage, onDone: close);
  }

  /// The browser instance that this is connected to via [_channel].
  final Chromium _browser;
  final Runtime _runtime;

  /// The channel used to communicate with the browser.
  ///
  /// This is connected to a page running `static/host.dart`.
  late MultiChannel<dynamic> _channel;

  /// The ID of the next suite to be loaded.
  ///
  /// This is used to ensure that the suites can be referred to consistently
  /// across the client and server.
  int _suiteID = 0;

  /// Whether the channel to the browser has closed.
  bool _closed = false;

  /// The completer for [_BrowserEnvironment.displayPause].
  ///
  /// This will be `null` as long as the browser isn't displaying a pause
  /// screen.
  CancelableCompleter<dynamic>? _pauseCompleter;

  /// The controller for [_BrowserEnvironment.onRestart].
  final StreamController<dynamic> _onRestartController = StreamController<dynamic>.broadcast();

  /// The environment to attach to each suite.
  late Future<_BrowserEnvironment> _environment;

  /// Controllers for every suite in this browser.
  ///
  /// These are used to mark suites as debugging or not based on the browser's
  /// pings.
  final Set<RunnerSuiteController> _controllers = <RunnerSuiteController>{};

  // A timer that's reset whenever we receive a message from the browser.
  //
  // Because the browser stops running code when the user is actively debugging,
  // this lets us detect whether they're debugging reasonably accurately.
  late RestartableTimer _timer;

  final AsyncMemoizer<dynamic> _closeMemoizer = AsyncMemoizer<dynamic>();

  /// Starts the browser identified by [runtime] and has it connect to [url].
  ///
  /// [url] should serve a page that establishes a WebSocket connection with
  /// this process. That connection, once established, should be emitted via
  /// [future]. If [debug] is true, starts the browser in debug mode, with its
  /// debugger interfaces on and detected.
  ///
  /// The browser will start in headless mode if [headless] is true.
  ///
  /// Add arbitrary browser flags via [webBrowserFlags].
  ///
  /// The [settings] indicate how to invoke this browser's executable.
  ///
  /// Returns the browser manager, or throws an [ApplicationException] if a
  /// connection fails to be established.
  static Future<BrowserManager> start(
    ChromiumLauncher chromiumLauncher,
    Runtime runtime,
    Uri url,
    Future<WebSocketChannel> future, {
    bool debug = false,
    bool headless = true,
    List<String> webBrowserFlags = const <String>[],
  }) async {
    final Chromium chrome = await chromiumLauncher.launch(
      url.toString(),
      headless: headless,
      webBrowserFlags: webBrowserFlags,
    );
    final Completer<BrowserManager> completer = Completer<BrowserManager>();

    unawaited(
      chrome.onExit
          .then<Object?>((int? browserExitCode) {
            throwToolExit('${runtime.name} exited with code $browserExitCode before connecting.');
          })
          .then(
            (Object? obj) => obj,
            onError: (Object error, StackTrace stackTrace) {
              if (!completer.isCompleted) {
                completer.completeError(error, stackTrace);
              }
              return null;
            },
          ),
    );
    unawaited(
      future.then(
        (WebSocketChannel webSocket) {
          if (completer.isCompleted) {
            return;
          }
          completer.complete(BrowserManager._(chrome, runtime, webSocket));
        },
        onError: (Object error, StackTrace stackTrace) {
          chrome.close();
          if (!completer.isCompleted) {
            completer.completeError(error, stackTrace);
          }
        },
      ),
    );

    return completer.future;
  }

  /// Loads [_BrowserEnvironment].
  Future<_BrowserEnvironment> _loadBrowserEnvironment() async {
    return _BrowserEnvironment(
      this,
      null,
      _browser.chromeConnection.url,
      _onRestartController.stream,
    );
  }

  /// Tells the browser to load a test suite from the URL [url].
  ///
  /// [url] should be an HTML page with a reference to the JS-compiled test
  /// suite. [path] is the path of the original test suite file, which is used
  /// for reporting. [suiteConfig] is the configuration for the test suite.
  ///
  /// If [mapper] is passed, it's used to map stack traces for errors coming
  /// from this test suite.
  Future<RunnerSuite> load(
    String path,
    Uri url,
    SuiteConfiguration suiteConfig,
    Object message, {
    Future<void> Function()? onDone,
  }) async {
    url = url.replace(
      fragment: Uri.encodeFull(
        jsonEncode(<String, Object>{
          'metadata': suiteConfig.metadata.serialize(),
          'browser': _runtime.identifier,
        }),
      ),
    );

    final int suiteID = _suiteID++;
    RunnerSuiteController? controller;
    void closeIframe() {
      if (_closed) {
        return;
      }
      _controllers.remove(controller);
      _channel.sink.add(<String, Object>{'command': 'closeSuite', 'id': suiteID});
    }

    // The virtual channel will be closed when the suite is closed, in which
    // case we should unload the iframe.
    final VirtualChannel<dynamic> virtualChannel = _channel.virtualChannel();
    final int suiteChannelID = virtualChannel.id;
    final StreamChannel<dynamic> suiteChannel = virtualChannel.transformStream(
      StreamTransformer<dynamic, dynamic>.fromHandlers(
        handleDone: (EventSink<dynamic> sink) {
          closeIframe();
          sink.close();
          onDone!();
        },
      ),
    );

    _channel.sink.add(<String, Object>{
      'command': 'loadSuite',
      'url': url.toString(),
      'id': suiteID,
      'channel': suiteChannelID,
    });

    try {
      controller = deserializeSuite(
        path,
        SuitePlatform(Runtime.chrome),
        suiteConfig,
        await _environment,
        suiteChannel,
        message,
      );

      _controllers.add(controller);
      return await controller.suite;
      // Not limiting to catching Exception because the exception is rethrown.
    } catch (_) {
      // ignore: avoid_catches_without_on_clauses
      closeIframe();
      rethrow;
    }
  }

  /// An implementation of [Environment.displayPause].
  CancelableOperation<dynamic> _displayPause() {
    if (_pauseCompleter != null) {
      return _pauseCompleter!.operation;
    }
    _pauseCompleter = CancelableCompleter<dynamic>(
      onCancel: () {
        _channel.sink.add(<String, String>{'command': 'resume'});
        _pauseCompleter = null;
      },
    );
    _pauseCompleter!.operation.value.whenComplete(() {
      _pauseCompleter = null;
    });
    _channel.sink.add(<String, String>{'command': 'displayPause'});

    return _pauseCompleter!.operation;
  }

  /// The callback for handling messages received from the host page.
  void _onMessage(dynamic message) {
    assert(message is Map<String, dynamic>);
    if (message is Map<String, dynamic>) {
      switch (message['command'] as String?) {
        case 'ping':
          break;
        case 'restart':
          _onRestartController.add(null);
        case 'resume':
          if (_pauseCompleter != null) {
            _pauseCompleter!.complete();
          }
        default:
          // Unreachable.
          assert(false);
      }
    }
  }

  /// Closes the manager and releases any resources it owns, including closing
  /// the browser.
  Future<dynamic> close() {
    return _closeMemoizer.runOnce(() {
      _closed = true;
      _timer.cancel();
      if (_pauseCompleter != null) {
        _pauseCompleter!.complete();
      }
      _pauseCompleter = null;
      _controllers.clear();
      return _browser.close();
    });
  }
}

/// An implementation of [Environment] for the browser.
///
/// All methods forward directly to [BrowserManager].
class _BrowserEnvironment implements Environment {
  _BrowserEnvironment(this._manager, this.observatoryUrl, this.remoteDebuggerUrl, this.onRestart);

  final BrowserManager _manager;

  @override
  final bool supportsDebugging = true;

  @override
  final Uri? observatoryUrl;

  @override
  final Uri remoteDebuggerUrl;

  @override
  final Stream<dynamic> onRestart;

  @override
  CancelableOperation<dynamic> displayPause() => _manager._displayPause();
}
