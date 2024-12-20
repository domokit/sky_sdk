// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:dwds/data/build_result.dart';
import 'package:dwds/dwds.dart';
import 'package:logging/logging.dart' as logging;
import 'package:meta/meta.dart';
import 'package:mime/mime.dart' as mime;
import 'package:package_config/package_config.dart';
import 'package:shelf/shelf.dart' as shelf;
import 'package:shelf/shelf_io.dart' as shelf;
import 'package:vm_service/vm_service.dart' as vm_service;

import '../artifacts.dart';
import '../asset.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/net.dart';
import '../base/platform.dart';
import '../build_info.dart';
import '../build_system/tools/scene_importer.dart';
import '../build_system/tools/shader_compiler.dart';
import '../bundle_builder.dart';
import '../cache.dart';
import '../compile.dart';
import '../convert.dart';
import '../dart/package_map.dart';
import '../devfs.dart';
import '../device.dart';
import '../globals.dart' as globals;
import '../project.dart';
import '../vmservice.dart';
import '../web/bootstrap.dart';
import '../web/chrome.dart';
import '../web/compile.dart';
import '../web/memory_fs.dart';
import '../web_template.dart';

typedef DwdsLauncher =
    Future<Dwds> Function({
      required AssetReader assetReader,
      required Stream<BuildResult> buildResults,
      required ConnectionProvider chromeConnection,
      required ToolConfiguration toolConfiguration,
    });

// A minimal index for projects that do not yet support web.
const String _kDefaultIndex = '''
<html>
    <head>
        <base href="/">
    </head>
    <body>
        <script src="main.dart.js"></script>
    </body>
</html>
''';

/// An expression compiler connecting to FrontendServer.
///
/// This is only used in development mode.
class WebExpressionCompiler implements ExpressionCompiler {
  WebExpressionCompiler(this._generator, {required FileSystem fileSystem})
    : _fileSystem = fileSystem;

  final ResidentCompiler _generator;
  final FileSystem _fileSystem;

  @override
  Future<ExpressionCompilationResult> compileExpressionToJs(
    String isolateId,
    String libraryUri,
    int line,
    int column,
    Map<String, String> jsModules,
    Map<String, String> jsFrameValues,
    String moduleName,
    String expression,
  ) async {
    final CompilerOutput? compilerOutput = await _generator.compileExpressionToJs(
      libraryUri,
      line,
      column,
      jsModules,
      jsFrameValues,
      moduleName,
      expression,
    );

    if (compilerOutput != null) {
      final String content = utf8.decode(
        _fileSystem.file(compilerOutput.outputFilename).readAsBytesSync(),
      );
      return ExpressionCompilationResult(content, compilerOutput.errorCount > 0);
    }

    return ExpressionCompilationResult(
      "InternalError: frontend server failed to compile '$expression'",
      true,
    );
  }

  @override
  Future<void> initialize(CompilerOptions options) async {}

  @override
  Future<bool> updateDependencies(Map<String, ModuleInfo> modules) async => true;
}

/// A web server which handles serving JavaScript and assets.
///
/// This is only used in development mode.
class WebAssetServer implements AssetReader {
  @visibleForTesting
  WebAssetServer(
    this._httpServer,
    this._packages,
    this.internetAddress,
    this._modules,
    this._digests,
    this._nullSafetyMode,
    this._ddcModuleSystem, {
    required this.webRenderer,
    required this.useLocalCanvasKit,
  }) : basePath = _getWebTemplate('index.html', _kDefaultIndex).getBaseHref();

  // Fallback to "application/octet-stream" on null which
  // makes no claims as to the structure of the data.
  static const String _kDefaultMimeType = 'application/octet-stream';

  final Map<String, String> _modules;
  final Map<String, String> _digests;

  int get selectedPort => _httpServer.port;

  void performRestart(List<String> modules) {
    for (final String module in modules) {
      // We skip computing the digest by using the hashCode of the underlying buffer.
      // Whenever a file is updated, the corresponding Uint8List.view it corresponds
      // to will change.
      final String moduleName = module.startsWith('/') ? module.substring(1) : module;
      final String name = moduleName.replaceAll('.lib.js', '');
      final String path = moduleName.replaceAll('.js', '');
      _modules[name] = path;
      _digests[name] = _webMemoryFS.files[moduleName].hashCode.toString();
    }
  }

  @visibleForTesting
  List<String> write(File codeFile, File manifestFile, File sourcemapFile, File metadataFile) {
    return _webMemoryFS.write(codeFile, manifestFile, sourcemapFile, metadataFile);
  }

  /// Start the web asset server on a [hostname] and [port].
  ///
  /// If [testMode] is true, do not actually initialize dwds or the shelf static
  /// server.
  ///
  /// Unhandled exceptions will throw a [ToolExit] with the error and stack
  /// trace.
  static Future<WebAssetServer> start(
    ChromiumLauncher? chromiumLauncher,
    String hostname,
    int port,
    String? tlsCertPath,
    String? tlsCertKeyPath,
    UrlTunneller? urlTunneller,
    bool useSseForDebugProxy,
    bool useSseForDebugBackend,
    bool useSseForInjectedClient,
    BuildInfo buildInfo,
    bool enableDwds,
    bool enableDds,
    Uri entrypoint,
    ExpressionCompiler? expressionCompiler,
    Map<String, String> extraHeaders,
    NullSafetyMode nullSafetyMode, {
    required WebRendererMode webRenderer,
    required bool isWasm,
    required bool useLocalCanvasKit,
    bool testMode = false,
    DwdsLauncher dwdsLauncher = Dwds.start,
    // TODO(markzipan): Make sure this default value aligns with that in the debugger options.
    bool ddcModuleSystem = false,
  }) async {
    InternetAddress address;
    if (hostname == 'any') {
      address = InternetAddress.anyIPv4;
    } else {
      address = (await InternetAddress.lookup(hostname)).first;
    }
    HttpServer? httpServer;
    const int kMaxRetries = 4;
    for (int i = 0; i <= kMaxRetries; i++) {
      try {
        if (tlsCertPath != null && tlsCertKeyPath != null) {
          final SecurityContext serverContext =
              SecurityContext()
                ..useCertificateChain(tlsCertPath)
                ..usePrivateKey(tlsCertKeyPath);
          httpServer = await HttpServer.bindSecure(address, port, serverContext);
        } else {
          httpServer = await HttpServer.bind(address, port);
        }
        break;
      } on SocketException catch (e, s) {
        if (i >= kMaxRetries) {
          globals.printError('Failed to bind web development server:\n$e', stackTrace: s);
          throwToolExit('Failed to bind web development server:\n$e');
        }
        await Future<void>.delayed(const Duration(milliseconds: 100));
      }
    }

    // Allow rendering in a iframe.
    httpServer!.defaultResponseHeaders.remove('x-frame-options', 'SAMEORIGIN');

    for (final MapEntry<String, String> header in extraHeaders.entries) {
      httpServer.defaultResponseHeaders.add(header.key, header.value);
    }

    final PackageConfig packageConfig = buildInfo.packageConfig;
    final Map<String, String> digests = <String, String>{};
    final Map<String, String> modules = <String, String>{};
    final WebAssetServer server = WebAssetServer(
      httpServer,
      packageConfig,
      address,
      modules,
      digests,
      nullSafetyMode,
      ddcModuleSystem,
      webRenderer: webRenderer,
      useLocalCanvasKit: useLocalCanvasKit,
    );
    if (testMode) {
      return server;
    }

    // In release builds (or wasm builds) deploy a simpler proxy server.
    if (buildInfo.mode != BuildMode.debug || isWasm) {
      final ReleaseAssetServer releaseAssetServer = ReleaseAssetServer(
        entrypoint,
        fileSystem: globals.fs,
        platform: globals.platform,
        flutterRoot: Cache.flutterRoot,
        webBuildDirectory: getWebBuildDirectory(),
        basePath: server.basePath,
        needsCoopCoep: webRenderer == WebRendererMode.skwasm,
      );
      runZonedGuarded(
        () {
          shelf.serveRequests(httpServer!, releaseAssetServer.handle);
        },
        (Object e, StackTrace s) {
          globals.printTrace('Release asset server: error serving requests: $e:$s');
        },
      );
      return server;
    }

    // Return a version string for all active modules. This is populated
    // along with the `moduleProvider` update logic.
    Future<Map<String, String>> digestProvider() async => digests;

    // Ensure dwds is present and provide middleware to avoid trying to
    // load the through the isolate APIs.
    final Directory directory = await _loadDwdsDirectory(globals.fs, globals.logger);
    shelf.Handler middleware(FutureOr<shelf.Response> Function(shelf.Request) innerHandler) {
      return (shelf.Request request) async {
        if (request.url.path.endsWith('dwds/src/injected/client.js')) {
          final Uri uri = directory.uri.resolve('src/injected/client.js');
          final String result = await globals.fs.file(uri.toFilePath()).readAsString();
          return shelf.Response.ok(
            result,
            headers: <String, String>{HttpHeaders.contentTypeHeader: 'application/javascript'},
          );
        }
        return innerHandler(request);
      };
    }

    logging.Logger.root.level = logging.Level.ALL;
    logging.Logger.root.onRecord.listen(log);

    // In debug builds, spin up DWDS and the full asset server.
    final Dwds dwds = await dwdsLauncher(
      assetReader: server,
      buildResults: const Stream<BuildResult>.empty(),
      chromeConnection: () async {
        final Chromium chromium = await chromiumLauncher!.connectedInstance;
        return chromium.chromeConnection;
      },
      toolConfiguration: ToolConfiguration(
        loadStrategy:
            ddcModuleSystem
                ? FrontendServerDdcStrategyProvider(
                  ReloadConfiguration.none,
                  server,
                  PackageUriMapper(packageConfig),
                  digestProvider,
                  BuildSettings(
                    appEntrypoint: packageConfig.toPackageUri(
                      globals.fs.file(entrypoint).absolute.uri,
                    ),
                  ),
                ).strategy
                : FrontendServerRequireStrategyProvider(
                  ReloadConfiguration.none,
                  server,
                  PackageUriMapper(packageConfig),
                  digestProvider,
                  BuildSettings(
                    appEntrypoint: packageConfig.toPackageUri(
                      globals.fs.file(entrypoint).absolute.uri,
                    ),
                  ),
                ).strategy,
        debugSettings: DebugSettings(
          enableDebugExtension: true,
          urlEncoder: urlTunneller,
          useSseForDebugProxy: useSseForDebugProxy,
          useSseForDebugBackend: useSseForDebugBackend,
          useSseForInjectedClient: useSseForInjectedClient,
          expressionCompiler: expressionCompiler,
          spawnDds: enableDds,
        ),
        appMetadata: AppMetadata(hostname: hostname),
      ),
    );
    shelf.Pipeline pipeline = const shelf.Pipeline();
    if (enableDwds) {
      pipeline = pipeline.addMiddleware(middleware);
      pipeline = pipeline.addMiddleware(dwds.middleware);
    }
    final shelf.Handler dwdsHandler = pipeline.addHandler(server.handleRequest);
    final shelf.Cascade cascade = shelf.Cascade().add(dwds.handler).add(dwdsHandler);
    runZonedGuarded(
      () {
        shelf.serveRequests(httpServer!, cascade.handler);
      },
      (Object e, StackTrace s) {
        globals.printTrace('Dwds server: error serving requests: $e:$s');
      },
    );
    server.dwds = dwds;
    server._dwdsInit = true;
    return server;
  }

  final NullSafetyMode _nullSafetyMode;
  final bool _ddcModuleSystem;
  final HttpServer _httpServer;
  final WebMemoryFS _webMemoryFS = WebMemoryFS();
  final PackageConfig _packages;
  final InternetAddress internetAddress;
  late final Dwds dwds;
  late Directory entrypointCacheDirectory;
  bool _dwdsInit = false;

  @visibleForTesting
  HttpHeaders get defaultResponseHeaders => _httpServer.defaultResponseHeaders;

  @visibleForTesting
  Uint8List? getFile(String path) => _webMemoryFS.files[path];

  @visibleForTesting
  Uint8List? getSourceMap(String path) => _webMemoryFS.sourcemaps[path];

  @visibleForTesting
  Uint8List? getMetadata(String path) => _webMemoryFS.metadataFiles[path];

  /// The base path to serve from.
  ///
  /// It should have no leading or trailing slashes.
  @visibleForTesting
  @override
  String basePath;

  // handle requests for JavaScript source, dart sources maps, or asset files.
  @visibleForTesting
  Future<shelf.Response> handleRequest(shelf.Request request) async {
    if (request.method != 'GET') {
      // Assets are served via GET only.
      return shelf.Response.notFound('');
    }

    final String? requestPath = _stripBasePath(request.url.path, basePath);

    if (requestPath == null) {
      return shelf.Response.notFound('');
    }

    // If the response is `/`, then we are requesting the index file.
    if (requestPath == '/' || requestPath.isEmpty) {
      return _serveIndexHtml();
    }

    if (requestPath == 'flutter_bootstrap.js') {
      return _serveFlutterBootstrapJs();
    }

    final Map<String, String> headers = <String, String>{};

    // Track etag headers for better caching of resources.
    final String? ifNoneMatch = request.headers[HttpHeaders.ifNoneMatchHeader];
    headers[HttpHeaders.cacheControlHeader] = 'max-age=0, must-revalidate';

    // If this is a JavaScript file, it must be in the in-memory cache.
    // Attempt to look up the file by URI.
    final String webServerPath = requestPath.replaceFirst('.dart.js', '.dart.lib.js');
    if (_webMemoryFS.files.containsKey(requestPath) ||
        _webMemoryFS.files.containsKey(webServerPath)) {
      final List<int>? bytes = getFile(requestPath) ?? getFile(webServerPath);
      // Use the underlying buffer hashCode as a revision string. This buffer is
      // replaced whenever the frontend_server produces new output files, which
      // will also change the hashCode.
      final String etag = bytes.hashCode.toString();
      if (ifNoneMatch == etag) {
        return shelf.Response.notModified();
      }
      headers[HttpHeaders.contentTypeHeader] = 'application/javascript';
      headers[HttpHeaders.etagHeader] = etag;
      return shelf.Response.ok(bytes, headers: headers);
    }
    // If this is a sourcemap file, then it might be in the in-memory cache.
    // Attempt to lookup the file by URI.
    if (_webMemoryFS.sourcemaps.containsKey(requestPath)) {
      final List<int>? bytes = getSourceMap(requestPath);
      final String etag = bytes.hashCode.toString();
      if (ifNoneMatch == etag) {
        return shelf.Response.notModified();
      }
      headers[HttpHeaders.contentTypeHeader] = 'application/json';
      headers[HttpHeaders.etagHeader] = etag;
      return shelf.Response.ok(bytes, headers: headers);
    }

    // If this is a metadata file, then it might be in the in-memory cache.
    // Attempt to lookup the file by URI.
    if (_webMemoryFS.metadataFiles.containsKey(requestPath)) {
      final List<int>? bytes = getMetadata(requestPath);
      final String etag = bytes.hashCode.toString();
      if (ifNoneMatch == etag) {
        return shelf.Response.notModified();
      }
      headers[HttpHeaders.contentTypeHeader] = 'application/json';
      headers[HttpHeaders.etagHeader] = etag;
      return shelf.Response.ok(bytes, headers: headers);
    }

    File file = _resolveDartFile(requestPath);

    if (!file.existsSync() && requestPath.startsWith('canvaskit/')) {
      final Directory canvasKitDirectory = globals.fs.directory(
        globals.fs.path.join(
          globals.artifacts!.getHostArtifact(HostArtifact.flutterWebSdk).path,
          'canvaskit',
        ),
      );
      final Uri potential = canvasKitDirectory.uri.resolve(
        requestPath.replaceFirst('canvaskit/', ''),
      );
      file = globals.fs.file(potential);
    }

    // If all of the lookups above failed, the file might have been an asset.
    // Try and resolve the path relative to the built asset directory.
    if (!file.existsSync()) {
      final Uri potential = globals.fs
          .directory(getAssetBuildDirectory())
          .uri
          .resolve(requestPath.replaceFirst('assets/', ''));
      file = globals.fs.file(potential);
    }

    if (!file.existsSync()) {
      final Uri webPath = globals.fs.currentDirectory
          .childDirectory('web')
          .uri
          .resolve(requestPath);
      file = globals.fs.file(webPath);
    }

    if (!file.existsSync()) {
      // Paths starting with these prefixes should've been resolved above.
      if (requestPath.startsWith('assets/') ||
          requestPath.startsWith('packages/') ||
          requestPath.startsWith('canvaskit/')) {
        return shelf.Response.notFound('');
      }
      return _serveIndexHtml();
    }

    // For real files, use a serialized file stat plus path as a revision.
    // This allows us to update between canvaskit and non-canvaskit SDKs.
    final String etag = file.lastModifiedSync().toIso8601String() + Uri.encodeComponent(file.path);
    if (ifNoneMatch == etag) {
      return shelf.Response.notModified();
    }

    final int length = file.lengthSync();
    // Attempt to determine the file's mime type. if this is not provided some
    // browsers will refuse to render images/show video etc. If the tool
    // cannot determine a mime type, fall back to application/octet-stream.
    final String mimeType =
        mime.lookupMimeType(
          file.path,
          headerBytes: await file.openRead(0, mime.defaultMagicNumbersMaxLength).first,
        ) ??
        _kDefaultMimeType;

    headers[HttpHeaders.contentLengthHeader] = length.toString();
    headers[HttpHeaders.contentTypeHeader] = mimeType;
    headers[HttpHeaders.etagHeader] = etag;
    return shelf.Response.ok(file.openRead(), headers: headers);
  }

  /// Tear down the http server running.
  Future<void> dispose() async {
    if (_dwdsInit) {
      await dwds.stop();
    }
    return _httpServer.close();
  }

  /// Write a single file into the in-memory cache.
  void writeFile(String filePath, String contents) {
    writeBytes(filePath, const Utf8Encoder().convert(contents));
  }

  void writeBytes(String filePath, Uint8List contents) {
    _webMemoryFS.files[filePath] = contents;
  }

  /// Determines what rendering backed to use.
  final WebRendererMode webRenderer;

  final bool useLocalCanvasKit;

  String get _buildConfigString {
    final Map<String, dynamic> buildConfig = <String, dynamic>{
      'engineRevision': globals.flutterVersion.engineRevision,
      'builds': <dynamic>[
        <String, dynamic>{
          'compileTarget': 'dartdevc',
          'renderer': webRenderer.name,
          'mainJsPath': 'main.dart.js',
        },
      ],
      if (useLocalCanvasKit) 'useLocalCanvasKit': true,
    };
    return '''
if (!window._flutter) {
  window._flutter = {};
}
_flutter.buildConfig = ${jsonEncode(buildConfig)};
''';
  }

  File get _flutterJsFile => globals.fs.file(
    globals.fs.path.join(
      globals.artifacts!.getHostArtifact(HostArtifact.flutterJsDirectory).path,
      'flutter.js',
    ),
  );

  String get _flutterBootstrapJsContent {
    final WebTemplate bootstrapTemplate = _getWebTemplate(
      'flutter_bootstrap.js',
      generateDefaultFlutterBootstrapScript(),
    );
    bootstrapTemplate.applySubstitutions(
      baseHref: '/',
      serviceWorkerVersion: null,
      buildConfig: _buildConfigString,
      flutterJsFile: _flutterJsFile,
    );
    return bootstrapTemplate.content;
  }

  shelf.Response _serveFlutterBootstrapJs() {
    return shelf.Response.ok(
      _flutterBootstrapJsContent,
      headers: <String, String>{HttpHeaders.contentTypeHeader: 'text/javascript'},
    );
  }

  shelf.Response _serveIndexHtml() {
    final WebTemplate indexHtml = _getWebTemplate('index.html', _kDefaultIndex);
    indexHtml.applySubstitutions(
      // Currently, we don't support --base-href for the "run" command.
      baseHref: '/',
      serviceWorkerVersion: null,
      buildConfig: _buildConfigString,
      flutterJsFile: _flutterJsFile,
      flutterBootstrapJs: _flutterBootstrapJsContent,
    );
    return shelf.Response.ok(
      indexHtml.content,
      headers: <String, String>{HttpHeaders.contentTypeHeader: 'text/html'},
    );
  }

  // Attempt to resolve `path` to a dart file.
  File _resolveDartFile(String path) {
    // Return the actual file objects so that local engine changes are automatically picked up.
    switch (path) {
      case 'dart_sdk.js':
        return _resolveDartSdkJsFile;
      case 'dart_sdk.js.map':
        return _resolveDartSdkJsMapFile;
    }
    // This is the special generated entrypoint.
    if (path == 'web_entrypoint.dart') {
      return entrypointCacheDirectory.childFile('web_entrypoint.dart');
    }

    // If this is a dart file, it must be on the local file system and is
    // likely coming from a source map request. The tool doesn't currently
    // consider the case of Dart files as assets.
    final File dartFile = globals.fs.file(globals.fs.currentDirectory.uri.resolve(path));
    if (dartFile.existsSync()) {
      return dartFile;
    }

    final List<String> segments = path.split('/');
    if (segments.first.isEmpty) {
      segments.removeAt(0);
    }

    // The file might have been a package file which is signaled by a
    // `/packages/<package>/<path>` request.
    if (segments.first == 'packages') {
      final Uri? filePath = _packages.resolve(
        Uri(scheme: 'package', pathSegments: segments.skip(1)),
      );
      if (filePath != null) {
        final File packageFile = globals.fs.file(filePath);
        if (packageFile.existsSync()) {
          return packageFile;
        }
      }
    }

    // Otherwise it must be a Dart SDK source or a Flutter Web SDK source.
    final Directory dartSdkParent =
        globals.fs
            .directory(
              globals.artifacts!.getArtifactPath(
                Artifact.engineDartSdkPath,
                platform: TargetPlatform.web_javascript,
              ),
            )
            .parent;
    final File dartSdkFile = globals.fs.file(dartSdkParent.uri.resolve(path));
    if (dartSdkFile.existsSync()) {
      return dartSdkFile;
    }

    final Directory flutterWebSdk = globals.fs.directory(
      globals.artifacts!.getHostArtifact(HostArtifact.flutterWebSdk),
    );
    final File webSdkFile = globals.fs.file(flutterWebSdk.uri.resolve(path));

    return webSdkFile;
  }

  File get _resolveDartSdkJsFile {
    final Map<WebRendererMode, Map<NullSafetyMode, HostArtifact>> dartSdkArtifactMap =
        _ddcModuleSystem ? kDdcDartSdkJsArtifactMap : kAmdDartSdkJsArtifactMap;
    return globals.fs.file(
      globals.artifacts!.getHostArtifact(dartSdkArtifactMap[webRenderer]![_nullSafetyMode]!),
    );
  }

  File get _resolveDartSdkJsMapFile {
    final Map<WebRendererMode, Map<NullSafetyMode, HostArtifact>> dartSdkArtifactMap =
        _ddcModuleSystem ? kDdcDartSdkJsMapArtifactMap : kAmdDartSdkJsMapArtifactMap;
    return globals.fs.file(
      globals.artifacts!.getHostArtifact(dartSdkArtifactMap[webRenderer]![_nullSafetyMode]!),
    );
  }

  @override
  Future<String?> dartSourceContents(String serverPath) async {
    serverPath = _stripBasePath(serverPath, basePath)!;
    final File result = _resolveDartFile(serverPath);
    if (result.existsSync()) {
      return result.readAsString();
    }
    return null;
  }

  @override
  Future<String> sourceMapContents(String serverPath) async {
    serverPath = _stripBasePath(serverPath, basePath)!;
    return utf8.decode(_webMemoryFS.sourcemaps[serverPath]!);
  }

  @override
  Future<String?> metadataContents(String serverPath) async {
    final String? resultPath = _stripBasePath(serverPath, basePath);
    if (resultPath == 'main_module.ddc_merged_metadata') {
      return _webMemoryFS.mergedMetadata;
    }
    if (_webMemoryFS.metadataFiles.containsKey(resultPath)) {
      return utf8.decode(_webMemoryFS.metadataFiles[resultPath]!);
    }
    throw Exception('Could not find metadata contents for $serverPath');
  }

  @override
  Future<void> close() async {}
}

class ConnectionResult {
  ConnectionResult(this.appConnection, this.debugConnection, this.vmService);

  final AppConnection? appConnection;
  final DebugConnection? debugConnection;
  final vm_service.VmService vmService;
}

typedef VmServiceFactory =
    Future<vm_service.VmService> Function(
      Uri, {
      CompressionOptions compression,
      required Logger logger,
    });

/// The web specific DevFS implementation.
class WebDevFS implements DevFS {
  /// Create a new [WebDevFS] instance.
  ///
  /// [testMode] is true, do not actually initialize dwds or the shelf static
  /// server.
  WebDevFS({
    required this.hostname,
    required int port,
    required this.tlsCertPath,
    required this.tlsCertKeyPath,
    required this.packagesFilePath,
    required this.urlTunneller,
    required this.useSseForDebugProxy,
    required this.useSseForDebugBackend,
    required this.useSseForInjectedClient,
    required this.buildInfo,
    required this.enableDwds,
    required this.enableDds,
    required this.entrypoint,
    required this.expressionCompiler,
    required this.extraHeaders,
    required this.chromiumLauncher,
    required this.nullAssertions,
    required this.nativeNullAssertions,
    required this.nullSafetyMode,
    required this.ddcModuleSystem,
    required this.webRenderer,
    required this.isWasm,
    required this.useLocalCanvasKit,
    required this.rootDirectory,
    this.testMode = false,
  }) : _port = port;

  final Uri entrypoint;
  final String hostname;
  final String packagesFilePath;
  final UrlTunneller? urlTunneller;
  final bool useSseForDebugProxy;
  final bool useSseForDebugBackend;
  final bool useSseForInjectedClient;
  final BuildInfo buildInfo;
  final bool enableDwds;
  final bool enableDds;
  final Map<String, String> extraHeaders;
  final bool testMode;
  final bool ddcModuleSystem;
  final ExpressionCompiler? expressionCompiler;
  final ChromiumLauncher? chromiumLauncher;
  final bool nullAssertions;
  final bool nativeNullAssertions;
  final int _port;
  final NullSafetyMode nullSafetyMode;
  final String? tlsCertPath;
  final String? tlsCertKeyPath;
  final WebRendererMode webRenderer;
  final bool isWasm;
  final bool useLocalCanvasKit;

  late WebAssetServer webAssetServer;

  Dwds get dwds => webAssetServer.dwds;

  // A flag to indicate whether we have called `setAssetDirectory` on the target device.
  @override
  bool hasSetAssetDirectory = false;

  @override
  bool didUpdateFontManifest = false;

  Future<DebugConnection>? _cachedExtensionFuture;
  StreamSubscription<void>? _connectedApps;

  /// Connect and retrieve the [DebugConnection] for the current application.
  ///
  /// Only calls [AppConnection.runMain] on the subsequent connections.
  Future<ConnectionResult?> connect(
    bool useDebugExtension, {
    @visibleForTesting VmServiceFactory vmServiceFactory = createVmServiceDelegate,
  }) {
    final Completer<ConnectionResult> firstConnection = Completer<ConnectionResult>();
    // Note there is an asynchronous gap between this being set to true and
    // [firstConnection] completing; thus test the boolean to determine if
    // the current connection is the first.
    bool foundFirstConnection = false;
    _connectedApps = dwds.connectedApps.listen(
      (AppConnection appConnection) async {
        try {
          final DebugConnection debugConnection =
              useDebugExtension
                  ? await (_cachedExtensionFuture ??= dwds.extensionDebugConnections.stream.first)
                  : await dwds.debugConnection(appConnection);
          if (foundFirstConnection) {
            appConnection.runMain();
          } else {
            foundFirstConnection = true;
            final vm_service.VmService vmService = await vmServiceFactory(
              Uri.parse(debugConnection.uri),
              logger: globals.logger,
            );
            firstConnection.complete(ConnectionResult(appConnection, debugConnection, vmService));
          }
        } on Exception catch (error, stackTrace) {
          if (!firstConnection.isCompleted) {
            firstConnection.completeError(error, stackTrace);
          }
        }
      },
      onError: (Object error, StackTrace stackTrace) {
        globals.printError('Unknown error while waiting for debug connection:$error\n$stackTrace');
        if (!firstConnection.isCompleted) {
          firstConnection.completeError(error, stackTrace);
        }
      },
    );
    return firstConnection.future;
  }

  @override
  List<Uri> sources = <Uri>[];

  @override
  DateTime? lastCompiled;

  @override
  PackageConfig? lastPackageConfig;

  // We do not evict assets on the web.
  @override
  Set<String> get assetPathsToEvict => const <String>{};

  @override
  Uri? get baseUri => _baseUri;
  Uri? _baseUri;

  @override
  Future<Uri> create() async {
    webAssetServer = await WebAssetServer.start(
      chromiumLauncher,
      hostname,
      _port,
      tlsCertPath,
      tlsCertKeyPath,
      urlTunneller,
      useSseForDebugProxy,
      useSseForDebugBackend,
      useSseForInjectedClient,
      buildInfo,
      enableDwds,
      enableDds,
      entrypoint,
      expressionCompiler,
      extraHeaders,
      nullSafetyMode,
      webRenderer: webRenderer,
      isWasm: isWasm,
      useLocalCanvasKit: useLocalCanvasKit,
      testMode: testMode,
      ddcModuleSystem: ddcModuleSystem,
    );

    final int selectedPort = webAssetServer.selectedPort;
    String url = '$hostname:$selectedPort';
    if (hostname == 'any') {
      url = 'localhost:$selectedPort';
    }
    _baseUri = Uri.http(url, webAssetServer.basePath);
    if (tlsCertPath != null && tlsCertKeyPath != null) {
      _baseUri = Uri.https(url, webAssetServer.basePath);
    }
    return _baseUri!;
  }

  @override
  Future<void> destroy() async {
    await webAssetServer.dispose();
    await _connectedApps?.cancel();
  }

  @override
  Uri deviceUriToHostUri(Uri deviceUri) {
    return deviceUri;
  }

  @override
  String get fsName => 'web_asset';

  @override
  final Directory rootDirectory;

  Future<void> _validateTemplateFile(String filename) async {
    final File file = globals.fs.currentDirectory.childDirectory('web').childFile(filename);
    if (!await file.exists()) {
      return;
    }

    final WebTemplate template = WebTemplate(await file.readAsString());
    for (final WebTemplateWarning warning in template.getWarnings()) {
      globals.logger.printWarning(
        'Warning: In $filename:${warning.lineNumber}: ${warning.warningText}',
      );
    }
  }

  @override
  Future<UpdateFSReport> update({
    required Uri mainUri,
    required ResidentCompiler generator,
    required bool trackWidgetCreation,
    required String pathToReload,
    required List<Uri> invalidatedFiles,
    required PackageConfig packageConfig,
    required String dillOutputPath,
    required DevelopmentShaderCompiler shaderCompiler,
    DevelopmentSceneImporter? sceneImporter,
    DevFSWriter? devFSWriter,
    String? target,
    AssetBundle? bundle,
    bool bundleFirstUpload = false,
    bool fullRestart = false,
    String? projectRootPath,
    File? dartPluginRegistrant,
  }) async {
    lastPackageConfig = packageConfig;
    final File mainFile = globals.fs.file(mainUri);
    final String outputDirectoryPath = mainFile.parent.path;

    if (bundleFirstUpload) {
      webAssetServer.entrypointCacheDirectory = globals.fs.directory(outputDirectoryPath);
      generator.addFileSystemRoot(outputDirectoryPath);
      final String entrypoint = globals.fs.path.basename(mainFile.path);
      webAssetServer.writeBytes(entrypoint, mainFile.readAsBytesSync());
      if (ddcModuleSystem) {
        webAssetServer.writeBytes('ddc_module_loader.js', ddcModuleLoaderJS.readAsBytesSync());
      } else {
        webAssetServer.writeBytes('require.js', requireJS.readAsBytesSync());
      }
      webAssetServer.writeBytes('flutter.js', flutterJs.readAsBytesSync());
      webAssetServer.writeBytes('stack_trace_mapper.js', stackTraceMapper.readAsBytesSync());
      webAssetServer.writeFile('manifest.json', '{"info":"manifest not generated in run mode."}');
      webAssetServer.writeFile(
        'flutter_service_worker.js',
        '// Service worker not loaded in run mode.',
      );
      webAssetServer.writeFile('version.json', FlutterProject.current().getVersionInfo());
      webAssetServer.writeFile(
        'main.dart.js',
        ddcModuleSystem
            ? generateDDCBootstrapScript(
              entrypoint: entrypoint,
              ddcModuleLoaderUrl: 'ddc_module_loader.js',
              mapperUrl: 'stack_trace_mapper.js',
              generateLoadingIndicator: enableDwds,
            )
            : generateBootstrapScript(
              requireUrl: 'require.js',
              mapperUrl: 'stack_trace_mapper.js',
              generateLoadingIndicator: enableDwds,
            ),
      );
      webAssetServer.writeFile(
        'main_module.bootstrap.js',
        ddcModuleSystem
            ? generateDDCMainModule(
              entrypoint: entrypoint,
              nullAssertions: nullAssertions,
              nativeNullAssertions: nativeNullAssertions,
              exportedMain: pathToJSIdentifier(entrypoint.split('.')[0]),
            )
            : generateMainModule(
              entrypoint: entrypoint,
              nullAssertions: nullAssertions,
              nativeNullAssertions: nativeNullAssertions,
            ),
      );
      // TODO(zanderso): refactor the asset code in this and the regular devfs to
      // be shared.
      if (bundle != null) {
        await writeBundle(
          globals.fs.directory(getAssetBuildDirectory()),
          bundle.entries,
          targetPlatform: TargetPlatform.web_javascript,
          impellerStatus: ImpellerStatus.disabled,
          processManager: globals.processManager,
          fileSystem: globals.fs,
          artifacts: globals.artifacts!,
          logger: globals.logger,
          projectDir: rootDirectory,
          buildMode: buildInfo.mode,
        );
      }
    }
    await _validateTemplateFile('index.html');
    await _validateTemplateFile('flutter_bootstrap.js');
    final DateTime candidateCompileTime = DateTime.now();
    if (fullRestart) {
      generator.reset();
    }

    // The tool generates an entrypoint file in a temp directory to handle
    // the web specific bootstrap logic. To make it easier for DWDS to handle
    // mapping the file name, this is done via an additional file root and
    // special hard-coded scheme.
    final CompilerOutput? compilerOutput = await generator.recompile(
      Uri(scheme: 'org-dartlang-app', path: '/${mainUri.pathSegments.last}'),
      invalidatedFiles,
      outputPath: dillOutputPath,
      packageConfig: packageConfig,
      projectRootPath: projectRootPath,
      fs: globals.fs,
      dartPluginRegistrant: dartPluginRegistrant,
    );
    if (compilerOutput == null || compilerOutput.errorCount > 0) {
      return UpdateFSReport();
    }

    // Only update the last compiled time if we successfully compiled.
    lastCompiled = candidateCompileTime;
    // list of sources that needs to be monitored are in [compilerOutput.sources]
    sources = compilerOutput.sources;
    late File codeFile;
    File manifestFile;
    File sourcemapFile;
    File metadataFile;
    late List<String> modules;
    try {
      final Directory parentDirectory = globals.fs.directory(outputDirectoryPath);
      codeFile = parentDirectory.childFile('${compilerOutput.outputFilename}.sources');
      manifestFile = parentDirectory.childFile('${compilerOutput.outputFilename}.json');
      sourcemapFile = parentDirectory.childFile('${compilerOutput.outputFilename}.map');
      metadataFile = parentDirectory.childFile('${compilerOutput.outputFilename}.metadata');
      modules = webAssetServer._webMemoryFS.write(
        codeFile,
        manifestFile,
        sourcemapFile,
        metadataFile,
      );
    } on FileSystemException catch (err) {
      throwToolExit('Failed to load recompiled sources:\n$err');
    }
    webAssetServer.performRestart(modules);
    return UpdateFSReport(
      success: true,
      syncedBytes: codeFile.lengthSync(),
      invalidatedSourcesCount: invalidatedFiles.length,
    );
  }

  @visibleForTesting
  final File requireJS = globals.fs.file(
    globals.fs.path.join(
      globals.artifacts!.getArtifactPath(
        Artifact.engineDartSdkPath,
        platform: TargetPlatform.web_javascript,
      ),
      'lib',
      'dev_compiler',
      'amd',
      'require.js',
    ),
  );

  @visibleForTesting
  final File ddcModuleLoaderJS = globals.fs.file(
    globals.fs.path.join(
      globals.artifacts!.getArtifactPath(
        Artifact.engineDartSdkPath,
        platform: TargetPlatform.web_javascript,
      ),
      'lib',
      'dev_compiler',
      'ddc',
      'ddc_module_loader.js',
    ),
  );

  @visibleForTesting
  final File flutterJs = globals.fs.file(
    globals.fs.path.join(
      globals.artifacts!.getHostArtifact(HostArtifact.flutterJsDirectory).path,
      'flutter.js',
    ),
  );

  @visibleForTesting
  final File stackTraceMapper = globals.fs.file(
    globals.fs.path.join(
      globals.artifacts!.getArtifactPath(
        Artifact.engineDartSdkPath,
        platform: TargetPlatform.web_javascript,
      ),
      'lib',
      'dev_compiler',
      'web',
      'dart_stack_trace_mapper.js',
    ),
  );

  @override
  void resetLastCompiled() {
    // Not used for web compilation.
  }

  @override
  Set<String> get shaderPathsToEvict => <String>{};

  @override
  Set<String> get scenePathsToEvict => <String>{};
}

class ReleaseAssetServer {
  ReleaseAssetServer(
    this.entrypoint, {
    required FileSystem fileSystem,
    required String? webBuildDirectory,
    required String? flutterRoot,
    required Platform platform,
    required bool needsCoopCoep,
    this.basePath = '',
  }) : _fileSystem = fileSystem,
       _platform = platform,
       _flutterRoot = flutterRoot,
       _webBuildDirectory = webBuildDirectory,
       _needsCoopCoep = needsCoopCoep,
       _fileSystemUtils = FileSystemUtils(fileSystem: fileSystem, platform: platform);

  final Uri entrypoint;
  final String? _flutterRoot;
  final String? _webBuildDirectory;
  final FileSystem _fileSystem;
  final FileSystemUtils _fileSystemUtils;
  final Platform _platform;
  final bool _needsCoopCoep;

  /// The base path to serve from.
  ///
  /// It should have no leading or trailing slashes.
  @visibleForTesting
  final String basePath;

  // Locations where source files, assets, or source maps may be located.
  List<Uri> _searchPaths() => <Uri>[
    _fileSystem.directory(_webBuildDirectory).uri,
    _fileSystem.directory(_flutterRoot).uri,
    _fileSystem.directory(_flutterRoot).parent.uri,
    _fileSystem.currentDirectory.uri,
    _fileSystem.directory(_fileSystemUtils.homeDirPath).uri,
  ];

  Future<shelf.Response> handle(shelf.Request request) async {
    if (request.method != 'GET') {
      // Assets are served via GET only.
      return shelf.Response.notFound('');
    }

    Uri? fileUri;
    final String? requestPath = _stripBasePath(request.url.path, basePath);

    if (requestPath == null) {
      return shelf.Response.notFound('');
    }

    if (request.url.toString() == 'main.dart') {
      fileUri = entrypoint;
    } else {
      for (final Uri uri in _searchPaths()) {
        final Uri potential = uri.resolve(requestPath);
        if (!_fileSystem.isFileSync(potential.toFilePath(windows: _platform.isWindows))) {
          continue;
        }
        fileUri = potential;
        break;
      }
    }
    if (fileUri != null) {
      final File file = _fileSystem.file(fileUri);
      final Uint8List bytes = file.readAsBytesSync();
      // Fallback to "application/octet-stream" on null which
      // makes no claims as to the structure of the data.
      final String mimeType =
          mime.lookupMimeType(file.path, headerBytes: bytes) ?? 'application/octet-stream';
      return shelf.Response.ok(
        bytes,
        headers: <String, String>{
          'Content-Type': mimeType,
          'Cross-Origin-Resource-Policy': 'cross-origin',
          'Access-Control-Allow-Origin': '*',
          if (_needsCoopCoep &&
              _fileSystem.path.extension(file.path) == '.html') ...<String, String>{
            'Cross-Origin-Opener-Policy': 'same-origin',
            'Cross-Origin-Embedder-Policy': 'credentialless',
          },
        },
      );
    }

    final File file = _fileSystem.file(_fileSystem.path.join(_webBuildDirectory!, 'index.html'));
    return shelf.Response.ok(
      file.readAsBytesSync(),
      headers: <String, String>{
        'Content-Type': 'text/html',
        if (_needsCoopCoep) ...<String, String>{
          'Cross-Origin-Opener-Policy': 'same-origin',
          'Cross-Origin-Embedder-Policy': 'credentialless',
        },
      },
    );
  }
}

@visibleForTesting
void log(logging.LogRecord event) {
  final String error = event.error == null ? '' : 'Error: ${event.error}';
  if (event.level >= logging.Level.SEVERE) {
    globals.printError('${event.loggerName}: ${event.message}$error', stackTrace: event.stackTrace);
  } else if (event.level == logging.Level.WARNING) {
    // Temporary fix for https://github.com/flutter/flutter/issues/109792
    // TODO(annagrin): Remove the condition after the bogus warning is
    // removed in dwds: https://github.com/dart-lang/webdev/issues/1722
    if (!event.message.contains('No module for')) {
      globals.printWarning('${event.loggerName}: ${event.message}$error');
    }
  } else {
    globals.printTrace('${event.loggerName}: ${event.message}$error');
  }
}

Future<Directory> _loadDwdsDirectory(FileSystem fileSystem, Logger logger) async {
  final PackageConfig packageConfig = await currentPackageConfig();
  return fileSystem.directory(packageConfig['dwds']!.packageUriRoot);
}

String? _stripBasePath(String path, String basePath) {
  path = stripLeadingSlash(path);
  if (path.startsWith(basePath)) {
    path = path.substring(basePath.length);
  } else {
    // The given path isn't under base path, return null to indicate that.
    return null;
  }
  return stripLeadingSlash(path);
}

WebTemplate _getWebTemplate(String filename, String fallbackContent) {
  final File template = globals.fs.currentDirectory.childDirectory('web').childFile(filename);
  final String htmlContent = template.existsSync() ? template.readAsStringSync() : fallbackContent;
  return WebTemplate(htmlContent);
}
