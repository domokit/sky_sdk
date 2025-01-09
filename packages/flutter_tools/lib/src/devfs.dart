// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:package_config/package_config.dart';
import 'package:process/process.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import 'artifacts.dart';
import 'asset.dart';
import 'base/config.dart';
import 'base/context.dart';
import 'base/file_system.dart';
import 'base/io.dart';
import 'base/logger.dart';
import 'base/net.dart';
import 'base/os.dart';
import 'build_info.dart';
import 'build_system/tools/asset_transformer.dart';
import 'build_system/tools/scene_importer.dart';
import 'build_system/tools/shader_compiler.dart';
import 'compile.dart';
import 'convert.dart' show base64, utf8;
import 'vmservice.dart';

const String _kFontManifest = 'FontManifest.json';

class DevFSConfig {
  /// Should DevFS assume that symlink targets are stable?
  bool cacheSymlinks = false;

  /// Should DevFS assume that there are no symlinks to directories?
  bool noDirectorySymlinks = false;
}

DevFSConfig? get devFSConfig => context.get<DevFSConfig>();

/// Common superclass for content copied to the device.
abstract class DevFSContent {
  /// Return true if this is the first time this method is called
  /// or if the entry has been modified since this method was last called.
  bool get isModified;

  /// Return true if this is the first time this method is called
  /// or if the entry has been modified after the given time
  /// or if the given time is null.
  bool isModifiedAfter(DateTime time);

  int get size;

  Future<List<int>> contentsAsBytes();

  Stream<List<int>> contentsAsStream();

  Stream<List<int>> contentsAsCompressedStream(OperatingSystemUtils osUtils) {
    return osUtils.gzipLevel1Stream(contentsAsStream());
  }
}

// File content to be copied to the device.
class DevFSFileContent extends DevFSContent {
  DevFSFileContent(this.file);

  final FileSystemEntity file;
  File? _linkTarget;
  FileStat? _fileStat;

  File _getFile() {
    final File? linkTarget = _linkTarget;
    if (linkTarget != null) {
      return linkTarget;
    }
    if (file is Link) {
      // The link target.
      return file.fileSystem.file(file.resolveSymbolicLinksSync());
    }
    return file as File;
  }

  void _stat() {
    final File? linkTarget = _linkTarget;
    if (linkTarget != null) {
      // Stat the cached symlink target.
      final FileStat fileStat = linkTarget.statSync();
      if (fileStat.type == FileSystemEntityType.notFound) {
        _linkTarget = null;
      } else {
        _fileStat = fileStat;
        return;
      }
    }
    final FileStat fileStat = file.statSync();
    _fileStat = fileStat.type == FileSystemEntityType.notFound ? null : fileStat;
    if (_fileStat != null && _fileStat?.type == FileSystemEntityType.link) {
      // Resolve, stat, and maybe cache the symlink target.
      final String resolved = file.resolveSymbolicLinksSync();
      final File linkTarget = file.fileSystem.file(resolved);
      // Stat the link target.
      final FileStat fileStat = linkTarget.statSync();
      if (fileStat.type == FileSystemEntityType.notFound) {
        _fileStat = null;
        _linkTarget = null;
      } else if (devFSConfig?.cacheSymlinks ?? false) {
        _linkTarget = linkTarget;
      }
    }
  }

  @override
  bool get isModified {
    final FileStat? oldFileStat = _fileStat;
    _stat();
    final FileStat? newFileStat = _fileStat;
    if (oldFileStat == null && newFileStat == null) {
      return false;
    }
    return oldFileStat == null ||
        newFileStat == null ||
        newFileStat.modified.isAfter(oldFileStat.modified);
  }

  @override
  bool isModifiedAfter(DateTime time) {
    final FileStat? oldFileStat = _fileStat;
    _stat();
    final FileStat? newFileStat = _fileStat;
    if (oldFileStat == null && newFileStat == null) {
      return false;
    }
    return oldFileStat == null || newFileStat == null || newFileStat.modified.isAfter(time);
  }

  @override
  int get size {
    if (_fileStat == null) {
      _stat();
    }
    // Can still be null if the file wasn't found.
    return _fileStat?.size ?? 0;
  }

  @override
  Future<List<int>> contentsAsBytes() async => _getFile().readAsBytes();

  @override
  Stream<List<int>> contentsAsStream() => _getFile().openRead();
}

/// Byte content to be copied to the device.
class DevFSByteContent extends DevFSContent {
  DevFSByteContent(this._bytes);

  final List<int> _bytes;
  final DateTime _creationTime = DateTime.now();
  bool _isModified = true;

  List<int> get bytes => _bytes;

  /// Return true only once so that the content is written to the device only once.
  @override
  bool get isModified {
    final bool modified = _isModified;
    _isModified = false;
    return modified;
  }

  @override
  bool isModifiedAfter(DateTime time) {
    return _creationTime.isAfter(time);
  }

  @override
  int get size => _bytes.length;

  @override
  Future<List<int>> contentsAsBytes() async => _bytes;

  @override
  Stream<List<int>> contentsAsStream() => Stream<List<int>>.fromIterable(<List<int>>[_bytes]);
}

/// String content to be copied to the device.
class DevFSStringContent extends DevFSByteContent {
  DevFSStringContent(String string) : _string = string, super(utf8.encode(string));

  final String _string;

  String get string => _string;
}

/// A string compressing DevFSContent.
///
/// A specialized DevFSContent similar to DevFSByteContent where the contents
/// are the compressed bytes of a string. Its difference is that the original
/// uncompressed string can be compared with directly without the indirection
/// of a compute-expensive uncompress/decode and compress/encode to compare
/// the strings.
///
/// The `hintString` parameter is a zlib dictionary hinting mechanism to suggest
/// the most common string occurrences to potentially assist with compression.
class DevFSStringCompressingBytesContent extends DevFSContent {
  DevFSStringCompressingBytesContent(this._string, {String? hintString})
    : _compressor = ZLibEncoder(
        dictionary: hintString == null ? null : utf8.encode(hintString),
        gzip: true,
        level: 9,
      );

  final String _string;
  final ZLibEncoder _compressor;
  final DateTime _creationTime = DateTime.now();

  bool _isModified = true;

  late final List<int> bytes = _compressor.convert(utf8.encode(_string));

  /// Return true only once so that the content is written to the device only once.
  @override
  bool get isModified {
    final bool modified = _isModified;
    _isModified = false;
    return modified;
  }

  @override
  bool isModifiedAfter(DateTime time) {
    return _creationTime.isAfter(time);
  }

  @override
  int get size => bytes.length;

  @override
  Future<List<int>> contentsAsBytes() async => bytes;

  @override
  Stream<List<int>> contentsAsStream() => Stream<List<int>>.value(bytes);

  /// This checks the source string with another string.
  bool equals(String string) => _string == string;
}

class DevFSException implements Exception {
  DevFSException(this.message, [this.error, this.stackTrace]);
  final String message;
  final dynamic error;
  final StackTrace? stackTrace;

  @override
  String toString() => 'DevFSException($message, $error, $stackTrace)';
}

/// Interface responsible for syncing asset files to a development device.
abstract class DevFSWriter {
  /// Write the assets in [entries] to the target device.
  ///
  /// The keys of the map are relative from the [baseUri].
  ///
  /// Throws a [DevFSException] if the process fails to complete.
  Future<void> write(Map<Uri, DevFSContent> entries, Uri baseUri, DevFSWriter parent);
}

class _DevFSHttpWriter implements DevFSWriter {
  _DevFSHttpWriter(
    this.fsName,
    FlutterVmService serviceProtocol, {
    required OperatingSystemUtils osUtils,
    required HttpClient httpClient,
    required Logger logger,
    Duration? uploadRetryThrottle,
  }) : httpAddress = serviceProtocol.httpAddress,
       _client = httpClient,
       _osUtils = osUtils,
       _uploadRetryThrottle = uploadRetryThrottle,
       _logger = logger;

  final HttpClient _client;
  final OperatingSystemUtils _osUtils;
  final Logger _logger;
  final Duration? _uploadRetryThrottle;

  final String fsName;
  final Uri? httpAddress;

  // 3 was chosen to try to limit the variance in the time it takes to execute
  // `await request.close()` since there is a known bug in Dart where it doesn't
  // always return a status code in response to a PUT request:
  // https://github.com/dart-lang/sdk/issues/43525.
  static const int kMaxInFlight = 3;

  int _inFlight = 0;
  late Map<Uri, DevFSContent> _outstanding;
  late Completer<void> _completer;

  @override
  Future<void> write(Map<Uri, DevFSContent> entries, Uri devFSBase, [DevFSWriter? parent]) async {
    try {
      _client.maxConnectionsPerHost = kMaxInFlight;
      _completer = Completer<void>();
      _outstanding = Map<Uri, DevFSContent>.of(entries);
      _scheduleWrites();
      await _completer.future;
    } on SocketException catch (socketException, stackTrace) {
      _logger.printTrace('DevFS sync failed. Lost connection to device: $socketException');
      throw DevFSException('Lost connection to device.', socketException, stackTrace);
    } on Exception catch (exception, stackTrace) {
      _logger.printError('Could not update files on device: $exception');
      throw DevFSException('Sync failed', exception, stackTrace);
    }
  }

  void _scheduleWrites() {
    while ((_inFlight < kMaxInFlight) && (!_completer.isCompleted) && _outstanding.isNotEmpty) {
      final Uri deviceUri = _outstanding.keys.first;
      final DevFSContent content = _outstanding.remove(deviceUri)!;
      _startWrite(deviceUri, content, retry: 10);
      _inFlight += 1;
    }
    if ((_inFlight == 0) && (!_completer.isCompleted) && _outstanding.isEmpty) {
      _completer.complete();
    }
  }

  Future<void> _startWrite(Uri deviceUri, DevFSContent content, {int retry = 0}) async {
    while (true) {
      try {
        final HttpClientRequest request = await _client.putUrl(httpAddress!);
        request.headers.removeAll(HttpHeaders.acceptEncodingHeader);
        request.headers.add('dev_fs_name', fsName);
        request.headers.add('dev_fs_uri_b64', base64.encode(utf8.encode('$deviceUri')));
        final Stream<List<int>> contents = content.contentsAsCompressedStream(_osUtils);
        await request.addStream(contents);
        // Once the bug in Dart is solved we can remove the timeout
        // (https://github.com/dart-lang/sdk/issues/43525).
        try {
          final HttpClientResponse response = await request.close().timeout(
            const Duration(seconds: 60),
          );
          response.listen(
            (_) {},
            onError: (dynamic error) {
              _logger.printTrace('error: $error');
            },
            cancelOnError: true,
          );
        } on TimeoutException {
          request.abort();
          // This should throw "HttpException: Request has been aborted".
          await request.done;
          // Just to be safe we rethrow the TimeoutException.
          rethrow;
        }
        break;
      } on Exception catch (error, trace) {
        if (!_completer.isCompleted) {
          _logger.printTrace('Error writing "$deviceUri" to DevFS: $error');
          if (retry > 0) {
            retry--;
            _logger.printTrace('trying again in a few - $retry more attempts left');
            await Future<void>.delayed(_uploadRetryThrottle ?? const Duration(milliseconds: 500));
            continue;
          }
          _completer.completeError(error, trace);
        }
      }
    }
    _inFlight -= 1;
    _scheduleWrites();
  }
}

// Basic statistics for DevFS update operation.
class UpdateFSReport {
  UpdateFSReport({
    bool success = false,
    int invalidatedSourcesCount = 0,
    int syncedBytes = 0,
    int scannedSourcesCount = 0,
    Duration compileDuration = Duration.zero,
    Duration transferDuration = Duration.zero,
    Duration findInvalidatedDuration = Duration.zero,
  }) : _success = success,
       _invalidatedSourcesCount = invalidatedSourcesCount,
       _syncedBytes = syncedBytes,
       _scannedSourcesCount = scannedSourcesCount,
       _compileDuration = compileDuration,
       _transferDuration = transferDuration,
       _findInvalidatedDuration = findInvalidatedDuration;

  bool get success => _success;
  int get invalidatedSourcesCount => _invalidatedSourcesCount;
  int get syncedBytes => _syncedBytes;
  int get scannedSourcesCount => _scannedSourcesCount;
  Duration get compileDuration => _compileDuration;
  Duration get transferDuration => _transferDuration;
  Duration get findInvalidatedDuration => _findInvalidatedDuration;

  bool _success;
  int _invalidatedSourcesCount;
  int _syncedBytes;
  int _scannedSourcesCount;
  Duration _compileDuration;
  Duration _transferDuration;
  Duration _findInvalidatedDuration;

  void incorporateResults(UpdateFSReport report) {
    if (!report._success) {
      _success = false;
    }
    _invalidatedSourcesCount += report._invalidatedSourcesCount;
    _syncedBytes += report._syncedBytes;
    _scannedSourcesCount += report._scannedSourcesCount;
    _compileDuration += report._compileDuration;
    _transferDuration += report._transferDuration;
    _findInvalidatedDuration += report._findInvalidatedDuration;
  }
}

class DevFS {
  /// Create a [DevFS] named [fsName] for the local files in [rootDirectory].
  ///
  /// Failed uploads are retried after [uploadRetryThrottle] duration, defaults to 500ms.
  DevFS(
    FlutterVmService serviceProtocol,
    this.fsName,
    this.rootDirectory, {
    required OperatingSystemUtils osUtils,
    required Logger logger,
    required FileSystem fileSystem,
    required ProcessManager processManager,
    required Artifacts artifacts,
    required BuildMode buildMode,
    HttpClient? httpClient,
    Duration? uploadRetryThrottle,
    StopwatchFactory stopwatchFactory = const StopwatchFactory(),
    Config? config,
  }) : _vmService = serviceProtocol,
       _logger = logger,
       _fileSystem = fileSystem,
       _httpWriter = _DevFSHttpWriter(
         fsName,
         serviceProtocol,
         osUtils: osUtils,
         logger: logger,
         uploadRetryThrottle: uploadRetryThrottle,
         httpClient:
             httpClient ??
             ((context.get<HttpClientFactory>() == null)
                 ? HttpClient()
                 : context.get<HttpClientFactory>()!()),
       ),
       _stopwatchFactory = stopwatchFactory,
       _config = config,
       _assetTransformer = DevelopmentAssetTransformer(
         transformer: AssetTransformer(
           processManager: processManager,
           fileSystem: fileSystem,
           dartBinaryPath: artifacts.getArtifactPath(Artifact.engineDartBinary),
           buildMode: buildMode,
         ),
         fileSystem: fileSystem,
         logger: logger,
       );

  final FlutterVmService _vmService;
  final _DevFSHttpWriter _httpWriter;
  final Logger _logger;
  final FileSystem _fileSystem;
  final StopwatchFactory _stopwatchFactory;
  final Config? _config;
  final DevelopmentAssetTransformer _assetTransformer;

  final String fsName;
  final Directory rootDirectory;
  final Set<String> assetPathsToEvict = <String>{};
  final Set<String> shaderPathsToEvict = <String>{};
  final Set<String> scenePathsToEvict = <String>{};

  // A flag to indicate whether we have called `setAssetDirectory` on the target device.
  bool hasSetAssetDirectory = false;

  /// Whether the font manifest was uploaded during [update].
  bool didUpdateFontManifest = false;

  List<Uri> sources = <Uri>[];
  DateTime? lastCompiled;
  DateTime? _previousCompiled;
  PackageConfig? lastPackageConfig;

  Uri? _baseUri;
  Uri? get baseUri => _baseUri;

  Uri deviceUriToHostUri(Uri deviceUri) {
    final String deviceUriString = deviceUri.toString();
    final String baseUriString = baseUri.toString();
    if (deviceUriString.startsWith(baseUriString)) {
      final String deviceUriSuffix = deviceUriString.substring(baseUriString.length);
      return rootDirectory.uri.resolve(deviceUriSuffix);
    }
    return deviceUri;
  }

  Future<Uri> create() async {
    _logger.printTrace('DevFS: Creating new filesystem on the device ($_baseUri)');
    try {
      final vm_service.Response response = await _vmService.createDevFS(fsName);
      _baseUri = Uri.parse(response.json!['uri'] as String);
    } on vm_service.RPCError catch (rpcException) {
      if (rpcException.code == vm_service.RPCErrorKind.kServiceDisappeared.code ||
          rpcException.message.contains('Service connection disposed')) {
        // This can happen if the device has been disconnected, so translate to
        // a DevFSException, which the caller will handle.
        throw DevFSException('Service disconnected', rpcException);
      }
      // 1001 is kFileSystemAlreadyExists in //dart/runtime/vm/json_stream.h
      if (rpcException.code != 1001) {
        // Other RPCErrors are unexpected. Rethrow so it will hit crash
        // logging.
        rethrow;
      }
      _logger.printTrace('DevFS: Creating failed. Destroying and trying again');
      await destroy();
      final vm_service.Response response = await _vmService.createDevFS(fsName);
      _baseUri = Uri.parse(response.json!['uri'] as String);
    }
    _logger.printTrace('DevFS: Created new filesystem on the device ($_baseUri)');
    return _baseUri!;
  }

  Future<void> destroy() async {
    _logger.printTrace('DevFS: Deleting filesystem on the device ($_baseUri)');
    await _vmService.deleteDevFS(fsName);
    _logger.printTrace('DevFS: Deleted filesystem on the device ($_baseUri)');
  }

  /// Mark the [lastCompiled] time to the previous successful compile.
  ///
  /// Sometimes a hot reload will be rejected by the VM due to a change in the
  /// structure of the code not supporting the hot reload. In these cases,
  /// the best resolution is a hot restart. However, the resident runner
  /// will not recognize this file as having been changed since the delta
  /// will already have been accepted. Instead, reset the compile time so
  /// that the last updated files are included in subsequent compilations until
  /// a reload is accepted.
  void resetLastCompiled() {
    lastCompiled = _previousCompiled;
  }

  /// Updates files on the device.
  ///
  /// Returns the number of bytes synced.
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
    File? dartPluginRegistrant,
  }) async {
    final DateTime candidateCompileTime = DateTime.now();
    didUpdateFontManifest = false;
    lastPackageConfig = packageConfig;

    // Update modified files
    final Map<Uri, DevFSContent> dirtyEntries = <Uri, DevFSContent>{};
    final List<Future<void>> pendingAssetBuilds = <Future<void>>[];
    bool assetBuildFailed = false;
    int syncedBytes = 0;
    if (fullRestart) {
      generator.reset();
    }
    // On a full restart, or on an initial compile for the attach based workflow,
    // this will produce a full dill. Subsequent invocations will produce incremental
    // dill files that depend on the invalidated files.
    _logger.printTrace('Compiling dart to kernel with ${invalidatedFiles.length} updated files');

    // Await the compiler response after checking if the bundle is updated. This allows the file
    // stating to be done while waiting for the frontend_server response.
    final Stopwatch compileTimer = _stopwatchFactory.createStopwatch('compile')..start();
    final Future<CompilerOutput?> pendingCompilerOutput = generator
        .recompile(
          mainUri,
          invalidatedFiles,
          outputPath: dillOutputPath,
          fs: _fileSystem,
          projectRootPath: rootDirectory.path,
          packageConfig: packageConfig,
          checkDartPluginRegistry: true, // The entry point is assumed not to have changed.
          dartPluginRegistrant: dartPluginRegistrant,
        )
        .then((CompilerOutput? result) {
          compileTimer.stop();
          return result;
        });

    if (bundle != null) {
      // Mark processing of bundle started for testability of starting the compile
      // before processing bundle.
      _logger.printTrace('Processing bundle.');
      // await null to give time for telling the compiler to compile.
      await null;

      // The tool writes the assets into the AssetBundle working dir so that they
      // are in the same location in DevFS and the iOS simulator.
      final String assetDirectory = getAssetBuildDirectory(_config, _fileSystem);
      final String assetBuildDirPrefix = _asUriPath(assetDirectory);
      bundle.entries.forEach((String archivePath, AssetBundleEntry entry) {
        // If the content is backed by a real file, isModified will file stat and return true if
        // it was modified since the last time this was called.
        if (!entry.content.isModified || bundleFirstUpload) {
          return;
        }
        // Modified shaders must be recompiled per-target platform.
        final Uri deviceUri = _fileSystem.path.toUri(
          _fileSystem.path.join(assetDirectory, archivePath),
        );
        if (deviceUri.path.startsWith(assetBuildDirPrefix)) {
          archivePath = deviceUri.path.substring(assetBuildDirPrefix.length);
        }
        // If the font manifest is updated, mark this as true so the hot runner
        // can invoke a service extension to force the engine to reload fonts.
        if (archivePath == _kFontManifest) {
          didUpdateFontManifest = true;
        }
        final AssetKind? kind = bundle.entries[archivePath]?.kind;
        switch (kind) {
          case AssetKind.shader:
            final Future<DevFSContent?> pending = shaderCompiler.recompileShader(entry.content);
            pendingAssetBuilds.add(pending);
            pending.then((DevFSContent? content) {
              if (content == null) {
                assetBuildFailed = true;
                return;
              }
              dirtyEntries[deviceUri] = content;
              syncedBytes += content.size;
              if (!bundleFirstUpload) {
                shaderPathsToEvict.add(archivePath);
              }
            });
          case AssetKind.model:
            if (sceneImporter == null) {
              break;
            }
            final Future<DevFSContent?> pending = sceneImporter.reimportScene(entry.content);
            pendingAssetBuilds.add(pending);
            pending.then((DevFSContent? content) {
              if (content == null) {
                assetBuildFailed = true;
                return;
              }
              dirtyEntries[deviceUri] = content;
              syncedBytes += content.size;
              if (!bundleFirstUpload) {
                scenePathsToEvict.add(archivePath);
              }
            });
          case AssetKind.regular:
          case AssetKind.font:
          case null:
            final Future<DevFSContent?> pending =
                (() async {
                  if (entry.transformers.isEmpty || kind != AssetKind.regular) {
                    return entry.content;
                  }
                  return _assetTransformer.retransformAsset(
                    inputAssetKey: archivePath,
                    inputAssetContent: entry.content,
                    transformerEntries: entry.transformers,
                    workingDirectory: rootDirectory.path,
                  );
                })();

            pendingAssetBuilds.add(pending);
            pending.then((DevFSContent? content) {
              if (content == null) {
                assetBuildFailed = true;
                return;
              }
              dirtyEntries[deviceUri] = content;
              syncedBytes += content.size;
              if (!bundleFirstUpload) {
                assetPathsToEvict.add(archivePath);
              }
            });
        }
      });

      // Mark processing of bundle done for testability of starting the compile
      // before processing bundle.
      _logger.printTrace('Bundle processing done.');
    }
    final CompilerOutput? compilerOutput = await pendingCompilerOutput;
    if (compilerOutput == null || compilerOutput.errorCount > 0) {
      return UpdateFSReport();
    }
    // Only update the last compiled time if we successfully compiled.
    _previousCompiled = lastCompiled;
    lastCompiled = candidateCompileTime;
    // list of sources that needs to be monitored are in [compilerOutput.sources]
    sources = compilerOutput.sources;
    //
    // Don't send full kernel file that would overwrite what VM already
    // started loading from.
    if (!bundleFirstUpload) {
      final String compiledBinary = compilerOutput.outputFilename;
      if (compiledBinary.isNotEmpty) {
        final Uri entryUri = _fileSystem.path.toUri(pathToReload);
        final DevFSFileContent content = DevFSFileContent(_fileSystem.file(compiledBinary));
        syncedBytes += content.size;
        dirtyEntries[entryUri] = content;
      }
    }
    _logger.printTrace('Updating files.');
    final Stopwatch transferTimer = _stopwatchFactory.createStopwatch('transfer')..start();

    await Future.wait(pendingAssetBuilds);
    if (assetBuildFailed) {
      return UpdateFSReport();
    }

    _logger.printTrace('Pending asset builds completed. Writing dirty entries.');
    if (dirtyEntries.isNotEmpty) {
      await (devFSWriter ?? _httpWriter).write(dirtyEntries, _baseUri!, _httpWriter);
    }
    transferTimer.stop();
    _logger.printTrace('DevFS: Sync finished');
    return UpdateFSReport(
      success: true,
      syncedBytes: syncedBytes,
      invalidatedSourcesCount: invalidatedFiles.length,
      compileDuration: compileTimer.elapsed,
      transferDuration: transferTimer.elapsed,
    );
  }

  /// Converts a platform-specific file path to a platform-independent URL path.
  String _asUriPath(String filePath) => '${_fileSystem.path.toUri(filePath).path}/';
}

/// An implementation of a devFS writer which copies physical files for devices
/// running on the same host.
///
/// DevFS entries which correspond to physical files are copied using [File.copySync],
/// while entries that correspond to arbitrary string/byte values are written from
/// memory.
///
/// Requires that the file system is the same for both the tool and application.
class LocalDevFSWriter implements DevFSWriter {
  LocalDevFSWriter({required FileSystem fileSystem}) : _fileSystem = fileSystem;

  final FileSystem _fileSystem;

  @override
  Future<void> write(Map<Uri, DevFSContent> entries, Uri baseUri, [DevFSWriter? parent]) async {
    try {
      for (final MapEntry<Uri, DevFSContent> entry in entries.entries) {
        final Uri uri = entry.key;
        final DevFSContent devFSContent = entry.value;
        final File destination = _fileSystem.file(baseUri.resolveUri(uri));
        if (!destination.parent.existsSync()) {
          destination.parent.createSync(recursive: true);
        }
        if (devFSContent is DevFSFileContent) {
          final File content = devFSContent.file as File;
          content.copySync(destination.path);
          continue;
        }
        destination.writeAsBytesSync(await devFSContent.contentsAsBytes());
      }
    } on FileSystemException catch (err) {
      throw DevFSException(err.toString());
    }
  }
}
