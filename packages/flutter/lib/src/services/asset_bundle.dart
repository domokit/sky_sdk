// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';

import 'binding.dart';

export 'dart:typed_data' show ByteData;
export 'dart:ui' show ImmutableBuffer;

/// A collection of resources used by the application.
///
/// Asset bundles contain resources, such as images and strings, that can be
/// used by an application. Access to these resources is asynchronous so that
/// they can be transparently loaded over a network (e.g., from a
/// [NetworkAssetBundle]) or from the local file system without blocking the
/// application's user interface.
///
/// Applications have a [rootBundle], which contains the resources that were
/// packaged with the application when it was built. To add resources to the
/// [rootBundle] for your application, add them to the `assets` subsection of
/// the `flutter` section of your application's `pubspec.yaml` manifest.
///
/// For example:
///
/// ```yaml
/// name: my_awesome_application
/// flutter:
///   assets:
///    - images/hamilton.jpeg
///    - images/lafayette.jpeg
/// ```
///
/// Rather than accessing the [rootBundle] global static directly, consider
/// obtaining the [AssetBundle] for the current [BuildContext] using
/// [DefaultAssetBundle.of]. This layer of indirection lets ancestor widgets
/// substitute a different [AssetBundle] (e.g., for testing or localization) at
/// runtime rather than directly replying upon the [rootBundle] created at build
/// time. For convenience, the [WidgetsApp] or [MaterialApp] widget at the top
/// of the widget hierarchy configures the [DefaultAssetBundle] to be the
/// [rootBundle].
///
/// See also:
///
///  * [DefaultAssetBundle]
///  * [NetworkAssetBundle]
///  * [rootBundle]
abstract class AssetBundle {
  /// Retrieve a binary resource from the asset bundle as a data stream.
  ///
  /// Throws an exception if the asset is not found.
  ///
  /// The returned [ByteData] can be converted to a [Uint8List] (a list of bytes)
  /// using [Uint8List.sublistView]. Lists of bytes can be used with APIs that
  /// accept [Uint8List] objects, such as [decodeImageFromList], as well as any
  /// API that accepts a [List<int>], such as [File.writeAsBytes] or
  /// [Utf8Codec.decode] (accessible via [utf8]).
  Future<ByteData> load(String key);

  /// Retrieve a binary resource from the asset bundle as an immutable
  /// buffer.
  ///
  /// Throws an exception if the asset is not found.
  Future<ui.ImmutableBuffer> loadBuffer(String key) async {
    final ByteData data = await load(key);
    return ui.ImmutableBuffer.fromUint8List(Uint8List.sublistView(data));
  }

  /// Retrieve a string from the asset bundle.
  ///
  /// Throws an exception if the asset is not found.
  ///
  /// If the `cache` argument is set to false, then the data will not be
  /// cached, and reading the data may bypass the cache. This is useful if the
  /// caller is going to be doing its own caching. (It might not be cached if
  /// it's set to true either, depending on the asset bundle implementation.)
  ///
  /// The function expects the stored string to be UTF-8-encoded as
  /// [Utf8Codec] will be used for decoding the string. If the string is
  /// larger than 50 KB, the decoding process is delegated to an
  /// isolate to avoid jank on the main thread.
  Future<String> loadString(String key, {bool cache = true}) async {
    final ByteData data = await load(key);
    // 50 KB of data should take 2-3 ms to parse on a Moto G4, and about 400 μs
    // on a Pixel 4. On the web we can't bail to isolates, though...
    if (data.lengthInBytes < 50 * 1024 || kIsWeb) {
      return utf8.decode(Uint8List.sublistView(data));
    }
    // For strings larger than 50 KB, run the computation in an isolate to
    // avoid causing main thread jank.
    return compute(_utf8decode, data, debugLabel: 'UTF8 decode for "$key"');
  }

  static String _utf8decode(ByteData data) {
    return utf8.decode(Uint8List.sublistView(data));
  }

  /// Retrieve a string from the asset bundle, parse it with the given function,
  /// and return that function's result.
  ///
  /// The result is not cached by the default implementation; the parser is run
  /// each time the resource is fetched. However, some subclasses may implement
  /// caching (notably, subclasses of [CachingAssetBundle]).
  Future<T> loadStructuredData<T>(String key, Future<T> Function(String value) parser) async {
    return parser(await loadString(key));
  }

  /// Retrieve [ByteData] from the asset bundle, parse it with the given function,
  /// and return that function's result.
  ///
  /// The result is not cached by the default implementation; the parser is run
  /// each time the resource is fetched. However, some subclasses may implement
  /// caching (notably, subclasses of [CachingAssetBundle]).
  Future<T> loadStructuredBinaryData<T>(
    String key,
    FutureOr<T> Function(ByteData data) parser,
  ) async {
    return parser(await load(key));
  }

  /// If this is a caching asset bundle, and the given key describes a cached
  /// asset, then evict the asset from the cache so that the next time it is
  /// loaded, the cache will be reread from the asset bundle.
  void evict(String key) {}

  /// If this is a caching asset bundle, clear all cached data.
  void clear() {}

  @override
  String toString() => '${describeIdentity(this)}()';
}

/// An [AssetBundle] that loads resources over the network.
///
/// This asset bundle does not cache any resources, though the underlying
/// network stack may implement some level of caching itself.
class NetworkAssetBundle extends AssetBundle {
  /// Creates a network asset bundle that resolves asset keys as URLs relative
  /// to the given base URL.
  NetworkAssetBundle(Uri baseUrl) : _baseUrl = baseUrl, _httpClient = HttpClient();

  final Uri _baseUrl;
  final HttpClient _httpClient;

  Uri _urlFromKey(String key) => _baseUrl.resolve(key);

  @override
  Future<ByteData> load(String key) async {
    final HttpClientRequest request = await _httpClient.getUrl(_urlFromKey(key));
    final HttpClientResponse response = await request.close();
    if (response.statusCode != HttpStatus.ok) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        _errorSummaryWithKey(key),
        IntProperty('HTTP status code', response.statusCode),
      ]);
    }
    final Uint8List bytes = await consolidateHttpClientResponseBytes(response);
    return ByteData.sublistView(bytes);
  }

  // TODO(ianh): Once the underlying network logic learns about caching, we
  // should implement evict().

  @override
  String toString() => '${describeIdentity(this)}($_baseUrl)';
}

/// An [AssetBundle] that permanently caches string and structured resources
/// that have been fetched.
///
/// Strings (for [loadString] and [loadStructuredData]) are decoded as UTF-8.
/// Data that is cached is cached for the lifetime of the asset bundle
/// (typically the lifetime of the application).
///
/// Binary resources (from [load]) are not cached.
abstract class CachingAssetBundle extends AssetBundle {
  // TODO(ianh): Replace this with an intelligent cache, see https://github.com/flutter/flutter/issues/3568
  final Map<String, Future<String>> _stringCache = <String, Future<String>>{};
  final Map<String, Future<dynamic>> _structuredDataCache = <String, Future<dynamic>>{};
  final Map<String, Future<dynamic>> _structuredBinaryDataCache = <String, Future<dynamic>>{};

  @override
  Future<String> loadString(String key, {bool cache = true}) {
    if (cache) {
      return _stringCache.putIfAbsent(key, () => super.loadString(key));
    }
    return super.loadString(key);
  }

  /// Retrieve a string from the asset bundle, parse it with the given function,
  /// and return the function's result.
  ///
  /// The result of parsing the string is cached (the string itself is not,
  /// unless you also fetch it with [loadString]). For any given `key`, the
  /// `parser` is only run the first time.
  ///
  /// Once the value has been successfully parsed, the future returned by this
  /// function for subsequent calls will be a [SynchronousFuture], which
  /// resolves its callback synchronously.
  ///
  /// Failures are not cached, and are returned as [Future]s with errors.
  @override
  Future<T> loadStructuredData<T>(String key, Future<T> Function(String value) parser) {
    if (_structuredDataCache.containsKey(key)) {
      return _structuredDataCache[key]! as Future<T>;
    }
    // loadString can return a SynchronousFuture in certain cases, like in the
    // flutter_test framework. So, we need to support both async and sync flows.
    Completer<T>? completer; // For async flow.
    Future<T>? synchronousResult; // For sync flow.
    loadString(key, cache: false)
        .then<T>(parser)
        .then<void>(
          (T value) {
            synchronousResult = SynchronousFuture<T>(value);
            _structuredDataCache[key] = synchronousResult!;
            // We already returned from the loadStructuredData function, which means
            // we are in the asynchronous mode. Pass the value to the completer. The
            // completer's future is what we returned.
            completer?.complete(value);
          },
          onError: (Object error, StackTrace stack) {
            assert(completer != null, 'unexpected synchronous failure');
            // Either loading or parsing failed. We must report the error back to the
            // caller and anyone waiting on this call. We clear the cache for this
            // key, however, because we want future attempts to try again.
            _structuredDataCache.remove(key);
            completer!.completeError(error, stack);
          },
        );
    if (synchronousResult != null) {
      // The above code ran synchronously. We can synchronously return the result.
      return synchronousResult!;
    }
    // The code above hasn't yet run its "then" handler yet. Let's prepare a
    // completer for it to use when it does run.
    completer = Completer<T>();
    _structuredDataCache[key] = completer.future;
    return completer.future;
  }

  /// Retrieve bytedata from the asset bundle, parse it with the given function,
  /// and return the function's result.
  ///
  /// The result of parsing the bytedata is cached (the bytedata itself is not).
  /// For any given `key`, the `parser` is only run the first time.
  ///
  /// Once the value has been successfully parsed, the future returned by this
  /// function for subsequent calls will be a [SynchronousFuture], which
  /// resolves its callback synchronously.
  ///
  /// Failures are not cached, and are returned as [Future]s with errors.
  @override
  Future<T> loadStructuredBinaryData<T>(String key, FutureOr<T> Function(ByteData data) parser) {
    if (_structuredBinaryDataCache.containsKey(key)) {
      return _structuredBinaryDataCache[key]! as Future<T>;
    }
    // load can return a SynchronousFuture in certain cases, like in the
    // flutter_test framework. So, we need to support both async and sync flows.
    Completer<T>? completer; // For async flow.
    Future<T>? synchronousResult; // For sync flow.
    load(key)
        .then<T>(parser)
        .then<void>(
          (T value) {
            synchronousResult = SynchronousFuture<T>(value);
            _structuredBinaryDataCache[key] = synchronousResult!;
            // The load and parse operation ran asynchronously. We already returned
            // from the loadStructuredBinaryData function and therefore the caller
            // was given the future of the completer.
            completer?.complete(value);
          },
          onError: (Object error, StackTrace stack) {
            assert(completer != null, 'unexpected synchronous failure');
            // Either loading or parsing failed. We must report the error back to the
            // caller and anyone waiting on this call. We clear the cache for this
            // key, however, because we want future attempts to try again.
            _structuredBinaryDataCache.remove(key);
            completer!.completeError(error, stack);
          },
        );
    if (synchronousResult != null) {
      // The above code ran synchronously. We can synchronously return the result.
      return synchronousResult!;
    }
    // Since the above code is being run asynchronously and thus hasn't run its
    // `then` handler yet, we'll return a completer that will be completed
    // when the handler does run.
    completer = Completer<T>();
    _structuredBinaryDataCache[key] = completer.future;
    return completer.future;
  }

  @override
  void evict(String key) {
    _stringCache.remove(key);
    _structuredDataCache.remove(key);
    _structuredBinaryDataCache.remove(key);
  }

  @override
  void clear() {
    _stringCache.clear();
    _structuredDataCache.clear();
    _structuredBinaryDataCache.clear();
  }

  @override
  Future<ui.ImmutableBuffer> loadBuffer(String key) async {
    final ByteData data = await load(key);
    return ui.ImmutableBuffer.fromUint8List(Uint8List.sublistView(data));
  }
}

/// An [AssetBundle] that loads resources using platform messages.
class PlatformAssetBundle extends CachingAssetBundle {
  @override
  Future<ByteData> load(String key) {
    final Uint8List encoded = utf8.encode(Uri(path: Uri.encodeFull(key)).path);
    final Future<ByteData>? future = ServicesBinding.instance.defaultBinaryMessenger
        .send('flutter/assets', ByteData.sublistView(encoded))
        ?.then((ByteData? asset) {
          if (asset == null) {
            throw FlutterError.fromParts(<DiagnosticsNode>[
              _errorSummaryWithKey(key),
              ErrorDescription('The asset does not exist or has empty data.'),
            ]);
          }
          return asset;
        });
    if (future == null) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        _errorSummaryWithKey(key),
        ErrorDescription('The asset does not exist or has empty data.'),
      ]);
    }
    return future;
  }

  @override
  Future<ui.ImmutableBuffer> loadBuffer(String key) async {
    if (kIsWeb) {
      final ByteData bytes = await load(key);
      return ui.ImmutableBuffer.fromUint8List(Uint8List.sublistView(bytes));
    }
    bool debugUsePlatformChannel = false;
    assert(() {
      // dart:io is safe to use here since we early return for web
      // above. If that code is changed, this needs to be guarded on
      // web presence. Override how assets are loaded in tests so that
      // the old loader behavior that allows tests to load assets from
      // the current package using the package prefix.
      if (Platform.environment.containsKey('UNIT_TEST_ASSETS')) {
        debugUsePlatformChannel = true;
      }
      return true;
    }());
    if (debugUsePlatformChannel) {
      final ByteData bytes = await load(key);
      return ui.ImmutableBuffer.fromUint8List(Uint8List.sublistView(bytes));
    }
    try {
      return await ui.ImmutableBuffer.fromAsset(key);
    } on Exception catch (e) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        _errorSummaryWithKey(key),
        ErrorDescription(e.toString()),
      ]);
    }
  }
}

AssetBundle _initRootBundle() {
  return PlatformAssetBundle();
}

ErrorSummary _errorSummaryWithKey(String key) {
  return ErrorSummary('Unable to load asset: "$key".');
}

/// The [AssetBundle] from which this application was loaded.
///
/// The [rootBundle] contains the resources that were packaged with the
/// application when it was built. To add resources to the [rootBundle] for your
/// application, add them to the `assets` subsection of the `flutter` section of
/// your application's `pubspec.yaml` manifest.
///
/// For example:
///
/// ```yaml
/// name: my_awesome_application
/// flutter:
///   assets:
///    - images/hamilton.jpeg
///    - images/lafayette.jpeg
/// ```
///
/// Rather than using [rootBundle] directly, consider obtaining the
/// [AssetBundle] for the current [BuildContext] using [DefaultAssetBundle.of].
/// This layer of indirection lets ancestor widgets substitute a different
/// [AssetBundle] at runtime (e.g., for testing or localization) rather than
/// directly replying upon the [rootBundle] created at build time. For
/// convenience, the [WidgetsApp] or [MaterialApp] widget at the top of the
/// widget hierarchy configures the [DefaultAssetBundle] to be the [rootBundle].
///
/// See also:
///
///  * [DefaultAssetBundle]
///  * [NetworkAssetBundle]
final AssetBundle rootBundle = _initRootBundle();
