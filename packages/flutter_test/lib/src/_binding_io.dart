// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:path/path.dart' as path;
import 'package:test_api/scaffolding.dart' as test_package;

import 'binding.dart';

/// Ensure the appropriate test binding is initialized.
TestWidgetsFlutterBinding ensureInitialized([@visibleForTesting Map<String, String>? environment]) {
  environment ??= Platform.environment;
  if (environment.containsKey('FLUTTER_TEST') && environment['FLUTTER_TEST'] != 'false') {
    return AutomatedTestWidgetsFlutterBinding.ensureInitialized();
  }
  return LiveTestWidgetsFlutterBinding.ensureInitialized();
}

/// Setup mocking of the global [HttpClient].
void setupHttpOverrides() {
  HttpOverrides.global = _MockHttpOverrides();
}

/// Setup mocking of platform assets if `UNIT_TEST_ASSETS` is defined.
void mockFlutterAssets() {
  if (!Platform.environment.containsKey('UNIT_TEST_ASSETS')) {
    return;
  }
  final String assetFolderPath = Platform.environment['UNIT_TEST_ASSETS']!;
  assert(Platform.environment['APP_NAME'] != null);
  final String prefix = 'packages/${Platform.environment['APP_NAME']!}/';

  /// Navigation related actions (pop, push, replace) broadcasts these actions via
  /// platform messages.
  TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
    SystemChannels.navigation,
    (MethodCall methodCall) async {
      return null;
    },
  );

  TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
    'flutter/assets',
    (ByteData? message) {
      assert(message != null);
      String key = utf8.decode(message!.buffer.asUint8List());
      File asset = File(path.join(assetFolderPath, key));

      if (!asset.existsSync()) {
        // For tests in package, it will load assets with its own package prefix.
        // In this case, we do a best-effort look up.
        if (!key.startsWith(prefix)) {
          return null;
        }

        key = key.replaceFirst(prefix, '');
        asset = File(path.join(assetFolderPath, key));
        if (!asset.existsSync()) {
          return null;
        }
      }

      final Uint8List encoded = Uint8List.fromList(asset.readAsBytesSync());
      return SynchronousFuture<ByteData>(encoded.buffer.asByteData());
    },
  );
}

/// Provides a default [HttpClient] which always returns empty 400 responses.
///
/// If another [HttpClient] is provided using [HttpOverrides.runZoned], that will
/// take precedence over this provider.
class _MockHttpOverrides extends HttpOverrides {
  bool warningPrinted = false;
  @override
  HttpClient createHttpClient(SecurityContext? context) {
    if (!warningPrinted) {
      test_package.printOnFailure(
        'Warning: At least one test in this suite creates an HttpClient. When '
                'running a test suite that uses TestWidgetsFlutterBinding, all HTTP '
                'requests will return status code 400, and no network request will '
                'actually be made. Any test expecting a real network connection and '
                'status code will fail.\n'
                'To test code that needs an HttpClient, provide your own HttpClient '
                'implementation to the code under test, so that your test can '
                'consistently provide a testable response to the code under test.'
            .split('\n')
            .expand<String>((String line) => debugWordWrap(line, FlutterError.wrapWidth))
            .join('\n'),
      );
      warningPrinted = true;
    }
    return _MockHttpClient();
  }
}

/// A mocked [HttpClient] which always returns a [_MockHttpRequest].
class _MockHttpClient implements HttpClient {
  @override
  bool autoUncompress = true;

  @override
  Duration? connectionTimeout;

  @override
  Duration idleTimeout = const Duration(seconds: 15);

  @override
  int? maxConnectionsPerHost;

  @override
  String? userAgent;

  @override
  void addCredentials(Uri url, String realm, HttpClientCredentials credentials) {}

  @override
  void addProxyCredentials(
    String host,
    int port,
    String realm,
    HttpClientCredentials credentials,
  ) {}

  @override
  Future<ConnectionTask<Socket>> Function(Uri url, String? proxyHost, int? proxyPort)?
  connectionFactory;

  @override
  Future<bool> Function(Uri url, String scheme, String realm)? authenticate;

  @override
  Future<bool> Function(String host, int port, String scheme, String realm)? authenticateProxy;

  @override
  bool Function(X509Certificate cert, String host, int port)? badCertificateCallback;

  @override
  void Function(String line)? keyLog;

  @override
  void close({bool force = false}) {}

  @override
  Future<HttpClientRequest> delete(String host, int port, String path) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  Future<HttpClientRequest> deleteUrl(Uri url) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  String Function(Uri url)? findProxy;

  @override
  Future<HttpClientRequest> get(String host, int port, String path) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  Future<HttpClientRequest> getUrl(Uri url) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  Future<HttpClientRequest> head(String host, int port, String path) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  Future<HttpClientRequest> headUrl(Uri url) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  Future<HttpClientRequest> open(String method, String host, int port, String path) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  Future<HttpClientRequest> openUrl(String method, Uri url) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  Future<HttpClientRequest> patch(String host, int port, String path) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  Future<HttpClientRequest> patchUrl(Uri url) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  Future<HttpClientRequest> post(String host, int port, String path) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  Future<HttpClientRequest> postUrl(Uri url) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  Future<HttpClientRequest> put(String host, int port, String path) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }

  @override
  Future<HttpClientRequest> putUrl(Uri url) {
    return Future<HttpClientRequest>.value(_MockHttpRequest());
  }
}

/// A mocked [HttpClientRequest] which always returns a [_MockHttpResponse].
class _MockHttpRequest implements HttpClientRequest {
  @override
  bool bufferOutput = true;

  @override
  int contentLength = -1;

  @override
  late Encoding encoding;

  @override
  bool followRedirects = true;

  @override
  final HttpHeaders headers = _MockHttpHeaders();

  @override
  void add(List<int> data) {}

  @override
  void addError(Object error, [StackTrace? stackTrace]) {}

  @override
  Future<void> addStream(Stream<List<int>> stream) {
    return Future<void>.value();
  }

  @override
  Future<HttpClientResponse> close() {
    return Future<HttpClientResponse>.value(_MockHttpResponse());
  }

  @override
  void abort([Object? exception, StackTrace? stackTrace]) {}

  @override
  HttpConnectionInfo? get connectionInfo => null;

  @override
  List<Cookie> get cookies => <Cookie>[];

  @override
  Future<HttpClientResponse> get done async => _MockHttpResponse();

  @override
  Future<void> flush() {
    return Future<void>.value();
  }

  @override
  int maxRedirects = 5;

  @override
  String get method => '';

  @override
  bool persistentConnection = true;

  @override
  Uri get uri => Uri();

  @override
  void write(Object? obj) {}

  @override
  void writeAll(Iterable<dynamic> objects, [String separator = '']) {}

  @override
  void writeCharCode(int charCode) {}

  @override
  void writeln([Object? obj = '']) {}
}

/// A mocked [HttpClientResponse] which is empty and has a [statusCode] of 400.
// TODO(tvolkert): Change to `extends Stream<Uint8List>` once
// https://dart-review.googlesource.com/c/sdk/+/104525 is rolled into the framework.
class _MockHttpResponse implements HttpClientResponse {
  final Stream<Uint8List> _delegate = Stream<Uint8List>.fromIterable(
    const Iterable<Uint8List>.empty(),
  );

  @override
  final HttpHeaders headers = _MockHttpHeaders();

  @override
  X509Certificate? get certificate => null;

  @override
  HttpConnectionInfo? get connectionInfo => null;

  @override
  int get contentLength => -1;

  @override
  HttpClientResponseCompressionState get compressionState {
    return HttpClientResponseCompressionState.decompressed;
  }

  @override
  List<Cookie> get cookies => <Cookie>[];

  @override
  Future<Socket> detachSocket() {
    return Future<Socket>.error(UnsupportedError('Mocked response'));
  }

  @override
  bool get isRedirect => false;

  @override
  StreamSubscription<Uint8List> listen(
    void Function(Uint8List event)? onData, {
    Function? onError,
    void Function()? onDone,
    bool? cancelOnError,
  }) {
    return const Stream<Uint8List>.empty().listen(
      onData,
      onError: onError,
      onDone: onDone,
      cancelOnError: cancelOnError,
    );
  }

  @override
  bool get persistentConnection => false;

  @override
  String get reasonPhrase => '';

  @override
  Future<HttpClientResponse> redirect([String? method, Uri? url, bool? followLoops]) {
    return Future<HttpClientResponse>.error(UnsupportedError('Mocked response'));
  }

  @override
  List<RedirectInfo> get redirects => <RedirectInfo>[];

  @override
  int get statusCode => 400;

  @override
  Future<bool> any(bool Function(Uint8List element) test) {
    return _delegate.any(test);
  }

  @override
  Stream<Uint8List> asBroadcastStream({
    void Function(StreamSubscription<Uint8List> subscription)? onListen,
    void Function(StreamSubscription<Uint8List> subscription)? onCancel,
  }) {
    return _delegate.asBroadcastStream(onListen: onListen, onCancel: onCancel);
  }

  @override
  Stream<E> asyncExpand<E>(Stream<E>? Function(Uint8List event) convert) {
    return _delegate.asyncExpand<E>(convert);
  }

  @override
  Stream<E> asyncMap<E>(FutureOr<E> Function(Uint8List event) convert) {
    return _delegate.asyncMap<E>(convert);
  }

  @override
  Stream<R> cast<R>() {
    return _delegate.cast<R>();
  }

  @override
  Future<bool> contains(Object? needle) {
    return _delegate.contains(needle);
  }

  @override
  Stream<Uint8List> distinct([bool Function(Uint8List previous, Uint8List next)? equals]) {
    return _delegate.distinct(equals);
  }

  @override
  Future<E> drain<E>([E? futureValue]) {
    return _delegate.drain<E>(futureValue);
  }

  @override
  Future<Uint8List> elementAt(int index) {
    return _delegate.elementAt(index);
  }

  @override
  Future<bool> every(bool Function(Uint8List element) test) {
    return _delegate.every(test);
  }

  @override
  Stream<S> expand<S>(Iterable<S> Function(Uint8List element) convert) {
    return _delegate.expand(convert);
  }

  @override
  Future<Uint8List> get first => _delegate.first;

  @override
  Future<Uint8List> firstWhere(
    bool Function(Uint8List element) test, {
    List<int> Function()? orElse,
  }) {
    return _delegate.firstWhere(
      test,
      orElse:
          orElse == null
              ? null
              : () {
                return Uint8List.fromList(orElse());
              },
    );
  }

  @override
  Future<S> fold<S>(S initialValue, S Function(S previous, Uint8List element) combine) {
    return _delegate.fold<S>(initialValue, combine);
  }

  @override
  Future<dynamic> forEach(void Function(Uint8List element) action) {
    return _delegate.forEach(action);
  }

  @override
  Stream<Uint8List> handleError(Function onError, {bool Function(dynamic error)? test}) {
    return _delegate.handleError(onError, test: test);
  }

  @override
  bool get isBroadcast => _delegate.isBroadcast;

  @override
  Future<bool> get isEmpty => _delegate.isEmpty;

  @override
  Future<String> join([String separator = '']) {
    return _delegate.join(separator);
  }

  @override
  Future<Uint8List> get last => _delegate.last;

  @override
  Future<Uint8List> lastWhere(
    bool Function(Uint8List element) test, {
    List<int> Function()? orElse,
  }) {
    return _delegate.lastWhere(
      test,
      orElse:
          orElse == null
              ? null
              : () {
                return Uint8List.fromList(orElse());
              },
    );
  }

  @override
  Future<int> get length => _delegate.length;

  @override
  Stream<S> map<S>(S Function(Uint8List event) convert) {
    return _delegate.map<S>(convert);
  }

  @override
  Future<dynamic> pipe(StreamConsumer<List<int>> streamConsumer) {
    return _delegate.cast<List<int>>().pipe(streamConsumer);
  }

  @override
  Future<Uint8List> reduce(List<int> Function(Uint8List previous, Uint8List element) combine) {
    return _delegate.reduce((Uint8List previous, Uint8List element) {
      return Uint8List.fromList(combine(previous, element));
    });
  }

  @override
  Future<Uint8List> get single => _delegate.single;

  @override
  Future<Uint8List> singleWhere(
    bool Function(Uint8List element) test, {
    List<int> Function()? orElse,
  }) {
    return _delegate.singleWhere(
      test,
      orElse:
          orElse == null
              ? null
              : () {
                return Uint8List.fromList(orElse());
              },
    );
  }

  @override
  Stream<Uint8List> skip(int count) {
    return _delegate.skip(count);
  }

  @override
  Stream<Uint8List> skipWhile(bool Function(Uint8List element) test) {
    return _delegate.skipWhile(test);
  }

  @override
  Stream<Uint8List> take(int count) {
    return _delegate.take(count);
  }

  @override
  Stream<Uint8List> takeWhile(bool Function(Uint8List element) test) {
    return _delegate.takeWhile(test);
  }

  @override
  Stream<Uint8List> timeout(
    Duration timeLimit, {
    void Function(EventSink<Uint8List> sink)? onTimeout,
  }) {
    return _delegate.timeout(timeLimit, onTimeout: onTimeout);
  }

  @override
  Future<List<Uint8List>> toList() {
    return _delegate.toList();
  }

  @override
  Future<Set<Uint8List>> toSet() {
    return _delegate.toSet();
  }

  @override
  Stream<S> transform<S>(StreamTransformer<List<int>, S> streamTransformer) {
    return _delegate.cast<List<int>>().transform<S>(streamTransformer);
  }

  @override
  Stream<Uint8List> where(bool Function(Uint8List event) test) {
    return _delegate.where(test);
  }
}

/// A mocked [HttpHeaders] that ignores all writes.
class _MockHttpHeaders implements HttpHeaders {
  @override
  List<String>? operator [](String name) => <String>[];

  @override
  void add(String name, Object value, {bool preserveHeaderCase = false}) {}

  @override
  late bool chunkedTransferEncoding;

  @override
  void clear() {}

  @override
  int contentLength = -1;

  @override
  ContentType? contentType;

  @override
  DateTime? date;

  @override
  DateTime? expires;

  @override
  void forEach(void Function(String name, List<String> values) f) {}

  @override
  String? host;

  @override
  DateTime? ifModifiedSince;

  @override
  void noFolding(String name) {}

  @override
  late bool persistentConnection;

  @override
  int? port;

  @override
  void remove(String name, Object value) {}

  @override
  void removeAll(String name) {}

  @override
  void set(String name, Object value, {bool preserveHeaderCase = false}) {}

  @override
  String? value(String name) => null;
}
