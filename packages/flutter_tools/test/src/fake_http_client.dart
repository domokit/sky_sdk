// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';

import 'dart:io';

import 'package:collection/collection.dart';

/// The HTTP verb for a [FakeRequest].
enum HttpMethod { get, put, delete, post, patch, head }

HttpMethod _fromMethodString(String value) {
  return switch (value.toLowerCase()) {
    'get' => HttpMethod.get,
    'put' => HttpMethod.put,
    'delete' => HttpMethod.delete,
    'post' => HttpMethod.post,
    'patch' => HttpMethod.patch,
    'head' => HttpMethod.head,
    _ => throw StateError('Unrecognized HTTP method $value'),
  };
}

String _toMethodString(HttpMethod method) {
  return switch (method) {
    HttpMethod.get => 'GET',
    HttpMethod.put => 'PUT',
    HttpMethod.delete => 'DELETE',
    HttpMethod.post => 'POST',
    HttpMethod.patch => 'PATCH',
    HttpMethod.head => 'HEAD',
  };
}

/// Create a fake request that configures the [FakeHttpClient] to respond
/// with the provided [response].
///
/// By default, returns a response with a 200 OK status code and an
/// empty response. If [responseError] is non-null, will throw this instead
/// of returning the response when closing the request.
class FakeRequest {
  const FakeRequest(
    this.uri, {
    this.method = HttpMethod.get,
    this.response = FakeResponse.empty,
    this.responseError,
    this.body,
  });

  final Uri uri;
  final HttpMethod method;
  final FakeResponse response;
  final Object? responseError;
  final List<int>? body;

  @override
  String toString() => 'Request{${_toMethodString(method)}, $uri}';
}

/// The response the server will create for a given [FakeRequest].
class FakeResponse {
  const FakeResponse({
    this.statusCode = HttpStatus.ok,
    this.body = const <int>[],
    this.headers = const <String, List<String>>{},
  });

  static const FakeResponse empty = FakeResponse();

  final int statusCode;
  final List<int> body;
  final Map<String, List<String>> headers;
}

/// A fake implementation of the HttpClient used for testing.
///
/// This does not fully implement the HttpClient. If an additional method
/// is actually needed by the test script, then it should be added here
/// instead of in another fake.
class FakeHttpClient implements HttpClient {
  /// Creates an HTTP client that responses to each provided
  /// fake request with the provided fake response.
  ///
  /// This does not enforce any order on the requests, but if multiple
  /// requests match then the first will be selected;
  FakeHttpClient.list(List<FakeRequest> requests) : _requests = requests.toList();

  /// Creates an HTTP client that always returns an empty 200 request.
  FakeHttpClient.any() : _any = true, _requests = <FakeRequest>[];

  bool _any = false;
  final List<FakeRequest> _requests;

  @override
  bool autoUncompress = true;

  @override
  Duration? connectionTimeout;

  @override
  Duration idleTimeout = Duration.zero;

  @override
  int? maxConnectionsPerHost;

  @override
  String? userAgent;

  @override
  void addCredentials(Uri url, String realm, HttpClientCredentials credentials) {
    throw UnimplementedError();
  }

  @override
  void addProxyCredentials(String host, int port, String realm, HttpClientCredentials credentials) {
    throw UnimplementedError();
  }

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
    final Uri uri = Uri(host: host, port: port, path: path);
    return deleteUrl(uri);
  }

  @override
  Future<HttpClientRequest> deleteUrl(Uri url) async {
    return _findRequest(HttpMethod.delete, url, StackTrace.current);
  }

  @override
  String Function(Uri url)? findProxy;

  @override
  Future<HttpClientRequest> get(String host, int port, String path) {
    final Uri uri = Uri(host: host, port: port, path: path);
    return getUrl(uri);
  }

  @override
  Future<HttpClientRequest> getUrl(Uri url) async {
    return _findRequest(HttpMethod.get, url, StackTrace.current);
  }

  @override
  Future<HttpClientRequest> head(String host, int port, String path) {
    final Uri uri = Uri(host: host, port: port, path: path);
    return headUrl(uri);
  }

  @override
  Future<HttpClientRequest> headUrl(Uri url) async {
    return _findRequest(HttpMethod.head, url, StackTrace.current);
  }

  @override
  Future<HttpClientRequest> open(String method, String host, int port, String path) {
    final Uri uri = Uri(host: host, port: port, path: path);
    return openUrl(method, uri);
  }

  @override
  Future<HttpClientRequest> openUrl(String method, Uri url) async {
    return _findRequest(_fromMethodString(method), url, StackTrace.current);
  }

  @override
  Future<HttpClientRequest> patch(String host, int port, String path) {
    final Uri uri = Uri(host: host, port: port, path: path);
    return patchUrl(uri);
  }

  @override
  Future<HttpClientRequest> patchUrl(Uri url) async {
    return _findRequest(HttpMethod.patch, url, StackTrace.current);
  }

  @override
  Future<HttpClientRequest> post(String host, int port, String path) {
    final Uri uri = Uri(host: host, port: port, path: path);
    return postUrl(uri);
  }

  @override
  Future<HttpClientRequest> postUrl(Uri url) async {
    return _findRequest(HttpMethod.post, url, StackTrace.current);
  }

  @override
  Future<HttpClientRequest> put(String host, int port, String path) {
    final Uri uri = Uri(host: host, port: port, path: path);
    return putUrl(uri);
  }

  @override
  Future<HttpClientRequest> putUrl(Uri url) async {
    return _findRequest(HttpMethod.put, url, StackTrace.current);
  }

  int _requestCount = 0;

  _FakeHttpClientRequest _findRequest(HttpMethod method, Uri uri, StackTrace stackTrace) {
    // Ensure the fake client throws similar errors to the real client.
    if (uri.host.isEmpty) {
      throw ArgumentError('No host specified in URI $uri');
    } else if (uri.scheme != 'http' && uri.scheme != 'https') {
      throw ArgumentError("Unsupported scheme '${uri.scheme}' in URI $uri");
    }
    final String methodString = _toMethodString(method);
    if (_any) {
      return _FakeHttpClientRequest(FakeResponse.empty, uri, methodString, null, null, stackTrace);
    }
    FakeRequest? matchedRequest;
    for (final FakeRequest request in _requests) {
      if (request.method == method && request.uri.toString() == uri.toString()) {
        matchedRequest = request;
        break;
      }
    }
    if (matchedRequest == null) {
      throw StateError(
        'Unexpected request for $method to $uri after $_requestCount requests.\n'
        'Pending requests: ${_requests.join(',')}',
      );
    }
    _requestCount += 1;
    _requests.remove(matchedRequest);
    return _FakeHttpClientRequest(
      matchedRequest.response,
      uri,
      methodString,
      matchedRequest.responseError,
      matchedRequest.body,
      stackTrace,
    );
  }
}

class _FakeHttpClientRequest implements HttpClientRequest {
  _FakeHttpClientRequest(
    this._response,
    this._uri,
    this._method,
    this._responseError,
    this._expectedBody,
    this._stackTrace,
  );

  final FakeResponse _response;
  final String _method;
  final Uri _uri;
  final Object? _responseError;
  final List<int> _body = <int>[];
  final List<int>? _expectedBody;
  final StackTrace _stackTrace;

  @override
  bool bufferOutput = true;

  @override
  int contentLength = 0;

  @override
  late Encoding encoding;

  @override
  bool followRedirects = true;

  @override
  int maxRedirects = 5;

  @override
  bool persistentConnection = true;

  @override
  void abort([Object? exception, StackTrace? stackTrace]) {
    throw UnimplementedError();
  }

  @override
  void add(List<int> data) {
    _body.addAll(data);
  }

  @override
  void addError(Object error, [StackTrace? stackTrace]) {}

  @override
  Future<void> addStream(Stream<List<int>> stream) async {
    final Completer<void> completer = Completer<void>();
    stream.listen(_body.addAll, onDone: completer.complete);
    await completer.future;
  }

  @override
  Future<HttpClientResponse> close() async {
    final Completer<void> completer = Completer<void>();
    Timer.run(() {
      if (_expectedBody != null && !const ListEquality<int>().equals(_expectedBody, _body)) {
        completer.completeError(
          StateError(
            'Expected a request with the following body:\n$_expectedBody\n but found:\n$_body',
          ),
          _stackTrace,
        );
      } else {
        completer.complete();
      }
    });
    await completer.future;
    if (_responseError != null) {
      return Future<HttpClientResponse>.error(_responseError);
    }
    return _FakeHttpClientResponse(_response);
  }

  @override
  HttpConnectionInfo get connectionInfo => throw UnimplementedError();

  @override
  List<Cookie> get cookies => throw UnimplementedError();

  @override
  Future<HttpClientResponse> get done => throw UnimplementedError();

  @override
  Future<void> flush() async {}

  @override
  final HttpHeaders headers = _FakeHttpHeaders(<String, List<String>>{});

  @override
  String get method => _method;

  @override
  Uri get uri => _uri;

  @override
  void write(Object? object) {
    _body.addAll(utf8.encode(object.toString()));
  }

  @override
  void writeAll(Iterable<dynamic> objects, [String separator = '']) {
    _body.addAll(utf8.encode(objects.join(separator)));
  }

  @override
  void writeCharCode(int charCode) {
    _body.add(charCode);
  }

  @override
  void writeln([Object? object = '']) {
    _body.addAll(utf8.encode('$object\n'));
  }
}

class _FakeHttpClientResponse extends Stream<List<int>> implements HttpClientResponse {
  _FakeHttpClientResponse(this._response)
    : headers = _FakeHttpHeaders(Map<String, List<String>>.from(_response.headers));

  final FakeResponse _response;

  @override
  X509Certificate get certificate => throw UnimplementedError();

  @override
  HttpClientResponseCompressionState get compressionState => throw UnimplementedError();

  @override
  HttpConnectionInfo get connectionInfo => throw UnimplementedError();

  @override
  int get contentLength => _response.body.length;

  @override
  List<Cookie> get cookies => throw UnimplementedError();

  @override
  Future<Socket> detachSocket() {
    throw UnimplementedError();
  }

  @override
  final HttpHeaders headers;

  @override
  bool get isRedirect => throw UnimplementedError();

  @override
  StreamSubscription<List<int>> listen(
    void Function(List<int> event)? onData, {
    Function? onError,
    void Function()? onDone,
    bool? cancelOnError,
  }) {
    final Stream<List<int>> response = Stream<List<int>>.fromIterable(<List<int>>[_response.body]);
    return response.listen(onData, onError: onError, onDone: onDone, cancelOnError: cancelOnError);
  }

  @override
  bool get persistentConnection => throw UnimplementedError();

  @override
  String get reasonPhrase => 'OK';

  @override
  Future<HttpClientResponse> redirect([String? method, Uri? url, bool? followLoops]) {
    throw UnimplementedError();
  }

  @override
  List<RedirectInfo> get redirects => throw UnimplementedError();

  @override
  int get statusCode => _response.statusCode;
}

class _FakeHttpHeaders implements HttpHeaders {
  _FakeHttpHeaders(this._backingData);

  final Map<String, List<String>> _backingData;

  @override
  List<String>? operator [](String name) => _backingData[name];

  @override
  void add(String name, Object value, {bool preserveHeaderCase = false}) {
    _backingData[name] ??= <String>[];
    _backingData[name]!.add(value.toString());
  }

  @override
  late bool chunkedTransferEncoding;

  @override
  void clear() {
    _backingData.clear();
  }

  @override
  int contentLength = -1;

  @override
  ContentType? contentType;

  @override
  DateTime? date;

  @override
  DateTime? expires;

  @override
  void forEach(void Function(String name, List<String> values) action) {}

  @override
  String? host;

  @override
  void noFolding(String name) {}

  @override
  void remove(String name, Object value) {
    _backingData[name]?.remove(value.toString());
  }

  @override
  void removeAll(String name) {
    _backingData.remove(name);
  }

  @override
  void set(String name, Object value, {bool preserveHeaderCase = false}) {
    _backingData[name] = <String>[value.toString()];
  }

  @override
  String? value(String name) {
    return _backingData[name]?.join('; ');
  }

  @override
  DateTime? ifModifiedSince;

  @override
  late bool persistentConnection;

  @override
  int? port;
}
