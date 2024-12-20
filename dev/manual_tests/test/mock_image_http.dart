// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:flutter_test/flutter_test.dart';

import '../../../packages/flutter/test/image_data.dart';

// Returns a mock HTTP client that responds with an image to all requests.
FakeHttpClient createMockImageHttpClient(SecurityContext? _) {
  final FakeHttpClient client = FakeHttpClient();
  return client;
}

class FakeHttpClient extends Fake implements HttpClient {
  FakeHttpClient([this.context]);

  SecurityContext? context;

  @override
  bool autoUncompress = false;

  final FakeHttpClientRequest request = FakeHttpClientRequest();

  @override
  Future<HttpClientRequest> getUrl(Uri url) async {
    return request;
  }
}

class FakeHttpClientRequest extends Fake implements HttpClientRequest {
  final FakeHttpClientResponse response = FakeHttpClientResponse();

  @override
  Future<HttpClientResponse> close() async {
    return response;
  }
}

class FakeHttpClientResponse extends Fake implements HttpClientResponse {
  @override
  int get statusCode => 200;

  @override
  int get contentLength => kTransparentImage.length;

  @override
  final FakeHttpHeaders headers = FakeHttpHeaders();

  @override
  HttpClientResponseCompressionState get compressionState =>
      HttpClientResponseCompressionState.notCompressed;

  @override
  StreamSubscription<List<int>> listen(
    void Function(List<int>)? onData, {
    void Function()? onDone,
    Function? onError,
    bool? cancelOnError,
  }) {
    return Stream<List<int>>.fromIterable(<List<int>>[
      kTransparentImage,
    ]).listen(onData, onDone: onDone, onError: onError, cancelOnError: cancelOnError);
  }
}

class FakeHttpHeaders extends Fake implements HttpHeaders {}
