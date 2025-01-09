// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import '../image_data.dart';

void main() {
  final MockHttpClient client = MockHttpClient();

  testWidgets('Headers', (WidgetTester tester) async {
    HttpOverrides.runZoned<Future<void>>(
      () async {
        await tester.pumpWidget(
          Image.network(
            'https://www.example.com/images/frame.png',
            headers: const <String, String>{'flutter': 'flutter'},
          ),
        );

        expect(MockHttpHeaders.headers['flutter'], <String>['flutter']);
        imageCache.clear();
      },
      createHttpClient: (SecurityContext? _) {
        return client;
      },
    );
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/57187
}

class MockHttpClient extends Fake implements HttpClient {
  @override
  Future<HttpClientRequest> getUrl(Uri url) async {
    return MockHttpClientRequest();
  }

  @override
  bool autoUncompress = false;
}

class MockHttpClientRequest extends Fake implements HttpClientRequest {
  @override
  final MockHttpHeaders headers = MockHttpHeaders();

  @override
  Future<HttpClientResponse> close() async {
    return MockHttpClientResponse();
  }
}

class MockHttpClientResponse extends Fake implements HttpClientResponse {
  @override
  int get contentLength => kTransparentImage.length;

  @override
  int get statusCode => HttpStatus.ok;

  @override
  HttpClientResponseCompressionState get compressionState =>
      HttpClientResponseCompressionState.decompressed;

  @override
  StreamSubscription<List<int>> listen(
    void Function(List<int> event)? onData, {
    Function? onError,
    void Function()? onDone,
    bool? cancelOnError,
  }) {
    return Stream<List<int>>.fromIterable(<List<int>>[
      kTransparentImage,
    ]).listen(onData, onDone: onDone, onError: onError, cancelOnError: cancelOnError);
  }
}

class MockHttpHeaders extends Fake implements HttpHeaders {
  static final Map<String, List<String>> headers = <String, List<String>>{};

  @override
  void add(String key, Object value, {bool preserveHeaderCase = false}) {
    headers[key] ??= <String>[];
    headers[key]!.add(value.toString());
  }
}
