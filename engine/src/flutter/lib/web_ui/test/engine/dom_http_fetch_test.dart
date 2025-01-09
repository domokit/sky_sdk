// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  // Test successful HTTP roundtrips where the server returns a happy status
  // code and a payload.
  await _testSuccessfulPayloads();

  // Test successful HTTP roundtrips where the server returned something other
  // than a happy result (404 in particular is a common one).
  await _testHttpErrorCodes();

  // Test network errors that prevent the HTTP roundtrip to complete. These
  // errors include invalid URLs, CORS issues, lost internet access, etc.
  await _testNetworkErrors();

  test('window.fetch is banned', () async {
    expect(() => domWindow.fetch('/'), throwsA(isA<UnsupportedError>()));
  });
}

Future<void> _testSuccessfulPayloads() async {
  test('httpFetch fetches a text file', () async {
    final HttpFetchResponse response = await httpFetch('/lib/src/engine/alarm_clock.dart');
    expect(response.status, 200);
    expect(response.contentLength, greaterThan(0));
    expect(response.hasPayload, isTrue);
    expect(response.payload, isNotNull);
    expect(response.url, '/lib/src/engine/alarm_clock.dart');
    expect(
      await response.text(),
      startsWith('''
// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.'''),
    );
  });

  test('httpFetch fetches a binary file as ByteBuffer', () async {
    final HttpFetchResponse response = await httpFetch('/test_images/1x1.png');
    expect(response.status, 200);
    expect(response.contentLength, greaterThan(0));
    expect(response.hasPayload, isTrue);
    expect(response.payload, isNotNull);
    expect(response.url, '/test_images/1x1.png');
    expect((await response.asByteBuffer()).asUint8List().sublist(0, 8), <int>[
      0x89,
      0x50,
      0x4E,
      0x47,
      0x0D,
      0x0A,
      0x1A,
      0x0A,
    ]);
  });

  test('httpFetch fetches a binary file as Uint8List', () async {
    final HttpFetchResponse response = await httpFetch('/test_images/1x1.png');
    expect(response.status, 200);
    expect(response.contentLength, greaterThan(0));
    expect(response.hasPayload, isTrue);
    expect(response.payload, isNotNull);
    expect(response.url, '/test_images/1x1.png');
    expect((await response.asUint8List()).sublist(0, 8), <int>[
      0x89,
      0x50,
      0x4E,
      0x47,
      0x0D,
      0x0A,
      0x1A,
      0x0A,
    ]);
  });

  test('httpFetch fetches json', () async {
    final HttpFetchResponse response = await httpFetch('/test_images/');
    expect(response.status, 200);
    expect(response.contentLength, greaterThan(0));
    expect(response.hasPayload, isTrue);
    expect(response.payload, isNotNull);
    expect(response.url, '/test_images/');
    expect(await response.json(), isA<List<Object?>>());
  });

  test('httpFetch reads data in chunks', () async {
    // There is no guarantee that the server will actually serve the data in any
    // particular chunk sizes, but breaking up the data in _some_ way does cause
    // it to chunk it.
    const List<List<int>> lengthAndChunks = <List<int>>[
      <int>[0, 0],
      <int>[10, 10],
      <int>[1000, 100],
      <int>[10000, 1000],
      <int>[100000, 10000],
    ];
    for (final List<int> lengthAndChunk in lengthAndChunks) {
      final int length = lengthAndChunk.first;
      final int chunkSize = lengthAndChunk.last;
      final String url = '/long_test_payload?length=$length&chunk=$chunkSize';
      final HttpFetchResponse response = await httpFetch(url);
      expect(response.status, 200);
      expect(response.contentLength, length);
      expect(response.hasPayload, isTrue);
      expect(response.payload, isNotNull);
      expect(response.url, url);

      final List<int> result = <int>[];
      await response.payload.read<JSUint8Array>(
        (JSUint8Array chunk) => result.addAll(chunk.toDart),
      );
      expect(result, hasLength(length));
      expect(result, List<int>.generate(length, (int i) => i & 0xFF));
    }
  });

  test('httpFetchText fetches a text file', () async {
    final String text = await httpFetchText('/lib/src/engine/alarm_clock.dart');
    expect(
      text,
      startsWith('''
// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.'''),
    );
  });

  test('httpFetchByteBuffer fetches a binary file as ByteBuffer', () async {
    final ByteBuffer response = await httpFetchByteBuffer('/test_images/1x1.png');
    expect(response.asUint8List().sublist(0, 8), <int>[
      0x89,
      0x50,
      0x4E,
      0x47,
      0x0D,
      0x0A,
      0x1A,
      0x0A,
    ]);
  });

  test('httpFetchJson fetches json', () async {
    final Object? json = await httpFetchJson('/test_images/');
    expect(json, isA<List<Object?>>());
  });
}

Future<void> _testHttpErrorCodes() async {
  test('httpFetch throws HttpFetchNoPayloadError on 404', () async {
    final HttpFetchResponse response = await httpFetch('/file_not_found');
    expect(response.status, 404);
    expect(response.hasPayload, isFalse);
    expect(response.url, '/file_not_found');

    try {
      // Attempting to read the payload when there isn't one should result in
      // HttpFetchNoPayloadError thrown.
      response.payload;
      fail('Expected HttpFetchNoPayloadError');
    } on HttpFetchNoPayloadError catch (error) {
      expect(error.status, 404);
      expect(error.url, '/file_not_found');
      expect(
        error.toString(),
        'Flutter Web engine failed to fetch "/file_not_found". '
        'HTTP request succeeded, but the server responded with HTTP status 404.',
      );
    }
  });

  test('httpFetch* functions throw HttpFetchNoPayloadError on 404', () async {
    final List<AsyncCallback> testFunctions = <AsyncCallback>[
      () async => httpFetchText('/file_not_found'),
      () async => httpFetchByteBuffer('/file_not_found'),
      () async => httpFetchJson('/file_not_found'),
    ];

    for (final AsyncCallback testFunction in testFunctions) {
      try {
        await testFunction();
        fail('Expected HttpFetchNoPayloadError');
      } on HttpFetchNoPayloadError catch (error) {
        expect(error.status, 404);
        expect(error.url, '/file_not_found');
        expect(
          error.toString(),
          'Flutter Web engine failed to fetch "/file_not_found". '
          'HTTP request succeeded, but the server responded with HTTP status 404.',
        );
      }
    }
  });
}

Future<void> _testNetworkErrors() async {
  test('httpFetch* functions throw HttpFetchError on network errors', () async {
    // Fetch throws the error this test wants on URLs with user credentials.
    const String badUrl = 'https://user:password@example.com/';

    final List<AsyncCallback> testFunctions = <AsyncCallback>[
      () async => httpFetch(badUrl),
      () async => httpFetchText(badUrl),
      () async => httpFetchByteBuffer(badUrl),
      () async => httpFetchJson(badUrl),
    ];

    for (final AsyncCallback testFunction in testFunctions) {
      try {
        await testFunction();
        fail('Expected HttpFetchError');
      } on HttpFetchError catch (error) {
        expect(error.url, badUrl);
        expect(
          error.toString(),
          // Browsers agree on throwing a TypeError, but they disagree on the
          // error message. So this only checks for the common error prefix, but
          // not the entire error message.
          startsWith(
            'Flutter Web engine failed to complete HTTP request to fetch '
            '"https://user:password@example.com/": TypeError: ',
          ),
        );
      }
    }
  });
}

typedef AsyncCallback = Future<void> Function();
