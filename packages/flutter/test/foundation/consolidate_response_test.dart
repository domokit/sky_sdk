// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('!chrome')
library;

import 'dart:async';
import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

final Uint8List chunkOne = Uint8List.fromList(<int>[0, 1, 2, 3, 4, 5]);
final Uint8List chunkTwo = Uint8List.fromList(<int>[6, 7, 8, 9, 10]);

void main() {
  group(consolidateHttpClientResponseBytes, () {
    late MockHttpClientResponse response;

    setUp(() {
      response = MockHttpClientResponse(chunkOne: chunkOne, chunkTwo: chunkTwo);
    });

    test('Converts an HttpClientResponse with contentLength to bytes', () async {
      response.contentLength = chunkOne.length + chunkTwo.length;
      final List<int> bytes = await consolidateHttpClientResponseBytes(response);

      expect(bytes, <int>[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
    });

    test('Converts a compressed HttpClientResponse with contentLength to bytes', () async {
      response.contentLength = chunkOne.length;
      final List<int> bytes = await consolidateHttpClientResponseBytes(response);

      expect(bytes, <int>[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
    });

    test('Converts an HttpClientResponse without contentLength to bytes', () async {
      response.contentLength = -1;
      final List<int> bytes = await consolidateHttpClientResponseBytes(response);

      expect(bytes, <int>[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
    });

    test('Notifies onBytesReceived for every chunk of bytes', () async {
      final int syntheticTotal = (chunkOne.length + chunkTwo.length) * 2;
      response.contentLength = syntheticTotal;
      final List<int?> records = <int?>[];
      await consolidateHttpClientResponseBytes(
        response,
        onBytesReceived: (int cumulative, int? total) {
          records.addAll(<int?>[cumulative, total]);
        },
      );

      expect(records, <int>[
        chunkOne.length,
        syntheticTotal,
        chunkOne.length + chunkTwo.length,
        syntheticTotal,
      ]);
    });

    test('forwards errors from HttpClientResponse', () async {
      response = MockHttpClientResponse(error: Exception('Test Error'));
      response.contentLength = -1;

      expect(consolidateHttpClientResponseBytes(response), throwsException);
    });

    test('Propagates error to Future return value if onBytesReceived throws', () async {
      response.contentLength = -1;
      final Future<List<int>> result = consolidateHttpClientResponseBytes(
        response,
        onBytesReceived: (int cumulative, int? total) {
          throw 'misbehaving callback';
        },
      );

      expect(result, throwsA(equals('misbehaving callback')));
    });

    group('when gzipped', () {
      final List<int> gzipped = gzip.encode(chunkOne.followedBy(chunkTwo).toList());
      final List<int> gzippedChunkOne = gzipped.sublist(0, gzipped.length ~/ 2);
      final List<int> gzippedChunkTwo = gzipped.sublist(gzipped.length ~/ 2);

      setUp(() {
        response = MockHttpClientResponse(chunkOne: gzippedChunkOne, chunkTwo: gzippedChunkTwo);
        response.compressionState = HttpClientResponseCompressionState.compressed;
      });

      test(
        'Uncompresses GZIP bytes if autoUncompress is true and response.compressionState is compressed',
        () async {
          response.contentLength = gzipped.length;
          final List<int> bytes = await consolidateHttpClientResponseBytes(response);
          expect(bytes, <int>[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
        },
      );

      test(
        'returns gzipped bytes if autoUncompress is false and response.compressionState is compressed',
        () async {
          response.contentLength = gzipped.length;
          final List<int> bytes = await consolidateHttpClientResponseBytes(
            response,
            autoUncompress: false,
          );
          expect(bytes, gzipped);
        },
      );

      test('Notifies onBytesReceived with gzipped numbers', () async {
        response.contentLength = gzipped.length;
        final List<int?> records = <int?>[];
        await consolidateHttpClientResponseBytes(
          response,
          onBytesReceived: (int cumulative, int? total) {
            records.addAll(<int?>[cumulative, total]);
          },
        );

        expect(records, <int>[
          gzippedChunkOne.length,
          gzipped.length,
          gzipped.length,
          gzipped.length,
        ]);
      });

      test(
        'Notifies onBytesReceived with expectedContentLength of -1 if response.compressionState is decompressed',
        () async {
          final int syntheticTotal = (chunkOne.length + chunkTwo.length) * 2;
          response.compressionState = HttpClientResponseCompressionState.decompressed;
          response.contentLength = syntheticTotal;
          final List<int?> records = <int?>[];
          await consolidateHttpClientResponseBytes(
            response,
            onBytesReceived: (int cumulative, int? total) {
              records.addAll(<int?>[cumulative, total]);
            },
          );

          expect(records, <int?>[gzippedChunkOne.length, null, gzipped.length, null]);
        },
      );
    });
  });
}

class MockHttpClientResponse extends Fake implements HttpClientResponse {
  MockHttpClientResponse({
    this.error,
    this.chunkOne = const <int>[],
    this.chunkTwo = const <int>[],
  });

  final dynamic error;
  final List<int> chunkOne;
  final List<int> chunkTwo;

  @override
  int contentLength = 0;

  @override
  HttpClientResponseCompressionState compressionState =
      HttpClientResponseCompressionState.notCompressed;

  @override
  StreamSubscription<List<int>> listen(
    void Function(List<int> event)? onData, {
    Function? onError,
    void Function()? onDone,
    bool? cancelOnError,
  }) {
    if (error != null) {
      return Stream<List<int>>.fromFuture(
        Future<List<int>>.error(error as Object),
      ).listen(onData, onDone: onDone, onError: onError, cancelOnError: cancelOnError);
    }
    return Stream<List<int>>.fromIterable(<List<int>>[
      chunkOne,
      chunkTwo,
    ]).listen(onData, onDone: onDone, onError: onError, cancelOnError: cancelOnError);
  }
}
