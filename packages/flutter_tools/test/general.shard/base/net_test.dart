// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:fake_async/fake_async.dart';
import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart' as io;
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/net.dart';
import 'package:flutter_tools/src/base/platform.dart';

import '../../src/common.dart';
import '../../src/fake_http_client.dart';

void main() {
  late BufferLogger testLogger;

  setUp(() {
    testLogger = BufferLogger.test();
  });

  Net createNet(io.HttpClient client) {
    return Net(httpClientFactory: () => client, logger: testLogger, platform: FakePlatform());
  }

  group('successful fetch', () {
    const String responseString = 'response string';
    late List<int> responseData;

    setUp(() {
      responseData = utf8.encode(responseString);
    });

    testWithoutContext('fetchUrl() gets the data', () async {
      final Net net = createNet(
        FakeHttpClient.list(<FakeRequest>[
          FakeRequest(
            Uri.parse('http://example.invalid/'),
            response: FakeResponse(body: utf8.encode(responseString)),
          ),
        ]),
      );

      final List<int>? data = await net.fetchUrl(Uri.parse('http://example.invalid/'));
      expect(data, equals(responseData));
    });

    testWithoutContext('fetchUrl(destFile) writes the data to a file', () async {
      final Net net = createNet(
        FakeHttpClient.list(<FakeRequest>[
          FakeRequest(
            Uri.parse('http://example.invalid/'),
            response: FakeResponse(body: utf8.encode(responseString)),
          ),
        ]),
      );
      final MemoryFileSystem fileSystem = MemoryFileSystem.test();
      final File destFile = fileSystem.file('dest_file')..createSync();
      final List<int>? data = await net.fetchUrl(
        Uri.parse('http://example.invalid/'),
        destFile: destFile,
      );
      expect(data, equals(<int>[]));
      expect(destFile.readAsStringSync(), responseString);
    });
  });

  testWithoutContext('retry from 500', () async {
    final Net net = createNet(
      FakeHttpClient.list(<FakeRequest>[
        FakeRequest(
          Uri.parse('http://example.invalid/'),
          response: const FakeResponse(statusCode: io.HttpStatus.internalServerError),
        ),
        FakeRequest(
          Uri.parse('http://example.invalid/'),
          response: const FakeResponse(statusCode: io.HttpStatus.internalServerError),
        ),
        FakeRequest(
          Uri.parse('http://example.invalid/'),
          response: const FakeResponse(statusCode: io.HttpStatus.internalServerError),
        ),
        FakeRequest(
          Uri.parse('http://example.invalid/'),
          response: const FakeResponse(statusCode: io.HttpStatus.internalServerError),
        ),
      ]),
    );

    await net.fetchUrl(
      Uri.parse('http://example.invalid/'),
      maxAttempts: 4,
      durationOverride: Duration.zero,
    );

    expect(
      testLogger.statusText,
      'Download failed -- attempting retry 1 in 1 second...\n'
      'Download failed -- attempting retry 2 in 2 seconds...\n'
      'Download failed -- attempting retry 3 in 4 seconds...\n'
      'Download failed -- retry 4\n',
    );
    expect(testLogger.errorText, isEmpty);
  });

  testWithoutContext('retry from network error', () async {
    final Uri invalid = Uri.parse('http://example.invalid/');
    final Net net = createNet(
      FakeHttpClient.list(<FakeRequest>[
        FakeRequest(invalid, responseError: const io.SocketException('test')),
        FakeRequest(invalid, responseError: const io.SocketException('test')),
        FakeRequest(invalid, responseError: const io.SocketException('test')),
        FakeRequest(invalid, responseError: const io.SocketException('test')),
      ]),
    );

    await net.fetchUrl(
      Uri.parse('http://example.invalid/'),
      maxAttempts: 4,
      durationOverride: Duration.zero,
    );

    expect(
      testLogger.statusText,
      'Download failed -- attempting retry 1 in 1 second...\n'
      'Download failed -- attempting retry 2 in 2 seconds...\n'
      'Download failed -- attempting retry 3 in 4 seconds...\n'
      'Download failed -- retry 4\n',
    );
    expect(testLogger.errorText, isEmpty);
  });

  testWithoutContext('retry from SocketException', () async {
    final Uri invalid = Uri.parse('http://example.invalid/');
    final Net net = createNet(
      FakeHttpClient.list(<FakeRequest>[
        FakeRequest(invalid, responseError: const io.SocketException('')),
        FakeRequest(invalid, responseError: const io.SocketException('')),
        FakeRequest(invalid, responseError: const io.SocketException('')),
        FakeRequest(invalid, responseError: const io.SocketException('')),
      ]),
    );
    String? error;
    FakeAsync().run((FakeAsync time) {
      net
          .fetchUrl(invalid)
          .then(
            (List<int>? value) async {
              error = 'test completed unexpectedly';
            },
            onError: (dynamic exception) {
              error = 'test failed unexpectedly: $exception';
            },
          );
      expect(testLogger.statusText, '');
      time.elapse(const Duration(milliseconds: 10000));
      expect(
        testLogger.statusText,
        'Download failed -- attempting retry 1 in 1 second...\n'
        'Download failed -- attempting retry 2 in 2 seconds...\n'
        'Download failed -- attempting retry 3 in 4 seconds...\n'
        'Download failed -- attempting retry 4 in 8 seconds...\n',
      );
    });
    expect(testLogger.errorText, isEmpty);
    expect(error, isNull);
    expect(testLogger.traceText, contains('Download error: SocketException'));
  });

  testWithoutContext('no retry from HandshakeException', () async {
    final Uri invalid = Uri.parse('http://example.invalid/');
    final Net net = createNet(
      FakeHttpClient.list(<FakeRequest>[
        FakeRequest(invalid, responseError: const io.HandshakeException()),
        FakeRequest(invalid, responseError: const io.HandshakeException()),
        FakeRequest(invalid, responseError: const io.HandshakeException()),
        FakeRequest(invalid, responseError: const io.HandshakeException()),
      ]),
    );
    String? error;
    FakeAsync().run((FakeAsync time) {
      net
          .fetchUrl(invalid)
          .then(
            (List<int>? value) async {
              error = 'test completed unexpectedly';
            },
            onError: (dynamic exception) {
              error = 'test failed: $exception';
            },
          );
      expect(testLogger.statusText, '');
      time.elapse(const Duration(milliseconds: 10000));
      expect(testLogger.statusText, '');
    });
    expect(error, startsWith('test failed'));
    expect(testLogger.traceText, contains('HandshakeException'));
  });

  testWithoutContext('check for bad override on ArgumentError', () async {
    final Uri invalid = Uri.parse('example.invalid/');
    final Net net = Net(
      httpClientFactory: () {
        return FakeHttpClient.list(<FakeRequest>[
          FakeRequest(invalid, responseError: ArgumentError()),
        ]);
      },
      logger: testLogger,
      platform: FakePlatform(
        environment: <String, String>{'FLUTTER_STORAGE_BASE_URL': 'example.invalid'},
      ),
    );
    String? error;
    FakeAsync().run((FakeAsync time) {
      net
          .fetchUrl(Uri.parse('example.invalid/'))
          .then(
            (List<int>? value) async {
              error = 'test completed unexpectedly';
            },
            onError: (dynamic exception) {
              error = 'test failed: $exception';
            },
          );
      expect(testLogger.statusText, '');
      time.elapse(const Duration(milliseconds: 10000));
      expect(testLogger.statusText, '');
    });
    expect(error, startsWith('test failed'));
    expect(testLogger.errorText, contains('Invalid argument'));
    expect(error, contains('FLUTTER_STORAGE_BASE_URL'));
  });

  testWithoutContext('retry from HttpException', () async {
    final Uri invalid = Uri.parse('http://example.invalid/');
    final Net net = createNet(
      FakeHttpClient.list(<FakeRequest>[
        FakeRequest(invalid, responseError: const io.HttpException('')),
        FakeRequest(invalid, responseError: const io.HttpException('')),
        FakeRequest(invalid, responseError: const io.HttpException('')),
        FakeRequest(invalid, responseError: const io.HttpException('')),
      ]),
    );
    String? error;
    FakeAsync().run((FakeAsync time) {
      net
          .fetchUrl(invalid)
          .then(
            (List<int>? value) async {
              error = 'test completed unexpectedly';
            },
            onError: (dynamic exception) {
              error = 'test failed unexpectedly: $exception';
            },
          );
      expect(testLogger.statusText, '');
      time.elapse(const Duration(milliseconds: 10000));
      expect(
        testLogger.statusText,
        'Download failed -- attempting retry 1 in 1 second...\n'
        'Download failed -- attempting retry 2 in 2 seconds...\n'
        'Download failed -- attempting retry 3 in 4 seconds...\n'
        'Download failed -- attempting retry 4 in 8 seconds...\n',
      );
    });
    expect(testLogger.errorText, isEmpty);
    expect(error, isNull);
    expect(testLogger.traceText, contains('Download error: HttpException'));
  });

  testWithoutContext('retry from HttpException when request throws', () async {
    final Uri invalid = Uri.parse('http://example.invalid/');
    final Net net = createNet(
      FakeHttpClient.list(<FakeRequest>[
        FakeRequest(invalid, responseError: const io.HttpException('')),
        FakeRequest(invalid, responseError: const io.HttpException('')),
        FakeRequest(invalid, responseError: const io.HttpException('')),
        FakeRequest(invalid, responseError: const io.HttpException('')),
      ]),
    );
    String? error;
    FakeAsync().run((FakeAsync time) {
      net
          .fetchUrl(invalid)
          .then(
            (List<int>? value) async {
              error = 'test completed unexpectedly';
            },
            onError: (dynamic exception) {
              error = 'test failed unexpectedly: $exception';
            },
          );
      expect(testLogger.statusText, '');
      time.elapse(const Duration(milliseconds: 10000));
      expect(
        testLogger.statusText,
        'Download failed -- attempting retry 1 in 1 second...\n'
        'Download failed -- attempting retry 2 in 2 seconds...\n'
        'Download failed -- attempting retry 3 in 4 seconds...\n'
        'Download failed -- attempting retry 4 in 8 seconds...\n',
      );
    });
    expect(testLogger.errorText, isEmpty);
    expect(error, isNull);
    expect(testLogger.traceText, contains('Download error: HttpException'));
  });

  testWithoutContext('max attempts', () async {
    final Uri invalid = Uri.parse('http://example.invalid/');
    final Net net = createNet(
      FakeHttpClient.list(<FakeRequest>[
        FakeRequest(
          invalid,
          response: const FakeResponse(statusCode: HttpStatus.internalServerError),
        ),
        FakeRequest(
          invalid,
          response: const FakeResponse(statusCode: HttpStatus.internalServerError),
        ),
        FakeRequest(
          invalid,
          response: const FakeResponse(statusCode: HttpStatus.internalServerError),
        ),
      ]),
    );
    String? error;
    List<int>? actualResult;
    FakeAsync().run((FakeAsync time) {
      net
          .fetchUrl(invalid, maxAttempts: 3)
          .then(
            (List<int>? value) async {
              actualResult = value;
            },
            onError: (dynamic exception) {
              error = 'test failed unexpectedly: $exception';
            },
          );
      expect(testLogger.statusText, '');
      time.elapse(const Duration(milliseconds: 10000));
      expect(
        testLogger.statusText,
        'Download failed -- attempting retry 1 in 1 second...\n'
        'Download failed -- attempting retry 2 in 2 seconds...\n'
        'Download failed -- retry 3\n',
      );
    });
    expect(testLogger.errorText, isEmpty);
    expect(error, isNull);
    expect(actualResult, isNull);
  });

  testWithoutContext('remote file non-existent', () async {
    final Uri invalid = Uri.parse('http://example.invalid/');
    final Net net = createNet(
      FakeHttpClient.list(<FakeRequest>[
        FakeRequest(
          invalid,
          method: HttpMethod.head,
          response: const FakeResponse(statusCode: HttpStatus.notFound),
        ),
      ]),
    );
    final bool result = await net.doesRemoteFileExist(invalid);
    expect(result, false);
  });

  testWithoutContext('remote file server error', () async {
    final Uri valid = Uri.parse('http://example.valid/');
    final Net net = createNet(
      FakeHttpClient.list(<FakeRequest>[
        FakeRequest(
          valid,
          method: HttpMethod.head,
          response: const FakeResponse(statusCode: HttpStatus.internalServerError),
        ),
      ]),
    );
    final bool result = await net.doesRemoteFileExist(valid);
    expect(result, false);
  });

  testWithoutContext('remote file exists', () async {
    final Uri valid = Uri.parse('http://example.valid/');
    final Net net = createNet(
      FakeHttpClient.list(<FakeRequest>[FakeRequest(valid, method: HttpMethod.head)]),
    );
    final bool result = await net.doesRemoteFileExist(valid);
    expect(result, true);
  });
}
