// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:developer' as dev;
import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart' as integration_test;

void main() {
  setUp(() {
    // Ensure that we reset to a throwing comparator by default.
    goldenFileComparator = const _NullGoldenFileComparator();
  });

  group('useIfRunningOnDevice', () {
    test('is skipped on the web', () {
      integration_test.VmServiceProxyGoldenFileComparator.useIfRunningOnDevice();
      expect(goldenFileComparator, isInstanceOf<_NullGoldenFileComparator>());
    }, testOn: 'js');

    test('is skipped on desktop platforms', () {
      integration_test.VmServiceProxyGoldenFileComparator.useIfRunningOnDevice();
      expect(goldenFileComparator, isInstanceOf<_NullGoldenFileComparator>());
    }, testOn: 'windows || mac-os || linux');

    test('is set on mobile platforms', () {
      integration_test.VmServiceProxyGoldenFileComparator.useIfRunningOnDevice();
      expect(
        goldenFileComparator,
        isInstanceOf<integration_test.VmServiceProxyGoldenFileComparator>(),
      );
    }, testOn: 'ios || android');
  });

  group('handleEvent', () {
    late integration_test.VmServiceProxyGoldenFileComparator goldenFileComparator;

    setUp(() {
      goldenFileComparator = integration_test.VmServiceProxyGoldenFileComparator.forTesting(
        (String operation, Map<Object?, Object?> params, {String stream = ''}) {},
      );
    });

    test('"id" must be provided', () async {
      final dev.ServiceExtensionResponse response = await goldenFileComparator.handleEvent(
        <String, String>{'result': 'true'},
      );
      expect(response.errorDetail, contains('Required parameter "id" not present in response'));
    });

    test('"id" must be an integer', () async {
      final dev.ServiceExtensionResponse response = await goldenFileComparator.handleEvent(
        <String, String>{'id': 'not-an-integer', 'result': 'true'},
      );
      expect(
        response.errorDetail,
        stringContainsInOrder(<String>[
          'Required parameter "id" not a valid integer',
          'not-an-integer',
        ]),
      );
    });

    test('"id" must match a pending request (never occurred)', () async {
      final dev.ServiceExtensionResponse response = await goldenFileComparator.handleEvent(
        <String, String>{'id': '12345', 'result': 'true'},
      );
      expect(
        response.errorDetail,
        stringContainsInOrder(<String>['No pending request with method ID', '12345']),
      );
    });

    test('"id" must match a pending request (already occurred)', () async {
      // This is based on an implementation detail of knowing how IDs are generated.
      const int nextId = 1;
      goldenFileComparator.update(Uri(path: 'some-file'), Uint8List(0));

      dev.ServiceExtensionResponse response;

      response = await goldenFileComparator.handleEvent(<String, String>{
        'id': '$nextId',
        'result': 'true',
      });
      expect(response.errorDetail, isNull);

      response = await goldenFileComparator.handleEvent(<String, String>{
        'id': '$nextId',
        'result': 'true',
      });
      expect(
        response.errorDetail,
        stringContainsInOrder(<String>['No pending request with method ID', '1']),
      );
    });

    test('requests that contain "error" completes it as an error', () async {
      // This is based on an implementation detail of knowing how IDs are generated.
      const int nextId = 1;

      expect(
        goldenFileComparator.compare(Uint8List(0), Uri(path: 'some-file')),
        throwsA(contains('We did a bad')),
      );

      final dev.ServiceExtensionResponse response = await goldenFileComparator.handleEvent(
        <String, String>{'id': '$nextId', 'error': 'We did a bad'},
      );
      expect(response.errorDetail, isNull);
      expect(response.result, '{}');
    });

    test('requests that do not contain "error" return an empty response', () async {
      // This is based on an implementation detail of knowing how IDs are generated.
      const int nextId = 1;
      goldenFileComparator.update(Uri(path: 'some-file'), Uint8List(0));

      final dev.ServiceExtensionResponse response = await goldenFileComparator.handleEvent(
        <String, String>{'id': '$nextId', 'result': 'true'},
      );
      expect(response.errorDetail, isNull);
      expect(response.result, '{}');
    });

    test('"result" must be provided if "error" is omitted', () async {
      // This is based on an implementation detail of knowing how IDs are generated.
      const int nextId = 1;
      goldenFileComparator.update(Uri(path: 'some-file'), Uint8List(0));

      final dev.ServiceExtensionResponse response = await goldenFileComparator.handleEvent(
        <String, String>{'id': '$nextId'},
      );
      expect(response.errorDetail, contains('Required parameter "result" not present in response'));
    });

    test('"result" must be a boolean', () async {
      // This is based on an implementation detail of knowing how IDs are generated.
      const int nextId = 1;
      goldenFileComparator.update(Uri(path: 'some-file'), Uint8List(0));

      final dev.ServiceExtensionResponse response = await goldenFileComparator.handleEvent(
        <String, String>{'id': '$nextId', 'result': 'not-a-boolean'},
      );
      expect(
        response.errorDetail,
        stringContainsInOrder(<String>[
          'Required parameter "result" not a valid boolean',
          'not-a-boolean',
        ]),
      );
    });

    group('compare', () {
      late integration_test.VmServiceProxyGoldenFileComparator goldenFileComparator;
      late List<(String, Map<Object?, Object?>)> postedEvents;

      setUp(() {
        postedEvents = <(String, Map<Object?, Object?>)>[];
        goldenFileComparator = integration_test.VmServiceProxyGoldenFileComparator.forTesting((
          String operation,
          Map<Object?, Object?> params, {
          String stream = '',
        }) {
          postedEvents.add((operation, params));
        });
      });

      test('posts an event and returns true', () async {
        // This is based on an implementation detail of knowing how IDs are generated.
        const int nextId = 1;

        final Uint8List bytes = Uint8List.fromList(<int>[1, 2, 3, 4, 5]);
        expect(goldenFileComparator.compare(bytes, Uri(path: 'golden-path')), completion(true));

        await goldenFileComparator.handleEvent(<String, String>{'id': '$nextId', 'result': 'true'});

        final (String event, Map<Object?, Object?> params) = postedEvents.single;
        expect(event, 'compare');
        expect(params, <Object?, Object?>{
          'id': nextId,
          'path': 'golden-path',
          'bytes': base64.encode(bytes),
        });
      });

      test('posts an event and returns false', () async {
        // This is based on an implementation detail of knowing how IDs are generated.
        const int nextId = 1;

        final Uint8List bytes = Uint8List.fromList(<int>[1, 2, 3, 4, 5]);
        expect(goldenFileComparator.compare(bytes, Uri(path: 'golden-path')), completion(false));

        await goldenFileComparator.handleEvent(<String, String>{
          'id': '$nextId',
          'result': 'false',
        });

        final (String event, Map<Object?, Object?> params) = postedEvents.single;
        expect(event, 'compare');
        expect(params, <Object?, Object?>{
          'id': nextId,
          'path': 'golden-path',
          'bytes': base64.encode(bytes),
        });
      });

      test('posts an event and returns an error', () async {
        // This is based on an implementation detail of knowing how IDs are generated.
        const int nextId = 1;

        final Uint8List bytes = Uint8List.fromList(<int>[1, 2, 3, 4, 5]);
        expect(
          goldenFileComparator.compare(bytes, Uri(path: 'golden-path')),
          throwsA(contains('We did a bad')),
        );

        await goldenFileComparator.handleEvent(<String, String>{
          'id': '$nextId',
          'error': 'We did a bad',
        });

        final (String event, Map<Object?, Object?> params) = postedEvents.single;
        expect(event, 'compare');
        expect(params, <Object?, Object?>{
          'id': nextId,
          'path': 'golden-path',
          'bytes': base64.encode(bytes),
        });
      });
    });

    group('update', () {
      late integration_test.VmServiceProxyGoldenFileComparator goldenFileComparator;
      late List<(String, Map<Object?, Object?>)> postedEvents;

      setUp(() {
        postedEvents = <(String, Map<Object?, Object?>)>[];
        goldenFileComparator = integration_test.VmServiceProxyGoldenFileComparator.forTesting((
          String operation,
          Map<Object?, Object?> params, {
          String stream = '',
        }) {
          postedEvents.add((operation, params));
        });
      });

      test('posts an event and returns', () async {
        // This is based on an implementation detail of knowing how IDs are generated.
        const int nextId = 1;

        final Uint8List bytes = Uint8List.fromList(<int>[1, 2, 3, 4, 5]);
        expect(goldenFileComparator.update(Uri(path: 'golden-path'), bytes), completes);

        await goldenFileComparator.handleEvent(<String, String>{'id': '$nextId', 'result': 'true'});

        final (String event, Map<Object?, Object?> params) = postedEvents.single;
        expect(event, 'update');
        expect(params, <Object?, Object?>{
          'id': nextId,
          'path': 'golden-path',
          'bytes': base64.encode(bytes),
        });
      });

      test('posts an event and returns an error', () async {
        // This is based on an implementation detail of knowing how IDs are generated.
        const int nextId = 1;

        final Uint8List bytes = Uint8List.fromList(<int>[1, 2, 3, 4, 5]);
        expect(
          goldenFileComparator.update(Uri(path: 'golden-path'), bytes),
          throwsA(contains('We did a bad')),
        );

        await goldenFileComparator.handleEvent(<String, String>{
          'id': '$nextId',
          'error': 'We did a bad',
        });

        final (String event, Map<Object?, Object?> params) = postedEvents.single;
        expect(event, 'update');
        expect(params, <Object?, Object?>{
          'id': nextId,
          'path': 'golden-path',
          'bytes': base64.encode(bytes),
        });
      });
    });
  });
}

final class _NullGoldenFileComparator with Fake implements GoldenFileComparator {
  const _NullGoldenFileComparator();
}
