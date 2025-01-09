// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

class TestAssetBundle extends CachingAssetBundle {
  Map<String, int> loadCallCount = <String, int>{};

  @override
  Future<ByteData> load(String key) async {
    loadCallCount[key] = (loadCallCount[key] ?? 0) + 1;
    if (key == 'AssetManifest.json') {
      return ByteData.sublistView(utf8.encode('{"one": ["one"]}'));
    }

    if (key == 'AssetManifest.bin') {
      return const StandardMessageCodec().encodeMessage(<String, Object>{'one': <Object>[]})!;
    }

    if (key == 'AssetManifest.bin.json') {
      // Encode the manifest data that will be used by the app
      final ByteData data =
          const StandardMessageCodec().encodeMessage(<String, Object>{'one': <Object>[]})!;
      // Simulate the behavior of NetworkAssetBundle.load here, for web tests
      return ByteData.sublistView(
        utf8.encode(
          json.encode(
            base64.encode(
              // Encode only the actual bytes of the buffer, and no more...
              data.buffer.asUint8List(data.offsetInBytes, data.lengthInBytes),
            ),
          ),
        ),
      );
    }

    if (key == 'counter') {
      return ByteData.sublistView(utf8.encode(loadCallCount[key]!.toString()));
    }

    if (key == 'one') {
      return ByteData(1)..setInt8(0, 49);
    }

    throw FlutterError('key not found');
  }
}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  test('Caching asset bundle test', () async {
    final TestAssetBundle bundle = TestAssetBundle();

    final ByteData assetData = await bundle.load('one');
    expect(assetData.getInt8(0), equals(49));

    expect(bundle.loadCallCount['one'], 1);

    final String assetString = await bundle.loadString('one');
    expect(assetString, equals('1'));

    expect(bundle.loadCallCount['one'], 2);

    late Object loadException;
    try {
      await bundle.loadString('foo');
    } catch (e) {
      loadException = e;
    }
    expect(loadException, isFlutterError);
  });

  group('CachingAssetBundle caching behavior', () {
    test(
      'caches results for loadString, loadStructuredData, and loadBinaryStructuredData',
      () async {
        final TestAssetBundle bundle = TestAssetBundle();

        final String firstLoadStringResult = await bundle.loadString('counter');
        final String secondLoadStringResult = await bundle.loadString('counter');
        expect(firstLoadStringResult, '1');
        expect(secondLoadStringResult, '1');

        final String firstLoadStructuredDataResult = await bundle.loadStructuredData(
          'AssetManifest.json',
          (String value) => Future<String>.value('one'),
        );
        final String secondLoadStructuredDataResult = await bundle.loadStructuredData(
          'AssetManifest.json',
          (String value) => Future<String>.value('two'),
        );
        expect(firstLoadStructuredDataResult, 'one');
        expect(secondLoadStructuredDataResult, 'one');

        final String firstLoadStructuredBinaryDataResult = await bundle.loadStructuredBinaryData(
          'AssetManifest.bin',
          (ByteData value) => Future<String>.value('one'),
        );
        final String secondLoadStructuredBinaryDataResult = await bundle.loadStructuredBinaryData(
          'AssetManifest.bin',
          (ByteData value) => Future<String>.value('two'),
        );
        expect(firstLoadStructuredBinaryDataResult, 'one');
        expect(secondLoadStructuredBinaryDataResult, 'one');
      },
    );

    test("clear clears all cached values'", () async {
      final TestAssetBundle bundle = TestAssetBundle();

      await bundle.loadString('counter');
      bundle.clear();
      final String secondLoadStringResult = await bundle.loadString('counter');
      expect(secondLoadStringResult, '2');

      await bundle.loadStructuredData(
        'AssetManifest.json',
        (String value) => Future<String>.value('one'),
      );
      bundle.clear();
      final String secondLoadStructuredDataResult = await bundle.loadStructuredData(
        'AssetManifest.json',
        (String value) => Future<String>.value('two'),
      );
      expect(secondLoadStructuredDataResult, 'two');

      await bundle.loadStructuredBinaryData(
        'AssetManifest.bin',
        (ByteData value) => Future<String>.value('one'),
      );
      bundle.clear();
      final String secondLoadStructuredBinaryDataResult = await bundle.loadStructuredBinaryData(
        'AssetManifest.bin',
        (ByteData value) => Future<String>.value('two'),
      );
      expect(secondLoadStructuredBinaryDataResult, 'two');
    });

    test('evict evicts a particular key from the cache', () async {
      final TestAssetBundle bundle = TestAssetBundle();

      await bundle.loadString('counter');
      bundle.evict('counter');
      final String secondLoadStringResult = await bundle.loadString('counter');
      expect(secondLoadStringResult, '2');

      await bundle.loadStructuredData(
        'AssetManifest.json',
        (String value) => Future<String>.value('one'),
      );
      bundle.evict('AssetManifest.json');
      final String secondLoadStructuredDataResult = await bundle.loadStructuredData(
        'AssetManifest.json',
        (String value) => Future<String>.value('two'),
      );
      expect(secondLoadStructuredDataResult, 'two');

      await bundle.loadStructuredBinaryData(
        'AssetManifest.bin',
        (ByteData value) => Future<String>.value('one'),
      );
      bundle.evict('AssetManifest.bin');
      final String secondLoadStructuredBinaryDataResult = await bundle.loadStructuredBinaryData(
        'AssetManifest.bin',
        (ByteData value) => Future<String>.value('two'),
      );
      expect(secondLoadStructuredBinaryDataResult, 'two');
    });

    test(
      'for a given key, subsequent loadStructuredData calls are synchronous after the first call resolves',
      () async {
        final TestAssetBundle bundle = TestAssetBundle();
        await bundle.loadStructuredData('one', (String data) => SynchronousFuture<int>(1));
        final Future<int> data = bundle.loadStructuredData(
          'one',
          (String data) => SynchronousFuture<int>(2),
        );
        expect(data, isA<SynchronousFuture<int>>());
        expect(await data, 1);
      },
    );

    test(
      'for a given key, subsequent loadStructuredBinaryData calls are synchronous after the first call resolves',
      () async {
        final TestAssetBundle bundle = TestAssetBundle();
        await bundle.loadStructuredBinaryData('one', (ByteData data) => 1);
        final Future<int> data = bundle.loadStructuredBinaryData('one', (ByteData data) => 2);
        expect(data, isA<SynchronousFuture<int>>());
        expect(await data, 1);
      },
    );

    testWidgets('loadStructuredData handles exceptions correctly', (WidgetTester tester) async {
      final TestAssetBundle bundle = TestAssetBundle();
      try {
        await bundle.loadStructuredData(
          'AssetManifest.json',
          (String value) => Future<String>.error('what do they say?'),
        );
        fail('expected exception did not happen');
      } catch (e) {
        expect(e.toString(), contains('what do they say?'));
      }
    });

    testWidgets('loadStructuredBinaryData handles exceptions correctly', (
      WidgetTester tester,
    ) async {
      final TestAssetBundle bundle = TestAssetBundle();
      try {
        await bundle.loadStructuredBinaryData(
          'AssetManifest.bin',
          (ByteData value) => Future<String>.error('buy more crystals'),
        );
        fail('expected exception did not happen');
      } catch (e) {
        expect(e.toString(), contains('buy more crystals'));
      }
    });
  });

  test('AssetImage.obtainKey succeeds with ImageConfiguration.empty', () async {
    // This is a regression test for https://github.com/flutter/flutter/issues/12392
    final AssetImage assetImage = AssetImage('one', bundle: TestAssetBundle());
    final AssetBundleImageKey key = await assetImage.obtainKey(ImageConfiguration.empty);
    expect(key.name, 'one');
    expect(key.scale, 1.0);
  });

  test('NetworkAssetBundle control test', () async {
    final Uri uri = Uri.http('example.org', '/path');
    final NetworkAssetBundle bundle = NetworkAssetBundle(uri);
    late FlutterError error;
    try {
      await bundle.load('key');
    } on FlutterError catch (e) {
      error = e;
    }
    expect(error, isNotNull);
    expect(error.diagnostics.length, 2);
    expect(error.diagnostics.last, isA<IntProperty>());
    expect(
      error.toStringDeep(),
      'FlutterError\n'
      '   Unable to load asset: "key".\n'
      '   HTTP status code: 400\n',
    );
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/39998

  test('toString works as intended', () {
    final Uri uri = Uri.http('example.org', '/path');
    final NetworkAssetBundle bundle = NetworkAssetBundle(uri);

    expect(bundle.toString(), 'NetworkAssetBundle#${shortHash(bundle)}($uri)');
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/39998

  test('Throws expected exceptions when loading not exists asset', () async {
    late final FlutterError error;
    try {
      await rootBundle.load('not-exists');
    } on FlutterError catch (e) {
      error = e;
    }
    expect(
      error.message,
      equals(
        'Unable to load asset: "not-exists".\n'
        'The asset does not exist or has empty data.',
      ),
    );
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/56314

  test('loadStructuredBinaryData correctly loads ByteData', () async {
    final TestAssetBundle bundle = TestAssetBundle();
    final Map<Object?, Object?> assetManifest = await bundle.loadStructuredBinaryData(
      'AssetManifest.bin',
      (ByteData data) => const StandardMessageCodec().decodeMessage(data) as Map<Object?, Object?>,
    );
    expect(assetManifest.keys.toList(), equals(<String>['one']));
    expect(assetManifest['one'], <Object>[]);
  });
}
