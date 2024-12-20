// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// See also packages/flutter_goldens/test/flutter_goldens_test.dart

import 'dart:typed_data';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_goldens/flutter_goldens.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:platform/platform.dart';

// 1x1 colored pixel
const List<int> _kFailPngBytes = <int>[
  137,
  80,
  78,
  71,
  13,
  10,
  26,
  10,
  0,
  0,
  0,
  13,
  73,
  72,
  68,
  82,
  0,
  0,
  0,
  1,
  0,
  0,
  0,
  1,
  8,
  6,
  0,
  0,
  0,
  31,
  21,
  196,
  137,
  0,
  0,
  0,
  13,
  73,
  68,
  65,
  84,
  120,
  1,
  99,
  249,
  207,
  240,
  255,
  63,
  0,
  7,
  18,
  3,
  2,
  164,
  147,
  160,
  197,
  0,
  0,
  0,
  0,
  73,
  69,
  78,
  68,
  174,
  66,
  96,
  130,
];

void main() {
  final List<String> log = <String>[];
  final MemoryFileSystem fs = MemoryFileSystem();
  final Directory basedir = fs.directory('flutter/test/library/')..createSync(recursive: true);
  final FakeSkiaGoldClient fakeSkiaClient =
      FakeSkiaGoldClient()..expectationForTestValues['flutter.new_golden_test.1'] = '';
  final FlutterLocalFileComparator comparator = FlutterLocalFileComparator(
    basedir.uri,
    fakeSkiaClient,
    fs: fs,
    platform: FakePlatform(
      environment: <String, String>{'FLUTTER_ROOT': '/flutter'},
      operatingSystem: 'macos',
    ),
    log: log.add,
  );

  test('Local passes non-existent baseline for new test, null expectation', () async {
    log.clear();
    expect(
      await comparator.compare(
        Uint8List.fromList(_kFailPngBytes),
        Uri.parse('flutter.new_golden_test.1.png'),
      ),
      isTrue,
    );
    const String expectation =
        'No expectations provided by Skia Gold for test: library.flutter.new_golden_test.1.png. '
        'This may be a new test. If this is an unexpected result, check https://flutter-gold.skia.org.\n'
        'Validate image output found at flutter/test/library/';
    expect(log, const <String>[expectation]);
  });

  test('Local passes non-existent baseline for new test, empty expectation', () async {
    log.clear();
    expect(
      await comparator.compare(
        Uint8List.fromList(_kFailPngBytes),
        Uri.parse('flutter.new_golden_test.2.png'),
      ),
      isTrue,
    );
    const String expectation =
        'No expectations provided by Skia Gold for test: library.flutter.new_golden_test.2.png. '
        'This may be a new test. If this is an unexpected result, check https://flutter-gold.skia.org.\n'
        'Validate image output found at flutter/test/library/';
    expect(log, const <String>[expectation]);
  });
}

// See also packages/flutter_goldens/test/flutter_goldens_test.dart
class FakeSkiaGoldClient extends Fake implements SkiaGoldClient {
  Map<String, String> expectationForTestValues = <String, String>{};
  Object? getExpectationForTestThrowable;
  @override
  Future<String> getExpectationForTest(String testName) async {
    if (getExpectationForTestThrowable != null) {
      throw getExpectationForTestThrowable!;
    }
    return expectationForTestValues[testName] ?? '';
  }

  Map<String, List<int>> imageBytesValues = <String, List<int>>{};
  @override
  Future<List<int>> getImageBytes(String imageHash) async => imageBytesValues[imageHash]!;

  Map<String, String> cleanTestNameValues = <String, String>{};
  @override
  String cleanTestName(String fileName) => cleanTestNameValues[fileName] ?? '';
}
