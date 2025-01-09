// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'matchers.dart';
library;

import 'dart:convert';
import 'dart:js_interop';
import 'dart:typed_data';

import 'package:matcher/expect.dart' show fail;

import 'goldens.dart';
import 'web.dart' as web;

/// An unsupported [GoldenFileComparator] that exists for API compatibility.
class LocalFileComparator extends GoldenFileComparator {
  @override
  Future<bool> compare(Uint8List imageBytes, Uri golden) {
    throw UnsupportedError('LocalFileComparator is not supported on the web.');
  }

  @override
  Future<void> update(Uri golden, Uint8List imageBytes) {
    throw UnsupportedError('LocalFileComparator is not supported on the web.');
  }
}

/// Returns whether [test] and [master] are pixel by pixel identical.
///
/// This method is not supported on the web and throws an [UnsupportedError]
/// when called.
Future<ComparisonResult> compareLists(List<int> test, List<int> master) async {
  throw UnsupportedError('Golden testing is not supported on the web.');
}

/// The default [WebGoldenComparator] implementation for `flutter test`.
///
/// This comparator will send a request to the test server for golden comparison
/// which will then defer the comparison to [goldenFileComparator].
///
/// See also:
///
///   * [matchesGoldenFile], the function that invokes the comparator.
class DefaultWebGoldenComparator extends WebGoldenComparator {
  /// Creates a new [DefaultWebGoldenComparator] for the specified [testUri].
  ///
  /// Golden file keys will be interpreted as file paths relative to the
  /// directory in which [testUri] resides.
  ///
  /// The [testUri] must represent a file.
  DefaultWebGoldenComparator(this.testUri);

  /// The test file currently being executed.
  ///
  /// Golden file keys will be interpreted as file paths relative to the
  /// directory in which this file resides.
  Uri testUri;

  @override
  Future<bool> compare(double width, double height, Uri golden) async {
    final String key = golden.toString();
    final web.Response response =
        await web.window
            .fetch(
              'flutter_goldens'.toJS,
              web.RequestInit(
                method: 'POST',
                body:
                    json.encode(<String, Object>{
                      'testUri': testUri.toString(),
                      'key': key,
                      'width': width.round(),
                      'height': height.round(),
                    }).toJS,
              ),
            )
            .toDart;
    final String responseText = (await response.text().toDart).toDart;
    if (responseText == 'true') {
      return true;
    }
    fail(responseText);
  }

  @override
  Future<void> update(double width, double height, Uri golden) async {
    // Update is handled on the server side, just use the same logic here
    await compare(width, height, golden);
  }

  @override
  Future<bool> compareBytes(Uint8List bytes, Uri golden) async {
    final String key = golden.toString();
    final String bytesEncoded = base64.encode(bytes);
    final web.Response response =
        await web.window
            .fetch(
              'flutter_goldens'.toJS,
              web.RequestInit(
                method: 'POST',
                body:
                    json.encode(<String, Object>{
                      'testUri': testUri.toString(),
                      'key': key,
                      'bytes': bytesEncoded,
                    }).toJS,
              ),
            )
            .toDart;
    final String responseText = (await response.text().toDart).toDart;
    if (responseText == 'true') {
      return true;
    }
    fail(responseText);
  }

  @override
  Future<void> updateBytes(Uint8List bytes, Uri golden) async {
    // Update is handled on the server side, just use the same logic here
    await compareBytes(bytes, golden);
  }
}
