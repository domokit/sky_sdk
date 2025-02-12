// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:android_driver_extensions/skia_gold.dart';
library;

// Similar to `flutter_test`, we ignore the implementation import.
// ignore: implementation_imports
import 'package:android_driver_extensions/native_driver.dart';
import 'package:matcher/src/expect/async_matcher.dart';
import 'package:matcher/src/interfaces.dart';

/// Invokes [matchesGoldenFile] with optional [retries] if a comparison fails.
AsyncMatcher matchesGoldenFileWithRetries(Object key, {int? version, int retries = 2}) {
  final AsyncMatcher delegate = matchesGoldenFile(key, version: version);
  if (retries == 0) {
    return delegate;
  }
  return _AsyncMatcherWithRetries(delegate, retries: retries);
}

final class _AsyncMatcherWithRetries extends AsyncMatcher {
  _AsyncMatcherWithRetries(this._delegate, {required int retries}) : _retries = retries {
    if (retries < 1) {
      throw RangeError.value(retries, 'retries', 'Must be at least 1');
    }
  }

  final AsyncMatcher _delegate;
  int _retries;

  @override
  Description describe(Description description) {
    description = _delegate.describe(description);
    description.add('Retries remaining: $_retries');
    return description;
  }

  @override
  Future<String?> matchAsync(Object? item) async {
    while (true) {
      final Object? error = await _delegate.matchAsync(item);
      if (error == null) {
        return null;
      }
      print('Failed: $error');
      if (--_retries == 0) {
        return 'Retries exceeded. Giving up.';
      } else {
        print('Retrying... $_retries retries left.');
      }
      assert(_retries >= 0, 'Unreachable');
    }
  }
}
