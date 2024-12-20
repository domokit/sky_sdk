// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('SynchronousFuture control test', () async {
    final Future<int> future = SynchronousFuture<int>(42);

    int? result;
    future.then<void>((int value) {
      result = value;
    });

    expect(result, equals(42));
    result = null;

    final Future<int> futureWithTimeout = future.timeout(const Duration(milliseconds: 1));
    futureWithTimeout.then<void>((int value) {
      result = value;
    });
    expect(result, isNull);
    await futureWithTimeout;
    expect(result, equals(42));
    result = null;

    final Stream<int> stream = future.asStream();

    expect(await stream.single, equals(42));

    bool ranAction = false;
    // ignore: void_checks, https://github.com/dart-lang/linter/issues/1675
    final Future<int> completeResult = future.whenComplete(() {
      ranAction = true;
      // verify that whenComplete does NOT propagate its return value:
      return Future<int>.value(31);
    });

    expect(ranAction, isTrue);
    ranAction = false;

    expect(await completeResult, equals(42));

    Object? exception;
    try {
      await future.whenComplete(() {
        throw ArgumentError();
      });
      // Unreached.
      expect(false, isTrue);
    } catch (e) {
      exception = e;
    }
    expect(exception, isArgumentError);
  });
}
