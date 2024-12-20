// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:fake_async/fake_async.dart';
import 'package:flutter_tools/src/base/async_guard.dart';

import '../../src/common.dart';

Future<void> asyncError() {
  final Completer<void> completer = Completer<void>();
  final Completer<void> errorCompleter = Completer<void>();
  errorCompleter.completeError(_CustomException('Async Doom'), StackTrace.current);
  return completer.future;
}

/// Specialized exception to be caught.
class _CustomException implements Exception {
  _CustomException(this.message);

  final String message;

  @override
  String toString() => message;
}

Future<void> syncError() {
  throw _CustomException('Sync Doom');
}

Future<void> syncAndAsyncError() {
  final Completer<void> errorCompleter = Completer<void>();
  errorCompleter.completeError(_CustomException('Async Doom'), StackTrace.current);
  throw _CustomException('Sync Doom');
}

Future<void> delayedThrow(FakeAsync time) {
  final Future<void> result = Future<void>.delayed(const Duration(milliseconds: 10)).then((
    _,
  ) async {
    throw _CustomException('Delayed Doom');
  });
  time.elapse(const Duration(seconds: 1));
  time.flushMicrotasks();
  return result;
}

void main() {
  late Completer<void> caughtInZone;
  bool caughtByZone = false;
  bool caughtByHandler = false;
  late Zone zone;

  setUp(() {
    caughtInZone = Completer<void>();
    caughtByZone = false;
    caughtByHandler = false;
    zone = Zone.current.fork(
      specification: ZoneSpecification(
        handleUncaughtError: (
          Zone self,
          ZoneDelegate parent,
          Zone zone,
          Object error,
          StackTrace stackTrace,
        ) {
          caughtByZone = true;
          if (!caughtInZone.isCompleted) {
            caughtInZone.complete();
          }
        },
      ),
    );
  });

  test('asyncError percolates through zone', () async {
    await zone.run(() async {
      try {
        // Completer is required or else we timeout.
        await Future.any(<Future<void>>[asyncError(), caughtInZone.future]);
      } on _CustomException {
        caughtByHandler = true;
      }
    });

    expect(caughtByZone, true);
    expect(caughtByHandler, false);
  });

  test('syncAndAsyncError percolates through zone', () async {
    await zone.run(() async {
      try {
        // Completer is required or else we timeout.
        await Future.any(<Future<void>>[syncAndAsyncError(), caughtInZone.future]);
      } on _CustomException {
        caughtByHandler = true;
      }
    });

    expect(caughtByZone, true);
    expect(caughtByHandler, true);
  });

  test('syncError percolates through zone', () async {
    await zone.run(() async {
      try {
        await syncError();
      } on _CustomException {
        caughtByHandler = true;
      }
    });

    expect(caughtByZone, false);
    expect(caughtByHandler, true);
  });

  test('syncError is caught by asyncGuard', () async {
    await zone.run(() async {
      try {
        await asyncGuard(syncError);
      } on _CustomException {
        caughtByHandler = true;
      }
    });

    expect(caughtByZone, false);
    expect(caughtByHandler, true);
  });

  test('asyncError is caught by asyncGuard', () async {
    await zone.run(() async {
      try {
        await asyncGuard(asyncError);
      } on _CustomException {
        caughtByHandler = true;
      }
    });

    expect(caughtByZone, false);
    expect(caughtByHandler, true);
  });

  test('asyncAndSyncError is caught by asyncGuard', () async {
    await zone.run(() async {
      try {
        await asyncGuard(syncAndAsyncError);
      } on _CustomException {
        caughtByHandler = true;
      }
    });

    expect(caughtByZone, false);
    expect(caughtByHandler, true);
  });

  test('asyncError is missed when catchError is attached too late', () async {
    bool caughtByZone = false;
    bool caughtByHandler = false;
    bool caughtByCatchError = false;

    final Completer<void> completer = Completer<void>();
    await FakeAsync().run((FakeAsync time) {
      unawaited(
        runZonedGuarded(
          () async {
            final Future<void> f = asyncGuard<void>(() => delayedThrow(time)).then(
              (Object? obj) => obj,
              onError: (Object e, StackTrace s) {
                caughtByCatchError = true;
              },
            );
            try {
              await f;
            } on _CustomException {
              caughtByHandler = true;
            }
            if (!completer.isCompleted) {
              completer.complete();
            }
          },
          (Object e, StackTrace s) {
            caughtByZone = true;
            if (!completer.isCompleted) {
              completer.complete();
            }
          },
        ),
      );
      time.elapse(const Duration(seconds: 1));
      time.flushMicrotasks();
      return completer.future;
    });

    expect(caughtByZone, true);
    expect(caughtByHandler, false);
    expect(caughtByCatchError, true);
  });

  test('asyncError is propagated with binary onError', () async {
    bool caughtByZone = false;
    bool caughtByHandler = false;
    bool caughtByOnError = false;

    final Completer<void> completer = Completer<void>();
    await FakeAsync().run((FakeAsync time) {
      unawaited(
        runZonedGuarded(
          () async {
            final Future<void> f = asyncGuard<void>(
              () => delayedThrow(time),
              onError: (Object e, StackTrace s) {
                caughtByOnError = true;
              },
            );
            try {
              await f;
            } on _CustomException {
              caughtByHandler = true;
            }
            if (!completer.isCompleted) {
              completer.complete();
            }
          },
          (Object e, StackTrace s) {
            caughtByZone = true;
            if (!completer.isCompleted) {
              completer.complete();
            }
          },
        ),
      );
      time.elapse(const Duration(seconds: 1));
      time.flushMicrotasks();
      return completer.future;
    });

    expect(caughtByZone, false);
    expect(caughtByHandler, false);
    expect(caughtByOnError, true);
  });

  test('asyncError is propagated with unary onError', () async {
    bool caughtByZone = false;
    bool caughtByHandler = false;
    bool caughtByOnError = false;

    final Completer<void> completer = Completer<void>();
    await FakeAsync().run((FakeAsync time) {
      unawaited(
        runZonedGuarded(
          () async {
            final Future<void> f = asyncGuard<void>(
              () => delayedThrow(time),
              onError: (Object e) {
                caughtByOnError = true;
              },
            );
            try {
              await f;
            } on _CustomException {
              caughtByHandler = true;
            }
            if (!completer.isCompleted) {
              completer.complete();
            }
          },
          (Object e, StackTrace s) {
            caughtByZone = true;
            if (!completer.isCompleted) {
              completer.complete();
            }
          },
        ),
      );
      time.elapse(const Duration(seconds: 1));
      time.flushMicrotasks();
      return completer.future;
    });

    expect(caughtByZone, false);
    expect(caughtByHandler, false);
    expect(caughtByOnError, true);
  });

  test('asyncError is propagated with optional stack trace', () async {
    bool caughtByZone = false;
    bool caughtByHandler = false;
    bool caughtByOnError = false;
    bool nonNullStackTrace = false;

    final Completer<void> completer = Completer<void>();
    await FakeAsync().run((FakeAsync time) {
      unawaited(
        runZonedGuarded(
          () async {
            final Future<void> f = asyncGuard<void>(
              () => delayedThrow(time),
              onError: (Object e, [StackTrace? s]) {
                caughtByOnError = true;
                nonNullStackTrace = s != null;
              },
            );
            try {
              await f;
            } on _CustomException {
              caughtByHandler = true;
            }
            if (!completer.isCompleted) {
              completer.complete();
            }
          },
          (Object e, StackTrace s) {
            caughtByZone = true;
            if (!completer.isCompleted) {
              completer.complete();
            }
          },
        ),
      );
      time.elapse(const Duration(seconds: 1));
      time.flushMicrotasks();
      return completer.future;
    });

    expect(caughtByZone, false);
    expect(caughtByHandler, false);
    expect(caughtByOnError, true);
    expect(nonNullStackTrace, true);
  });
}
