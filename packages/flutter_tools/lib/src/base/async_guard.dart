// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

/// Runs [fn] with special handling of asynchronous errors.
///
/// If the execution of [fn] does not throw a synchronous exception, and if the
/// [Future] returned by [fn] is completed with a value, then the [Future]
/// returned by [asyncGuard] is completed with that value if it has not already
/// been completed with an error.
///
/// If the execution of [fn] throws a synchronous exception, and no [onError]
/// callback is provided, then the [Future] returned by [asyncGuard] is
/// completed with an error whose object and stack trace are given by the
/// synchronous exception. If an [onError] callback is provided, then the
/// [Future] returned by [asyncGuard] is completed with its result when passed
/// the error object and stack trace.
///
/// If the execution of [fn] results in an asynchronous exception that would
/// otherwise be unhandled, and no [onError] callback is provided, then the
/// [Future] returned by [asyncGuard] is completed with an error whose object
/// and stack trace are given by the asynchronous exception. If an [onError]
/// callback is provided, then the [Future] returned by [asyncGuard] is
/// completed with its result when passed the error object and stack trace.
///
/// After the returned [Future] is completed, whether it be with a value or an
/// error, all further errors resulting from the execution of [fn] are ignored.
///
/// Rationale:
///
/// Consider the following snippet:
/// ```dart
/// try {
///   await foo();
///   ...
/// } catch (e) {
///   ...
/// }
/// ```
/// If the [Future] returned by `foo` is completed with an error, that error is
/// handled by the catch block. However, if `foo` spawns an asynchronous
/// operation whose errors are unhandled, those errors will not be caught by
/// the catch block, and will instead propagate to the containing [Zone]. This
/// behavior is non-intuitive to programmers expecting the `catch` to catch all
/// the errors resulting from the code under the `try`.
///
/// As such, it would be convenient if the `try {} catch {}` here could handle
/// not only errors completing the awaited [Future]s it contains, but also
/// any otherwise unhandled asynchronous errors occurring as a result of awaited
/// expressions. This is how `await` is often assumed to work, which leads to
/// unexpected unhandled exceptions.
///
/// [asyncGuard] is intended to wrap awaited expressions occurring in a `try`
/// block. The behavior described above gives the behavior that users
/// intuitively expect from `await`. Consider the snippet:
/// ```dart
/// try {
///   await asyncGuard(() async {
///     var c = Completer();
///     c.completeError('Error');
///   });
/// } catch (e) {
///   // e is 'Error';
/// }
/// ```
/// Without the [asyncGuard] the error 'Error' would be propagated to the
/// error handler of the containing [Zone]. With the [asyncGuard], the error
/// 'Error' is instead caught by the `catch`.
///
/// [asyncGuard] also accepts an [onError] callback for situations in which
/// completing the returned [Future] with an error is not appropriate.
/// For example, it is not always possible to immediately await the returned
/// [Future]. In these cases, an [onError] callback is needed to prevent an
/// error from propagating to the containing [Zone].
///
/// [onError] must have type `FutureOr<T> Function(Object error)` or
/// `FutureOr<T> Function(Object error, StackTrace stackTrace)` otherwise an
/// [ArgumentError] will be thrown synchronously.
Future<T> asyncGuard<T>(Future<T> Function() fn, {Function? onError}) {
  if (onError != null && onError is! _UnaryOnError<T> && onError is! _BinaryOnError<T>) {
    throw ArgumentError(
      'onError must be a unary function accepting an Object, '
      'or a binary function accepting an Object and '
      'StackTrace. onError must return a T',
    );
  }
  final Completer<T> completer = Completer<T>();

  void handleError(Object e, StackTrace s) {
    if (completer.isCompleted) {
      return;
    }
    if (onError == null) {
      completer.completeError(e, s);
      return;
    }
    if (onError is _BinaryOnError<T>) {
      completer.complete(onError(e, s));
    } else if (onError is _UnaryOnError<T>) {
      completer.complete(onError(e));
    }
  }

  runZoned<void>(
    () async {
      try {
        final T result = await fn();
        if (!completer.isCompleted) {
          completer.complete(result);
        }
        // This catches all exceptions so that they can be propagated to the
        // caller-supplied error handling or the completer.
        // ignore: avoid_catches_without_on_clauses, forwards to Future
      } catch (e, s) {
        handleError(e, s);
      }
    },
    onError: (Object e, StackTrace s) {
      handleError(e, s);
    },
  );

  return completer.future;
}

typedef _UnaryOnError<T> = FutureOr<T> Function(Object error);
typedef _BinaryOnError<T> = FutureOr<T> Function(Object error, StackTrace stackTrace);
