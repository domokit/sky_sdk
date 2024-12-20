// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
part of dart.ui;

/// Runs [computation] on the platform thread and returns the result.
///
/// This may run the computation on a separate isolate. That isolate will be
/// reused for subsequent [runOnPlatformThread] calls. This means that global
/// state is maintained in that isolate between calls.
///
/// The [computation] and any state it captures may be sent to that isolate.
/// See [SendPort.send] for information about what types can be sent.
///
/// If [computation] is asynchronous (returns a `Future<R>`) then
/// that future is awaited in the new isolate, completing the entire
/// asynchronous computation, before returning the result.
///
/// If [computation] throws, the `Future` returned by this function completes
/// with that error.
///
/// The [computation] function and its result (or error) must be
/// sendable between isolates. Objects that cannot be sent include open
/// files and sockets (see [SendPort.send] for details).
///
/// This method can only be invoked from the main isolate.
///
/// This API is currently experimental.
Future<R> runOnPlatformThread<R>(FutureOr<R> Function() computation) {
  if (!_platformIsolatesEnabled) {
    throw UnsupportedError('Platform thread isolates are not supported by this platform.');
  }
  if (isRunningOnPlatformThread) {
    return Future<R>(computation);
  }
  final SendPort? sendPort = _platformRunnerSendPort;
  if (sendPort != null) {
    return _sendComputation(sendPort, computation);
  } else {
    return (_platformRunnerSendPortFuture ??= _spawnPlatformIsolate()).then(
      (SendPort port) => _sendComputation(port, computation),
    );
  }
}

SendPort? _platformRunnerSendPort;
Future<SendPort>? _platformRunnerSendPortFuture;
final Map<int, Completer<Object?>> _pending = <int, Completer<Object?>>{};
int _nextId = 0;

Future<SendPort> _spawnPlatformIsolate() {
  final Completer<SendPort> sendPortCompleter = Completer<SendPort>();
  final RawReceivePort receiver = RawReceivePort()..keepIsolateAlive = false;
  receiver.handler = (Object? message) {
    if (message == null) {
      // This is the platform isolate's onExit handler.
      // This shouldn't really happen, since Isolate.exit is disabled, the
      // pause and terminate capabilities aren't provided to the parent
      // isolate, and errors are fatal is false. But if the isolate does
      // shutdown unexpectedly, clear the singleton so we can create another.
      for (final Completer<Object?> completer in _pending.values) {
        completer.completeError(
          RemoteError('PlatformIsolate shutdown unexpectedly', StackTrace.empty.toString()),
        );
      }
      _pending.clear();
      _platformRunnerSendPort = null;
      _platformRunnerSendPortFuture = null;
    } else if (message is _PlatformIsolateReadyMessage) {
      _platformRunnerSendPort = message.computationPort;
      sendPortCompleter.complete(message.computationPort);
    } else if (message is _ComputationResult) {
      final Completer<Object?> resultCompleter = _pending.remove(message.id)!;
      final Object? remoteStack = message.remoteStack;
      final Object? remoteError = message.remoteError;
      if (remoteStack != null) {
        if (remoteStack is StackTrace) {
          // Typed error.
          resultCompleter.completeError(remoteError!, remoteStack);
        } else {
          // onError handler message, uncaught async error.
          // Both values are strings, so calling `toString` is efficient.
          final RemoteError error = RemoteError(remoteError!.toString(), remoteStack.toString());
          resultCompleter.completeError(error, error.stackTrace);
        }
      } else {
        resultCompleter.complete(message.result);
      }
    } else {
      // We encountered an error while starting the new isolate.
      if (!sendPortCompleter.isCompleted) {
        sendPortCompleter.completeError(IsolateSpawnException('Unable to spawn isolate: $message'));
        return;
      }
      // This shouldn't happen.
      throw IsolateSpawnException("Internal error: unexpected message: '$message'");
    }
  };
  final Isolate parentIsolate = Isolate.current;
  final SendPort sendPort = receiver.sendPort;
  try {
    _nativeSpawn(() => _platformIsolateMain(parentIsolate, sendPort));
  } on Object {
    receiver.close();
    rethrow;
  }
  return sendPortCompleter.future;
}

Future<R> _sendComputation<R>(SendPort port, FutureOr<R> Function() computation) {
  final int id = ++_nextId;
  final Completer<R> resultCompleter = Completer<R>();
  _pending[id] = resultCompleter;
  port.send(_ComputationRequest(id, computation));
  return resultCompleter.future;
}

void _safeSend(SendPort sendPort, int id, Object? result, Object? error, Object? stackTrace) {
  try {
    sendPort.send(_ComputationResult(id, result, error, stackTrace));
  } catch (sendError, sendStack) {
    sendPort.send(_ComputationResult(id, null, sendError, sendStack));
  }
}

void _platformIsolateMain(Isolate parentIsolate, SendPort sendPort) {
  final RawReceivePort computationPort = RawReceivePort();
  computationPort.handler = (_ComputationRequest? message) {
    if (message == null) {
      // The parent isolate has shutdown. Allow this isolate to shutdown.
      computationPort.keepIsolateAlive = false;
      return;
    }

    late final FutureOr<Object?> potentiallyAsyncResult;
    try {
      potentiallyAsyncResult = message.computation();
    } catch (e, s) {
      _safeSend(sendPort, message.id, null, e, s);
      return;
    }

    if (potentiallyAsyncResult is Future<Object?>) {
      potentiallyAsyncResult.then(
        (Object? result) {
          _safeSend(sendPort, message.id, result, null, null);
        },
        onError: (Object? e, Object? s) {
          _safeSend(sendPort, message.id, null, e, s ?? StackTrace.empty);
        },
      );
    } else {
      _safeSend(sendPort, message.id, potentiallyAsyncResult, null, null);
    }
  };
  Isolate.current.addOnExitListener(sendPort);
  parentIsolate.addOnExitListener(computationPort.sendPort);
  sendPort.send(_PlatformIsolateReadyMessage(computationPort.sendPort));
}

@Native<Void Function(Handle)>(symbol: 'PlatformIsolateNativeApi::Spawn')
external void _nativeSpawn(Function entryPoint);

/// Whether the current isolate is running on the platform thread.
final bool isRunningOnPlatformThread = _isRunningOnPlatformThread();

@Native<Bool Function()>(
  symbol: 'PlatformIsolateNativeApi::IsRunningOnPlatformThread',
  isLeaf: true,
)
external bool _isRunningOnPlatformThread();

class _PlatformIsolateReadyMessage {
  _PlatformIsolateReadyMessage(this.computationPort);

  final SendPort computationPort;
}

class _ComputationRequest {
  _ComputationRequest(this.id, this.computation);

  final int id;
  final FutureOr<Object?> Function() computation;
}

class _ComputationResult {
  _ComputationResult(this.id, this.result, this.remoteError, this.remoteStack);

  final int id;
  final Object? result;
  final Object? remoteError;
  final Object? remoteStack;
}
