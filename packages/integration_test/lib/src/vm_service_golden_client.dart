// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Examples can assume:
// import 'dart:developer' as dev;

import 'dart:async';
import 'dart:convert';
import 'dart:developer' as dev;
import 'dart:io' as io;

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

/// Compares image pixels against a golden image file on the host system.
///
/// This comparator will send a request, using the VM service protocol, to a
/// host script (i.e. the _driver_ script, running in a Dart VM on the host
/// desktop OS), which will then forward the comparison to a concrete
/// [GoldenFileComparator].
///
/// To use, run [useIfRunningOnDevice] in the `main()` of a test file or similar:
///
/// ```dart
/// import 'package:integration_test/integration_test.dart';
///
/// void main() {
///   VmServiceProxyGoldenFileComparator.useIfRunningOnDevice();
///
///   // Actual tests and such below.
/// }
/// ```
///
/// When either [compare] or [update] is called, the following event is sent
/// with [dev.postEvent]:
///
/// ```dart
/// dev.postEvent('compare' /* or 'update' */, <String, Object?>{
///   'id':    1001,                 // a valid unique integer, often incrementing;
///   'path':  'path/to/image.png',  // golden key created by matchesGoldenFile;
///   'bytes': '...base64encoded',   // base64 encoded bytes representing the current image.
/// }, stream: 'integration_test.VmServiceProxyGoldenFileComparator');
/// ```
///
/// The comparator expects a response at the service extension
/// `ext.integration_test.VmServiceProxyGoldenFileComparator` that is either
/// of the following formats:
///
/// ```dart
/// <String, Object?>{
///   'error': 'Description of why the operation failed'
/// }
/// ```
///
/// or:
///
/// ```dart
/// <String, Object?>{
///   'result': true /* or possibly false, in the case of 'compare' calls */
/// }
/// ```
///
/// See also:
///
///   * [matchesGoldenFile], the function that invokes the comparator.
final class VmServiceProxyGoldenFileComparator extends GoldenFileComparator {
  VmServiceProxyGoldenFileComparator._() : _postEvent = dev.postEvent {
    dev.registerExtension(_kServiceName, (_, Map<String, String> parameters) {
      return handleEvent(parameters);
    });
  }

  /// Creates an instance of [VmServiceProxyGoldenFileComparator] for internal testing.
  ///
  /// @nodoc
  @visibleForTesting
  VmServiceProxyGoldenFileComparator.forTesting(this._postEvent);

  static bool get _isRunningOnHost {
    if (kIsWeb) {
      return false;
    }
    return !io.Platform.isAndroid && !io.Platform.isIOS;
  }

  static void _assertNotRunningOnFuchsia() {
    if (!kIsWeb && io.Platform.isFuchsia) {
      throw UnsupportedError('Golden testing with integration_test does not support Fuchsia.');
    }
  }

  /// Conditionally sets [goldenFileComparator] to [VmServiceProxyGoldenFileComparator].
  ///
  /// If running on a non-mobile non-web platform (i.e. desktop), this method has no effect.
  static void useIfRunningOnDevice() {
    if (_isRunningOnHost) {
      return;
    }
    _assertNotRunningOnFuchsia();
    goldenFileComparator = _kInstance;
  }

  static final GoldenFileComparator _kInstance = VmServiceProxyGoldenFileComparator._();
  static const String _kServiceName = 'ext.$_kEventName';
  static const String _kEventName = 'integration_test.VmServiceProxyGoldenFileComparator';
  final void Function(String, Map<Object?, Object?>, {String stream}) _postEvent;

  /// Handles the received method and parameters as an incoming event.
  ///
  /// Each event is treated as if it were received by the Dart developer
  /// extension protocol; this method is public only to be able to write unit
  /// tests that do not have to bring up and use a VM service.
  ///
  /// @nodoc
  @visibleForTesting
  Future<dev.ServiceExtensionResponse> handleEvent(Map<String, String> parameters) async {
    // Treat the method as the ID number of the pending request.
    final String? methodIdString = parameters['id'];
    if (methodIdString == null) {
      return dev.ServiceExtensionResponse.error(
        dev.ServiceExtensionResponse.extensionError,
        'Required parameter "id" not present in response.',
      );
    }
    final int? methodId = int.tryParse(methodIdString);
    if (methodId == null) {
      return dev.ServiceExtensionResponse.error(
        dev.ServiceExtensionResponse.extensionError,
        'Required parameter "id" not a valid integer: "$methodIdString".',
      );
    }
    final Completer<_Result>? completer = _pendingRequests[methodId];
    if (completer == null) {
      return dev.ServiceExtensionResponse.error(
        dev.ServiceExtensionResponse.extensionError,
        'No pending request with method ID "$methodIdString".',
      );
    }
    assert(!completer.isCompleted, 'Can never occur, as the completer should be removed');
    final String? error = parameters['error'];
    if (error != null) {
      completer.complete(_Failure(error));
      return dev.ServiceExtensionResponse.result('{}');
    }
    final String? result = parameters['result'];
    if (result == null) {
      return dev.ServiceExtensionResponse.error(
        dev.ServiceExtensionResponse.invalidParams,
        'Required parameter "result" not present in response.',
      );
    }
    if (bool.tryParse(result) case final bool result) {
      completer.complete(_Success(result));
      return dev.ServiceExtensionResponse.result('{}');
    } else {
      return dev.ServiceExtensionResponse.error(
        dev.ServiceExtensionResponse.invalidParams,
        'Required parameter "result" not a valid boolean: "$result".',
      );
    }
  }

  int _nextId = 0;
  final Map<int, Completer<_Result>> _pendingRequests = <int, Completer<_Result>>{};

  Future<_Result> _postAndWait(
    Uint8List imageBytes,
    Uri golden, {
    required String operation,
  }) async {
    final int nextId = ++_nextId;
    assert(!_pendingRequests.containsKey(nextId));

    final Completer<_Result> completer = Completer<_Result>();
    _postEvent(operation, <String, Object?>{
      'id': nextId,
      'path': '$golden',
      'bytes': base64.encode(imageBytes),
    }, stream: _kEventName);

    _pendingRequests[nextId] = completer;
    completer.future.whenComplete(() {
      _pendingRequests.remove(nextId);
    });
    return completer.future;
  }

  @override
  Future<bool> compare(Uint8List imageBytes, Uri golden) async {
    return switch (await _postAndWait(imageBytes, golden, operation: 'compare')) {
      _Success(:final bool result) => result,
      _Failure(:final String error) => Future<bool>.error(error),
    };
  }

  @override
  Future<void> update(Uri golden, Uint8List imageBytes) async {
    final _Result result = await _postAndWait(imageBytes, golden, operation: 'update');
    if (result is _Failure) {
      return Future<void>.error(result.error);
    }
  }
}

// These wrapper classes allow us to use a Completer to indicate both a failed
// response and a successful response, without making a call of completeError
// within handleEvent, which is difficult or impossible to use correctly because
// of the semantics of error zones.
//
// Of course, this is a private implementation detail, others are welcome to try
// an alternative approach that might simplify the code above, but it's probably
// not worth it.
sealed class _Result {}

final class _Success implements _Result {
  _Success(this.result);
  final bool result;
}

final class _Failure implements _Result {
  _Failure(this.error);
  final String error;
}
