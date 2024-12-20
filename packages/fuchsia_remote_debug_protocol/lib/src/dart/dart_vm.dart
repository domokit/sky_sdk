// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:vm_service/vm_service.dart' as vms;

import '../common/logging.dart';

const Duration _kConnectTimeout = Duration(seconds: 3);
final Logger _log = Logger('DartVm');

/// Signature of an asynchronous function for establishing a [vms.VmService]
/// connection to a [Uri].
typedef RpcPeerConnectionFunction =
    Future<vms.VmService> Function(Uri uri, {required Duration timeout});

/// [DartVm] uses this function to connect to the Dart VM on Fuchsia.
///
/// This function can be assigned to a different one in the event that a
/// custom connection function is needed.
RpcPeerConnectionFunction fuchsiaVmServiceConnectionFunction = _waitAndConnect;

/// Attempts to connect to a Dart VM service.
///
/// Gives up after `timeout` has elapsed.
Future<vms.VmService> _waitAndConnect(Uri uri, {Duration timeout = _kConnectTimeout}) async {
  int attempts = 0;
  late WebSocket socket;
  while (true) {
    try {
      socket = await WebSocket.connect(uri.toString());
      final StreamController<dynamic> controller = StreamController<dynamic>();
      final Completer<void> streamClosedCompleter = Completer<void>();
      socket.listen(
        (dynamic data) => controller.add(data),
        onDone: () => streamClosedCompleter.complete(),
      );
      final vms.VmService service = vms.VmService(
        controller.stream,
        socket.add,
        disposeHandler: () => socket.close(),
        streamClosed: streamClosedCompleter.future,
      );
      // This call is to ensure we are able to establish a connection instead of
      // keeping on trucking and failing farther down the process.
      await service.getVersion();
      return service;
    } catch (e) {
      // We should not be catching all errors arbitrarily here, this might hide real errors.
      // TODO(ianh): Determine which exceptions to catch here.
      await socket.close();
      if (attempts > 5) {
        _log.warning('It is taking an unusually long time to connect to the VM...');
      }
      attempts += 1;
      await Future<void>.delayed(timeout);
    }
  }
}

/// Restores the VM service connection function to the default implementation.
void restoreVmServiceConnectionFunction() {
  fuchsiaVmServiceConnectionFunction = _waitAndConnect;
}

/// An error raised when a malformed RPC response is received from the Dart VM.
///
/// A more detailed description of the error is found within the [message]
/// field.
class RpcFormatError extends Error {
  /// Basic constructor outlining the reason for the format error.
  RpcFormatError(this.message);

  /// The reason for format error.
  final String message;

  @override
  String toString() {
    return '$RpcFormatError: $message\n${super.stackTrace}';
  }
}

/// Handles JSON RPC-2 communication with a Dart VM service.
///
/// Wraps existing RPC calls to the Dart VM service.
class DartVm {
  DartVm._(this._vmService, this.uri);

  final vms.VmService _vmService;

  /// The URL through which this DartVM instance is connected.
  final Uri uri;

  /// Attempts to connect to the given [Uri].
  ///
  /// Throws an error if unable to connect.
  static Future<DartVm> connect(Uri uri, {Duration timeout = _kConnectTimeout}) async {
    if (uri.scheme == 'http') {
      uri = uri.replace(scheme: 'ws', path: '/ws');
    }

    final vms.VmService service = await fuchsiaVmServiceConnectionFunction(uri, timeout: timeout);
    return DartVm._(service, uri);
  }

  /// Returns a [List] of [IsolateRef] objects whose name matches `pattern`.
  ///
  /// This is not limited to Isolates running Flutter, but to any Isolate on the
  /// VM. Therefore, the [pattern] argument should be written to exclude
  /// matching unintended isolates.
  Future<List<IsolateRef>> getMainIsolatesByPattern(Pattern pattern) async {
    final vms.VM vmRef = await _vmService.getVM();
    final List<IsolateRef> result = <IsolateRef>[];
    for (final vms.IsolateRef isolateRef in vmRef.isolates!) {
      if (pattern.matchAsPrefix(isolateRef.name!) != null) {
        _log.fine('Found Isolate matching "$pattern": "${isolateRef.name}"');
        result.add(IsolateRef._fromJson(isolateRef.json!, this));
      }
    }
    return result;
  }

  /// Returns a list of [FlutterView] objects running across all Dart VM's.
  ///
  /// If there is no associated isolate with the flutter view (used to determine
  /// the flutter view's name), then the flutter view's ID will be added
  /// instead. If none of these things can be found (isolate has no name or the
  /// flutter view has no ID), then the result will not be added to the list.
  Future<List<FlutterView>> getAllFlutterViews() async {
    final List<FlutterView> views = <FlutterView>[];
    final vms.Response rpcResponse = await _vmService.callMethod('_flutter.listViews');
    for (final Map<String, dynamic> jsonView
        in (rpcResponse.json!['views'] as List<dynamic>).cast<Map<String, dynamic>>()) {
      views.add(FlutterView._fromJson(jsonView));
    }
    return views;
  }

  /// Tests that the connection to the [vms.VmService] is valid.
  Future<void> ping() async {
    final vms.Version version = await _vmService.getVersion();
    _log.fine('DartVM($uri) version check result: $version');
  }

  /// Disconnects from the Dart VM Service.
  ///
  /// After this function completes this object is no longer usable.
  Future<void> stop() async {
    await _vmService.dispose();
    await _vmService.onDone;
  }
}

/// Represents an instance of a Flutter view running on a Fuchsia device.
class FlutterView {
  FlutterView._(this._name, this._id);

  /// Attempts to construct a [FlutterView] from a json representation.
  ///
  /// If there is no isolate and no ID for the view, throws an [RpcFormatError].
  /// If there is an associated isolate, and there is no name for said isolate,
  /// also throws an [RpcFormatError].
  ///
  /// All other cases return a [FlutterView] instance. The name of the
  /// view may be null, but the id will always be set.
  factory FlutterView._fromJson(Map<String, dynamic> json) {
    final Map<String, dynamic>? isolate = json['isolate'] as Map<String, dynamic>?;
    final String? id = json['id'] as String?;
    String? name;
    if (id == null) {
      throw RpcFormatError('Unable to find view name for the following JSON structure "$json"');
    }
    if (isolate != null) {
      name = isolate['name'] as String?;
      if (name == null) {
        throw RpcFormatError('Unable to find name for isolate "$isolate"');
      }
    }
    return FlutterView._(name, id);
  }

  /// Determines the name of the isolate associated with this view. If there is
  /// no associated isolate, this will be set to the view's ID.
  final String? _name;

  /// The ID of the Flutter view.
  final String _id;

  /// The ID of the [FlutterView].
  String get id => _id;

  /// Returns the name of the [FlutterView].
  ///
  /// May be null if there is no associated isolate.
  String? get name => _name;
}

/// This is a wrapper class for the `@Isolate` RPC object.
///
/// See:
/// https://github.com/dart-lang/sdk/blob/main/runtime/vm/service/service.md#isolate
///
/// This class contains information about the Isolate like its name and ID, as
/// well as a reference to the parent DartVM on which it is running.
class IsolateRef {
  IsolateRef._(this.name, this.number, this.dartVm);

  factory IsolateRef._fromJson(Map<String, dynamic> json, DartVm dartVm) {
    final String? number = json['number'] as String?;
    final String? name = json['name'] as String?;
    final String? type = json['type'] as String?;
    if (type == null) {
      throw RpcFormatError('Unable to find type within JSON "$json"');
    }
    if (type != '@Isolate') {
      throw RpcFormatError('Type "$type" does not match for IsolateRef');
    }
    if (number == null) {
      throw RpcFormatError('Unable to find number for isolate ref within JSON "$json"');
    }
    if (name == null) {
      throw RpcFormatError('Unable to find name for isolate ref within JSON "$json"');
    }
    return IsolateRef._(name, int.parse(number), dartVm);
  }

  /// The full name of this Isolate (not guaranteed to be unique).
  final String name;

  /// The unique number ID of this isolate.
  final int number;

  /// The parent [DartVm] on which this Isolate lives.
  final DartVm dartVm;
}
