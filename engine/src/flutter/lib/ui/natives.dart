// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
part of dart.ui;

// ignore_for_file: avoid_classes_with_only_static_members

/// Helper functions for Dart Plugin Registrants.
abstract final class DartPluginRegistrant {
  static bool _wasInitialized = false;

  /// Makes sure the that the Dart Plugin Registrant has been called for this
  /// isolate. This can safely be executed multiple times on the same isolate,
  /// but should not be called on the Root isolate.
  static void ensureInitialized() {
    if (!_wasInitialized) {
      _wasInitialized = true;
      _ensureInitialized();
    }
  }

  @Native<Void Function()>(symbol: 'DartPluginRegistrant_EnsureInitialized')
  external static void _ensureInitialized();
}

// Corelib 'print' implementation.
void _print(String arg) {
  _Logger._printString(arg);
}

void _printDebug(String arg) {
  _Logger._printDebugString(arg);
}

class _Logger {
  @Native<Void Function(Handle)>(symbol: 'DartRuntimeHooks::Logger_PrintString')
  external static void _printString(String? s);

  @Native<Void Function(Handle)>(symbol: 'DartRuntimeHooks::Logger_PrintDebugString')
  external static void _printDebugString(String? s);
}

// If we actually run on big endian machines, we'll need to do something smarter
// here. We don't use [Endian.Host] because it's not a compile-time
// constant and can't propagate into the set/get calls.
const Endian _kFakeHostEndian = Endian.little;

// A service protocol extension to schedule a frame to be rendered into the
// window.
Future<developer.ServiceExtensionResponse> _scheduleFrame(
  String method,
  Map<String, String> parameters,
) async {
  // Schedule the frame.
  PlatformDispatcher.instance.scheduleFrame();
  // Always succeed.
  return developer.ServiceExtensionResponse.result(
    json.encode(<String, String>{'type': 'Success'}),
  );
}

Future<developer.ServiceExtensionResponse> _reinitializeShader(
  String method,
  Map<String, String> parameters,
) async {
  final String? assetKey = parameters['assetKey'];
  if (assetKey != null) {
    FragmentProgram._reinitializeShader(assetKey);
  }

  // Always succeed.
  return developer.ServiceExtensionResponse.result(
    json.encode(<String, String>{'type': 'Success'}),
  );
}

Future<developer.ServiceExtensionResponse> _getImpellerEnabled(
  String method,
  Map<String, String> parameters,
) async {
  return developer.ServiceExtensionResponse.result(
    json.encode(<String, Object>{'type': 'Success', 'enabled': _impellerEnabled}),
  );
}

const bool _kReleaseMode = bool.fromEnvironment('dart.vm.product');

@Native<Void Function(Handle)>(symbol: 'DartRuntimeHooks::ScheduleMicrotask')
external void _scheduleMicrotask(void Function() callback);

@Native<Handle Function(Handle)>(symbol: 'DartRuntimeHooks::GetCallbackHandle')
external int? _getCallbackHandle(Function closure);

@Native<Handle Function(Int64)>(symbol: 'DartRuntimeHooks::GetCallbackFromHandle')
external Function? _getCallbackFromHandle(int handle);

typedef _PrintClosure = void Function(String line);

// Used by the embedder to initialize how printing is performed.
// See also https://github.com/dart-lang/sdk/blob/main/sdk/lib/_internal/vm/lib/print_patch.dart
@pragma('vm:entry-point')
_PrintClosure _getPrintClosure() => _print;

typedef _ScheduleImmediateClosure = void Function(void Function());

// Used by the embedder to initialize how microtasks are scheduled.
// See also https://github.com/dart-lang/sdk/blob/main/sdk/lib/_internal/vm/lib/schedule_microtask_patch.dart
@pragma('vm:entry-point')
_ScheduleImmediateClosure _getScheduleMicrotaskClosure() => _scheduleMicrotask;

// Used internally to indicate whether the Engine is using Impeller for
// rendering.
@pragma('vm:entry-point')
bool _impellerEnabled = false;

// Used internally to indicate whether the embedder enables the implicit view,
// and the implicit view's ID if so.
//
// The exact value of this variable is an implementation detail that may change
// at any time. Apps should always use PlatformDispatcher.implicitView to
// determine the current implicit view, if any.
@pragma('vm:entry-point')
int? _implicitViewId;

// Used internally to indicate whether isolates running on the platform thread
// are enabled.
@pragma('vm:entry-point')
bool _platformIsolatesEnabled = false;
