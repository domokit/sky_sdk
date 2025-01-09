// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:developer';
///
/// @docImport 'package:flutter/foundation.dart';
/// @docImport 'package:flutter/rendering.dart';
/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:ui' as ui show Brightness;

import 'assertions.dart';
import 'platform.dart';
import 'print.dart';

export 'dart:ui' show Brightness;

export 'print.dart' show DebugPrintCallback;

/// Returns true if none of the foundation library debug variables have been
/// changed.
///
/// This function is used by the test framework to ensure that debug variables
/// haven't been inadvertently changed.
///
/// The `debugPrintOverride` argument can be specified to indicate the expected
/// value of the [debugPrint] variable. This is useful for test frameworks that
/// override [debugPrint] themselves and want to check that their own custom
/// value wasn't overridden by a test.
///
/// See [the foundation library](foundation/foundation-library.html)
/// for a complete list.
bool debugAssertAllFoundationVarsUnset(
  String reason, {
  DebugPrintCallback debugPrintOverride = debugPrintThrottled,
}) {
  assert(() {
    if (debugPrint != debugPrintOverride ||
        debugDefaultTargetPlatformOverride != null ||
        debugDoublePrecision != null ||
        debugBrightnessOverride != null) {
      throw FlutterError(reason);
    }
    return true;
  }());
  return true;
}

/// Boolean value indicating whether [debugInstrumentAction] will instrument
/// actions in debug builds.
///
/// The framework does not use [debugInstrumentAction] internally, so this
/// does not enable any additional instrumentation for the framework itself.
///
/// See also:
///
///  * [debugProfileBuildsEnabled], which enables additional tracing of builds
///    in [Widget]s.
///  * [debugProfileLayoutsEnabled], which enables additional tracing of layout
///    events in [RenderObject]s.
///  * [debugProfilePaintsEnabled], which enables additional tracing of paint
///    events in [RenderObject]s.
bool debugInstrumentationEnabled = false;

/// Runs the specified [action], timing how long the action takes in debug
/// builds when [debugInstrumentationEnabled] is true.
///
/// The instrumentation will be printed to the logs using [debugPrint]. In
/// non-debug builds, or when [debugInstrumentationEnabled] is false, this will
/// run [action] without any instrumentation.
///
/// Returns the result of running [action].
///
/// See also:
///
///  * [Timeline], which is used to record synchronous tracing events for
///    visualization in Chrome's tracing format. This method does not
///    implicitly add any timeline events.
Future<T> debugInstrumentAction<T>(String description, Future<T> Function() action) async {
  bool instrument = false;
  assert(() {
    instrument = debugInstrumentationEnabled;
    return true;
  }());
  if (instrument) {
    final Stopwatch stopwatch =
        Stopwatch()..start(); // flutter_ignore: stopwatch (see analyze.dart)
    // Ignore context: The framework does not use this function internally so it will not cause flakes.
    try {
      return await action();
    } finally {
      stopwatch.stop();
      debugPrint('Action "$description" took ${stopwatch.elapsed}');
    }
  } else {
    return action();
  }
}

/// Configure [debugFormatDouble] using [num.toStringAsPrecision].
///
/// Defaults to null, which uses the default logic of [debugFormatDouble].
int? debugDoublePrecision;

/// Formats a double to have standard formatting.
///
/// This behavior can be overridden by [debugDoublePrecision].
String debugFormatDouble(double? value) {
  if (value == null) {
    return 'null';
  }
  if (debugDoublePrecision != null) {
    return value.toStringAsPrecision(debugDoublePrecision!);
  }
  return value.toStringAsFixed(1);
}

/// A setting that can be used to override the platform [Brightness] exposed
/// from [BindingBase.platformDispatcher].
///
/// See also:
///
///  * [WidgetsApp], which uses the [debugBrightnessOverride] setting in debug mode
///    to construct a [MediaQueryData].
ui.Brightness? debugBrightnessOverride;

/// The address for the active DevTools server used for debugging this
/// application.
String? activeDevToolsServerAddress;

/// The uri for the connected vm service protocol.
String? connectedVmServiceUri;
