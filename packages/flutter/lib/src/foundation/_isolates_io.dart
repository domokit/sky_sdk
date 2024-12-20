// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:isolate';

import 'constants.dart';
import 'isolates.dart' as isolates;

export 'isolates.dart' show ComputeCallback;

/// The dart:io implementation of [isolates.compute].
@pragma('vm:prefer-inline')
Future<R> compute<M, R>(
  isolates.ComputeCallback<M, R> callback,
  M message, {
  String? debugLabel,
}) async {
  debugLabel ??= kReleaseMode ? 'compute' : callback.toString();

  return Isolate.run<R>(() {
    return callback(message);
  }, debugName: debugLabel);
}
