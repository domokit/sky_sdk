// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Built-in types and core primitives for a Flutter application.
///
/// To use, import `dart:ui`.
///
/// This library exposes the lowest-level services that Flutter frameworks use
/// to bootstrap applications, such as classes for driving the input, graphics
/// text, layout, and rendering subsystems.
library dart.ui;

import 'dart:async';
import 'dart:collection' as collection;
import 'dart:convert';
import 'dart:developer' as developer;
import 'dart:ffi';
import 'dart:io';
import 'dart:isolate' show Isolate, IsolateSpawnException, RawReceivePort, RemoteError, SendPort;
import 'dart:math' as math;
import 'dart:nativewrappers';
import 'dart:typed_data';

part 'annotations.dart';
part 'channel_buffers.dart';
part 'compositing.dart';
part 'geometry.dart';
part 'hooks.dart';
part 'isolate_name_server.dart';
part 'key.dart';
part 'lerp.dart';
part 'math.dart';
part 'natives.dart';
part 'painting.dart';
part 'platform_dispatcher.dart';
part 'platform_isolate.dart';
part 'plugins.dart';
part 'pointer.dart';
part 'semantics.dart';
part 'setup_hooks.dart';
part 'text.dart';
part 'window.dart';
