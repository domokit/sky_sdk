// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Flutter GPU is a low level API for building rendering packages from scratch.
///
/// To use, first add an SDK dependency in your `pubspec.yaml` file:
/// ```
/// dependencies:
///   flutter_gpu:
///     sdk: flutter
/// ```
/// And then add an import statement in your Dart files:
/// ```dart
/// import `package:flutter_gpu/gpu.dart`;
/// ```
///
/// See also:
///
///  * [Flutter GPU documentation](https://github.com/flutter/engine/blob/main/docs/impeller/Flutter-GPU.md).
library flutter_gpu;

import 'dart:ffi';
import 'dart:nativewrappers';
import 'dart:typed_data';
// ignore: uri_does_not_exist
import 'dart:ui' as ui;

import 'package:vector_math/vector_math.dart' as vm;

part 'src/buffer.dart';
part 'src/command_buffer.dart';
part 'src/context.dart';
part 'src/formats.dart';
part 'src/texture.dart';
part 'src/render_pass.dart';
part 'src/render_pipeline.dart';
part 'src/shader.dart';
part 'src/shader_library.dart';
