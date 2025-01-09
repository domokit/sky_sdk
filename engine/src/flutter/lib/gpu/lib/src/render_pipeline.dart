// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: public_member_api_docs

part of flutter_gpu;

base class RenderPipeline extends NativeFieldWrapperClass1 {
  /// Creates a new RenderPipeline.
  RenderPipeline._(
    GpuContext gpuContext,
    Shader vertexShader,
    Shader fragmentShader,
  ) : vertexShader = vertexShader,
      fragmentShader = fragmentShader {
    String? error = _initialize(gpuContext, vertexShader, fragmentShader);
    if (error != null) {
      throw Exception(error);
    }
  }

  final Shader vertexShader;
  final Shader fragmentShader;

  /// Wrap with native counterpart.
  @Native<Handle Function(Handle, Pointer<Void>, Pointer<Void>, Pointer<Void>)>(
    symbol: 'InternalFlutterGpu_RenderPipeline_Initialize',
  )
  external String? _initialize(
    GpuContext gpuContext,
    Shader vertexShader,
    Shader fragmentShader,
  );
}
