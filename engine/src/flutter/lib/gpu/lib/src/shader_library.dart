// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: public_member_api_docs

part of flutter_gpu;

base class ShaderLibrary extends NativeFieldWrapperClass1 {
  static ShaderLibrary? fromAsset(String assetName) {
    final lib = ShaderLibrary._();
    final error = lib._initializeWithAsset(assetName);
    if (error != null) {
      throw Exception("Failed to initialize ShaderLibrary: ${error}");
    }
    return lib;
  }

  ShaderLibrary._();

  // Hold a Dart-side reference to shaders in the library as they're wrapped for
  // the first time. This prevents the wrapper from getting prematurely
  // destroyed.
  final Map<String, Shader?> shaders_ = {};

  Shader? operator [](String shaderName) {
    // This `flutter_gpu` library isn't always registered as part of the builtin
    // DartClassLibrary, and so we can't instantiate the Dart classes on the
    // engine side.
    // Providing a new wrapper to [_getShader] for wrapping the native
    // counterpart (if it hasn't been wrapped already) is a hack to work around
    // this.
    return shaders_.putIfAbsent(
      shaderName,
      () => _getShader(shaderName, Shader._()),
    );
  }

  @Native<Handle Function(Handle, Handle)>(
    symbol: 'InternalFlutterGpu_ShaderLibrary_InitializeWithAsset',
  )
  external String? _initializeWithAsset(String assetName);

  @Native<Handle Function(Pointer<Void>, Handle, Handle)>(
    symbol: 'InternalFlutterGpu_ShaderLibrary_GetShader',
  )
  external Shader? _getShader(String shaderName, Shader shaderWrapper);
}
