// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: public_member_api_docs

part of flutter_gpu;

base class UniformSlot {
  UniformSlot._(this.shader, this.uniformName);
  final Shader shader;
  final String uniformName;

  /// The reflected total size of a shader's uniform struct by name.
  ///
  /// Returns [null] if the shader does not contain a uniform struct with the
  /// given name.
  int? get sizeInBytes {
    int size = shader._getUniformStructSize(uniformName);
    return size < 0 ? null : size;
  }

  /// Get the reflected offset of a named member in the uniform struct.
  ///
  /// Returns [null] if the shader does not contain a uniform struct with the
  /// given name, or if the uniform struct does not contain a member with the
  /// given name.
  int? getMemberOffsetInBytes(String memberName) {
    int offset = shader._getUniformMemberOffset(uniformName, memberName);
    return offset < 0 ? null : offset;
  }
}

base class Shader extends NativeFieldWrapperClass1 {
  // [Shader] handles are instantiated when interacting with a [ShaderLibrary].
  Shader._();

  UniformSlot getUniformSlot(String uniformName) {
    return UniformSlot._(this, uniformName);
  }

  @Native<Int Function(Pointer<Void>, Handle)>(
    symbol: 'InternalFlutterGpu_Shader_GetUniformStructSize',
  )
  external int _getUniformStructSize(String uniformStructName);

  @Native<Int Function(Pointer<Void>, Handle, Handle)>(
    symbol: 'InternalFlutterGpu_Shader_GetUniformMemberOffset',
  )
  external int _getUniformMemberOffset(
    String uniformStructName,
    String memberName,
  );
}
