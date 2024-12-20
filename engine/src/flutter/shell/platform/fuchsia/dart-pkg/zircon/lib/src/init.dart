// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of zircon;

final _kZirconFFILibName = 'libzircon_ffi.so';
final _kLibZirconDartPath = '/pkg/lib/$_kZirconFFILibName';

class _Bindings {
  static ZirconFFIBindings? _bindings;

  @pragma('vm:entry-point')
  static ZirconFFIBindings? get() {
    // For soft-transition until libzircon_ffi.so rolls into GI.
    if (!File(_kLibZirconDartPath).existsSync()) {
      return null;
    }

    if (_bindings == null) {
      final _dylib = DynamicLibrary.open(_kZirconFFILibName);
      _bindings = ZirconFFIBindings(_dylib);
    }

    final initializer = _bindings!.zircon_dart_dl_initialize;
    if (initializer(NativeApi.initializeApiDLData) != 1) {
      throw UnsupportedError('Unable to initialize dart:zircon_ffi.');
    }

    return _bindings;
  }
}

final ZirconFFIBindings? zirconFFIBindings = _Bindings.get();
