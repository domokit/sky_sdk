// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi';
import 'dart:js_interop';

import 'package:ui/src/engine.dart';

class SkwasmObjectWrapper<T extends NativeType> {
  SkwasmObjectWrapper(this.handle, this.registry) {
    registry.register(this);
  }
  final SkwasmFinalizationRegistry<T> registry;
  final Pointer<T> handle;
  bool _isDisposed = false;

  void dispose() {
    assert(!_isDisposed);
    registry.evict(this);
    _isDisposed = true;
  }

  bool get debugDisposed => _isDisposed;
}

typedef DisposeFunction<T extends NativeType> = void Function(Pointer<T>);

class SkwasmFinalizationRegistry<T extends NativeType> {
  SkwasmFinalizationRegistry(this.dispose)
    : registry = DomFinalizationRegistry(
        ((ExternalDartReference<int> address) =>
                dispose(Pointer<T>.fromAddress(address.toDartObject)))
            .toJS,
      );

  final DomFinalizationRegistry registry;
  final DisposeFunction<T> dispose;

  void register(SkwasmObjectWrapper<T> wrapper) {
    final ExternalDartReference jsWrapper = wrapper.toExternalReference;
    registry.registerWithToken(jsWrapper, wrapper.handle.address.toExternalReference, jsWrapper);
  }

  void evict(SkwasmObjectWrapper<T> wrapper) {
    final ExternalDartReference jsWrapper = wrapper.toExternalReference;
    registry.unregister(jsWrapper);
    dispose(wrapper.handle);
  }
}
