// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of zircon;

// ignore_for_file: native_function_body_in_non_sdk_code
// ignore_for_file: public_member_api_docs

@pragma('vm:entry-point')
base class Handle extends NativeFieldWrapperClass1 {
  // No public constructor - this can only be created from native code.
  @pragma('vm:entry-point')
  Handle._();

  // Create an invalid handle object.
  factory Handle.invalid() {
    return _createInvalid();
  }
  @pragma('vm:external-name', 'Handle_CreateInvalid')
  external static Handle _createInvalid();

  @pragma('vm:external-name', 'Handle_handle')
  external int get handle;

  @pragma('vm:external-name', 'Handle_koid')
  external int get koid;

  @override
  String toString() => 'Handle($handle)';

  @override
  bool operator ==(Object other) {
    return other is Handle && other.handle == handle;
  }

  @override
  int get hashCode => handle.hashCode;

  // Common handle operations.
  @pragma('vm:external-name', 'Handle_is_valid')
  external bool get isValid;
  @pragma('vm:external-name', 'Handle_Close')
  external int close();
  @pragma('vm:external-name', 'Handle_AsyncWait')
  external HandleWaiter asyncWait(int signals, AsyncWaitCallback callback);

  @pragma('vm:external-name', 'Handle_Duplicate')
  external Handle duplicate(int rights);

  @pragma('vm:external-name', 'Handle_Replace')
  external Handle replace(int rights);
}

@pragma('vm:entry-point')
class _OnWaitCompleteClosure {
  // No public constructor - this can only be created from native code.
  @pragma('vm:entry-point')
  _OnWaitCompleteClosure(this._callback, this._arg1, this._arg2);

  Function _callback;
  Object _arg1;
  Object _arg2;

  @pragma('vm:entry-point')
  Function get _closure => () => _callback(_arg1, _arg2);
}
