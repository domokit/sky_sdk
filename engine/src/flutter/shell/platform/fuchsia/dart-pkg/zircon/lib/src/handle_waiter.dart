// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of zircon;

// ignore_for_file: native_function_body_in_non_sdk_code
// ignore_for_file: public_member_api_docs

typedef AsyncWaitCallback = void Function(int status, int pending);

@pragma('vm:entry-point')
base class HandleWaiter extends NativeFieldWrapperClass1 {
  // Private constructor.
  @pragma('vm:entry-point')
  HandleWaiter._();

  @pragma('vm:external-name', 'HandleWaiter_Cancel')
  external void cancel();
}
