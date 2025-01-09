// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

library fuchsia;

import 'dart:io';
import 'dart:isolate';
import 'dart:zircon';

// ignore_for_file: native_function_body_in_non_sdk_code
// ignore_for_file: public_member_api_docs

// TODO: refactors this incomingServices instead
@pragma('vm:entry-point')
Handle? _environment;

@pragma('vm:entry-point')
Handle? _outgoingServices;

@pragma('vm:entry-point')
Handle? _viewRef;

class MxStartupInfo {
  // TODO: refactor Handle to a Channel
  // https://github.com/flutter/flutter/issues/49439
  static Handle takeEnvironment() {
    if (_environment == null && Platform.isFuchsia) {
      throw Exception('Attempting to call takeEnvironment more than once per process');
    }
    final handle = _environment;
    _environment = null;
    return handle!;
  }

  // TODO: refactor Handle to a Channel
  // https://github.com/flutter/flutter/issues/49439
  static Handle takeOutgoingServices() {
    if (_outgoingServices == null && Platform.isFuchsia) {
      throw Exception('Attempting to call takeOutgoingServices more than once per process');
    }
    final handle = _outgoingServices;
    _outgoingServices = null;
    return handle!;
  }

  // TODO: refactor Handle to a ViewRef
  // https://github.com/flutter/flutter/issues/49439
  static Handle takeViewRef() {
    if (_viewRef == null && Platform.isFuchsia) {
      throw Exception('Attempting to call takeViewRef more than once per process');
    }
    final handle = _viewRef;
    _viewRef = null;
    return handle!;
  }
}

@pragma('vm:external-name', 'SetReturnCode')
external void _setReturnCode(int returnCode);

void exit(int returnCode) {
  _setReturnCode(returnCode);
  Isolate.current.kill(priority: Isolate.immediate);
}

// ignore: always_declare_return_types, prefer_generic_function_type_aliases
typedef _ListStringArgFunction(List<String> args);

// This function is used as the entry point for code in the dart runner and is
// not meant to be called directly outside of that context. The code will invoke
// the given main entry point and pass the args if the function takes args. This
// function is needed because without it the snapshot compiler will tree shake
// the function away unless the user marks it as being an entry point.
//
// The code does not catch any exceptions since this is handled in the dart
// runner calling code.
@pragma('vm:entry-point')
void _runUserMainForDartRunner(Function userMainFunction, List<String> args) {
  if (userMainFunction is _ListStringArgFunction) {
    (userMainFunction as dynamic)(args);
  } else {
    userMainFunction();
  }
}
