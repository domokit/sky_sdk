// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_driver/driver_extension.dart';

typedef DriverHandler = Future<String> Function();

/// Wraps a flutter driver [DataHandler] with one that waits until a delegate is set.
///
/// This allows the driver test to call [FlutterDriver.requestData] before the handler was
/// set by the app in which case the requestData call will only complete once the app is ready
/// for it.
class FutureDataHandler {
  final Map<String, Completer<DriverHandler>> _handlers = <String, Completer<DriverHandler>>{};

  /// Registers a lazy handler that will be invoked on the next message from the driver.
  Completer<DriverHandler> registerHandler(String key) {
    _handlers[key] = Completer<DriverHandler>();
    return _handlers[key]!;
  }

  Future<String> handleMessage(String? message) async {
    if (_handlers[message] == null) {
      return 'Unsupported driver message: $message.\n'
          'Supported messages are: ${_handlers.keys}.';
    }
    final DriverHandler handler = await _handlers[message]!.future;
    return handler();
  }
}

FutureDataHandler driverDataHandler = FutureDataHandler();
