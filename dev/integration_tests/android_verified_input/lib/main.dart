// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_driver/driver_extension.dart';

class FutureDataHandler {
  final Completer<DataHandler> handlerCompleter = Completer<DataHandler>();

  Future<String> handleMessage(String? message) async {
    final DataHandler handler = await handlerCompleter.future;
    return handler(message);
  }
}

FutureDataHandler driverDataHandler = FutureDataHandler();

MethodChannel channel = const MethodChannel('verified_input_test');

Future<dynamic> onMethodChannelCall(MethodCall call) {
  switch (call.method) {
    // Android side is notifying us of the result of verifying the input
    // event.
    case 'notify_verified_input':
      final bool result = call.arguments as bool;
      // FlutterDriver handler, note that this captures the notification
      // value delivered via the method channel.
      Future<String> handler(String? message) async {
        switch (message) {
          case 'input_was_verified':
            return '$result';
        }
        return 'unknown message: "$message"';
      }
      // Install the handler now.
      driverDataHandler.handlerCompleter.complete(handler);
  }
  return Future<dynamic>.value();
}

void main() {
  enableFlutterDriverExtension(handler: driverDataHandler.handleMessage);
  channel.setMethodCallHandler(onMethodChannelCall);
  runApp(MaterialApp(home: _Home()));
}

class _Home extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Verified Input Integration Test'),
        centerTitle: true,
        backgroundColor: Colors.black45,
      ),
      body: Container(
        padding: const EdgeInsets.all(30.0),
        color: Colors.black26,
        child: const AndroidView(key: Key('PlatformView'), viewType: 'verified-input-view'),
      ),
    );
  }
}
