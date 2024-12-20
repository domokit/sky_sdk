// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:ui';

/// Util method to replicate the behavior of a `MethodChannel` in the Flutter
/// framework.
void sendJsonMethodCall({
  required PlatformDispatcher dispatcher,
  required String channel,
  required String method,
  dynamic arguments,
  PlatformMessageResponseCallback? callback,
}) {
  sendJsonMessage(
    dispatcher: dispatcher,
    channel: channel,
    json: <String, dynamic>{'method': method, 'args': arguments},
  );
}

/// Send a JSON message over a channel.
void sendJsonMessage({
  required PlatformDispatcher dispatcher,
  required String channel,
  required Map<String, dynamic> json,
  PlatformMessageResponseCallback? callback,
}) {
  dispatcher.sendPlatformMessage(
    channel,
    // This recreates a combination of OptionalMethodChannel, JSONMethodCodec,
    // and _DefaultBinaryMessenger in the framework.
    utf8.encode(const JsonCodec().encode(json)).buffer.asByteData(),
    callback,
  );
}
