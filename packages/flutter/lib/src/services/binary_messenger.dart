// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
///
/// @docImport 'package:flutter_test/flutter_test.dart';
library;

import 'dart:typed_data';
import 'dart:ui' as ui;

export 'dart:typed_data' show ByteData;
export 'dart:ui' show PlatformMessageResponseCallback;

/// A function which takes a platform message and asynchronously returns an encoded response.
typedef MessageHandler = Future<ByteData?>? Function(ByteData? message);

/// A messenger which sends binary data across the Flutter platform barrier.
///
/// This class also registers handlers for incoming messages.
abstract class BinaryMessenger {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const BinaryMessenger();

  /// Queues a message.
  ///
  /// The returned future completes immediately.
  ///
  /// This method adds the provided message to the given channel (named by the
  /// `channel` argument) of the [ChannelBuffers] object. This simulates what
  /// happens when a plugin on the platform thread (e.g. Kotlin or Swift code)
  /// sends a message to the plugin package on the Dart thread.
  ///
  /// The `data` argument contains the message as encoded bytes. (The format
  /// used for the message depends on the channel.)
  ///
  /// The `callback` argument, if non-null, is eventually invoked with the
  /// response that would have been sent to the platform thread.
  ///
  /// In production code, it is more efficient to call
  /// `ServicesBinding.instance.channelBuffers.push` directly.
  ///
  /// In tests, consider using
  /// `tester.binding.defaultBinaryMessenger.handlePlatformMessage` (see
  /// [WidgetTester], [TestWidgetsFlutterBinding], [TestDefaultBinaryMessenger],
  /// and [TestDefaultBinaryMessenger.handlePlatformMessage] respectively).
  ///
  /// To register a handler for a given message channel, see [setMessageHandler].
  ///
  /// To send a message _to_ a plugin on the platform thread, see [send].
  @Deprecated(
    'Instead of calling this method, use ServicesBinding.instance.channelBuffers.push. '
    'In tests, consider using tester.binding.defaultBinaryMessenger.handlePlatformMessage '
    'or TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage. '
    'This feature was deprecated after v3.9.0-19.0.pre.',
  )
  Future<void> handlePlatformMessage(
    String channel,
    ByteData? data,
    ui.PlatformMessageResponseCallback? callback,
  );

  /// Send a binary message to the platform plugins on the given channel.
  ///
  /// Returns a [Future] which completes to the received response, undecoded,
  /// in binary form.
  Future<ByteData?>? send(String channel, ByteData? message);

  /// Set a callback for receiving messages from the platform plugins on the
  /// given channel, without decoding them.
  ///
  /// The given callback will replace the currently registered callback for that
  /// channel, if any. To remove the handler, pass null as the [handler]
  /// argument.
  ///
  /// The handler's return value, if non-null, is sent as a response, unencoded.
  void setMessageHandler(String channel, MessageHandler? handler);

  // Looking for setMockMessageHandler or checkMockMessageHandler?
  // See this shim package: packages/flutter_test/lib/src/deprecated.dart
}
