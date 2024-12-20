// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'test_default_binary_messenger.dart';
library;

import 'package:flutter/services.dart';

import 'binding.dart';

/// Shim to support the obsolete [setMockMessageHandler] and
/// [checkMockMessageHandler] methods on [BinaryMessenger] in tests.
///
/// The implementations defer to [TestDefaultBinaryMessengerBinding.defaultBinaryMessenger].
///
/// Rather than calling [setMockMessageHandler] on the
/// `ServicesBinding.defaultBinaryMessenger`, use
/// `tester.binding.defaultBinaryMessenger.setMockMessageHandler` directly. This
/// more accurately represents the actual method invocation.
extension TestBinaryMessengerExtension on BinaryMessenger {
  /// Shim for [TestDefaultBinaryMessenger.setMockMessageHandler].
  @Deprecated(
    'Use tester.binding.defaultBinaryMessenger.setMockMessageHandler or '
    'TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler instead. '
    'For the first argument, pass channel.name. '
    'This feature was deprecated after v3.9.0-19.0.pre.',
  )
  void setMockMessageHandler(String channel, MessageHandler? handler) {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
      channel,
      handler,
    );
  }

  /// Shim for [TestDefaultBinaryMessenger.checkMockMessageHandler].
  @Deprecated(
    'Use tester.binding.defaultBinaryMessenger.checkMockMessageHandler or '
    'TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.checkMockMessageHandler instead. '
    'For the first argument, pass channel.name. '
    'This feature was deprecated after v3.9.0-19.0.pre.',
  )
  bool checkMockMessageHandler(String channel, Object? handler) {
    return TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
        .checkMockMessageHandler(channel, handler);
  }
}

/// Shim to support the obsolete [setMockMessageHandler] and
/// [checkMockMessageHandler] methods on [BasicMessageChannel] in tests.
///
/// The implementations defer to [TestDefaultBinaryMessengerBinding.defaultBinaryMessenger].
///
/// Rather than calling [setMockMessageHandler] on the message channel, use
/// `tester.binding.defaultBinaryMessenger.setMockDecodedMessageHandler`
/// directly. This more accurately represents the actual method invocation.
extension TestBasicMessageChannelExtension<T> on BasicMessageChannel<T> {
  /// Shim for [TestDefaultBinaryMessenger.setMockDecodedMessageHandler].
  @Deprecated(
    'Use tester.binding.defaultBinaryMessenger.setMockDecodedMessageHandler or '
    'TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockDecodedMessageHandler instead. '
    'Pass the channel as the first argument. '
    'This feature was deprecated after v3.9.0-19.0.pre.',
  )
  void setMockMessageHandler(Future<T> Function(T? message)? handler) {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
        .setMockDecodedMessageHandler<T>(this, handler);
  }

  /// Shim for [TestDefaultBinaryMessenger.checkMockMessageHandler].
  @Deprecated(
    'Use tester.binding.defaultBinaryMessenger.checkMockMessageHandler or '
    'TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.checkMockMessageHandler instead. '
    'For the first argument, pass channel.name. '
    'This feature was deprecated after v3.9.0-19.0.pre.',
  )
  bool checkMockMessageHandler(Object? handler) {
    return TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
        .checkMockMessageHandler(name, handler);
  }
}

/// Shim to support the obsolete [setMockMethodCallHandler] and
/// [checkMockMethodCallHandler] methods on [MethodChannel] in tests.
///
/// The implementations defer to [TestDefaultBinaryMessengerBinding.defaultBinaryMessenger].
///
/// Rather than calling [setMockMethodCallHandler] on the method channel, use
/// `tester.binding.defaultBinaryMessenger.setMockMethodCallHandler` directly.
/// This more accurately represents the actual method invocation.
extension TestMethodChannelExtension on MethodChannel {
  /// Shim for [TestDefaultBinaryMessenger.setMockMethodCallHandler].
  @Deprecated(
    'Use tester.binding.defaultBinaryMessenger.setMockMethodCallHandler or '
    'TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler instead. '
    'Pass the channel as the first argument. '
    'This feature was deprecated after v3.9.0-19.0.pre.',
  )
  void setMockMethodCallHandler(Future<dynamic>? Function(MethodCall call)? handler) {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      this,
      handler,
    );
  }

  /// Shim for [TestDefaultBinaryMessenger.checkMockMessageHandler].
  @Deprecated(
    'Use tester.binding.defaultBinaryMessenger.checkMockMessageHandler or '
    'TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.checkMockMessageHandler instead. '
    'For the first argument, pass channel.name. '
    'This feature was deprecated after v3.9.0-19.0.pre.',
  )
  bool checkMockMethodCallHandler(Object? handler) {
    return TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
        .checkMockMessageHandler(name, handler);
  }
}
