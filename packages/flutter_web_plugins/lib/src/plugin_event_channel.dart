// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/services.dart';

import 'plugin_registry.dart';

/// A named channel for sending events to the framework-side using streams.
///
/// This is the platform-side equivalent of [EventChannel]. Whereas
/// [EventChannel] receives a stream of events from platform plugins, this
/// channel sends a stream of events to the handler listening on the
/// framework-side.
///
/// If no [codec] is provided, then [StandardMethodCodec] is used. If no
/// [binaryMessenger] is provided, then [pluginBinaryMessenger], which sends
/// messages to the framework-side, is used.
///
/// Channels created using this class implement two methods for
/// subscribing to the event stream. The methods use the encoding of
/// the specified [codec].
///
/// The first method is `listen`. When called, it begins forwarding
/// messages to the framework side when they are added to the
/// `controller`. This triggers the [StreamController.onListen] callback
/// on the `controller`.
///
/// The other method is `cancel`. When called, it stops forwarding
/// events to the framework. This triggers the [StreamController.onCancel]
/// callback on the `controller`.
///
/// Events added to the `controller` when the framework is not
/// subscribed are silently discarded.
class PluginEventChannel<T> {
  /// Creates a new plugin event channel.
  const PluginEventChannel(
    this.name, [
    this.codec = const StandardMethodCodec(),
    this.binaryMessenger,
  ]);

  /// The logical channel on which communication happens.
  final String name;

  /// The message codec used by this channel.
  ///
  /// Defaults to [StandardMethodCodec].
  final MethodCodec codec;

  /// The messenger used by this channel to send platform messages.
  ///
  /// When this is null, the [pluginBinaryMessenger] is used instead,
  /// which sends messages from the platform-side to the
  /// framework-side.
  final BinaryMessenger? binaryMessenger;

  /// Use [setController] instead.
  ///
  /// This setter is deprecated because it has no corresponding getter,
  /// and providing a getter would require making this class non-const.
  @Deprecated(
    'Replace calls to the "controller" setter with calls to the "setController" method. '
    'This feature was deprecated after v1.23.0-7.0.pre.',
  )
  // ignore: avoid_setters_without_getters
  set controller(StreamController<T> controller) {
    setController(controller);
  }

  /// Changes the stream controller for this event channel.
  ///
  /// Setting the controller to null disconnects from the channel (setting
  /// the message handler on the [binaryMessenger] to null).
  void setController(StreamController<T>? controller) {
    final BinaryMessenger messenger = binaryMessenger ?? pluginBinaryMessenger;
    if (controller == null) {
      messenger.setMessageHandler(name, null);
    } else {
      // The handler object is kept alive via its handle() method
      // keeping a reference to itself. Ideally we would keep a
      // reference to it so that there was a clear ownership model,
      // but that would require making this class non-const. Having
      // this class be const is convenient since it allows references
      // to be obtained by using the constructor rather than having
      // to literally pass references around.
      final _EventChannelHandler<T> handler = _EventChannelHandler<T>(
        name,
        codec,
        controller,
        messenger,
      );
      messenger.setMessageHandler(name, handler.handle);
    }
  }
}

class _EventChannelHandler<T> {
  _EventChannelHandler(this.name, this.codec, this.controller, this.messenger);

  final String name;
  final MethodCodec codec;
  final StreamController<T> controller;
  final BinaryMessenger messenger;

  StreamSubscription<T>? subscription;

  Future<ByteData>? handle(ByteData? message) {
    final MethodCall call = codec.decodeMethodCall(message);
    switch (call.method) {
      case 'listen':
        assert(call.arguments == null);
        return _listen();
      case 'cancel':
        assert(call.arguments == null);
        return _cancel();
    }
    return null;
  }

  Future<ByteData> _listen() async {
    // Cancel any existing subscription.
    await subscription?.cancel();
    subscription = controller.stream.listen(
      (dynamic event) {
        messenger.send(name, codec.encodeSuccessEnvelope(event));
      },
      onError: (dynamic error) {
        messenger.send(name, codec.encodeErrorEnvelope(code: 'error', message: '$error'));
      },
    );
    return codec.encodeSuccessEnvelope(null);
  }

  Future<ByteData> _cancel() async {
    if (subscription == null) {
      return codec.encodeErrorEnvelope(code: 'error', message: 'No active subscription to cancel.');
    }
    await subscription!.cancel();
    subscription = null;
    return codec.encodeSuccessEnvelope(null);
  }
}
