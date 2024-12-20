// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// KEEP THIS SYNCHRONIZED WITH ../web_ui/lib/channel_buffers.dart
part of dart.ui;

/// Deprecated. Migrate to [ChannelCallback] instead.
///
/// Signature for [ChannelBuffers.drain]'s `callback` argument.
///
/// The first argument is the data sent by the plugin.
///
/// The second argument is a closure that, when called, will send messages
/// back to the plugin.
@Deprecated(
  'Migrate to ChannelCallback instead. '
  'This feature was deprecated after v3.11.0-20.0.pre.',
)
typedef DrainChannelCallback =
    Future<void> Function(ByteData? data, PlatformMessageResponseCallback callback);

/// Signature for [ChannelBuffers.setListener]'s `callback` argument.
///
/// The first argument is the data sent by the plugin.
///
/// The second argument is a closure that, when called, will send messages
/// back to the plugin.
///
/// See also:
///
///  * [PlatformMessageResponseCallback], the type used for replies.
typedef ChannelCallback = void Function(ByteData? data, PlatformMessageResponseCallback callback);

/// The data and logic required to store and invoke a callback.
///
/// This tracks (and applies) the [Zone].
class _ChannelCallbackRecord {
  _ChannelCallbackRecord(this._callback) : _zone = Zone.current;
  final ChannelCallback _callback;
  final Zone _zone;

  /// Call [callback] in [zone], using the given arguments.
  void invoke(ByteData? dataArg, PlatformMessageResponseCallback callbackArg) {
    _invoke2<ByteData?, PlatformMessageResponseCallback>(_callback, _zone, dataArg, callbackArg);
  }
}

/// A saved platform message for a channel with its callback.
class _StoredMessage {
  /// Wraps the data and callback for a platform message into
  /// a [_StoredMessage] instance.
  ///
  /// The first argument is a [ByteData] that represents the
  /// payload of the message and a [PlatformMessageResponseCallback]
  /// that represents the callback that will be called when the message
  /// is handled.
  _StoredMessage(this.data, this._callback) : _zone = Zone.current;

  /// Representation of the message's payload.
  final ByteData? data;

  /// Callback to be used when replying to the message.
  final PlatformMessageResponseCallback _callback;

  final Zone _zone;

  void invoke(ByteData? dataArg) {
    _invoke1(_callback, _zone, dataArg);
  }
}

/// The internal storage for a platform channel.
///
/// This consists of a fixed-size circular queue of [_StoredMessage]s,
/// and the channel's callback, if any has been registered.
class _Channel {
  _Channel([this._capacity = ChannelBuffers.kDefaultBufferSize])
    : _queue = collection.ListQueue<_StoredMessage>(_capacity);

  /// The underlying data for the buffered messages.
  final collection.ListQueue<_StoredMessage> _queue;

  /// The number of messages currently in the [_Channel].
  ///
  /// This is equal to or less than the [capacity].
  int get length => _queue.length;

  /// Whether to dump messages to the console when a message is
  /// discarded due to the channel overflowing.
  ///
  /// Has no effect in release builds.
  bool debugEnableDiscardWarnings = true;

  /// The number of messages that _can_ be stored in the [_Channel].
  ///
  /// When additional messages are stored, earlier ones are discarded,
  /// in a first-in-first-out fashion.
  int get capacity => _capacity;
  int _capacity;

  /// Set the [capacity] of the channel to the given size.
  ///
  /// If the new size is smaller than the [length], the oldest
  /// messages are discarded until the capacity is reached. No
  /// message is shown in case of overflow, regardless of the
  /// value of [debugEnableDiscardWarnings].
  set capacity(int newSize) {
    _capacity = newSize;
    _dropOverflowMessages(newSize);
  }

  /// Whether a microtask is queued to call [_drainStep].
  ///
  /// This is used to queue messages received while draining, rather
  /// than sending them out of order. This generally cannot happen in
  /// production but is possible in test scenarios.
  ///
  /// This is also necessary to avoid situations where multiple drains are
  /// invoked simultaneously. For example, if a listener is set
  /// (queuing a drain), then unset, then set again (which would queue
  /// a drain again), all in one stack frame (not allowing the drain
  /// itself an opportunity to check if a listener is set).
  bool _draining = false;

  /// Adds a message to the channel.
  ///
  /// If the channel overflows, earlier messages are discarded, in a
  /// first-in-first-out fashion. See [capacity]. If
  /// [debugEnableDiscardWarnings] is true, this method returns true
  /// on overflow. It is the responsibility of the caller to show the
  /// warning message.
  bool push(_StoredMessage message) {
    if (!_draining && _channelCallbackRecord != null) {
      assert(_queue.isEmpty);
      _channelCallbackRecord!.invoke(message.data, message.invoke);
      return false;
    }
    if (_capacity <= 0) {
      return debugEnableDiscardWarnings;
    }
    final bool result = _dropOverflowMessages(_capacity - 1);
    _queue.addLast(message);
    return result;
  }

  /// Returns the first message in the channel and removes it.
  ///
  /// Throws when empty.
  _StoredMessage pop() => _queue.removeFirst();

  /// Removes messages until [length] reaches `lengthLimit`.
  ///
  /// The callback of each removed message is invoked with null
  /// as its argument.
  ///
  /// If any messages are removed, and [debugEnableDiscardWarnings] is
  /// true, then returns true. The caller is responsible for showing
  /// the warning message in that case.
  bool _dropOverflowMessages(int lengthLimit) {
    bool result = false;
    while (_queue.length > lengthLimit) {
      final _StoredMessage message = _queue.removeFirst();
      message.invoke(null); // send empty reply to the plugin side
      result = true;
    }
    return result;
  }

  _ChannelCallbackRecord? _channelCallbackRecord;

  /// Sets the listener for this channel.
  ///
  /// When there is a listener, messages are sent immediately.
  ///
  /// If any messages were queued before the listener is added,
  /// they are drained asynchronously after this method returns.
  /// (See [_drain].)
  ///
  /// Only one listener may be set at a time. Setting a
  /// new listener clears the previous one.
  ///
  /// Callbacks are invoked in their own stack frame and
  /// use the zone that was current when the callback was
  /// registered.
  void setListener(ChannelCallback callback) {
    final bool needDrain = _channelCallbackRecord == null;
    _channelCallbackRecord = _ChannelCallbackRecord(callback);
    if (needDrain && !_draining) {
      _drain();
    }
  }

  /// Clears the listener for this channel.
  ///
  /// When there is no listener, messages are queued, up to [capacity],
  /// and then discarded in a first-in-first-out fashion.
  void clearListener() {
    _channelCallbackRecord = null;
  }

  /// Drains all the messages in the channel (invoking the currently
  /// registered listener for each one).
  ///
  /// Each message is handled in its own microtask. No messages can
  /// be queued by plugins while the queue is being drained, but any
  /// microtasks queued by the handler itself will be processed before
  /// the next message is handled.
  ///
  /// The draining stops if the listener is removed.
  ///
  /// See also:
  ///
  ///  * [setListener], which is used to register the callback.
  ///  * [clearListener], which removes it.
  void _drain() {
    assert(!_draining);
    _draining = true;
    scheduleMicrotask(_drainStep);
  }

  /// Drains a single message and then reinvokes itself asynchronously.
  ///
  /// See [_drain] for more details.
  void _drainStep() {
    assert(_draining);
    if (_queue.isNotEmpty && _channelCallbackRecord != null) {
      final _StoredMessage message = pop();
      _channelCallbackRecord!.invoke(message.data, message.invoke);
      scheduleMicrotask(_drainStep);
    } else {
      _draining = false;
    }
  }
}

/// The buffering and dispatch mechanism for messages sent by plugins
/// on the engine side to their corresponding plugin code on the
/// framework side.
///
/// Messages for a channel are stored until a listener is provided for that channel,
/// using [setListener]. Only one listener may be configured per channel.
///
/// Typically these buffers are drained once a callback is set up on
/// the [BinaryMessenger] in the Flutter framework. (See [setListener].)
///
/// ## Channel names
///
/// By convention, channels are normally named with a reverse-DNS prefix, a
/// slash, and then a domain-specific name. For example, `com.example/demo`.
///
/// Channel names cannot contain the U+0000 NULL character, because they
/// are passed through APIs that use null-terminated strings.
///
/// ## Buffer capacity and overflow
///
/// Each channel has a finite buffer capacity and messages will
/// be deleted in a first-in-first-out (FIFO) manner if the capacity is exceeded.
///
/// By default buffers store one message per channel, and when a
/// message overflows, in debug mode, a message is printed to the
/// console. The message looks like the following:
///
/// > A message on the com.example channel was discarded before it could be
/// > handled.
/// > This happens when a plugin sends messages to the framework side before the
/// > framework has had an opportunity to register a listener. See the
/// > ChannelBuffers API documentation for details on how to configure the channel
/// > to expect more messages, or to expect messages to get discarded:
/// >   https://api.flutter.dev/flutter/dart-ui/ChannelBuffers-class.html
///
/// There are tradeoffs associated with any size. The correct size
/// should be chosen for the semantics of the channel. To change the
/// size a plugin can send a message using the control channel,
/// as described below.
///
/// Size 0 is appropriate for channels where messages sent before
/// the engine and framework are ready should be ignored. For
/// example, a plugin that notifies the framework any time a
/// radiation sensor detects an ionization event might set its size
/// to zero since past ionization events are typically not
/// interesting, only instantaneous readings are worth tracking.
///
/// Size 1 is appropriate for level-triggered plugins. For example,
/// a plugin that notifies the framework of the current value of a
/// pressure sensor might leave its size at one (the default), while
/// sending messages continually; once the framework side of the plugin
/// registers with the channel, it will immediately receive the most
/// up to date value and earlier messages will have been discarded.
///
/// Sizes greater than one are appropriate for plugins where every
/// message is important. For example, a plugin that itself
/// registers with another system that has been buffering events,
/// and immediately forwards all the previously-buffered events,
/// would likely wish to avoid having any messages dropped on the
/// floor. In such situations, it is important to select a size that
/// will avoid overflows. It is also important to consider the
/// potential for the framework side to never fully initialize (e.g. if
/// the user starts the application, but terminates it soon
/// afterwards, leaving time for the platform side of a plugin to
/// run but not the framework side).
///
/// ## The control channel
///
/// A plugin can configure its channel's buffers by sending messages to the
/// control channel, `dev.flutter/channel-buffers` (see [kControlChannelName]).
///
/// There are two messages that can be sent to this control channel, to adjust
/// the buffer size and to disable the overflow warnings. See [handleMessage]
/// for details on these messages.
class ChannelBuffers {
  /// Create a buffer pool for platform messages.
  ///
  /// It is generally not necessary to create an instance of this class;
  /// the global [channelBuffers] instance is the one used by the engine.
  ChannelBuffers();

  /// The number of messages that channel buffers will store by default.
  static const int kDefaultBufferSize = 1;

  /// The name of the channel that plugins can use to communicate with the
  /// channel buffers system.
  ///
  /// These messages are handled by [handleMessage].
  static const String kControlChannelName = 'dev.flutter/channel-buffers';

  /// A mapping between a channel name and its associated [_Channel].
  final Map<String, _Channel> _channels = <String, _Channel>{};

  /// Adds a message (`data`) to the named channel buffer (`name`).
  ///
  /// The `callback` argument is a closure that, when called, will send messages
  /// back to the plugin.
  ///
  /// If a message overflows the channel, and the channel has not been
  /// configured to expect overflow, then, in debug mode, a message
  /// will be printed to the console warning about the overflow.
  ///
  /// Channel names cannot contain the U+0000 NULL character, because they
  /// are passed through APIs that use null-terminated strings.
  void push(String name, ByteData? data, PlatformMessageResponseCallback callback) {
    assert(!name.contains('\u0000'), 'Channel names must not contain U+0000 NULL characters.');
    final _Channel channel = _channels.putIfAbsent(name, () => _Channel());
    if (channel.push(_StoredMessage(data, callback))) {
      _printDebug(
        'A message on the $name channel was discarded before it could be handled.\n'
        'This happens when a plugin sends messages to the framework side before the '
        'framework has had an opportunity to register a listener. See the ChannelBuffers '
        'API documentation for details on how to configure the channel to expect more '
        'messages, or to expect messages to get discarded:\n'
        '  https://api.flutter.dev/flutter/dart-ui/ChannelBuffers-class.html\n'
        'The capacity of the $name channel is ${channel._capacity} message${channel._capacity != 1 ? 's' : ''}.',
      );
    }
  }

  /// Sets the listener for the specified channel.
  ///
  /// When there is a listener, messages are sent immediately.
  ///
  /// Each channel may have up to one listener set at a time. Setting
  /// a new listener on a channel with an existing listener clears the
  /// previous one.
  ///
  /// Callbacks are invoked in their own stack frame and
  /// use the zone that was current when the callback was
  /// registered.
  ///
  /// ## Draining
  ///
  /// If any messages were queued before the listener is added,
  /// they are drained asynchronously after this method returns.
  ///
  /// Each message is handled in its own microtask. No messages can
  /// be queued by plugins while the queue is being drained, but any
  /// microtasks queued by the handler itself will be processed before
  /// the next message is handled.
  ///
  /// The draining stops if the listener is removed.
  void setListener(String name, ChannelCallback callback) {
    assert(!name.contains('\u0000'), 'Channel names must not contain U+0000 NULL characters.');
    final _Channel channel = _channels.putIfAbsent(name, () => _Channel());
    channel.setListener(callback);
    sendChannelUpdate(name, listening: true);
  }

  /// Clears the listener for the specified channel.
  ///
  /// When there is no listener, messages on that channel are queued,
  /// up to [kDefaultBufferSize] (or the size configured via the
  /// control channel), and then discarded in a first-in-first-out
  /// fashion.
  void clearListener(String name) {
    final _Channel? channel = _channels[name];
    if (channel != null) {
      channel.clearListener();
      sendChannelUpdate(name, listening: false);
    }
  }

  @Native<Void Function(Handle, Bool)>(symbol: 'PlatformConfigurationNativeApi::SendChannelUpdate')
  external static void _sendChannelUpdate(String name, bool listening);

  // TODO(matanlurey): have original authors document; see https://github.com/flutter/flutter/issues/151917.
  // ignore: public_member_api_docs
  void sendChannelUpdate(String name, {required bool listening}) =>
      _sendChannelUpdate(name, listening);

  /// Deprecated. Migrate to [setListener] instead.
  ///
  /// Remove and process all stored messages for a given channel.
  ///
  /// This should be called once a channel is prepared to handle messages
  /// (i.e. when a message handler is set up in the framework).
  ///
  /// The messages are processed by calling the given `callback`. Each message
  /// is processed in its own microtask.
  @Deprecated(
    'Migrate to setListener instead. '
    'This feature was deprecated after v3.11.0-20.0.pre.',
  )
  Future<void> drain(String name, DrainChannelCallback callback) async {
    final _Channel? channel = _channels[name];
    while (channel != null && !channel._queue.isEmpty) {
      final _StoredMessage message = channel.pop();
      await callback(message.data, message.invoke);
    }
  }

  /// Handle a control message.
  ///
  /// This is intended to be called by the platform messages dispatcher, forwarding
  /// messages from plugins to the [kControlChannelName] channel.
  ///
  /// Messages use the [StandardMethodCodec] format. There are two methods
  /// supported: `resize` and `overflow`. The `resize` method changes the size
  /// of the buffer, and the `overflow` method controls whether overflow is
  /// expected or not.
  ///
  /// ## `resize`
  ///
  /// The `resize` method takes as its argument a list with two values, first
  /// the channel name (a UTF-8 string less than 254 bytes long and not
  /// containing any null bytes), and second the allowed size of the channel
  /// buffer (an integer between 0 and 2147483647).
  ///
  /// Upon receiving the message, the channel's buffer is resized. If necessary,
  /// messages are silently discarded to ensure the buffer is no bigger than
  /// specified.
  ///
  /// For historical reasons, this message can also be sent using a bespoke
  /// format consisting of a UTF-8-encoded string with three parts separated
  /// from each other by U+000D CARRIAGE RETURN (CR) characters, the three parts
  /// being the string `resize`, the string giving the channel name, and then
  /// the string giving the decimal serialization of the new channel buffer
  /// size. For example: `resize\rchannel\r1`
  ///
  /// ## `overflow`
  ///
  /// The `overflow` method takes as its argument a list with two values, first
  /// the channel name (a UTF-8 string less than 254 bytes long and not
  /// containing any null bytes), and second a boolean which is true if overflow
  /// is expected and false if it is not.
  ///
  /// This sets a flag on the channel in debug mode. In release mode the message
  /// is silently ignored. The flag indicates whether overflow is expected on this
  /// channel. When the flag is set, messages are discarded silently. When the
  /// flag is cleared (the default), any overflow on the channel causes a message
  /// to be printed to the console, warning that a message was lost.
  void handleMessage(ByteData data) {
    // We hard-code the deserialization here because the StandardMethodCodec class
    // is part of the framework, not dart:ui.
    final Uint8List bytes = data.buffer.asUint8List(data.offsetInBytes, data.lengthInBytes);
    if (bytes[0] == 0x07) {
      // 7 = value code for string
      final int methodNameLength = bytes[1];
      if (methodNameLength >= 254) {
        // lengths greater than 253 have more elaborate encoding
        throw Exception('Unrecognized message sent to $kControlChannelName (method name too long)');
      }
      int index = 2; // where we are in reading the bytes
      final String methodName = utf8.decode(bytes.sublist(index, index + methodNameLength));
      index += methodNameLength;
      switch (methodName) {
        case 'resize':
          if (bytes[index] != 0x0C) {
            // 12 = value code for list
            throw Exception(
              "Invalid arguments for 'resize' method sent to $kControlChannelName (arguments must be a two-element list, channel name and new capacity)",
            );
          }
          index += 1;
          if (bytes[index] < 0x02) {
            // We ignore extra arguments, in case we need to support them in the future, hence <2 rather than !=2.
            throw Exception(
              "Invalid arguments for 'resize' method sent to $kControlChannelName (arguments must be a two-element list, channel name and new capacity)",
            );
          }
          index += 1;
          if (bytes[index] != 0x07) {
            // 7 = value code for string
            throw Exception(
              "Invalid arguments for 'resize' method sent to $kControlChannelName (first argument must be a string)",
            );
          }
          index += 1;
          final int channelNameLength = bytes[index];
          if (channelNameLength >= 254) {
            // lengths greater than 253 have more elaborate encoding
            throw Exception(
              "Invalid arguments for 'resize' method sent to $kControlChannelName (channel name must be less than 254 characters long)",
            );
          }
          index += 1;
          final String channelName = utf8.decode(bytes.sublist(index, index + channelNameLength));
          if (channelName.contains('\u0000')) {
            throw Exception(
              "Invalid arguments for 'resize' method sent to $kControlChannelName (channel name must not contain any null bytes)",
            );
          }
          index += channelNameLength;
          if (bytes[index] != 0x03) {
            // 3 = value code for uint32
            throw Exception(
              "Invalid arguments for 'resize' method sent to $kControlChannelName (second argument must be an integer in the range 0 to 2147483647)",
            );
          }
          index += 1;
          resize(channelName, data.getUint32(index, Endian.host));
        case 'overflow':
          if (bytes[index] != 0x0C) {
            // 12 = value code for list
            throw Exception(
              "Invalid arguments for 'overflow' method sent to $kControlChannelName (arguments must be a two-element list, channel name and flag state)",
            );
          }
          index += 1;
          if (bytes[index] < 0x02) {
            // We ignore extra arguments, in case we need to support them in the future, hence <2 rather than !=2.
            throw Exception(
              "Invalid arguments for 'overflow' method sent to $kControlChannelName (arguments must be a two-element list, channel name and flag state)",
            );
          }
          index += 1;
          if (bytes[index] != 0x07) {
            // 7 = value code for string
            throw Exception(
              "Invalid arguments for 'overflow' method sent to $kControlChannelName (first argument must be a string)",
            );
          }
          index += 1;
          final int channelNameLength = bytes[index];
          if (channelNameLength >= 254) {
            // lengths greater than 253 have more elaborate encoding
            throw Exception(
              "Invalid arguments for 'overflow' method sent to $kControlChannelName (channel name must be less than 254 characters long)",
            );
          }
          index += 1;
          final String channelName = utf8.decode(bytes.sublist(index, index + channelNameLength));
          index += channelNameLength;
          if (bytes[index] != 0x01 && bytes[index] != 0x02) {
            // 1 = value code for true, 2 = value code for false
            throw Exception(
              "Invalid arguments for 'overflow' method sent to $kControlChannelName (second argument must be a boolean)",
            );
          }
          allowOverflow(channelName, bytes[index] == 0x01);
        default:
          throw Exception("Unrecognized method '$methodName' sent to $kControlChannelName");
      }
    } else {
      final List<String> parts = utf8.decode(bytes).split('\r');
      if (parts.length == 1 + /*arity=*/ 2 && parts[0] == 'resize') {
        resize(parts[1], int.parse(parts[2]));
      } else {
        // If the message couldn't be decoded as UTF-8, a FormatException will
        // have been thrown by utf8.decode() above.
        throw Exception('Unrecognized message $parts sent to $kControlChannelName.');
      }
    }
  }

  /// Changes the capacity of the queue associated with the given channel.
  ///
  /// This could result in the dropping of messages if newSize is less
  /// than the current length of the queue.
  ///
  /// This is expected to be called by platform-specific plugin code (indirectly
  /// via the control channel), not by code on the framework side. See
  /// [handleMessage].
  ///
  /// Calling this from framework code is redundant since by the time framework
  /// code can be running, it can just subscribe to the relevant channel and
  /// there is therefore no need for any buffering.
  void resize(String name, int newSize) {
    _Channel? channel = _channels[name];
    if (channel == null) {
      assert(!name.contains('\u0000'), 'Channel names must not contain U+0000 NULL characters.');
      channel = _Channel(newSize);
      _channels[name] = channel;
    } else {
      channel.capacity = newSize;
    }
  }

  /// Toggles whether the channel should show warning messages when discarding
  /// messages due to overflow.
  ///
  /// This is expected to be called by platform-specific plugin code (indirectly
  /// via the control channel), not by code on the framework side. See
  /// [handleMessage].
  ///
  /// Calling this from framework code is redundant since by the time framework
  /// code can be running, it can just subscribe to the relevant channel and
  /// there is therefore no need for any messages to overflow.
  ///
  /// This method has no effect in release builds.
  void allowOverflow(String name, bool allowed) {
    assert(() {
      _Channel? channel = _channels[name];
      if (channel == null && allowed) {
        assert(!name.contains('\u0000'), 'Channel names must not contain U+0000 NULL characters.');
        channel = _Channel();
        _channels[name] = channel;
      }
      channel?.debugEnableDiscardWarnings = !allowed;
      return true;
    }());
  }
}

/// [ChannelBuffers] that allow the storage of messages between the
/// Engine and the Framework.  Typically messages that can't be delivered
/// are stored here until the Framework is able to process them.
///
/// See also:
///
/// * [BinaryMessenger], where [ChannelBuffers] are typically read.
final ChannelBuffers channelBuffers = ChannelBuffers();
