// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This is identical to ../../../../ui/channel_buffers.dart with the
// following exceptions:
//
//  * All comments except this one are removed.
//  * _invokeX is replaced with engine.invokeX (X=1,2)
//  * _printDebug is replaced with print in an assert.

part of ui;

typedef DrainChannelCallback =
    Future<void> Function(ByteData? data, PlatformMessageResponseCallback callback);

typedef ChannelCallback = void Function(ByteData? data, PlatformMessageResponseCallback callback);

class _ChannelCallbackRecord {
  _ChannelCallbackRecord(this._callback) : _zone = Zone.current;
  final ChannelCallback _callback;
  final Zone _zone;

  void invoke(ByteData? dataArg, PlatformMessageResponseCallback callbackArg) {
    engine.invoke2<ByteData?, PlatformMessageResponseCallback>(
      _callback,
      _zone,
      dataArg,
      callbackArg,
    );
  }
}

class _StoredMessage {
  _StoredMessage(this.data, this._callback) : _zone = Zone.current;

  final ByteData? data;

  final PlatformMessageResponseCallback _callback;

  final Zone _zone;

  void invoke(ByteData? dataArg) {
    engine.invoke1(_callback, _zone, dataArg);
  }
}

class _Channel {
  _Channel([this._capacity = ChannelBuffers.kDefaultBufferSize])
    : _queue = collection.ListQueue<_StoredMessage>(_capacity);

  final collection.ListQueue<_StoredMessage> _queue;

  int get length => _queue.length;

  bool debugEnableDiscardWarnings = true;

  int get capacity => _capacity;
  int _capacity;
  set capacity(int newSize) {
    _capacity = newSize;
    _dropOverflowMessages(newSize);
  }

  bool _draining = false;

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

  _StoredMessage pop() => _queue.removeFirst();

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

  void setListener(ChannelCallback callback) {
    final bool needDrain = _channelCallbackRecord == null;
    _channelCallbackRecord = _ChannelCallbackRecord(callback);
    if (needDrain && !_draining) {
      _drain();
    }
  }

  void clearListener() {
    _channelCallbackRecord = null;
  }

  void _drain() {
    assert(!_draining);
    _draining = true;
    scheduleMicrotask(_drainStep);
  }

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

class ChannelBuffers {
  ChannelBuffers();

  static const int kDefaultBufferSize = 1;

  static const String kControlChannelName = 'dev.flutter/channel-buffers';

  final Map<String, _Channel> _channels = <String, _Channel>{};

  void push(String name, ByteData? data, PlatformMessageResponseCallback callback) {
    final _Channel channel = _channels.putIfAbsent(name, () => _Channel());
    if (channel.push(_StoredMessage(data, callback))) {
      assert(() {
        engine.printWarning(
          'A message on the $name channel was discarded before it could be handled.\n'
          'This happens when a plugin sends messages to the framework side before the '
          'framework has had an opportunity to register a listener. See the ChannelBuffers '
          'API documentation for details on how to configure the channel to expect more '
          'messages, or to expect messages to get discarded:\n'
          '  https://api.flutter.dev/flutter/dart-ui/ChannelBuffers-class.html',
        );
        return true;
      }());
    }
  }

  void setListener(String name, ChannelCallback callback) {
    final _Channel channel = _channels.putIfAbsent(name, () => _Channel());
    channel.setListener(callback);
  }

  void clearListener(String name) {
    final _Channel? channel = _channels[name];
    if (channel != null) {
      channel.clearListener();
    }
  }

  Future<void> drain(String name, DrainChannelCallback callback) async {
    final _Channel? channel = _channels[name];
    while (channel != null && !channel._queue.isEmpty) {
      final _StoredMessage message = channel.pop();
      await callback(message.data, message.invoke);
    }
  }

  void handleMessage(ByteData data) {
    final Uint8List bytes = data.buffer.asUint8List(data.offsetInBytes, data.lengthInBytes);
    if (bytes[0] == 0x07) {
      // 7 = value code for string
      final int methodNameLength = bytes[1];
      if (methodNameLength >= 254) {
        throw Exception('Unrecognized message sent to $kControlChannelName (method name too long)');
      }
      int index = 2; // where we are in reading the bytes
      final String methodName = utf8.decode(bytes.sublist(index, index + methodNameLength));
      index += methodNameLength;
      switch (methodName) {
        case 'resize':
          if (bytes[index] != 0x0C) {
            throw Exception(
              "Invalid arguments for 'resize' method sent to $kControlChannelName (arguments must be a two-element list, channel name and new capacity)",
            );
          }
          index += 1;
          if (bytes[index] < 0x02) {
            throw Exception(
              "Invalid arguments for 'resize' method sent to $kControlChannelName (arguments must be a two-element list, channel name and new capacity)",
            );
          }
          index += 1;
          if (bytes[index] != 0x07) {
            throw Exception(
              "Invalid arguments for 'resize' method sent to $kControlChannelName (first argument must be a string)",
            );
          }
          index += 1;
          final int channelNameLength = bytes[index];
          if (channelNameLength >= 254) {
            throw Exception(
              "Invalid arguments for 'resize' method sent to $kControlChannelName (channel name must be less than 254 characters long)",
            );
          }
          index += 1;
          final String channelName = utf8.decode(bytes.sublist(index, index + channelNameLength));
          index += channelNameLength;
          if (bytes[index] != 0x03) {
            throw Exception(
              "Invalid arguments for 'resize' method sent to $kControlChannelName (second argument must be an integer in the range 0 to 2147483647)",
            );
          }
          index += 1;
          resize(channelName, data.getUint32(index, Endian.host));
        case 'overflow':
          if (bytes[index] != 0x0C) {
            throw Exception(
              "Invalid arguments for 'overflow' method sent to $kControlChannelName (arguments must be a two-element list, channel name and flag state)",
            );
          }
          index += 1;
          if (bytes[index] < 0x02) {
            throw Exception(
              "Invalid arguments for 'overflow' method sent to $kControlChannelName (arguments must be a two-element list, channel name and flag state)",
            );
          }
          index += 1;
          if (bytes[index] != 0x07) {
            throw Exception(
              "Invalid arguments for 'overflow' method sent to $kControlChannelName (first argument must be a string)",
            );
          }
          index += 1;
          final int channelNameLength = bytes[index];
          if (channelNameLength >= 254) {
            throw Exception(
              "Invalid arguments for 'overflow' method sent to $kControlChannelName (channel name must be less than 254 characters long)",
            );
          }
          index += 1;
          final String channelName = utf8.decode(bytes.sublist(index, index + channelNameLength));
          index += channelNameLength;
          if (bytes[index] != 0x01 && bytes[index] != 0x02) {
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
        throw Exception('Unrecognized message $parts sent to $kControlChannelName.');
      }
    }
  }

  void resize(String name, int newSize) {
    _Channel? channel = _channels[name];
    if (channel == null) {
      channel = _Channel(newSize);
      _channels[name] = channel;
    } else {
      channel.capacity = newSize;
    }
  }

  void allowOverflow(String name, bool allowed) {
    assert(() {
      _Channel? channel = _channels[name];
      if (channel == null && allowed) {
        channel = _Channel();
        _channels[name] = channel;
      }
      channel?.debugEnableDiscardWarnings = !allowed;
      return true;
    }());
  }

  void sendChannelUpdate(String name, {required bool listening}) {}
}

final ChannelBuffers channelBuffers = ChannelBuffers();
