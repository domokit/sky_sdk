// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:meta/meta.dart';

import 'base/common.dart';
import 'base/io.dart';
import 'base/logger.dart';
import 'base/utils.dart';
import 'convert.dart';

/// A single message passed through the [DaemonConnection].
class DaemonMessage {
  DaemonMessage(this.data, [this.binary]);

  /// Content of the JSON message in the message.
  final Map<String, Object?> data;

  /// Stream of the binary content of the message.
  ///
  /// Must be listened to if binary data is present.
  final Stream<List<int>>? binary;
}

/// Data of an event passed through the [DaemonConnection].
class DaemonEventData {
  DaemonEventData(this.eventName, this.data, [this.binary]);

  /// The name of the event.
  final String eventName;

  /// The data of the event.
  final Object? data;

  /// Stream of the binary content of the event.
  ///
  /// Must be listened to if binary data is present.
  final Stream<List<int>>? binary;
}

const String _binaryLengthKey = '_binaryLength';

enum _InputStreamParseState { json, binary }

/// Converts a binary stream to a stream of [DaemonMessage].
///
/// The daemon JSON-RPC protocol is defined as follows: every single line of
/// text that starts with `[{` and ends with `}]` will be parsed as a JSON
/// message. The array should contain only one single object which contains the
/// message data.
///
/// If the JSON object contains the key [_binaryLengthKey] with an integer
/// value (will be referred to as N), the following N bytes after the newline
/// character will contain the binary part of the message.
@visibleForTesting
class DaemonInputStreamConverter {
  DaemonInputStreamConverter(this.inputStream) {
    // Lazily listen to the input stream.
    _controller.onListen = () {
      final StreamSubscription<List<int>> subscription = inputStream.listen(
        (List<int> chunk) {
          _processChunk(chunk);
        },
        onError: (Object error, StackTrace stackTrace) {
          _controller.addError(error, stackTrace);
        },
        onDone: () {
          unawaited(_controller.close());
        },
      );

      _controller.onCancel = subscription.cancel;
      // We should not handle onPause or onResume. When the stream is paused, we
      // still need to read from the input stream.
    };
  }

  final Stream<List<int>> inputStream;

  final StreamController<DaemonMessage> _controller = StreamController<DaemonMessage>();
  Stream<DaemonMessage> get convertedStream => _controller.stream;

  // Internal states
  /// The current parse state, whether we are expecting JSON or binary data.
  _InputStreamParseState state = _InputStreamParseState.json;

  /// The binary stream that is being transferred.
  late StreamController<List<int>> currentBinaryStream;

  /// Remaining length in bytes that have to be sent to the binary stream.
  int remainingBinaryLength = 0;

  /// Buffer to hold the current line of input data.
  final BytesBuilder bytesBuilder = BytesBuilder(copy: false);

  // Processes a single chunk received in the input stream.
  void _processChunk(List<int> chunk) {
    int start = 0;
    while (start < chunk.length) {
      switch (state) {
        case _InputStreamParseState.json:
          start += _processChunkInJsonMode(chunk, start);
        case _InputStreamParseState.binary:
          final int bytesSent = _addBinaryChunk(chunk, start, remainingBinaryLength);
          start += bytesSent;
          remainingBinaryLength -= bytesSent;
          if (remainingBinaryLength <= 0) {
            assert(remainingBinaryLength == 0);
            unawaited(currentBinaryStream.close());
            state = _InputStreamParseState.json;
          }
      }
    }
  }

  /// Processes a chunk in JSON mode, and returns the number of bytes processed.
  int _processChunkInJsonMode(List<int> chunk, int start) {
    const int LF = 10; // The '\n' character

    // Search for newline character.
    final int indexOfNewLine = chunk.indexOf(LF, start);
    if (indexOfNewLine < 0) {
      bytesBuilder.add(chunk.sublist(start));
      return chunk.length - start;
    }

    bytesBuilder.add(chunk.sublist(start, indexOfNewLine + 1));

    // Process chunk here
    final Uint8List combinedChunk = bytesBuilder.takeBytes();
    String jsonString = utf8.decode(combinedChunk).trim();
    if (jsonString.startsWith('[{') && jsonString.endsWith('}]')) {
      jsonString = jsonString.substring(1, jsonString.length - 1);
      final Map<String, Object?>? value = castStringKeyedMap(json.decode(jsonString));
      if (value != null) {
        // Check if we need to consume another binary blob.
        if (value[_binaryLengthKey] != null) {
          remainingBinaryLength = value[_binaryLengthKey]! as int;
          currentBinaryStream = StreamController<List<int>>();
          state = _InputStreamParseState.binary;
          _controller.add(DaemonMessage(value, currentBinaryStream.stream));
        } else {
          _controller.add(DaemonMessage(value));
        }
      }
    }

    return indexOfNewLine + 1 - start;
  }

  int _addBinaryChunk(List<int> chunk, int start, int maximumSizeToRead) {
    if (start == 0 && chunk.length <= remainingBinaryLength) {
      currentBinaryStream.add(chunk);
      return chunk.length;
    } else {
      final int chunkRemainingLength = chunk.length - start;
      final int sizeToRead =
          chunkRemainingLength < remainingBinaryLength
              ? chunkRemainingLength
              : remainingBinaryLength;
      currentBinaryStream.add(chunk.sublist(start, start + sizeToRead));
      return sizeToRead;
    }
  }
}

/// A stream that a [DaemonConnection] uses to communicate with each other.
class DaemonStreams {
  DaemonStreams(
    Stream<List<int>> rawInputStream,
    StreamSink<List<int>> outputSink, {
    required Logger logger,
  }) : _outputSink = outputSink,
       inputStream = DaemonInputStreamConverter(rawInputStream).convertedStream,
       _logger = logger;

  /// Creates a [DaemonStreams] that uses stdin and stdout as the underlying streams.
  DaemonStreams.fromStdio(Stdio stdio, {required Logger logger})
    : this(stdio.stdin, stdio.stdout, logger: logger);

  /// Creates a [DaemonStreams] that uses [Socket] as the underlying streams.
  DaemonStreams.fromSocket(Socket socket, {required Logger logger})
    : this(socket, socket, logger: logger);

  /// Connects to a server and creates a [DaemonStreams] from the connection as the underlying streams.
  factory DaemonStreams.connect(String host, int port, {required Logger logger}) {
    final Future<Socket> socketFuture = Socket.connect(host, port);
    final StreamController<List<int>> inputStreamController = StreamController<List<int>>();
    final StreamController<List<int>> outputStreamController = StreamController<List<int>>();
    socketFuture.then<void>(
      (Socket socket) {
        inputStreamController.addStream(socket);
        socket.addStream(outputStreamController.stream);
      },
      onError: (Object error, StackTrace stackTrace) {
        logger.printError('Socket error: $error');
        logger.printTrace('$stackTrace');
        // Propagate the error to the streams.
        inputStreamController.addError(error, stackTrace);
        unawaited(outputStreamController.close());
      },
    );
    return DaemonStreams(inputStreamController.stream, outputStreamController.sink, logger: logger);
  }

  final StreamSink<List<int>> _outputSink;
  final Logger _logger;

  /// Stream that contains input to the [DaemonConnection].
  final Stream<DaemonMessage> inputStream;

  /// Outputs a message through the connection.
  void send(Map<String, Object?> message, [List<int>? binary]) {
    try {
      if (binary != null) {
        message[_binaryLengthKey] = binary.length;
      }
      _outputSink.add(utf8.encode('[${json.encode(message)}]\n'));
      if (binary != null) {
        _outputSink.add(binary);
      }
    } on StateError catch (error) {
      _logger.printError('Failed to write daemon command response: $error');
      // Failed to send, close the connection
      _outputSink.close();
    } on IOException catch (error) {
      _logger.printError('Failed to write daemon command response: $error');
      // Failed to send, close the connection
      _outputSink.close();
    }
  }

  /// Cleans up any resources used.
  Future<void> dispose() async {
    unawaited(_outputSink.close());
  }
}

/// Connection between a flutter daemon and a client.
class DaemonConnection {
  DaemonConnection({required DaemonStreams daemonStreams, required Logger logger})
    : _logger = logger,
      _daemonStreams = daemonStreams {
    _commandSubscription = daemonStreams.inputStream.listen(
      _handleMessage,
      onError: (Object error, StackTrace stackTrace) {
        // We have to listen for on error otherwise the error on the socket
        // will end up in the Zone error handler.
        // Do nothing here and let the stream close handlers handle shutting
        // down the daemon.
      },
    );
  }

  final DaemonStreams _daemonStreams;

  final Logger _logger;

  late final StreamSubscription<DaemonMessage> _commandSubscription;

  int _outgoingRequestId = 0;
  final Map<String, Completer<Object?>> _outgoingRequestCompleters = <String, Completer<Object?>>{};

  final StreamController<DaemonEventData> _events = StreamController<DaemonEventData>.broadcast();
  final StreamController<DaemonMessage> _incomingCommands = StreamController<DaemonMessage>();

  /// A stream that contains all the incoming requests.
  Stream<DaemonMessage> get incomingCommands => _incomingCommands.stream;

  /// Listens to the event with the event name [eventToListen].
  Stream<DaemonEventData> listenToEvent(String eventToListen) {
    return _events.stream.where((DaemonEventData event) => event.eventName == eventToListen);
  }

  /// Sends a request to the other end of the connection.
  ///
  /// Returns a [Future] that resolves with the content.
  Future<Object?> sendRequest(String method, [Object? params, List<int>? binary]) async {
    final String id = '${++_outgoingRequestId}';
    final Completer<Object?> completer = Completer<Object?>();
    _outgoingRequestCompleters[id] = completer;
    final Map<String, Object?> data = <String, Object?>{
      'id': id,
      'method': method,
      if (params != null) 'params': params,
    };
    _logger.printTrace('-> Sending to daemon, id = $id, method = $method');
    _daemonStreams.send(data, binary);
    return completer.future;
  }

  /// Sends a response to the other end of the connection.
  void sendResponse(Object id, [Object? result]) {
    _daemonStreams.send(<String, Object?>{'id': id, if (result != null) 'result': result});
  }

  /// Sends an error response to the other end of the connection.
  void sendErrorResponse(Object id, Object? error, StackTrace trace) {
    _daemonStreams.send(<String, Object?>{'id': id, 'error': error, 'trace': '$trace'});
  }

  /// Sends an event to the client.
  void sendEvent(String name, [Object? params, List<int>? binary]) {
    _daemonStreams.send(<String, Object?>{
      'event': name,
      if (params != null) 'params': params,
    }, binary);
  }

  /// Handles the input from the stream.
  ///
  /// There are three kinds of data: Request, Response, Event.
  ///
  /// Request:
  /// {"id": <Object>. "method": <String>, "params": <optional, Object?>}
  ///
  /// Response:
  /// {"id": <Object>. "result": <optional, Object?>} for a successful response.
  /// {"id": <Object>. "error": <Object>, "stackTrace": <String>} for an error response.
  ///
  /// Event:
  /// {"event": <String>. "params": <optional, Object?>}
  void _handleMessage(DaemonMessage message) {
    final Map<String, Object?> data = message.data;
    if (data['id'] != null) {
      if (data['method'] == null) {
        // This is a response to previously sent request.
        final String id = data['id']! as String;
        if (data['error'] != null) {
          // This is an error response.
          _logger.printTrace('<- Error response received from daemon, id = $id');
          final Object error = data['error']!;
          final String stackTrace = data['trace'] as String? ?? '';
          _outgoingRequestCompleters
              .remove(id)
              ?.completeError(error, StackTrace.fromString(stackTrace));
        } else {
          _logger.printTrace('<- Response received from daemon, id = $id');
          final Object? result = data['result'];
          _outgoingRequestCompleters.remove(id)?.complete(result);
        }
      } else {
        _incomingCommands.add(message);
      }
    } else if (data case {'event': final Object eventName}) {
      _logger.printTrace('<- Event received: $eventName');
      if (eventName is String) {
        _events.add(DaemonEventData(eventName, data['params'], message.binary));
      } else {
        throwToolExit('event name received is not string!');
      }
    } else {
      _logger.printError('Unknown data received from daemon');
    }
  }

  /// Cleans up any resources used in the connection.
  Future<void> dispose() async {
    await _commandSubscription.cancel();
    await _daemonStreams.dispose();
    unawaited(_events.close());
    unawaited(_incomingCommands.close());
  }
}
