// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: avoid_print

import 'dart:async';
import 'dart:convert';
import 'dart:io';

class ServiceClient {
  ServiceClient(this.client, {this.isolateStartedId, this.isolatePausedId, this.isolateResumeId}) {
    client.listen(_onData, onError: _onError, cancelOnError: true);
  }

  Completer<Object?>? isolateStartedId;
  Completer<Object?>? isolatePausedId;
  Completer<Object?>? isolateResumeId;

  Future<Map<String, Object?>> invokeRPC(String method, [Map<String, Object?>? params]) async {
    final String key = _createKey();
    final String request = json.encode({
      'jsonrpc': '2.0',
      'method': method,
      'params': params ?? <String, Object?>{},
      'id': key,
    });
    client.add(request);
    final completer = Completer<Map<String, Object?>>();
    _outstandingRequests[key] = completer;
    print('-> $key ($method)');
    return completer.future;
  }

  String _createKey() {
    final String key = '$_id';
    _id++;
    return key;
  }

  void _onData(dynamic message) {
    final response = json.decode(message as String) as Map<String, Object?>;
    final dynamic key = response['id'];
    if (key != null) {
      print('<- $key');
      final completer = _outstandingRequests.remove(key)!;
      final result = response['result'];
      final error = response['error'];
      if (error != null) {
        completer.completeError(error);
      } else {
        completer.complete(result);
      }
    } else {
      if (response['method'] == 'streamNotify') {
        _onServiceEvent(response['params'] as Map<String, Object?>?);
      }
    }
  }

  void _onServiceEvent(Map<String, Object?>? params) {
    if (params == null) {
      return;
    }
    final event = params['event'] as Map<String, Object?>?;
    if (event == null || event['type'] != 'Event') {
      return;
    }
    final dynamic isolateId = (event['isolate']! as Map<String, Object?>)['id'];
    switch (params['streamId']) {
      case 'Isolate':
        if (event['kind'] == 'IsolateStart') {
          isolateStartedId?.complete(isolateId);
        }
      case 'Debug':
        switch (event['kind']) {
          case 'Resume':
            isolateResumeId?.complete(isolateId);
          case 'PauseStart':
            isolatePausedId?.complete(isolateId);
        }
    }
  }

  void _onError(dynamic error) {
    print('WebSocket error: $error');
  }

  final WebSocket client;
  final _outstandingRequests = <String, Completer<dynamic>>{};
  int _id = 1;
}
