// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

/// Tuple-like test class for storing a [stream] and [eventKind].
///
/// Used to store the [stream] and [eventKind] that a dispatched event would be
/// sent on.
@immutable
class DispatchedEventKey {
  const DispatchedEventKey({required this.stream, required this.eventKind});

  final String stream;
  final String eventKind;

  @override
  String toString() {
    return '[DispatchedEventKey]($stream, $eventKind)';
  }

  @override
  bool operator ==(Object other) {
    return other is DispatchedEventKey && stream == other.stream && eventKind == other.eventKind;
  }

  @override
  int get hashCode => Object.hash(stream, eventKind);
}

class TestWidgetInspectorService extends Object with WidgetInspectorService {
  TestWidgetInspectorService() {
    selection.addListener(() => selectionChangedCallback?.call());
  }

  final Map<String, ServiceExtensionCallback> extensions = <String, ServiceExtensionCallback>{};

  final Map<DispatchedEventKey, List<Map<Object, Object?>>> eventsDispatched =
      <DispatchedEventKey, List<Map<Object, Object?>>>{};

  final List<Object?> objectsInspected = <Object?>[];

  @override
  void registerServiceExtension({
    required String name,
    required ServiceExtensionCallback callback,
    required RegisterServiceExtensionCallback registerExtension,
  }) {
    assert(!extensions.containsKey(name));
    extensions[name] = callback;
  }

  @override
  void postEvent(String eventKind, Map<Object, Object?> eventData, {String stream = 'Extension'}) {
    dispatchedEvents(eventKind, stream: stream).add(eventData);
  }

  @override
  void inspect(Object? object) {
    objectsInspected.add(object);
  }

  List<Map<Object, Object?>> dispatchedEvents(String eventKind, {String stream = 'Extension'}) {
    return eventsDispatched.putIfAbsent(
      DispatchedEventKey(stream: stream, eventKind: eventKind),
      () => <Map<Object, Object?>>[],
    );
  }

  List<Object?> inspectedObjects() {
    return objectsInspected;
  }

  Iterable<Map<Object, Object?>> getServiceExtensionStateChangedEvents(String extensionName) {
    return dispatchedEvents(
      'Flutter.ServiceExtensionStateChanged',
    ).where((Map<Object, Object?> event) => event['extension'] == extensionName);
  }

  Future<Object?> testExtension(String name, Map<String, String> arguments) async {
    expect(extensions, contains(name));
    // Encode and decode to JSON to match behavior using a real service
    // extension where only JSON is allowed.
    return (json.decode(json.encode(await extensions[name]!(arguments)))
        as Map<String, dynamic>)['result'];
  }

  Future<String> testBoolExtension(String name, Map<String, String> arguments) async {
    expect(extensions, contains(name));
    // Encode and decode to JSON to match behavior using a real service
    // extension where only JSON is allowed.
    return (json.decode(json.encode(await extensions[name]!(arguments)))
            as Map<String, dynamic>)['enabled']
        as String;
  }

  int rebuildCount = 0;

  @override
  Future<void> forceRebuild() async {
    rebuildCount++;
    final WidgetsBinding binding = WidgetsBinding.instance;

    if (binding.rootElement != null) {
      binding.buildOwner!.reassemble(binding.rootElement!);
    }
  }

  @override
  void resetAllState() {
    super.resetAllState();
    eventsDispatched.clear();
    objectsInspected.clear();
    rebuildCount = 0;
  }
}
