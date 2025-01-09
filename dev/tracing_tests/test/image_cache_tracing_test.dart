// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

import 'common.dart';

void main() {
  initTimelineTests();
  TestWidgetsFlutterBinding.ensureInitialized();

  test('Image cache tracing', () async {
    final TestImageStreamCompleter completer1 = TestImageStreamCompleter();
    final TestImageStreamCompleter completer2 = TestImageStreamCompleter();
    PaintingBinding.instance.imageCache.putIfAbsent('Test', () => completer1);
    PaintingBinding.instance.imageCache.clear();

    completer2.testSetImage(ImageInfo(image: await createTestImage()));
    PaintingBinding.instance.imageCache.putIfAbsent('Test2', () => completer2);
    PaintingBinding.instance.imageCache.evict('Test2');

    _expectTimelineEvents(await fetchTimelineEvents(), <Map<String, dynamic>>[
      <String, dynamic>{
        'name': 'ImageCache.putIfAbsent',
        'args': <String, dynamic>{'key': 'Test', 'isolateId': isolateId, 'parentId': null},
      },
      <String, dynamic>{
        'name': 'listener',
        'args': <String, dynamic>{'isolateId': isolateId, 'parentId': null},
      },
      <String, dynamic>{
        'name': 'ImageCache.clear',
        'args': <String, dynamic>{
          'pendingImages': 1,
          'keepAliveImages': 0,
          'liveImages': 1,
          'currentSizeInBytes': 0,
          'isolateId': isolateId,
          'parentId': null,
        },
      },
      <String, dynamic>{
        'name': 'ImageCache.putIfAbsent',
        'args': <String, dynamic>{'key': 'Test2', 'isolateId': isolateId, 'parentId': null},
      },
      <String, dynamic>{
        'name': 'ImageCache.evict',
        'args': <String, dynamic>{'sizeInBytes': 4, 'isolateId': isolateId, 'parentId': null},
      },
    ]);
  }, skip: isBrowser); // [intended] uses dart:isolate and io.
}

void _expectTimelineEvents(List<TimelineEvent> events, List<Map<String, dynamic>> expected) {
  for (final TimelineEvent event in events) {
    for (int index = 0; index < expected.length; index += 1) {
      if (expected[index]['name'] == event.json!['name']) {
        final Map<String, dynamic> expectedArgs = expected[index]['args'] as Map<String, dynamic>;
        final Map<String, dynamic> args = event.json!['args'] as Map<String, dynamic>;
        if (_mapsEqual(expectedArgs, args)) {
          expected.removeAt(index);
        }
      }
    }
  }
  if (expected.isNotEmpty) {
    final String encodedEvents = jsonEncode(events);
    fail('Timeline did not contain expected events: $expected\nactual: $encodedEvents');
  }
}

bool _mapsEqual(Map<String, dynamic> expectedArgs, Map<String, dynamic> args) {
  for (final String key in expectedArgs.keys) {
    if (expectedArgs[key] != args[key]) {
      return false;
    }
  }
  return true;
}

class TestImageStreamCompleter extends ImageStreamCompleter {
  void testSetImage(ImageInfo image) {
    setImage(image);
  }
}
