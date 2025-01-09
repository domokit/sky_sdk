// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/src/driver/timeline.dart';

import '../../common.dart';

void main() {
  group('Timeline', () {
    test('parses JSON', () {
      final Timeline timeline = Timeline.fromJson(<String, dynamic>{
        'traceEvents': <Map<String, dynamic>>[
          <String, dynamic>{
            'name': 'test event',
            'cat': 'test category',
            'ph': 'B',
            'pid': 123,
            'tid': 234,
            'dur': 345,
            'tdur': 245,
            'ts': 456,
            'tts': 567,
            'args': <String, dynamic>{'arg1': true},
          },
          // Tests that we don't choke on missing data
          <String, dynamic>{},
        ],
      });

      expect(timeline.events, hasLength(2));

      final TimelineEvent e1 = timeline.events![1];
      expect(e1.name, 'test event');
      expect(e1.category, 'test category');
      expect(e1.phase, 'B');
      expect(e1.processId, 123);
      expect(e1.threadId, 234);
      expect(e1.duration, const Duration(microseconds: 345));
      expect(e1.threadDuration, const Duration(microseconds: 245));
      expect(e1.timestampMicros, 456);
      expect(e1.threadTimestampMicros, 567);
      expect(e1.arguments, <String, dynamic>{'arg1': true});

      final TimelineEvent e2 = timeline.events![0];
      expect(e2.name, isNull);
      expect(e2.category, isNull);
      expect(e2.phase, isNull);
      expect(e2.processId, isNull);
      expect(e2.threadId, isNull);
      expect(e2.duration, isNull);
      expect(e2.threadDuration, isNull);
      expect(e2.timestampMicros, isNull);
      expect(e2.threadTimestampMicros, isNull);
      expect(e2.arguments, isNull);
    });

    test('sorts JSON', () {
      final Timeline timeline = Timeline.fromJson(<String, dynamic>{
        'traceEvents': <Map<String, dynamic>>[
          <String, dynamic>{'name': 'test event 1', 'ts': 457},
          <String, dynamic>{'name': 'test event 2', 'ts': 456},
        ],
      });

      expect(timeline.events, hasLength(2));
      expect(timeline.events![0].timestampMicros, equals(456));
      expect(timeline.events![1].timestampMicros, equals(457));
      expect(timeline.events![0].name, equals('test event 2'));
      expect(timeline.events![1].name, equals('test event 1'));
    });

    test('sorts JSON nulls first', () {
      final Timeline timeline = Timeline.fromJson(<String, dynamic>{
        'traceEvents': <Map<String, dynamic>>[
          <String, dynamic>{'name': 'test event 0', 'ts': null},
          <String, dynamic>{'name': 'test event 1', 'ts': 457},
          <String, dynamic>{'name': 'test event 2', 'ts': 456},
          <String, dynamic>{'name': 'test event 3', 'ts': null},
        ],
      });

      expect(timeline.events, hasLength(4));
      expect(timeline.events![0].timestampMicros, isNull);
      expect(timeline.events![1].timestampMicros, isNull);
      expect(timeline.events![2].timestampMicros, equals(456));
      expect(timeline.events![3].timestampMicros, equals(457));
      expect(timeline.events![0].name, equals('test event 0'));
      expect(timeline.events![1].name, equals('test event 3'));
      expect(timeline.events![2].name, equals('test event 2'));
      expect(timeline.events![3].name, equals('test event 1'));
    });
  });
}
