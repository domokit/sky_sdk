// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'timeline.dart';

/// GC related timeline events.
///
/// All these events occur only on the UI thread and are non overlapping.
const Set<String> kGCRootEvents = <String>{
  'CollectNewGeneration',
  'CollectOldGeneration',
  'EvacuateNewGeneration',
  'StartConcurrentMark',
};

/// Summarizes [TimelineEvent]s corresponding to [kGCRootEvents] category.
///
/// A sample event (some fields have been omitted for brevity):
/// ```json
/// {
///   "name": "StartConcurrentMarking",
///   "cat": "GC",
///   "ts": 3240710599608,
/// }
/// ```
/// This class provides methods to compute the total time spend in GC on
/// the UI thread.
class GCSummarizer {
  GCSummarizer._(this.totalGCTimeMillis);

  /// Creates a [GCSummarizer] given the timeline events.
  static GCSummarizer fromEvents(List<TimelineEvent> gcEvents) {
    double totalGCTimeMillis = 0;
    TimelineEvent? lastGCBeginEvent;

    for (final TimelineEvent event in gcEvents) {
      if (!kGCRootEvents.contains(event.name)) {
        continue;
      }
      if (event.phase == 'B') {
        lastGCBeginEvent = event;
      } else if (lastGCBeginEvent != null) {
        // These events must not overlap.
        assert(
          event.name == lastGCBeginEvent.name,
          'Expected "${lastGCBeginEvent.name}" got "${event.name}"',
        );
        final double st = lastGCBeginEvent.timestampMicros!.toDouble();
        final double end = event.timestampMicros!.toDouble();
        lastGCBeginEvent = null;
        totalGCTimeMillis += (end - st) / 1000;
      }
    }

    return GCSummarizer._(totalGCTimeMillis);
  }

  /// Total time spent doing GC on the UI thread.
  final double totalGCTimeMillis;
}
