// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Timeline data recorded by the Flutter runtime.
class Timeline {
  /// Creates a timeline given JSON-encoded timeline data.
  ///
  /// [json] is in the `chrome://tracing` format. It can be saved to a file
  /// and loaded in Chrome for visual inspection.
  ///
  /// See https://docs.google.com/document/d/1CvAClvFfyA5R-PhYUmn5OOQtYMH4h6I0nSsKchNAySU/preview
  Timeline.fromJson(this.json) : events = _parseEvents(json);

  /// The original timeline JSON.
  final Map<String, dynamic> json;

  /// List of all timeline events.
  ///
  /// This is parsed from "traceEvents" data within [json] and sorted by
  /// timestamp. Anything without a valid timestamp is put in the beginning.
  ///
  /// This will be null if there are no "traceEvents" in the [json].
  final List<TimelineEvent>? events;
}

/// A single timeline event.
class TimelineEvent {
  /// Creates a timeline event given JSON-encoded event data.
  TimelineEvent(this.json)
    : name = json['name'] as String?,
      category = json['cat'] as String?,
      phase = json['ph'] as String?,
      processId = json['pid'] as int?,
      threadId = json['tid'] as int?,
      duration = json['dur'] != null ? Duration(microseconds: json['dur'] as int) : null,
      threadDuration = json['tdur'] != null ? Duration(microseconds: json['tdur'] as int) : null,
      timestampMicros = json['ts'] as int?,
      threadTimestampMicros = json['tts'] as int?,
      arguments = json['args'] as Map<String, dynamic>?;

  /// The original event JSON.
  final Map<String, dynamic> json;

  /// The name of the event.
  ///
  /// Corresponds to the "name" field in the JSON event.
  final String? name;

  /// Event category. Events with different names may share the same category.
  ///
  /// Corresponds to the "cat" field in the JSON event.
  final String? category;

  /// For a given long lasting event, denotes the phase of the event, such as
  /// "B" for "event began", and "E" for "event ended".
  ///
  /// Corresponds to the "ph" field in the JSON event.
  final String? phase;

  /// ID of process that emitted the event.
  ///
  /// Corresponds to the "pid" field in the JSON event.
  final int? processId;

  /// ID of thread that issues the event.
  ///
  /// Corresponds to the "tid" field in the JSON event.
  final int? threadId;

  /// The duration of the event.
  ///
  /// Note, some events are reported with duration. Others are reported as a
  /// pair of begin/end events.
  ///
  /// Corresponds to the "dur" field in the JSON event.
  final Duration? duration;

  /// The thread duration of the event.
  ///
  /// Note, some events are reported with duration. Others are reported as a
  /// pair of begin/end events.
  ///
  /// Corresponds to the "tdur" field in the JSON event.
  final Duration? threadDuration;

  /// Time passed since tracing was enabled, in microseconds.
  ///
  /// Corresponds to the "ts" field in the JSON event.
  final int? timestampMicros;

  /// Thread clock time, in microseconds.
  ///
  /// Corresponds to the "tts" field in the JSON event.
  final int? threadTimestampMicros;

  /// Arbitrary data attached to the event.
  ///
  /// Corresponds to the "args" field in the JSON event.
  final Map<String, dynamic>? arguments;
}

List<TimelineEvent>? _parseEvents(Map<String, dynamic> json) {
  final List<dynamic>? jsonEvents = json['traceEvents'] as List<dynamic>?;

  if (jsonEvents == null) {
    return null;
  }

  final List<TimelineEvent> timelineEvents =
      jsonEvents
          .cast<Map<String, dynamic>>()
          .map<TimelineEvent>((Map<String, dynamic> eventJson) => TimelineEvent(eventJson))
          .toList();

  timelineEvents.sort((TimelineEvent e1, TimelineEvent e2) {
    return switch ((e1.timestampMicros, e2.timestampMicros)) {
      (null, null) => 0,
      (_, null) => 1,
      (null, _) => -1,
      (final int ts1, final int ts2) => ts1.compareTo(ts2),
    };
  });

  return timelineEvents;
}
