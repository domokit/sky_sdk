// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'percentile_utils.dart';
import 'timeline.dart';

/// Key for SceneDisplayLag timeline events.
const String kSceneDisplayLagEvent = 'SceneDisplayLag';

const String _kVsyncTransitionsMissed = 'vsync_transitions_missed';

/// Summarizes [TimelineEvent]s corresponding to [kSceneDisplayLagEvent] events.
///
/// A sample event (some fields have been omitted for brevity):
/// ```json
///     {
///      "name": "SceneDisplayLag",
///      "ts": 408920509340,
///      "ph": "b", (this can be 'b' or 'e' for begin or end)
///      "args": {
///        "frame_target_time": "408920509340458",
///        "current_frame_target_time": "408920542689291",
///        "vsync_transitions_missed": "2"
///      }
///    },
/// ```
///
/// `vsync_transitions_missed` corresponds to the elapsed number of frame budget
/// durations between when the frame was scheduled to be displayed, i.e, the
/// `frame_target_time` and the next vsync pulse timestamp, i.e, the
/// `current_frame_target_time`.
class SceneDisplayLagSummarizer {
  /// Creates a SceneDisplayLagSummarizer given the timeline events.
  SceneDisplayLagSummarizer(this.sceneDisplayLagEvents) {
    for (final TimelineEvent event in sceneDisplayLagEvents) {
      assert(event.name == kSceneDisplayLagEvent);
    }
  }

  /// The scene display lag events.
  final List<TimelineEvent> sceneDisplayLagEvents;

  /// Computes the average of the `vsync_transitions_missed` over the lag events.
  double computeAverageVsyncTransitionsMissed() {
    if (sceneDisplayLagEvents.isEmpty) {
      return 0;
    }

    final double total = sceneDisplayLagEvents
        .map(_getVsyncTransitionsMissed)
        .reduce((double a, double b) => a + b);
    return total / sceneDisplayLagEvents.length;
  }

  /// The [percentile]-th percentile `vsync_transitions_missed` over the lag events.
  double computePercentileVsyncTransitionsMissed(double percentile) {
    if (sceneDisplayLagEvents.isEmpty) {
      return 0;
    }

    final List<double> doubles = sceneDisplayLagEvents.map(_getVsyncTransitionsMissed).toList();
    return findPercentile(doubles, percentile);
  }

  double _getVsyncTransitionsMissed(TimelineEvent e) {
    assert(e.name == kSceneDisplayLagEvent);
    assert(e.arguments!.containsKey(_kVsyncTransitionsMissed));
    final dynamic transitionsMissed = e.arguments![_kVsyncTransitionsMissed];
    assert(transitionsMissed is String);
    return double.parse(transitionsMissed as String);
  }
}
