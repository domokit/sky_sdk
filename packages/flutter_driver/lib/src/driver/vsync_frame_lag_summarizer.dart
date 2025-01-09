// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'percentile_utils.dart';
import 'timeline.dart';

const String _kPlatformVsyncEvent = 'VSYNC';
const String _kUIThreadVsyncProcessEvent = 'VsyncProcessCallback';

/// Event names for frame lag related timeline events.
const Set<String> kVsyncTimelineEventNames = <String>{
  _kUIThreadVsyncProcessEvent,
  _kPlatformVsyncEvent,
};

/// Summarizes [TimelineEvent]s corresponding to [kVsyncTimelineEventNames] events.
///
/// `VsyncFrameLag` is the time between when a platform vsync event is received to
/// when the frame starts getting processed by the Flutter Engine. This delay is
/// typically seen due to non-frame workload related dart tasks being scheduled
/// on the UI thread.
class VsyncFrameLagSummarizer {
  /// Creates a VsyncFrameLagSummarizer given the timeline events.
  VsyncFrameLagSummarizer(this.vsyncEvents);

  /// Timeline events with names in [kVsyncTimelineEventNames].
  final List<TimelineEvent> vsyncEvents;

  /// Computes the average `VsyncFrameLag` over the period of the timeline.
  double computeAverageVsyncFrameLag() {
    final List<double> vsyncFrameLags = _computePlatformToFlutterVsyncBeginLags();
    if (vsyncFrameLags.isEmpty) {
      return 0;
    }

    final double total = vsyncFrameLags.reduce((double a, double b) => a + b);
    return total / vsyncFrameLags.length;
  }

  /// Computes the [percentile]-th percentile `VsyncFrameLag` over the
  /// period of the timeline.
  double computePercentileVsyncFrameLag(double percentile) {
    final List<double> vsyncFrameLags = _computePlatformToFlutterVsyncBeginLags();
    if (vsyncFrameLags.isEmpty) {
      return 0;
    }
    return findPercentile(vsyncFrameLags, percentile);
  }

  List<double> _computePlatformToFlutterVsyncBeginLags() {
    int platformIdx = -1;
    final List<double> result = <double>[];
    for (int i = 0; i < vsyncEvents.length; i++) {
      final TimelineEvent event = vsyncEvents[i];
      if (event.phase != 'B') {
        continue;
      }
      if (event.name == _kPlatformVsyncEvent) {
        // There was a vsync that resulted in a frame not being built.
        // This needs to be penalized.
        if (platformIdx != -1) {
          final int prevTS = vsyncEvents[platformIdx].timestampMicros!;
          result.add((event.timestampMicros! - prevTS).toDouble());
        }
        platformIdx = i;
      } else if (platformIdx != -1) {
        final int platformTS = vsyncEvents[platformIdx].timestampMicros!;
        result.add((event.timestampMicros! - platformTS).toDouble());
        platformIdx = -1;
      }
    }
    return result;
  }
}
