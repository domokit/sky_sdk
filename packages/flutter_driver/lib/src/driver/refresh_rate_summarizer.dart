// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'timeline.dart';

/// Event name for refresh rate related timeline events.
const String kUIThreadVsyncProcessEvent = 'VsyncProcessCallback';

/// A summary of [TimelineEvent]s corresponding to `kUIThreadVsyncProcessEvent` events.
///
/// `RefreshRate` is the time between the start of a vsync pulse and the target time of that vsync.
class RefreshRateSummary {
  /// Creates a [RefreshRateSummary] given the timeline events.
  RefreshRateSummary({required List<TimelineEvent> vsyncEvents}) {
    final List<double> refreshRates = _computeRefreshRates(vsyncEvents);
    _numberOfTotalFrames = refreshRates.length;
    for (final double refreshRate in refreshRates) {
      if ((refreshRate - 30).abs() < _kErrorMargin) {
        _numberOf30HzFrames++;
        continue;
      }
      if ((refreshRate - 60).abs() < _kErrorMargin) {
        _numberOf60HzFrames++;
        continue;
      }
      if ((refreshRate - 80).abs() < _kErrorMargin) {
        _numberOf80HzFrames++;
        continue;
      }
      if ((refreshRate - 90).abs() < _kErrorMargin) {
        _numberOf90HzFrames++;
        continue;
      }
      if ((refreshRate - 120).abs() < _kErrorMargin) {
        _numberOf120HzFrames++;
        continue;
      }
      _framesWithIllegalRefreshRate.add(refreshRate);
    }
    assert(
      _numberOfTotalFrames ==
          _numberOf30HzFrames +
              _numberOf60HzFrames +
              _numberOf80HzFrames +
              _numberOf90HzFrames +
              _numberOf120HzFrames +
              _framesWithIllegalRefreshRate.length,
    );
  }

  // The error margin to determine the frame refresh rate.
  // For example, when we calculated a frame that has a refresh rate of 65, we consider the frame to be a 60Hz frame.
  // Can be adjusted if necessary.
  static const double _kErrorMargin = 6.0;

  /// The percentage of 30hz frames.
  ///
  /// For example, if this value is 20, it means there are 20 percent of total
  /// frames are 30hz. 0 means no frames are 30hz, 100 means all frames are 30hz.
  double get percentageOf30HzFrames =>
      _numberOfTotalFrames > 0 ? _numberOf30HzFrames / _numberOfTotalFrames * 100 : 0;

  /// The percentage of 60hz frames.
  ///
  /// For example, if this value is 20, it means there are 20 percent of total
  /// frames are 60hz. 0 means no frames are 60hz, 100 means all frames are 60hz.
  double get percentageOf60HzFrames =>
      _numberOfTotalFrames > 0 ? _numberOf60HzFrames / _numberOfTotalFrames * 100 : 0;

  /// The percentage of 80hz frames.
  ///
  /// For example, if this value is 20, it means there are 20 percent of total
  /// frames are 80hz. 0 means no frames are 80hz, 100 means all frames are 80hz.
  double get percentageOf80HzFrames =>
      _numberOfTotalFrames > 0 ? _numberOf80HzFrames / _numberOfTotalFrames * 100 : 0;

  /// The percentage of 90hz frames.
  ///
  /// For example, if this value is 20, it means there are 20 percent of total
  /// frames are 90hz. 0 means no frames are 90hz, 100 means all frames are 90hz.
  double get percentageOf90HzFrames =>
      _numberOfTotalFrames > 0 ? _numberOf90HzFrames / _numberOfTotalFrames * 100 : 0;

  /// The percentage of 120hz frames.
  ///
  /// For example, if this value is 20, it means there are 20 percent of total
  /// frames are 120hz. 0 means no frames are 120hz, 100 means all frames are 120hz.
  double get percentageOf120HzFrames =>
      _numberOfTotalFrames > 0 ? _numberOf120HzFrames / _numberOfTotalFrames * 100 : 0;

  /// A list of all the frames with Illegal refresh rate.
  ///
  /// A refresh rate is consider illegal if it does not belong to anyone of the refresh rate this class is
  /// explicitly tracking.
  List<double> get framesWithIllegalRefreshRate => _framesWithIllegalRefreshRate;

  int _numberOf30HzFrames = 0;
  int _numberOf60HzFrames = 0;
  int _numberOf80HzFrames = 0;
  int _numberOf90HzFrames = 0;
  int _numberOf120HzFrames = 0;
  int _numberOfTotalFrames = 0;

  final List<double> _framesWithIllegalRefreshRate = <double>[];

  static List<double> _computeRefreshRates(List<TimelineEvent> vsyncEvents) {
    final List<double> result = <double>[];
    for (int i = 0; i < vsyncEvents.length; i++) {
      final TimelineEvent event = vsyncEvents[i];
      if (event.phase != 'B') {
        continue;
      }
      assert(event.name == kUIThreadVsyncProcessEvent);
      assert(event.arguments != null);
      final Map<String, dynamic> arguments = event.arguments!;
      const double nanosecondsPerSecond = 1e+9;
      final int startTimeInNanoseconds = int.parse(arguments['StartTime'] as String);
      final int targetTimeInNanoseconds = int.parse(arguments['TargetTime'] as String);
      final int frameDurationInNanoseconds = targetTimeInNanoseconds - startTimeInNanoseconds;
      final double refreshRate = nanosecondsPerSecond / frameDurationInNanoseconds;
      result.add(refreshRate);
    }
    return result;
  }
}
