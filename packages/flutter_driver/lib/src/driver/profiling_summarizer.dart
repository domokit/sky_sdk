// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'percentile_utils.dart';
import 'timeline.dart';

/// Profiling related timeline events.
///
/// We do not use a profiling category for these as all the dart timeline events
/// have the same profiling category "embedder".
const Set<String> kProfilingEvents = <String>{_kCpuProfile, _kGpuProfile, _kMemoryProfile};

// These field names need to be in-sync with:
// https://github.com/flutter/engine/blob/main/shell/profiling/sampling_profiler.cc
const String _kCpuProfile = 'CpuUsage';
const String _kGpuProfile = 'GpuUsage';
const String _kMemoryProfile = 'MemoryUsage';

/// Represents the supported profiling event types.
enum ProfileType {
  /// Profiling events corresponding to CPU usage.
  CPU,

  /// Profiling events corresponding to GPU usage.
  GPU,

  /// Profiling events corresponding to memory usage.
  Memory,
}

/// Summarizes [TimelineEvent]s corresponding to [kProfilingEvents] category.
///
/// A sample event (some fields have been omitted for brevity):
/// ```json
///     {
///      "category": "embedder",
///      "name": "CpuUsage",
///      "ts": 121120,
///      "args": {
///        "total_cpu_usage": "20.5",
///        "num_threads": "6"
///      }
///    },
/// ```
/// This class provides methods to compute the average and percentile information
/// for supported profiles, i.e, CPU, Memory and GPU. Not all of these exist for
/// all the platforms.
class ProfilingSummarizer {
  ProfilingSummarizer._(this.eventByType);

  /// Creates a ProfilingSummarizer given the timeline events.
  static ProfilingSummarizer fromEvents(List<TimelineEvent> profilingEvents) {
    final Map<ProfileType, List<TimelineEvent>> eventsByType = <ProfileType, List<TimelineEvent>>{};
    for (final TimelineEvent event in profilingEvents) {
      assert(kProfilingEvents.contains(event.name));
      final ProfileType type = _getProfileType(event.name);
      eventsByType[type] ??= <TimelineEvent>[];
      eventsByType[type]!.add(event);
    }
    return ProfilingSummarizer._(eventsByType);
  }

  /// Key is the type of profiling event, for e.g. CPU, GPU, Memory.
  final Map<ProfileType, List<TimelineEvent>> eventByType;

  /// Returns the average, 90th and 99th percentile summary of CPU, GPU and Memory
  /// usage from the recorded events. If a given profile type isn't available
  /// for any reason, the map will not contain the said profile type.
  Map<String, dynamic> summarize() {
    final Map<String, dynamic> summary = <String, dynamic>{};
    summary.addAll(_summarize(ProfileType.CPU, 'cpu_usage'));
    summary.addAll(_summarize(ProfileType.GPU, 'gpu_usage'));
    summary.addAll(_summarize(ProfileType.Memory, 'memory_usage'));
    return summary;
  }

  Map<String, double> _summarize(ProfileType profileType, String name) {
    final Map<String, double> summary = <String, double>{};
    if (!hasProfilingInfo(profileType)) {
      return summary;
    }
    summary['average_$name'] = computeAverage(profileType);
    summary['90th_percentile_$name'] = computePercentile(profileType, 90);
    summary['99th_percentile_$name'] = computePercentile(profileType, 99);
    return summary;
  }

  /// Returns true if there are events in the timeline corresponding to [profileType].
  bool hasProfilingInfo(ProfileType profileType) {
    if (eventByType.containsKey(profileType)) {
      return eventByType[profileType]!.isNotEmpty;
    } else {
      return false;
    }
  }

  /// Computes the average of the `profileType` over the recorded events.
  double computeAverage(ProfileType profileType) {
    final List<TimelineEvent> events = eventByType[profileType]!;
    assert(events.isNotEmpty);
    final double total = events
        .map((TimelineEvent e) => _getProfileValue(profileType, e))
        .reduce((double a, double b) => a + b);
    return total / events.length;
  }

  /// The [percentile]-th percentile `profileType` over the recorded events.
  double computePercentile(ProfileType profileType, double percentile) {
    final List<TimelineEvent> events = eventByType[profileType]!;
    assert(events.isNotEmpty);
    final List<double> doubles =
        events.map((TimelineEvent e) => _getProfileValue(profileType, e)).toList();
    return findPercentile(doubles, percentile);
  }

  static ProfileType _getProfileType(String? eventName) {
    return switch (eventName) {
      _kCpuProfile => ProfileType.CPU,
      _kGpuProfile => ProfileType.GPU,
      _kMemoryProfile => ProfileType.Memory,
      _ => throw Exception('Invalid profiling event: $eventName.'),
    };
  }

  double _getProfileValue(ProfileType profileType, TimelineEvent e) {
    switch (profileType) {
      case ProfileType.CPU:
        return _getArgValue('total_cpu_usage', e);
      case ProfileType.GPU:
        return _getArgValue('gpu_usage', e);
      case ProfileType.Memory:
        final double dirtyMem = _getArgValue('dirty_memory_usage', e);
        final double ownedSharedMem = _getArgValue('owned_shared_memory_usage', e);
        return dirtyMem + ownedSharedMem;
    }
  }

  double _getArgValue(String argKey, TimelineEvent e) {
    assert(e.arguments!.containsKey(argKey));
    final dynamic argVal = e.arguments![argKey];
    assert(argVal is String);
    return double.parse(argVal as String);
  }
}
