// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter_driver/flutter_driver.dart';
library;

import 'dart:ui';

/// The maximum amount of time considered safe to spend for a frame's build
/// phase. Anything past that is in the danger of missing the frame as 60FPS.
///
/// Changing this doesn't re-evaluate existing summary.
Duration kBuildBudget = const Duration(milliseconds: 16);
// TODO(CareF): Automatically calculate the refresh budget (#61958)

/// This class and summarizes a list of [FrameTiming] for the performance
/// metrics.
class FrameTimingSummarizer {
  /// Summarize `data` to frame build time and frame rasterizer time statistics.
  ///
  /// See [TimelineSummary.summaryJson] for detail.
  factory FrameTimingSummarizer(List<FrameTiming> data, {int? newGenGCCount, int? oldGenGCCount}) {
    assert(data.isNotEmpty);
    final List<Duration> frameBuildTime = List<Duration>.unmodifiable(
      data.map<Duration>((FrameTiming datum) => datum.buildDuration),
    );
    final List<Duration> frameBuildTimeSorted = List<Duration>.from(frameBuildTime)..sort();
    final List<Duration> frameRasterizerTime = List<Duration>.unmodifiable(
      data.map<Duration>((FrameTiming datum) => datum.rasterDuration),
    );
    final List<Duration> frameRasterizerTimeSorted = List<Duration>.from(frameRasterizerTime)
      ..sort();
    final List<Duration> vsyncOverhead = List<Duration>.unmodifiable(
      data.map<Duration>((FrameTiming datum) => datum.vsyncOverhead),
    );
    final List<int> layerCacheCounts = List<int>.unmodifiable(
      data.map<int>((FrameTiming datum) => datum.layerCacheCount),
    );
    final List<int> layerCacheCountsSorted = List<int>.from(layerCacheCounts)..sort();
    final List<int> layerCacheBytes = List<int>.unmodifiable(
      data.map<int>((FrameTiming datum) => datum.layerCacheBytes),
    );
    final List<int> layerCacheBytesSorted = List<int>.from(layerCacheBytes)..sort();
    final List<int> pictureCacheCounts = List<int>.unmodifiable(
      data.map<int>((FrameTiming datum) => datum.pictureCacheCount),
    );
    final List<int> pictureCacheCountsSorted = List<int>.from(pictureCacheCounts)..sort();
    final List<int> pictureCacheBytes = List<int>.unmodifiable(
      data.map<int>((FrameTiming datum) => datum.pictureCacheBytes),
    );
    final List<int> pictureCacheBytesSorted = List<int>.from(pictureCacheBytes)..sort();
    final List<Duration> vsyncOverheadSorted = List<Duration>.from(vsyncOverhead)..sort();
    Duration add(Duration a, Duration b) => a + b;
    int addInts(int a, int b) => a + b;
    return FrameTimingSummarizer._(
      frameBuildTime: frameBuildTime,
      frameRasterizerTime: frameRasterizerTime,
      vsyncOverhead: vsyncOverhead,
      // This average calculation is microsecond precision, which is fine
      // because typical values of these times are milliseconds.
      averageFrameBuildTime: frameBuildTime.reduce(add) ~/ data.length,
      p90FrameBuildTime: _findPercentile(frameBuildTimeSorted, 0.90),
      p99FrameBuildTime: _findPercentile(frameBuildTimeSorted, 0.99),
      worstFrameBuildTime: frameBuildTimeSorted.last,
      missedFrameBuildBudget: _countExceed(frameBuildTimeSorted, kBuildBudget),
      averageFrameRasterizerTime: frameRasterizerTime.reduce(add) ~/ data.length,
      p90FrameRasterizerTime: _findPercentile(frameRasterizerTimeSorted, 0.90),
      p99FrameRasterizerTime: _findPercentile(frameRasterizerTimeSorted, 0.99),
      worstFrameRasterizerTime: frameRasterizerTimeSorted.last,
      averageLayerCacheCount: layerCacheCounts.reduce(addInts) / data.length,
      p90LayerCacheCount: _findPercentile(layerCacheCountsSorted, 0.90),
      p99LayerCacheCount: _findPercentile(layerCacheCountsSorted, 0.99),
      worstLayerCacheCount: layerCacheCountsSorted.last,
      averageLayerCacheBytes: layerCacheBytes.reduce(addInts) / data.length,
      p90LayerCacheBytes: _findPercentile(layerCacheBytesSorted, 0.90),
      p99LayerCacheBytes: _findPercentile(layerCacheBytesSorted, 0.99),
      worstLayerCacheBytes: layerCacheBytesSorted.last,
      averagePictureCacheCount: pictureCacheCounts.reduce(addInts) / data.length,
      p90PictureCacheCount: _findPercentile(pictureCacheCountsSorted, 0.90),
      p99PictureCacheCount: _findPercentile(pictureCacheCountsSorted, 0.99),
      worstPictureCacheCount: pictureCacheCountsSorted.last,
      averagePictureCacheBytes: pictureCacheBytes.reduce(addInts) / data.length,
      p90PictureCacheBytes: _findPercentile(pictureCacheBytesSorted, 0.90),
      p99PictureCacheBytes: _findPercentile(pictureCacheBytesSorted, 0.99),
      worstPictureCacheBytes: pictureCacheBytesSorted.last,
      missedFrameRasterizerBudget: _countExceed(frameRasterizerTimeSorted, kBuildBudget),
      averageVsyncOverhead: vsyncOverhead.reduce(add) ~/ data.length,
      p90VsyncOverhead: _findPercentile(vsyncOverheadSorted, 0.90),
      p99VsyncOverhead: _findPercentile(vsyncOverheadSorted, 0.99),
      worstVsyncOverhead: vsyncOverheadSorted.last,
      newGenGCCount: newGenGCCount ?? -1,
      oldGenGCCount: oldGenGCCount ?? -1,
    );
  }

  const FrameTimingSummarizer._({
    required this.frameBuildTime,
    required this.frameRasterizerTime,
    required this.averageFrameBuildTime,
    required this.p90FrameBuildTime,
    required this.p99FrameBuildTime,
    required this.worstFrameBuildTime,
    required this.missedFrameBuildBudget,
    required this.averageFrameRasterizerTime,
    required this.p90FrameRasterizerTime,
    required this.p99FrameRasterizerTime,
    required this.worstFrameRasterizerTime,
    required this.averageLayerCacheCount,
    required this.p90LayerCacheCount,
    required this.p99LayerCacheCount,
    required this.worstLayerCacheCount,
    required this.averageLayerCacheBytes,
    required this.p90LayerCacheBytes,
    required this.p99LayerCacheBytes,
    required this.worstLayerCacheBytes,
    required this.averagePictureCacheCount,
    required this.p90PictureCacheCount,
    required this.p99PictureCacheCount,
    required this.worstPictureCacheCount,
    required this.averagePictureCacheBytes,
    required this.p90PictureCacheBytes,
    required this.p99PictureCacheBytes,
    required this.worstPictureCacheBytes,
    required this.missedFrameRasterizerBudget,
    required this.vsyncOverhead,
    required this.averageVsyncOverhead,
    required this.p90VsyncOverhead,
    required this.p99VsyncOverhead,
    required this.worstVsyncOverhead,
    required this.newGenGCCount,
    required this.oldGenGCCount,
  });

  /// List of frame build time in microseconds
  final List<Duration> frameBuildTime;

  /// List of frame rasterizer time in microseconds
  final List<Duration> frameRasterizerTime;

  /// List of the time difference between vsync signal and frame building start
  /// time
  final List<Duration> vsyncOverhead;

  /// The average value of [frameBuildTime] in milliseconds.
  final Duration averageFrameBuildTime;

  /// The 90-th percentile value of [frameBuildTime] in milliseconds
  final Duration p90FrameBuildTime;

  /// The 99-th percentile value of [frameBuildTime] in milliseconds
  final Duration p99FrameBuildTime;

  /// The largest value of [frameBuildTime] in milliseconds
  final Duration worstFrameBuildTime;

  /// Number of items in [frameBuildTime] that's greater than [kBuildBudget]
  final int missedFrameBuildBudget;

  /// The average value of [frameRasterizerTime] in milliseconds.
  final Duration averageFrameRasterizerTime;

  /// The 90-th percentile value of [frameRasterizerTime] in milliseconds.
  final Duration p90FrameRasterizerTime;

  /// The 99-th percentile value of [frameRasterizerTime] in milliseconds.
  final Duration p99FrameRasterizerTime;

  /// The largest value of [frameRasterizerTime] in milliseconds.
  final Duration worstFrameRasterizerTime;

  /// The average number of layers cached across all frames.
  final double averageLayerCacheCount;

  /// The 90-th percentile number of layers cached across all frames.
  final int p90LayerCacheCount;

  /// The 90-th percentile number of layers cached across all frames.
  final int p99LayerCacheCount;

  /// The most number of layers cached across all frames.
  final int worstLayerCacheCount;

  /// The average number of bytes consumed by cached layers across all frames.
  final double averageLayerCacheBytes;

  /// The 90-th percentile number of bytes consumed by cached layers across all frames.
  final int p90LayerCacheBytes;

  /// The 90-th percentile number of bytes consumed by cached layers across all frames.
  final int p99LayerCacheBytes;

  /// The highest number of bytes consumed by cached layers across all frames.
  final int worstLayerCacheBytes;

  /// The average number of pictures cached across all frames.
  final double averagePictureCacheCount;

  /// The 90-th percentile number of pictures cached across all frames.
  final int p90PictureCacheCount;

  /// The 90-th percentile number of pictures cached across all frames.
  final int p99PictureCacheCount;

  /// The most number of pictures cached across all frames.
  final int worstPictureCacheCount;

  /// The average number of bytes consumed by cached pictures across all frames.
  final double averagePictureCacheBytes;

  /// The 90-th percentile number of bytes consumed by cached pictures across all frames.
  final int p90PictureCacheBytes;

  /// The 90-th percentile number of bytes consumed by cached pictures across all frames.
  final int p99PictureCacheBytes;

  /// The highest number of bytes consumed by cached pictures across all frames.
  final int worstPictureCacheBytes;

  /// Number of items in [frameRasterizerTime] that's greater than [kBuildBudget]
  final int missedFrameRasterizerBudget;

  /// The average value of [vsyncOverhead];
  final Duration averageVsyncOverhead;

  /// The 90-th percentile value of [vsyncOverhead] in milliseconds
  final Duration p90VsyncOverhead;

  /// The 99-th percentile value of [vsyncOverhead] in milliseconds
  final Duration p99VsyncOverhead;

  /// The largest value of [vsyncOverhead] in milliseconds.
  final Duration worstVsyncOverhead;

  /// The number of new generation GCs.
  final int newGenGCCount;

  /// The number of old generation GCs.
  final int oldGenGCCount;

  /// Convert the summary result to a json object.
  ///
  /// See [TimelineSummary.summaryJson] for detail.
  Map<String, dynamic> get summary => <String, dynamic>{
    'average_frame_build_time_millis': averageFrameBuildTime.inMicroseconds / 1E3,
    '90th_percentile_frame_build_time_millis': p90FrameBuildTime.inMicroseconds / 1E3,
    '99th_percentile_frame_build_time_millis': p99FrameBuildTime.inMicroseconds / 1E3,
    'worst_frame_build_time_millis': worstFrameBuildTime.inMicroseconds / 1E3,
    'missed_frame_build_budget_count': missedFrameBuildBudget,
    'average_frame_rasterizer_time_millis': averageFrameRasterizerTime.inMicroseconds / 1E3,
    '90th_percentile_frame_rasterizer_time_millis': p90FrameRasterizerTime.inMicroseconds / 1E3,
    '99th_percentile_frame_rasterizer_time_millis': p99FrameRasterizerTime.inMicroseconds / 1E3,
    'worst_frame_rasterizer_time_millis': worstFrameRasterizerTime.inMicroseconds / 1E3,
    'average_layer_cache_count': averageLayerCacheCount,
    '90th_percentile_layer_cache_count': p90LayerCacheCount,
    '99th_percentile_layer_cache_count': p99LayerCacheCount,
    'worst_layer_cache_count': worstLayerCacheCount,
    'average_layer_cache_memory': averageLayerCacheBytes / 1024.0 / 1024.0,
    '90th_percentile_layer_cache_memory': p90LayerCacheBytes / 1024.0 / 1024.0,
    '99th_percentile_layer_cache_memory': p99LayerCacheBytes / 1024.0 / 1024.0,
    'worst_layer_cache_memory': worstLayerCacheBytes / 1024.0 / 1024.0,
    'average_picture_cache_count': averagePictureCacheCount,
    '90th_percentile_picture_cache_count': p90PictureCacheCount,
    '99th_percentile_picture_cache_count': p99PictureCacheCount,
    'worst_picture_cache_count': worstPictureCacheCount,
    'average_picture_cache_memory': averagePictureCacheBytes / 1024.0 / 1024.0,
    '90th_percentile_picture_cache_memory': p90PictureCacheBytes / 1024.0 / 1024.0,
    '99th_percentile_picture_cache_memory': p99PictureCacheBytes / 1024.0 / 1024.0,
    'worst_picture_cache_memory': worstPictureCacheBytes / 1024.0 / 1024.0,
    'missed_frame_rasterizer_budget_count': missedFrameRasterizerBudget,
    'frame_count': frameBuildTime.length,
    'frame_build_times': frameBuildTime.map<int>((Duration datum) => datum.inMicroseconds).toList(),
    'frame_rasterizer_times':
        frameRasterizerTime.map<int>((Duration datum) => datum.inMicroseconds).toList(),
    'new_gen_gc_count': newGenGCCount,
    'old_gen_gc_count': oldGenGCCount,
  };
}

/// Returns the 100*p-th percentile of [data].
///
/// [data] must be sorted in ascending order.
T _findPercentile<T>(List<T> data, double p) {
  assert(p >= 0 && p <= 1);
  return data[((data.length - 1) * p).round()];
}

/// Returns the number of elements in [data] that exceed [threshold].
///
/// [data] must be sorted in ascending order.
int _countExceed<T extends Comparable<T>>(List<T> data, T threshold) {
  final int exceedsThresholdIndex = data.indexWhere((T datum) => datum.compareTo(threshold) > 0);
  if (exceedsThresholdIndex == -1) {
    return 0;
  }
  return data.length - exceedsThresholdIndex;
}
