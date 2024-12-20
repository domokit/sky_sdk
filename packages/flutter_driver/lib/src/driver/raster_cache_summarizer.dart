// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'percentile_utils.dart';
import 'timeline.dart';

/// Key for RasterCache timeline events.
const String kRasterCacheEvent = 'RasterCache';

const String _kLayerCount = 'LayerCount';
const String _kLayerMemory = 'LayerMBytes';
const String _kPictureCount = 'PictureCount';
const String _kPictureMemory = 'PictureMBytes';

/// Summarizes [TimelineEvent]s corresponding to [kRasterCacheEvent] events.
///
/// A sample event (some fields have been omitted for brevity):
/// ```json
///     {
///       "name": "RasterCache",
///       "ts": 75598996256,
///       "ph": "C",
///       "args": {
///         "LayerCount": "1",
///         "LayerMBytes": "0.336491",
///         "PictureCount": "0",
///         "PictureMBytes": "0.000000",
///       }
///     },
/// ```
class RasterCacheSummarizer {
  /// Creates a RasterCacheSummarizer given the timeline events.
  RasterCacheSummarizer(this.rasterCacheEvents) {
    for (final TimelineEvent event in rasterCacheEvents) {
      assert(event.name == kRasterCacheEvent);
    }
  }

  /// The raster cache events.
  final List<TimelineEvent> rasterCacheEvents;

  late final List<double> _layerCounts = _extractValues(_kLayerCount);
  late final List<double> _layerMemories = _extractValues(_kLayerMemory);
  late final List<double> _pictureCounts = _extractValues(_kPictureCount);
  late final List<double> _pictureMemories = _extractValues(_kPictureMemory);

  /// Computes the average of the `LayerCount` values over the cache events.
  double computeAverageLayerCount() => _computeAverage(_layerCounts);

  /// Computes the average of the `LayerMemory` values over the cache events.
  double computeAverageLayerMemory() => _computeAverage(_layerMemories);

  /// Computes the average of the `PictureCount` values over the cache events.
  double computeAveragePictureCount() => _computeAverage(_pictureCounts);

  /// Computes the average of the `PictureMemory` values over the cache events.
  double computeAveragePictureMemory() => _computeAverage(_pictureMemories);

  /// The [percentile]-th percentile `LayerCount` over the cache events.
  double computePercentileLayerCount(double percentile) =>
      _computePercentile(_layerCounts, percentile);

  /// The [percentile]-th percentile `LayerMemory` over the cache events.
  double computePercentileLayerMemory(double percentile) =>
      _computePercentile(_layerMemories, percentile);

  /// The [percentile]-th percentile `PictureCount` over the cache events.
  double computePercentilePictureCount(double percentile) =>
      _computePercentile(_pictureCounts, percentile);

  /// The [percentile]-th percentile `PictureMemory` over the cache events.
  double computePercentilePictureMemory(double percentile) =>
      _computePercentile(_pictureMemories, percentile);

  /// Computes the worst of the `LayerCount` values over the cache events.
  double computeWorstLayerCount() => _computeWorst(_layerCounts);

  /// Computes the worst of the `LayerMemory` values over the cache events.
  double computeWorstLayerMemory() => _computeWorst(_layerMemories);

  /// Computes the worst of the `PictureCount` values over the cache events.
  double computeWorstPictureCount() => _computeWorst(_pictureCounts);

  /// Computes the worst of the `PictureMemory` values over the cache events.
  double computeWorstPictureMemory() => _computeWorst(_pictureMemories);

  static double _computeAverage(List<double> values) {
    if (values.isEmpty) {
      return 0;
    }

    final double total = values.reduce((double a, double b) => a + b);
    return total / values.length;
  }

  static double _computePercentile(List<double> values, double percentile) {
    if (values.isEmpty) {
      return 0;
    }

    return findPercentile(values, percentile);
  }

  static double _computeWorst(List<double> values) {
    if (values.isEmpty) {
      return 0;
    }

    values.sort();
    return values.last;
  }

  List<double> _extractValues(String name) =>
      rasterCacheEvents.map((TimelineEvent e) => _getValue(e, name)).toList();

  double _getValue(TimelineEvent e, String name) {
    assert(e.name == kRasterCacheEvent);
    assert(e.arguments!.containsKey(name));
    final dynamic valueString = e.arguments![name];
    assert(valueString is String);
    return double.parse(valueString as String);
  }
}
