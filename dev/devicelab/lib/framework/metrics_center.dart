// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:collection/collection.dart';
import 'package:metrics_center/metrics_center.dart';

/// Authenticate and connect to gcloud storage.
///
/// It supports both token and credential authentications.
Future<FlutterDestination> connectFlutterDestination() async {
  final Map<String, String> env = Platform.environment;
  final bool isTesting = env['IS_TESTING'] == 'true';
  if (env case {'TOKEN_PATH': final String path, 'GCP_PROJECT': final String project}) {
    return FlutterDestination.makeFromAccessToken(
      File(path).readAsStringSync(),
      project,
      isTesting: isTesting,
    );
  }
  return FlutterDestination.makeFromCredentialsJson(
    jsonDecode(env['BENCHMARK_GCP_CREDENTIALS']!) as Map<String, dynamic>,
    isTesting: isTesting,
  );
}

/// Parse results and append additional benchmark tags into Metric Points.
///
/// An example of `resultsJson`:
///   {
///     "CommitBranch": "master",
///     "CommitSha": "abc",
///     "BuilderName": "test",
///     "ResultData": {
///       "average_frame_build_time_millis": 0.4550425531914895,
///       "90th_percentile_frame_build_time_millis": 0.473
///     },
///     "BenchmarkScoreKeys": [
///       "average_frame_build_time_millis",
///       "90th_percentile_frame_build_time_millis"
///     ]
///   }
///
/// An example of `benchmarkTags`:
///   {
///     "arch": "intel",
///     "device_type": "Moto G Play",
///     "device_version": "android-25",
///     "host_type": "linux",
///     "host_version": "debian-10.11"
///   }
List<MetricPoint> parse(
  Map<String, dynamic> resultsJson,
  Map<String, dynamic> benchmarkTags,
  String taskName,
) {
  print('Results to upload to skia perf: $resultsJson');
  print('Benchmark tags to upload to skia perf: $benchmarkTags');
  final List<String> scoreKeys =
      (resultsJson['BenchmarkScoreKeys'] as List<dynamic>?)?.cast<String>() ?? const <String>[];
  final Map<String, dynamic> resultData =
      resultsJson['ResultData'] as Map<String, dynamic>? ?? const <String, dynamic>{};
  final String gitBranch = (resultsJson['CommitBranch'] as String).trim();
  final String gitSha = (resultsJson['CommitSha'] as String).trim();
  final List<MetricPoint> metricPoints = <MetricPoint>[];
  for (final String scoreKey in scoreKeys) {
    Map<String, String> tags = <String, String>{
      kGithubRepoKey: kFlutterFrameworkRepo,
      kGitRevisionKey: gitSha,
      'branch': gitBranch,
      kNameKey: taskName,
      kSubResultKey: scoreKey,
    };
    // Append additional benchmark tags, which will surface in Skia Perf dashboards.
    tags = mergeMaps<String, String>(
      tags,
      benchmarkTags.map(
        (String key, dynamic value) => MapEntry<String, String>(key, value.toString()),
      ),
    );
    metricPoints.add(MetricPoint((resultData[scoreKey] as num).toDouble(), tags));
  }
  return metricPoints;
}

/// Upload metrics to GCS bucket used by Skia Perf.
///
/// Skia Perf picks up all available files under the folder, and
/// is robust to duplicate entries.
///
/// Files will be named based on `taskName`, such as
/// `complex_layout_scroll_perf__timeline_summary_values.json`.
/// If no `taskName` is specified, data will be saved to
/// `default_values.json`.
Future<void> upload(
  FlutterDestination metricsDestination,
  List<MetricPoint> metricPoints,
  int commitTimeSinceEpoch,
  String taskName,
) async {
  await metricsDestination.update(
    metricPoints,
    DateTime.fromMillisecondsSinceEpoch(commitTimeSinceEpoch, isUtc: true),
    taskName,
  );
}

/// Upload JSON results to skia perf.
///
/// Flutter infrastructure's workflow is:
/// 1. Run DeviceLab test, writing results to a known path
/// 2. Request service account token from luci auth (valid for at least 3 minutes)
/// 3. Upload results from (1) to skia perf.
Future<void> uploadToSkiaPerf(
  String? resultsPath,
  String? commitTime,
  String? taskName,
  String? benchmarkTags,
) async {
  int commitTimeSinceEpoch;
  if (resultsPath == null) {
    return;
  }
  if (commitTime != null) {
    commitTimeSinceEpoch = 1000 * int.parse(commitTime);
  } else {
    commitTimeSinceEpoch = DateTime.now().millisecondsSinceEpoch;
  }
  taskName = taskName ?? 'default';
  final Map<String, dynamic> benchmarkTagsMap =
      jsonDecode(benchmarkTags ?? '{}') as Map<String, dynamic>;
  final File resultFile = File(resultsPath);
  Map<String, dynamic> resultsJson = <String, dynamic>{};
  resultsJson = json.decode(await resultFile.readAsString()) as Map<String, dynamic>;
  final List<MetricPoint> metricPoints = parse(resultsJson, benchmarkTagsMap, taskName);
  final FlutterDestination metricsDestination = await connectFlutterDestination();
  await upload(
    metricsDestination,
    metricPoints,
    commitTimeSinceEpoch,
    metricFileName(taskName, benchmarkTagsMap),
  );
}

/// Create metric file name based on `taskName`, `arch`, `host type`, and `device type`.
///
/// Same `taskName` may run on different platforms. Considering host/device tags to
/// use different metric file names.
///
/// This affects only the metric file name which contains metric data, and does not affect
/// real host/device tags.
///
/// For example:
///   Old file name: `backdrop_filter_perf__timeline_summary`
///   New file name: `backdrop_filter_perf__timeline_summary_intel_linux_motoG4`
String metricFileName(String taskName, Map<String, dynamic> benchmarkTagsMap) {
  final StringBuffer fileName = StringBuffer(taskName);
  if (benchmarkTagsMap.containsKey('arch')) {
    fileName
      ..write('_')
      ..write(_fileNameFormat(benchmarkTagsMap['arch'] as String));
  }
  if (benchmarkTagsMap.containsKey('host_type')) {
    fileName
      ..write('_')
      ..write(_fileNameFormat(benchmarkTagsMap['host_type'] as String));
  }
  if (benchmarkTagsMap.containsKey('device_type')) {
    fileName
      ..write('_')
      ..write(_fileNameFormat(benchmarkTagsMap['device_type'] as String));
  }
  return fileName.toString();
}

/// Format `fileName` removing non letter and number characters.
String _fileNameFormat(String fileName) {
  return fileName.replaceAll(RegExp('[^a-zA-Z0-9]'), '');
}
