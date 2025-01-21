// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:args/args.dart';
import 'package:metrics_center/metrics_center.dart';
import 'package:path/path.dart' as p;

Future<ProcessResult> runGit(List<String> args, {String? processWorkingDir}) async {
  return Process.run('git', args, workingDirectory: processWorkingDir, runInShell: true);
}

Future<List<String>> getGitLog() async {
  final String gitRoot = p.absolute('../..');
  final ProcessResult logResult = await runGit(<String>[
    'log',
    '--pretty=format:%H %ct',
    '-n',
    '1',
  ], processWorkingDir: gitRoot);
  if (logResult.exitCode != 0) {
    throw 'Unexpected exit code ${logResult.exitCode}';
  }
  return logResult.stdout.toString().trim().split(' ');
}

class PointsAndDate {
  PointsAndDate(this.points, this.date);

  final List<FlutterEngineMetricPoint> points;
  final String date;
}

Future<PointsAndDate> parse(String jsonFileName) async {
  final List<String> gitLog = await getGitLog();
  final String gitRevision = gitLog[0];
  final String gitCommitDate = gitLog[1];
  final List<MetricPoint> rawPoints = await GoogleBenchmarkParser.parse(jsonFileName);
  final List<FlutterEngineMetricPoint> points = <FlutterEngineMetricPoint>[];
  for (final MetricPoint rawPoint in rawPoints) {
    points.add(
      FlutterEngineMetricPoint(
        rawPoint.tags[kNameKey]!,
        rawPoint.value!,
        gitRevision,
        moreTags: rawPoint.tags,
      ),
    );
  }
  return PointsAndDate(points, gitCommitDate);
}

Future<FlutterDestination> connectFlutterDestination() async {
  const String kTokenPath = 'TOKEN_PATH';
  const String kGcpProject = 'GCP_PROJECT';
  final Map<String, String> env = Platform.environment;
  final bool isTesting = env['IS_TESTING'] == 'true';
  if (env.containsKey(kTokenPath) && env.containsKey(kGcpProject)) {
    return FlutterDestination.makeFromAccessToken(
      File(env[kTokenPath]!).readAsStringSync(),
      env[kGcpProject]!,
      isTesting: isTesting,
    );
  }
  return FlutterDestination.makeFromCredentialsJson(
    jsonDecode(Platform.environment['BENCHMARK_GCP_CREDENTIALS']!) as Map<String, dynamic>,
    isTesting: isTesting,
  );
}

ArgParser _serupOptions() {
  final ArgParser parser = ArgParser();
  parser.addOption('json', mandatory: true, help: 'Path to the benchmarks json file.');
  parser.addFlag('no-upload', help: 'Upload the parsed benchmarks.');
  return parser;
}

Future<void> main(List<String> args) async {
  final ArgParser parser = _serupOptions();
  final ArgResults options = parser.parse(args);

  final String json = options['json'] as String;
  final PointsAndDate pointsAndDate = await parse(json);

  final bool noUpload = options['no-upload'] as bool;
  if (noUpload) {
    return;
  }

  // The data will be sent to the Datastore of the GCP project specified through
  // environment variable BENCHMARK_GCP_CREDENTIALS, or TOKEN_PATH/GCP_PROJECT.
  // The engine LUCI job writes it to the `flutter-infra` project.
  final FlutterDestination destination = await connectFlutterDestination();
  await destination.update(
    pointsAndDate.points,
    DateTime.fromMillisecondsSinceEpoch(int.parse(pointsAndDate.date) * 1000, isUtc: true),
    'flutter_engine_benchmark',
  );
}
