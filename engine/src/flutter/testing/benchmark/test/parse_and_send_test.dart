// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:metrics_center/metrics_center.dart';
import 'package:path/path.dart' as p;
import 'package:test/test.dart';

import '../bin/parse_and_send.dart' as pas;

void main() {
  test('runGit() succeeds', () async {
    // Check that git --version runs successfully
    final ProcessResult result = await pas.runGit(<String>['--version']);
    expect(result.exitCode, equals(0));
  });

  test('getGitLog() succeeds', () async {
    final List<String> gitLog = await pas.getGitLog();

    // Check that gitLog[0] is a hash
    final RegExp sha1re = RegExp(r'[a-f0-9]{40}');
    expect(sha1re.hasMatch(gitLog[0]), true);

    // Check that gitLog[1] is an int
    final int secondsSinceEpoch = int.parse(gitLog[1]);

    // Check that gitLog[1] is a sensible Unix Epoch
    final int millisecondsSinceEpoch = secondsSinceEpoch * 1000;
    final DateTime commitDate = DateTime.fromMillisecondsSinceEpoch(
      millisecondsSinceEpoch,
      isUtc: true,
    );
    expect(commitDate.year > 2000, true);
    expect(commitDate.year < 3000, true);
  });

  test('parse() succeeds', () async {
    // Check that the results of parse() on ../example/txt_benchmarks.json
    //   is as expeted.
    final String exampleJson = p.join('example', 'txt_benchmarks.json');
    final pas.PointsAndDate pad = await pas.parse(exampleJson);
    final List<FlutterEngineMetricPoint> points = pad.points;

    expect(points[0].value, equals(101.0));
    expect(points[2].value, equals(4460.0));
    expect(points[4].value, equals(6548.0));
  });
}
