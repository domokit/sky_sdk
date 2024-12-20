// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';
import 'dart:math';

import 'package:args/args.dart';
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';

import 'benchmark_binding.dart';
import 'foundation/all_elements_bench.dart' as all_elements_bench;
import 'foundation/change_notifier_bench.dart' as change_notifier_bench;
import 'foundation/clamp.dart' as clamp;
import 'foundation/decode_and_parse_asset_manifest.dart' as decode_and_parse_asset_manifest;
import 'foundation/observer_list_bench.dart' as observer_list_bench;
import 'foundation/platform_asset_bundle.dart' as platform_asset_bundle;
import 'foundation/standard_message_codec_bench.dart' as standard_message_codec_bench;
import 'foundation/standard_method_codec_bench.dart' as standard_method_codec_bench;
import 'foundation/timeline_bench.dart' as timeline_bench;
import 'geometry/matrix_utils_transform_bench.dart' as matrix_utils_transform_bench;
import 'geometry/rrect_contains_bench.dart' as rrect_contains_bench;
import 'gestures/gesture_detector_bench.dart' as gesture_detector_bench;
import 'gestures/velocity_tracker_bench.dart' as velocity_tracker_bench;
import 'language/compute_bench.dart' as compute_bench;
import 'language/sync_star_bench.dart' as sync_star_bench;
import 'language/sync_star_semantics_bench.dart' as sync_star_semantics_bench;
import 'layout/text_intrinsic_bench.dart' as text_intrinsic_bench;
import 'stocks/animation_bench.dart' as animation_bench;
import 'stocks/build_bench.dart' as build_bench;
import 'stocks/build_bench_profiled.dart' as build_bench_profiled;
import 'stocks/layout_bench.dart' as layout_bench;
import 'ui/image_bench.dart' as image_bench;

typedef Benchmark = (String name, Future<void> Function() value);

Future<void> main() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");

  // BenchmarkingBinding is used by animation_bench, providing a simple
  // stopwatch interface over rendering. Lifting it here makes all
  // benchmarks run together.
  final BenchmarkingBinding binding = BenchmarkingBinding();
  final List<Benchmark> benchmarks = <Benchmark>[
    ('foundation/change_notifier_bench.dart', change_notifier_bench.execute),
    ('foundation/clamp.dart', clamp.execute),
    ('foundation/platform_asset_bundle.dart', platform_asset_bundle.execute),
    ('foundation/standard_message_codec_bench.dart', standard_message_codec_bench.execute),
    ('foundation/standard_method_codec_bench.dart', standard_method_codec_bench.execute),
    ('foundation/timeline_bench.dart', timeline_bench.execute),
    ('foundation/decode_and_parse_asset_manifest.dart', decode_and_parse_asset_manifest.execute),
    ('foundation/observer_list_bench.dart', observer_list_bench.execute),
    ('geometry/matrix_utils_transform_bench.dart', matrix_utils_transform_bench.execute),
    ('geometry/rrect_contains_bench.dart', rrect_contains_bench.execute),
    ('gestures/gesture_detector_bench.dart', gesture_detector_bench.execute),
    ('gestures/velocity_tracker_bench.dart', velocity_tracker_bench.execute),
    ('language/compute_bench.dart', compute_bench.execute),
    ('language/sync_star_bench.dart', sync_star_bench.execute),
    ('language/sync_star_semantics_bench.dart', sync_star_semantics_bench.execute),
    ('stocks/animation_bench.dart', () => animation_bench.execute(binding)),
    ('stocks/build_bench.dart', build_bench.execute),
    ('stocks/build_bench_profiled.dart', build_bench_profiled.execute),
    ('stocks/layout_bench.dart', layout_bench.execute),
    ('ui/image_bench.dart', image_bench.execute),
    ('layout/text_intrinsic_bench.dart', text_intrinsic_bench.execute),
    (
      'foundation/all_elements_bench.dart',
      () async {
        binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.benchmarkLive;
        runApp(const SizedBox.shrink()); // ensure dispose
        await SchedulerBinding.instance.endOfFrame;
        await all_elements_bench.execute();
      },
    ),
  ];

  // Parses the optional compile-time dart variables; we can't have
  // arguments passed in to main.
  final ArgParser parser = ArgParser();
  final List<String> allowed = benchmarks.map((Benchmark e) => e.$1).toList();
  parser.addMultiOption(
    'tests',
    abbr: 't',
    defaultsTo: allowed,
    allowed: allowed,
    help: 'selected tests to run',
  );
  parser.addOption('seed', defaultsTo: '12345', help: 'selects seed to sort tests by');
  final List<String> mainArgs = <String>[];
  const String testArgs = String.fromEnvironment('tests');
  if (testArgs.isNotEmpty) {
    mainArgs.addAll(<String>['--tests', testArgs]);
    print('╡ ••• environment test override: $testArgs ••• ╞');
  }
  const String seedArgs = String.fromEnvironment('seed');
  if (seedArgs.isNotEmpty) {
    mainArgs.addAll(<String>['--seed', seedArgs]);
    print('╡ ••• environment seed override: $seedArgs ••• ╞');
  }
  final ArgResults results = parser.parse(mainArgs);
  final List<String> selectedTests = results.multiOption('tests');

  // Shuffle the tests because we don't want order dependent tests.
  // It is the responsibility of the infra to tell us what the seed value is,
  // in case we want to have the seed stable for some time period.
  final List<Benchmark> tests =
      benchmarks.where((Benchmark e) => selectedTests.contains(e.$1)).toList();
  tests.shuffle(Random(int.parse(results.option('seed')!)));

  print('╡ ••• Running microbenchmarks ••• ╞');
  for (final Benchmark mark in tests) {
    // Reset the frame policy to default - each test can set it on their own.
    binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.fadePointers;
    print('╡ ••• Running ${mark.$1} ••• ╞');
    await mark.$2();
  }

  print('\n\n╡ ••• Done ••• ╞\n\n');
  exit(0);
}
