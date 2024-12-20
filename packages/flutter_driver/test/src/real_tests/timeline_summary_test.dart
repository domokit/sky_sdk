// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show json;
import 'dart:math';

import 'package:file/file.dart';
import 'package:flutter_driver/flutter_driver.dart';
import 'package:flutter_driver/src/driver/profiling_summarizer.dart';
import 'package:flutter_driver/src/driver/refresh_rate_summarizer.dart';
import 'package:flutter_driver/src/driver/scene_display_lag_summarizer.dart';
import 'package:flutter_driver/src/driver/vsync_frame_lag_summarizer.dart';
import 'package:path/path.dart' as path;

import '../../common.dart';

void main() {
  group('TimelineSummary', () {
    TimelineSummary summarize(List<Map<String, dynamic>> testEvents) {
      return TimelineSummary.summarize(
        Timeline.fromJson(<String, dynamic>{'traceEvents': testEvents}),
      );
    }

    Map<String, dynamic> frameBegin(int timeStamp) => <String, dynamic>{
      'name': 'Frame',
      'ph': 'B',
      'ts': timeStamp,
    };

    Map<String, dynamic> frameEnd(int timeStamp) => <String, dynamic>{
      'name': 'Frame',
      'ph': 'E',
      'ts': timeStamp,
    };

    Map<String, dynamic> begin(int timeStamp) => <String, dynamic>{
      'name': 'GPURasterizer::Draw',
      'ph': 'B',
      'ts': timeStamp,
    };

    Map<String, dynamic> end(int timeStamp) => <String, dynamic>{
      'name': 'GPURasterizer::Draw',
      'ph': 'E',
      'ts': timeStamp,
    };

    Map<String, dynamic> lagBegin(int timeStamp, int vsyncsMissed) => <String, dynamic>{
      'name': 'SceneDisplayLag',
      'ph': 'b',
      'ts': timeStamp,
      'args': <String, String>{'vsync_transitions_missed': vsyncsMissed.toString()},
    };

    Map<String, dynamic> lagEnd(int timeStamp, int vsyncsMissed) => <String, dynamic>{
      'name': 'SceneDisplayLag',
      'ph': 'e',
      'ts': timeStamp,
      'args': <String, String>{'vsync_transitions_missed': vsyncsMissed.toString()},
    };

    Map<String, dynamic> cpuUsage(int timeStamp, double cpuUsage) => <String, dynamic>{
      'cat': 'embedder',
      'name': 'CpuUsage',
      'ts': timeStamp,
      'args': <String, String>{'total_cpu_usage': cpuUsage.toString()},
    };

    Map<String, dynamic> memoryUsage(int timeStamp, double dirty, double shared) =>
        <String, dynamic>{
          'cat': 'embedder',
          'name': 'MemoryUsage',
          'ts': timeStamp,
          'args': <String, String>{
            'owned_shared_memory_usage': shared.toString(),
            'dirty_memory_usage': dirty.toString(),
          },
        };

    Map<String, dynamic> platformVsync(int timeStamp) => <String, dynamic>{
      'name': 'VSYNC',
      'ph': 'B',
      'ts': timeStamp,
    };

    Map<String, dynamic> vsyncCallback(
      int timeStamp, {
      String phase = 'B',
      String startTime = '2750850055428',
      String endTime = '2750866722095',
    }) => <String, dynamic>{
      'name': 'VsyncProcessCallback',
      'ph': phase,
      'ts': timeStamp,
      'args': <String, dynamic>{'StartTime': startTime, 'TargetTime': endTime},
    };

    List<Map<String, dynamic>> genGC(String name, int count, int startTime, int timeDiff) {
      int ts = startTime;
      bool begin = true;
      final List<Map<String, dynamic>> ret = <Map<String, dynamic>>[];
      for (int i = 0; i < count; i++) {
        ret.add(<String, dynamic>{
          'name': name,
          'cat': 'GC',
          'tid': 19695,
          'pid': 19650,
          'ts': ts,
          'tts': ts,
          'ph': begin ? 'B' : 'E',
          'args': <String, dynamic>{'isolateGroupId': 'isolateGroups/10824099774666259225'},
        });
        ts = ts + timeDiff;
        begin = !begin;
      }
      return ret;
    }

    List<Map<String, dynamic>> newGenGC(int count, int startTime, int timeDiff) {
      return genGC('CollectNewGeneration', count, startTime, timeDiff);
    }

    List<Map<String, dynamic>> oldGenGC(int count, int startTime, int timeDiff) {
      return genGC('CollectOldGeneration', count, startTime, timeDiff);
    }

    List<Map<String, dynamic>> rasterizeTimeSequenceInMillis(List<int> sequence) {
      final List<Map<String, dynamic>> result = <Map<String, dynamic>>[];
      int t = 0;
      for (final int duration in sequence) {
        result.add(begin(t));
        t += duration * 1000;
        result.add(end(t));
      }
      return result;
    }

    Map<String, dynamic> frameRequestPendingStart(String id, int timeStamp) => <String, dynamic>{
      'name': 'Frame Request Pending',
      'ph': 'b',
      'id': id,
      'ts': timeStamp,
    };

    Map<String, dynamic> frameRequestPendingEnd(String id, int timeStamp) => <String, dynamic>{
      'name': 'Frame Request Pending',
      'ph': 'e',
      'id': id,
      'ts': timeStamp,
    };

    group('frame_count', () {
      test('counts frames', () {
        expect(
          summarize(<Map<String, dynamic>>[
            frameBegin(1000),
            frameEnd(2000),
            frameBegin(3000),
            frameEnd(5000),
          ]).countFrames(),
          2,
        );
      });
    });

    group('average_frame_build_time_millis', () {
      test('throws when there is no data', () {
        expect(
          () => summarize(<Map<String, dynamic>>[]).computeAverageFrameBuildTimeMillis(),
          throwsA(
            isA<StateError>().having(
              (StateError e) => e.message,
              'message',
              contains('The TimelineSummary had no events to summarize.'),
            ),
          ),
        );
      });

      test('computes average frame build time in milliseconds', () {
        expect(
          summarize(<Map<String, dynamic>>[
            frameBegin(1000),
            frameEnd(2000),
            frameBegin(3000),
            frameEnd(5000),
          ]).computeAverageFrameBuildTimeMillis(),
          1.5,
        );
      });

      test('skips leading "end" events', () {
        expect(
          summarize(<Map<String, dynamic>>[
            frameEnd(1000),
            frameBegin(2000),
            frameEnd(4000),
          ]).computeAverageFrameBuildTimeMillis(),
          2.0,
        );
      });

      test('skips trailing "begin" events', () {
        expect(
          summarize(<Map<String, dynamic>>[
            frameBegin(2000),
            frameEnd(4000),
            frameBegin(5000),
          ]).computeAverageFrameBuildTimeMillis(),
          2.0,
        );
      });

      // see https://github.com/flutter/flutter/issues/54095.
      test('ignore multiple "end" events', () {
        expect(
          summarize(<Map<String, dynamic>>[
            frameBegin(2000), frameEnd(4000),
            frameEnd(4300), // rogue frame end.
            frameBegin(5000), frameEnd(6000),
          ]).computeAverageFrameBuildTimeMillis(),
          1.5,
        );
      });

      test('pick latest when there are multiple "begin" events', () {
        expect(
          summarize(<Map<String, dynamic>>[
            frameBegin(1000), // rogue frame begin.
            frameBegin(2000), frameEnd(4000),
            frameEnd(4300), // rogue frame end.
            frameBegin(4400), // rogue frame begin.
            frameBegin(5000), frameEnd(6000),
          ]).computeAverageFrameBuildTimeMillis(),
          1.5,
        );
      });
    });

    group('worst_frame_build_time_millis', () {
      test('throws when there is no data', () {
        expect(
          () => summarize(<Map<String, dynamic>>[]).computeWorstFrameBuildTimeMillis(),
          throwsA(
            isA<StateError>().having(
              (StateError e) => e.message,
              'message',
              contains('The TimelineSummary had no events to summarize.'),
            ),
          ),
        );
      });

      test('computes worst frame build time in milliseconds', () {
        expect(
          summarize(<Map<String, dynamic>>[
            frameBegin(1000),
            frameEnd(2000),
            frameBegin(3000),
            frameEnd(5000),
          ]).computeWorstFrameBuildTimeMillis(),
          2.0,
        );
        expect(
          summarize(<Map<String, dynamic>>[
            frameBegin(3000),
            frameEnd(5000),
            frameBegin(1000),
            frameEnd(2000),
          ]).computeWorstFrameBuildTimeMillis(),
          2.0,
        );
      });

      test('skips leading "end" events', () {
        expect(
          summarize(<Map<String, dynamic>>[
            frameEnd(1000),
            frameBegin(2000),
            frameEnd(4000),
          ]).computeWorstFrameBuildTimeMillis(),
          2.0,
        );
      });

      test('skips trailing "begin" events', () {
        expect(
          summarize(<Map<String, dynamic>>[
            frameBegin(2000),
            frameEnd(4000),
            frameBegin(5000),
          ]).computeWorstFrameBuildTimeMillis(),
          2.0,
        );
      });
    });

    group('computeMissedFrameBuildBudgetCount', () {
      test('computes the number of missed build budgets', () {
        final TimelineSummary summary = summarize(<Map<String, dynamic>>[
          frameBegin(1000),
          frameEnd(18000),
          frameBegin(19000),
          frameEnd(28000),
          frameBegin(29000),
          frameEnd(47000),
        ]);

        expect(summary.countFrames(), 3);
        expect(summary.computeMissedFrameBuildBudgetCount(), 2);
      });
    });

    group('average_frame_rasterizer_time_millis', () {
      test('throws when there is no data', () {
        expect(
          () => summarize(<Map<String, dynamic>>[]).computeAverageFrameRasterizerTimeMillis(),
          throwsA(
            isA<StateError>().having(
              (StateError e) => e.message,
              'message',
              contains('The TimelineSummary had no events to summarize.'),
            ),
          ),
        );
      });

      test('computes average frame rasterizer time in milliseconds', () {
        expect(
          summarize(<Map<String, dynamic>>[
            begin(1000),
            end(2000),
            begin(3000),
            end(5000),
          ]).computeAverageFrameRasterizerTimeMillis(),
          1.5,
        );
      });

      test('skips leading "end" events', () {
        expect(
          summarize(<Map<String, dynamic>>[
            end(1000),
            begin(2000),
            end(4000),
          ]).computeAverageFrameRasterizerTimeMillis(),
          2.0,
        );
      });

      test('skips trailing "begin" events', () {
        expect(
          summarize(<Map<String, dynamic>>[
            begin(2000),
            end(4000),
            begin(5000),
          ]).computeAverageFrameRasterizerTimeMillis(),
          2.0,
        );
      });
    });

    group('worst_frame_rasterizer_time_millis', () {
      test('throws when there is no data', () {
        expect(
          () => summarize(<Map<String, dynamic>>[]).computeWorstFrameRasterizerTimeMillis(),
          throwsA(
            isA<StateError>().having(
              (StateError e) => e.message,
              'message',
              contains('The TimelineSummary had no events to summarize.'),
            ),
          ),
        );
      });

      test('computes worst frame rasterizer time in milliseconds', () {
        expect(
          summarize(<Map<String, dynamic>>[
            begin(1000),
            end(2000),
            begin(3000),
            end(5000),
          ]).computeWorstFrameRasterizerTimeMillis(),
          2.0,
        );
        expect(
          summarize(<Map<String, dynamic>>[
            begin(3000),
            end(5000),
            begin(1000),
            end(2000),
          ]).computeWorstFrameRasterizerTimeMillis(),
          2.0,
        );
      });

      test('skips leading "end" events', () {
        expect(
          summarize(<Map<String, dynamic>>[
            end(1000),
            begin(2000),
            end(4000),
          ]).computeWorstFrameRasterizerTimeMillis(),
          2.0,
        );
      });

      test('skips trailing "begin" events', () {
        expect(
          summarize(<Map<String, dynamic>>[
            begin(2000),
            end(4000),
            begin(5000),
          ]).computeWorstFrameRasterizerTimeMillis(),
          2.0,
        );
      });
    });

    group('percentile_frame_rasterizer_time_millis', () {
      test('throws when there is no data', () {
        expect(
          () =>
              summarize(<Map<String, dynamic>>[]).computePercentileFrameRasterizerTimeMillis(90.0),
          throwsA(
            isA<StateError>().having(
              (StateError e) => e.message,
              'message',
              contains('The TimelineSummary had no events to summarize.'),
            ),
          ),
        );
      });

      const List<List<int>> sequences = <List<int>>[
        <int>[1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
        <int>[1, 2, 3, 4, 5],
        <int>[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20],
      ];

      const List<int> p90s = <int>[9, 5, 18];

      test('computes 90th frame rasterizer time in milliseconds', () {
        for (int i = 0; i < sequences.length; ++i) {
          expect(
            summarize(
              rasterizeTimeSequenceInMillis(sequences[i]),
            ).computePercentileFrameRasterizerTimeMillis(90.0),
            p90s[i],
          );
        }
      });

      test('compute 99th frame rasterizer time in milliseconds', () {
        final List<int> sequence = <int>[];
        for (int i = 1; i <= 100; ++i) {
          sequence.add(i);
        }
        expect(
          summarize(
            rasterizeTimeSequenceInMillis(sequence),
          ).computePercentileFrameRasterizerTimeMillis(99.0),
          99,
        );
      });
    });

    group('computeMissedFrameRasterizerBudgetCount', () {
      test('computes the number of missed rasterizer budgets', () {
        final TimelineSummary summary = summarize(<Map<String, dynamic>>[
          begin(1000),
          end(18000),
          begin(19000),
          end(28000),
          begin(29000),
          end(47000),
        ]);

        expect(summary.computeMissedFrameRasterizerBudgetCount(), 2);
      });
    });

    group('summaryJson', () {
      test('computes and returns summary as JSON', () {
        expect(
          summarize(<Map<String, dynamic>>[
            begin(1000),
            end(19000),
            begin(19001),
            end(29001),
            begin(29002),
            end(49002),
            ...newGenGC(4, 10, 100),
            ...oldGenGC(5, 10000, 100),
            frameBegin(1000),
            frameEnd(18000),
            frameBegin(19000),
            frameEnd(28000),
            frameBegin(29000),
            frameEnd(48000),
            frameRequestPendingStart('1', 1000),
            frameRequestPendingEnd('1', 2000),
            frameRequestPendingStart('2', 3000),
            frameRequestPendingEnd('2', 5000),
            frameRequestPendingStart('3', 6000),
            frameRequestPendingEnd('3', 9000),
          ]).summaryJson,
          <String, dynamic>{
            'average_frame_build_time_millis': 15.0,
            '90th_percentile_frame_build_time_millis': 19.0,
            '99th_percentile_frame_build_time_millis': 19.0,
            'worst_frame_build_time_millis': 19.0,
            'missed_frame_build_budget_count': 2,
            'average_frame_rasterizer_time_millis': 16.0,
            'stddev_frame_rasterizer_time_millis': 4.0,
            '90th_percentile_frame_rasterizer_time_millis': 20.0,
            '99th_percentile_frame_rasterizer_time_millis': 20.0,
            'worst_frame_rasterizer_time_millis': 20.0,
            'missed_frame_rasterizer_budget_count': 2,
            'frame_count': 3,
            'frame_rasterizer_count': 3,
            'new_gen_gc_count': 4,
            'old_gen_gc_count': 5,
            'frame_build_times': <int>[17000, 9000, 19000],
            'frame_rasterizer_times': <int>[18000, 10000, 20000],
            'frame_begin_times': <int>[0, 18000, 28000],
            'frame_rasterizer_begin_times': <int>[0, 18001, 28002],
            'average_vsync_transitions_missed': 0.0,
            '90th_percentile_vsync_transitions_missed': 0.0,
            '99th_percentile_vsync_transitions_missed': 0.0,
            'average_vsync_frame_lag': 0.0,
            '90th_percentile_vsync_frame_lag': 0.0,
            '99th_percentile_vsync_frame_lag': 0.0,
            'average_layer_cache_count': 0.0,
            '90th_percentile_layer_cache_count': 0.0,
            '99th_percentile_layer_cache_count': 0.0,
            'worst_layer_cache_count': 0.0,
            'average_layer_cache_memory': 0.0,
            '90th_percentile_layer_cache_memory': 0.0,
            '99th_percentile_layer_cache_memory': 0.0,
            'worst_layer_cache_memory': 0.0,
            'average_picture_cache_count': 0.0,
            '90th_percentile_picture_cache_count': 0.0,
            '99th_percentile_picture_cache_count': 0.0,
            'worst_picture_cache_count': 0.0,
            'average_picture_cache_memory': 0.0,
            '90th_percentile_picture_cache_memory': 0.0,
            '99th_percentile_picture_cache_memory': 0.0,
            'worst_picture_cache_memory': 0.0,
            'total_ui_gc_time': 0.4,
            '30hz_frame_percentage': 0,
            '60hz_frame_percentage': 0,
            '80hz_frame_percentage': 0,
            '90hz_frame_percentage': 0,
            '120hz_frame_percentage': 0,
            'illegal_refresh_rate_frame_count': 0,
            'average_frame_request_pending_latency': 2000.0,
            '90th_percentile_frame_request_pending_latency': 3000.0,
            '99th_percentile_frame_request_pending_latency': 3000.0,
            'average_gpu_frame_time': 0,
            '90th_percentile_gpu_frame_time': 0,
            '99th_percentile_gpu_frame_time': 0,
            'worst_gpu_frame_time': 0,
            'average_gpu_memory_mb': 0,
            '90th_percentile_gpu_memory_mb': 0,
            '99th_percentile_gpu_memory_mb': 0,
            'worst_gpu_memory_mb': 0,
          },
        );
      });
    });

    group('writeTimelineToFile', () {
      late Directory tempDir;

      setUp(() {
        useMemoryFileSystemForTesting();
        tempDir = fs.systemTempDirectory.createTempSync('flutter_driver_test.');
      });

      tearDown(() {
        tryToDelete(tempDir);
        restoreFileSystem();
      });

      test('writes timeline to JSON file without summary', () async {
        await summarize(<Map<String, String>>[
          <String, String>{'foo': 'bar'},
        ]).writeTimelineToFile('test', destinationDirectory: tempDir.path, includeSummary: false);
        final String written =
            await fs.file(path.join(tempDir.path, 'test.timeline.json')).readAsString();
        expect(written, '{"traceEvents":[{"foo":"bar"}]}');
      });

      test('writes timeline to JSON file with summary', () async {
        await summarize(<Map<String, dynamic>>[
          <String, String>{'foo': 'bar'},
          begin(1000),
          end(19000),
          frameBegin(1000),
          frameEnd(18000),
        ]).writeTimelineToFile('test', destinationDirectory: tempDir.path);
        final String written =
            await fs.file(path.join(tempDir.path, 'test.timeline.json')).readAsString();
        expect(
          written,
          '{"traceEvents":[{"foo":"bar"},'
          '{"name":"GPURasterizer::Draw","ph":"B","ts":1000},'
          '{"name":"GPURasterizer::Draw","ph":"E","ts":19000},'
          '{"name":"Frame","ph":"B","ts":1000},'
          '{"name":"Frame","ph":"E","ts":18000}]}',
        );
      });

      test('writes summary to JSON file', () async {
        await summarize(<Map<String, dynamic>>[
          begin(1000),
          end(19000),
          begin(19001),
          end(29001),
          begin(29002),
          end(49002),
          frameBegin(1000),
          frameEnd(18000),
          frameBegin(19000),
          frameEnd(28000),
          frameBegin(29000),
          frameEnd(48000),
          lagBegin(1000, 4),
          lagEnd(2000, 4),
          lagBegin(1200, 12),
          lagEnd(2400, 12),
          lagBegin(4200, 8),
          lagEnd(9400, 8),
          ...newGenGC(4, 10, 100),
          ...oldGenGC(5, 10000, 100),
          cpuUsage(5000, 20),
          cpuUsage(5010, 60),
          memoryUsage(6000, 20, 40),
          memoryUsage(6100, 30, 45),
          platformVsync(7000),
          vsyncCallback(7500),
          frameRequestPendingStart('1', 1000),
          frameRequestPendingEnd('1', 2000),
          frameRequestPendingStart('2', 3000),
          frameRequestPendingEnd('2', 5000),
          frameRequestPendingStart('3', 6000),
          frameRequestPendingEnd('3', 9000),
        ]).writeTimelineToFile('test', destinationDirectory: tempDir.path);
        final String written =
            await fs.file(path.join(tempDir.path, 'test.timeline_summary.json')).readAsString();
        expect(json.decode(written), <String, dynamic>{
          'average_frame_build_time_millis': 15.0,
          'worst_frame_build_time_millis': 19.0,
          '90th_percentile_frame_build_time_millis': 19.0,
          '99th_percentile_frame_build_time_millis': 19.0,
          'missed_frame_build_budget_count': 2,
          'average_frame_rasterizer_time_millis': 16.0,
          'stddev_frame_rasterizer_time_millis': 4.0,
          '90th_percentile_frame_rasterizer_time_millis': 20.0,
          '99th_percentile_frame_rasterizer_time_millis': 20.0,
          'worst_frame_rasterizer_time_millis': 20.0,
          'missed_frame_rasterizer_budget_count': 2,
          'frame_count': 3,
          'frame_rasterizer_count': 3,
          'new_gen_gc_count': 4,
          'old_gen_gc_count': 5,
          'frame_build_times': <int>[17000, 9000, 19000],
          'frame_rasterizer_times': <int>[18000, 10000, 20000],
          'frame_begin_times': <int>[0, 18000, 28000],
          'frame_rasterizer_begin_times': <int>[0, 18001, 28002],
          'average_vsync_transitions_missed': 8.0,
          '90th_percentile_vsync_transitions_missed': 12.0,
          '99th_percentile_vsync_transitions_missed': 12.0,
          'average_vsync_frame_lag': 500.0,
          '90th_percentile_vsync_frame_lag': 500.0,
          '99th_percentile_vsync_frame_lag': 500.0,
          'average_cpu_usage': 40.0,
          '90th_percentile_cpu_usage': 60.0,
          '99th_percentile_cpu_usage': 60.0,
          'average_memory_usage': 67.5,
          '90th_percentile_memory_usage': 75.0,
          '99th_percentile_memory_usage': 75.0,
          'average_layer_cache_count': 0.0,
          '90th_percentile_layer_cache_count': 0.0,
          '99th_percentile_layer_cache_count': 0.0,
          'worst_layer_cache_count': 0.0,
          'average_layer_cache_memory': 0.0,
          '90th_percentile_layer_cache_memory': 0.0,
          '99th_percentile_layer_cache_memory': 0.0,
          'worst_layer_cache_memory': 0.0,
          'average_picture_cache_count': 0.0,
          '90th_percentile_picture_cache_count': 0.0,
          '99th_percentile_picture_cache_count': 0.0,
          'worst_picture_cache_count': 0.0,
          'average_picture_cache_memory': 0.0,
          '90th_percentile_picture_cache_memory': 0.0,
          '99th_percentile_picture_cache_memory': 0.0,
          'worst_picture_cache_memory': 0.0,
          'total_ui_gc_time': 0.4,
          '30hz_frame_percentage': 0,
          '60hz_frame_percentage': 100,
          '80hz_frame_percentage': 0,
          '90hz_frame_percentage': 0,
          '120hz_frame_percentage': 0,
          'illegal_refresh_rate_frame_count': 0,
          'average_frame_request_pending_latency': 2000.0,
          '90th_percentile_frame_request_pending_latency': 3000.0,
          '99th_percentile_frame_request_pending_latency': 3000.0,
          'average_gpu_frame_time': 0,
          '90th_percentile_gpu_frame_time': 0,
          '99th_percentile_gpu_frame_time': 0,
          'worst_gpu_frame_time': 0,
          'average_gpu_memory_mb': 0,
          '90th_percentile_gpu_memory_mb': 0,
          '99th_percentile_gpu_memory_mb': 0,
          'worst_gpu_memory_mb': 0,
        });
      });
    });

    group('SceneDisplayLagSummarizer tests', () {
      SceneDisplayLagSummarizer summarize(List<Map<String, dynamic>> traceEvents) {
        final Timeline timeline = Timeline.fromJson(<String, dynamic>{'traceEvents': traceEvents});
        return SceneDisplayLagSummarizer(timeline.events!);
      }

      test('average_vsyncs_missed', () async {
        final SceneDisplayLagSummarizer summarizer = summarize(<Map<String, dynamic>>[
          lagBegin(1000, 4),
          lagEnd(2000, 4),
          lagBegin(1200, 12),
          lagEnd(2400, 12),
          lagBegin(4200, 8),
          lagEnd(9400, 8),
        ]);
        expect(summarizer.computeAverageVsyncTransitionsMissed(), 8.0);
      });

      test('all stats are 0 for 0 missed transitions', () async {
        final SceneDisplayLagSummarizer summarizer = summarize(<Map<String, dynamic>>[]);
        expect(summarizer.computeAverageVsyncTransitionsMissed(), 0.0);
        expect(summarizer.computePercentileVsyncTransitionsMissed(90.0), 0.0);
        expect(summarizer.computePercentileVsyncTransitionsMissed(99.0), 0.0);
      });

      test('90th_percentile_vsyncs_missed', () async {
        final SceneDisplayLagSummarizer summarizer = summarize(<Map<String, dynamic>>[
          lagBegin(1000, 4),
          lagEnd(2000, 4),
          lagBegin(1200, 12),
          lagEnd(2400, 12),
          lagBegin(4200, 8),
          lagEnd(9400, 8),
          lagBegin(6100, 14),
          lagEnd(11000, 14),
          lagBegin(7100, 16),
          lagEnd(11500, 16),
          lagBegin(7400, 11),
          lagEnd(13000, 11),
          lagBegin(8200, 27),
          lagEnd(14100, 27),
          lagBegin(8700, 7),
          lagEnd(14300, 7),
          lagBegin(24200, 4187),
          lagEnd(39400, 4187),
        ]);
        expect(summarizer.computePercentileVsyncTransitionsMissed(90), 27.0);
      });

      test('99th_percentile_vsyncs_missed', () async {
        final SceneDisplayLagSummarizer summarizer = summarize(<Map<String, dynamic>>[
          lagBegin(1000, 4),
          lagEnd(2000, 4),
          lagBegin(1200, 12),
          lagEnd(2400, 12),
          lagBegin(4200, 8),
          lagEnd(9400, 8),
          lagBegin(6100, 14),
          lagEnd(11000, 14),
          lagBegin(24200, 4187),
          lagEnd(39400, 4187),
        ]);
        expect(summarizer.computePercentileVsyncTransitionsMissed(99), 4187.0);
      });
    });

    group('ProfilingSummarizer tests', () {
      ProfilingSummarizer summarize(List<Map<String, dynamic>> traceEvents) {
        final Timeline timeline = Timeline.fromJson(<String, dynamic>{'traceEvents': traceEvents});
        return ProfilingSummarizer.fromEvents(timeline.events!);
      }

      test('has_both_cpu_and_memory_usage', () async {
        final ProfilingSummarizer summarizer = summarize(<Map<String, dynamic>>[
          cpuUsage(0, 10),
          memoryUsage(0, 6, 10),
          cpuUsage(0, 12),
          memoryUsage(0, 8, 40),
        ]);
        expect(summarizer.computeAverage(ProfileType.CPU), 11.0);
        expect(summarizer.computeAverage(ProfileType.Memory), 32.0);
      });

      test('has_only_memory_usage', () async {
        final ProfilingSummarizer summarizer = summarize(<Map<String, dynamic>>[
          memoryUsage(0, 6, 10),
          memoryUsage(0, 8, 40),
        ]);
        expect(summarizer.computeAverage(ProfileType.Memory), 32.0);
        expect(summarizer.summarize().containsKey('average_cpu_usage'), false);
      });

      test('90th_percentile_cpu_usage', () async {
        final ProfilingSummarizer summarizer = summarize(<Map<String, dynamic>>[
          cpuUsage(0, 10),
          cpuUsage(1, 20),
          cpuUsage(2, 20),
          cpuUsage(3, 80),
          cpuUsage(4, 70),
          cpuUsage(4, 72),
          cpuUsage(4, 85),
          cpuUsage(4, 100),
        ]);
        expect(summarizer.computePercentile(ProfileType.CPU, 90), 85.0);
      });
    });

    group('VsyncFrameLagSummarizer tests', () {
      VsyncFrameLagSummarizer summarize(List<Map<String, dynamic>> traceEvents) {
        final Timeline timeline = Timeline.fromJson(<String, dynamic>{'traceEvents': traceEvents});
        return VsyncFrameLagSummarizer(timeline.events!);
      }

      test('average_vsync_frame_lag', () async {
        final VsyncFrameLagSummarizer summarizer = summarize(<Map<String, dynamic>>[
          platformVsync(10),
          vsyncCallback(12),
          platformVsync(16),
          vsyncCallback(29),
        ]);
        expect(summarizer.computeAverageVsyncFrameLag(), 7.5);
      });

      test('malformed_event_ordering', () async {
        final VsyncFrameLagSummarizer summarizer = summarize(<Map<String, dynamic>>[
          vsyncCallback(10),
          platformVsync(10),
        ]);
        expect(summarizer.computeAverageVsyncFrameLag(), 0);
        expect(summarizer.computePercentileVsyncFrameLag(80), 0);
      });

      test('penalize_consecutive_vsyncs', () async {
        final VsyncFrameLagSummarizer summarizer = summarize(<Map<String, dynamic>>[
          platformVsync(10),
          platformVsync(12),
        ]);
        expect(summarizer.computeAverageVsyncFrameLag(), 2);
      });

      test('pick_nearest_platform_vsync', () async {
        final VsyncFrameLagSummarizer summarizer = summarize(<Map<String, dynamic>>[
          platformVsync(10),
          platformVsync(12),
          vsyncCallback(18),
        ]);
        expect(summarizer.computeAverageVsyncFrameLag(), 4);
      });

      test('percentile_vsync_frame_lag', () async {
        final List<Map<String, dynamic>> events = <Map<String, dynamic>>[];
        int ts = 100;
        for (int i = 0; i < 100; i++) {
          events.add(platformVsync(ts));
          ts = ts + 10 * (i + 1);
          events.add(vsyncCallback(ts));
        }

        final VsyncFrameLagSummarizer summarizer = summarize(events);
        expect(summarizer.computePercentileVsyncFrameLag(90), 890);
        expect(summarizer.computePercentileVsyncFrameLag(99), 990);
      });
    });

    group('RefreshRateSummarizer tests', () {
      const double kCompareDelta = 0.01;
      RefreshRateSummary summarizeRefresh(List<Map<String, dynamic>> traceEvents) {
        final Timeline timeline = Timeline.fromJson(<String, dynamic>{'traceEvents': traceEvents});
        return RefreshRateSummary(vsyncEvents: timeline.events!);
      }

      List<Map<String, dynamic>> populateEvents({
        required int numberOfEvents,
        required int startTime,
        required int interval,
        required int margin,
      }) {
        final List<Map<String, dynamic>> events = <Map<String, dynamic>>[];
        int startTimeInNanoseconds = startTime;
        for (int i = 0; i < numberOfEvents; i++) {
          final int randomMargin = margin >= 1 ? (-margin + Random().nextInt(margin * 2)) : 0;
          final int endTime = startTimeInNanoseconds + interval + randomMargin;
          events.add(
            vsyncCallback(
              0,
              startTime: startTimeInNanoseconds.toString(),
              endTime: endTime.toString(),
            ),
          );
          startTimeInNanoseconds = endTime;
        }
        return events;
      }

      test('Recognize 30 hz frames.', () async {
        const int startTimeInNanoseconds = 2750850055430;
        const int intervalInNanoseconds = 33333333;
        // allow some margins
        const int margin = 3000000;
        final List<Map<String, dynamic>> events = populateEvents(
          numberOfEvents: 100,
          startTime: startTimeInNanoseconds,
          interval: intervalInNanoseconds,
          margin: margin,
        );
        final RefreshRateSummary summary = summarizeRefresh(events);
        expect(summary.percentageOf30HzFrames, closeTo(100, kCompareDelta));
        expect(summary.percentageOf60HzFrames, 0);
        expect(summary.percentageOf90HzFrames, 0);
        expect(summary.percentageOf120HzFrames, 0);
        expect(summary.framesWithIllegalRefreshRate, isEmpty);
      });

      test('Recognize 60 hz frames.', () async {
        const int startTimeInNanoseconds = 2750850055430;
        const int intervalInNanoseconds = 16666666;
        // allow some margins
        const int margin = 1200000;
        final List<Map<String, dynamic>> events = populateEvents(
          numberOfEvents: 100,
          startTime: startTimeInNanoseconds,
          interval: intervalInNanoseconds,
          margin: margin,
        );

        final RefreshRateSummary summary = summarizeRefresh(events);
        expect(summary.percentageOf30HzFrames, 0);
        expect(summary.percentageOf60HzFrames, closeTo(100, kCompareDelta));
        expect(summary.percentageOf90HzFrames, 0);
        expect(summary.percentageOf120HzFrames, 0);
        expect(summary.framesWithIllegalRefreshRate, isEmpty);
      });

      test('Recognize 90 hz frames.', () async {
        const int startTimeInNanoseconds = 2750850055430;
        const int intervalInNanoseconds = 11111111;
        // allow some margins
        const int margin = 500000;
        final List<Map<String, dynamic>> events = populateEvents(
          numberOfEvents: 100,
          startTime: startTimeInNanoseconds,
          interval: intervalInNanoseconds,
          margin: margin,
        );

        final RefreshRateSummary summary = summarizeRefresh(events);
        expect(summary.percentageOf30HzFrames, 0);
        expect(summary.percentageOf60HzFrames, 0);
        expect(summary.percentageOf90HzFrames, closeTo(100, kCompareDelta));
        expect(summary.percentageOf120HzFrames, 0);
        expect(summary.framesWithIllegalRefreshRate, isEmpty);
      });

      test('Recognize 120 hz frames.', () async {
        const int startTimeInNanoseconds = 2750850055430;
        const int intervalInNanoseconds = 8333333;
        // allow some margins
        const int margin = 300000;
        final List<Map<String, dynamic>> events = populateEvents(
          numberOfEvents: 100,
          startTime: startTimeInNanoseconds,
          interval: intervalInNanoseconds,
          margin: margin,
        );
        final RefreshRateSummary summary = summarizeRefresh(events);
        expect(summary.percentageOf30HzFrames, 0);
        expect(summary.percentageOf60HzFrames, 0);
        expect(summary.percentageOf90HzFrames, 0);
        expect(summary.percentageOf120HzFrames, closeTo(100, kCompareDelta));
        expect(summary.framesWithIllegalRefreshRate, isEmpty);
      });

      test('Identify illegal refresh rates.', () async {
        const int startTimeInNanoseconds = 2750850055430;
        const int intervalInNanoseconds = 10000000;
        final List<Map<String, dynamic>> events = populateEvents(
          numberOfEvents: 1,
          startTime: startTimeInNanoseconds,
          interval: intervalInNanoseconds,
          margin: 0,
        );
        final RefreshRateSummary summary = summarizeRefresh(events);
        expect(summary.percentageOf30HzFrames, 0);
        expect(summary.percentageOf60HzFrames, 0);
        expect(summary.percentageOf90HzFrames, 0);
        expect(summary.percentageOf120HzFrames, 0);
        expect(summary.framesWithIllegalRefreshRate, isNotEmpty);
        expect(summary.framesWithIllegalRefreshRate.first, closeTo(100, kCompareDelta));
      });

      test('Mixed refresh rates.', () async {
        final List<Map<String, dynamic>> events = <Map<String, dynamic>>[];
        const int num30Hz = 10;
        const int num60Hz = 20;
        const int num80Hz = 20;
        const int num90Hz = 20;
        const int num120Hz = 40;
        const int numIllegal = 10;
        const int totalFrames = num30Hz + num60Hz + num80Hz + num90Hz + num120Hz + numIllegal;

        // Add 30hz frames
        events.addAll(
          populateEvents(numberOfEvents: num30Hz, startTime: 0, interval: 32000000, margin: 0),
        );

        // Add 60hz frames
        events.addAll(
          populateEvents(numberOfEvents: num60Hz, startTime: 0, interval: 16000000, margin: 0),
        );

        // Add 80hz frames
        events.addAll(
          populateEvents(numberOfEvents: num80Hz, startTime: 0, interval: 12000000, margin: 0),
        );

        // Add 90hz frames
        events.addAll(
          populateEvents(numberOfEvents: num90Hz, startTime: 0, interval: 11000000, margin: 0),
        );

        // Add 120hz frames
        events.addAll(
          populateEvents(numberOfEvents: num120Hz, startTime: 0, interval: 8000000, margin: 0),
        );

        // Add illegal refresh rate frames
        events.addAll(
          populateEvents(numberOfEvents: numIllegal, startTime: 0, interval: 60000, margin: 0),
        );

        final RefreshRateSummary summary = summarizeRefresh(events);

        expect(summary.percentageOf30HzFrames, closeTo(num30Hz / totalFrames * 100, kCompareDelta));
        expect(summary.percentageOf60HzFrames, closeTo(num60Hz / totalFrames * 100, kCompareDelta));
        expect(summary.percentageOf80HzFrames, closeTo(num80Hz / totalFrames * 100, kCompareDelta));
        expect(summary.percentageOf90HzFrames, closeTo(num90Hz / totalFrames * 100, kCompareDelta));
        expect(
          summary.percentageOf120HzFrames,
          closeTo(num120Hz / totalFrames * 100, kCompareDelta),
        );
        expect(summary.framesWithIllegalRefreshRate, isNotEmpty);
        expect(summary.framesWithIllegalRefreshRate.length, 10);
      });
    });
  });
}
