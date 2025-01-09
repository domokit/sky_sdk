// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show JsonEncoder, json;

import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:flutter_driver/flutter_driver.dart';
import 'package:flutter_gallery/demo_lists.dart';
import 'package:path/path.dart' as path;
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

const FileSystem _fs = LocalFileSystem();

/// The demos we don't run as part of the integration test.
///
/// Demo names are formatted as 'DEMO_NAME@DEMO_CATEGORY' (see
/// `demo_lists.dart` for more examples).
const List<String> kSkippedDemos = <String>[
  // This demo is flaky on CI due to hitting the network.
  // See: https://github.com/flutter/flutter/issues/100497
  'Video@Media',
];

// All of the gallery demos, identified as "title@category".
//
// These names are reported by the test app, see _handleMessages()
// in transitions_perf.dart.
List<String> _allDemos = <String>[];

/// Extracts event data from [events] recorded by timeline, validates it, turns
/// it into a histogram, and saves to a JSON file.
Future<void> saveDurationsHistogram(List<Map<String, dynamic>> events, String outputPath) async {
  final Map<String, List<int>> durations = <String, List<int>>{};
  Map<String, dynamic>? startEvent;
  int? frameStart;

  // Save the duration of the first frame after each 'Start Transition' event.
  for (final Map<String, dynamic> event in events) {
    final String eventName = event['name'] as String;
    if (eventName == 'Start Transition') {
      assert(startEvent == null);
      startEvent = event;
    } else if (startEvent != null && eventName == 'Frame') {
      final String phase = event['ph'] as String;
      final int timestamp = event['ts'] as int;
      if (phase == 'B' || phase == 'b') {
        assert(frameStart == null);
        frameStart = timestamp;
      } else {
        assert(phase == 'E' || phase == 'e');
        final String routeName = (startEvent['args'] as Map<String, dynamic>)['to'] as String;
        durations[routeName] ??= <int>[];
        durations[routeName]!.add(timestamp - frameStart!);
        startEvent = null;
        frameStart = null;
      }
    }
  }

  // Verify that the durations data is valid.
  if (durations.keys.isEmpty) {
    throw 'no "Start Transition" timeline events found';
  }
  final Map<String, int> unexpectedValueCounts = <String, int>{};
  durations.forEach((String routeName, List<int> values) {
    if (values.length != 2) {
      unexpectedValueCounts[routeName] = values.length;
    }
  });

  if (unexpectedValueCounts.isNotEmpty) {
    final StringBuffer error = StringBuffer(
      'Some routes recorded wrong number of values (expected 2 values/route):\n\n',
    );
    // When run with --trace-startup, the VM stores trace events in an endless buffer instead of a ring buffer.
    error.write('You must add the --trace-startup parameter to run the test. \n\n');
    unexpectedValueCounts.forEach((String routeName, int count) {
      error.writeln(' - $routeName recorded $count values.');
    });
    error.writeln('\nFull event sequence:');
    final Iterator<Map<String, dynamic>> eventIter = events.iterator;
    String lastEventName = '';
    String lastRouteName = '';
    while (eventIter.moveNext()) {
      final String eventName = eventIter.current['name'] as String;

      if (!<String>['Start Transition', 'Frame'].contains(eventName)) {
        continue;
      }

      final String routeName =
          eventName == 'Start Transition'
              ? (eventIter.current['args'] as Map<String, dynamic>)['to'] as String
              : '';

      if (eventName == lastEventName && routeName == lastRouteName) {
        error.write('.');
      } else {
        error.write('\n - $eventName $routeName .');
      }

      lastEventName = eventName;
      lastRouteName = routeName;
    }
    throw error;
  }

  // Save the durations Map to a file.
  final File file = await _fs.file(outputPath).create(recursive: true);
  await file.writeAsString(const JsonEncoder.withIndent('  ').convert(durations));
}

/// Scrolls each demo menu item into view, launches it, then returns to the
/// home screen twice.
Future<void> runDemos(List<String> demos, FlutterDriver driver) async {
  final SerializableFinder demoList = find.byValueKey('GalleryDemoList');
  String? currentDemoCategory;

  for (final String demo in demos) {
    if (kSkippedDemos.contains(demo)) {
      continue;
    }

    final String demoName = demo.substring(0, demo.indexOf('@'));
    final String demoCategory = demo.substring(demo.indexOf('@') + 1);
    print('> $demo');

    final SerializableFinder demoCategoryItem = find.text(demoCategory);
    if (currentDemoCategory == null) {
      await driver.scrollIntoView(demoCategoryItem);
      await driver.tap(demoCategoryItem);
    } else if (currentDemoCategory != demoCategory) {
      await driver.tap(find.byTooltip('Back'));
      await driver.scrollIntoView(demoCategoryItem);
      await driver.tap(demoCategoryItem);
      // Scroll back to the top
      await driver.scroll(demoList, 0.0, 10000.0, const Duration(milliseconds: 100));
    }
    currentDemoCategory = demoCategory;

    final SerializableFinder demoItem = find.text(demoName);
    await driver.scrollUntilVisible(demoList, demoItem, dyScroll: -48.0, alignment: 0.5);

    for (int i = 0; i < 2; i += 1) {
      await driver.tap(demoItem); // Launch the demo

      if (kUnsynchronizedDemos.contains(demo)) {
        await driver.runUnsynchronized<void>(() async {
          await driver.tap(find.pageBack());
        });
      } else {
        await driver.tap(find.pageBack());
      }
    }

    print('< Success');
  }

  // Return to the home screen
  await driver.tap(find.byTooltip('Back'));
}

void main([List<String> args = const <String>[]]) {
  final bool withSemantics = args.contains('--with_semantics');
  final bool hybrid = args.contains('--hybrid');
  group('flutter gallery transitions', () {
    late FlutterDriver driver;
    setUpAll(() async {
      driver = await FlutterDriver.connect();

      // Wait for the first frame to be rasterized.
      await driver.waitUntilFirstFrameRasterized();
      if (withSemantics) {
        print('Enabling semantics...');
        await driver.setSemantics(true);
      }

      // See _handleMessages() in transitions_perf.dart.
      _allDemos = List<String>.from(
        json.decode(await driver.requestData('demoNames')) as List<dynamic>,
      );
      if (_allDemos.isEmpty) {
        throw 'no demo names found';
      }
    });

    tearDownAll(() async {
      await driver.close();
    });

    test(
      'find.bySemanticsLabel',
      () async {
        // Assert that we can use semantics related finders in profile mode.
        final int id = await driver.getSemanticsId(find.bySemanticsLabel('Material'));
        expect(id, greaterThan(-1));
      },
      skip: !withSemantics, // [intended] test only makes sense when semantics are turned on.
      timeout: Timeout.none,
    );

    test('all demos', () async {
      // Collect timeline data for just a limited set of demos to avoid OOMs.
      final Timeline timeline = await driver.traceAction(
        () async {
          if (hybrid) {
            await driver.requestData('profileDemos');
          } else {
            await runDemos(kProfiledDemos, driver);
          }
        },
        streams: const <TimelineStream>[
          TimelineStream.dart,
          TimelineStream.embedder,
          TimelineStream.gc,
        ],
        retainPriorEvents: true,
      );

      // Save the duration (in microseconds) of the first timeline Frame event
      // that follows a 'Start Transition' event. The Gallery app adds a
      // 'Start Transition' event when a demo is launched (see GalleryItem).
      final TimelineSummary summary = TimelineSummary.summarize(timeline);
      await summary.writeTimelineToFile('transitions', pretty: true);
      final String histogramPath = path.join(
        testOutputsDirectory,
        'transition_durations.timeline.json',
      );
      await saveDurationsHistogram(
        List<Map<String, dynamic>>.from(timeline.json['traceEvents'] as List<dynamic>),
        histogramPath,
      );

      // Execute the remaining tests.
      if (hybrid) {
        await driver.requestData('restDemos');
      } else {
        final Set<String> unprofiledDemos = Set<String>.from(_allDemos)..removeAll(kProfiledDemos);
        await runDemos(unprofiledDemos.toList(), driver);
      }
    }, timeout: Timeout.none);
  });
}
