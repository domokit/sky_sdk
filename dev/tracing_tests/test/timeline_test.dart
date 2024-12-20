// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';

import 'common.dart';

final Set<String> interestingLabels = <String>{
  'BUILD',
  'LAYOUT',
  'UPDATING COMPOSITING BITS',
  'PAINT',
  'COMPOSITING',
  'FINALIZE TREE',
  '$Placeholder',
  '$CustomPaint',
  '$RenderCustomPaint',
};

class TestRoot extends StatefulWidget {
  const TestRoot({super.key});

  static late final TestRootState state;

  @override
  State<TestRoot> createState() => TestRootState();
}

class TestRootState extends State<TestRoot> {
  @override
  void initState() {
    super.initState();
    TestRoot.state = this;
  }

  Widget _widget = const Placeholder();

  void updateWidget(Widget newWidget) {
    setState(() {
      _widget = newWidget;
    });
  }

  void rebuild() {
    setState(() {
      // no change, just force a rebuild
    });
  }

  @override
  Widget build(BuildContext context) {
    return _widget;
  }
}

void main() {
  ZoneIgnoringTestBinding.ensureInitialized();
  initTimelineTests();
  test('Timeline', () async {
    // We don't have expectations around the first frame because there's a race around
    // the warm-up frame that we don't want to get involved in here.
    await runFrame(() {
      runApp(const TestRoot());
    });
    await SchedulerBinding.instance.endOfFrame;
    await fetchInterestingEvents(interestingLabels);

    // The next few cases build the exact same tree so should have no effect.

    debugProfileBuildsEnabled = true;
    await runFrame(() {
      TestRoot.state.rebuild();
    });
    expect(await fetchInterestingEventNames(interestingLabels), <String>[
      'BUILD',
      'LAYOUT',
      'UPDATING COMPOSITING BITS',
      'PAINT',
      'COMPOSITING',
      'FINALIZE TREE',
    ]);
    debugProfileBuildsEnabled = false;

    debugProfileLayoutsEnabled = true;
    await runFrame(() {
      TestRoot.state.rebuild();
    });
    expect(await fetchInterestingEventNames(interestingLabels), <String>[
      'BUILD',
      'LAYOUT',
      'UPDATING COMPOSITING BITS',
      'PAINT',
      'COMPOSITING',
      'FINALIZE TREE',
    ]);
    debugProfileLayoutsEnabled = false;

    debugProfilePaintsEnabled = true;
    await runFrame(() {
      TestRoot.state.rebuild();
    });
    expect(await fetchInterestingEventNames(interestingLabels), <String>[
      'BUILD',
      'LAYOUT',
      'UPDATING COMPOSITING BITS',
      'PAINT',
      'COMPOSITING',
      'FINALIZE TREE',
    ]);
    debugProfilePaintsEnabled = false;

    // Now we replace the widgets each time to cause a rebuild.

    List<TimelineEvent> events;
    Map<String, String> args;

    debugProfileBuildsEnabled = true;
    debugEnhanceBuildTimelineArguments = true;
    await runFrame(() {
      TestRoot.state.updateWidget(Placeholder(key: UniqueKey(), color: const Color(0xFFFFFFFF)));
    });
    events = await fetchInterestingEvents(interestingLabels);
    expect(events.map<String>(eventToName), <String>[
      'BUILD',
      'Placeholder',
      'CustomPaint',
      'LAYOUT',
      'UPDATING COMPOSITING BITS',
      'PAINT',
      'COMPOSITING',
      'FINALIZE TREE',
    ]);
    args =
        (events
                    .where((TimelineEvent event) => event.json!['name'] == '$Placeholder')
                    .single
                    .json!['args']
                as Map<String, Object?>)
            .cast<String, String>();
    expect(args['color'], '${const Color(0xffffffff)}');
    debugProfileBuildsEnabled = false;
    debugEnhanceBuildTimelineArguments = false;

    debugProfileBuildsEnabledUserWidgets = true;
    debugEnhanceBuildTimelineArguments = true;
    await runFrame(() {
      TestRoot.state.updateWidget(Placeholder(key: UniqueKey(), color: const Color(0xFFFFFFFF)));
    });
    events = await fetchInterestingEvents(interestingLabels);
    expect(events.map<String>(eventToName), <String>[
      'BUILD',
      'Placeholder',
      'LAYOUT',
      'UPDATING COMPOSITING BITS',
      'PAINT',
      'COMPOSITING',
      'FINALIZE TREE',
    ]);
    args =
        (events
                    .where((TimelineEvent event) => event.json!['name'] == '$Placeholder')
                    .single
                    .json!['args']
                as Map<String, Object?>)
            .cast<String, String>();
    expect(args['color'], '${const Color(0xffffffff)}');
    debugProfileBuildsEnabledUserWidgets = false;
    debugEnhanceBuildTimelineArguments = false;

    debugProfileLayoutsEnabled = true;
    debugEnhanceLayoutTimelineArguments = true;
    await runFrame(() {
      TestRoot.state.updateWidget(Placeholder(key: UniqueKey()));
    });
    events = await fetchInterestingEvents(interestingLabels);
    expect(events.map<String>(eventToName), <String>[
      'BUILD',
      'LAYOUT',
      'RenderCustomPaint',
      'UPDATING COMPOSITING BITS',
      'PAINT',
      'COMPOSITING',
      'FINALIZE TREE',
    ]);
    args =
        (events
                    .where((TimelineEvent event) => event.json!['name'] == '$RenderCustomPaint')
                    .single
                    .json!['args']
                as Map<String, Object?>)
            .cast<String, String>();
    expect(args['creator'], startsWith('CustomPaint'));
    expect(args['creator'], contains('Placeholder'));
    expect(args['painter'], startsWith('_PlaceholderPainter#'));
    debugProfileLayoutsEnabled = false;
    debugEnhanceLayoutTimelineArguments = false;

    debugProfilePaintsEnabled = true;
    debugEnhancePaintTimelineArguments = true;
    await runFrame(() {
      TestRoot.state.updateWidget(Placeholder(key: UniqueKey()));
    });
    events = await fetchInterestingEvents(interestingLabels);
    expect(events.map<String>(eventToName), <String>[
      'BUILD',
      'LAYOUT',
      'UPDATING COMPOSITING BITS',
      'PAINT',
      'RenderCustomPaint',
      'COMPOSITING',
      'FINALIZE TREE',
    ]);
    args =
        (events
                    .where((TimelineEvent event) => event.json!['name'] == '$RenderCustomPaint')
                    .single
                    .json!['args']
                as Map<String, Object?>)
            .cast<String, String>();
    expect(args['creator'], startsWith('CustomPaint'));
    expect(args['creator'], contains('Placeholder'));
    expect(args['painter'], startsWith('_PlaceholderPainter#'));
    debugProfilePaintsEnabled = false;
    debugEnhancePaintTimelineArguments = false;
  }, skip: isBrowser); // [intended] uses dart:isolate and io.
}
