// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/scheduler.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'common.dart';

void main() {
  ZoneIgnoringTestBinding.ensureInitialized();
  initTimelineTests();
  test('Widgets with updated keys produce well formed timelines', () async {
    await runFrame(() {
      runApp(const TestRoot());
    });
    await SchedulerBinding.instance.endOfFrame;

    debugProfileBuildsEnabled = true;

    await runFrame(() {
      TestRoot.state.updateKey();
    });

    int buildCount = 0;
    for (final TimelineEvent event in await fetchTimelineEvents()) {
      if (event.json!['name'] == 'BUILD') {
        final String ph = event.json!['ph'] as String;
        if (ph == 'B') {
          buildCount++;
        } else if (ph == 'E') {
          buildCount--;
        }
      }
    }
    expect(buildCount, 0);

    debugProfileBuildsEnabled = false;
  }, skip: isBrowser); // [intended] uses dart:isolate and io.
}

class TestRoot extends StatefulWidget {
  const TestRoot({super.key});

  static late TestRootState state;

  @override
  State<TestRoot> createState() => TestRootState();
}

class TestRootState extends State<TestRoot> {
  final Key _globalKey = GlobalKey();
  Key _localKey = UniqueKey();

  @override
  void initState() {
    super.initState();
    TestRoot.state = this;
  }

  void updateKey() {
    setState(() {
      _localKey = UniqueKey();
    });
  }

  @override
  Widget build(BuildContext context) {
    return Center(key: _localKey, child: SizedBox(key: _globalKey, width: 100, height: 100));
  }
}
