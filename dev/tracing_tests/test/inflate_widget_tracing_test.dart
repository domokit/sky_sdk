// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/scheduler.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'common.dart';

final Set<String> interestingLabels = <String>{
  '$Row',
  '$TestRoot',
  '$TestChildWidget',
  '$Container',
};

void main() {
  ZoneIgnoringTestBinding.ensureInitialized();
  initTimelineTests();
  test('Children of MultiChildRenderObjectElement show up in tracing', () async {
    // We don't have expectations around the first frame because there's a race around
    // the warm-up frame that we don't want to get involved in here.
    await runFrame(() {
      runApp(const TestRoot());
    });
    await SchedulerBinding.instance.endOfFrame;
    await fetchInterestingEvents(interestingLabels);

    debugProfileBuildsEnabled = true;

    await runFrame(() {
      TestRoot.state.showRow();
    });
    expect(await fetchInterestingEventNames(interestingLabels), <String>[
      'TestRoot',
      'Row',
      'TestChildWidget',
      'Container',
      'TestChildWidget',
      'Container',
    ]);

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
  @override
  void initState() {
    super.initState();
    TestRoot.state = this;
  }

  bool _showRow = false;
  void showRow() {
    setState(() {
      _showRow = true;
    });
  }

  @override
  Widget build(BuildContext context) {
    return _showRow
        ? const Row(children: <Widget>[TestChildWidget(), TestChildWidget()])
        : Container();
  }
}

class TestChildWidget extends StatelessWidget {
  const TestChildWidget({super.key});

  @override
  Widget build(BuildContext context) {
    return Container();
  }
}
