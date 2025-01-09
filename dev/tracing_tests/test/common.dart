// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:developer' as developer;
import 'dart:isolate' as isolate;

import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vm_service/vm_service.dart';
import 'package:vm_service/vm_service_io.dart';

export 'package:vm_service/vm_service.dart' show TimelineEvent;

late String isolateId;

late VmService _vmService;

void initTimelineTests() {
  setUpAll(() async {
    final developer.ServiceProtocolInfo info = await developer.Service.getInfo();
    if (info.serverUri == null) {
      fail('This test _must_ be run with --enable-vmservice.');
    }
    _vmService = await vmServiceConnectUri(
      'ws://localhost:${info.serverUri!.port}${info.serverUri!.path}ws',
    );
    await _vmService.setVMTimelineFlags(<String>['Dart']);
    isolateId = developer.Service.getIsolateId(isolate.Isolate.current)!;
  });
}

Future<List<TimelineEvent>> fetchTimelineEvents() async {
  final Timeline timeline = await _vmService.getVMTimeline();
  await _vmService.clearVMTimeline();
  return timeline.traceEvents!;
}

Future<List<TimelineEvent>> fetchInterestingEvents(Set<String> interestingLabels) async {
  return (await fetchTimelineEvents()).where((TimelineEvent event) {
    return interestingLabels.contains(event.json!['name']) &&
        event.json!['ph'] ==
            'B'; // "Begin" mark of events, vs E which is for the "End" mark of events.
  }).toList();
}

String eventToName(TimelineEvent event) => event.json!['name'] as String;

Future<List<String>> fetchInterestingEventNames(Set<String> interestingLabels) async {
  return (await fetchInterestingEvents(interestingLabels)).map<String>(eventToName).toList();
}

Future<void> runFrame(VoidCallback callback) {
  final Future<void> result = SchedulerBinding.instance.endOfFrame; // schedules a frame
  callback();
  return result;
}

// This binding skips the zones tests. These tests were written before we
// verified zones properly, and they have been legacied-in to avoid having
// to refactor them.
//
// When creating new tests, avoid relying on this class.
class ZoneIgnoringTestBinding extends WidgetsFlutterBinding {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
  }

  @override
  bool debugCheckZone(String entryPoint) {
    return true;
  }

  static ZoneIgnoringTestBinding get instance => BindingBase.checkInstance(_instance);
  static ZoneIgnoringTestBinding? _instance;

  static ZoneIgnoringTestBinding ensureInitialized() {
    if (ZoneIgnoringTestBinding._instance == null) {
      ZoneIgnoringTestBinding();
    }
    return ZoneIgnoringTestBinding.instance;
  }
}
