// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:developer';

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

class TestWidget extends LeafRenderObjectWidget {
  const TestWidget({super.key});

  @override
  RenderObject createRenderObject(BuildContext context) => RenderTest();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    // This string is searched for verbatim by dev/bots/test.dart:
    properties.add(MessageProperty('test', 'TestWidget.debugFillProperties called'));
  }
}

class RenderTest extends RenderBox {
  @override
  bool get sizedByParent => true;

  @override
  void performResize() {
    Timeline.instantSync('RenderTest.performResize called');
    size = constraints.biggest;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    // This string is searched for verbatim by dev/bots/test.dart:
    properties.add(MessageProperty('test', 'RenderTest.debugFillProperties called'));
  }
}

Future<void> main() async {
  // This section introduces strings that we can search for in dev/bots/test.dart
  // as a sanity check:
  if (kDebugMode) {
    print('BUILT IN DEBUG MODE');
  }
  if (kProfileMode) {
    print('BUILT IN PROFILE MODE');
  }
  if (kReleaseMode) {
    print('BUILT IN RELEASE MODE');
  }

  // The point of this file is to make sure that toTimelineArguments is not
  // called when we have debugProfileBuildsEnabled (et al) turned on. If that
  // method is not called then the debugFillProperties methods above should also
  // not get called and we should end up tree-shaking the entire Diagnostics
  // logic out of the app. The dev/bots/test.dart test checks for this by
  // looking for the strings in the methods above.

  debugProfileBuildsEnabled = true;
  debugProfileLayoutsEnabled = true;
  debugProfilePaintsEnabled = true;
  runApp(const TestWidget());
}
