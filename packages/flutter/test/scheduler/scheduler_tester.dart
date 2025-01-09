// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/scheduler.dart';

// flutter_ignore: deprecation_syntax (see analyze.dart)
@Deprecated('scheduler_tester is not compatible with dart:async')
class Future {} // so that people can't import us and dart:async

void tick(Duration duration) {
  // We don't bother running microtasks between these two calls
  // because we don't use Futures in these tests and so don't care.
  SchedulerBinding.instance.handleBeginFrame(duration);
  SchedulerBinding.instance.handleDrawFrame();
}
