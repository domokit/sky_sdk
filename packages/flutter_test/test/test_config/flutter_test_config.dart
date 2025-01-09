// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

Future<void> testExecutable(FutureOr<void> Function() testMain) async {
  await runZoned<dynamic>(testMain, zoneValues: <Type, String>{String: '/test_config'});
}
