// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

import 'binding.dart';

/// Ensure the appropriate test binding is initialized.
TestWidgetsFlutterBinding ensureInitialized([@visibleForTesting Map<String, String>? environment]) {
  return AutomatedTestWidgetsFlutterBinding.ensureInitialized();
}

/// This method is a noop on the web.
void setupHttpOverrides() {}

/// This method is a noop on the web.
void mockFlutterAssets() {}
