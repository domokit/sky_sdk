// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// The flavor this app was built with.
///
/// This is equivalent to the value argued to the `--flavor` option at build time.
/// This will be `null` if the `--flavor` option was not provided.
const String? appFlavor =
    String.fromEnvironment('FLUTTER_APP_FLAVOR') != ''
        ? String.fromEnvironment('FLUTTER_APP_FLAVOR')
        : null;
