// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import { FlutterLoader } from './loader.js';

if (!window._flutter) {
  window._flutter = {};
}

if (!window._flutter.loader) {
  window._flutter.loader = new FlutterLoader();
}
