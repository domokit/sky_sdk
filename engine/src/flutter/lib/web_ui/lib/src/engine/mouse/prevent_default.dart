// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../dom.dart';

/// Listener for DOM events that prevents the default browser behavior.
final DomEventListener preventDefaultListener = createDomEventListener((DomEvent event) {
  event.preventDefault();
});
