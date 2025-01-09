// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test(
    'AnimationLocalStatusListenersMixin with AnimationLazyListenerMixin - removing unregistered listener is no-op',
    () {
      final _TestAnimationLocalStatusListeners uut = _TestAnimationLocalStatusListeners();
      void fakeListener(AnimationStatus status) {}
      uut.removeStatusListener(fakeListener);
      expect(uut.callsToStart, 0);
      expect(uut.callsToStop, 0);
    },
  );

  test(
    'AnimationLocalListenersMixin with AnimationLazyListenerMixin - removing unregistered listener is no-op',
    () {
      final _TestAnimationLocalListeners uut = _TestAnimationLocalListeners();
      void fakeListener() {}
      uut.removeListener(fakeListener);
      expect(uut.callsToStart, 0);
      expect(uut.callsToStop, 0);
    },
  );
}

class _TestAnimationLocalStatusListeners
    with AnimationLocalStatusListenersMixin, AnimationLazyListenerMixin {
  int callsToStart = 0;
  int callsToStop = 0;

  @override
  void didStartListening() {
    callsToStart += 1;
  }

  @override
  void didStopListening() {
    callsToStop += 1;
  }
}

class _TestAnimationLocalListeners with AnimationLocalListenersMixin, AnimationLazyListenerMixin {
  int callsToStart = 0;
  int callsToStop = 0;

  @override
  void didStartListening() {
    callsToStart += 1;
  }

  @override
  void didStopListening() {
    callsToStop += 1;
  }
}
