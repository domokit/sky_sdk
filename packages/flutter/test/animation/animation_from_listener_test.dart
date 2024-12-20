// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('Animation created from ValueListenable', () {
    final ValueNotifier<double> listenable = ValueNotifier<double>(0.0);
    final Animation<double> animation = Animation<double>.fromValueListenable(listenable);

    expect(animation.status, AnimationStatus.forward);
    expect(animation.value, 0.0);

    listenable.value = 1.0;

    expect(animation.value, 1.0);

    bool listenerCalled = false;
    void listener() {
      listenerCalled = true;
    }

    animation.addListener(listener);

    listenable.value = 0.5;

    expect(listenerCalled, true);
    listenerCalled = false;

    animation.removeListener(listener);

    listenable.value = 0.2;
    expect(listenerCalled, false);
  });

  test('Animation created from ValueListenable can transform value', () {
    final ValueNotifier<double> listenable = ValueNotifier<double>(0.0);
    final Animation<double> animation = Animation<double>.fromValueListenable(
      listenable,
      transformer: (double input) {
        return input / 10;
      },
    );

    expect(animation.status, AnimationStatus.forward);
    expect(animation.value, 0.0);

    listenable.value = 10.0;

    expect(animation.value, 1.0);
  });

  test('Animation created from ValueListenable can be transformed via drive', () {
    final ValueNotifier<double> listenable = ValueNotifier<double>(0.0);
    final Animation<double> animation = Animation<double>.fromValueListenable(listenable);
    final Animation<Offset> offset = animation.drive(
      Animatable<Offset>.fromCallback((double value) {
        return Offset(0.0, value);
      }),
    );

    expect(offset.value, Offset.zero);
    expect(offset.status, AnimationStatus.forward);

    listenable.value = 10;

    expect(offset.value, const Offset(0.0, 10.0));

    bool listenerCalled = false;
    void listener() {
      listenerCalled = true;
    }

    offset.addListener(listener);

    listenable.value = 0.5;

    expect(listenerCalled, true);
    listenerCalled = false;

    offset.removeListener(listener);

    listenable.value = 0.2;
    expect(listenerCalled, false);
  });
}
