// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('DefaultAnimationStyle notifier value matches widget', (WidgetTester tester) async {
    late StateSetter setState;
    AnimationStyle style = AnimationStyle();
    const Widget child = SizedBox.shrink(key: Key('key'));

    await tester.pumpWidget(StatefulBuilder(
      builder: (BuildContext context, StateSetter stateSetter) {
        setState = stateSetter;
        return DefaultAnimationStyle(style: style, child: child);
      },
    ));

    final ValueListenable<AnimationStyle> styleNotifier = DefaultAnimationStyle.getNotifier(
      tester.element(find.byKey(const Key('key'))),
    );
    expect(styleNotifier.value, style);

    setState(() {
      style = AnimationStyle(curve: Curves.ease);
    });
    await tester.pump();
    expect(styleNotifier.value, style);

    setState(() {
      style = AnimationStyle(
        curve: Curves.bounceIn,
        duration: Durations.extralong4,
        reverseCurve: const SawTooth(2),
        reverseDuration: const Duration(days: DateTime.thursday),
      );
    });
    await tester.pump();
    expect(styleNotifier.value, style);
  });
}
