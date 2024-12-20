// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('gets local coordinates', (WidgetTester tester) async {
    final List<ScaleStartDetails> startDetails = <ScaleStartDetails>[];
    final List<ScaleUpdateDetails> updateDetails = <ScaleUpdateDetails>[];

    final Key redContainer = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: GestureDetector(
          onScaleStart: (ScaleStartDetails details) {
            startDetails.add(details);
          },
          onScaleUpdate: (ScaleUpdateDetails details) {
            updateDetails.add(details);
          },
          child: Container(key: redContainer, width: 100, height: 100, color: Colors.red),
        ),
      ),
    );

    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byKey(redContainer)) - const Offset(20, 20),
    );
    final TestGesture pointer2 = await tester.startGesture(
      tester.getCenter(find.byKey(redContainer)) + const Offset(30, 30),
    );
    await pointer2.moveTo(tester.getCenter(find.byKey(redContainer)) + const Offset(20, 20));

    expect(updateDetails.single.localFocalPoint, const Offset(50, 50));
    expect(updateDetails.single.focalPoint, const Offset(400, 300));

    expect(startDetails, hasLength(2));
    expect(startDetails.first.localFocalPoint, const Offset(30, 30));
    expect(startDetails.first.focalPoint, const Offset(380, 280));
    expect(startDetails.last.localFocalPoint, const Offset(50, 50));
    expect(startDetails.last.focalPoint, const Offset(400, 300));

    await tester.pumpAndSettle();
    await gesture.up();
    await pointer2.up();
    await tester.pumpAndSettle();
  });
}
