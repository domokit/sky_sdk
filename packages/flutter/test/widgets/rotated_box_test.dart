// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Rotated box control test', (WidgetTester tester) async {
    final List<String> log = <String>[];
    final Key rotatedBoxKey = UniqueKey();

    await tester.pumpWidget(
      Center(
        child: RotatedBox(
          key: rotatedBoxKey,
          quarterTurns: 1,
          child: Row(
            textDirection: TextDirection.ltr,
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              GestureDetector(
                onTap: () {
                  log.add('left');
                },
                child: Container(width: 100.0, height: 40.0, color: Colors.blue[500]),
              ),
              GestureDetector(
                onTap: () {
                  log.add('right');
                },
                child: Container(width: 75.0, height: 65.0, color: Colors.blue[500]),
              ),
            ],
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byKey(rotatedBoxKey));
    expect(box.size.width, equals(65.0));
    expect(box.size.height, equals(175.0));

    await tester.tapAt(const Offset(420.0, 280.0));
    expect(log, equals(<String>['left']));
    log.clear();

    await tester.tapAt(const Offset(380.0, 320.0));
    expect(log, equals(<String>['right']));
    log.clear();
  });
}
