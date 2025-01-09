// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

class InvalidOnInitLifecycleWidget extends StatefulWidget {
  const InvalidOnInitLifecycleWidget({super.key});

  @override
  InvalidOnInitLifecycleWidgetState createState() => InvalidOnInitLifecycleWidgetState();
}

class InvalidOnInitLifecycleWidgetState extends State<InvalidOnInitLifecycleWidget> {
  @override
  Future<void> initState() async {
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Container();
  }
}

class InvalidDidUpdateWidgetLifecycleWidget extends StatefulWidget {
  const InvalidDidUpdateWidgetLifecycleWidget({super.key, required this.color});

  final Color color;

  @override
  InvalidDidUpdateWidgetLifecycleWidgetState createState() =>
      InvalidDidUpdateWidgetLifecycleWidgetState();
}

class InvalidDidUpdateWidgetLifecycleWidgetState
    extends State<InvalidDidUpdateWidgetLifecycleWidget> {
  @override
  Future<void> didUpdateWidget(InvalidDidUpdateWidgetLifecycleWidget oldWidget) async {
    super.didUpdateWidget(oldWidget);
  }

  @override
  Widget build(BuildContext context) {
    return ColoredBox(color: widget.color);
  }
}

void main() {
  testWidgets(
    'async onInit throws FlutterError',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      await tester.pumpWidget(const InvalidOnInitLifecycleWidget());

      expect(tester.takeException(), isFlutterError);
    },
  );

  testWidgets(
    'async didUpdateWidget throws FlutterError',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      await tester.pumpWidget(const InvalidDidUpdateWidgetLifecycleWidget(color: Colors.green));
      await tester.pumpWidget(const InvalidDidUpdateWidgetLifecycleWidget(color: Colors.red));

      expect(tester.takeException(), isFlutterError);
    },
  );
}
