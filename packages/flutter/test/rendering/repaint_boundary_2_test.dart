// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('repaint boundary with constraint changes', (WidgetTester tester) async {
    // Regression test for as https://github.com/flutter/flutter/issues/39151.
    await tester.pumpWidget(const RelayoutBoundariesCrash());
    tester.state<RelayoutBoundariesCrashState>(find.byType(RelayoutBoundariesCrash))._toggleMode();
    await tester.pump();
  });
}

class RelayoutBoundariesCrash extends StatefulWidget {
  const RelayoutBoundariesCrash({super.key});

  @override
  RelayoutBoundariesCrashState createState() => RelayoutBoundariesCrashState();
}

class RelayoutBoundariesCrashState extends State<RelayoutBoundariesCrash> {
  bool _mode = true;

  void _toggleMode() {
    setState(() {
      _mode = !_mode;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: SizedBox(
        // when _mode is true, constraints are tight, otherwise constraints are loose
        width: !_mode ? 100.0 : null,
        height: !_mode ? 100.0 : null,
        child: LayoutBuilder(
          builder: (BuildContext context, BoxConstraints constraints) {
            // Make the outer SizedBoxes relayout without making the Placeholders relayout.
            final double dimension = !_mode ? 10.0 : 20.0;
            return Column(
              children: <Widget>[
                SizedBox(width: dimension, height: dimension, child: const Placeholder()),
                SizedBox(width: dimension, height: dimension, child: const Placeholder()),
              ],
            );
          },
        ),
      ),
    );
  }
}
