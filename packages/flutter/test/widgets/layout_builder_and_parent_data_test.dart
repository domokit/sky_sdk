// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class SizeChanger extends StatefulWidget {
  const SizeChanger({super.key, required this.child});

  final Widget child;

  @override
  SizeChangerState createState() => SizeChangerState();
}

class SizeChangerState extends State<SizeChanger> {
  bool _flag = false;

  void trigger() {
    setState(() {
      _flag = true;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Row(
      textDirection: TextDirection.ltr,
      children: <Widget>[SizedBox(height: _flag ? 50.0 : 100.0, width: 100.0, child: widget.child)],
    );
  }
}

void main() {
  testWidgets('Applying parent data inside a LayoutBuilder', (WidgetTester tester) async {
    int frame = 1;
    await tester.pumpWidget(
      SizeChanger(
        // when this is triggered, the child LayoutBuilder will build again
        child: LayoutBuilder(
          builder: (BuildContext context, BoxConstraints constraints) {
            return Column(
              children: <Widget>[
                Expanded(
                  flex:
                      frame, // this is different after the next pump, so that the parentData has to be applied again
                  child: Container(height: 100.0),
                ),
              ],
            );
          },
        ),
      ),
    );
    frame += 1;
    tester.state<SizeChangerState>(find.byType(SizeChanger)).trigger();
    await tester.pump();
  });
}
