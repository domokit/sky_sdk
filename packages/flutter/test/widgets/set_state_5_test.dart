// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class BadWidget extends StatefulWidget {
  const BadWidget({super.key});
  @override
  State<StatefulWidget> createState() => BadWidgetState();
}

class BadWidgetState extends State<BadWidget> {
  BadWidgetState() {
    setState(() {
      _count = 1;
    });
  }

  int _count = 0;

  @override
  Widget build(BuildContext context) {
    return Text(_count.toString());
  }
}

void main() {
  testWidgets('setState() catches being used inside a constructor', (WidgetTester tester) async {
    await tester.pumpWidget(const BadWidget());
    expect(tester.takeException(), isFlutterError);
  });
}
