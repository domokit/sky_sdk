// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('StatefulWidget BuildContext.mounted', (WidgetTester tester) async {
    late BuildContext capturedContext;
    await tester.pumpWidget(
      TestStatefulWidget(
        onBuild: (BuildContext context) {
          capturedContext = context;
        },
      ),
    );
    expect(capturedContext.mounted, isTrue);
    await tester.pumpWidget(Container());
    expect(capturedContext.mounted, isFalse);
  });

  testWidgets('StatelessWidget BuildContext.mounted', (WidgetTester tester) async {
    late BuildContext capturedContext;
    await tester.pumpWidget(
      TestStatelessWidget(
        onBuild: (BuildContext context) {
          capturedContext = context;
        },
      ),
    );
    expect(capturedContext.mounted, isTrue);
    await tester.pumpWidget(Container());
    expect(capturedContext.mounted, isFalse);
  });
}

typedef BuildCallback = void Function(BuildContext context);

class TestStatelessWidget extends StatelessWidget {
  const TestStatelessWidget({super.key, required this.onBuild});

  final BuildCallback onBuild;

  @override
  Widget build(BuildContext context) {
    onBuild(context);
    return Container();
  }
}

class TestStatefulWidget extends StatefulWidget {
  const TestStatefulWidget({super.key, required this.onBuild});

  final BuildCallback onBuild;

  @override
  State<TestStatefulWidget> createState() => _TestStatefulWidgetState();
}

class _TestStatefulWidgetState extends State<TestStatefulWidget> {
  @override
  Widget build(BuildContext context) {
    widget.onBuild(context);
    return Container();
  }
}
