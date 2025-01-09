// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

const String _actualContent = 'Actual Content';
const String _loading = 'Loading...';

void main() {
  testWidgets('deferFirstFrame/allowFirstFrame stops sending frames to engine', (
    WidgetTester tester,
  ) async {
    expect(RendererBinding.instance.sendFramesToEngine, isTrue);

    final Completer<void> completer = Completer<void>();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: _DeferringWidget(key: UniqueKey(), loader: completer.future),
      ),
    );
    final _DeferringWidgetState state = tester.state<_DeferringWidgetState>(
      find.byType(_DeferringWidget),
    );

    expect(find.text(_loading), findsOneWidget);
    expect(find.text(_actualContent), findsNothing);
    expect(RendererBinding.instance.sendFramesToEngine, isFalse);

    await tester.pump();
    expect(find.text(_loading), findsOneWidget);
    expect(find.text(_actualContent), findsNothing);
    expect(RendererBinding.instance.sendFramesToEngine, isFalse);
    expect(state.doneLoading, isFalse);

    // Complete the future to start sending frames.
    completer.complete();
    await tester.idle();
    expect(state.doneLoading, isTrue);
    expect(RendererBinding.instance.sendFramesToEngine, isTrue);

    await tester.pump();
    expect(find.text(_loading), findsNothing);
    expect(find.text(_actualContent), findsOneWidget);
    expect(RendererBinding.instance.sendFramesToEngine, isTrue);
  });

  testWidgets('Two widgets can defer frames', (WidgetTester tester) async {
    expect(RendererBinding.instance.sendFramesToEngine, isTrue);

    final Completer<void> completer1 = Completer<void>();
    final Completer<void> completer2 = Completer<void>();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Row(
          children: <Widget>[
            _DeferringWidget(key: UniqueKey(), loader: completer1.future),
            _DeferringWidget(key: UniqueKey(), loader: completer2.future),
          ],
        ),
      ),
    );
    expect(find.text(_loading), findsNWidgets(2));
    expect(find.text(_actualContent), findsNothing);
    expect(RendererBinding.instance.sendFramesToEngine, isFalse);

    completer1.complete();
    completer2.complete();
    await tester.idle();

    await tester.pump();
    expect(find.text(_loading), findsNothing);
    expect(find.text(_actualContent), findsNWidgets(2));
    expect(RendererBinding.instance.sendFramesToEngine, isTrue);
  });
}

class _DeferringWidget extends StatefulWidget {
  const _DeferringWidget({required Key super.key, required this.loader});

  final Future<void> loader;

  @override
  State<_DeferringWidget> createState() => _DeferringWidgetState();
}

class _DeferringWidgetState extends State<_DeferringWidget> {
  bool doneLoading = false;

  @override
  void initState() {
    super.initState();
    RendererBinding.instance.deferFirstFrame();
    widget.loader.then((_) {
      setState(() {
        doneLoading = true;
        RendererBinding.instance.allowFirstFrame();
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return doneLoading ? const Text(_actualContent) : const Text(_loading);
  }
}
