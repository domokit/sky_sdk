// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

class TestPage extends StatelessWidget {
  const TestPage({super.key, this.useMaterial3});

  final bool? useMaterial3;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Test',
      debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
      theme: ThemeData(useMaterial3: useMaterial3, primarySwatch: Colors.blue),
      home: const HomePage(),
    );
  }
}

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<StatefulWidget> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  void _presentModalPage() {
    Navigator.of(context).push(
      PageRouteBuilder<void>(
        barrierColor: Colors.black54,
        opaque: false,
        pageBuilder: (BuildContext context, _, _) {
          return const ModalPage();
        },
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: const Center(child: Text('Test Home')),
      floatingActionButton: FloatingActionButton(
        onPressed: _presentModalPage,
        child: const Icon(Icons.add),
      ),
    );
  }
}

class ModalPage extends StatelessWidget {
  const ModalPage({super.key});

  @override
  Widget build(BuildContext context) {
    return Material(
      type: MaterialType.transparency,
      child: SafeArea(
        child: Stack(
          children: <Widget>[
            InkWell(
              highlightColor: Colors.transparent,
              splashColor: Colors.transparent,
              onTap: () {
                Navigator.of(context).pop();
              },
              child: const SizedBox.expand(),
            ),
            Align(
              alignment: Alignment.bottomCenter,
              child: Container(height: 150, color: Colors.teal),
            ),
          ],
        ),
      ),
    );
  }
}

void main() {
  testWidgets('Material2 - Barriers show when using PageRouteBuilder', (WidgetTester tester) async {
    await tester.pumpWidget(const TestPage(useMaterial3: false));
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pumpAndSettle();
    await expectLater(
      find.byType(TestPage),
      matchesGoldenFile('m2_page_route_builder.barrier.png'),
    );
  });

  testWidgets('Material3 - Barriers show when using PageRouteBuilder', (WidgetTester tester) async {
    await tester.pumpWidget(const TestPage(useMaterial3: true));
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pumpAndSettle();
    await expectLater(
      find.byType(TestPage),
      matchesGoldenFile('m3_page_route_builder.barrier.png'),
    );
  });
}
