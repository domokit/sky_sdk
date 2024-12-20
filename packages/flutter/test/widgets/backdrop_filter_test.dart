// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Backdrop key is passed to backdrop Layer', (WidgetTester tester) async {
    final BackdropKey backdropKey = BackdropKey();

    Widget build({required bool enableKeys}) {
      return MaterialApp(
        home: Scaffold(
          body: ListView(
            children: <Widget>[
              ClipRect(
                child: BackdropFilter(
                  filter: ImageFilter.blur(sigmaX: 40, sigmaY: 40),
                  backdropGroupKey: enableKeys ? backdropKey : null,
                  child: Container(
                    color: Colors.black.withAlpha(40),
                    height: 200,
                    child: const Text('Item 1'),
                  ),
                ),
              ),
              ClipRect(
                child: BackdropFilter(
                  filter: ImageFilter.blur(sigmaX: 40, sigmaY: 40),
                  backdropGroupKey: enableKeys ? backdropKey : null,
                  child: Container(
                    color: Colors.black.withAlpha(40),
                    height: 200,
                    child: const Text('Item 1'),
                  ),
                ),
              ),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(build(enableKeys: true));

    List<BackdropFilterLayer> layers = tester.layers.whereType<BackdropFilterLayer>().toList();

    expect(layers.length, 2);
    expect(layers[0].backdropKey, backdropKey);
    expect(layers[1].backdropKey, backdropKey);

    await tester.pumpWidget(build(enableKeys: false));

    layers = tester.layers.whereType<BackdropFilterLayer>().toList();

    expect(layers.length, 2);
    expect(layers[0].backdropKey, null);
    expect(layers[1].backdropKey, null);
  });

  testWidgets('Backdrop key is passed to backdrop Layer via backdrop group', (
    WidgetTester tester,
  ) async {
    Widget build() {
      return MaterialApp(
        home: Scaffold(
          body: BackdropGroup(
            child: ListView(
              children: <Widget>[
                ClipRect(
                  child: BackdropFilter.grouped(
                    filter: ImageFilter.blur(sigmaX: 40, sigmaY: 40),
                    child: Container(
                      color: Colors.black.withAlpha(40),
                      height: 200,
                      child: const Text('Item 1'),
                    ),
                  ),
                ),
                ClipRect(
                  child: BackdropFilter.grouped(
                    filter: ImageFilter.blur(sigmaX: 40, sigmaY: 40),
                    child: Container(
                      color: Colors.black.withAlpha(40),
                      height: 200,
                      child: const Text('Item 1'),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(build());

    final List<BackdropFilterLayer> layers =
        tester.layers.whereType<BackdropFilterLayer>().toList();

    expect(layers.length, 2);
    expect(layers[0].backdropKey, layers[1].backdropKey);
  });

  testWidgets("Material2 - BackdropFilter's cull rect does not shrink", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Stack(
            fit: StackFit.expand,
            children: <Widget>[
              Text('0 0 ' * 10000),
              Center(
                // ClipRect needed for filtering the 200x200 area instead of the
                // whole screen.
                child: ClipRect(
                  child: BackdropFilter(
                    filter: ImageFilter.blur(sigmaX: 5.0, sigmaY: 5.0),
                    child: Container(
                      alignment: Alignment.center,
                      width: 200.0,
                      height: 200.0,
                      child: const Text('Hello World'),
                    ),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(RepaintBoundary).first,
      matchesGoldenFile('m2_backdrop_filter_test.cull_rect.png'),
    );
  });

  testWidgets("Material3 - BackdropFilter's cull rect does not shrink", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Scaffold(
          body: Stack(
            fit: StackFit.expand,
            children: <Widget>[
              Text('0 0 ' * 10000),
              Center(
                // ClipRect needed for filtering the 200x200 area instead of the
                // whole screen.
                child: ClipRect(
                  child: BackdropFilter(
                    filter: ImageFilter.blur(sigmaX: 5.0, sigmaY: 5.0),
                    child: Container(
                      alignment: Alignment.center,
                      width: 200.0,
                      height: 200.0,
                      child: const Text('Hello World'),
                    ),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(RepaintBoundary).first,
      matchesGoldenFile('m3_backdrop_filter_test.cull_rect.png'),
    );
  });

  testWidgets('Material2 - BackdropFilter blendMode on saveLayer', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Opacity(
            opacity: 0.9,
            child: Stack(
              fit: StackFit.expand,
              children: <Widget>[
                Text('0 0 ' * 10000),
                Column(
                  mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                  // ClipRect needed for filtering the 200x200 area instead of the
                  // whole screen.
                  children: <Widget>[
                    ClipRect(
                      child: BackdropFilter(
                        filter: ImageFilter.blur(sigmaX: 5.0, sigmaY: 5.0),
                        child: Container(
                          alignment: Alignment.center,
                          width: 200.0,
                          height: 200.0,
                          color: Colors.yellow.withAlpha(0x7),
                        ),
                      ),
                    ),
                    ClipRect(
                      child: BackdropFilter(
                        filter: ImageFilter.blur(sigmaX: 5.0, sigmaY: 5.0),
                        blendMode: BlendMode.src,
                        child: Container(
                          alignment: Alignment.center,
                          width: 200.0,
                          height: 200.0,
                          color: Colors.yellow.withAlpha(0x7),
                        ),
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(RepaintBoundary).first,
      matchesGoldenFile('m2_backdrop_filter_test.saveLayer.blendMode.png'),
    );
  });

  testWidgets('Material3 - BackdropFilter blendMode on saveLayer', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Scaffold(
          body: Opacity(
            opacity: 0.9,
            child: Stack(
              fit: StackFit.expand,
              children: <Widget>[
                Text('0 0 ' * 10000),
                Column(
                  mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                  // ClipRect needed for filtering the 200x200 area instead of the
                  // whole screen.
                  children: <Widget>[
                    ClipRect(
                      child: BackdropFilter(
                        filter: ImageFilter.blur(sigmaX: 5.0, sigmaY: 5.0),
                        child: Container(
                          alignment: Alignment.center,
                          width: 200.0,
                          height: 200.0,
                          color: Colors.yellow.withAlpha(0x7),
                        ),
                      ),
                    ),
                    ClipRect(
                      child: BackdropFilter(
                        filter: ImageFilter.blur(sigmaX: 5.0, sigmaY: 5.0),
                        blendMode: BlendMode.src,
                        child: Container(
                          alignment: Alignment.center,
                          width: 200.0,
                          height: 200.0,
                          color: Colors.yellow.withAlpha(0x7),
                        ),
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(RepaintBoundary).first,
      matchesGoldenFile('m3_backdrop_filter_test.saveLayer.blendMode.png'),
    );
  });
}
