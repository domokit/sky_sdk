// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SliverFillRemaining].

void main() => runApp(const SliverFillRemainingExampleApp());

class SliverFillRemainingExampleApp extends StatelessWidget {
  const SliverFillRemainingExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('SliverFillRemaining Sample')),
        body: const SliverFillRemainingExample(),
      ),
    );
  }
}

class SliverFillRemainingExample extends StatelessWidget {
  const SliverFillRemainingExample({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScrollView(
      // The ScrollPhysics are overridden here to illustrate the functionality
      // of fillOverscroll on all devices this sample may be run on.
      // fillOverscroll only changes the behavior of your layout when applied to
      // Scrollables that allow for overscroll. BouncingScrollPhysics are one
      // example, which are provided by default on the iOS platform.
      // BouncingScrollPhysics is combined with AlwaysScrollableScrollPhysics to
      // allow for the overscroll, regardless of the depth of the scrollable.
      physics: const BouncingScrollPhysics(parent: AlwaysScrollableScrollPhysics()),
      slivers: <Widget>[
        SliverToBoxAdapter(child: Container(color: Colors.tealAccent[700], height: 150.0)),
        SliverFillRemaining(
          hasScrollBody: false,
          // Switch for different overscroll behavior in your layout. If your
          // ScrollPhysics do not allow for overscroll, setting fillOverscroll
          // to true will have no effect.
          fillOverscroll: true,
          child: Container(
            color: Colors.teal[100],
            child: Align(
              alignment: Alignment.bottomCenter,
              child: Padding(
                padding: const EdgeInsets.all(16.0),
                child: ElevatedButton(
                  onPressed: () {
                    /* Place your onPressed code here! */
                  },
                  child: const Text('Bottom Pinned Button!'),
                ),
              ),
            ),
          ),
        ),
      ],
    );
  }
}
