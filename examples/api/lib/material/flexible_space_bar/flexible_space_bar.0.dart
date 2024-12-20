// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';

/// Flutter code sample for [FlexibleSpaceBar].

void main() => runApp(const FlexibleSpaceBarExampleApp());

class FlexibleSpaceBarExampleApp extends StatelessWidget {
  const FlexibleSpaceBarExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      scrollBehavior: const MaterialScrollBehavior().copyWith(
        dragDevices: PointerDeviceKind.values.toSet(),
      ),
      home: Scaffold(
        body: CustomScrollView(
          physics: const BouncingScrollPhysics(parent: AlwaysScrollableScrollPhysics()),
          slivers: <Widget>[
            SliverAppBar(
              stretch: true,
              onStretchTrigger: () {
                // Function callback for stretch
                return Future<void>.value();
              },
              expandedHeight: 300.0,
              flexibleSpace: FlexibleSpaceBar(
                stretchModes: const <StretchMode>[
                  StretchMode.zoomBackground,
                  StretchMode.blurBackground,
                  StretchMode.fadeTitle,
                ],
                centerTitle: true,
                title: const Text('Flight Report'),
                background: Stack(
                  fit: StackFit.expand,
                  children: <Widget>[
                    Image.network(
                      'https://flutter.github.io/assets-for-api-docs/assets/widgets/owl-2.jpg',
                      fit: BoxFit.cover,
                    ),
                    const DecoratedBox(
                      decoration: BoxDecoration(
                        gradient: LinearGradient(
                          begin: Alignment(0.0, 0.5),
                          end: Alignment.center,
                          colors: <Color>[Color(0x60000000), Color(0x00000000)],
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
            SliverList(
              delegate: SliverChildListDelegate(const <Widget>[
                ListTile(
                  leading: Icon(Icons.wb_sunny),
                  title: Text('Sunday'),
                  subtitle: Text('sunny, h: 80, l: 65'),
                ),
                ListTile(
                  leading: Icon(Icons.wb_sunny),
                  title: Text('Monday'),
                  subtitle: Text('sunny, h: 80, l: 65'),
                ),
                // ListTiles++
              ]),
            ),
          ],
        ),
      ),
    );
  }
}
