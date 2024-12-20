// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() => runApp(const SliverDecorationExampleApp());

class SliverDecorationExampleApp extends StatelessWidget {
  const SliverDecorationExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(
        textTheme: const TextTheme(titleLarge: TextStyle(fontSize: 24, color: Colors.white30)),
      ),
      home: Scaffold(
        appBar: AppBar(title: const Text('SliverDecoration Sample')),
        body: const SliverDecorationExample(),
      ),
    );
  }
}

class SliverDecorationExample extends StatelessWidget {
  const SliverDecorationExample({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScrollView(
      slivers: <Widget>[
        DecoratedSliver(
          key: const ValueKey<String>('radial-gradient'),
          decoration: const BoxDecoration(
            gradient: RadialGradient(
              center: Alignment(-0.5, -0.6),
              radius: 0.15,
              colors: <Color>[Color(0xFFEEEEEE), Color(0xFF111133)],
              stops: <double>[0.4, 0.8],
            ),
          ),
          sliver: SliverList(
            delegate: SliverChildListDelegate(<Widget>[
              SizedBox(
                height: 200.0,
                child: Center(
                  child: Text(
                    'A moon on a night sky',
                    style: Theme.of(context).textTheme.titleLarge,
                  ),
                ),
              ),
            ]),
          ),
        ),
        DecoratedSliver(
          key: const ValueKey<String>('linear-gradient'),
          decoration: const BoxDecoration(
            gradient: LinearGradient(
              begin: Alignment.topCenter,
              end: Alignment.bottomCenter,
              colors: <Color>[
                Color(0xFF111133),
                Color(0xFF1A237E),
                Color(0xFF283593),
                Color(0xFF3949AB),
                Color(0xFF3F51B5),
                Color(0xFF1976D2),
                Color(0xFF1E88E5),
                Color(0xFF42A5F5),
              ],
            ),
          ),
          sliver: SliverList(
            delegate: SliverChildListDelegate(<Widget>[
              SizedBox(
                height: 500.0,
                child: Container(
                  alignment: Alignment.topCenter,
                  padding: const EdgeInsets.only(top: 56.0),
                  child: Text('A blue sky', style: Theme.of(context).textTheme.titleLarge),
                ),
              ),
            ]),
          ),
        ),
      ],
    );
  }
}
