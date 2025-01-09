// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [RawScrollbar].

void main() => runApp(const RawScrollbarExampleApp());

class RawScrollbarExampleApp extends StatelessWidget {
  const RawScrollbarExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('RawScrollbar Sample')),
        body: const Center(child: RawScrollbarExample()),
      ),
    );
  }
}

class RawScrollbarExample extends StatefulWidget {
  const RawScrollbarExample({super.key});

  @override
  State<RawScrollbarExample> createState() => _RawScrollbarExampleState();
}

class _RawScrollbarExampleState extends State<RawScrollbarExample> {
  final ScrollController _controller = ScrollController();

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        return Row(
          children: <Widget>[
            SizedBox(
              width: constraints.maxWidth / 2,
              // When using the PrimaryScrollController and a Scrollbar
              // together, only one ScrollPosition can be attached to the
              // PrimaryScrollController at a time. Providing a
              // unique scroll controller to this scroll view prevents it
              // from attaching to the PrimaryScrollController.
              child: Scrollbar(
                thumbVisibility: true,
                controller: _controller,
                child: ListView.builder(
                  controller: _controller,
                  itemCount: 100,
                  itemBuilder: (BuildContext context, int index) {
                    return Padding(
                      padding: const EdgeInsets.all(8.0),
                      child: Text('Scrollable 1 : Index $index'),
                    );
                  },
                ),
              ),
            ),
            SizedBox(
              width: constraints.maxWidth / 2,
              // This vertical scroll view has primary set to true, so it is
              // using the PrimaryScrollController. On mobile platforms, the
              // PrimaryScrollController automatically attaches to vertical
              // ScrollViews, unlike on Desktop platforms, where the primary
              // parameter is required.
              child: Scrollbar(
                thumbVisibility: true,
                child: ListView.builder(
                  primary: true,
                  itemCount: 100,
                  itemBuilder: (BuildContext context, int index) {
                    return Container(
                      height: 50,
                      color: index.isEven ? Colors.amberAccent : Colors.blueAccent,
                      child: Padding(
                        padding: const EdgeInsets.all(8.0),
                        child: Text('Scrollable 2 : Index $index'),
                      ),
                    );
                  },
                ),
              ),
            ),
          ],
        );
      },
    );
  }
}
