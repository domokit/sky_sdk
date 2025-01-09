// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ScrollController].

void main() => runApp(const ScrollControllerDemo());

class ScrollControllerDemo extends StatefulWidget {
  const ScrollControllerDemo({super.key});

  @override
  State<ScrollControllerDemo> createState() => _ScrollControllerDemoState();
}

class _ScrollControllerDemoState extends State<ScrollControllerDemo> {
  late final ScrollController _controller;
  bool isScrolling = false;

  void _handleScrollChange() {
    if (isScrolling != _controller.position.isScrollingNotifier.value) {
      setState(() {
        isScrolling = _controller.position.isScrollingNotifier.value;
      });
    }
  }

  void _handlePositionAttach(ScrollPosition position) {
    // From here, add a listener to the given ScrollPosition.
    // Here the isScrollingNotifier will be used to inform when scrolling starts
    // and stops and change the AppBar's color in response.
    position.isScrollingNotifier.addListener(_handleScrollChange);
  }

  void _handlePositionDetach(ScrollPosition position) {
    // From here, add a listener to the given ScrollPosition.
    // Here the isScrollingNotifier will be used to inform when scrolling starts
    // and stops and change the AppBar's color in response.
    position.isScrollingNotifier.removeListener(_handleScrollChange);
  }

  @override
  void initState() {
    _controller = ScrollController(
      // These methods will be called in response to a scroll position
      // being attached to or detached from this ScrollController. This happens
      // when the Scrollable is built.
      onAttach: _handlePositionAttach,
      onDetach: _handlePositionDetach,
    );
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(
          title: Text(isScrolling ? 'Scrolling' : 'Not Scrolling'),
          backgroundColor:
              isScrolling
                  ? Colors.green[800]!.withOpacity(.85)
                  : Colors.redAccent[700]!.withOpacity(.85),
        ),
        // ListView.builder works very similarly to this example with
        // CustomScrollView & SliverList.
        body: CustomScrollView(
          // Provide the scroll controller to the scroll view.
          controller: _controller,
          slivers: <Widget>[
            SliverList.builder(
              itemCount: 50,
              itemBuilder: (_, int index) {
                return Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: Center(
                    child: DecoratedBox(
                      decoration: BoxDecoration(
                        color: Colors.blueGrey[50],
                        boxShadow: const <BoxShadow>[
                          BoxShadow(color: Colors.black12, offset: Offset(5, 5), blurRadius: 5),
                        ],
                        borderRadius: const BorderRadius.all(Radius.circular(10)),
                      ),
                      child: Padding(
                        padding: const EdgeInsets.symmetric(vertical: 12.0, horizontal: 20.0),
                        child: Text('Item $index'),
                      ),
                    ),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
  }
}
