// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ScrollNotificationObserver].

void main() => runApp(const ScrollNotificationObserverApp());

class ScrollNotificationObserverApp extends StatelessWidget {
  const ScrollNotificationObserverApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(useMaterial3: true),
      // The Scaffold widget contains a [ScrollNotificationObserver].
      // This is used by [AppBar] for its scrolled under behavior.
      //
      // We can use [ScrollNotificationObserver.maybeOf] to get the
      // state of this [ScrollNotificationObserver] from descendants
      // of the Scaffold widget.
      //
      // If you're not using a [Scaffold] widget, you can create a  [ScrollNotificationObserver]
      // to notify its descendants of scroll notifications by adding it to the subtree.
      home: Scaffold(
        appBar: AppBar(title: const Text('ScrollNotificationObserver Sample')),
        body: const ScrollNotificationObserverExample(),
      ),
    );
  }
}

class ScrollNotificationObserverExample extends StatefulWidget {
  const ScrollNotificationObserverExample({super.key});

  @override
  State<ScrollNotificationObserverExample> createState() =>
      _ScrollNotificationObserverExampleState();
}

class _ScrollNotificationObserverExampleState extends State<ScrollNotificationObserverExample> {
  ScrollNotificationObserverState? _scrollNotificationObserver;
  ScrollController controller = ScrollController();
  bool _scrolledDown = false;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    // Remove any previous listener.
    _scrollNotificationObserver?.removeListener(_handleScrollNotification);
    // Get the ScrollNotificationObserverState from the Scaffold widget.
    _scrollNotificationObserver = ScrollNotificationObserver.maybeOf(context);
    // Add a new listener.
    _scrollNotificationObserver?.addListener(_handleScrollNotification);
  }

  @override
  void dispose() {
    if (_scrollNotificationObserver != null) {
      _scrollNotificationObserver!.removeListener(_handleScrollNotification);
      _scrollNotificationObserver = null;
    }
    controller.dispose();
    super.dispose();
  }

  void _handleScrollNotification(ScrollNotification notification) {
    // Check if the notification is a scroll update notification and if the
    // `notification.depth` is 0. This way we only listen to the scroll
    // notifications from the closest scrollable, instead of those that may be nested.
    if (notification is ScrollUpdateNotification &&
        defaultScrollNotificationPredicate(notification)) {
      final ScrollMetrics metrics = notification.metrics;
      // Check if the user scrolled down.
      if (_scrolledDown != metrics.extentBefore > 0) {
        setState(() {
          _scrolledDown = metrics.extentBefore > 0;
        });
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: <Widget>[
        SampleList(controller: controller),
        // Show the button only if the user scrolled down.
        if (_scrolledDown)
          Positioned(
            right: 25,
            bottom: 20,
            child: Center(
              child: GestureDetector(
                onTap: () {
                  // Scroll to the top when the user taps the button.
                  controller.animateTo(
                    0,
                    duration: const Duration(milliseconds: 200),
                    curve: Curves.fastOutSlowIn,
                  );
                },
                child: const Card(
                  child: Padding(
                    padding: EdgeInsets.all(8.0),
                    child: Column(
                      children: <Widget>[Icon(Icons.arrow_upward_rounded), Text('Scroll to top')],
                    ),
                  ),
                ),
              ),
            ),
          ),
      ],
    );
  }
}

class SampleList extends StatelessWidget {
  const SampleList({super.key, required this.controller});

  final ScrollController controller;

  @override
  Widget build(BuildContext context) {
    return ListView.builder(
      controller: controller,
      itemCount: 30,
      itemBuilder: (BuildContext context, int index) {
        return ListTile(title: Text('Item $index'));
      },
    );
  }
}
