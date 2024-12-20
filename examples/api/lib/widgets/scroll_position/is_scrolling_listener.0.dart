// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';

/// Flutter code sample for [ScrollPosition.isScrollingNotifier].
void main() {
  runApp(const IsScrollingListenerApp());
}

class IsScrollingListenerApp extends StatelessWidget {
  const IsScrollingListenerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: IsScrollingListenerExample());
  }
}

class IsScrollingListenerExample extends StatefulWidget {
  const IsScrollingListenerExample({super.key});

  @override
  State<IsScrollingListenerExample> createState() => _IsScrollingListenerExampleState();
}

class _IsScrollingListenerExampleState extends State<IsScrollingListenerExample> {
  static const int itemCount = 25;
  static const double itemExtent = 100;

  late final ScrollController scrollController;
  late double lastScrollOffset;
  bool isScrolling = false;

  @override
  void initState() {
    scrollController = ScrollController(
      onAttach: (ScrollPosition position) {
        position.isScrollingNotifier.addListener(handleScrollChange);
      },
      onDetach: (ScrollPosition position) {
        position.isScrollingNotifier.removeListener(handleScrollChange);
      },
    );
    super.initState();
  }

  @override
  void dispose() {
    scrollController.dispose();
    super.dispose();
  }

  // After an interactive scroll "ends", auto-scroll so that last item in the
  // viewport is completely visible. To accommodate mouse-wheel scrolls, other small
  // adjustments, and scrolling to the top, scrolls that put the scroll offset at
  // zero or change the scroll offset by less than itemExtent don't trigger
  // an auto-scroll.
  void handleScrollChange() {
    final bool isScrollingNow = scrollController.position.isScrollingNotifier.value;
    if (isScrolling == isScrollingNow) {
      return;
    }
    isScrolling = isScrollingNow;
    if (isScrolling) {
      // scroll-start
      lastScrollOffset = scrollController.position.pixels;
    } else {
      // scroll-end
      final ScrollPosition p = scrollController.position;
      final int lastIndex = ((p.extentBefore + p.extentInside) ~/ itemExtent).clamp(
        0,
        itemCount - 1,
      );
      final double alignedScrollOffset = itemExtent * (lastIndex + 1) - p.extentInside;
      final double scrollOffset = scrollController.position.pixels;
      if (scrollOffset > 0 && (scrollOffset - lastScrollOffset).abs() > itemExtent) {
        SchedulerBinding.instance.addPostFrameCallback((Duration duration) {
          scrollController.animateTo(
            alignedScrollOffset,
            duration: const Duration(milliseconds: 400),
            curve: Curves.fastOutSlowIn,
          );
        });
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        child: Scrollbar(
          controller: scrollController,
          thumbVisibility: true,
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 8),
            child: CustomScrollView(
              controller: scrollController,
              slivers: <Widget>[
                SliverFixedExtentList(
                  itemExtent: itemExtent,
                  delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
                    return Item(
                      title: 'Item $index',
                      color: Color.lerp(Colors.red, Colors.blue, index / itemCount)!,
                    );
                  }, childCount: itemCount),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class Item extends StatelessWidget {
  const Item({super.key, required this.title, required this.color});

  final String title;
  final Color color;

  @override
  Widget build(BuildContext context) {
    return Card(color: color, child: ListTile(textColor: Colors.white, title: Text(title)));
  }
}
