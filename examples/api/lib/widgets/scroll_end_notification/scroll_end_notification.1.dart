// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';

void main() {
  runApp(const SliverAutoScrollExampleApp());
}

class SliverAutoScrollExampleApp extends StatelessWidget {
  const SliverAutoScrollExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: SliverAutoScrollExample());
  }
}

class SliverAutoScrollExample extends StatefulWidget {
  const SliverAutoScrollExample({super.key});

  @override
  State<SliverAutoScrollExample> createState() => _SliverAutoScrollExampleState();
}

class _SliverAutoScrollExampleState extends State<SliverAutoScrollExample> {
  final GlobalKey alignedItemKey = GlobalKey();
  late final ScrollController scrollController;
  late double lastScrollOffset;

  @override
  void initState() {
    super.initState();
    scrollController = ScrollController();
  }

  @override
  void dispose() {
    scrollController.dispose();
    super.dispose();
  }

  void autoScrollTo(double offset) {
    scrollController.position.animateTo(
      offset,
      duration: const Duration(milliseconds: 300),
      curve: Curves.easeInOut,
    );
  }

  // After an interactive scroll ends, if the alignedItem is partially visible
  // at the top or bottom of the viewport, then auto-scroll so that it's
  // completely visible. To accommodate mouse-wheel scrolls and other small
  // adjustments, scrolls that change the scroll offset by less than
  // the alignedItem's extent don't trigger an auto-scroll.
  void maybeAutoScrollAlignedItem(RenderSliver alignedItem) {
    final SliverConstraints constraints = alignedItem.constraints;
    final SliverGeometry geometry = alignedItem.geometry!;
    final double sliverOffset = constraints.scrollOffset;

    if ((scrollController.offset - lastScrollOffset).abs() <= geometry.maxPaintExtent) {
      // Ignore scrolls that are smaller than the aligned item's extent.
      return;
    }
    final double overflow = geometry.maxPaintExtent - geometry.paintExtent;
    if (overflow > 0 && overflow < geometry.scrollExtent) {
      // indicates partial visibility
      if (sliverOffset > 0) {
        autoScrollTo(constraints.precedingScrollExtent); // top
      } else if (sliverOffset == 0) {
        autoScrollTo(scrollController.offset + overflow); // bottom
      }
    }
  }

  // Calls maybeAutoScrollAlignedItem in a post-frame callback so that
  // auto-scrolls are triggered _after_ the current scroll activity
  // has completed. Otherwise auto-scrolling would be a no-op.
  bool handleScrollNotification(ScrollNotification notification) {
    if (notification is ScrollStartNotification) {
      lastScrollOffset = scrollController.offset;
    }
    if (notification is ScrollEndNotification) {
      SchedulerBinding.instance.addPostFrameCallback((Duration duration) {
        final RenderSliver? sliver =
            alignedItemKey.currentContext?.findAncestorRenderObjectOfType<RenderSliver>();
        if (sliver != null && sliver.geometry != null) {
          maybeAutoScrollAlignedItem(sliver);
        }
      });
    }
    return false;
  }

  @override
  Widget build(BuildContext context) {
    const EdgeInsets horizontalPadding = EdgeInsets.symmetric(horizontal: 8);

    return Scaffold(
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 4),
          child: NotificationListener<ScrollNotification>(
            onNotification: handleScrollNotification,
            child: Scrollbar(
              controller: scrollController,
              thumbVisibility: true,
              child: CustomScrollView(
                controller: scrollController,
                slivers: <Widget>[
                  const SliverPadding(padding: horizontalPadding, sliver: ItemList(itemCount: 15)),
                  SliverPadding(
                    padding: horizontalPadding,
                    sliver: BigOrangeSliver(sliverChildKey: alignedItemKey),
                  ),
                  const SliverPadding(padding: horizontalPadding, sliver: ItemList(itemCount: 25)),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }
}

// A big list item that's easy to spot. The provided key is assigned to
// the aligned sliver's child so that we can find the this item's RenderSliver
// later with BuildContext.findAncestorRenderObjectOfType.
class BigOrangeSliver extends StatelessWidget {
  const BigOrangeSliver({super.key, required this.sliverChildKey});

  final Key sliverChildKey;

  @override
  Widget build(BuildContext context) {
    return SliverToBoxAdapter(
      child: Card(
        key: sliverChildKey,
        color: Colors.orange,
        child: const SizedBox(
          width: 300,
          child: ListTile(
            textColor: Colors.white,
            title: Padding(
              padding: EdgeInsets.symmetric(vertical: 32),
              child: Text('Aligned Item'),
            ),
          ),
        ),
      ),
    );
  }
}

// A placeholder SliverList of 50 items.
class ItemList extends StatelessWidget {
  const ItemList({super.key, this.itemCount = 50});

  final int itemCount;

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    return SliverList(
      delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
        return Card(
          color: colorScheme.onSecondary,
          child: SizedBox(
            width: 100,
            child: ListTile(
              textColor: colorScheme.secondary,
              title: Text('Item $index.$itemCount'),
            ),
          ),
        );
      }, childCount: itemCount),
    );
  }
}
