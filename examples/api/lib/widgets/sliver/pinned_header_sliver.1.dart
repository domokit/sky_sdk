// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

void main() {
  runApp(const SettingsAppBarApp());
}

class SettingsAppBarApp extends StatelessWidget {
  const SettingsAppBarApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: SettingsAppBarExample());
  }
}

class SettingsAppBarExample extends StatefulWidget {
  const SettingsAppBarExample({super.key});

  @override
  State<SettingsAppBarExample> createState() => _SettingsAppBarExampleState();
}

class _SettingsAppBarExampleState extends State<SettingsAppBarExample> {
  final GlobalKey headerSliverKey = GlobalKey();
  final GlobalKey titleSliverKey = GlobalKey();
  late final ScrollController scrollController;
  double headerOpacity = 0;

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

  // The key must be for a widget _below_ a RenderSliver so that
  // findAncestorRenderObjectOfType can find the RenderSliver when it searches
  // the key widget's renderer ancestors.
  RenderSliver? keyToSliver(GlobalKey key) =>
      key.currentContext?.findAncestorRenderObjectOfType<RenderSliver>();

  // Each time the app's list scrolls: if the Title sliver has scrolled completely behind
  // the (pinned) header sliver, then change the header's opacity from 0 to 1.
  //
  // The header RenderSliver's SliverConstraints.scrollOffset is the distance
  // above the top of the viewport where the top of header sliver would appear
  // if it were laid out normally. Since it's a pinned sliver, it's unconditionally
  // painted at the top of the viewport, even though its scrollOffset constraint
  // increases as the user scrolls upwards. The "Settings" title RenderSliver's
  // scrollExtent is the vertical space it wants to occupy. It doesn't change as
  // the user scrolls.
  bool handleScrollNotification(ScrollNotification notification) {
    final RenderSliver? headerSliver = keyToSliver(headerSliverKey);
    final RenderSliver? titleSliver = keyToSliver(titleSliverKey);
    if (headerSliver != null && titleSliver != null && titleSliver.geometry != null) {
      final double opacity =
          headerSliver.constraints.scrollOffset > titleSliver.geometry!.scrollExtent ? 1 : 0;
      if (opacity != headerOpacity) {
        setState(() {
          headerOpacity = opacity;
        });
      }
    }
    return false;
  }

  @override
  Widget build(BuildContext context) {
    const EdgeInsets horizontalPadding = EdgeInsets.symmetric(horizontal: 8);
    final ThemeData theme = Theme.of(context);
    final TextTheme textTheme = theme.textTheme;
    final ColorScheme colorScheme = theme.colorScheme;

    return Scaffold(
      backgroundColor: colorScheme.surfaceContainer,
      body: SafeArea(
        child: NotificationListener<ScrollNotification>(
          onNotification: handleScrollNotification,
          child: CustomScrollView(
            controller: scrollController,
            slivers: <Widget>[
              PinnedHeaderSliver(
                child: Header(
                  key: headerSliverKey,
                  opacity: headerOpacity,
                  child: Text('Settings', style: textTheme.titleMedium),
                ),
              ),
              SliverPadding(
                padding: horizontalPadding,
                sliver: SliverToBoxAdapter(
                  child: TitleItem(
                    key: titleSliverKey,
                    child: Text(
                      'Settings',
                      style: textTheme.titleLarge!.copyWith(
                        fontWeight: FontWeight.bold,
                        fontSize: 32,
                      ),
                    ),
                  ),
                ),
              ),
              const SliverPadding(padding: horizontalPadding, sliver: ItemList()),
            ],
          ),
        ),
      ),
    );
  }
}

// The pinned item at the top of the list. This is an implicitly
// animated widget: when the opacity changes the title and divider
// fade in or out.
class Header extends StatelessWidget {
  const Header({super.key, required this.opacity, required this.child});

  final double opacity;
  final Widget child;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final ColorScheme colorScheme = theme.colorScheme;

    return AnimatedContainer(
      duration: const Duration(milliseconds: 300),
      padding: const EdgeInsets.symmetric(vertical: 12),
      decoration: ShapeDecoration(
        color: opacity == 0 ? colorScheme.surfaceContainer : colorScheme.surfaceContainerLowest,
        shape: LinearBorder.bottom(
          side: BorderSide(
            color:
                opacity == 0 ? colorScheme.surfaceContainer : colorScheme.surfaceContainerHighest,
          ),
        ),
      ),
      alignment: Alignment.center,
      child: AnimatedOpacity(
        opacity: opacity,
        duration: const Duration(milliseconds: 300),
        child: child,
      ),
    );
  }
}

// The second item in the list. It scrolls normally. When it has scrolled
// completely out of view behind the first, pinned, Header item, the Header
// fades in.
class TitleItem extends StatelessWidget {
  const TitleItem({super.key, required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) {
    return Container(
      alignment: AlignmentDirectional.bottomStart,
      padding: const EdgeInsets.symmetric(vertical: 8),
      child: child,
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
          child: ListTile(textColor: colorScheme.secondary, title: Text('Item $index')),
        );
      }, childCount: itemCount),
    );
  }
}
