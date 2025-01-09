// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  runApp(const SliverResizingHeaderApp());
}

class SliverResizingHeaderApp extends StatelessWidget {
  const SliverResizingHeaderApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: ResizingHeaderExample());
  }
}

class ResizingHeaderExample extends StatefulWidget {
  const ResizingHeaderExample({super.key});

  @override
  State<ResizingHeaderExample> createState() => _ResizingHeaderExampleState();
}

class _ResizingHeaderExampleState extends State<ResizingHeaderExample> {
  late final ScrollController scrollController;

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

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(4),
          child: Scrollbar(
            controller: scrollController,
            child: CustomScrollView(
              controller: scrollController,
              slivers: const <Widget>[
                SliverResizingHeader(
                  minExtentPrototype: ListHeader(text: 'One'),
                  maxExtentPrototype: ListHeader(text: 'One\nTwo\nThree'),
                  child: ListHeader(text: 'SliverResizingHeader\nWith Two Optional\nLines of Text'),
                ),
                ItemList(),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

// A widget that displays its text within a thick rounded rectangle border
class ListHeader extends StatelessWidget {
  const ListHeader({super.key, required this.text});

  final String text;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final ColorScheme colorScheme = theme.colorScheme;

    return Container(
      color: colorScheme.surface,
      padding: const EdgeInsets.symmetric(horizontal: 4),
      child: Material(
        color: colorScheme.primaryContainer,
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(8),
          side: BorderSide(width: 7, color: colorScheme.outline),
        ),
        child: Container(
          alignment: Alignment.center,
          padding: const EdgeInsets.symmetric(vertical: 16),
          child: Text(
            text,
            textAlign: TextAlign.center,
            style: theme.textTheme.headlineMedium!.copyWith(color: colorScheme.onPrimaryContainer),
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
          child: ListTile(textColor: colorScheme.secondary, title: Text('Item $index')),
        );
      }, childCount: itemCount),
    );
  }
}
