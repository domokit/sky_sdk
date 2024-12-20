// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  runApp(const SliverFloatingHeaderApp());
}

class SliverFloatingHeaderApp extends StatelessWidget {
  const SliverFloatingHeaderApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: FloatingHeaderExample());
  }
}

class FloatingHeaderExample extends StatefulWidget {
  const FloatingHeaderExample({super.key});

  @override
  State<FloatingHeaderExample> createState() => _FloatingHeaderExampleState();
}

class _FloatingHeaderExampleState extends State<FloatingHeaderExample> {
  @override
  Widget build(BuildContext context) {
    return const Scaffold(
      body: SafeArea(
        child: Padding(
          padding: EdgeInsets.all(4),
          child: CustomScrollView(
            slivers: <Widget>[
              SliverFloatingHeader(
                child: ListHeader(
                  text:
                      'SliverFloatingHeader\nScroll down a little to show\nScroll up a little to hide',
                ),
              ),
              ItemList(),
            ],
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
