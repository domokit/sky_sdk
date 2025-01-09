// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SliverAnimatedGrid].

void main() => runApp(const SliverAnimatedGridSample());

class SliverAnimatedGridSample extends StatefulWidget {
  const SliverAnimatedGridSample({super.key});

  @override
  State<SliverAnimatedGridSample> createState() => _SliverAnimatedGridSampleState();
}

class _SliverAnimatedGridSampleState extends State<SliverAnimatedGridSample> {
  final GlobalKey<SliverAnimatedGridState> _listKey = GlobalKey<SliverAnimatedGridState>();
  final GlobalKey<ScaffoldState> _scaffoldKey = GlobalKey<ScaffoldState>();
  final GlobalKey<ScaffoldMessengerState> _scaffoldMessengerKey =
      GlobalKey<ScaffoldMessengerState>();
  late ListModel<int> _list;
  int? _selectedItem;
  late int _nextItem; // The next item inserted when the user presses the '+' button.

  @override
  void initState() {
    super.initState();
    _list = ListModel<int>(
      listKey: _listKey,
      initialItems: <int>[0, 1, 2, 3, 4, 5],
      removedItemBuilder: _buildRemovedItem,
    );
    _nextItem = 6;
  }

  // Used to build list items that haven't been removed.
  Widget _buildItem(BuildContext context, int index, Animation<double> animation) {
    return CardItem(
      animation: animation,
      item: _list[index],
      selected: _selectedItem == _list[index],
      onTap: () {
        setState(() {
          _selectedItem = _selectedItem == _list[index] ? null : _list[index];
        });
      },
    );
  }

  // Used to build an item after it has been removed from the list. This
  // method is needed because a removed item remains visible until its
  // animation has completed (even though it's gone as far this ListModel is
  // concerned). The widget will be used by the
  // [AnimatedGridState.removeItem] method's
  // [AnimatedGridRemovedItemBuilder] parameter.
  Widget _buildRemovedItem(int item, BuildContext context, Animation<double> animation) {
    return CardItem(animation: animation, removing: true, item: item);
  }

  // Insert the "next item" into the list model.
  void _insert() {
    final int index = _selectedItem == null ? _list.length : _list.indexOf(_selectedItem!);
    _list.insert(index, _nextItem++);
  }

  // Remove the selected item from the list model.
  void _remove() {
    if (_selectedItem != null) {
      _list.removeAt(_list.indexOf(_selectedItem!));
    } else {
      _list.removeAt(_list.length - 1);
    }
    setState(() {
      _selectedItem = null;
    });
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      scaffoldMessengerKey: _scaffoldMessengerKey,
      debugShowCheckedModeBanner: false,
      home: Scaffold(
        key: _scaffoldKey,
        body: CustomScrollView(
          slivers: <Widget>[
            SliverAppBar(
              title: const Text('SliverAnimatedGrid', style: TextStyle(fontSize: 30)),
              expandedHeight: 60,
              centerTitle: true,
              backgroundColor: Colors.amber[900],
              leading: IconButton(
                icon: const Icon(Icons.remove_circle),
                onPressed: _remove,
                tooltip: 'Remove the selected item, or the last item if none selected.',
                iconSize: 32,
              ),
              actions: <Widget>[
                IconButton(
                  icon: const Icon(Icons.add_circle),
                  onPressed: _insert,
                  tooltip: 'Insert a new item.',
                  iconSize: 32,
                ),
              ],
            ),
            SliverAnimatedGrid(
              key: _listKey,
              initialItemCount: _list.length,
              gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
                maxCrossAxisExtent: 100.0,
                mainAxisSpacing: 10.0,
                crossAxisSpacing: 10.0,
              ),
              itemBuilder: _buildItem,
            ),
          ],
        ),
      ),
    );
  }
}

typedef RemovedItemBuilder =
    Widget Function(int item, BuildContext context, Animation<double> animation);

// Keeps a Dart [List] in sync with an [AnimatedGrid].
//
// The [insert] and [removeAt] methods apply to both the internal list and
// the animated list that belongs to [listKey].
//
// This class only exposes as much of the Dart List API as is needed by the
// sample app. More list methods are easily added, however methods that
// mutate the list must make the same changes to the animated list in terms
// of [AnimatedGridState.insertItem] and [AnimatedGrid.removeItem].
class ListModel<E> {
  ListModel({required this.listKey, required this.removedItemBuilder, Iterable<E>? initialItems})
    : _items = List<E>.from(initialItems ?? <E>[]);

  final GlobalKey<SliverAnimatedGridState> listKey;
  final RemovedItemBuilder removedItemBuilder;
  final List<E> _items;

  SliverAnimatedGridState get _animatedGrid => listKey.currentState!;

  void insert(int index, E item) {
    _items.insert(index, item);
    _animatedGrid.insertItem(index);
  }

  E removeAt(int index) {
    final E removedItem = _items.removeAt(index);
    if (removedItem != null) {
      _animatedGrid.removeItem(
        index,
        (BuildContext context, Animation<double> animation) =>
            removedItemBuilder(index, context, animation),
      );
    }
    return removedItem;
  }

  int get length => _items.length;

  E operator [](int index) => _items[index];

  int indexOf(E item) => _items.indexOf(item);
}

// Displays its integer item as 'Item N' on a Card whose color is based on
// the item's value.
//
// The card turns gray when [selected] is true. This widget's height
// is based on the [animation] parameter. It varies as the animation value
// transitions from 0.0 to 1.0.
class CardItem extends StatelessWidget {
  const CardItem({
    super.key,
    this.onTap,
    this.selected = false,
    this.removing = false,
    required this.animation,
    required this.item,
  }) : assert(item >= 0);

  final Animation<double> animation;
  final VoidCallback? onTap;
  final int item;
  final bool selected;
  final bool removing;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(left: 2.0, right: 2.0, top: 2.0),
      child: ScaleTransition(
        scale: CurvedAnimation(
          parent: animation,
          curve: removing ? Curves.easeInOut : Curves.bounceOut,
        ),
        child: GestureDetector(
          onTap: onTap,
          child: SizedBox(
            height: 80.0,
            child: Card(
              color: selected ? Colors.black12 : Colors.primaries[item % Colors.primaries.length],
              child: Center(
                child: Text(
                  (item + 1).toString(),
                  style: Theme.of(context).textTheme.headlineMedium,
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
