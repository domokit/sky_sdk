// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [AnimatedGrid].

void main() {
  runApp(const AnimatedGridSample());
}

class AnimatedGridSample extends StatefulWidget {
  const AnimatedGridSample({super.key});

  @override
  State<AnimatedGridSample> createState() => _AnimatedGridSampleState();
}

class _AnimatedGridSampleState extends State<AnimatedGridSample> {
  final GlobalKey<AnimatedGridState> _gridKey = GlobalKey<AnimatedGridState>();
  late ListModel<int> _list;
  int? _selectedItem;
  late int _nextItem; // The next item inserted when the user presses the '+' button.

  @override
  void initState() {
    super.initState();
    _list = ListModel<int>(
      listKey: _gridKey,
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

  // Used to build an item after it has been removed from the list. This method
  // is needed because a removed item remains visible until its animation has
  // completed (even though it's gone as far as this ListModel is concerned).
  // The widget will be used by the [AnimatedGridState.removeItem] method's
  // [AnimatedGridRemovedItemBuilder] parameter.
  Widget _buildRemovedItem(int item, BuildContext context, Animation<double> animation) {
    return CardItem(
      animation: animation,
      item: item,
      removing: true,
      // No gesture detector here: we don't want removed items to be interactive.
    );
  }

  // Insert the "next item" into the list model.
  void _insert() {
    final int index = _selectedItem == null ? _list.length : _list.indexOf(_selectedItem!);
    setState(() {
      _list.insert(index, _nextItem++);
    });
  }

  // Remove the selected item from the list model.
  void _remove() {
    if (_selectedItem != null) {
      setState(() {
        _list.removeAt(_list.indexOf(_selectedItem!));
        _selectedItem = null;
      });
    } else if (_list.length > 0) {
      setState(() {
        _list.removeAt(_list.length - 1);
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: Scaffold(
        appBar: AppBar(
          title: const Text('AnimatedGrid', style: TextStyle(fontSize: 30)),
          centerTitle: true,
          leading: IconButton(
            icon: const Icon(Icons.remove_circle),
            iconSize: 32,
            onPressed: (_list.length > 0) ? _remove : null,
            tooltip: 'remove the selected item',
          ),
          actions: <Widget>[
            IconButton(
              icon: const Icon(Icons.add_circle),
              iconSize: 32,
              onPressed: _insert,
              tooltip: 'insert a new item',
            ),
          ],
        ),
        body: Padding(
          padding: const EdgeInsets.all(16.0),
          child: AnimatedGrid(
            key: _gridKey,
            gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
              maxCrossAxisExtent: 100.0,
              mainAxisSpacing: 10.0,
              crossAxisSpacing: 10.0,
            ),
            initialItemCount: _list.length,
            itemBuilder: _buildItem,
          ),
        ),
      ),
    );
  }
}

typedef RemovedItemBuilder<T> =
    Widget Function(T item, BuildContext context, Animation<double> animation);

/// Keeps a Dart [List] in sync with an [AnimatedGrid].
///
/// The [insert] and [removeAt] methods apply to both the internal list and
/// the animated list that belongs to [listKey].
///
/// This class only exposes as much of the Dart List API as is needed by the
/// sample app. More list methods are easily added, however methods that
/// mutate the list must make the same changes to the animated list in terms
/// of [AnimatedGridState.insertItem] and [AnimatedGridState.removeItem].
class ListModel<E> {
  ListModel({required this.listKey, required this.removedItemBuilder, Iterable<E>? initialItems})
    : _items = List<E>.from(initialItems ?? <E>[]);

  final GlobalKey<AnimatedGridState> listKey;
  final RemovedItemBuilder<E> removedItemBuilder;
  final List<E> _items;

  AnimatedGridState? get _animatedGrid => listKey.currentState;

  void insert(int index, E item) {
    _items.insert(index, item);
    _animatedGrid!.insertItem(index, duration: const Duration(milliseconds: 500));
  }

  E removeAt(int index) {
    final E removedItem = _items.removeAt(index);
    if (removedItem != null) {
      _animatedGrid!.removeItem(index, (BuildContext context, Animation<double> animation) {
        return removedItemBuilder(removedItem, context, animation);
      });
    }
    return removedItem;
  }

  int get length => _items.length;

  E operator [](int index) => _items[index];

  int indexOf(E item) => _items.indexOf(item);
}

/// Displays its integer item as 'item N' on a Card whose color is based on
/// the item's value.
///
/// The text is displayed in bright green if [selected] is
/// true. This widget's height is based on the [animation] parameter, it
/// varies from 0 to 128 as the animation varies from 0.0 to 1.0.
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
    TextStyle textStyle = Theme.of(context).textTheme.headlineMedium!;
    if (selected) {
      textStyle = textStyle.copyWith(color: Colors.lightGreenAccent[400]);
    }
    return Padding(
      padding: const EdgeInsets.all(2.0),
      child: ScaleTransition(
        scale: CurvedAnimation(
          parent: animation,
          curve: removing ? Curves.easeInOut : Curves.bounceOut,
        ),
        child: GestureDetector(
          behavior: HitTestBehavior.opaque,
          onTap: onTap,
          child: SizedBox(
            height: 80.0,
            child: Card(
              color: Colors.primaries[item % Colors.primaries.length],
              child: Center(child: Text('${item + 1}', style: textStyle)),
            ),
          ),
        ),
      ),
    );
  }
}
