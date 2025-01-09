// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:collection/collection.dart' show lowerBound;

import 'package:flutter/material.dart';
import 'package:flutter/semantics.dart';

import '../../gallery/demo.dart';

enum LeaveBehindDemoAction { reset, horizontalSwipe, leftSwipe, rightSwipe, confirmDismiss }

class LeaveBehindItem implements Comparable<LeaveBehindItem> {
  LeaveBehindItem({this.index, this.name, this.subject, this.body});

  LeaveBehindItem.from(LeaveBehindItem item)
    : index = item.index,
      name = item.name,
      subject = item.subject,
      body = item.body;

  final int? index;
  final String? name;
  final String? subject;
  final String? body;

  @override
  int compareTo(LeaveBehindItem other) => index!.compareTo(other.index!);
}

class LeaveBehindDemo extends StatefulWidget {
  const LeaveBehindDemo({super.key});

  static const String routeName = '/material/leave-behind';

  @override
  LeaveBehindDemoState createState() => LeaveBehindDemoState();
}

class LeaveBehindDemoState extends State<LeaveBehindDemo> {
  DismissDirection _dismissDirection = DismissDirection.horizontal;
  bool _confirmDismiss = true;
  late List<LeaveBehindItem> leaveBehindItems;

  void initListItems() {
    leaveBehindItems = List<LeaveBehindItem>.generate(16, (int index) {
      return LeaveBehindItem(
        index: index,
        name: 'Item $index Sender',
        subject: 'Subject: $index',
        body: "[$index] first line of the message's body...",
      );
    });
  }

  @override
  void initState() {
    super.initState();
    initListItems();
  }

  void handleDemoAction(LeaveBehindDemoAction action) {
    setState(() {
      switch (action) {
        case LeaveBehindDemoAction.reset:
          initListItems();
        case LeaveBehindDemoAction.horizontalSwipe:
          _dismissDirection = DismissDirection.horizontal;
        case LeaveBehindDemoAction.leftSwipe:
          _dismissDirection = DismissDirection.endToStart;
        case LeaveBehindDemoAction.rightSwipe:
          _dismissDirection = DismissDirection.startToEnd;
        case LeaveBehindDemoAction.confirmDismiss:
          _confirmDismiss = !_confirmDismiss;
      }
    });
  }

  void handleUndo(LeaveBehindItem item) {
    final int insertionIndex = lowerBound(leaveBehindItems, item);
    setState(() {
      leaveBehindItems.insert(insertionIndex, item);
    });
  }

  void _handleArchive(LeaveBehindItem item) {
    setState(() {
      leaveBehindItems.remove(item);
    });
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('You archived item ${item.index}'),
        action: SnackBarAction(
          label: 'UNDO',
          onPressed: () {
            handleUndo(item);
          },
        ),
      ),
    );
  }

  void _handleDelete(LeaveBehindItem item) {
    setState(() {
      leaveBehindItems.remove(item);
    });
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('You deleted item ${item.index}'),
        action: SnackBarAction(
          label: 'UNDO',
          onPressed: () {
            handleUndo(item);
          },
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    Widget body;
    if (leaveBehindItems.isEmpty) {
      body = Center(
        child: ElevatedButton(
          onPressed: () => handleDemoAction(LeaveBehindDemoAction.reset),
          child: const Text('Reset the list'),
        ),
      );
    } else {
      body = Scrollbar(
        child: ListView(
          primary: true,
          children:
              leaveBehindItems.map<Widget>((LeaveBehindItem item) {
                return _LeaveBehindListItem(
                  confirmDismiss: _confirmDismiss,
                  item: item,
                  onArchive: _handleArchive,
                  onDelete: _handleDelete,
                  dismissDirection: _dismissDirection,
                );
              }).toList(),
        ),
      );
    }

    return Scaffold(
      appBar: AppBar(
        title: const Text('Swipe to dismiss'),
        actions: <Widget>[
          MaterialDemoDocumentationButton(LeaveBehindDemo.routeName),
          PopupMenuButton<LeaveBehindDemoAction>(
            onSelected: handleDemoAction,
            itemBuilder:
                (BuildContext context) => <PopupMenuEntry<LeaveBehindDemoAction>>[
                  const PopupMenuItem<LeaveBehindDemoAction>(
                    value: LeaveBehindDemoAction.reset,
                    child: Text('Reset the list'),
                  ),
                  const PopupMenuDivider(),
                  CheckedPopupMenuItem<LeaveBehindDemoAction>(
                    value: LeaveBehindDemoAction.horizontalSwipe,
                    checked: _dismissDirection == DismissDirection.horizontal,
                    child: const Text('Horizontal swipe'),
                  ),
                  CheckedPopupMenuItem<LeaveBehindDemoAction>(
                    value: LeaveBehindDemoAction.leftSwipe,
                    checked: _dismissDirection == DismissDirection.endToStart,
                    child: const Text('Only swipe left'),
                  ),
                  CheckedPopupMenuItem<LeaveBehindDemoAction>(
                    value: LeaveBehindDemoAction.rightSwipe,
                    checked: _dismissDirection == DismissDirection.startToEnd,
                    child: const Text('Only swipe right'),
                  ),
                  CheckedPopupMenuItem<LeaveBehindDemoAction>(
                    value: LeaveBehindDemoAction.confirmDismiss,
                    checked: _confirmDismiss,
                    child: const Text('Confirm dismiss'),
                  ),
                ],
          ),
        ],
      ),
      body: body,
    );
  }
}

class _LeaveBehindListItem extends StatelessWidget {
  const _LeaveBehindListItem({
    required this.item,
    required this.onArchive,
    required this.onDelete,
    required this.dismissDirection,
    required this.confirmDismiss,
  });

  final LeaveBehindItem item;
  final DismissDirection dismissDirection;
  final void Function(LeaveBehindItem) onArchive;
  final void Function(LeaveBehindItem) onDelete;
  final bool confirmDismiss;

  void _handleArchive() {
    onArchive(item);
  }

  void _handleDelete() {
    onDelete(item);
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    return Semantics(
      customSemanticsActions: <CustomSemanticsAction, VoidCallback>{
        const CustomSemanticsAction(label: 'Archive'): _handleArchive,
        const CustomSemanticsAction(label: 'Delete'): _handleDelete,
      },
      child: Dismissible(
        key: ObjectKey(item),
        direction: dismissDirection,
        onDismissed: (DismissDirection direction) {
          if (direction == DismissDirection.endToStart) {
            _handleArchive();
          } else {
            _handleDelete();
          }
        },
        confirmDismiss:
            !confirmDismiss
                ? null
                : (DismissDirection dismissDirection) async {
                  switch (dismissDirection) {
                    case DismissDirection.endToStart:
                      return await _showConfirmationDialog(context, 'archive') ?? false;
                    case DismissDirection.startToEnd:
                      return await _showConfirmationDialog(context, 'delete') ?? false;
                    case DismissDirection.horizontal:
                    case DismissDirection.vertical:
                    case DismissDirection.up:
                    case DismissDirection.down:
                    case DismissDirection.none:
                      assert(false);
                  }
                  return false;
                },
        background: ColoredBox(
          color: theme.primaryColor,
          child: const Center(
            child: ListTile(leading: Icon(Icons.delete, color: Colors.white, size: 36.0)),
          ),
        ),
        secondaryBackground: ColoredBox(
          color: theme.primaryColor,
          child: const Center(
            child: ListTile(trailing: Icon(Icons.archive, color: Colors.white, size: 36.0)),
          ),
        ),
        child: Container(
          decoration: BoxDecoration(
            color: theme.canvasColor,
            border: Border(bottom: BorderSide(color: theme.dividerColor)),
          ),
          child: ListTile(
            title: Text(item.name!),
            subtitle: Text('${item.subject}\n${item.body}'),
            isThreeLine: true,
          ),
        ),
      ),
    );
  }

  Future<bool?> _showConfirmationDialog(BuildContext context, String action) {
    return showDialog<bool>(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: Text('Do you want to $action this item?'),
          actions: <Widget>[
            TextButton(
              child: const Text('Yes'),
              onPressed: () {
                Navigator.pop(context, true); // showDialog() returns true
              },
            ),
            TextButton(
              child: const Text('No'),
              onPressed: () {
                Navigator.pop(context, false); // showDialog() returns false
              },
            ),
          ],
        );
      },
    );
  }
}
