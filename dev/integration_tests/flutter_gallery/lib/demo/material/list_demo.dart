// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

enum _MaterialListType {
  /// A list tile that contains a single line of text.
  oneLine,

  /// A list tile that contains a [CircleAvatar] followed by a single line of text.
  oneLineWithAvatar,

  /// A list tile that contains two lines of text.
  twoLine,

  /// A list tile that contains three lines of text.
  threeLine,
}

class ListDemo extends StatefulWidget {
  const ListDemo({super.key});

  static const String routeName = '/material/list';

  @override
  State<ListDemo> createState() => _ListDemoState();
}

class _ListDemoState extends State<ListDemo> {
  static final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

  PersistentBottomSheetController? _bottomSheet;
  _MaterialListType? _itemType = _MaterialListType.threeLine;
  bool? _dense = false;
  bool? _showAvatars = true;
  bool? _showIcons = false;
  bool? _showDividers = false;
  bool _reverseSort = false;
  List<String> items = <String>[
    'A',
    'B',
    'C',
    'D',
    'E',
    'F',
    'G',
    'H',
    'I',
    'J',
    'K',
    'L',
    'M',
    'N',
  ];

  void changeItemType(_MaterialListType? type) {
    setState(() {
      _itemType = type;
    });
    _bottomSheet?.setState!(() {});
  }

  void _showConfigurationSheet() {
    final PersistentBottomSheetController bottomSheet = scaffoldKey.currentState!.showBottomSheet((
      BuildContext bottomSheetContext,
    ) {
      return Container(
        decoration: const BoxDecoration(border: Border(top: BorderSide(color: Colors.black26))),
        child: ListView(
          shrinkWrap: true,
          primary: false,
          children: <Widget>[
            MergeSemantics(
              child: ListTile(
                dense: true,
                title: const Text('One-line'),
                trailing: Radio<_MaterialListType>(
                  value:
                      _showAvatars!
                          ? _MaterialListType.oneLineWithAvatar
                          : _MaterialListType.oneLine,
                  groupValue: _itemType,
                  onChanged: changeItemType,
                ),
              ),
            ),
            MergeSemantics(
              child: ListTile(
                dense: true,
                title: const Text('Two-line'),
                trailing: Radio<_MaterialListType>(
                  value: _MaterialListType.twoLine,
                  groupValue: _itemType,
                  onChanged: changeItemType,
                ),
              ),
            ),
            MergeSemantics(
              child: ListTile(
                dense: true,
                title: const Text('Three-line'),
                trailing: Radio<_MaterialListType>(
                  value: _MaterialListType.threeLine,
                  groupValue: _itemType,
                  onChanged: changeItemType,
                ),
              ),
            ),
            MergeSemantics(
              child: ListTile(
                dense: true,
                title: const Text('Show avatar'),
                trailing: Checkbox(
                  value: _showAvatars,
                  onChanged: (bool? value) {
                    setState(() {
                      _showAvatars = value;
                    });
                    final StateSetter? bottomSheetSetState = _bottomSheet?.setState;
                    if (bottomSheetSetState != null) {
                      bottomSheetSetState(() {});
                    }
                  },
                ),
              ),
            ),
            MergeSemantics(
              child: ListTile(
                dense: true,
                title: const Text('Show icon'),
                trailing: Checkbox(
                  value: _showIcons,
                  onChanged: (bool? value) {
                    setState(() {
                      _showIcons = value;
                    });
                    final StateSetter? bottomSheetSetState = _bottomSheet?.setState;
                    if (bottomSheetSetState != null) {
                      bottomSheetSetState(() {});
                    }
                  },
                ),
              ),
            ),
            MergeSemantics(
              child: ListTile(
                dense: true,
                title: const Text('Show dividers'),
                trailing: Checkbox(
                  value: _showDividers,
                  onChanged: (bool? value) {
                    setState(() {
                      _showDividers = value;
                    });
                    final StateSetter? bottomSheetSetState = _bottomSheet?.setState;
                    if (bottomSheetSetState != null) {
                      bottomSheetSetState(() {});
                    }
                  },
                ),
              ),
            ),
            MergeSemantics(
              child: ListTile(
                dense: true,
                title: const Text('Dense layout'),
                trailing: Checkbox(
                  value: _dense,
                  onChanged: (bool? value) {
                    setState(() {
                      _dense = value;
                    });
                    final StateSetter? bottomSheetSetState = _bottomSheet?.setState;
                    if (bottomSheetSetState != null) {
                      bottomSheetSetState(() {});
                    }
                  },
                ),
              ),
            ),
          ],
        ),
      );
    });

    setState(() {
      _bottomSheet = bottomSheet;
    });

    _bottomSheet?.closed.whenComplete(() {
      if (mounted) {
        setState(() {
          _bottomSheet = null;
        });
      }
    });
  }

  Widget buildListTile(BuildContext context, String item) {
    final String? subtitle = switch (_itemType) {
      _MaterialListType.oneLine || _MaterialListType.oneLineWithAvatar || null => null,
      _MaterialListType.twoLine => 'Additional item information.',
      _MaterialListType.threeLine =>
        'Even more additional list item information appears on line three.',
    };
    return MergeSemantics(
      child: ListTile(
        isThreeLine: _itemType == _MaterialListType.threeLine,
        dense: _dense,
        leading:
            _showAvatars != null ? ExcludeSemantics(child: CircleAvatar(child: Text(item))) : null,
        title: Text('This item represents $item.'),
        subtitle: subtitle != null ? Text(subtitle) : null,
        trailing:
            _showIcons != null ? Icon(Icons.info, color: Theme.of(context).disabledColor) : null,
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final String layoutText = _dense != null ? ' \u2013 Dense' : '';
    final String? itemTypeText = switch (_itemType) {
      _MaterialListType.oneLine || _MaterialListType.oneLineWithAvatar => 'Single-line',
      _MaterialListType.twoLine => 'Two-line',
      _MaterialListType.threeLine => 'Three-line',
      null => null,
    };

    Iterable<Widget> listTiles = items.map<Widget>((String item) => buildListTile(context, item));
    if (_showDividers != null) {
      listTiles = ListTile.divideTiles(context: context, tiles: listTiles);
    }

    return Scaffold(
      key: scaffoldKey,
      appBar: AppBar(
        title: Text('Scrolling list\n$itemTypeText$layoutText'),
        actions: <Widget>[
          MaterialDemoDocumentationButton(ListDemo.routeName),
          IconButton(
            icon: const Icon(Icons.sort_by_alpha),
            tooltip: 'Sort',
            onPressed: () {
              setState(() {
                _reverseSort = !_reverseSort;
                items.sort((String a, String b) => _reverseSort ? b.compareTo(a) : a.compareTo(b));
              });
            },
          ),
          IconButton(
            icon: Icon(
              Theme.of(context).platform == TargetPlatform.iOS ? Icons.more_horiz : Icons.more_vert,
            ),
            tooltip: 'Show menu',
            onPressed: _bottomSheet == null ? _showConfigurationSheet : null,
          ),
        ],
      ),
      body: Scrollbar(
        child: ListView(
          primary: true,
          padding: EdgeInsets.symmetric(vertical: _dense != null ? 4.0 : 8.0),
          children: listTiles.toList(),
        ),
      ),
    );
  }
}
