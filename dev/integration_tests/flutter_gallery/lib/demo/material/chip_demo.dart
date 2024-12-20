// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

const List<String> _defaultMaterialsA = <String>['poker', 'tortilla', 'fish and', 'micro', 'wood'];

const List<String> _defaultMaterialsB = <String>['apple', 'orange', 'tomato', 'grape', 'lettuce'];

const List<String> _defaultActions = <String>[
  'flake',
  'cut',
  'fragment',
  'splinter',
  'nick',
  'fry',
  'solder',
  'cash in',
  'eat',
];

const Map<String, String> _results = <String, String>{
  'flake': 'flaking',
  'cut': 'cutting',
  'fragment': 'fragmenting',
  'splinter': 'splintering',
  'nick': 'nicking',
  'fry': 'frying',
  'solder': 'soldering',
  'cash in': 'cashing in',
  'eat': 'eating',
};

const List<String> _defaultToolsA = <String>['hammer', 'chisel', 'fryer', 'fabricator', 'customer'];

const List<String> _defaultToolsB = <String>['keyboard', 'mouse', 'monitor', 'printer', 'cable'];

const Map<String, String> _avatars = <String, String>{
  'hammer': 'people/square/ali.png',
  'chisel': 'people/square/sandra.png',
  'fryer': 'people/square/trevor.png',
  'fabricator': 'people/square/stella.png',
  'customer': 'people/square/peter.png',
};

const Map<String, Set<String>> _toolActions = <String, Set<String>>{
  'hammer': <String>{'flake', 'fragment', 'splinter'},
  'chisel': <String>{'flake', 'nick', 'splinter'},
  'fryer': <String>{'fry'},
  'fabricator': <String>{'solder'},
  'customer': <String>{'cash in', 'eat'},
};

const Map<String, Set<String>> _materialActions = <String, Set<String>>{
  'poker': <String>{'cash in'},
  'tortilla': <String>{'fry', 'eat'},
  'fish and': <String>{'fry', 'eat'},
  'micro': <String>{'solder', 'fragment'},
  'wood': <String>{'flake', 'cut', 'splinter', 'nick'},
};

class _ChipsTile extends StatelessWidget {
  const _ChipsTile({this.label, this.children});

  final String? label;
  final List<Widget>? children;

  // Wraps a list of chips into a ListTile for display as a section in the demo.
  @override
  Widget build(BuildContext context) {
    return Card(
      semanticContainer: false,
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Container(
            padding: const EdgeInsets.only(top: 16.0, bottom: 4.0),
            alignment: Alignment.center,
            child: Text(label!, textAlign: TextAlign.start),
          ),
          if (children!.isNotEmpty)
            Wrap(
              children:
                  children!.map<Widget>((Widget chip) {
                    return Padding(padding: const EdgeInsets.all(2.0), child: chip);
                  }).toList(),
            )
          else
            Semantics(
              container: true,
              child: Container(
                alignment: Alignment.center,
                constraints: const BoxConstraints(minWidth: 48.0, minHeight: 48.0),
                padding: const EdgeInsets.all(8.0),
                child: Text(
                  'None',
                  style: Theme.of(
                    context,
                  ).textTheme.bodySmall!.copyWith(fontStyle: FontStyle.italic),
                ),
              ),
            ),
        ],
      ),
    );
  }
}

class ChipDemo extends StatefulWidget {
  const ChipDemo({super.key});

  static const String routeName = '/material/chip';

  @override
  State<ChipDemo> createState() => _ChipDemoState();
}

class _ChipDemoState extends State<ChipDemo> {
  _ChipDemoState() {
    _reset();
  }

  final Set<String> _materialsA = <String>{};
  final Set<String> _materialsB = <String>{};
  String _selectedMaterial = '';
  String _selectedAction = '';
  final Set<String> _toolsA = <String>{};
  final Set<String> _toolsB = <String>{};
  final Set<String> _selectedTools = <String>{};
  final Set<String> _actions = <String>{};
  bool _showShapeBorder = false;

  // Initialize members with the default data.
  void _reset() {
    _materialsA.clear();
    _materialsA.addAll(_defaultMaterialsA);
    _materialsB.clear();
    _materialsB.addAll(_defaultMaterialsB);
    _actions.clear();
    _actions.addAll(_defaultActions);
    _toolsA.clear();
    _toolsA.addAll(_defaultToolsA);
    _toolsB.clear();
    _toolsB.addAll(_defaultToolsB);
    _selectedMaterial = '';
    _selectedAction = '';
    _selectedTools.clear();
  }

  void _removeMaterial(String name) {
    _materialsA.remove(name);
    _materialsB.remove(name);
    if (_selectedMaterial == name) {
      _selectedMaterial = '';
    }
  }

  void _removeTool(String name) {
    _toolsA.remove(name);
    _toolsB.remove(name);
    _selectedTools.remove(name);
  }

  String _capitalize(String name) {
    assert(name.isNotEmpty);
    return name.substring(0, 1).toUpperCase() + name.substring(1);
  }

  // This converts a String to a unique color, based on the hash value of the
  // String object. It takes the bottom 16 bits of the hash, and uses that to
  // pick a hue for an HSV color, and then creates the color (with a preset
  // saturation and value). This means that any unique strings will also have
  // unique colors, but they'll all be readable, since they have the same
  // saturation and value.
  Color _nameToColor(String name, ThemeData theme) {
    assert(name.length > 1);
    final int hash = name.hashCode & 0xffff;
    final double hue = (360.0 * hash / (1 << 15)) % 360.0;
    final double themeValue = HSVColor.fromColor(theme.colorScheme.surface).value;
    return HSVColor.fromAHSV(1.0, hue, 0.4, themeValue).toColor();
  }

  AssetImage _nameToAvatar(String name) {
    assert(_avatars.containsKey(name));
    return AssetImage(_avatars[name]!, package: 'flutter_gallery_assets');
  }

  String _createResult() {
    if (_selectedAction.isEmpty) {
      return '';
    }
    final String value = _capitalize(_results[_selectedAction]!);
    return '$value!';
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final List<Widget> chips =
        _materialsA.map<Widget>((String name) {
          return Chip(
            key: ValueKey<String>(name),
            backgroundColor: _nameToColor(name, theme),
            label: Text(_capitalize(name)),
            onDeleted: () {
              setState(() {
                _removeMaterial(name);
              });
            },
          );
        }).toList();

    final List<Widget> inputChips =
        _toolsA.map<Widget>((String name) {
          return InputChip(
            key: ValueKey<String>(name),
            avatar: CircleAvatar(backgroundImage: _nameToAvatar(name)),
            label: Text(_capitalize(name)),
            onDeleted: () {
              setState(() {
                _removeTool(name);
              });
            },
          );
        }).toList();

    final List<Widget> choiceChips =
        _materialsB.map<Widget>((String name) {
          return ChoiceChip(
            key: ValueKey<String>(name),
            backgroundColor: _nameToColor(name, theme),
            label: Text(_capitalize(name)),
            selected: _selectedMaterial == name,
            onSelected: (bool value) {
              setState(() {
                _selectedMaterial = value ? name : '';
              });
            },
          );
        }).toList();

    final List<Widget> filterChips =
        _toolsB.map<Widget>((String name) {
          return FilterChip(
            key: ValueKey<String>(name),
            label: Text(_capitalize(name)),
            selected: _toolsB.contains(name) && _selectedTools.contains(name),
            onSelected:
                !_toolsB.contains(name)
                    ? null
                    : (bool value) {
                      setState(() {
                        if (!value) {
                          _selectedTools.remove(name);
                        } else {
                          _selectedTools.add(name);
                        }
                      });
                    },
          );
        }).toList();

    Set<String> allowedActions = <String>{};
    if (_selectedMaterial.isNotEmpty) {
      for (final String tool in _selectedTools) {
        allowedActions.addAll(_toolActions[tool]!);
      }
      allowedActions = allowedActions.intersection(_materialActions[_selectedMaterial]!);
    }

    final List<Widget> actionChips =
        allowedActions.map<Widget>((String name) {
          return ActionChip(
            label: Text(_capitalize(name)),
            onPressed: () {
              setState(() {
                _selectedAction = name;
              });
            },
          );
        }).toList();

    final List<Widget> tiles = <Widget>[
      const SizedBox(height: 8.0, width: 0.0),
      _ChipsTile(label: 'Available Materials (Chip)', children: chips),
      _ChipsTile(label: 'Available Tools (InputChip)', children: inputChips),
      _ChipsTile(label: 'Choose a Material (ChoiceChip)', children: choiceChips),
      _ChipsTile(label: 'Choose Tools (FilterChip)', children: filterChips),
      _ChipsTile(label: 'Perform Allowed Action (ActionChip)', children: actionChips),
      const Divider(),
      Padding(
        padding: const EdgeInsets.all(8.0),
        child: Center(child: Text(_createResult(), style: theme.textTheme.titleLarge)),
      ),
    ];

    return Scaffold(
      appBar: AppBar(
        title: const Text('Chips'),
        actions: <Widget>[
          MaterialDemoDocumentationButton(ChipDemo.routeName),
          IconButton(
            onPressed: () {
              setState(() {
                _showShapeBorder = !_showShapeBorder;
              });
            },
            icon: const Icon(Icons.vignette, semanticLabel: 'Update border shape'),
          ),
        ],
      ),
      body: ChipTheme(
        data:
            _showShapeBorder
                ? theme.chipTheme.copyWith(
                  shape: BeveledRectangleBorder(
                    side: const BorderSide(width: 0.66, color: Colors.grey),
                    borderRadius: BorderRadius.circular(10.0),
                  ),
                )
                : theme.chipTheme,
        child: Scrollbar(child: ListView(primary: true, children: tiles)),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () => setState(_reset),
        child: const Icon(Icons.refresh, semanticLabel: 'Reset chips'),
      ),
    );
  }
}
