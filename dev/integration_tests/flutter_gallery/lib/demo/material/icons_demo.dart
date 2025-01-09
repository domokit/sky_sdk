// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

class IconsDemo extends StatefulWidget {
  const IconsDemo({super.key});

  static const String routeName = '/material/icons';

  @override
  IconsDemoState createState() => IconsDemoState();
}

class IconsDemoState extends State<IconsDemo> {
  static final List<MaterialColor> iconColors = <MaterialColor>[
    Colors.red,
    Colors.pink,
    Colors.purple,
    Colors.deepPurple,
    Colors.indigo,
    Colors.blue,
    Colors.lightBlue,
    Colors.cyan,
    Colors.teal,
    Colors.green,
    Colors.lightGreen,
    Colors.lime,
    Colors.yellow,
    Colors.amber,
    Colors.orange,
    Colors.deepOrange,
    Colors.brown,
    Colors.grey,
    Colors.blueGrey,
  ];

  int iconColorIndex = 8; // teal

  Color get iconColor => iconColors[iconColorIndex];

  void handleIconButtonPress() {
    setState(() {
      iconColorIndex = (iconColorIndex + 1) % iconColors.length;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Icons'),
        actions: <Widget>[MaterialDemoDocumentationButton(IconsDemo.routeName)],
      ),
      body: IconTheme(
        data: IconThemeData(color: iconColor),
        child: SafeArea(
          top: false,
          bottom: false,
          child: Scrollbar(
            child: ListView(
              primary: true,
              padding: const EdgeInsets.all(24.0),
              children: <Widget>[
                _IconsDemoCard(handleIconButtonPress, Icons.face), // direction-agnostic icon
                const SizedBox(height: 24.0),
                _IconsDemoCard(
                  handleIconButtonPress,
                  Icons.battery_unknown,
                ), // direction-aware icon
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class _IconsDemoCard extends StatelessWidget {
  const _IconsDemoCard(this.handleIconButtonPress, this.icon);

  final VoidCallback handleIconButtonPress;
  final IconData icon;

  Widget _buildIconButton(double iconSize, IconData icon, bool enabled) {
    return IconButton(
      icon: Icon(icon),
      iconSize: iconSize,
      tooltip: "${enabled ? 'Enabled' : 'Disabled'} icon button",
      onPressed: enabled ? handleIconButtonPress : null,
    );
  }

  Widget _centeredText(String label) => Padding(
    // Match the default padding of IconButton.
    padding: const EdgeInsets.all(8.0),
    child: Text(label, textAlign: TextAlign.center),
  );

  TableRow _buildIconRow(double size) {
    return TableRow(
      children: <Widget>[
        _centeredText('${size.floor()} $icon'),
        _buildIconButton(size, icon, true),
        _buildIconButton(size, icon, false),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final TextStyle textStyle = theme.textTheme.titleMedium!.copyWith(
      color: theme.textTheme.bodySmall!.color,
    );
    return Card(
      child: DefaultTextStyle(
        style: textStyle,
        child: Semantics(
          explicitChildNodes: true,
          child: Table(
            defaultVerticalAlignment: TableCellVerticalAlignment.middle,
            children: <TableRow>[
              TableRow(
                children: <Widget>[
                  _centeredText('Size $icon'),
                  _centeredText('Enabled $icon'),
                  _centeredText('Disabled $icon'),
                ],
              ),
              _buildIconRow(18.0),
              _buildIconRow(24.0),
              _buildIconRow(36.0),
              _buildIconRow(48.0),
            ],
          ),
        ),
      ),
    );
  }
}
