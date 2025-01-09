// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

const String _elevatedText =
    'Elevated buttons add dimension to mostly flat layouts. They emphasize '
    'functions on busy or wide spaces.';

const String _elevatedCode = 'buttons_elevated';

const String _textText =
    'A text button displays an ink splash on press '
    'but does not lift. Use text buttons on toolbars, in dialogs and '
    'inline with padding';

const String _textCode = 'buttons_text';

const String _outlinedText =
    'Outlined buttons become opaque and elevate when pressed. They are often '
    'paired with elevated buttons to indicate an alternative, secondary action.';

const String _outlinedCode = 'buttons_outlined';

const String _dropdownText =
    "A dropdown button displays a menu that's used to select a value from a "
    'small set of values. The button displays the current value and a down '
    'arrow.';

const String _dropdownCode = 'buttons_dropdown';

const String _iconText =
    'IconButtons are appropriate for toggle buttons that allow a single choice '
    "to be selected or deselected, such as adding or removing an item's star.";

const String _iconCode = 'buttons_icon';

const String _actionText =
    'Floating action buttons are used for a promoted action. They are '
    'distinguished by a circled icon floating above the UI and can have motion '
    'behaviors that include morphing, launching, and a transferring anchor '
    'point.';

const String _actionCode = 'buttons_action';

class ButtonsDemo extends StatefulWidget {
  const ButtonsDemo({super.key});

  static const String routeName = '/material/buttons';

  @override
  State<ButtonsDemo> createState() => _ButtonsDemoState();
}

class _ButtonsDemoState extends State<ButtonsDemo> {
  OutlinedBorder? _buttonShape;

  @override
  Widget build(BuildContext context) {
    final List<ComponentDemoTabData> demos = <ComponentDemoTabData>[
      ComponentDemoTabData(
        tabName: 'ELEVATED',
        description: _elevatedText,
        demoWidget: buildElevatedButton(_buttonShape),
        exampleCodeTag: _elevatedCode,
        documentationUrl: 'https://api.flutter.dev/flutter/material/ElevatedButton-class.html',
      ),
      ComponentDemoTabData(
        tabName: 'TEXT',
        description: _textText,
        demoWidget: buildTextButton(_buttonShape),
        exampleCodeTag: _textCode,
        documentationUrl: 'https://api.flutter.dev/flutter/material/TextButton-class.html',
      ),
      ComponentDemoTabData(
        tabName: 'OUTLINED',
        description: _outlinedText,
        demoWidget: buildOutlinedButton(_buttonShape),
        exampleCodeTag: _outlinedCode,
        documentationUrl: 'https://api.flutter.dev/flutter/material/OutlinedButton-class.html',
      ),
      ComponentDemoTabData(
        tabName: 'DROPDOWN',
        description: _dropdownText,
        demoWidget: buildDropdownButton(),
        exampleCodeTag: _dropdownCode,
        documentationUrl: 'https://api.flutter.dev/flutter/material/DropdownButton-class.html',
      ),
      ComponentDemoTabData(
        tabName: 'ICON',
        description: _iconText,
        demoWidget: buildIconButton(),
        exampleCodeTag: _iconCode,
        documentationUrl: 'https://api.flutter.dev/flutter/material/IconButton-class.html',
      ),
      ComponentDemoTabData(
        tabName: 'ACTION',
        description: _actionText,
        demoWidget: buildActionButton(),
        exampleCodeTag: _actionCode,
        documentationUrl:
            'https://api.flutter.dev/flutter/material/FloatingActionButton-class.html',
      ),
    ];

    return TabbedComponentDemoScaffold(
      title: 'Buttons',
      demos: demos,
      actions: <Widget>[
        IconButton(
          icon: const Icon(Icons.sentiment_very_satisfied, semanticLabel: 'Update shape'),
          onPressed: () {
            setState(() {
              _buttonShape = _buttonShape == null ? const StadiumBorder() : null;
            });
          },
        ),
      ],
    );
  }

  Widget buildElevatedButton(OutlinedBorder? shape) {
    final ButtonStyle style = ElevatedButton.styleFrom(shape: shape);
    return Align(
      alignment: const Alignment(0.0, -0.2),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          const SizedBox(height: 2),
          OverflowBar(
            spacing: 8,
            children: <Widget>[
              ElevatedButton(
                style: style,
                child: const Text('ELEVATED BUTTON', semanticsLabel: 'ELEVATED BUTTON 1'),
                onPressed: () {
                  // Perform some action
                },
              ),
              const ElevatedButton(
                onPressed: null,
                child: Text('DISABLED', semanticsLabel: 'DISABLED BUTTON 1'),
              ),
            ],
          ),
          const SizedBox(height: 16),
          OverflowBar(
            spacing: 8,
            children: <Widget>[
              ElevatedButton.icon(
                style: style,
                icon: const Icon(Icons.add, size: 18.0),
                label: const Text('ELEVATED BUTTON', semanticsLabel: 'ELEVATED BUTTON 2'),
                onPressed: () {
                  // Perform some action
                },
              ),
              ElevatedButton.icon(
                style: style,
                icon: const Icon(Icons.add, size: 18.0),
                label: const Text('DISABLED', semanticsLabel: 'DISABLED BUTTON 2'),
                onPressed: () {},
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget buildTextButton(OutlinedBorder? shape) {
    final ButtonStyle style = ElevatedButton.styleFrom(shape: shape);
    return Align(
      alignment: const Alignment(0.0, -0.2),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          const SizedBox(height: 2),
          OverflowBar(
            spacing: 8,
            children: <Widget>[
              TextButton(
                style: style,
                child: const Text('TEXT BUTTON', semanticsLabel: 'TEXT BUTTON 1'),
                onPressed: () {
                  // Perform some action
                },
              ),
              const TextButton(
                onPressed: null,
                child: Text('DISABLED', semanticsLabel: 'DISABLED BUTTON 3'),
              ),
            ],
          ),
          OverflowBar(
            spacing: 8,
            children: <Widget>[
              TextButton.icon(
                style: style,
                icon: const Icon(Icons.add_circle_outline, size: 18.0),
                label: const Text('TEXT BUTTON', semanticsLabel: 'TEXT BUTTON 2'),
                onPressed: () {
                  // Perform some action
                },
              ),
              TextButton.icon(
                style: style,
                icon: const Icon(Icons.add_circle_outline, size: 18.0),
                label: const Text('DISABLED', semanticsLabel: 'DISABLED BUTTON 4'),
                onPressed: () {},
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget buildOutlinedButton(OutlinedBorder? shape) {
    final ButtonStyle style = ElevatedButton.styleFrom(shape: shape);
    return Align(
      alignment: const Alignment(0.0, -0.2),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          const SizedBox(height: 2),
          OverflowBar(
            spacing: 8,
            children: <Widget>[
              OutlinedButton(
                style: style,
                child: const Text('OUTLINED BUTTON', semanticsLabel: 'OUTLINED BUTTON 1'),
                onPressed: () {
                  // Perform some action
                },
              ),
              OutlinedButton(
                style: style,
                onPressed: null,
                child: const Text('DISABLED', semanticsLabel: 'DISABLED BUTTON 5'),
              ),
            ],
          ),
          const SizedBox(height: 16),
          OverflowBar(
            spacing: 8,
            children: <Widget>[
              OutlinedButton.icon(
                style: style,
                icon: const Icon(Icons.add, size: 18.0),
                label: const Text('OUTLINED BUTTON', semanticsLabel: 'OUTLINED BUTTON 2'),
                onPressed: () {
                  // Perform some action
                },
              ),
              OutlinedButton.icon(
                icon: const Icon(Icons.add, size: 18.0),
                label: const Text('DISABLED', semanticsLabel: 'DISABLED BUTTON 6'),
                onPressed: null,
              ),
            ],
          ),
        ],
      ),
    );
  }

  // https://en.wikipedia.org/wiki/Free_Four
  String? dropdown1Value = 'Free';
  String? dropdown2Value;
  String? dropdown3Value = 'Four';

  Widget buildDropdownButton() {
    return Padding(
      padding: const EdgeInsets.all(24.0),
      child: Column(
        children: <Widget>[
          ListTile(
            title: const Text('Simple dropdown:'),
            trailing: DropdownButton<String>(
              value: dropdown1Value,
              onChanged: (String? newValue) {
                setState(() {
                  dropdown1Value = newValue;
                });
              },
              items:
                  <String>['One', 'Two', 'Free', 'Four'].map<DropdownMenuItem<String>>((
                    String value,
                  ) {
                    return DropdownMenuItem<String>(value: value, child: Text(value));
                  }).toList(),
            ),
          ),
          const SizedBox(height: 24.0),
          ListTile(
            title: const Text('Dropdown with a hint:'),
            trailing: DropdownButton<String>(
              value: dropdown2Value,
              hint: const Text('Choose'),
              onChanged: (String? newValue) {
                setState(() {
                  dropdown2Value = newValue;
                });
              },
              items:
                  <String>['One', 'Two', 'Free', 'Four'].map<DropdownMenuItem<String>>((
                    String value,
                  ) {
                    return DropdownMenuItem<String>(value: value, child: Text(value));
                  }).toList(),
            ),
          ),
          const SizedBox(height: 24.0),
          ListTile(
            title: const Text('Scrollable dropdown:'),
            trailing: DropdownButton<String>(
              value: dropdown3Value,
              onChanged: (String? newValue) {
                setState(() {
                  dropdown3Value = newValue;
                });
              },
              items:
                  <String>[
                    'One',
                    'Two',
                    'Free',
                    'Four',
                    'Can',
                    'I',
                    'Have',
                    'A',
                    'Little',
                    'Bit',
                    'More',
                    'Five',
                    'Six',
                    'Seven',
                    'Eight',
                    'Nine',
                    'Ten',
                  ].map<DropdownMenuItem<String>>((String value) {
                    return DropdownMenuItem<String>(value: value, child: Text(value));
                  }).toList(),
            ),
          ),
        ],
      ),
    );
  }

  bool iconButtonToggle = false;

  Widget buildIconButton() {
    return Align(
      alignment: const Alignment(0.0, -0.2),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children:
            <Widget>[
                  IconButton(
                    icon: const Icon(Icons.thumb_up, semanticLabel: 'Thumbs up'),
                    onPressed: () {
                      setState(() => iconButtonToggle = !iconButtonToggle);
                    },
                    color: iconButtonToggle ? Theme.of(context).primaryColor : null,
                  ),
                  const IconButton(
                    icon: Icon(Icons.thumb_up, semanticLabel: 'Thumbs not up'),
                    onPressed: null,
                  ),
                ]
                .map<Widget>((Widget button) => SizedBox(width: 64.0, height: 64.0, child: button))
                .toList(),
      ),
    );
  }

  Widget buildActionButton() {
    return Align(
      alignment: const Alignment(0.0, -0.2),
      child: FloatingActionButton(
        tooltip: 'floating action button',
        child: const Icon(Icons.add),
        onPressed: () {
          // Perform some action
        },
      ),
    );
  }
}
