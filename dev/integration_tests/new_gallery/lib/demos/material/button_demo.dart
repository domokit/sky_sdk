// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';
import 'material_demo_types.dart';

class ButtonDemo extends StatelessWidget {
  const ButtonDemo({super.key, required this.type});

  final ButtonDemoType type;

  String _title(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return switch (type) {
      ButtonDemoType.text => localizations.demoTextButtonTitle,
      ButtonDemoType.elevated => localizations.demoElevatedButtonTitle,
      ButtonDemoType.outlined => localizations.demoOutlinedButtonTitle,
      ButtonDemoType.toggle => localizations.demoToggleButtonTitle,
      ButtonDemoType.floating => localizations.demoFloatingButtonTitle,
    };
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(automaticallyImplyLeading: false, title: Text(_title(context))),
      body: switch (type) {
        ButtonDemoType.text => _TextButtonDemo(),
        ButtonDemoType.elevated => _ElevatedButtonDemo(),
        ButtonDemoType.outlined => _OutlinedButtonDemo(),
        ButtonDemoType.toggle => _ToggleButtonsDemo(),
        ButtonDemoType.floating => _FloatingActionButtonDemo(),
      },
    );
  }
}

// BEGIN buttonDemoText

class _TextButtonDemo extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            TextButton(onPressed: () {}, child: Text(localizations.buttonText)),
            const SizedBox(width: 12),
            TextButton.icon(
              icon: const Icon(Icons.add, size: 18),
              label: Text(localizations.buttonText),
              onPressed: () {},
            ),
          ],
        ),
        const SizedBox(height: 12),
        // Disabled buttons
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            TextButton(onPressed: null, child: Text(localizations.buttonText)),
            const SizedBox(width: 12),
            TextButton.icon(
              icon: const Icon(Icons.add, size: 18),
              label: Text(localizations.buttonText),
              onPressed: null,
            ),
          ],
        ),
      ],
    );
  }
}

// END

// BEGIN buttonDemoElevated

class _ElevatedButtonDemo extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            ElevatedButton(onPressed: () {}, child: Text(localizations.buttonText)),
            const SizedBox(width: 12),
            ElevatedButton.icon(
              icon: const Icon(Icons.add, size: 18),
              label: Text(localizations.buttonText),
              onPressed: () {},
            ),
          ],
        ),
        const SizedBox(height: 12),
        // Disabled buttons
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            ElevatedButton(onPressed: null, child: Text(localizations.buttonText)),
            const SizedBox(width: 12),
            ElevatedButton.icon(
              icon: const Icon(Icons.add, size: 18),
              label: Text(localizations.buttonText),
              onPressed: null,
            ),
          ],
        ),
      ],
    );
  }
}

// END

// BEGIN buttonDemoOutlined

class _OutlinedButtonDemo extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            OutlinedButton(onPressed: () {}, child: Text(localizations.buttonText)),
            const SizedBox(width: 12),
            OutlinedButton.icon(
              icon: const Icon(Icons.add, size: 18),
              label: Text(localizations.buttonText),
              onPressed: () {},
            ),
          ],
        ),
        const SizedBox(height: 12),
        // Disabled buttons
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            OutlinedButton(onPressed: null, child: Text(localizations.buttonText)),
            const SizedBox(width: 12),
            OutlinedButton.icon(
              icon: const Icon(Icons.add, size: 18),
              label: Text(localizations.buttonText),
              onPressed: null,
            ),
          ],
        ),
      ],
    );
  }
}

// END

// BEGIN buttonDemoToggle

class _ToggleButtonsDemo extends StatefulWidget {
  @override
  _ToggleButtonsDemoState createState() => _ToggleButtonsDemoState();
}

class _ToggleButtonsDemoState extends State<_ToggleButtonsDemo> with RestorationMixin {
  final List<RestorableBool> isSelected = <RestorableBool>[
    RestorableBool(false),
    RestorableBool(true),
    RestorableBool(false),
  ];

  @override
  String get restorationId => 'toggle_button_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(isSelected[0], 'first_item');
    registerForRestoration(isSelected[1], 'second_item');
    registerForRestoration(isSelected[2], 'third_item');
  }

  @override
  void dispose() {
    for (final RestorableBool restorableBool in isSelected) {
      restorableBool.dispose();
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          ToggleButtons(
            onPressed: (int index) {
              setState(() {
                isSelected[index].value = !isSelected[index].value;
              });
            },
            isSelected: isSelected.map((RestorableBool element) => element.value).toList(),
            children: const <Widget>[
              Icon(Icons.format_bold),
              Icon(Icons.format_italic),
              Icon(Icons.format_underline),
            ],
          ),
          const SizedBox(height: 12),
          // Disabled toggle buttons
          ToggleButtons(
            isSelected: isSelected.map((RestorableBool element) => element.value).toList(),
            children: const <Widget>[
              Icon(Icons.format_bold),
              Icon(Icons.format_italic),
              Icon(Icons.format_underline),
            ],
          ),
        ],
      ),
    );
  }
}

// END

// BEGIN buttonDemoFloating

class _FloatingActionButtonDemo extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Center(
      child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          FloatingActionButton(
            onPressed: () {},
            tooltip: localizations.buttonTextCreate,
            child: const Icon(Icons.add),
          ),
          const SizedBox(width: 12),
          FloatingActionButton.extended(
            icon: const Icon(Icons.add),
            label: Text(localizations.buttonTextCreate),
            onPressed: () {},
          ),
        ],
      ),
    );
  }
}

// END
