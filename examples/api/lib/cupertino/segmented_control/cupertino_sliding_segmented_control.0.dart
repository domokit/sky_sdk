// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoSlidingSegmentedControl].

enum Sky { midnight, viridian, cerulean }

Map<Sky, Color> skyColors = <Sky, Color>{
  Sky.midnight: const Color(0xff191970),
  Sky.viridian: const Color(0xff40826d),
  Sky.cerulean: const Color(0xff007ba7),
};

void main() => runApp(const SegmentedControlApp());

class SegmentedControlApp extends StatelessWidget {
  const SegmentedControlApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: SegmentedControlExample(),
    );
  }
}

class SegmentedControlExample extends StatefulWidget {
  const SegmentedControlExample({super.key});

  @override
  State<SegmentedControlExample> createState() => _SegmentedControlExampleState();
}

class _SegmentedControlExampleState extends State<SegmentedControlExample> {
  Sky _selectedSegment = Sky.midnight;

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      backgroundColor: skyColors[_selectedSegment],
      navigationBar: CupertinoNavigationBar(
        // This Cupertino segmented control has the enum "Sky" as the type.
        middle: CupertinoSlidingSegmentedControl<Sky>(
          backgroundColor: CupertinoColors.systemGrey2,
          thumbColor: skyColors[_selectedSegment]!,
          // This represents the currently selected segmented control.
          groupValue: _selectedSegment,
          // Callback that sets the selected segmented control.
          onValueChanged: (Sky? value) {
            if (value != null) {
              setState(() {
                _selectedSegment = value;
              });
            }
          },
          children: const <Sky, Widget>{
            Sky.midnight: Padding(
              padding: EdgeInsets.symmetric(horizontal: 20),
              child: Text('Midnight', style: TextStyle(color: CupertinoColors.white)),
            ),
            Sky.viridian: Padding(
              padding: EdgeInsets.symmetric(horizontal: 20),
              child: Text('Viridian', style: TextStyle(color: CupertinoColors.white)),
            ),
            Sky.cerulean: Padding(
              padding: EdgeInsets.symmetric(horizontal: 20),
              child: Text('Cerulean', style: TextStyle(color: CupertinoColors.white)),
            ),
          },
        ),
      ),
      child: Center(
        child: Text(
          'Selected Segment: ${_selectedSegment.name}',
          style: const TextStyle(color: CupertinoColors.white),
        ),
      ),
    );
  }
}
