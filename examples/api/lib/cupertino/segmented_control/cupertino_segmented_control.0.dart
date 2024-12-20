// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoSegmentedControl].

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
  bool _toggleOne = false;
  bool _toggleAll = true;
  Set<Sky> _disabledChildren = <Sky>{};

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      backgroundColor: skyColors[_selectedSegment],
      navigationBar: CupertinoNavigationBar(
        // This Cupertino segmented control has the enum "Sky" as the type.
        middle: CupertinoSegmentedControl<Sky>(
          disabledChildren: _disabledChildren,
          selectedColor: skyColors[_selectedSegment],
          // Provide horizontal padding around the children.
          padding: const EdgeInsets.symmetric(horizontal: 12),
          // This represents a currently selected segmented control.
          groupValue: _selectedSegment,
          // Callback that sets the selected segmented control.
          onValueChanged: (Sky value) {
            setState(() {
              _selectedSegment = value;
            });
          },
          children: const <Sky, Widget>{
            Sky.midnight: Padding(
              padding: EdgeInsets.symmetric(horizontal: 20),
              child: Text('Midnight'),
            ),
            Sky.viridian: Padding(
              padding: EdgeInsets.symmetric(horizontal: 20),
              child: Text('Viridian'),
            ),
            Sky.cerulean: Padding(
              padding: EdgeInsets.symmetric(horizontal: 20),
              child: Text('Cerulean'),
            ),
          },
        ),
      ),
      child: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text(
              'Selected Segment: ${_selectedSegment.name}',
              style: const TextStyle(color: CupertinoColors.white),
            ),
            const SizedBox(height: 20),
            Row(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                const Text('Disable one segment', style: TextStyle(color: CupertinoColors.white)),
                CupertinoSwitch(
                  value: _toggleOne,
                  onChanged: (bool value) {
                    setState(() {
                      _toggleOne = value;
                      if (value) {
                        _toggleAll = false;
                        _disabledChildren = <Sky>{Sky.midnight};
                      } else {
                        _toggleAll = true;
                        _disabledChildren = <Sky>{};
                      }
                    });
                  },
                ),
              ],
            ),
            Row(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                const Text('Toggle all segments', style: TextStyle(color: CupertinoColors.white)),
                CupertinoSwitch(
                  value: _toggleAll,
                  onChanged: (bool value) {
                    setState(() {
                      _toggleAll = value;
                      if (value) {
                        _toggleOne = false;
                        _disabledChildren = <Sky>{};
                      } else {
                        _disabledChildren = <Sky>{Sky.midnight, Sky.viridian, Sky.cerulean};
                      }
                    });
                  },
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}
