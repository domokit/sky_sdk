// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoSwitch].

void main() => runApp(const CupertinoSwitchApp());

class CupertinoSwitchApp extends StatelessWidget {
  const CupertinoSwitchApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: CupertinoSwitchExample(),
    );
  }
}

class CupertinoSwitchExample extends StatefulWidget {
  const CupertinoSwitchExample({super.key});

  @override
  State<CupertinoSwitchExample> createState() => _CupertinoSwitchExampleState();
}

class _CupertinoSwitchExampleState extends State<CupertinoSwitchExample> {
  bool switchValue = true;

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(middle: Text('CupertinoSwitch Sample')),
      child: Center(
        child: CupertinoSwitch(
          // This bool value toggles the switch.
          value: switchValue,
          activeTrackColor: CupertinoColors.activeBlue,
          onChanged: (bool? value) {
            // This is called when the user toggles the switch.
            setState(() {
              switchValue = value ?? false;
            });
          },
        ),
      ),
    );
  }
}
