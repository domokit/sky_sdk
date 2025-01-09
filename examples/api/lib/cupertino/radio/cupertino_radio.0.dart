// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoRadio].

void main() => runApp(const CupertinoRadioApp());

class CupertinoRadioApp extends StatelessWidget {
  const CupertinoRadioApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: CupertinoPageScaffold(
        navigationBar: CupertinoNavigationBar(middle: Text('CupertinoRadio Example')),
        child: SafeArea(child: CupertinoRadioExample()),
      ),
    );
  }
}

enum SingingCharacter { lafayette, jefferson }

class CupertinoRadioExample extends StatefulWidget {
  const CupertinoRadioExample({super.key});

  @override
  State<CupertinoRadioExample> createState() => _CupertinoRadioExampleState();
}

class _CupertinoRadioExampleState extends State<CupertinoRadioExample> {
  SingingCharacter? _character = SingingCharacter.lafayette;

  @override
  Widget build(BuildContext context) {
    return CupertinoListSection(
      children: <Widget>[
        CupertinoListTile(
          title: const Text('Lafayette'),
          leading: CupertinoRadio<SingingCharacter>(
            value: SingingCharacter.lafayette,
            groupValue: _character,
            onChanged: (SingingCharacter? value) {
              setState(() {
                _character = value;
              });
            },
          ),
        ),
        CupertinoListTile(
          title: const Text('Thomas Jefferson'),
          leading: CupertinoRadio<SingingCharacter>(
            value: SingingCharacter.jefferson,
            groupValue: _character,
            onChanged: (SingingCharacter? value) {
              setState(() {
                _character = value;
              });
            },
          ),
        ),
      ],
    );
  }
}
