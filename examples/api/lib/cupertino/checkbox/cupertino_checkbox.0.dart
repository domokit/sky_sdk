// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoCheckbox].

void main() => runApp(const CupertinoCheckboxApp());

class CupertinoCheckboxApp extends StatelessWidget {
  const CupertinoCheckboxApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: CupertinoPageScaffold(
        navigationBar: CupertinoNavigationBar(middle: Text('CupertinoCheckbox Example')),
        child: SafeArea(child: CupertinoCheckboxExample()),
      ),
    );
  }
}

class CupertinoCheckboxExample extends StatefulWidget {
  const CupertinoCheckboxExample({super.key});

  @override
  State<CupertinoCheckboxExample> createState() => _CupertinoCheckboxExampleState();
}

class _CupertinoCheckboxExampleState extends State<CupertinoCheckboxExample> {
  bool? isChecked = true;

  @override
  Widget build(BuildContext context) {
    return CupertinoCheckbox(
      checkColor: CupertinoColors.white,
      // Set tristate to true to make the checkbox display a null value
      // in addition to the default true and false values.
      tristate: true,
      value: isChecked,
      onChanged: (bool? value) {
        setState(() {
          isChecked = value;
        });
      },
    );
  }
}
