// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../utils.dart';
import 'use_cases.dart';

class RadioListTileUseCase extends UseCase {
  @override
  String get name => 'RadioListTile';

  @override
  String get route => '/radio-list-tile';

  @override
  Widget build(BuildContext context) => _MainWidget();
}

class _MainWidget extends StatefulWidget {
  @override
  State<_MainWidget> createState() => _MainWidgetState();
}

enum SingingCharacter { lafayette, jefferson }

class _MainWidgetState extends State<_MainWidget> {
  SingingCharacter _value = SingingCharacter.lafayette;

  void _onChanged(SingingCharacter? value) {
    setState(() {
      _value = value!;
    });
  }

  String pageTitle = getUseCaseName(RadioListTileUseCase());

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Semantics(headingLevel: 1, child: Text('$pageTitle Demo'))),
      body: ListView(
        children: <Widget>[
          RadioListTile<SingingCharacter>(
            title: const Text('Lafayette'),
            value: SingingCharacter.lafayette,
            groupValue: _value,
            onChanged: _onChanged,
          ),
          RadioListTile<SingingCharacter>(
            title: const Text('Jefferson'),
            value: SingingCharacter.jefferson,
            groupValue: _value,
            onChanged: _onChanged,
          ),
        ],
      ),
    );
  }
}
