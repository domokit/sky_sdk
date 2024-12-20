// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'src/tests/controls_page.dart';
import 'src/tests/headings_page.dart';
import 'src/tests/popup_page.dart';
import 'src/tests/text_field_page.dart';

void main() {
  runApp(const TestApp());
}

Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
  selectionControlsRoute: (BuildContext context) => const SelectionControlsPage(),
  popupControlsRoute: (BuildContext context) => const PopupControlsPage(),
  textFieldRoute: (BuildContext context) => const TextFieldPage(),
  headingsRoute: (BuildContext context) => const HeadingsPage(),
};

class TestApp extends StatelessWidget {
  const TestApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      routes: routes,
      home: Builder(
        builder: (BuildContext context) {
          return Scaffold(
            body: ListView(
              children:
                  routes.keys.map<Widget>((String value) {
                    return MaterialButton(
                      child: Text(value),
                      onPressed: () {
                        Navigator.of(context).pushNamed(value);
                      },
                    );
                  }).toList(),
            ),
          );
        },
      ),
    );
  }
}
