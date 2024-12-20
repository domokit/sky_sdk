// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'text_field_constants.dart';

export 'text_field_constants.dart';

/// A page with a normal text field and a password field.
class TextFieldPage extends StatefulWidget {
  const TextFieldPage({super.key});

  @override
  State<StatefulWidget> createState() => _TextFieldPageState();
}

class _TextFieldPageState extends State<TextFieldPage> {
  final TextEditingController _normalController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();
  final Key backButtonKey = const ValueKey<String>(backButtonKeyValue);
  final Key normalTextFieldKey = const ValueKey<String>(normalTextFieldKeyValue);
  final Key passwordTextFieldKey = const ValueKey<String>(passwordTextFieldKeyValue);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(leading: BackButton(key: backButtonKey)),
      body: Material(
        child: Column(
          children: <Widget>[
            TextField(key: normalTextFieldKey, controller: _normalController),
            const Spacer(),
            TextField(
              key: passwordTextFieldKey,
              controller: _passwordController,
              obscureText: true,
            ),
          ],
        ),
      ),
    );
  }
}
