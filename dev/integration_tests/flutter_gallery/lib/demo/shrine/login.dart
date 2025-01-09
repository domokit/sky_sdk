// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'colors.dart';

class LoginPage extends StatefulWidget {
  const LoginPage({super.key});

  @override
  State<LoginPage> createState() => _LoginPageState();
}

class _LoginPageState extends State<LoginPage> {
  final TextEditingController _usernameController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();
  static const ShapeDecoration _decoration = ShapeDecoration(
    shape: BeveledRectangleBorder(
      side: BorderSide(color: kShrineBrown900, width: 0.5),
      borderRadius: BorderRadius.all(Radius.circular(7.0)),
    ),
  );

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        elevation: 0.0,
        backgroundColor: Colors.white,
        leading: IconButton(
          icon: const BackButtonIcon(),
          tooltip: MaterialLocalizations.of(context).backButtonTooltip,
          onPressed: () {
            // The login screen is immediately displayed on top of the Shrine
            // home screen using onGenerateRoute and so rootNavigator must be
            // set to true in order to get out of Shrine completely.
            Navigator.of(context, rootNavigator: true).pop();
          },
        ),
      ),
      body: SafeArea(
        child: ListView(
          padding: const EdgeInsets.symmetric(horizontal: 24.0),
          children: <Widget>[
            const SizedBox(height: 80.0),
            Column(
              children: <Widget>[
                Image.asset('packages/shrine_images/diamond.png'),
                const SizedBox(height: 16.0),
                Text('SHRINE', style: Theme.of(context).textTheme.headlineSmall),
              ],
            ),
            const SizedBox(height: 120.0),
            PrimaryColorOverride(
              color: kShrineBrown900,
              child: Container(
                decoration: _decoration,
                child: TextField(
                  controller: _usernameController,
                  decoration: const InputDecoration(labelText: 'Username'),
                ),
              ),
            ),
            const SizedBox(height: 12.0),
            PrimaryColorOverride(
              color: kShrineBrown900,
              child: Container(
                decoration: _decoration,
                child: TextField(
                  controller: _passwordController,
                  decoration: const InputDecoration(labelText: 'Password'),
                ),
              ),
            ),
            const SizedBox(height: 12.0),
            OverflowBar(
              spacing: 8,
              alignment: MainAxisAlignment.end,
              children: <Widget>[
                TextButton(
                  style: TextButton.styleFrom(
                    shape: const BeveledRectangleBorder(
                      borderRadius: BorderRadius.all(Radius.circular(7.0)),
                    ),
                  ),
                  onPressed: () {
                    // The login screen is immediately displayed on top of
                    // the Shrine home screen using onGenerateRoute and so
                    // rootNavigator must be set to true in order to get out
                    // of Shrine completely.
                    Navigator.of(context, rootNavigator: true).pop();
                  },
                  child: Text('CANCEL', style: Theme.of(context).textTheme.bodySmall),
                ),
                ElevatedButton(
                  style: ElevatedButton.styleFrom(
                    elevation: 8.0,
                    shape: const BeveledRectangleBorder(
                      borderRadius: BorderRadius.all(Radius.circular(7.0)),
                    ),
                  ),
                  onPressed: () {
                    Navigator.pop(context);
                  },
                  child: Text('NEXT', style: Theme.of(context).textTheme.bodySmall),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}

class PrimaryColorOverride extends StatelessWidget {
  const PrimaryColorOverride({super.key, this.color, this.child});

  final Color? color;
  final Widget? child;

  @override
  Widget build(BuildContext context) {
    return Theme(data: Theme.of(context).copyWith(primaryColor: color), child: child!);
  }
}
