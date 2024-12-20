// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

import '../../gallery/demo.dart';

class CupertinoSwitchDemo extends StatefulWidget {
  const CupertinoSwitchDemo({super.key});

  static const String routeName = '/cupertino/switch';

  @override
  State<CupertinoSwitchDemo> createState() => _CupertinoSwitchDemoState();
}

class _CupertinoSwitchDemoState extends State<CupertinoSwitchDemo> {
  bool _switchValue = false;

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        middle: const Text('Switch'),
        // We're specifying a back label here because the previous page is a
        // Material page. CupertinoPageRoutes could auto-populate these back
        // labels.
        previousPageTitle: 'Cupertino',
        trailing: CupertinoDemoDocumentationButton(CupertinoSwitchDemo.routeName),
      ),
      child: DefaultTextStyle(
        style: CupertinoTheme.of(context).textTheme.textStyle,
        child: SafeArea(
          child: Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: <Widget>[
                Semantics(
                  container: true,
                  child: Column(
                    children: <Widget>[
                      CupertinoSwitch(
                        value: _switchValue,
                        onChanged: (bool value) {
                          setState(() {
                            _switchValue = value;
                          });
                        },
                      ),
                      Text("Enabled - ${_switchValue ? "On" : "Off"}"),
                    ],
                  ),
                ),
                Semantics(
                  container: true,
                  child: const Column(
                    children: <Widget>[
                      CupertinoSwitch(value: true, onChanged: null),
                      Text('Disabled - On'),
                    ],
                  ),
                ),
                Semantics(
                  container: true,
                  child: const Column(
                    children: <Widget>[
                      CupertinoSwitch(value: false, onChanged: null),
                      Text('Disabled - Off'),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
