// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

import '../../gallery/demo.dart';

class CupertinoButtonsDemo extends StatefulWidget {
  const CupertinoButtonsDemo({super.key});

  static const String routeName = '/cupertino/buttons';

  @override
  State<CupertinoButtonsDemo> createState() => _CupertinoButtonDemoState();
}

class _CupertinoButtonDemoState extends State<CupertinoButtonsDemo> {
  int _pressedCount = 0;

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        middle: const Text('Buttons'),
        // We're specifying a back label here because the previous page is a
        // Material page. CupertinoPageRoutes could auto-populate these back
        // labels.
        previousPageTitle: 'Cupertino',
        trailing: CupertinoDemoDocumentationButton(CupertinoButtonsDemo.routeName),
      ),
      child: DefaultTextStyle(
        style: CupertinoTheme.of(context).textTheme.textStyle,
        child: SafeArea(
          child: Column(
            children: <Widget>[
              const Padding(
                padding: EdgeInsets.all(16.0),
                child: Text(
                  'iOS themed buttons are flat. They can have borders or backgrounds but '
                  'only when necessary.',
                ),
              ),
              Expanded(
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: <Widget>[
                    Text(
                      _pressedCount > 0
                          ? 'Button pressed $_pressedCount time${_pressedCount == 1 ? "" : "s"}'
                          : ' ',
                    ),
                    const Padding(padding: EdgeInsets.all(12.0)),
                    Align(
                      alignment: const Alignment(0.0, -0.2),
                      child: Row(
                        mainAxisSize: MainAxisSize.min,
                        children: <Widget>[
                          CupertinoButton(
                            child: const Text('Cupertino Button'),
                            onPressed: () {
                              setState(() {
                                _pressedCount += 1;
                              });
                            },
                          ),
                          const CupertinoButton(onPressed: null, child: Text('Disabled')),
                        ],
                      ),
                    ),
                    const Padding(padding: EdgeInsets.all(12.0)),
                    CupertinoButton.filled(
                      child: const Text('With Background'),
                      onPressed: () {
                        setState(() {
                          _pressedCount += 1;
                        });
                      },
                    ),
                    const Padding(padding: EdgeInsets.all(12.0)),
                    const CupertinoButton.filled(onPressed: null, child: Text('Disabled')),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
