// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

const String _introText =
    'Tooltips are short identifying messages that briefly appear in response to '
    'a long press. Tooltip messages are also used by services that make Flutter '
    'apps accessible, like screen readers.';

class TooltipDemo extends StatelessWidget {
  const TooltipDemo({super.key});

  static const String routeName = '/material/tooltips';

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    return Scaffold(
      appBar: AppBar(
        title: const Text('Tooltips'),
        actions: <Widget>[MaterialDemoDocumentationButton(routeName)],
      ),
      body: Builder(
        builder: (BuildContext context) {
          return SafeArea(
            top: false,
            bottom: false,
            child: ListView(
              children:
                  <Widget>[
                    Text(_introText, style: theme.textTheme.titleMedium),
                    Row(
                      children: <Widget>[
                        Text('Long press the ', style: theme.textTheme.titleMedium),
                        Tooltip(
                          message: 'call icon',
                          child: Icon(Icons.call, size: 18.0, color: theme.iconTheme.color),
                        ),
                        Text(' icon.', style: theme.textTheme.titleMedium),
                      ],
                    ),
                    Center(
                      child: IconButton(
                        iconSize: 48.0,
                        icon: const Icon(Icons.call),
                        color: theme.iconTheme.color,
                        tooltip: 'Place a phone call',
                        onPressed: () {
                          ScaffoldMessenger.of(context).showSnackBar(
                            const SnackBar(content: Text('That was an ordinary tap.')),
                          );
                        },
                      ),
                    ),
                  ].map<Widget>((Widget widget) {
                    return Padding(
                      padding: const EdgeInsets.only(top: 16.0, left: 16.0, right: 16.0),
                      child: widget,
                    );
                  }).toList(),
            ),
          );
        },
      ),
    );
  }
}
