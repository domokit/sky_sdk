// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

enum BannerDemoAction { reset, showMultipleActions, showLeading }

class BannerDemo extends StatefulWidget {
  const BannerDemo({super.key});

  static const String routeName = '/material/banner';

  @override
  State<BannerDemo> createState() => _BannerDemoState();
}

class _BannerDemoState extends State<BannerDemo> {
  static const int _numItems = 20;
  bool _displayBanner = true;
  bool _showMultipleActions = true;
  bool _showLeading = true;

  void handleDemoAction(BannerDemoAction action) {
    setState(() {
      switch (action) {
        case BannerDemoAction.reset:
          _displayBanner = true;
          _showMultipleActions = true;
          _showLeading = true;
        case BannerDemoAction.showMultipleActions:
          _showMultipleActions = !_showMultipleActions;
        case BannerDemoAction.showLeading:
          _showLeading = !_showLeading;
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    final Widget banner = MaterialBanner(
      content: const Text('Your password was updated on your other device. Please sign in again.'),
      leading: _showLeading ? const CircleAvatar(child: Icon(Icons.access_alarm)) : null,
      actions: <Widget>[
        TextButton(
          child: const Text('SIGN IN'),
          onPressed: () {
            setState(() {
              _displayBanner = false;
            });
          },
        ),
        if (_showMultipleActions)
          TextButton(
            child: const Text('DISMISS'),
            onPressed: () {
              setState(() {
                _displayBanner = false;
              });
            },
          ),
      ],
    );

    return Scaffold(
      appBar: AppBar(
        title: const Text('Banner'),
        actions: <Widget>[
          MaterialDemoDocumentationButton(BannerDemo.routeName),
          PopupMenuButton<BannerDemoAction>(
            onSelected: handleDemoAction,
            itemBuilder:
                (BuildContext context) => <PopupMenuEntry<BannerDemoAction>>[
                  const PopupMenuItem<BannerDemoAction>(
                    value: BannerDemoAction.reset,
                    child: Text('Reset the banner'),
                  ),
                  const PopupMenuDivider(),
                  CheckedPopupMenuItem<BannerDemoAction>(
                    value: BannerDemoAction.showMultipleActions,
                    checked: _showMultipleActions,
                    child: const Text('Multiple actions'),
                  ),
                  CheckedPopupMenuItem<BannerDemoAction>(
                    value: BannerDemoAction.showLeading,
                    checked: _showLeading,
                    child: const Text('Leading icon'),
                  ),
                ],
          ),
        ],
      ),
      body: ListView.builder(
        itemCount: _displayBanner ? _numItems + 1 : _numItems,
        itemBuilder: (BuildContext context, int index) {
          if (index == 0 && _displayBanner) {
            return banner;
          }
          return ListTile(title: Text('Item ${_displayBanner ? index : index + 1}'));
        },
      ),
    );
  }
}
