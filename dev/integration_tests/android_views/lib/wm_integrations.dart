// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'page.dart';

class WindowManagerIntegrationsPage extends PageWidget {
  const WindowManagerIntegrationsPage({Key? key})
    : super(
        'Window Manager Integrations Tests',
        const ValueKey<String>('WmIntegrationsListTile'),
        key: key,
      );

  @override
  Widget build(BuildContext context) => const WindowManagerBody();
}

class WindowManagerBody extends StatefulWidget {
  const WindowManagerBody({super.key});

  @override
  State<WindowManagerBody> createState() => WindowManagerBodyState();
}

enum _LastTestStatus { pending, success, error }

class WindowManagerBodyState extends State<WindowManagerBody> {
  MethodChannel? viewChannel;
  _LastTestStatus _lastTestStatus = _LastTestStatus.pending;
  String? lastError;
  int? id;
  int windowClickCount = 0;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Window Manager Integrations')),
      body: Column(
        children: <Widget>[
          SizedBox(
            height: 300,
            child: AndroidView(
              viewType: 'simple_view',
              onPlatformViewCreated: onPlatformViewCreated,
            ),
          ),
          if (_lastTestStatus != _LastTestStatus.pending) _statusWidget(),
          if (viewChannel != null) ...<Widget>[
            ElevatedButton(
              key: const ValueKey<String>('ShowAlertDialog'),
              onPressed: onShowAlertDialogPressed,
              child: const Text('SHOW ALERT DIALOG'),
            ),
            Row(
              children: <Widget>[
                ElevatedButton(
                  key: const ValueKey<String>('AddWindow'),
                  onPressed: onAddWindowPressed,
                  child: const Text('ADD WINDOW'),
                ),
                ElevatedButton(
                  key: const ValueKey<String>('TapWindow'),
                  onPressed: onTapWindowPressed,
                  child: const Text('TAP WINDOW'),
                ),
                if (windowClickCount > 0)
                  Text(
                    'Click count: $windowClickCount',
                    key: const ValueKey<String>('WindowClickCount'),
                  ),
              ],
            ),
          ],
        ],
      ),
    );
  }

  Widget _statusWidget() {
    assert(_lastTestStatus != _LastTestStatus.pending);
    final String? message = _lastTestStatus == _LastTestStatus.success ? 'Success' : lastError;
    return ColoredBox(
      color: _lastTestStatus == _LastTestStatus.success ? Colors.green : Colors.red,
      child: Text(
        message!,
        key: const ValueKey<String>('Status'),
        style: TextStyle(color: _lastTestStatus == _LastTestStatus.error ? Colors.yellow : null),
      ),
    );
  }

  Future<void> onShowAlertDialogPressed() async {
    if (_lastTestStatus != _LastTestStatus.pending) {
      setState(() {
        _lastTestStatus = _LastTestStatus.pending;
      });
    }
    try {
      await viewChannel?.invokeMethod<void>('showAndHideAlertDialog');
      setState(() {
        _lastTestStatus = _LastTestStatus.success;
      });
    } catch (e) {
      setState(() {
        _lastTestStatus = _LastTestStatus.error;
        lastError = '$e';
      });
    }
  }

  Future<void> onAddWindowPressed() async {
    try {
      await viewChannel?.invokeMethod<void>('addWindowAndWaitForClick');
      setState(() {
        windowClickCount++;
      });
    } catch (e) {
      setState(() {
        _lastTestStatus = _LastTestStatus.error;
        lastError = '$e';
      });
    }
  }

  Future<void> onTapWindowPressed() async {
    await Future<void>.delayed(const Duration(seconds: 1));

    // Dispatch a tap event on the child view inside the platform view.
    //
    // Android mutates `MotionEvent` instances, so in this case *do not* dispatch
    // new instances as it won't cover the `MotionEventTracker` class in the embedding
    // which tracks events.
    //
    // See the issue this prevents: https://github.com/flutter/flutter/issues/61169
    await Process.run('input', const <String>['tap', '250', '550']);
  }

  void onPlatformViewCreated(int id) {
    this.id = id;
    setState(() {
      viewChannel = MethodChannel('simple_view/$id');
    });
  }
}
