// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'android_platform_view.dart';
import 'future_data_handler.dart';
import 'page.dart';

class NestedViewEventPage extends PageWidget {
  const NestedViewEventPage({Key? key})
    : super('Nested View Event Tests', const ValueKey<String>('NestedViewEventTile'), key: key);

  @override
  Widget build(BuildContext context) => const NestedViewEventBody();
}

class NestedViewEventBody extends StatefulWidget {
  const NestedViewEventBody({super.key});

  @override
  State<NestedViewEventBody> createState() => NestedViewEventBodyState();
}

enum _LastTestStatus { pending, success, error }

class NestedViewEventBodyState extends State<NestedViewEventBody> {
  MethodChannel? viewChannel;
  _LastTestStatus _lastTestStatus = _LastTestStatus.pending;
  String? lastError;
  int? id;
  int nestedViewClickCount = 0;
  bool showPlatformView = true;
  bool useHybridComposition = false;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Nested view event')),
      body: Column(
        children: <Widget>[
          SizedBox(
            height: 300,
            child: Stack(
              alignment: Alignment.topCenter,
              children: <Widget>[
                if (showPlatformView)
                  AndroidPlatformView(
                    key: const ValueKey<String>('PlatformView'),
                    viewType: 'simple_view',
                    onPlatformViewCreated: onPlatformViewCreated,
                    useHybridComposition: useHybridComposition,
                  ),
                // The overlapping widget stabilizes the view tree by ensuring
                // that there is widget content on top of the platform view.
                // Without this widget, we rely on the UI elements down below
                // to "incidentally" draw on top of the PlatformView which
                // is not a reliable behavior as we eliminate non-visible
                // rendering operations throughout the framework and engine.
                const Positioned(
                  top: 50,
                  child: Text('overlapping widget', style: TextStyle(color: Colors.yellow)),
                ),
              ],
            ),
          ),
          if (_lastTestStatus != _LastTestStatus.pending) _statusWidget(),
          if (viewChannel != null) ...<Widget>[
            Row(
              children: <Widget>[
                Expanded(
                  child: ElevatedButton(
                    key: const ValueKey<String>('ShowAlertDialog'),
                    onPressed: onShowAlertDialogPressed,
                    child: const Text('SHOW ALERT DIALOG'),
                  ),
                ),
                Expanded(
                  child: ElevatedButton(
                    key: const ValueKey<String>('TogglePlatformView'),
                    onPressed: onTogglePlatformView,
                    child: const Text('TOGGLE PLATFORM VIEW'),
                  ),
                ),
              ],
            ),
            Row(
              children: <Widget>[
                Expanded(
                  child: ElevatedButton(
                    key: const ValueKey<String>('ToggleHybridComposition'),
                    child: const Text('TOGGLE HC'),
                    onPressed: () {
                      setState(() {
                        useHybridComposition = !useHybridComposition;
                      });
                    },
                  ),
                ),
                Expanded(
                  child: ElevatedButton(
                    key: const ValueKey<String>('AddChildView'),
                    onPressed: onChildViewPressed,
                    child: const Text('ADD CHILD VIEW'),
                  ),
                ),
                Expanded(
                  child: ElevatedButton(
                    key: const ValueKey<String>('TapChildView'),
                    onPressed: onTapChildViewPressed,
                    child: const Text('TAP CHILD VIEW'),
                  ),
                ),
              ],
            ),
            if (nestedViewClickCount > 0)
              Text(
                'Click count: $nestedViewClickCount',
                key: const ValueKey<String>('NestedViewClickCount'),
              ),
          ],
        ],
      ),
    );
  }

  Widget _statusWidget() {
    assert(_lastTestStatus != _LastTestStatus.pending);
    final String message = _lastTestStatus == _LastTestStatus.success ? 'Success' : lastError!;
    return ColoredBox(
      color: _lastTestStatus == _LastTestStatus.success ? Colors.green : Colors.red,
      child: Text(
        message,
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
      await viewChannel!.invokeMethod<void>('showAndHideAlertDialog');
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

  Future<void> onTogglePlatformView() async {
    setState(() {
      showPlatformView = !showPlatformView;
    });
  }

  Future<void> onChildViewPressed() async {
    try {
      await viewChannel!.invokeMethod<void>('addChildViewAndWaitForClick');
      setState(() {
        nestedViewClickCount++;
      });
    } catch (e) {
      setState(() {
        _lastTestStatus = _LastTestStatus.error;
        lastError = '$e';
      });
    }
  }

  Future<void> onTapChildViewPressed() async {
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
    driverDataHandler
        .registerHandler('hierarchy')
        .complete(() async => (await channel.invokeMethod<String>('getViewHierarchy'))!);
  }
}
