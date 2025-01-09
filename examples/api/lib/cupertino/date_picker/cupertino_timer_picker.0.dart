// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoTimerPicker].

void main() => runApp(const TimerPickerApp());

class TimerPickerApp extends StatelessWidget {
  const TimerPickerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: TimerPickerExample(),
    );
  }
}

class TimerPickerExample extends StatefulWidget {
  const TimerPickerExample({super.key});

  @override
  State<TimerPickerExample> createState() => _TimerPickerExampleState();
}

class _TimerPickerExampleState extends State<TimerPickerExample> {
  Duration duration = const Duration(hours: 1, minutes: 23);

  // This shows a CupertinoModalPopup with a reasonable fixed height which hosts
  // a CupertinoTimerPicker.
  void _showDialog(Widget child) {
    showCupertinoModalPopup<void>(
      context: context,
      builder:
          (BuildContext context) => Container(
            height: 216,
            padding: const EdgeInsets.only(top: 6.0),
            // The bottom margin is provided to align the popup above the system
            // navigation bar.
            margin: EdgeInsets.only(bottom: MediaQuery.of(context).viewInsets.bottom),
            // Provide a background color for the popup.
            color: CupertinoColors.systemBackground.resolveFrom(context),
            // Use a SafeArea widget to avoid system overlaps.
            child: SafeArea(top: false, child: child),
          ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(middle: Text('CupertinoTimerPicker Sample')),
      child: DefaultTextStyle(
        style: TextStyle(color: CupertinoColors.label.resolveFrom(context), fontSize: 22.0),
        child: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              _TimerPickerItem(
                children: <Widget>[
                  const Text('Timer'),
                  CupertinoButton(
                    // Display a CupertinoTimerPicker with hour/minute mode.
                    onPressed:
                        () => _showDialog(
                          CupertinoTimerPicker(
                            mode: CupertinoTimerPickerMode.hm,
                            initialTimerDuration: duration,
                            // This is called when the user changes the timer's
                            // duration.
                            onTimerDurationChanged: (Duration newDuration) {
                              setState(() => duration = newDuration);
                            },
                          ),
                        ),
                    // In this example, the timer's value is formatted manually.
                    // You can use the intl package to format the value based on
                    // the user's locale settings.
                    child: Text('$duration', style: const TextStyle(fontSize: 22.0)),
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }
}

// This class simply decorates a row of widgets.
class _TimerPickerItem extends StatelessWidget {
  const _TimerPickerItem({required this.children});

  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    return DecoratedBox(
      decoration: const BoxDecoration(
        border: Border(
          top: BorderSide(color: CupertinoColors.inactiveGray, width: 0.0),
          bottom: BorderSide(color: CupertinoColors.inactiveGray, width: 0.0),
        ),
      ),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16.0),
        child: Row(mainAxisAlignment: MainAxisAlignment.spaceBetween, children: children),
      ),
    );
  }
}
