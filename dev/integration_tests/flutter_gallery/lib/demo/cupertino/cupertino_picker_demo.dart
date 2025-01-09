// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:intl/intl.dart';

import '../../gallery/demo.dart';
import 'cupertino_navigation_demo.dart' show coolColorNames;

const double _kPickerSheetHeight = 216.0;
const double _kPickerItemHeight = 32.0;

class CupertinoPickerDemo extends StatefulWidget {
  const CupertinoPickerDemo({super.key});

  static const String routeName = '/cupertino/picker';

  @override
  State<CupertinoPickerDemo> createState() => _CupertinoPickerDemoState();
}

class _BottomPicker extends StatelessWidget {
  const _BottomPicker({required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) {
    return Container(
      height: _kPickerSheetHeight,
      padding: const EdgeInsets.only(top: 6.0),
      color: CupertinoColors.systemBackground.resolveFrom(context),
      child: DefaultTextStyle(
        style: TextStyle(color: CupertinoColors.label.resolveFrom(context), fontSize: 22.0),
        child: GestureDetector(
          // Blocks taps from propagating to the modal sheet and popping.
          onTap: () {},
          child: SafeArea(top: false, child: child),
        ),
      ),
    );
  }
}

class _Menu extends StatelessWidget {
  const _Menu({required this.children});

  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: CupertinoTheme.of(context).scaffoldBackgroundColor,
        border: const Border(
          top: BorderSide(color: Color(0xFFBCBBC1), width: 0.0),
          bottom: BorderSide(color: Color(0xFFBCBBC1), width: 0.0),
        ),
      ),
      height: 44.0,
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16.0),
        child: SafeArea(
          top: false,
          bottom: false,
          child: Row(mainAxisAlignment: MainAxisAlignment.spaceBetween, children: children),
        ),
      ),
    );
  }
}

class _CupertinoPickerDemoState extends State<CupertinoPickerDemo> {
  int _selectedColorIndex = 0;

  Duration timer = Duration.zero;

  // Value that is shown in the date picker in date mode.
  DateTime date = DateTime.now();

  // Value that is shown in the date picker in time mode.
  DateTime time = DateTime.now();

  // Value that is shown in the date picker in dateAndTime mode.
  DateTime dateTime = DateTime.now();

  Widget _buildColorPicker(BuildContext context) {
    final FixedExtentScrollController scrollController = FixedExtentScrollController(
      initialItem: _selectedColorIndex,
    );

    return GestureDetector(
      onTap: () async {
        await showCupertinoModalPopup<void>(
          context: context,
          semanticsDismissible: true,
          builder: (BuildContext context) {
            return _BottomPicker(
              child: CupertinoPicker(
                scrollController: scrollController,
                itemExtent: _kPickerItemHeight,
                backgroundColor: CupertinoColors.systemBackground.resolveFrom(context),
                onSelectedItemChanged: (int index) {
                  setState(() => _selectedColorIndex = index);
                },
                children: List<Widget>.generate(coolColorNames.length, (int index) {
                  return Center(child: Text(coolColorNames[index]));
                }),
              ),
            );
          },
        );
      },
      child: _Menu(
        children: <Widget>[
          const Text('Favorite Color'),
          Text(
            coolColorNames[_selectedColorIndex],
            style: TextStyle(
              color: CupertinoDynamicColor.resolve(CupertinoColors.inactiveGray, context),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildCountdownTimerPicker(BuildContext context) {
    return GestureDetector(
      onTap: () {
        showCupertinoModalPopup<void>(
          context: context,
          semanticsDismissible: true,
          builder: (BuildContext context) {
            return _BottomPicker(
              child: CupertinoTimerPicker(
                backgroundColor: CupertinoColors.systemBackground.resolveFrom(context),
                initialTimerDuration: timer,
                onTimerDurationChanged: (Duration newTimer) {
                  setState(() => timer = newTimer);
                },
              ),
            );
          },
        );
      },
      child: _Menu(
        children: <Widget>[
          const Text('Countdown Timer'),
          Text(
            '${timer.inHours}:'
            '${(timer.inMinutes % 60).toString().padLeft(2, '0')}:'
            '${(timer.inSeconds % 60).toString().padLeft(2, '0')}',
            style: TextStyle(
              color: CupertinoDynamicColor.resolve(CupertinoColors.inactiveGray, context),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildDatePicker(BuildContext context) {
    return GestureDetector(
      onTap: () {
        showCupertinoModalPopup<void>(
          context: context,
          semanticsDismissible: true,
          builder: (BuildContext context) {
            return _BottomPicker(
              child: CupertinoDatePicker(
                backgroundColor: CupertinoColors.systemBackground.resolveFrom(context),
                mode: CupertinoDatePickerMode.date,
                initialDateTime: date,
                onDateTimeChanged: (DateTime newDateTime) {
                  setState(() => date = newDateTime);
                },
              ),
            );
          },
        );
      },
      child: _Menu(
        children: <Widget>[
          const Text('Date'),
          Text(
            DateFormat.yMMMMd().format(date),
            style: TextStyle(
              color: CupertinoDynamicColor.resolve(CupertinoColors.inactiveGray, context),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildTimePicker(BuildContext context) {
    return GestureDetector(
      onTap: () {
        showCupertinoModalPopup<void>(
          context: context,
          semanticsDismissible: true,
          builder: (BuildContext context) {
            return _BottomPicker(
              child: CupertinoDatePicker(
                backgroundColor: CupertinoColors.systemBackground.resolveFrom(context),
                mode: CupertinoDatePickerMode.time,
                initialDateTime: time,
                onDateTimeChanged: (DateTime newDateTime) {
                  setState(() => time = newDateTime);
                },
              ),
            );
          },
        );
      },
      child: _Menu(
        children: <Widget>[
          const Text('Time'),
          Text(
            DateFormat.jm().format(time),
            style: TextStyle(
              color: CupertinoDynamicColor.resolve(CupertinoColors.inactiveGray, context),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildDateAndTimePicker(BuildContext context) {
    return GestureDetector(
      onTap: () {
        showCupertinoModalPopup<void>(
          context: context,
          semanticsDismissible: true,
          builder: (BuildContext context) {
            return _BottomPicker(
              child: CupertinoDatePicker(
                backgroundColor: CupertinoColors.systemBackground.resolveFrom(context),
                initialDateTime: dateTime,
                onDateTimeChanged: (DateTime newDateTime) {
                  setState(() => dateTime = newDateTime);
                },
              ),
            );
          },
        );
      },
      child: _Menu(
        children: <Widget>[
          const Text('Date and Time'),
          Text(
            DateFormat.yMMMd().add_jm().format(dateTime),
            style: TextStyle(
              color: CupertinoDynamicColor.resolve(CupertinoColors.inactiveGray, context),
            ),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        middle: const Text('Picker'),
        // We're specifying a back label here because the previous page is a
        // Material page. CupertinoPageRoutes could auto-populate these back
        // labels.
        previousPageTitle: 'Cupertino',
        trailing: CupertinoDemoDocumentationButton(CupertinoPickerDemo.routeName),
      ),
      child: DefaultTextStyle(
        style: CupertinoTheme.of(context).textTheme.textStyle,
        child: ListView(
          children: <Widget>[
            const Padding(padding: EdgeInsets.only(top: 32.0)),
            _buildColorPicker(context),
            _buildCountdownTimerPicker(context),
            _buildDatePicker(context),
            _buildTimePicker(context),
            _buildDateAndTimePicker(context),
          ],
        ),
      ),
    );
  }
}
