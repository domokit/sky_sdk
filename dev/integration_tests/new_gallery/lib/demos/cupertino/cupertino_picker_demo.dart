// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:intl/intl.dart';

import '../../gallery_localizations.dart';

// BEGIN cupertinoPickersDemo

class CupertinoPickerDemo extends StatefulWidget {
  const CupertinoPickerDemo({super.key});

  @override
  State<CupertinoPickerDemo> createState() => _CupertinoPickerDemoState();
}

class _CupertinoPickerDemoState extends State<CupertinoPickerDemo> {
  Duration timer = Duration.zero;

  // Value that is shown in the date picker in date mode.
  DateTime date = DateTime.now();

  // Value that is shown in the date picker in time mode.
  DateTime time = DateTime.now();

  // Value that is shown in the date picker in dateAndTime mode.
  DateTime dateTime = DateTime.now();

  int _selectedWeekday = 0;

  static List<String> getDaysOfWeek([String? locale]) {
    final DateTime now = DateTime.now();
    final DateTime firstDayOfWeek = now.subtract(Duration(days: now.weekday - 1));
    return List<int>.generate(7, (int index) => index)
        .map(
          (int value) => DateFormat(
            DateFormat.WEEKDAY,
            locale,
          ).format(firstDayOfWeek.add(Duration(days: value))),
        )
        .toList();
  }

  void _showDemoPicker({required BuildContext context, required Widget child}) {
    final CupertinoThemeData themeData = CupertinoTheme.of(context);
    final CupertinoTheme dialogBody = CupertinoTheme(data: themeData, child: child);

    showCupertinoModalPopup<void>(context: context, builder: (BuildContext context) => dialogBody);
  }

  Widget _buildDatePicker(BuildContext context) {
    return MouseRegion(
      cursor: SystemMouseCursors.click,
      child: GestureDetector(
        onTap: () {
          _showDemoPicker(
            context: context,
            child: _BottomPicker(
              child: CupertinoDatePicker(
                backgroundColor: CupertinoColors.systemBackground.resolveFrom(context),
                mode: CupertinoDatePickerMode.date,
                initialDateTime: date,
                onDateTimeChanged: (DateTime newDateTime) {
                  setState(() => date = newDateTime);
                },
              ),
            ),
          );
        },
        child: _Menu(
          children: <Widget>[
            Text(GalleryLocalizations.of(context)!.demoCupertinoPickerDate),
            Text(
              DateFormat.yMMMMd().format(date),
              style: const TextStyle(color: CupertinoColors.inactiveGray),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildTimePicker(BuildContext context) {
    return MouseRegion(
      cursor: SystemMouseCursors.click,
      child: GestureDetector(
        onTap: () {
          _showDemoPicker(
            context: context,
            child: _BottomPicker(
              child: CupertinoDatePicker(
                backgroundColor: CupertinoColors.systemBackground.resolveFrom(context),
                mode: CupertinoDatePickerMode.time,
                initialDateTime: time,
                onDateTimeChanged: (DateTime newDateTime) {
                  setState(() => time = newDateTime);
                },
              ),
            ),
          );
        },
        child: _Menu(
          children: <Widget>[
            Text(GalleryLocalizations.of(context)!.demoCupertinoPickerTime),
            Text(
              DateFormat.jm().format(time),
              style: const TextStyle(color: CupertinoColors.inactiveGray),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildDateAndTimePicker(BuildContext context) {
    return MouseRegion(
      cursor: SystemMouseCursors.click,
      child: GestureDetector(
        onTap: () {
          _showDemoPicker(
            context: context,
            child: _BottomPicker(
              child: CupertinoDatePicker(
                backgroundColor: CupertinoColors.systemBackground.resolveFrom(context),
                initialDateTime: dateTime,
                onDateTimeChanged: (DateTime newDateTime) {
                  setState(() => dateTime = newDateTime);
                },
              ),
            ),
          );
        },
        child: _Menu(
          children: <Widget>[
            Text(GalleryLocalizations.of(context)!.demoCupertinoPickerDateTime),
            Flexible(
              child: Text(
                DateFormat.yMMMd().add_jm().format(dateTime),
                style: const TextStyle(color: CupertinoColors.inactiveGray),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildCountdownTimerPicker(BuildContext context) {
    return MouseRegion(
      cursor: SystemMouseCursors.click,
      child: GestureDetector(
        onTap: () {
          _showDemoPicker(
            context: context,
            child: _BottomPicker(
              child: CupertinoTimerPicker(
                backgroundColor: CupertinoColors.systemBackground.resolveFrom(context),
                initialTimerDuration: timer,
                onTimerDurationChanged: (Duration newTimer) {
                  setState(() => timer = newTimer);
                },
              ),
            ),
          );
        },
        child: _Menu(
          children: <Widget>[
            Text(GalleryLocalizations.of(context)!.demoCupertinoPickerTimer),
            Text(
              '${timer.inHours}:'
              '${(timer.inMinutes % 60).toString().padLeft(2, '0')}:'
              '${(timer.inSeconds % 60).toString().padLeft(2, '0')}',
              style: const TextStyle(color: CupertinoColors.inactiveGray),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildPicker(BuildContext context) {
    final String? locale = GalleryLocalizations.of(context)?.localeName;
    final List<String> daysOfWeek = getDaysOfWeek(locale);
    return MouseRegion(
      cursor: SystemMouseCursors.click,
      child: GestureDetector(
        onTap: () {
          _showDemoPicker(
            context: context,
            child: _BottomPicker(
              child: CupertinoPicker(
                backgroundColor: CupertinoColors.systemBackground.resolveFrom(context),
                itemExtent: 32.0,
                magnification: 1.22,
                squeeze: 1.2,
                useMagnifier: true,
                // This is called when selected item is changed.
                onSelectedItemChanged: (int selectedItem) {
                  setState(() {
                    _selectedWeekday = selectedItem;
                  });
                },
                children: List<Widget>.generate(daysOfWeek.length, (int index) {
                  return Center(child: Text(daysOfWeek[index]));
                }),
              ),
            ),
          );
        },
        child: _Menu(
          children: <Widget>[
            Text(GalleryLocalizations.of(context)!.demoCupertinoPicker),
            Text(
              daysOfWeek[_selectedWeekday],
              style: const TextStyle(color: CupertinoColors.inactiveGray),
            ),
          ],
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        automaticallyImplyLeading: false,
        middle: Text(GalleryLocalizations.of(context)!.demoCupertinoPickerTitle),
      ),
      child: DefaultTextStyle(
        style: CupertinoTheme.of(context).textTheme.textStyle,
        child: ListView(
          children: <Widget>[
            const SizedBox(height: 32),
            _buildDatePicker(context),
            _buildTimePicker(context),
            _buildDateAndTimePicker(context),
            _buildCountdownTimerPicker(context),
            _buildPicker(context),
          ],
        ),
      ),
    );
  }
}

class _BottomPicker extends StatelessWidget {
  const _BottomPicker({required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) {
    return Container(
      height: 216,
      padding: const EdgeInsets.only(top: 6),
      margin: EdgeInsets.only(bottom: MediaQuery.of(context).viewInsets.bottom),
      color: CupertinoColors.systemBackground.resolveFrom(context),
      child: DefaultTextStyle(
        style: TextStyle(color: CupertinoColors.label.resolveFrom(context), fontSize: 22),
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
      decoration: const BoxDecoration(
        border: Border(
          top: BorderSide(color: CupertinoColors.inactiveGray, width: 0),
          bottom: BorderSide(color: CupertinoColors.inactiveGray, width: 0),
        ),
      ),
      height: 44,
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16),
        child: Row(mainAxisAlignment: MainAxisAlignment.spaceBetween, children: children),
      ),
    );
  }
}

// END
