// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:intl/intl.dart';

// This demo is based on
// https://material.io/design/components/dialogs.html#full-screen-dialog

enum DismissDialogAction { cancel, discard, save }

class DateTimeItem extends StatelessWidget {
  DateTimeItem({super.key, required DateTime dateTime, required this.onChanged})
    : date = DateTime(dateTime.year, dateTime.month, dateTime.day),
      time = TimeOfDay(hour: dateTime.hour, minute: dateTime.minute);

  final DateTime date;
  final TimeOfDay time;
  final ValueChanged<DateTime> onChanged;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);

    return DefaultTextStyle(
      style: theme.textTheme.titleMedium!,
      child: Row(
        children: <Widget>[
          Expanded(
            child: Container(
              padding: const EdgeInsets.symmetric(vertical: 8.0),
              decoration: BoxDecoration(
                border: Border(bottom: BorderSide(color: theme.dividerColor)),
              ),
              child: InkWell(
                onTap: () {
                  showDatePicker(
                    context: context,
                    initialDate: date,
                    firstDate: date.subtract(const Duration(days: 30)),
                    lastDate: date.add(const Duration(days: 30)),
                  ).then((DateTime? value) {
                    if (value != null) {
                      onChanged(
                        DateTime(value.year, value.month, value.day, time.hour, time.minute),
                      );
                    }
                  });
                },
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: <Widget>[
                    Text(DateFormat('EEE, MMM d yyyy').format(date)),
                    const Icon(Icons.arrow_drop_down, color: Colors.black54),
                  ],
                ),
              ),
            ),
          ),
          Container(
            margin: const EdgeInsets.only(left: 8.0),
            padding: const EdgeInsets.symmetric(vertical: 8.0),
            decoration: BoxDecoration(
              border: Border(bottom: BorderSide(color: theme.dividerColor)),
            ),
            child: InkWell(
              onTap: () {
                showTimePicker(context: context, initialTime: time).then((TimeOfDay? value) {
                  if (value != null) {
                    onChanged(DateTime(date.year, date.month, date.day, value.hour, value.minute));
                  }
                });
              },
              child: Row(
                children: <Widget>[
                  Text(time.format(context)),
                  const Icon(Icons.arrow_drop_down, color: Colors.black54),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class FullScreenDialogDemo extends StatefulWidget {
  const FullScreenDialogDemo({super.key});

  @override
  FullScreenDialogDemoState createState() => FullScreenDialogDemoState();
}

class FullScreenDialogDemoState extends State<FullScreenDialogDemo> {
  DateTime _fromDateTime = DateTime.now();
  DateTime _toDateTime = DateTime.now();
  bool? _allDayValue = false;
  bool _saveNeeded = false;
  bool _hasLocation = false;
  bool _hasName = false;
  late String _eventName;

  Future<void> _handlePopInvoked(bool didPop, Object? result) async {
    if (didPop) {
      return;
    }

    final ThemeData theme = Theme.of(context);
    final TextStyle dialogTextStyle = theme.textTheme.titleMedium!.copyWith(
      color: theme.textTheme.bodySmall!.color,
    );

    final bool? shouldDiscard = await showDialog<bool>(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          content: Text('Discard new event?', style: dialogTextStyle),
          actions: <Widget>[
            TextButton(
              child: const Text('CANCEL'),
              onPressed: () {
                // Pop the confirmation dialog and indicate that the page should
                // not be popped.
                Navigator.of(context).pop(false);
              },
            ),
            TextButton(
              child: const Text('DISCARD'),
              onPressed: () {
                // Pop the confirmation dialog and indicate that the page should
                // be popped, too.
                Navigator.of(context).pop(true);
              },
            ),
          ],
        );
      },
    );

    if (shouldDiscard ?? false) {
      // Since this is the root route, quit the app where possible by invoking
      // the SystemNavigator. If this wasn't the root route, then
      // Navigator.maybePop could be used instead.
      // See https://github.com/flutter/flutter/issues/11490
      SystemNavigator.pop();
    }
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);

    return Scaffold(
      appBar: AppBar(
        title: Text(_hasName ? _eventName : 'Event Name TBD'),
        actions: <Widget>[
          TextButton(
            child: Text('SAVE', style: theme.textTheme.bodyMedium!.copyWith(color: Colors.white)),
            onPressed: () {
              Navigator.pop(context, DismissDialogAction.save);
            },
          ),
        ],
      ),
      body: Form(
        canPop: !_saveNeeded && !_hasLocation && !_hasName,
        onPopInvokedWithResult: _handlePopInvoked,
        child: Scrollbar(
          child: ListView(
            primary: true,
            padding: const EdgeInsets.all(16.0),
            children:
                <Widget>[
                  Container(
                    padding: const EdgeInsets.symmetric(vertical: 8.0),
                    alignment: Alignment.bottomLeft,
                    child: TextField(
                      decoration: const InputDecoration(labelText: 'Event name', filled: true),
                      style: theme.textTheme.headlineSmall,
                      onChanged: (String value) {
                        setState(() {
                          _hasName = value.isNotEmpty;
                          if (_hasName) {
                            _eventName = value;
                          }
                        });
                      },
                    ),
                  ),
                  Container(
                    padding: const EdgeInsets.symmetric(vertical: 8.0),
                    alignment: Alignment.bottomLeft,
                    child: TextField(
                      decoration: const InputDecoration(
                        labelText: 'Location',
                        hintText: 'Where is the event?',
                        filled: true,
                      ),
                      onChanged: (String value) {
                        setState(() {
                          _hasLocation = value.isNotEmpty;
                        });
                      },
                    ),
                  ),
                  Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      Text('From', style: theme.textTheme.bodySmall),
                      DateTimeItem(
                        dateTime: _fromDateTime,
                        onChanged: (DateTime value) {
                          setState(() {
                            _fromDateTime = value;
                            _saveNeeded = true;
                          });
                        },
                      ),
                    ],
                  ),
                  Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      Text('To', style: theme.textTheme.bodySmall),
                      DateTimeItem(
                        dateTime: _toDateTime,
                        onChanged: (DateTime value) {
                          setState(() {
                            _toDateTime = value;
                            _saveNeeded = true;
                          });
                        },
                      ),
                      const Text('All-day'),
                    ],
                  ),
                  Container(
                    decoration: BoxDecoration(
                      border: Border(bottom: BorderSide(color: theme.dividerColor)),
                    ),
                    child: Row(
                      children: <Widget>[
                        Checkbox(
                          value: _allDayValue,
                          onChanged: (bool? value) {
                            setState(() {
                              _allDayValue = value;
                              _saveNeeded = true;
                            });
                          },
                        ),
                        const Text('All-day'),
                      ],
                    ),
                  ),
                ].map<Widget>((Widget child) {
                  return Container(
                    padding: const EdgeInsets.symmetric(vertical: 8.0),
                    height: 96.0,
                    child: child,
                  );
                }).toList(),
          ),
        ),
      ),
    );
  }
}
