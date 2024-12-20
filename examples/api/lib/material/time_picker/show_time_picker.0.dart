// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [showTimePicker].

void main() {
  runApp(const ShowTimePickerApp());
}

class ShowTimePickerApp extends StatefulWidget {
  const ShowTimePickerApp({super.key});

  @override
  State<ShowTimePickerApp> createState() => _ShowTimePickerAppState();
}

class _ShowTimePickerAppState extends State<ShowTimePickerApp> {
  ThemeMode themeMode = ThemeMode.dark;
  bool useMaterial3 = true;

  void setThemeMode(ThemeMode mode) {
    setState(() {
      themeMode = mode;
    });
  }

  void setUseMaterial3(bool? value) {
    setState(() {
      useMaterial3 = value!;
    });
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData.light(useMaterial3: useMaterial3),
      darkTheme: ThemeData.dark(useMaterial3: useMaterial3),
      themeMode: themeMode,
      home: TimePickerOptions(
        themeMode: themeMode,
        useMaterial3: useMaterial3,
        setThemeMode: setThemeMode,
        setUseMaterial3: setUseMaterial3,
      ),
    );
  }
}

class TimePickerOptions extends StatefulWidget {
  const TimePickerOptions({
    super.key,
    required this.themeMode,
    required this.useMaterial3,
    required this.setThemeMode,
    required this.setUseMaterial3,
  });

  final ThemeMode themeMode;
  final bool useMaterial3;
  final ValueChanged<ThemeMode> setThemeMode;
  final ValueChanged<bool?> setUseMaterial3;

  @override
  State<TimePickerOptions> createState() => _TimePickerOptionsState();
}

class _TimePickerOptionsState extends State<TimePickerOptions> {
  TimeOfDay? selectedTime;
  TimePickerEntryMode entryMode = TimePickerEntryMode.dial;
  Orientation? orientation;
  TextDirection textDirection = TextDirection.ltr;
  MaterialTapTargetSize tapTargetSize = MaterialTapTargetSize.padded;
  bool use24HourTime = false;

  void _entryModeChanged(TimePickerEntryMode? value) {
    if (value != entryMode) {
      setState(() {
        entryMode = value!;
      });
    }
  }

  void _orientationChanged(Orientation? value) {
    if (value != orientation) {
      setState(() {
        orientation = value;
      });
    }
  }

  void _textDirectionChanged(TextDirection? value) {
    if (value != textDirection) {
      setState(() {
        textDirection = value!;
      });
    }
  }

  void _tapTargetSizeChanged(MaterialTapTargetSize? value) {
    if (value != tapTargetSize) {
      setState(() {
        tapTargetSize = value!;
      });
    }
  }

  void _use24HourTimeChanged(bool? value) {
    if (value != use24HourTime) {
      setState(() {
        use24HourTime = value!;
      });
    }
  }

  void _themeModeChanged(ThemeMode? value) {
    widget.setThemeMode(value!);
  }

  @override
  Widget build(BuildContext context) {
    return Material(
      child: Column(
        children: <Widget>[
          Expanded(
            child: GridView(
              gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
                maxCrossAxisExtent: 350,
                mainAxisSpacing: 4,
                mainAxisExtent: 200,
                crossAxisSpacing: 4,
              ),
              children: <Widget>[
                EnumCard<TimePickerEntryMode>(
                  choices: TimePickerEntryMode.values,
                  value: entryMode,
                  onChanged: _entryModeChanged,
                ),
                EnumCard<ThemeMode>(
                  choices: ThemeMode.values,
                  value: widget.themeMode,
                  onChanged: _themeModeChanged,
                ),
                EnumCard<TextDirection>(
                  choices: TextDirection.values,
                  value: textDirection,
                  onChanged: _textDirectionChanged,
                ),
                EnumCard<MaterialTapTargetSize>(
                  choices: MaterialTapTargetSize.values,
                  value: tapTargetSize,
                  onChanged: _tapTargetSizeChanged,
                ),
                ChoiceCard<Orientation?>(
                  choices: const <Orientation?>[...Orientation.values, null],
                  value: orientation,
                  title: '$Orientation',
                  choiceLabels: <Orientation?, String>{
                    for (final Orientation choice in Orientation.values) choice: choice.name,
                    null: 'from MediaQuery',
                  },
                  onChanged: _orientationChanged,
                ),
                ChoiceCard<bool>(
                  choices: const <bool>[false, true],
                  value: use24HourTime,
                  onChanged: _use24HourTimeChanged,
                  title: 'Time Mode',
                  choiceLabels: const <bool, String>{
                    false: '12-hour am/pm time',
                    true: '24-hour time',
                  },
                ),
                ChoiceCard<bool>(
                  choices: const <bool>[false, true],
                  value: widget.useMaterial3,
                  onChanged: widget.setUseMaterial3,
                  title: 'Material Version',
                  choiceLabels: const <bool, String>{false: 'Material 2', true: 'Material 3'},
                ),
              ],
            ),
          ),
          SingleChildScrollView(
            scrollDirection: Axis.horizontal,
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                Padding(
                  padding: const EdgeInsets.all(12.0),
                  child: ElevatedButton(
                    child: const Text('Open time picker'),
                    onPressed: () async {
                      final TimeOfDay? time = await showTimePicker(
                        context: context,
                        initialTime: selectedTime ?? TimeOfDay.now(),
                        initialEntryMode: entryMode,
                        orientation: orientation,
                        builder: (BuildContext context, Widget? child) {
                          // We just wrap these environmental changes around the
                          // child in this builder so that we can apply the
                          // options selected above. In regular usage, this is
                          // rarely necessary, because the default values are
                          // usually used as-is.
                          return Theme(
                            data: Theme.of(context).copyWith(materialTapTargetSize: tapTargetSize),
                            child: Directionality(
                              textDirection: textDirection,
                              child: MediaQuery(
                                data: MediaQuery.of(
                                  context,
                                ).copyWith(alwaysUse24HourFormat: use24HourTime),
                                child: child!,
                              ),
                            ),
                          );
                        },
                      );
                      setState(() {
                        selectedTime = time;
                      });
                    },
                  ),
                ),
                if (selectedTime != null) Text('Selected time: ${selectedTime!.format(context)}'),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

// This is a simple card that presents a set of radio buttons (inside of a
// RadioSelection, defined below) for the user to select from.
class ChoiceCard<T extends Object?> extends StatelessWidget {
  const ChoiceCard({
    super.key,
    required this.value,
    required this.choices,
    required this.onChanged,
    required this.choiceLabels,
    required this.title,
  });

  final T value;
  final Iterable<T> choices;
  final Map<T, String> choiceLabels;
  final String title;
  final ValueChanged<T?> onChanged;

  @override
  Widget build(BuildContext context) {
    return Card(
      // If the card gets too small, let it scroll both directions.
      child: SingleChildScrollView(
        child: SingleChildScrollView(
          scrollDirection: Axis.horizontal,
          child: Padding(
            padding: const EdgeInsets.all(8.0),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                Padding(padding: const EdgeInsets.all(8.0), child: Text(title)),
                for (final T choice in choices)
                  RadioSelection<T>(
                    value: choice,
                    groupValue: value,
                    onChanged: onChanged,
                    child: Text(choiceLabels[choice]!),
                  ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

// This aggregates a ChoiceCard so that it presents a set of radio buttons for
// the allowed enum values for the user to select from.
class EnumCard<T extends Enum> extends StatelessWidget {
  const EnumCard({super.key, required this.value, required this.choices, required this.onChanged});

  final T value;
  final Iterable<T> choices;
  final ValueChanged<T?> onChanged;

  @override
  Widget build(BuildContext context) {
    return ChoiceCard<T>(
      value: value,
      choices: choices,
      onChanged: onChanged,
      choiceLabels: <T, String>{for (final T choice in choices) choice: choice.name},
      title: value.runtimeType.toString(),
    );
  }
}

// A button that has a radio button on one side and a label child. Tapping on
// the label or the radio button selects the item.
class RadioSelection<T extends Object?> extends StatefulWidget {
  const RadioSelection({
    super.key,
    required this.value,
    required this.groupValue,
    required this.onChanged,
    required this.child,
  });

  final T value;
  final T? groupValue;
  final ValueChanged<T?> onChanged;
  final Widget child;

  @override
  State<RadioSelection<T>> createState() => _RadioSelectionState<T>();
}

class _RadioSelectionState<T extends Object?> extends State<RadioSelection<T>> {
  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        Padding(
          padding: const EdgeInsetsDirectional.only(end: 8),
          child: Radio<T>(
            groupValue: widget.groupValue,
            value: widget.value,
            onChanged: widget.onChanged,
          ),
        ),
        GestureDetector(onTap: () => widget.onChanged(widget.value), child: widget.child),
      ],
    );
  }
}
