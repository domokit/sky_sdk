// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [FilterChip].

enum ExerciseFilter { walking, running, cycling, hiking }

void main() => runApp(const ChipApp());

class ChipApp extends StatelessWidget {
  const ChipApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(useMaterial3: true),
      home: Scaffold(
        appBar: AppBar(title: const Text('FilterChip Sample')),
        body: const FilterChipExample(),
      ),
    );
  }
}

class FilterChipExample extends StatefulWidget {
  const FilterChipExample({super.key});

  @override
  State<FilterChipExample> createState() => _FilterChipExampleState();
}

class _FilterChipExampleState extends State<FilterChipExample> {
  Set<ExerciseFilter> filters = <ExerciseFilter>{};

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;

    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          Text('Choose an exercise', style: textTheme.labelLarge),
          const SizedBox(height: 5.0),
          Wrap(
            spacing: 5.0,
            children:
                ExerciseFilter.values.map((ExerciseFilter exercise) {
                  return FilterChip(
                    label: Text(exercise.name),
                    selected: filters.contains(exercise),
                    onSelected: (bool selected) {
                      setState(() {
                        if (selected) {
                          filters.add(exercise);
                        } else {
                          filters.remove(exercise);
                        }
                      });
                    },
                  );
                }).toList(),
          ),
          const SizedBox(height: 10.0),
          Text(
            'Looking for: ${filters.map((ExerciseFilter e) => e.name).join(', ')}',
            style: textTheme.labelLarge,
          ),
        ],
      ),
    );
  }
}
