// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for the [DropdownMenuEntry] `labelWidget` property.

enum ColorItem {
  blue('Blue', Colors.blue),
  pink('Pink', Colors.pink),
  green('Green', Colors.green),
  yellow('Yellow', Colors.yellow),
  grey('Grey', Colors.grey);

  const ColorItem(this.label, this.color);
  final String label;
  final Color color;
}

class DropdownMenuEntryLabelWidgetExample extends StatefulWidget {
  const DropdownMenuEntryLabelWidgetExample({super.key});

  @override
  State<DropdownMenuEntryLabelWidgetExample> createState() =>
      _DropdownMenuEntryLabelWidgetExampleState();
}

class _DropdownMenuEntryLabelWidgetExampleState extends State<DropdownMenuEntryLabelWidgetExample> {
  late final TextEditingController controller;

  @override
  void initState() {
    super.initState();
    controller = TextEditingController();
  }

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    // Created by Google Bard from 'create a lyrical phrase of about 25 words that begins with "is a color"'.
    const String longText =
        'is a color that sings of hope, A hue that shines like gold. It is the color of dreams, A shade that never grows old.';

    return Scaffold(
      body: Center(
        child: DropdownMenu<ColorItem>(
          width: 300,
          controller: controller,
          initialSelection: ColorItem.green,
          label: const Text('Color'),
          onSelected: (ColorItem? color) {
            print('Selected $color');
          },
          dropdownMenuEntries:
              ColorItem.values.map<DropdownMenuEntry<ColorItem>>((ColorItem item) {
                final String labelText = '${item.label} $longText\n';
                return DropdownMenuEntry<ColorItem>(
                  value: item,
                  label: labelText,
                  // Try commenting the labelWidget out or changing
                  // the labelWidget's Text parameters.
                  labelWidget: Text(labelText, maxLines: 1, overflow: TextOverflow.ellipsis),
                );
              }).toList(),
        ),
      ),
    );
  }
}

class DropdownMenuEntryLabelWidgetExampleApp extends StatelessWidget {
  const DropdownMenuEntryLabelWidgetExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: DropdownMenuEntryLabelWidgetExample());
  }
}

void main() {
  runApp(const DropdownMenuEntryLabelWidgetExampleApp());
}
