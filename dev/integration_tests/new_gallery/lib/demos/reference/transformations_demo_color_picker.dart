// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

// A generic widget for a list of selectable colors.
@immutable
class ColorPicker extends StatelessWidget {
  const ColorPicker({
    super.key,
    required this.colors,
    required this.selectedColor,
    this.onColorSelection,
  });

  final Set<Color> colors;
  final Color selectedColor;
  final ValueChanged<Color>? onColorSelection;

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.center,
      children:
          colors.map((Color color) {
            return _ColorPickerSwatch(
              color: color,
              selected: color == selectedColor,
              onTap: () {
                if (onColorSelection != null) {
                  onColorSelection!(color);
                }
              },
            );
          }).toList(),
    );
  }
}

// A single selectable color widget in the ColorPicker.
@immutable
class _ColorPickerSwatch extends StatelessWidget {
  const _ColorPickerSwatch({required this.color, required this.selected, this.onTap});

  final Color color;
  final bool selected;
  final void Function()? onTap;

  @override
  Widget build(BuildContext context) {
    return Container(
      width: 60,
      height: 60,
      padding: const EdgeInsets.fromLTRB(2, 0, 2, 0),
      child: RawMaterialButton(
        fillColor: color,
        onPressed: () {
          if (onTap != null) {
            onTap!();
          }
        },
        child: !selected ? null : const Icon(Icons.check, color: Colors.white),
      ),
    );
  }
}
