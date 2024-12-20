// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'recorder.dart';

/// Measures how expensive it is to construct material checkboxes.
///
/// Creates a 10x10 grid of tristate checkboxes.
class BenchBuildMaterialCheckbox extends WidgetBuildRecorder {
  BenchBuildMaterialCheckbox() : super(name: benchmarkName);

  static const String benchmarkName = 'build_material_checkbox';

  static bool? _isChecked;

  @override
  Widget createWidget() {
    return Directionality(
      textDirection: TextDirection.ltr,
      child: Material(
        child: Column(
          children: List<Widget>.generate(10, (int i) {
            return _buildRow();
          }),
        ),
      ),
    );
  }

  Row _buildRow() {
    _isChecked = switch (_isChecked) {
      null => true,
      true => false,
      false => null,
    };

    return Row(
      children: List<Widget>.generate(10, (int i) {
        return Expanded(
          child: Checkbox(
            value: _isChecked,
            tristate: true,
            onChanged: (bool? newValue) {
              // Intentionally empty.
            },
          ),
        );
      }),
    );
  }
}
