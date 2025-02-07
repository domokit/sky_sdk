// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  // Changes made in https://github.com/flutter/flutter/pull/46115.
  const InputDecoration inputDecoration = InputDecoration(
    hasFloatingPlaceholder: true,
  );
  InputDecoration(hasFloatingPlaceholder: false);
  InputDecoration();
  InputDecoration(error: '');
  InputDecoration.collapsed(hasFloatingPlaceholder: true);
  InputDecoration.collapsed(hasFloatingPlaceholder: false);
  InputDecoration.collapsed();
  InputDecoration.collapsed(error: '');
  inputDecoration.hasFloatingPlaceholder;
  const InputDecorationTheme inputDecorationTheme = InputDecorationTheme(
    hasFloatingPlaceholder: true,
  );
  InputDecorationTheme(hasFloatingPlaceholder: false);
  InputDecorationTheme();
  InputDecorationTheme(error: '');
  inputDecorationTheme.hasFloatingPlaceholder;
  inputDecorationTheme.copyWith(hasFloatingPlaceholder: false);
  inputDecorationTheme.copyWith(hasFloatingPlaceholder: true);
  inputDecorationTheme.copyWith();
  inputDecorationTheme.copyWith(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/152486.
  const InputDecoration decoration = InputDecoration.collapsed(
    hintText: 'Hint',
    floatingLabelAlignment: FloatingLabelAlignment.center,
    floatingLabelBehavior: FloatingLabelBehavior.always,
  );

  // Changes made in https://github.com/flutter/flutter/pull/161235.
  const InputDecoration decoration = InputDecoration(maintainHintHeight: false);
  decoration.maintainHintHeight;

  const InputDecoration decoration = InputDecoration.collapsed(
    maintainHintHeight: false,
  );
}
