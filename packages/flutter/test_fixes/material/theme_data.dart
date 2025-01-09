// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  // Changes made in https://github.com/flutter/flutter/pull/66482
  ThemeData(textSelectionColor: Colors.red);
  ThemeData(cursorColor: Colors.blue);
  ThemeData(textSelectionHandleColor: Colors.yellow);
  ThemeData(useTextSelectionTheme: false);
  ThemeData(textSelectionColor: Colors.red, useTextSelectionTheme: false);
  ThemeData(cursorColor: Colors.blue, useTextSelectionTheme: false);
  ThemeData(
    textSelectionHandleColor: Colors.yellow,
    useTextSelectionTheme: false,
  );
  ThemeData(textSelectionColor: Colors.red, cursorColor: Colors.blue);
  ThemeData(textSelectionHandleColor: Colors.yellow, cursorColor: Colors.blue);
  ThemeData(
    textSelectionColor: Colors.red,
    textSelectionHandleColor: Colors.yellow,
  );
  ThemeData(
    textSelectionColor: Colors.red,
    cursorColor: Colors.blue,
    useTextSelectionTheme: false,
  );
  ThemeData(
    textSelectionHandleColor: Colors.yellow,
    cursorColor: Colors.blue,
    useTextSelectionTheme: true,
  );
  ThemeData(
    textSelectionColor: Colors.red,
    textSelectionHandleColor: Colors.yellow,
    useTextSelectionTheme: false,
  );
  ThemeData(
    textSelectionColor: Colors.red,
    cursorColor: Colors.blue,
    textSelectionHandleColor: Colors.yellow,
  );
  ThemeData(
    textSelectionColor: Colors.red,
    cursorColor: Colors.blue,
    textSelectionHandleColor: Colors.yellow,
    useTextSelectionTheme: false,
  );
  ThemeData(error: '');
  ThemeData.raw(error: '');
  ThemeData.raw(textSelectionColor: Colors.red);
  ThemeData.raw(cursorColor: Colors.blue);
  ThemeData.raw(textSelectionHandleColor: Colors.yellow);
  ThemeData.raw(useTextSelectionTheme: false);
  ThemeData.raw(textSelectionColor: Colors.red, useTextSelectionTheme: false);
  ThemeData.raw(cursorColor: Colors.blue, useTextSelectionTheme: false);
  ThemeData.raw(
    textSelectionHandleColor: Colors.yellow,
    useTextSelectionTheme: false,
  );
  ThemeData.raw(textSelectionColor: Colors.red, cursorColor: Colors.blue);
  ThemeData.raw(
    textSelectionHandleColor: Colors.yellow,
    cursorColor: Colors.blue,
  );
  ThemeData.raw(
    textSelectionColor: Colors.red,
    textSelectionHandleColor: Colors.yellow,
  );
  ThemeData.raw(
    textSelectionColor: Colors.red,
    cursorColor: Colors.blue,
    useTextSelectionTheme: false,
  );
  ThemeData.raw(
    textSelectionHandleColor: Colors.yellow,
    cursorColor: Colors.blue,
    useTextSelectionTheme: true,
  );
  ThemeData.raw(
    textSelectionColor: Colors.red,
    textSelectionHandleColor: Colors.yellow,
    useTextSelectionTheme: false,
  );
  ThemeData.raw(
    textSelectionColor: Colors.red,
    cursorColor: Colors.blue,
    textSelectionHandleColor: Colors.yellow,
  );
  ThemeData.raw(
    textSelectionColor: Colors.red,
    cursorColor: Colors.blue,
    textSelectionHandleColor: Colors.yellow,
    useTextSelectionTheme: false,
  );

  // Changes made in https://github.com/flutter/flutter/pull/81336
  ThemeData themeData = ThemeData();
  themeData = ThemeData(accentColor: Colors.red);
  themeData = ThemeData(accentColor: Colors.red, primarySwatch: Colors.blue);
  themeData = ThemeData(
    accentColor: Colors.red,
    colorScheme: ColorScheme.light(),
  );
  themeData = ThemeData(
    accentColor: Colors.red,
    colorScheme: ColorScheme.light(),
    primarySwatch: Colors.blue,
  );
  themeData = ThemeData(error: '');
  themeData = ThemeData.raw(accentColor: Colors.red);
  themeData = ThemeData.raw(
    accentColor: Colors.red,
    primarySwatch: Colors.blue,
  );
  themeData = ThemeData.raw(
    accentColor: Colors.red,
    colorScheme: ColorScheme.light(),
  );
  themeData = ThemeData.raw(
    accentColor: Colors.red,
    colorScheme: ColorScheme.light(),
    primarySwatch: Colors.blue,
  );
  themeData = ThemeData.raw(error: '');
  themeData = themeData.copyWith(accentColor: Colors.red);
  themeData = themeData.copyWith(error: '');
  themeData = themeData.copyWith(
    accentColor: Colors.red,
    primarySwatch: Colors.blue,
  );
  themeData = themeData.copyWith(
    accentColor: Colors.red,
    colorScheme: ColorScheme.light(),
  );
  themeData = themeData.copyWith(
    accentColor: Colors.red,
    colorScheme: ColorScheme.light(),
    primarySwatch: Colors.blue,
  );
  themeData.accentColor;

  // Changes made in https://github.com/flutter/flutter/pull/81336
  ThemeData themeData = ThemeData();
  themeData = ThemeData(accentColorBrightness: Brightness.dark);
  themeData = ThemeData.raw(accentColorBrightness: Brightness.dark);
  themeData = themeData.copyWith(accentColorBrightness: Brightness.dark);
  themeData.accentColorBrightness; // Removing field reference not supported.

  // Changes made in https://github.com/flutter/flutter/pull/81336
  ThemeData themeData = ThemeData();
  themeData = ThemeData(accentTextTheme: TextTheme());
  themeData = ThemeData.raw(accentTextTheme: TextTheme());
  themeData = themeData.copyWith(accentTextTheme: TextTheme());
  themeData.accentTextTheme; // Removing field reference not supported.

  // Changes made in https://github.com/flutter/flutter/pull/81336
  ThemeData themeData = ThemeData();
  themeData = ThemeData(accentIconTheme: IconThemeData());
  themeData = ThemeData.raw(accentIconTheme: IconThemeData());
  themeData = themeData.copyWith(accentIconTheme: IconThemeData());
  themeData.accentIconTheme; // Removing field reference not supported.

  // Changes made in https://github.com/flutter/flutter/pull/81336
  ThemeData themeData = ThemeData();
  themeData = ThemeData(buttonColor: Colors.red);
  themeData = ThemeData.raw(buttonColor: Colors.red);
  themeData = themeData.copyWith(buttonColor: Colors.red);
  themeData.buttonColor; // Removing field reference not supported.

  // Changes made in https://github.com/flutter/flutter/pull/87281
  ThemeData themeData = ThemeData();
  themeData = ThemeData(fixTextFieldOutlineLabel: true);
  themeData = ThemeData.raw(fixTextFieldOutlineLabel: true);
  themeData = themeData.copyWith(fixTextFieldOutlineLabel: true);
  themeData.fixTextFieldOutlineLabel; // Removing field reference not supported.

  // Changes made in https://github.com/flutter/flutter/pull/93396
  ThemeData themeData = ThemeData();
  themeData = ThemeData(primaryColorBrightness: Brightness.dark);
  themeData = ThemeData.raw(primaryColorBrightness: Brightness.dark);
  themeData = themeData.copyWith(primaryColorBrightness: Brightness.dark);
  themeData.primaryColorBrightness; // Removing field reference not supported.

  // Changes made in https://github.com/flutter/flutter/pull/97972
  ThemeData themeData = ThemeData();
  themeData = ThemeData(toggleableActiveColor: Colors.black);
  themeData = ThemeData(toggleableActiveColor: Colors.black);
  themeData = ThemeData.raw(toggleableActiveColor: Colors.black);
  themeData = ThemeData.raw(toggleableActiveColor: Colors.black);
  themeData = themeData.copyWith(toggleableActiveColor: Colors.black);
  themeData = themeData.copyWith(toggleableActiveColor: Colors.black);
  themeData.toggleableActiveColor; // Removing field reference not supported.

  // Changes made in https://github.com/flutter/flutter/pull/109070
  ThemeData themeData = ThemeData();
  themeData = ThemeData(selectedRowColor: Brightness.dark);
  themeData = ThemeData.raw(selectedRowColor: Brightness.dark);
  themeData = themeData.copyWith(selectedRowColor: Brightness.dark);
  themeData.selectedRowColor; // Removing field reference not supported.

  // Changes made in https://github.com/flutter/flutter/pull/110162
  ThemeData themeData = ThemeData();
  themeData = ThemeData(errorColor: Colors.red);
  themeData = ThemeData(errorColor: Colors.red, primarySwatch: Colors.blue);
  themeData = ThemeData(
    errorColor: Colors.red,
    colorScheme: ColorScheme.light(),
  );
  themeData = ThemeData(
    errorColor: Colors.red,
    colorScheme: ColorScheme.light(),
    primarySwatch: Colors.blue,
  );
  themeData = ThemeData(otherParam: '');
  themeData = ThemeData.raw(errorColor: Colors.red);
  themeData = ThemeData.raw(errorColor: Colors.red, primarySwatch: Colors.blue);
  themeData = ThemeData.raw(
    errorColor: Colors.red,
    colorScheme: ColorScheme.light(),
  );
  themeData = ThemeData.raw(
    errorColor: Colors.red,
    colorScheme: ColorScheme.light(),
    primarySwatch: Colors.blue,
  );
  themeData = ThemeData.raw(otherParam: '');
  themeData = themeData.copyWith(errorColor: Colors.red);
  themeData = themeData.copyWith(otherParam: '');
  themeData = themeData.copyWith(
    errorColor: Colors.red,
    primarySwatch: Colors.blue,
  );
  themeData = themeData.copyWith(
    errorColor: Colors.red,
    colorScheme: ColorScheme.light(),
  );
  themeData = themeData.copyWith(
    errorColor: Colors.red,
    colorScheme: ColorScheme.light(),
    primarySwatch: Colors.blue,
  );
  themeData.errorColor;

  // Changes made in https://github.com/flutter/flutter/pull/110162
  ThemeData themeData = ThemeData();
  themeData = ThemeData(backgroundColor: Colors.grey);
  themeData = ThemeData(
    backgroundColor: Colors.grey,
    primarySwatch: Colors.blue,
  );
  themeData = ThemeData(
    backgroundColor: Colors.grey,
    colorScheme: ColorScheme.light(),
  );
  themeData = ThemeData(
    backgroundColor: Colors.grey,
    colorScheme: ColorScheme.light(),
    primarySwatch: Colors.blue,
  );
  themeData = ThemeData(otherParam: '');
  themeData = ThemeData.raw(backgroundColor: Colors.grey);
  themeData = ThemeData.raw(
    backgroundColor: Colors.grey,
    primarySwatch: Colors.blue,
  );
  themeData = ThemeData.raw(
    backgroundColor: Colors.grey,
    colorScheme: ColorScheme.light(),
  );
  themeData = ThemeData.raw(
    backgroundColor: Colors.grey,
    colorScheme: ColorScheme.light(),
    primarySwatch: Colors.blue,
  );
  themeData = ThemeData.raw(otherParam: '');
  themeData = themeData.copyWith(backgroundColor: Colors.grey);
  themeData = themeData.copyWith(otherParam: '');
  themeData = themeData.copyWith(
    backgroundColor: Colors.grey,
    primarySwatch: Colors.blue,
  );
  themeData = themeData.copyWith(
    backgroundColor: Colors.grey,
    colorScheme: ColorScheme.light(),
  );
  themeData = themeData.copyWith(
    backgroundColor: Colors.grey,
    colorScheme: ColorScheme.light(),
    primarySwatch: Colors.blue,
  );
  themeData.backgroundColor;

  // Changes made in https://github.com/flutter/flutter/pull/110162
  ThemeData themeData = ThemeData();
  themeData = ThemeData(backgroundColor: Colors.grey, errorColor: Colors.red);
  themeData = ThemeData.raw(
    backgroundColor: Colors.grey,
    errorColor: Colors.red,
  );
  themeData = themeData.copyWith(
    backgroundColor: Colors.grey,
    errorColor: Colors.red,
  );

  // Changes made in https://github.com/flutter/flutter/pull/111080
  ThemeData themeData = ThemeData();
  themeData = ThemeData(bottomAppBarColor: Colors.green);
  themeData = ThemeData.raw(bottomAppBarColor: Colors.green);
  themeData = ThemeData.copyWith(bottomAppBarColor: Colors.green);

  // Changes made in https://github.com/flutter/flutter/pull/131455
  ThemeData themeData = ThemeData.copyWith(useMaterial3: false);

  // Changes made in https://github.com/flutter/flutter/pull/155072
  ThemeData themeData = ThemeData();
  themeData = ThemeData(dialogBackgroundColor: Colors.orange);
  themeData = ThemeData(
    dialogBackgroundColor: Colors.orange,
    dialogTheme: DialogThemeData(backgroundColor: Colors.red),
  );
  themeData = themeData.copyWith(dialogBackgroundColor: Colors.orange);
  themeData = themeData.copyWith(
    dialogBackgroundColor: Colors.orange,
    dialogTheme: DialogThemeData(backgroundColor: Colors.red),
  );
  themeData.dialogBackgroundColor; // Removing field reference not supported.
}
