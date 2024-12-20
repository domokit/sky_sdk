// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';

/// Flutter code sample for [ThemeExtension].

@immutable
class MyColors extends ThemeExtension<MyColors> {
  const MyColors({required this.brandColor, required this.danger});

  final Color? brandColor;
  final Color? danger;

  @override
  MyColors copyWith({Color? brandColor, Color? danger}) {
    return MyColors(brandColor: brandColor ?? this.brandColor, danger: danger ?? this.danger);
  }

  @override
  MyColors lerp(MyColors? other, double t) {
    if (other is! MyColors) {
      return this;
    }
    return MyColors(
      brandColor: Color.lerp(brandColor, other.brandColor, t),
      danger: Color.lerp(danger, other.danger, t),
    );
  }

  // Optional
  @override
  String toString() => 'MyColors(brandColor: $brandColor, danger: $danger)';
}

void main() {
  // Slow down time to see lerping.
  timeDilation = 5.0;
  runApp(const ThemeExtensionExampleApp());
}

class ThemeExtensionExampleApp extends StatefulWidget {
  const ThemeExtensionExampleApp({super.key});

  @override
  State<ThemeExtensionExampleApp> createState() => _ThemeExtensionExampleAppState();
}

class _ThemeExtensionExampleAppState extends State<ThemeExtensionExampleApp> {
  bool isLightTheme = true;

  void toggleTheme() {
    setState(() => isLightTheme = !isLightTheme);
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData.light().copyWith(
        extensions: <ThemeExtension<dynamic>>[
          const MyColors(brandColor: Color(0xFF1E88E5), danger: Color(0xFFE53935)),
        ],
      ),
      darkTheme: ThemeData.dark().copyWith(
        extensions: <ThemeExtension<dynamic>>[
          const MyColors(brandColor: Color(0xFF90CAF9), danger: Color(0xFFEF9A9A)),
        ],
      ),
      themeMode: isLightTheme ? ThemeMode.light : ThemeMode.dark,
      home: Home(isLightTheme: isLightTheme, toggleTheme: toggleTheme),
    );
  }
}

class Home extends StatelessWidget {
  const Home({super.key, required this.isLightTheme, required this.toggleTheme});

  final bool isLightTheme;
  final void Function() toggleTheme;

  @override
  Widget build(BuildContext context) {
    final MyColors myColors = Theme.of(context).extension<MyColors>()!;
    return Material(
      child: Center(
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Container(width: 100, height: 100, color: myColors.brandColor),
            const SizedBox(width: 10),
            Container(width: 100, height: 100, color: myColors.danger),
            const SizedBox(width: 50),
            IconButton(
              icon: Icon(isLightTheme ? Icons.nightlight : Icons.wb_sunny),
              onPressed: toggleTheme,
            ),
          ],
        ),
      ),
    );
  }
}
