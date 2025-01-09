// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class TextStyleItem extends StatelessWidget {
  const TextStyleItem({super.key, required this.name, required this.style, required this.text});

  final String name;
  final TextStyle style;
  final String text;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final TextStyle nameStyle = theme.textTheme.bodySmall!.copyWith(
      color: theme.textTheme.bodySmall!.color,
    );
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 8.0, vertical: 16.0),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          SizedBox(width: 72.0, child: Text(name, style: nameStyle)),
          Expanded(child: Text(text, style: style.copyWith(height: 1.0))),
        ],
      ),
    );
  }
}

class TypographyDemo extends StatelessWidget {
  const TypographyDemo({super.key});

  static const String routeName = '/typography';

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    final List<Widget> styleItems = <Widget>[
      TextStyleItem(
        name: 'Display Large',
        style: textTheme.displayLarge!,
        text: 'Regular 57/64 +0',
      ),
      TextStyleItem(
        name: 'Display Medium',
        style: textTheme.displayMedium!,
        text: 'Regular 45/52 +0',
      ),
      TextStyleItem(
        name: 'Display Small',
        style: textTheme.displaySmall!,
        text: 'Regular 36/44 +0',
      ),
      TextStyleItem(
        name: 'Headline Large',
        style: textTheme.headlineLarge!,
        text: 'Regular 32/40 +0',
      ),
      TextStyleItem(
        name: 'Headline Medium',
        style: textTheme.headlineMedium!,
        text: 'Regular 28/36 +0',
      ),
      TextStyleItem(
        name: 'Headline Small',
        style: textTheme.headlineSmall!,
        text: 'Regular 24/32 +0',
      ),
      TextStyleItem(name: 'Title Large', style: textTheme.titleLarge!, text: 'Medium 22/28 +0'),
      TextStyleItem(
        name: 'Title Medium',
        style: textTheme.titleMedium!,
        text: 'Medium 16/24 +0.15',
      ),
      TextStyleItem(name: 'Title Small', style: textTheme.titleSmall!, text: 'Medium 14/20 +0.1'),
      TextStyleItem(name: 'Body Large', style: textTheme.bodyLarge!, text: 'Regular 16/24 +0.5'),
      TextStyleItem(name: 'Body Medium', style: textTheme.bodyMedium!, text: 'Regular 14/20 +0.25'),
      TextStyleItem(name: 'Body Small', style: textTheme.bodySmall!, text: 'Regular 12/16 +0.4'),
      TextStyleItem(name: 'Label Large', style: textTheme.labelLarge!, text: 'Medium 14/20 +0.1'),
      TextStyleItem(name: 'Label Medium', style: textTheme.labelMedium!, text: 'Medium 12/16 +0.5'),
      TextStyleItem(name: 'Label Small', style: textTheme.labelSmall!, text: 'Medium 11/16 +0.5'),
    ];

    return Scaffold(
      appBar: AppBar(title: const Text('Typography')),
      body: SafeArea(
        top: false,
        bottom: false,
        child: Scrollbar(child: ListView(primary: true, children: styleItems)),
      ),
    );
  }
}
