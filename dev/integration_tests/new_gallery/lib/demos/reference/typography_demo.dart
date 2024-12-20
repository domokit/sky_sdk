// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery_localizations.dart';

// BEGIN typographyDemo

class _TextStyleItem extends StatelessWidget {
  const _TextStyleItem({required this.name, required this.style, required this.text});

  final String name;
  final TextStyle style;
  final String text;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 16),
      child: Row(
        children: <Widget>[
          SizedBox(width: 72, child: Text(name, style: Theme.of(context).textTheme.bodySmall)),
          Expanded(child: Text(text, style: style)),
        ],
      ),
    );
  }
}

class TypographyDemo extends StatelessWidget {
  const TypographyDemo({super.key});

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    final List<_TextStyleItem> styleItems = <_TextStyleItem>[
      _TextStyleItem(name: 'Headline 1', style: textTheme.displayLarge!, text: 'Light 96sp'),
      _TextStyleItem(name: 'Headline 2', style: textTheme.displayMedium!, text: 'Light 60sp'),
      _TextStyleItem(name: 'Headline 3', style: textTheme.displaySmall!, text: 'Regular 48sp'),
      _TextStyleItem(name: 'Headline 4', style: textTheme.headlineMedium!, text: 'Regular 34sp'),
      _TextStyleItem(name: 'Headline 5', style: textTheme.headlineSmall!, text: 'Regular 24sp'),
      _TextStyleItem(name: 'Headline 6', style: textTheme.titleLarge!, text: 'Medium 20sp'),
      _TextStyleItem(name: 'Subtitle 1', style: textTheme.titleMedium!, text: 'Regular 16sp'),
      _TextStyleItem(name: 'Subtitle 2', style: textTheme.titleSmall!, text: 'Medium 14sp'),
      _TextStyleItem(name: 'Body Text 1', style: textTheme.bodyLarge!, text: 'Regular 16sp'),
      _TextStyleItem(name: 'Body Text 2', style: textTheme.bodyMedium!, text: 'Regular 14sp'),
      _TextStyleItem(name: 'Button', style: textTheme.labelLarge!, text: 'MEDIUM (ALL CAPS) 14sp'),
      _TextStyleItem(name: 'Caption', style: textTheme.bodySmall!, text: 'Regular 12sp'),
      _TextStyleItem(
        name: 'Overline',
        style: textTheme.labelSmall!,
        text: 'REGULAR (ALL CAPS) 10sp',
      ),
    ];

    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Text(GalleryLocalizations.of(context)!.demoTypographyTitle),
      ),
      body: Scrollbar(child: ListView(children: styleItems)),
    );
  }
}

// END
