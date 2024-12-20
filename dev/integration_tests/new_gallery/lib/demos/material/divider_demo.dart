// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';
import 'material_demo_types.dart';

class DividerDemo extends StatelessWidget {
  const DividerDemo({super.key, required this.type});

  final DividerDemoType type;

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Text(switch (type) {
          DividerDemoType.horizontal => localizations.demoDividerTitle,
          DividerDemoType.vertical => localizations.demoVerticalDividerTitle,
        }),
      ),
      body: switch (type) {
        DividerDemoType.horizontal => _HorizontalDividerDemo(),
        DividerDemoType.vertical => _VerticalDividerDemo(),
      },
    );
  }
}

// BEGIN dividerDemo

class _HorizontalDividerDemo extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(10),
      child: Column(
        children: <Widget>[
          Expanded(
            child: Container(
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(10),
                color: Colors.deepPurpleAccent,
              ),
            ),
          ),
          const Divider(color: Colors.grey, height: 20, thickness: 1, indent: 20, endIndent: 0),
          Expanded(
            child: Container(
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(10),
                color: Colors.deepOrangeAccent,
              ),
            ),
          ),
        ],
      ),
    );
  }
}

// END

// BEGIN verticalDividerDemo

class _VerticalDividerDemo extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(10),
      child: Row(
        children: <Widget>[
          Expanded(
            child: Container(
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(10),
                color: Colors.deepPurpleAccent,
              ),
            ),
          ),
          const VerticalDivider(
            color: Colors.grey,
            thickness: 1,
            indent: 20,
            endIndent: 0,
            width: 20,
          ),
          Expanded(
            child: Container(
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(10),
                color: Colors.deepOrangeAccent,
              ),
            ),
          ),
        ],
      ),
    );
  }
}

// END
