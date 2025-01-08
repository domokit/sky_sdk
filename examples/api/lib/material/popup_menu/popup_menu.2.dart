// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [PopupMenuButton].

void main() => runApp(const PopupMenuApp());

class PopupMenuApp extends StatelessWidget {
  const PopupMenuApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: PopupMenuExample());
  }
}

enum AnimationStyles { defaultStyle, custom, none }

const List<(AnimationStyles, String)> animationStyleSegments = <(AnimationStyles, String)>[
  (AnimationStyles.defaultStyle, 'Default'),
  (AnimationStyles.custom, 'Custom'),
  (AnimationStyles.none, 'None'),
];

enum Menu { preview, share, getLink, remove, download }

class PopupMenuExample extends StatefulWidget {
  const PopupMenuExample({super.key});

  @override
  State<PopupMenuExample> createState() => _PopupMenuExampleState();
}

class _PopupMenuExampleState extends State<PopupMenuExample> {
  Set<AnimationStyles> _animationStyleSelection = <AnimationStyles>{AnimationStyles.defaultStyle};
  AnimationStyle? _animationStyle;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.only(top: 50),
          child: Align(
            alignment: Alignment.topCenter,
            child: Column(
              mainAxisSize: MainAxisSize.min,
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                SegmentedButton<AnimationStyles>(
                  selected: _animationStyleSelection,
                  onSelectionChanged: (Set<AnimationStyles> styles) {
                    setState(() {
                      _animationStyleSelection = styles;
                      switch (styles.first) {
                        case AnimationStyles.defaultStyle:
                          _animationStyle = null;
                        case AnimationStyles.custom:
                          _animationStyle = const AnimationStyle(
                            curve: Easing.emphasizedDecelerate,
                            duration: Duration(seconds: 3),
                          );
                        case AnimationStyles.none:
                          _animationStyle = AnimationStyle.noAnimation;
                      }
                    });
                  },
                  segments:
                      animationStyleSegments.map<ButtonSegment<AnimationStyles>>((
                        (AnimationStyles, String) shirt,
                      ) {
                        return ButtonSegment<AnimationStyles>(
                          value: shirt.$1,
                          label: Text(shirt.$2),
                        );
                      }).toList(),
                ),
                const SizedBox(height: 10),
                PopupMenuButton<Menu>(
                  popUpAnimationStyle: _animationStyle,
                  icon: const Icon(Icons.more_vert),
                  onSelected: (Menu item) {},
                  itemBuilder:
                      (BuildContext context) => <PopupMenuEntry<Menu>>[
                        const PopupMenuItem<Menu>(
                          value: Menu.preview,
                          child: ListTile(
                            leading: Icon(Icons.visibility_outlined),
                            title: Text('Preview'),
                          ),
                        ),
                        const PopupMenuItem<Menu>(
                          value: Menu.share,
                          child: ListTile(
                            leading: Icon(Icons.share_outlined),
                            title: Text('Share'),
                          ),
                        ),
                        const PopupMenuItem<Menu>(
                          value: Menu.getLink,
                          child: ListTile(
                            leading: Icon(Icons.link_outlined),
                            title: Text('Get link'),
                          ),
                        ),
                        const PopupMenuDivider(),
                        const PopupMenuItem<Menu>(
                          value: Menu.remove,
                          child: ListTile(
                            leading: Icon(Icons.delete_outline),
                            title: Text('Remove'),
                          ),
                        ),
                        const PopupMenuItem<Menu>(
                          value: Menu.download,
                          child: ListTile(
                            leading: Icon(Icons.download_outlined),
                            title: Text('Download'),
                          ),
                        ),
                      ],
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
