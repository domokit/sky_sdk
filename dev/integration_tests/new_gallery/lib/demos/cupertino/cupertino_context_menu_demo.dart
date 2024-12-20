// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

// BEGIN cupertinoContextMenuDemo

class CupertinoContextMenuDemo extends StatelessWidget {
  const CupertinoContextMenuDemo({super.key});

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations galleryLocalizations = GalleryLocalizations.of(context)!;
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        automaticallyImplyLeading: false,
        middle: Text(galleryLocalizations.demoCupertinoContextMenuTitle),
      ),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: CupertinoContextMenu(
                actions: <Widget>[
                  CupertinoContextMenuAction(
                    onPressed: () {
                      Navigator.pop(context);
                    },
                    child: Text(galleryLocalizations.demoCupertinoContextMenuActionOne),
                  ),
                  CupertinoContextMenuAction(
                    onPressed: () {
                      Navigator.pop(context);
                    },
                    child: Text(galleryLocalizations.demoCupertinoContextMenuActionTwo),
                  ),
                ],
                child: const FlutterLogo(size: 250),
              ),
            ),
          ),
          const SizedBox(height: 20),
          Padding(
            padding: const EdgeInsets.all(30),
            child: Text(
              galleryLocalizations.demoCupertinoContextMenuActionText,
              textAlign: TextAlign.center,
              style: const TextStyle(color: Colors.black),
            ),
          ),
        ],
      ),
    );
  }
}

// END
