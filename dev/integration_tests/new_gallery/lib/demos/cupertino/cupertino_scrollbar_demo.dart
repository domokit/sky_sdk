// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import '../../gallery_localizations.dart';

// BEGIN cupertinoScrollbarDemo

class CupertinoScrollbarDemo extends StatelessWidget {
  const CupertinoScrollbarDemo({super.key});

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        automaticallyImplyLeading: false,
        middle: Text(localizations.demoCupertinoScrollbarTitle),
      ),
      child: CupertinoScrollbar(
        thickness: 6.0,
        thicknessWhileDragging: 10.0,
        radius: const Radius.circular(34.0),
        radiusWhileDragging: Radius.zero,
        child: ListView.builder(
          itemCount: 120,
          itemBuilder: (BuildContext context, int index) {
            return Center(
              child: Text('item $index', style: CupertinoTheme.of(context).textTheme.textStyle),
            );
          },
        ),
      ),
    );
  }
}

// END
