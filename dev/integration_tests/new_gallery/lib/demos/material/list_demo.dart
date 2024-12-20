// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery_localizations.dart';
import 'material_demo_types.dart';

// BEGIN listDemo

class ListDemo extends StatelessWidget {
  const ListDemo({super.key, required this.type});

  final ListDemoType type;

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Scaffold(
      appBar: AppBar(automaticallyImplyLeading: false, title: Text(localizations.demoListsTitle)),
      body: Scrollbar(
        child: ListView(
          restorationId: 'list_demo_list_view',
          padding: const EdgeInsets.symmetric(vertical: 8),
          children: <Widget>[
            for (int index = 1; index < 21; index++)
              ListTile(
                leading: ExcludeSemantics(child: CircleAvatar(child: Text('$index'))),
                title: Text(localizations.demoBottomSheetItem(index)),
                subtitle:
                    type == ListDemoType.twoLine ? Text(localizations.demoListsSecondary) : null,
              ),
          ],
        ),
      ),
    );
  }
}

// END
