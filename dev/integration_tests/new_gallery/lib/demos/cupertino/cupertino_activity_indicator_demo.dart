// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import '../../gallery_localizations.dart';

// BEGIN cupertinoActivityIndicatorDemo

class CupertinoProgressIndicatorDemo extends StatelessWidget {
  const CupertinoProgressIndicatorDemo({super.key});

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        automaticallyImplyLeading: false,
        middle: Text(GalleryLocalizations.of(context)!.demoCupertinoActivityIndicatorTitle),
      ),
      child: const Center(child: CupertinoActivityIndicator()),
    );
  }
}

// END
