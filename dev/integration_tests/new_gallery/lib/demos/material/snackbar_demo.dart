// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

// BEGIN snackbarsDemo

class SnackbarsDemo extends StatelessWidget {
  const SnackbarsDemo({super.key});

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Text(localizations.demoSnackbarsTitle),
      ),
      body: Center(
        child: ElevatedButton(
          onPressed: () {
            ScaffoldMessenger.of(context).hideCurrentSnackBar();
            ScaffoldMessenger.of(context).showSnackBar(
              SnackBar(
                content: Text(localizations.demoSnackbarsText),
                action: SnackBarAction(
                  label: localizations.demoSnackbarsActionButtonLabel,
                  onPressed: () {
                    ScaffoldMessenger.of(context).hideCurrentSnackBar();
                    ScaffoldMessenger.of(
                      context,
                    ).showSnackBar(SnackBar(content: Text(localizations.demoSnackbarsAction)));
                  },
                ),
              ),
            );
          },
          child: Text(localizations.demoSnackbarsButtonLabel),
        ),
      ),
    );
  }
}

// END
