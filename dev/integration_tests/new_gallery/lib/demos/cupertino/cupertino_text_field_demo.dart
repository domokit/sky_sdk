// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import '../../gallery_localizations.dart';

// BEGIN cupertinoTextFieldDemo

class CupertinoTextFieldDemo extends StatelessWidget {
  const CupertinoTextFieldDemo({super.key});

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        automaticallyImplyLeading: false,
        middle: Text(localizations.demoCupertinoTextFieldTitle),
      ),
      child: SafeArea(
        child: ListView(
          restorationId: 'text_field_demo_list_view',
          padding: const EdgeInsets.all(16),
          children: <Widget>[
            Padding(
              padding: const EdgeInsets.symmetric(vertical: 8),
              child: CupertinoTextField(
                textInputAction: TextInputAction.next,
                restorationId: 'email_address_text_field',
                placeholder: localizations.demoTextFieldEmail,
                keyboardType: TextInputType.emailAddress,
                clearButtonMode: OverlayVisibilityMode.editing,
                autocorrect: false,
              ),
            ),
            Padding(
              padding: const EdgeInsets.symmetric(vertical: 8),
              child: CupertinoTextField(
                textInputAction: TextInputAction.next,
                restorationId: 'login_password_text_field',
                placeholder: localizations.rallyLoginPassword,
                clearButtonMode: OverlayVisibilityMode.editing,
                obscureText: true,
                autocorrect: false,
              ),
            ),
            // Disabled text field
            Padding(
              padding: const EdgeInsets.symmetric(vertical: 8),
              child: CupertinoTextField(
                enabled: false,
                textInputAction: TextInputAction.next,
                restorationId: 'login_password_text_field_disabled',
                placeholder: localizations.rallyLoginPassword,
                clearButtonMode: OverlayVisibilityMode.editing,
                obscureText: true,
                autocorrect: false,
              ),
            ),
            CupertinoTextField(
              textInputAction: TextInputAction.done,
              restorationId: 'pin_number_text_field',
              prefix: const Icon(CupertinoIcons.padlock_solid, size: 28),
              padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 12),
              clearButtonMode: OverlayVisibilityMode.editing,
              keyboardType: TextInputType.number,
              decoration: const BoxDecoration(
                border: Border(bottom: BorderSide(width: 0, color: CupertinoColors.inactiveGray)),
              ),
              placeholder: localizations.demoCupertinoTextFieldPIN,
            ),
          ],
        ),
      ),
    );
  }
}

// END
