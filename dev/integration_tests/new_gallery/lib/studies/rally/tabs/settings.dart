// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../../layout/adaptive.dart';
import '../colors.dart';
import '../data.dart';
import '../routes.dart' as rally_route;

class SettingsView extends StatefulWidget {
  const SettingsView({super.key});

  @override
  State<SettingsView> createState() => _SettingsViewState();
}

class _SettingsViewState extends State<SettingsView> {
  @override
  Widget build(BuildContext context) {
    return FocusTraversalGroup(
      child: Container(
        padding: EdgeInsets.only(top: isDisplayDesktop(context) ? 24 : 0),
        child: ListView(
          restorationId: 'settings_list_view',
          shrinkWrap: true,
          children: <Widget>[
            for (final String title in DummyDataService.getSettingsTitles(context)) ...<Widget>[
              _SettingsItem(title),
              const Divider(color: RallyColors.dividerColor, height: 1),
            ],
          ],
        ),
      ),
    );
  }
}

class _SettingsItem extends StatelessWidget {
  const _SettingsItem(this.title);

  final String title;

  @override
  Widget build(BuildContext context) {
    return TextButton(
      style: TextButton.styleFrom(foregroundColor: Colors.white, padding: EdgeInsets.zero),
      onPressed: () {
        Navigator.of(context).restorablePushNamed(rally_route.loginRoute);
      },
      child: Container(
        alignment: AlignmentDirectional.centerStart,
        padding: const EdgeInsets.symmetric(vertical: 24, horizontal: 28),
        child: Text(title),
      ),
    );
  }
}
