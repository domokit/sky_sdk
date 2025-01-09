// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

// BEGIN cupertinoSearchTextFieldDemo

class CupertinoSearchTextFieldDemo extends StatefulWidget {
  const CupertinoSearchTextFieldDemo({super.key});

  @override
  State<CupertinoSearchTextFieldDemo> createState() => _CupertinoSearchTextFieldDemoState();
}

class _CupertinoSearchTextFieldDemoState extends State<CupertinoSearchTextFieldDemo> {
  final List<String> platforms = <String>['Android', 'iOS', 'Windows', 'Linux', 'MacOS', 'Web'];

  final TextEditingController _queryTextController = TextEditingController();
  String _searchPlatform = '';
  List<String> filteredPlatforms = <String>[];

  @override
  void initState() {
    super.initState();
    filteredPlatforms = platforms;
    _queryTextController.addListener(() {
      if (_queryTextController.text.isEmpty) {
        setState(() {
          _searchPlatform = '';
          filteredPlatforms = platforms;
        });
      } else {
        setState(() {
          _searchPlatform = _queryTextController.text;
        });
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        automaticallyImplyLeading: false,
        middle: Text(localizations.demoCupertinoSearchTextFieldTitle),
      ),
      child: SafeArea(
        child: Column(
          children: <Widget>[
            CupertinoSearchTextField(
              controller: _queryTextController,
              restorationId: 'search_text_field',
              padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 12),
              decoration: const BoxDecoration(
                border: Border(bottom: BorderSide(width: 0, color: CupertinoColors.inactiveGray)),
              ),
              placeholder: localizations.demoCupertinoSearchTextFieldPlaceholder,
            ),
            _buildPlatformList(),
          ],
        ),
      ),
    );
  }

  Widget _buildPlatformList() {
    if (_searchPlatform.isNotEmpty) {
      final String search = _searchPlatform.toLowerCase();
      filteredPlatforms = <String>[
        for (final String platform in filteredPlatforms)
          if (platform.toLowerCase().contains(search)) platform,
      ];
    }
    return ListView.builder(
      itemCount: filteredPlatforms.length,
      shrinkWrap: true,
      itemBuilder: (BuildContext context, int index) {
        return ListTile(title: Text(filteredPlatforms[index]));
      },
    );
  }
}

// END
