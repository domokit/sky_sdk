// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SearchAnchor].

void main() => runApp(const SearchBarApp());

class SearchBarApp extends StatefulWidget {
  const SearchBarApp({super.key});

  @override
  State<SearchBarApp> createState() => _SearchBarAppState();
}

class _SearchBarAppState extends State<SearchBarApp> {
  final SearchController controller = SearchController();

  @override
  Widget build(BuildContext context) {
    final ThemeData themeData = ThemeData(useMaterial3: true);

    return MaterialApp(
      theme: themeData,
      home: Scaffold(
        appBar: AppBar(title: const Text('Search Anchor Sample')),
        body: Column(
          children: <Widget>[
            SearchAnchor(
              searchController: controller,
              builder: (BuildContext context, SearchController controller) {
                return IconButton(
                  icon: const Icon(Icons.search),
                  onPressed: () {
                    controller.openView();
                  },
                );
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return List<ListTile>.generate(5, (int index) {
                  final String item = 'item $index';
                  return ListTile(
                    title: Text(item),
                    onTap: () {
                      setState(() {
                        controller.closeView(item);
                      });
                    },
                  );
                });
              },
            ),
            Center(
              child:
                  controller.text.isEmpty
                      ? const Text('No item selected')
                      : Text('Selected item: ${controller.value.text}'),
            ),
          ],
        ),
      ),
    );
  }
}
