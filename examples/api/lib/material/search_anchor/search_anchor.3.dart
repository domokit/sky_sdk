// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SearchAnchor].

const Duration fakeAPIDuration = Duration(seconds: 1);

void main() => runApp(const SearchAnchorAsyncExampleApp());

class SearchAnchorAsyncExampleApp extends StatelessWidget {
  const SearchAnchorAsyncExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('SearchAnchor - async')),
        body: const Center(child: _AsyncSearchAnchor()),
      ),
    );
  }
}

class _AsyncSearchAnchor extends StatefulWidget {
  const _AsyncSearchAnchor();

  @override
  State<_AsyncSearchAnchor> createState() => _AsyncSearchAnchorState();
}

class _AsyncSearchAnchorState extends State<_AsyncSearchAnchor> {
  // The query currently being searched for. If null, there is no pending
  // request.
  String? _searchingWithQuery;

  // The most recent options received from the API.
  late Iterable<Widget> _lastOptions = <Widget>[];

  @override
  Widget build(BuildContext context) {
    return SearchAnchor(
      builder: (BuildContext context, SearchController controller) {
        return IconButton(
          icon: const Icon(Icons.search),
          onPressed: () {
            controller.openView();
          },
        );
      },
      suggestionsBuilder: (BuildContext context, SearchController controller) async {
        _searchingWithQuery = controller.text;
        final List<String> options = (await _FakeAPI.search(_searchingWithQuery!)).toList();

        // If another search happened after this one, throw away these options.
        // Use the previous options instead and wait for the newer request to
        // finish.
        if (_searchingWithQuery != controller.text) {
          return _lastOptions;
        }

        _lastOptions = List<ListTile>.generate(options.length, (int index) {
          final String item = options[index];
          return ListTile(title: Text(item));
        });

        return _lastOptions;
      },
    );
  }
}

// Mimics a remote API.
class _FakeAPI {
  static const List<String> _kOptions = <String>['aardvark', 'bobcat', 'chameleon'];

  // Searches the options, but injects a fake "network" delay.
  static Future<Iterable<String>> search(String query) async {
    await Future<void>.delayed(fakeAPIDuration); // Fake 1 second delay.
    if (query == '') {
      return const Iterable<String>.empty();
    }
    return _kOptions.where((String option) {
      return option.contains(query.toLowerCase());
    });
  }
}
