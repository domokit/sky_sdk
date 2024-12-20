// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Autocomplete] that shows how to fetch the options
/// from a remote API.

const Duration fakeAPIDuration = Duration(seconds: 1);

void main() => runApp(const AutocompleteExampleApp());

class AutocompleteExampleApp extends StatelessWidget {
  const AutocompleteExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('Autocomplete - async')),
        body: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              Text(
                'Type below to autocomplete the following possible results: ${_FakeAPI._kOptions}.',
              ),
              const _AsyncAutocomplete(),
            ],
          ),
        ),
      ),
    );
  }
}

class _AsyncAutocomplete extends StatefulWidget {
  const _AsyncAutocomplete();

  @override
  State<_AsyncAutocomplete> createState() => _AsyncAutocompleteState();
}

class _AsyncAutocompleteState extends State<_AsyncAutocomplete> {
  // The query currently being searched for. If null, there is no pending
  // request.
  String? _searchingWithQuery;

  // The most recent options received from the API.
  late Iterable<String> _lastOptions = <String>[];

  @override
  Widget build(BuildContext context) {
    return Autocomplete<String>(
      optionsBuilder: (TextEditingValue textEditingValue) async {
        _searchingWithQuery = textEditingValue.text;
        final Iterable<String> options = await _FakeAPI.search(_searchingWithQuery!);

        // If another search happened after this one, throw away these options.
        // Use the previous options instead and wait for the newer request to
        // finish.
        if (_searchingWithQuery != textEditingValue.text) {
          return _lastOptions;
        }

        _lastOptions = options;
        return options;
      },
      onSelected: (String selection) {
        debugPrint('You just selected $selection');
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
