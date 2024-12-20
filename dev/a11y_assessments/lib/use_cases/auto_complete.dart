// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../utils.dart';
import 'use_cases.dart';

class AutoCompleteUseCase extends UseCase {
  @override
  String get name => 'AutoComplete';

  @override
  String get route => '/auto-complete';

  @override
  Widget build(BuildContext context) => const _MainWidget();
}

class _MainWidget extends StatefulWidget {
  const _MainWidget();

  @override
  State<_MainWidget> createState() => _MainWidgetState();
}

class _MainWidgetState extends State<_MainWidget> {
  static const List<String> _kOptions = <String>['apple', 'banana', 'lemon'];

  static Widget _fieldViewBuilder(
    BuildContext context,
    TextEditingController textEditingController,
    FocusNode focusNode,
    VoidCallback onFieldSubmitted,
  ) {
    return TextFormField(
      focusNode: focusNode,
      controller: textEditingController,
      onFieldSubmitted: (String value) {
        onFieldSubmitted();
      },
    );
  }

  String pageTitle = getUseCaseName(AutoCompleteUseCase());

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Semantics(headingLevel: 1, child: Text('$pageTitle Demo')),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text('Type below to autocomplete the following possible results: $_kOptions.'),
            Autocomplete<String>(
              optionsBuilder: (TextEditingValue textEditingValue) {
                if (textEditingValue.text == '') {
                  return const Iterable<String>.empty();
                }
                return _kOptions.where((String option) {
                  return option.contains(textEditingValue.text.toLowerCase());
                });
              },
              fieldViewBuilder: _fieldViewBuilder,
            ),
          ],
        ),
      ),
    );
  }
}
