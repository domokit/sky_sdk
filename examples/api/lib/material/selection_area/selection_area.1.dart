// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';

/// Flutter code sample for [SelectionArea].

void main() => runApp(const SelectionAreaSelectionListenerExampleApp());

class SelectionAreaSelectionListenerExampleApp extends StatelessWidget {
  const SelectionAreaSelectionListenerExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple)),
      home: const MyHomePage(title: 'Flutter Demo Home Page'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});

  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  final SelectionListenerNotifier _selectionNotifier = SelectionListenerNotifier();
  SelectableRegionSelectionStatus? _selectableRegionStatus;

  void _handleOnSelectionStateChanged(SelectableRegionSelectionStatus status) {
    setState(() {
      _selectableRegionStatus = status;
    });
  }

  @override
  void dispose() {
    _selectionNotifier.dispose();
    _selectableRegionStatus = null;
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(widget.title),
      ),
      body: Center(
        child: Column(
          children: <Widget>[
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                for (final (int? offset, String label) in <(int? offset, String label)>[
                  (
                    _selectionNotifier.registered
                        ? _selectionNotifier.selection.range?.startOffset
                        : null,
                    'StartOffset',
                  ),
                  (
                    _selectionNotifier.registered
                        ? _selectionNotifier.selection.range?.endOffset
                        : null,
                    'EndOffset',
                  ),
                ])
                  Text('Selection $label: $offset'),
                Text(
                  'Selection Status: ${_selectionNotifier.registered ? _selectionNotifier.selection.status : 'SelectionListenerNotifier not registered.'}',
                ),
                Text('Selectable Region Status: $_selectableRegionStatus'),
              ],
            ),
            const SizedBox(height: 15.0),
            SelectionArea(
              child: MySelectableText(
                selectionNotifier: _selectionNotifier,
                onChanged: _handleOnSelectionStateChanged,
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class MySelectableText extends StatefulWidget {
  const MySelectableText({super.key, required this.selectionNotifier, required this.onChanged});

  final SelectionListenerNotifier selectionNotifier;
  final ValueChanged<SelectableRegionSelectionStatus> onChanged;

  @override
  State<MySelectableText> createState() => _MySelectableTextState();
}

class _MySelectableTextState extends State<MySelectableText> {
  ValueListenable<SelectableRegionSelectionStatus>? _selectableRegionScope;

  void _handleOnSelectableRegionChanged() {
    if (_selectableRegionScope == null) {
      return;
    }
    widget.onChanged.call(_selectableRegionScope!.value);
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _selectableRegionScope?.removeListener(_handleOnSelectableRegionChanged);
    _selectableRegionScope = SelectableRegionSelectionStatusScope.maybeOf(context);
    _selectableRegionScope?.addListener(_handleOnSelectableRegionChanged);
  }

  @override
  void dispose() {
    _selectableRegionScope?.removeListener(_handleOnSelectableRegionChanged);
    _selectableRegionScope = null;
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return SelectionListener(
      selectionNotifier: widget.selectionNotifier,
      child: const Text('This is some text under a SelectionArea that can be selected.'),
    );
  }
}
