// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery_localizations.dart';
import 'material_demo_types.dart';

class BottomSheetDemo extends StatelessWidget {
  const BottomSheetDemo({super.key, required this.type});

  final BottomSheetDemoType type;

  String _title(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return switch (type) {
      BottomSheetDemoType.persistent => localizations.demoBottomSheetPersistentTitle,
      BottomSheetDemoType.modal => localizations.demoBottomSheetModalTitle,
    };
  }

  Widget _bottomSheetDemo(BuildContext context) {
    return switch (type) {
      BottomSheetDemoType.persistent => _PersistentBottomSheetDemo(),
      BottomSheetDemoType.modal => _ModalBottomSheetDemo(),
    };
  }

  @override
  Widget build(BuildContext context) {
    // We wrap the demo in a [Navigator] to make sure that the modal bottom
    // sheets gets dismissed when changing demo.
    return Navigator(
      // Adding [ValueKey] to make sure that the widget gets rebuilt when
      // changing type.
      key: ValueKey<BottomSheetDemoType>(type),
      onGenerateRoute: (RouteSettings settings) {
        return MaterialPageRoute<void>(
          builder:
              (BuildContext context) => Scaffold(
                appBar: AppBar(automaticallyImplyLeading: false, title: Text(_title(context))),
                floatingActionButton: FloatingActionButton(
                  onPressed: () {},
                  backgroundColor: Theme.of(context).colorScheme.secondary,
                  child: Icon(
                    Icons.add,
                    semanticLabel: GalleryLocalizations.of(context)!.demoBottomSheetAddLabel,
                  ),
                ),
                body: _bottomSheetDemo(context),
              ),
        );
      },
    );
  }
}

// BEGIN bottomSheetDemoModal#1 bottomSheetDemoPersistent#1

class _BottomSheetContent extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return SizedBox(
      height: 300,
      child: Column(
        children: <Widget>[
          SizedBox(
            height: 70,
            child: Center(
              child: Text(localizations.demoBottomSheetHeader, textAlign: TextAlign.center),
            ),
          ),
          const Divider(thickness: 1),
          Expanded(
            child: ListView.builder(
              itemCount: 21,
              itemBuilder: (BuildContext context, int index) {
                return ListTile(title: Text(localizations.demoBottomSheetItem(index)));
              },
            ),
          ),
        ],
      ),
    );
  }
}

// END bottomSheetDemoModal#1 bottomSheetDemoPersistent#1

// BEGIN bottomSheetDemoModal#2

class _ModalBottomSheetDemo extends StatelessWidget {
  void _showModalBottomSheet(BuildContext context) {
    showModalBottomSheet<void>(
      context: context,
      builder: (BuildContext context) {
        return _BottomSheetContent();
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: ElevatedButton(
        onPressed: () {
          _showModalBottomSheet(context);
        },
        child: Text(GalleryLocalizations.of(context)!.demoBottomSheetButtonText),
      ),
    );
  }
}

// END

// BEGIN bottomSheetDemoPersistent#2

class _PersistentBottomSheetDemo extends StatefulWidget {
  @override
  _PersistentBottomSheetDemoState createState() => _PersistentBottomSheetDemoState();
}

class _PersistentBottomSheetDemoState extends State<_PersistentBottomSheetDemo> {
  VoidCallback? _showBottomSheetCallback;

  @override
  void initState() {
    super.initState();
    _showBottomSheetCallback = _showPersistentBottomSheet;
  }

  void _showPersistentBottomSheet() {
    setState(() {
      // Disable the show bottom sheet button.
      _showBottomSheetCallback = null;
    });

    Scaffold.of(context)
        .showBottomSheet((BuildContext context) {
          return _BottomSheetContent();
        }, elevation: 25)
        .closed
        .whenComplete(() {
          if (mounted) {
            setState(() {
              // Re-enable the bottom sheet button.
              _showBottomSheetCallback = _showPersistentBottomSheet;
            });
          }
        });
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: ElevatedButton(
        onPressed: _showBottomSheetCallback,
        child: Text(GalleryLocalizations.of(context)!.demoBottomSheetButtonText),
      ),
    );
  }
}

// END
