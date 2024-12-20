// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Overlay].

void main() => runApp(const OverlayApp());

class OverlayApp extends StatelessWidget {
  const OverlayApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: OverlayExample());
  }
}

class OverlayExample extends StatefulWidget {
  const OverlayExample({super.key});

  @override
  State<OverlayExample> createState() => _OverlayExampleState();
}

class _OverlayExampleState extends State<OverlayExample> {
  OverlayEntry? overlayEntry;
  int currentPageIndex = 0;

  void createHighlightOverlay({
    required AlignmentDirectional alignment,
    required Color borderColor,
  }) {
    // Remove the existing OverlayEntry.
    removeHighlightOverlay();

    assert(overlayEntry == null);

    Widget builder(BuildContext context) {
      final (String label, Color? color) = switch (currentPageIndex) {
        0 => ('Explore page', Colors.red),
        1 => ('Commute page', Colors.green),
        2 => ('Saved page', Colors.orange),
        _ => ('No page selected.', null),
      };
      if (color == null) {
        return Text(label);
      }
      return Column(
        children: <Widget>[
          Text(label, style: TextStyle(color: color)),
          Icon(Icons.arrow_downward, color: color),
        ],
      );
    }

    overlayEntry = OverlayEntry(
      // Create a new OverlayEntry.
      builder: (BuildContext context) {
        // Align is used to position the highlight overlay
        // relative to the NavigationBar destination.
        return SafeArea(
          child: Align(
            alignment: alignment,
            heightFactor: 1.0,
            child: DefaultTextStyle(
              style: const TextStyle(
                color: Colors.blue,
                fontWeight: FontWeight.bold,
                fontSize: 14.0,
              ),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: <Widget>[
                  const Text('Tap here for'),
                  Builder(builder: builder),
                  SizedBox(
                    width: MediaQuery.of(context).size.width / 3,
                    height: 80.0,
                    child: Center(
                      child: Container(
                        decoration: BoxDecoration(
                          border: Border.all(color: borderColor, width: 4.0),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        );
      },
    );

    // Add the OverlayEntry to the Overlay.
    Overlay.of(context, debugRequiredFor: widget).insert(overlayEntry!);
  }

  // Remove the OverlayEntry.
  void removeHighlightOverlay() {
    overlayEntry?.remove();
    overlayEntry?.dispose();
    overlayEntry = null;
  }

  @override
  void dispose() {
    // Make sure to remove OverlayEntry when the widget is disposed.
    removeHighlightOverlay();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Overlay Sample')),
      bottomNavigationBar: NavigationBar(
        selectedIndex: currentPageIndex,
        destinations: const <NavigationDestination>[
          NavigationDestination(icon: Icon(Icons.explore), label: 'Explore'),
          NavigationDestination(icon: Icon(Icons.commute), label: 'Commute'),
          NavigationDestination(
            selectedIcon: Icon(Icons.bookmark),
            icon: Icon(Icons.bookmark_border),
            label: 'Saved',
          ),
        ],
      ),
      body: Center(
        child: Padding(
          padding: const EdgeInsets.all(8.0),
          child: Column(
            spacing: 10.0,
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              Text(
                'Use Overlay to highlight a NavigationBar destination',
                style: Theme.of(context).textTheme.bodyMedium,
              ),
              Wrap(
                spacing: 10.0,
                runSpacing: 10.0,
                alignment: WrapAlignment.center,
                runAlignment: WrapAlignment.center,
                children: <Widget>[
                  // This creates a highlight Overlay for
                  // the Explore item.
                  ElevatedButton(
                    onPressed: () {
                      setState(() {
                        currentPageIndex = 0;
                      });
                      createHighlightOverlay(
                        alignment: AlignmentDirectional.bottomStart,
                        borderColor: Colors.red,
                      );
                    },
                    child: const Text('Explore'),
                  ),
                  // This creates a highlight Overlay for
                  // the Commute item.
                  ElevatedButton(
                    onPressed: () {
                      setState(() {
                        currentPageIndex = 1;
                      });
                      createHighlightOverlay(
                        alignment: AlignmentDirectional.bottomCenter,
                        borderColor: Colors.green,
                      );
                    },
                    child: const Text('Commute'),
                  ),
                  // This creates a highlight Overlay for
                  // the Saved item.
                  ElevatedButton(
                    onPressed: () {
                      setState(() {
                        currentPageIndex = 2;
                      });
                      createHighlightOverlay(
                        alignment: AlignmentDirectional.bottomEnd,
                        borderColor: Colors.orange,
                      );
                    },
                    child: const Text('Saved'),
                  ),
                ],
              ),
              ElevatedButton(
                onPressed: () {
                  removeHighlightOverlay();
                },
                child: const Text('Remove Overlay'),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
