// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Scrollbar].

void main() => runApp(const ScrollbarApp());

class ScrollbarApp extends StatelessWidget {
  const ScrollbarApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('Scrollbar Sample')),
        body: const Center(child: DesktopExample()),
      ),
    );
  }
}

class DesktopExample extends StatefulWidget {
  const DesktopExample({super.key});

  @override
  State<DesktopExample> createState() => _DesktopExampleState();
}

class _DesktopExampleState extends State<DesktopExample> {
  final ScrollController _controller = ScrollController();

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        return Row(
          children: <Widget>[
            SizedBox(
              width: constraints.maxWidth / 2,
              // When running this sample on desktop, two scrollbars will be
              // visible here. One is the default scrollbar and the other is the
              // Scrollbar widget with custom thickness.
              child: Scrollbar(
                thickness: 20.0,
                thumbVisibility: true,
                controller: _controller,
                child: ListView.builder(
                  controller: _controller,
                  itemCount: 100,
                  itemBuilder: (BuildContext context, int index) {
                    return SizedBox(
                      height: 50,
                      child: Padding(
                        padding: const EdgeInsets.all(8.0),
                        child: Text('Scrollable 1 : Index $index'),
                      ),
                    );
                  },
                ),
              ),
            ),
            SizedBox(
              width: constraints.maxWidth / 2,
              // When running this sample on desktop, one scrollbar will be
              // visible here. The default scrollbar is hidden by setting the
              // ScrollConfiguration's scrollbars to false. The Scrollbar widget
              // with custom thickness is visible.
              child: Scrollbar(
                thickness: 20.0,
                thumbVisibility: true,
                child: ScrollConfiguration(
                  behavior: ScrollConfiguration.of(context).copyWith(scrollbars: false),
                  child: ListView.builder(
                    primary: true,
                    itemCount: 100,
                    itemBuilder: (BuildContext context, int index) {
                      return SizedBox(
                        height: 50,
                        child: Padding(
                          padding: const EdgeInsets.all(8.0),
                          child: Text('Scrollable 2 : Index $index'),
                        ),
                      );
                    },
                  ),
                ),
              ),
            ),
          ],
        );
      },
    );
  }
}
