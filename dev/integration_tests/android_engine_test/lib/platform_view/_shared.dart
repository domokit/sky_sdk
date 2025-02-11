// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

// This should appear as the yellow line over a blue box. The
// green box should not be visible unless the platform view has not loaded yet.
final class MainApp extends StatefulWidget {
  const MainApp({super.key, required this.platformView});

  final Widget platformView;

  @override
  State<MainApp> createState() => _MainAppState();
}

class _MainAppState extends State<MainApp> {
  bool showPlatformView = true;

  void _togglePlatformView() {
    setState(() {
      showPlatformView = !showPlatformView;
    });
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: Stack(
        alignment: AlignmentDirectional.center,
        children: <Widget>[
          TextButton(
            key: const ValueKey<String>('AddOverlay'),
            onPressed: _togglePlatformView,
            child: const SizedBox(width: 190, height: 190, child: ColoredBox(color: Colors.green)),
          ),
          if (showPlatformView) ...<Widget>[
            SizedBox(width: 200, height: 200, child: widget.platformView),
            TextButton(
              key: const ValueKey<String>('RemoveOverlay'),
              onPressed: _togglePlatformView,
              child: const SizedBox(
                width: 800,
                height: 25,
                child: ColoredBox(color: Colors.yellow),
              ),
            ),
          ],
        ],
      ),
    );
  }
}
