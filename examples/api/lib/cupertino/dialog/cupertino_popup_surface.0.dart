// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoPopupSurface].

void main() => runApp(const PopupSurfaceApp());

class PopupSurfaceApp extends StatelessWidget {
  const PopupSurfaceApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(home: PopupSurfaceExample());
  }
}

class PopupSurfaceExample extends StatefulWidget {
  const PopupSurfaceExample({super.key});

  @override
  State<PopupSurfaceExample> createState() => _PopupSurfaceExampleState();
}

class _PopupSurfaceExampleState extends State<PopupSurfaceExample> {
  bool _shouldPaintSurface = true;

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      child: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Row(
              mainAxisSize: MainAxisSize.min,
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: <Widget>[
                const Text('Paint surface'),
                const SizedBox(width: 16.0),
                CupertinoSwitch(
                  value: _shouldPaintSurface,
                  onChanged: (bool value) => setState(() => _shouldPaintSurface = value),
                ),
              ],
            ),
            CupertinoButton(
              onPressed: () => _showPopupSurface(context),
              child: const Text('Show popup'),
            ),
          ],
        ),
      ),
    );
  }

  void _showPopupSurface(BuildContext context) {
    showCupertinoModalPopup<void>(
      context: context,
      builder: (BuildContext context) {
        return CupertinoPopupSurface(
          isSurfacePainted: _shouldPaintSurface,
          child: Container(
            height: 240,
            padding: const EdgeInsets.all(8.0),
            child: Column(
              children: <Widget>[
                Expanded(
                  child: Container(
                    alignment: Alignment.center,
                    decoration:
                        _shouldPaintSurface
                            ? null
                            : BoxDecoration(
                              color: CupertinoTheme.of(context).scaffoldBackgroundColor,
                              borderRadius: BorderRadius.circular(8.0),
                            ),
                    child: const Text('This is a popup surface.'),
                  ),
                ),
                const SizedBox(height: 8.0),
                SizedBox(
                  width: double.infinity,
                  child: CupertinoButton(
                    color:
                        _shouldPaintSurface
                            ? null
                            : CupertinoTheme.of(context).scaffoldBackgroundColor,
                    onPressed: () => Navigator.pop(context),
                    child: const Text('Close', style: TextStyle(color: CupertinoColors.systemBlue)),
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}
