// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [OverlayPortal].

void main() => runApp(const OverlayPortalExampleApp());

class OverlayPortalExampleApp extends StatelessWidget {
  const OverlayPortalExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('OverlayPortal Example')),
        body: const Center(child: ClickableTooltipWidget()),
      ),
    );
  }
}

class ClickableTooltipWidget extends StatefulWidget {
  const ClickableTooltipWidget({super.key});

  @override
  State<StatefulWidget> createState() => ClickableTooltipWidgetState();
}

class ClickableTooltipWidgetState extends State<ClickableTooltipWidget> {
  final OverlayPortalController _tooltipController = OverlayPortalController();

  @override
  Widget build(BuildContext context) {
    return TextButton(
      onPressed: _tooltipController.toggle,
      child: DefaultTextStyle(
        style: DefaultTextStyle.of(context).style.copyWith(fontSize: 50),
        child: OverlayPortal(
          controller: _tooltipController,
          overlayChildBuilder: (BuildContext context) {
            return const Positioned(
              right: 50,
              bottom: 50,
              child: ColoredBox(color: Colors.amberAccent, child: Text('tooltip')),
            );
          },
          child: const Text('Press to show/hide tooltip'),
        ),
      ),
    );
  }
}
