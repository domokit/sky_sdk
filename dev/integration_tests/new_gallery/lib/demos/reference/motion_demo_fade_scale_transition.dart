// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:animations/animations.dart';
import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

// BEGIN fadeScaleTransitionDemo

class FadeScaleTransitionDemo extends StatefulWidget {
  const FadeScaleTransitionDemo({super.key});

  @override
  State<FadeScaleTransitionDemo> createState() => _FadeScaleTransitionDemoState();
}

class _FadeScaleTransitionDemoState extends State<FadeScaleTransitionDemo>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      value: 0,
      duration: const Duration(milliseconds: 150),
      reverseDuration: const Duration(milliseconds: 75),
      vsync: this,
    )..addStatusListener((AnimationStatus status) {
      setState(() {
        // setState needs to be called to trigger a rebuild because
        // the 'HIDE FAB'/'SHOW FAB' button needs to be updated based
        // the latest value of [_controller.status].
      });
    });
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  bool get _isAnimationRunningForwardsOrComplete {
    switch (_controller.status) {
      case AnimationStatus.forward:
      case AnimationStatus.completed:
        return true;
      case AnimationStatus.reverse:
      case AnimationStatus.dismissed:
        return false;
    }
  }

  Widget _showExampleAlertDialog() {
    return Theme(data: Theme.of(context), child: _ExampleAlertDialog());
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Column(
          children: <Widget>[
            Text(localizations.demoFadeScaleTitle),
            Text(
              '(${localizations.demoFadeScaleDemoInstructions})',
              style: Theme.of(context).textTheme.titleSmall!.copyWith(color: Colors.white),
            ),
          ],
        ),
      ),
      floatingActionButton: AnimatedBuilder(
        animation: _controller,
        builder: (BuildContext context, Widget? child) {
          return FadeScaleTransition(animation: _controller, child: child);
        },
        child: Visibility(
          visible: !_controller.isDismissed,
          child: FloatingActionButton(onPressed: () {}, child: const Icon(Icons.add)),
        ),
      ),
      bottomNavigationBar: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          const Divider(height: 0),
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 8),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                ElevatedButton(
                  onPressed: () {
                    showModal<void>(
                      context: context,
                      builder: (BuildContext context) => _showExampleAlertDialog(),
                    );
                  },
                  child: Text(localizations.demoFadeScaleShowAlertDialogButton),
                ),
                const SizedBox(width: 10),
                ElevatedButton(
                  onPressed: () {
                    if (_isAnimationRunningForwardsOrComplete) {
                      _controller.reverse();
                    } else {
                      _controller.forward();
                    }
                  },
                  child: Text(
                    _isAnimationRunningForwardsOrComplete
                        ? localizations.demoFadeScaleHideFabButton
                        : localizations.demoFadeScaleShowFabButton,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class _ExampleAlertDialog extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return AlertDialog(
      content: Text(localizations.demoFadeScaleAlertDialogHeader),
      actions: <Widget>[
        TextButton(
          onPressed: () {
            Navigator.of(context).pop();
          },
          child: Text(localizations.demoFadeScaleAlertDialogCancelButton),
        ),
        TextButton(
          onPressed: () {
            Navigator.of(context).pop();
          },
          child: Text(localizations.demoFadeScaleAlertDialogDiscardButton),
        ),
      ],
    );
  }
}

// END fadeScaleTransitionDemo
