// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [FocusScope].

void main() => runApp(const FocusScopeExampleApp());

class FocusScopeExampleApp extends StatelessWidget {
  const FocusScopeExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: FocusScopeExample());
  }
}

/// A demonstration pane.
///
/// This is just a separate widget to simplify the example.
class Pane extends StatelessWidget {
  const Pane({
    super.key,
    required this.focusNode,
    this.onPressed,
    required this.backgroundColor,
    required this.icon,
    this.child,
  });

  final FocusNode focusNode;
  final VoidCallback? onPressed;
  final Color backgroundColor;
  final Widget icon;
  final Widget? child;

  @override
  Widget build(BuildContext context) {
    return Material(
      color: backgroundColor,
      child: Stack(
        fit: StackFit.expand,
        children: <Widget>[
          Center(child: child),
          Align(
            alignment: Alignment.topLeft,
            child: IconButton(
              autofocus: true,
              focusNode: focusNode,
              onPressed: onPressed,
              icon: icon,
            ),
          ),
        ],
      ),
    );
  }
}

class FocusScopeExample extends StatefulWidget {
  const FocusScopeExample({super.key});

  @override
  State<FocusScopeExample> createState() => _FocusScopeExampleState();
}

class _FocusScopeExampleState extends State<FocusScopeExample> {
  bool backdropIsVisible = false;
  FocusNode backdropNode = FocusNode(debugLabel: 'Close Backdrop Button');
  FocusNode foregroundNode = FocusNode(debugLabel: 'Option Button');

  @override
  void dispose() {
    backdropNode.dispose();
    foregroundNode.dispose();
    super.dispose();
  }

  Widget _buildStack(BuildContext context, BoxConstraints constraints) {
    final Size stackSize = constraints.biggest;
    return Stack(
      fit: StackFit.expand,
      // The backdrop is behind the front widget in the Stack, but the widgets
      // would still be active and traversable without the FocusScope.
      children: <Widget>[
        // TRY THIS: Try removing this FocusScope entirely to see how it affects
        // the behavior. Without this FocusScope, the "ANOTHER BUTTON TO FOCUS"
        // button, and the IconButton in the backdrop Pane would be focusable
        // even when the backdrop wasn't visible.
        FocusScope(
          // TRY THIS: Try commenting out this line. Notice that the focus
          // starts on the backdrop and is stuck there? It seems like the app is
          // non-responsive, but it actually isn't. This line makes sure that
          // this focus scope and its children can't be focused when they're not
          // visible. It might help to make the background color of the
          // foreground pane semi-transparent to see it clearly.
          canRequestFocus: backdropIsVisible,
          child: Pane(
            icon: const Icon(Icons.close),
            focusNode: backdropNode,
            backgroundColor: Colors.lightBlue,
            onPressed: () => setState(() => backdropIsVisible = false),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                // This button would be not visible, but still focusable from
                // the foreground pane without the FocusScope.
                ElevatedButton(
                  onPressed: () => debugPrint('You pressed the other button!'),
                  child: const Text('ANOTHER BUTTON TO FOCUS'),
                ),
                DefaultTextStyle(
                  style: Theme.of(context).textTheme.displayMedium!,
                  child: const Text('BACKDROP'),
                ),
              ],
            ),
          ),
        ),
        AnimatedPositioned(
          curve: Curves.easeInOut,
          duration: const Duration(milliseconds: 300),
          top: backdropIsVisible ? stackSize.height * 0.9 : 0.0,
          width: stackSize.width,
          height: stackSize.height,
          onEnd: () {
            if (backdropIsVisible) {
              backdropNode.requestFocus();
            } else {
              foregroundNode.requestFocus();
            }
          },
          child: Pane(
            icon: const Icon(Icons.menu),
            focusNode: foregroundNode,
            // TRY THIS: Try changing this to Colors.green.withOpacity(0.8) to see for
            // yourself that the hidden components do/don't get focus.
            backgroundColor: Colors.green,
            onPressed: backdropIsVisible ? null : () => setState(() => backdropIsVisible = true),
            child: DefaultTextStyle(
              style: Theme.of(context).textTheme.displayMedium!,
              child: const Text('FOREGROUND'),
            ),
          ),
        ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    // Use a LayoutBuilder so that we can base the size of the stack on the size
    // of its parent.
    return LayoutBuilder(builder: _buildStack);
  }
}
