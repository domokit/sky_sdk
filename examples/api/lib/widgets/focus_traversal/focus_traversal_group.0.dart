// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [FocusTraversalGroup].

void main() => runApp(const FocusTraversalGroupExampleApp());

class FocusTraversalGroupExampleApp extends StatelessWidget {
  const FocusTraversalGroupExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: FocusTraversalGroupExample());
  }
}

/// A button wrapper that adds either a numerical or lexical order, depending on
/// the type of T.
class OrderedButton<T> extends StatefulWidget {
  const OrderedButton({
    super.key,
    required this.name,
    this.canRequestFocus = true,
    this.autofocus = false,
    required this.order,
  });

  final String name;
  final bool canRequestFocus;
  final bool autofocus;
  final T order;

  @override
  State<OrderedButton<T>> createState() => _OrderedButtonState<T>();
}

class _OrderedButtonState<T> extends State<OrderedButton<T>> {
  late FocusNode focusNode;

  @override
  void initState() {
    super.initState();
    focusNode = FocusNode(debugLabel: widget.name, canRequestFocus: widget.canRequestFocus);
  }

  @override
  void dispose() {
    focusNode.dispose();
    super.dispose();
  }

  @override
  void didUpdateWidget(OrderedButton<T> oldWidget) {
    super.didUpdateWidget(oldWidget);
    focusNode.canRequestFocus = widget.canRequestFocus;
  }

  void _handleOnPressed() {
    focusNode.requestFocus();
    debugPrint('Button ${widget.name} pressed.');
    debugDumpFocusTree();
  }

  @override
  Widget build(BuildContext context) {
    final FocusOrder order = switch (widget.order) {
      final num number => NumericFocusOrder(number.toDouble()),
      final Object? object => LexicalFocusOrder(object.toString()),
    };

    return FocusTraversalOrder(
      order: order,
      child: Padding(
        padding: const EdgeInsets.all(8.0),
        child: OutlinedButton(
          focusNode: focusNode,
          autofocus: widget.autofocus,
          style: const ButtonStyle(
            overlayColor: WidgetStateProperty<Color?>.fromMap(
              // If neither of these states is active, the property will
              // resolve to null, deferring to the default overlay color.
              <WidgetState, Color>{
                WidgetState.focused: Colors.red,
                WidgetState.hovered: Colors.blue,
              },
            ),
            foregroundColor: WidgetStateProperty<Color?>.fromMap(
              // "WidgetState.focused | WidgetState.hovered" could be used
              // instead of separate map keys, but this setup allows setting
              // the button style to a constant value for improved efficiency.
              <WidgetState, Color>{
                WidgetState.focused: Colors.white,
                WidgetState.hovered: Colors.white,
              },
            ),
          ),
          onPressed: () => _handleOnPressed(),
          child: Text(widget.name),
        ),
      ),
    );
  }
}

class FocusTraversalGroupExample extends StatelessWidget {
  const FocusTraversalGroupExample({super.key});

  @override
  Widget build(BuildContext context) {
    return ColoredBox(
      color: Colors.white,
      child: FocusTraversalGroup(
        policy: OrderedTraversalPolicy(),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            // A group that is ordered with a numerical order, from left to right.
            FocusTraversalGroup(
              policy: OrderedTraversalPolicy(),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: List<Widget>.generate(3, (int index) {
                  return OrderedButton<num>(
                    name: 'num: $index',
                    // TRY THIS: change this to "3 - index" and see how the order changes.
                    order: index,
                  );
                }),
              ),
            ),
            // A group that is ordered with a lexical order, from right to left.
            FocusTraversalGroup(
              policy: OrderedTraversalPolicy(),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: List<Widget>.generate(3, (int index) {
                  // Order as "C" "B", "A".
                  final String order = String.fromCharCode('A'.codeUnitAt(0) + (2 - index));
                  return OrderedButton<String>(name: 'String: $order', order: order);
                }),
              ),
            ),
            // A group that orders in widget order, regardless of what the order is set to.
            FocusTraversalGroup(
              // Because this is NOT an OrderedTraversalPolicy, the
              // assigned order of these OrderedButtons is ignored, and they
              // are traversed in widget order. TRY THIS: change this to
              // "OrderedTraversalPolicy()" and see that it now follows the
              // numeric order set on them instead of the widget order.
              policy: WidgetOrderTraversalPolicy(),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: List<Widget>.generate(3, (int index) {
                  return OrderedButton<num>(name: 'ignored num: ${3 - index}', order: 3 - index);
                }),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
