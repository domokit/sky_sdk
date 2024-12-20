// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// An example of [AnimationController] and [SlideTransition].

// Occupies the same width as the widest single digit used by AnimatedDigit.
//
// By stacking this widget behind AnimatedDigit's visible digit, we
// ensure that AnimatedWidget's width will not change when its value
// changes.  Typically digits like '8' or '9' are wider than '1'.  If
// an app arranges several AnimatedDigits in a centered Row, we don't
// want the Row to wiggle when the digits change because the overall
// width of the Row changes.
class _PlaceholderDigit extends StatelessWidget {
  const _PlaceholderDigit();

  @override
  Widget build(BuildContext context) {
    final TextStyle textStyle = Theme.of(
      context,
    ).textTheme.displayLarge!.copyWith(fontWeight: FontWeight.w500);

    final Iterable<Widget> placeholderDigits = <int>[0, 1, 2, 3, 4, 5, 6, 7, 8, 9].map<Widget>((
      int n,
    ) {
      return Text('$n', style: textStyle);
    });

    return Opacity(opacity: 0, child: Stack(children: placeholderDigits.toList()));
  }
}

// Displays a single digit [value].
//
// When the value changes the old value slides upwards and out of sight
// at the same as the new value slides into view.
class AnimatedDigit extends StatefulWidget {
  const AnimatedDigit({super.key, required this.value});

  final int value;

  @override
  State<AnimatedDigit> createState() => _AnimatedDigitState();
}

class _AnimatedDigitState extends State<AnimatedDigit> with SingleTickerProviderStateMixin {
  static const Duration defaultDuration = Duration(milliseconds: 300);

  late final AnimationController controller;
  late int incomingValue;
  late int outgoingValue;
  List<int> pendingValues =
      <int>[]; // widget.value updates that occurred while the animation is underway
  Duration duration = defaultDuration;

  @override
  void initState() {
    super.initState();
    controller = AnimationController(duration: duration, vsync: this);
    controller.addStatusListener(handleAnimationCompleted);
    incomingValue = widget.value;
    outgoingValue = widget.value;
  }

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  void handleAnimationCompleted(AnimationStatus status) {
    if (status.isCompleted) {
      if (pendingValues.isNotEmpty) {
        // Display the next pending value. The duration was scaled down
        // in didUpdateWidget by the total number of pending values so
        // that all of the pending changes are shown within
        // defaultDuration of the last one (the past pending change).
        controller.duration = duration;
        animateValueUpdate(incomingValue, pendingValues.removeAt(0));
      } else {
        controller.duration = defaultDuration;
      }
    }
  }

  void animateValueUpdate(int outgoing, int incoming) {
    setState(() {
      outgoingValue = outgoing;
      incomingValue = incoming;
      controller.forward(from: 0);
    });
  }

  // Rebuilding the widget with a new value causes the animations to run.
  // If the widget is updated while the value is being changed the new
  // value is added to pendingValues and is taken care of when the current
  // animation is complete (see handleAnimationCompleted()).
  @override
  void didUpdateWidget(AnimatedDigit oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.value != oldWidget.value) {
      if (controller.isAnimating) {
        // We're in the middle of animating outgoingValue out and
        // incomingValue in. Shorten the duration of the current
        // animation as well as the duration for animations that
        // will show the pending values.
        pendingValues.add(widget.value);
        final double percentRemaining = 1 - controller.value;
        duration = defaultDuration * (1 / (percentRemaining + pendingValues.length));
        controller.animateTo(1.0, duration: duration * percentRemaining);
      } else {
        animateValueUpdate(incomingValue, widget.value);
      }
    }
  }

  // When the controller runs forward both SlideTransitions' children
  // animate upwards. This takes the outgoingValue out of sight and the
  // incoming value into view. See animateValueUpdate().
  @override
  Widget build(BuildContext context) {
    final TextStyle textStyle = Theme.of(context).textTheme.displayLarge!;
    return ClipRect(
      child: Stack(
        children: <Widget>[
          const _PlaceholderDigit(),
          SlideTransition(
            position: controller.drive(
              Tween<Offset>(
                begin: Offset.zero,
                end: const Offset(0, -1), // Out of view above the top.
              ),
            ),
            child: Text(key: ValueKey<int>(outgoingValue), '$outgoingValue', style: textStyle),
          ),
          SlideTransition(
            position: controller.drive(
              Tween<Offset>(
                begin: const Offset(0, 1), // Out of view below the bottom.
                end: Offset.zero,
              ),
            ),
            child: Text(key: ValueKey<int>(incomingValue), '$incomingValue', style: textStyle),
          ),
        ],
      ),
    );
  }
}

class AnimatedDigitApp extends StatelessWidget {
  const AnimatedDigitApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(title: 'AnimatedDigit', home: AnimatedDigitHome());
  }
}

class AnimatedDigitHome extends StatefulWidget {
  const AnimatedDigitHome({super.key});

  @override
  State<AnimatedDigitHome> createState() => _AnimatedDigitHomeState();
}

class _AnimatedDigitHomeState extends State<AnimatedDigitHome> {
  int value = 0;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(child: AnimatedDigit(value: value % 10)),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          setState(() {
            value += 1;
          });
        },
        tooltip: 'Increment Digit',
        child: const Icon(Icons.add),
      ),
    );
  }
}

void main() {
  runApp(const AnimatedDigitApp());
}
