// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

class ProgressIndicatorDemo extends StatefulWidget {
  const ProgressIndicatorDemo({super.key});

  static const String routeName = '/material/progress-indicator';

  @override
  State<ProgressIndicatorDemo> createState() => _ProgressIndicatorDemoState();
}

class _ProgressIndicatorDemoState extends State<ProgressIndicatorDemo>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;
  late Animation<double> _animation;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
      animationBehavior: AnimationBehavior.preserve,
    )..forward();

    _animation = CurvedAnimation(
      parent: _controller,
      curve: const Interval(0.0, 0.9, curve: Curves.fastOutSlowIn),
      reverseCurve: Curves.fastOutSlowIn,
    )..addStatusListener((AnimationStatus status) {
      if (status == AnimationStatus.dismissed) {
        _controller.forward();
      } else if (status == AnimationStatus.completed) {
        _controller.reverse();
      }
    });
  }

  @override
  void dispose() {
    _controller.stop();
    super.dispose();
  }

  void _handleTap() {
    setState(() {
      // valueAnimation.isAnimating is part of our build state
      if (_controller.isAnimating) {
        _controller.stop();
      } else {
        switch (_controller.status) {
          case AnimationStatus.dismissed:
          case AnimationStatus.forward:
            _controller.forward();
          case AnimationStatus.reverse:
          case AnimationStatus.completed:
            _controller.reverse();
        }
      }
    });
  }

  Widget _buildIndicators(BuildContext context, Widget? child) {
    final List<Widget> indicators = <Widget>[
      const SizedBox(width: 200.0, child: LinearProgressIndicator()),
      const LinearProgressIndicator(),
      const LinearProgressIndicator(),
      LinearProgressIndicator(value: _animation.value),
      Row(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: <Widget>[
          const CircularProgressIndicator(),
          SizedBox(
            width: 20.0,
            height: 20.0,
            child: CircularProgressIndicator(value: _animation.value),
          ),
          SizedBox(
            width: 100.0,
            height: 20.0,
            child: Text(
              '${(_animation.value * 100.0).toStringAsFixed(1)}%',
              textAlign: TextAlign.right,
            ),
          ),
        ],
      ),
    ];
    return Column(
      children:
          indicators
              .map<Widget>(
                (Widget c) => Container(
                  margin: const EdgeInsets.symmetric(vertical: 15.0, horizontal: 20.0),
                  child: c,
                ),
              )
              .toList(),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Progress indicators'),
        actions: <Widget>[MaterialDemoDocumentationButton(ProgressIndicatorDemo.routeName)],
      ),
      body: Center(
        child: SingleChildScrollView(
          child: DefaultTextStyle(
            style: Theme.of(context).textTheme.titleLarge!,
            child: GestureDetector(
              onTap: _handleTap,
              behavior: HitTestBehavior.opaque,
              child: SafeArea(
                top: false,
                bottom: false,
                child: Container(
                  padding: const EdgeInsets.symmetric(vertical: 12.0, horizontal: 8.0),
                  child: AnimatedBuilder(animation: _animation, builder: _buildIndicators),
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
