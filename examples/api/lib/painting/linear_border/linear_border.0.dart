// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Examples for [LinearBorder] and [LinearBorderEdge].

void main() {
  runApp(const ExampleApp());
}

class ExampleApp extends StatelessWidget {
  const ExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData.light(useMaterial3: true),
      home: const Directionality(
        // TRY THIS: Switch to TextDirection.rtl to see how the borders change.
        textDirection: TextDirection.ltr,
        child: Home(),
      ),
    );
  }
}

class SampleCard extends StatelessWidget {
  const SampleCard({
    super.key,
    required this.title,
    required this.subtitle,
    required this.children,
  });

  final String title;
  final String subtitle;
  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final TextTheme textTheme = theme.textTheme;
    final ColorScheme colorScheme = theme.colorScheme;

    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            Text(title, style: textTheme.titleMedium),
            Text(subtitle, style: textTheme.bodyMedium!.copyWith(color: colorScheme.secondary)),
            const SizedBox(height: 16),
            Row(
              children: List<Widget>.generate(children.length * 2 - 1, (int index) {
                return index.isEven ? children[index ~/ 2] : const SizedBox(width: 16);
              }),
            ),
          ],
        ),
      ),
    );
  }
}

class Home extends StatefulWidget {
  const Home({super.key});

  @override
  State<Home> createState() => _HomeState();
}

class _HomeState extends State<Home> {
  final LinearBorder shape0 = LinearBorder.top();
  final LinearBorder shape1 = LinearBorder.top(size: 0);
  late LinearBorder shape = shape0;

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    final BorderSide primarySide0 = BorderSide(
      width: 0,
      color: colorScheme.inversePrimary,
    ); // hairline
    final BorderSide primarySide2 = BorderSide(width: 2, color: colorScheme.onPrimaryContainer);
    final BorderSide primarySide3 = BorderSide(width: 3, color: colorScheme.inversePrimary);

    return Scaffold(
      body: SingleChildScrollView(
        child: Padding(
          padding: const EdgeInsets.all(16),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              // Demonstrates using LinearBorder.bottom() to define
              // an underline border for the standard button types.
              // The underline's color and width is defined by the ButtonStyle's
              // side parameter. The side can also be specified as a
              // LinearBorder parameter and if both are specified then the
              // ButtonStyle's side is used. This set up makes it possible
              // for a button theme to specify the shape and for individual
              // buttons to specify the shape border's color and width.
              SampleCard(
                title: 'LinearBorder.bottom()',
                subtitle: 'Standard button widgets',
                children: <Widget>[
                  TextButton(
                    style: TextButton.styleFrom(side: primarySide3, shape: LinearBorder.bottom()),
                    onPressed: () {},
                    child: const Text('Text'),
                  ),
                  OutlinedButton(
                    style: OutlinedButton.styleFrom(
                      side: primarySide3,
                      shape: LinearBorder.bottom(),
                    ),
                    onPressed: () {},
                    child: const Text('Outlined'),
                  ),
                  ElevatedButton(
                    style: ElevatedButton.styleFrom(
                      side: primarySide3,
                      shape: LinearBorder.bottom(),
                    ),
                    onPressed: () {},
                    child: const Text('Elevated'),
                  ),
                ],
              ),
              const SizedBox(height: 32),
              // Demonstrates creating LinearBorders with a single edge
              // by using the convenience constructors like LinearBorder.start().
              // The edges are drawn with a BorderSide with width:0, which
              // means that a "hairline" line is stroked. Wider borders are
              // drawn with filled rectangles.
              SampleCard(
                title: 'LinearBorder',
                subtitle: 'Convenience constructors',
                children: <Widget>[
                  TextButton(
                    style: TextButton.styleFrom(side: primarySide0, shape: LinearBorder.start()),
                    onPressed: () {},
                    child: const Text('Start()'),
                  ),
                  TextButton(
                    style: TextButton.styleFrom(side: primarySide0, shape: LinearBorder.end()),
                    onPressed: () {},
                    child: const Text('End()'),
                  ),
                  TextButton(
                    style: TextButton.styleFrom(side: primarySide0, shape: LinearBorder.top()),
                    onPressed: () {},
                    child: const Text('Top()'),
                  ),
                  TextButton(
                    style: TextButton.styleFrom(side: primarySide0, shape: LinearBorder.bottom()),
                    onPressed: () {},
                    child: const Text('Bottom()'),
                  ),
                ],
              ),
              const SizedBox(height: 32),
              // Demonstrates creating LinearBorders with a single edge
              // that's smaller than the button's bounding box. The size
              // parameter specifies a percentage of the available space
              // and alignment is -1 for start-alignment, 0 for centered,
              // and 1 for end-alignment.
              SampleCard(
                title: 'LinearBorder',
                subtitle: 'Size and alignment parameters',
                children: <Widget>[
                  TextButton(
                    style: TextButton.styleFrom(
                      side: primarySide2,
                      shape: LinearBorder.bottom(size: 0.5),
                    ),
                    onPressed: () {},
                    child: const Text('Center'),
                  ),
                  TextButton(
                    style: TextButton.styleFrom(
                      side: primarySide2,
                      shape: LinearBorder.bottom(size: 0.75, alignment: -1),
                    ),
                    onPressed: () {},
                    child: const Text('Start'),
                  ),
                  TextButton(
                    style: TextButton.styleFrom(
                      side: primarySide2,
                      shape: LinearBorder.bottom(size: 0.75, alignment: 1),
                    ),
                    onPressed: () {},
                    child: const Text('End'),
                  ),
                ],
              ),
              const SizedBox(height: 32),
              // Demonstrates creating LinearBorders with more than one edge.
              // In these cases the default constructor is used and each edge
              // is defined with one LinearBorderEdge object.
              SampleCard(
                title: 'LinearBorder',
                subtitle: 'LinearBorderEdge parameters',
                children: <Widget>[
                  TextButton(
                    style: TextButton.styleFrom(
                      side: primarySide0,
                      shape: const LinearBorder(
                        top: LinearBorderEdge(),
                        bottom: LinearBorderEdge(),
                      ),
                    ),
                    onPressed: () {},
                    child: const Text('Horizontal'),
                  ),
                  TextButton(
                    style: TextButton.styleFrom(
                      side: primarySide0,
                      shape: const LinearBorder(start: LinearBorderEdge(), end: LinearBorderEdge()),
                    ),
                    onPressed: () {},
                    child: const Text('Vertical'),
                  ),
                  TextButton(
                    style: TextButton.styleFrom(
                      side: primarySide0,
                      shape: const LinearBorder(
                        start: LinearBorderEdge(),
                        bottom: LinearBorderEdge(),
                      ),
                    ),
                    onPressed: () {},
                    child: const Text('Corner'),
                  ),
                ],
              ),
              const SizedBox(height: 32),
              // Demonstrates that changing properties of LinearBorders
              // causes them to animate to their new configuration.
              SampleCard(
                title: 'Interpolation',
                subtitle: 'LinearBorder.top() => LinearBorder.top(size: 0)',
                children: <Widget>[
                  IconButton(
                    icon: const Icon(Icons.play_arrow),
                    onPressed: () {
                      setState(() {
                        shape = shape == shape0 ? shape1 : shape0;
                      });
                    },
                  ),
                  TextButton(
                    style: TextButton.styleFrom(side: primarySide3, shape: shape),
                    onPressed: () {},
                    child: const Text('Press Play'),
                  ),
                  TextButton(
                    style: ButtonStyle(
                      side: WidgetStateProperty.resolveWith<BorderSide?>((Set<WidgetState> states) {
                        return states.contains(WidgetState.hovered) ? primarySide3 : null;
                      }),
                      shape: WidgetStateProperty.resolveWith<OutlinedBorder>((
                        Set<WidgetState> states,
                      ) {
                        return states.contains(WidgetState.hovered) ? shape0 : shape1;
                      }),
                    ),
                    onPressed: () {},
                    child: const Text('Hover'),
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }
}
