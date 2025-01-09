// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

/// Flutter code sample for [GrowthDirection]s.

void main() => runApp(const ExampleApp());

class ExampleApp extends StatelessWidget {
  const ExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: MyWidget());
  }
}

class MyWidget extends StatefulWidget {
  const MyWidget({super.key});

  @override
  State<MyWidget> createState() => _MyWidgetState();
}

class _MyWidgetState extends State<MyWidget> {
  final List<String> _alphabet = <String>[
    'A',
    'B',
    'C',
    'D',
    'E',
    'F',
    'G',
    'H',
    'I',
    'J',
    'K',
    'L',
    'M',
    'N',
    'O',
    'P',
    'Q',
    'R',
    'S',
    'T',
    'U',
    'V',
    'W',
    'X',
    'Y',
    'Z',
  ];
  final Widget _spacer = const SizedBox.square(dimension: 10);
  final UniqueKey _center = UniqueKey();
  AxisDirection _axisDirection = AxisDirection.down;

  Widget _getArrows(AxisDirection axisDirection) {
    final Widget arrow = switch (axisDirection) {
      AxisDirection.up => const Icon(Icons.arrow_upward_rounded),
      AxisDirection.down => const Icon(Icons.arrow_downward_rounded),
      AxisDirection.left => const Icon(Icons.arrow_back_rounded),
      AxisDirection.right => const Icon(Icons.arrow_forward_rounded),
    };
    return Flex(
      direction: flipAxis(axisDirectionToAxis(axisDirection)),
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[arrow, arrow],
    );
  }

  void _onAxisDirectionChanged(AxisDirection? axisDirection) {
    if (axisDirection != null && axisDirection != _axisDirection) {
      setState(() {
        // Respond to change in axis direction.
        _axisDirection = axisDirection;
      });
    }
  }

  Widget _getLeading(SliverConstraints constraints, bool isForward) {
    return Container(
      color: isForward ? Colors.orange[300] : Colors.green[400],
      padding: const EdgeInsets.all(8.0),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: <Widget>[
          Text(constraints.axis.toString()),
          _spacer,
          Text(constraints.axisDirection.toString()),
          _spacer,
          Text(constraints.growthDirection.toString()),
          _spacer,
          _getArrows(
            isForward
                ? _axisDirection
                // This method is available to conveniently flip an AxisDirection
                // into its opposite direction.
                : flipAxisDirection(_axisDirection),
          ),
        ],
      ),
    );
  }

  Widget _getRadioRow() {
    return DefaultTextStyle(
      style: const TextStyle(fontWeight: FontWeight.bold, color: Colors.white),
      child: RadioTheme(
        data: RadioThemeData(fillColor: WidgetStateProperty.all<Color>(Colors.white)),
        child: Padding(
          padding: const EdgeInsets.all(8.0),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceAround,
            children: <Widget>[
              Radio<AxisDirection>(
                value: AxisDirection.up,
                groupValue: _axisDirection,
                onChanged: _onAxisDirectionChanged,
              ),
              const Text('up'),
              _spacer,
              Radio<AxisDirection>(
                value: AxisDirection.down,
                groupValue: _axisDirection,
                onChanged: _onAxisDirectionChanged,
              ),
              const Text('down'),
              _spacer,
              Radio<AxisDirection>(
                value: AxisDirection.left,
                groupValue: _axisDirection,
                onChanged: _onAxisDirectionChanged,
              ),
              const Text('left'),
              _spacer,
              Radio<AxisDirection>(
                value: AxisDirection.right,
                groupValue: _axisDirection,
                onChanged: _onAxisDirectionChanged,
              ),
              const Text('right'),
              _spacer,
            ],
          ),
        ),
      ),
    );
  }

  Widget _getList({required bool isForward}) {
    // The SliverLayoutBuilder is not necessary, and is here to allow us to see
    // the SliverConstraints & directional information that is provided to the
    // SliverList when laying out.
    return SliverLayoutBuilder(
      builder: (BuildContext context, SliverConstraints constraints) {
        return SliverList.builder(
          itemCount: 27,
          itemBuilder: (BuildContext context, int index) {
            final Widget child;
            if (index == 0) {
              child = _getLeading(constraints, isForward);
            } else {
              child = Container(
                color:
                    isForward
                        ? (index.isEven ? Colors.amber[100] : Colors.amberAccent)
                        : (index.isEven ? Colors.green[100] : Colors.lightGreen),
                padding: const EdgeInsets.all(8.0),
                child: Center(child: Text(_alphabet[index - 1])),
              );
            }
            return Padding(padding: const EdgeInsets.all(8.0), child: child);
          },
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('GrowthDirections'),
        bottom: PreferredSize(
          preferredSize: const Size.fromHeight(50),
          child: Padding(padding: const EdgeInsets.all(8.0), child: _getRadioRow()),
        ),
      ),
      body: CustomScrollView(
        // This method is available to conveniently determine if an scroll
        // view is reversed by its AxisDirection.
        reverse: axisDirectionIsReversed(_axisDirection),
        // This method is available to conveniently convert an AxisDirection
        // into its Axis.
        scrollDirection: axisDirectionToAxis(_axisDirection),
        // Places the leading edge of the center sliver in the middle of the
        // viewport. Changing this value between 0.0 (the default) and 1.0
        // changes the position of the inflection point between GrowthDirections
        // in the viewport when the slivers are laid out.
        anchor: 0.5,
        center: _center,
        slivers: <Widget>[
          _getList(isForward: false),
          SliverToBoxAdapter(
            // This sliver will be located at the anchor. The scroll position
            // will progress in either direction from this point.
            key: _center,
            child: const Padding(
              padding: EdgeInsets.all(8.0),
              child: Center(child: Text('0', style: TextStyle(fontWeight: FontWeight.bold))),
            ),
          ),
          _getList(isForward: true),
        ],
      ),
    );
  }
}
