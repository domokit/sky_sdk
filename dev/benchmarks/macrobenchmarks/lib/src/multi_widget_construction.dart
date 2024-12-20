// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class MultiWidgetConstructTable extends StatefulWidget {
  const MultiWidgetConstructTable(this.columnCount, this.rowCount, {super.key});

  final int columnCount;
  final int rowCount;

  @override
  State<MultiWidgetConstructTable> createState() => _MultiWidgetConstructTableState();
}

class _MultiWidgetConstructTableState extends State<MultiWidgetConstructTable>
    with SingleTickerProviderStateMixin {
  static const List<MaterialColor> colorList = <MaterialColor>[
    Colors.pink,
    Colors.red,
    Colors.deepOrange,
    Colors.orange,
    Colors.amber,
    Colors.yellow,
    Colors.lime,
    Colors.lightGreen,
    Colors.green,
    Colors.teal,
    Colors.cyan,
    Colors.lightBlue,
    Colors.blue,
    Colors.indigo,
    Colors.purple,
  ];
  int counter = 0;

  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 10000),
      upperBound: colorList.length + 1.0,
    )..repeat();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: _controller,
      builder: (BuildContext context, _) {
        final int totalLength = widget.rowCount * widget.columnCount;
        final int widgetCounter = counter * totalLength;
        final double height = MediaQuery.of(context).size.height / widget.rowCount;
        final double colorPosition = _controller.value;
        final int c1Position = colorPosition.floor();
        final Color c1 = colorList[c1Position % colorList.length][900]!;
        final Color c2 = colorList[(c1Position + 1) % colorList.length][900]!;
        final Color baseColor = Color.lerp(c1, c2, colorPosition - c1Position)!;
        counter++;
        return Scaffold(
          body: Table(
            children: List<TableRow>.generate(
              widget.rowCount,
              (int row) => TableRow(
                children: List<Widget>.generate(widget.columnCount, (int column) {
                  final int label = row * widget.columnCount + column;
                  // This implementation rebuild the widget tree for every
                  // frame, and is intentionally designed of poor performance
                  // for benchmark purposes.
                  return counter.isEven
                      ? Container(
                        // This key forces rebuilding the element
                        key: ValueKey<int>(widgetCounter + label),
                        color: Color.lerp(Colors.white, baseColor, label / totalLength),
                        constraints: BoxConstraints.expand(height: height),
                        child: Text('${widgetCounter + label}'),
                      )
                      : MyContainer(
                        // This key forces rebuilding the element
                        key: ValueKey<int>(widgetCounter + label),
                        color: Color.lerp(Colors.white, baseColor, label / totalLength)!,
                        constraints: BoxConstraints.expand(height: height),
                        child: Text('${widgetCounter + label}'),
                      );
                }),
              ),
            ),
          ),
        );
      },
    );
  }
}

// This class is intended to break the original Widget tree
class MyContainer extends StatelessWidget {
  const MyContainer({
    required this.color,
    required this.child,
    required this.constraints,
    super.key,
  });
  final Color color;
  final Widget child;
  final BoxConstraints constraints;

  @override
  Widget build(BuildContext context) {
    return Container(color: color, constraints: constraints, child: child);
  }
}
