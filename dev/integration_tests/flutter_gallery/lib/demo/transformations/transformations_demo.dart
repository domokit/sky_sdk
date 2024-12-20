// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' show Vertices;
import 'package:flutter/material.dart';
import 'transformations_demo_board.dart';
import 'transformations_demo_edit_board_point.dart';
import 'transformations_demo_gesture_transformable.dart';

class TransformationsDemo extends StatefulWidget {
  const TransformationsDemo({super.key});

  static const String routeName = '/transformations';

  @override
  State<TransformationsDemo> createState() => _TransformationsDemoState();
}

class _TransformationsDemoState extends State<TransformationsDemo> {
  // The radius of a hexagon tile in pixels.
  static const double _kHexagonRadius = 32.0;
  // The margin between hexagons.
  static const double _kHexagonMargin = 1.0;
  // The radius of the entire board in hexagons, not including the center.
  static const int _kBoardRadius = 8;

  bool _reset = false;
  Board _board = Board(
    boardRadius: _kBoardRadius,
    hexagonRadius: _kHexagonRadius,
    hexagonMargin: _kHexagonMargin,
  );

  @override
  Widget build(BuildContext context) {
    final BoardPainter painter = BoardPainter(board: _board);

    // The scene is drawn by a CustomPaint, but user interaction is handled by
    // the GestureTransformable parent widget.
    return Scaffold(
      appBar: AppBar(
        title: const Text('2D Transformations'),
        actions: <Widget>[
          IconButton(
            icon: const Icon(Icons.help),
            tooltip: 'Help',
            onPressed: () {
              showDialog<Column>(
                context: context,
                builder: (BuildContext context) => instructionDialog,
              );
            },
          ),
        ],
      ),
      body: LayoutBuilder(
        builder: (BuildContext context, BoxConstraints constraints) {
          // Draw the scene as big as is available, but allow the user to
          // translate beyond that to a visibleSize that's a bit bigger.
          final Size size = Size(constraints.maxWidth, constraints.maxHeight);
          final Size visibleSize = Size(size.width * 3, size.height * 2);
          return GestureTransformable(
            reset: _reset,
            onResetEnd: () {
              setState(() {
                _reset = false;
              });
            },
            boundaryRect: Rect.fromLTWH(
              -visibleSize.width / 2,
              -visibleSize.height / 2,
              visibleSize.width,
              visibleSize.height,
            ),
            // Center the board in the middle of the screen. It's drawn centered
            // at the origin, which is the top left corner of the
            // GestureTransformable.
            initialTranslation: Offset(size.width / 2, size.height / 2),
            onTapUp: _onTapUp,
            size: size,
            child: CustomPaint(painter: painter),
          );
        },
      ),
      floatingActionButton: _board.selected == null ? resetButton : editButton,
    );
  }

  Widget get instructionDialog {
    return AlertDialog(
      title: const Text('2D Transformations'),
      content: const Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Text('Tap to edit hex tiles, and use gestures to move around the scene:\n'),
          Text('- Drag to pan.'),
          Text('- Pinch to zoom.'),
          Text('- Rotate with two fingers.'),
          Text('\nYou can always press the home button to return to the starting orientation!'),
        ],
      ),
      actions: <Widget>[
        TextButton(
          child: const Text('OK'),
          onPressed: () {
            Navigator.of(context).pop();
          },
        ),
      ],
    );
  }

  FloatingActionButton get resetButton {
    return FloatingActionButton(
      onPressed: () {
        setState(() {
          _reset = true;
        });
      },
      tooltip: 'Reset Transform',
      backgroundColor: Theme.of(context).primaryColor,
      child: const Icon(Icons.home),
    );
  }

  FloatingActionButton get editButton {
    return FloatingActionButton(
      onPressed: () {
        if (_board.selected == null) {
          return;
        }
        showModalBottomSheet<Widget>(
          context: context,
          builder: (BuildContext context) {
            return Container(
              width: double.infinity,
              height: 150,
              padding: const EdgeInsets.all(12.0),
              child: EditBoardPoint(
                boardPoint: _board.selected!,
                onColorSelection: (Color color) {
                  setState(() {
                    _board = _board.copyWithBoardPointColor(_board.selected!, color);
                    Navigator.pop(context);
                  });
                },
              ),
            );
          },
        );
      },
      tooltip: 'Edit Tile',
      child: const Icon(Icons.edit),
    );
  }

  void _onTapUp(TapUpDetails details) {
    final Offset scenePoint = details.globalPosition;
    final BoardPoint? boardPoint = _board.pointToBoardPoint(scenePoint);
    setState(() {
      _board = _board.copyWithSelected(boardPoint);
    });
  }
}

// CustomPainter is what is passed to CustomPaint and actually draws the scene
// when its `paint` method is called.
class BoardPainter extends CustomPainter {
  const BoardPainter({this.board});

  final Board? board;

  @override
  void paint(Canvas canvas, Size size) {
    void drawBoardPoint(BoardPoint? boardPoint) {
      final Color color = boardPoint!.color.withOpacity(board!.selected == boardPoint ? 0.2 : 1.0);
      final Vertices vertices = board!.getVerticesForBoardPoint(boardPoint, color);
      canvas.drawVertices(vertices, BlendMode.color, Paint());
      vertices.dispose();
    }

    board!.forEach(drawBoardPoint);
  }

  // We should repaint whenever the board changes, such as board.selected.
  @override
  bool shouldRepaint(BoardPainter oldDelegate) {
    return oldDelegate.board != board;
  }
}
