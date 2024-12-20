// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

import 'transformations_demo_board.dart';
import 'transformations_demo_edit_board_point.dart';

// BEGIN transformationsDemo#1

class TransformationsDemo extends StatefulWidget {
  const TransformationsDemo({super.key});

  @override
  State<TransformationsDemo> createState() => _TransformationsDemoState();
}

class _TransformationsDemoState extends State<TransformationsDemo> with TickerProviderStateMixin {
  final GlobalKey _targetKey = GlobalKey();
  // The radius of a hexagon tile in pixels.
  static const double _kHexagonRadius = 16.0;
  // The margin between hexagons.
  static const double _kHexagonMargin = 1.0;
  // The radius of the entire board in hexagons, not including the center.
  static const int _kBoardRadius = 8;

  Board _board = Board(
    boardRadius: _kBoardRadius,
    hexagonRadius: _kHexagonRadius,
    hexagonMargin: _kHexagonMargin,
  );

  final TransformationController _transformationController = TransformationController();
  Animation<Matrix4>? _animationReset;
  late AnimationController _controllerReset;
  Matrix4? _homeMatrix;

  // Handle reset to home transform animation.
  void _onAnimateReset() {
    _transformationController.value = _animationReset!.value;
    if (!_controllerReset.isAnimating) {
      _animationReset?.removeListener(_onAnimateReset);
      _animationReset = null;
      _controllerReset.reset();
    }
  }

  // Initialize the reset to home transform animation.
  void _animateResetInitialize() {
    _controllerReset.reset();
    _animationReset = Matrix4Tween(
      begin: _transformationController.value,
      end: _homeMatrix,
    ).animate(_controllerReset);
    _controllerReset.duration = const Duration(milliseconds: 400);
    _animationReset!.addListener(_onAnimateReset);
    _controllerReset.forward();
  }

  // Stop a running reset to home transform animation.
  void _animateResetStop() {
    _controllerReset.stop();
    _animationReset?.removeListener(_onAnimateReset);
    _animationReset = null;
    _controllerReset.reset();
  }

  void _onScaleStart(ScaleStartDetails details) {
    // If the user tries to cause a transformation while the reset animation is
    // running, cancel the reset animation.
    if (_controllerReset.status == AnimationStatus.forward) {
      _animateResetStop();
    }
  }

  void _onTapUp(TapUpDetails details) {
    final RenderBox renderBox = _targetKey.currentContext!.findRenderObject()! as RenderBox;
    final Offset offset = details.globalPosition - renderBox.localToGlobal(Offset.zero);
    final Offset scenePoint = _transformationController.toScene(offset);
    final BoardPoint? boardPoint = _board.pointToBoardPoint(scenePoint);
    setState(() {
      _board = _board.copyWithSelected(boardPoint);
    });
  }

  @override
  void initState() {
    super.initState();
    _controllerReset = AnimationController(vsync: this);
  }

  @override
  Widget build(BuildContext context) {
    // The scene is drawn by a CustomPaint, but user interaction is handled by
    // the InteractiveViewer parent widget.
    return Scaffold(
      backgroundColor: Theme.of(context).colorScheme.primary,
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Text(GalleryLocalizations.of(context)!.demo2dTransformationsTitle),
      ),
      body: ColoredBox(
        color: backgroundColor,
        child: LayoutBuilder(
          builder: (BuildContext context, BoxConstraints constraints) {
            // Draw the scene as big as is available, but allow the user to
            // translate beyond that to a visibleSize that's a bit bigger.
            final Size viewportSize = Size(constraints.maxWidth, constraints.maxHeight);

            // Start the first render, start the scene centered in the viewport.
            if (_homeMatrix == null) {
              _homeMatrix =
                  Matrix4.identity()..translate(
                    viewportSize.width / 2 - _board.size.width / 2,
                    viewportSize.height / 2 - _board.size.height / 2,
                  );
              _transformationController.value = _homeMatrix!;
            }

            return ClipRect(
              child: MouseRegion(
                cursor: SystemMouseCursors.click,
                child: GestureDetector(
                  behavior: HitTestBehavior.opaque,
                  onTapUp: _onTapUp,
                  child: InteractiveViewer(
                    key: _targetKey,
                    transformationController: _transformationController,
                    boundaryMargin: EdgeInsets.symmetric(
                      horizontal: viewportSize.width,
                      vertical: viewportSize.height,
                    ),
                    minScale: 0.01,
                    onInteractionStart: _onScaleStart,
                    child: SizedBox.expand(
                      child: CustomPaint(size: _board.size, painter: _BoardPainter(board: _board)),
                    ),
                  ),
                ),
              ),
            );
          },
        ),
      ),
      persistentFooterButtons: <Widget>[resetButton, editButton],
    );
  }

  IconButton get resetButton {
    return IconButton(
      onPressed: () {
        setState(() {
          _animateResetInitialize();
        });
      },
      tooltip: 'Reset',
      color: Theme.of(context).colorScheme.surface,
      icon: const Icon(Icons.replay),
    );
  }

  IconButton get editButton {
    return IconButton(
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
              padding: const EdgeInsets.all(12),
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
      tooltip: 'Edit',
      color: Theme.of(context).colorScheme.surface,
      icon: const Icon(Icons.edit),
    );
  }

  @override
  void dispose() {
    _controllerReset.dispose();
    super.dispose();
  }
}

// CustomPainter is what is passed to CustomPaint and actually draws the scene
// when its `paint` method is called.
class _BoardPainter extends CustomPainter {
  const _BoardPainter({required this.board});

  final Board board;

  @override
  void paint(Canvas canvas, Size size) {
    void drawBoardPoint(BoardPoint? boardPoint) {
      final Color color = boardPoint!.color.withOpacity(board.selected == boardPoint ? 0.7 : 1);
      final Vertices vertices = board.getVerticesForBoardPoint(boardPoint, color);
      canvas.drawVertices(vertices, BlendMode.color, Paint());
    }

    board.forEach(drawBoardPoint);
  }

  // We should repaint whenever the board changes, such as board.selected.
  @override
  bool shouldRepaint(_BoardPainter oldDelegate) {
    return oldDelegate.board != board;
  }
}

// END
