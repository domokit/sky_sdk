// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';

enum _DragTarget { start, end }

// How close a drag's start position must be to the target point. This is
// a distance squared.
const double _kTargetSlop = 2500.0;

// Used by the Painter classes.
const double _kPointRadius = 6.0;

class _DragHandler extends Drag {
  _DragHandler(this.onUpdate, this.onCancel, this.onEnd);

  final GestureDragUpdateCallback onUpdate;
  final GestureDragCancelCallback onCancel;
  final GestureDragEndCallback onEnd;

  @override
  void update(DragUpdateDetails details) {
    onUpdate(details);
  }

  @override
  void cancel() {
    onCancel();
  }

  @override
  void end(DragEndDetails details) {
    onEnd(details);
  }
}

class _IgnoreDrag extends Drag {}

class _PointDemoPainter extends CustomPainter {
  _PointDemoPainter({Animation<double>? repaint, required this.arc})
    : _repaint = repaint,
      super(repaint: repaint);

  final MaterialPointArcTween arc;
  final Animation<double>? _repaint;

  void drawPoint(Canvas canvas, Offset point, Color color) {
    final Paint paint =
        Paint()
          ..color = color.withOpacity(0.25)
          ..style = PaintingStyle.fill;
    canvas.drawCircle(point, _kPointRadius, paint);
    paint
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;
    canvas.drawCircle(point, _kPointRadius + 1.0, paint);
  }

  @override
  void paint(Canvas canvas, Size size) {
    final Paint paint = Paint();

    if (arc.center != null) {
      drawPoint(canvas, arc.center!, Colors.grey.shade400);
    }

    paint
      ..isAntiAlias =
          false // Work-around for github.com/flutter/flutter/issues/5720
      ..color = Colors.green.withOpacity(0.25)
      ..strokeWidth = 4.0
      ..style = PaintingStyle.stroke;
    if (arc.center != null && arc.radius != null) {
      canvas.drawCircle(arc.center!, arc.radius!, paint);
    } else {
      canvas.drawLine(arc.begin!, arc.end!, paint);
    }

    drawPoint(canvas, arc.begin!, Colors.green);
    drawPoint(canvas, arc.end!, Colors.red);

    paint
      ..color = Colors.green
      ..style = PaintingStyle.fill;
    canvas.drawCircle(arc.lerp(_repaint!.value), _kPointRadius, paint);
  }

  @override
  bool hitTest(Offset position) {
    return (arc.begin! - position).distanceSquared < _kTargetSlop ||
        (arc.end! - position).distanceSquared < _kTargetSlop;
  }

  @override
  bool shouldRepaint(_PointDemoPainter oldPainter) => arc != oldPainter.arc;
}

class _PointDemo extends StatefulWidget {
  const _PointDemo({super.key, required this.controller});

  final AnimationController controller;

  @override
  _PointDemoState createState() => _PointDemoState();
}

class _PointDemoState extends State<_PointDemo> {
  final GlobalKey _painterKey = GlobalKey();

  CurvedAnimation? _animation;
  _DragTarget? _dragTarget;
  Size? _screenSize;
  Offset? _begin;
  Offset? _end;

  @override
  void initState() {
    super.initState();
    _animation = CurvedAnimation(parent: widget.controller, curve: Curves.fastOutSlowIn);
  }

  @override
  void dispose() {
    widget.controller.value = 0.0;
    super.dispose();
  }

  Drag _handleOnStart(Offset position) {
    // TODO(hansmuller): allow the user to drag both points at the same time.
    if (_dragTarget != null) {
      return _IgnoreDrag();
    }

    final RenderBox? box = _painterKey.currentContext!.findRenderObject() as RenderBox?;
    final double startOffset = (box!.localToGlobal(_begin!) - position).distanceSquared;
    final double endOffset = (box.localToGlobal(_end!) - position).distanceSquared;
    setState(() {
      if (startOffset < endOffset && startOffset < _kTargetSlop) {
        _dragTarget = _DragTarget.start;
      } else if (endOffset < _kTargetSlop) {
        _dragTarget = _DragTarget.end;
      } else {
        _dragTarget = null;
      }
    });

    return _DragHandler(_handleDragUpdate, _handleDragCancel, _handleDragEnd);
  }

  void _handleDragUpdate(DragUpdateDetails details) {
    switch (_dragTarget!) {
      case _DragTarget.start:
        setState(() {
          _begin = _begin! + details.delta;
        });
      case _DragTarget.end:
        setState(() {
          _end = _end! + details.delta;
        });
    }
  }

  void _handleDragCancel() {
    _dragTarget = null;
    widget.controller.value = 0.0;
  }

  void _handleDragEnd(DragEndDetails details) {
    _dragTarget = null;
  }

  @override
  Widget build(BuildContext context) {
    final Size screenSize = MediaQuery.of(context).size;
    if (_screenSize == null || _screenSize != screenSize) {
      _screenSize = screenSize;
      _begin = Offset(screenSize.width * 0.5, screenSize.height * 0.2);
      _end = Offset(screenSize.width * 0.1, screenSize.height * 0.4);
    }

    final MaterialPointArcTween arc = MaterialPointArcTween(begin: _begin, end: _end);
    return RawGestureDetector(
      behavior: _dragTarget == null ? HitTestBehavior.deferToChild : HitTestBehavior.opaque,
      gestures: <Type, GestureRecognizerFactory>{
        ImmediateMultiDragGestureRecognizer:
            GestureRecognizerFactoryWithHandlers<ImmediateMultiDragGestureRecognizer>(
              () => ImmediateMultiDragGestureRecognizer(),
              (ImmediateMultiDragGestureRecognizer instance) {
                instance.onStart = _handleOnStart;
              },
            ),
      },
      child: ClipRect(
        child: CustomPaint(
          key: _painterKey,
          foregroundPainter: _PointDemoPainter(repaint: _animation, arc: arc),
          // Watch out: if this IgnorePointer is left out, then gestures that
          // fail _PointDemoPainter.hitTest() will still be recognized because
          // they do overlap this child, which is as big as the CustomPaint.
          child: IgnorePointer(
            child: Padding(
              padding: const EdgeInsets.all(16.0),
              child: Text(
                'Tap the refresh button to run the animation. Drag the green '
                "and red points to change the animation's path.",
                style: Theme.of(context).textTheme.bodySmall?.copyWith(fontSize: 16.0),
              ),
            ),
          ),
        ),
      ),
    );
  }
}

class _RectangleDemoPainter extends CustomPainter {
  _RectangleDemoPainter({required Animation<double> repaint, required this.arc})
    : _repaint = repaint,
      super(repaint: repaint);

  final MaterialRectArcTween arc;
  final Animation<double> _repaint;

  void drawPoint(Canvas canvas, Offset p, Color color) {
    final Paint paint =
        Paint()
          ..color = color.withOpacity(0.25)
          ..style = PaintingStyle.fill;
    canvas.drawCircle(p, _kPointRadius, paint);
    paint
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;
    canvas.drawCircle(p, _kPointRadius + 1.0, paint);
  }

  void drawRect(Canvas canvas, Rect rect, Color color) {
    final Paint paint =
        Paint()
          ..color = color.withOpacity(0.25)
          ..strokeWidth = 4.0
          ..style = PaintingStyle.stroke;
    canvas.drawRect(rect, paint);
    drawPoint(canvas, rect.center, color);
  }

  @override
  void paint(Canvas canvas, Size size) {
    drawRect(canvas, arc.begin!, Colors.green);
    drawRect(canvas, arc.end!, Colors.red);
    drawRect(canvas, arc.lerp(_repaint.value), Colors.blue);
  }

  @override
  bool hitTest(Offset position) {
    return (arc.begin!.center - position).distanceSquared < _kTargetSlop ||
        (arc.end!.center - position).distanceSquared < _kTargetSlop;
  }

  @override
  bool shouldRepaint(_RectangleDemoPainter oldPainter) => arc != oldPainter.arc;
}

class _RectangleDemo extends StatefulWidget {
  const _RectangleDemo({super.key, required this.controller});

  final AnimationController controller;

  @override
  _RectangleDemoState createState() => _RectangleDemoState();
}

class _RectangleDemoState extends State<_RectangleDemo> {
  final GlobalKey _painterKey = GlobalKey();

  late final CurvedAnimation _animation = CurvedAnimation(
    parent: widget.controller,
    curve: Curves.fastOutSlowIn,
  );
  _DragTarget? _dragTarget;
  Size? _screenSize;
  Rect? _begin;
  Rect? _end;

  @override
  void dispose() {
    widget.controller.value = 0.0;
    super.dispose();
  }

  Drag _handleOnStart(Offset position) {
    // TODO(hansmuller): allow the user to drag both points at the same time.
    if (_dragTarget != null) {
      return _IgnoreDrag();
    }

    final RenderBox? box = _painterKey.currentContext?.findRenderObject() as RenderBox?;
    final double startOffset = (box!.localToGlobal(_begin!.center) - position).distanceSquared;
    final double endOffset = (box.localToGlobal(_end!.center) - position).distanceSquared;
    setState(() {
      if (startOffset < endOffset && startOffset < _kTargetSlop) {
        _dragTarget = _DragTarget.start;
      } else if (endOffset < _kTargetSlop) {
        _dragTarget = _DragTarget.end;
      } else {
        _dragTarget = null;
      }
    });
    return _DragHandler(_handleDragUpdate, _handleDragCancel, _handleDragEnd);
  }

  void _handleDragUpdate(DragUpdateDetails details) {
    switch (_dragTarget!) {
      case _DragTarget.start:
        setState(() {
          _begin = _begin?.shift(details.delta);
        });
      case _DragTarget.end:
        setState(() {
          _end = _end?.shift(details.delta);
        });
    }
  }

  void _handleDragCancel() {
    _dragTarget = null;
    widget.controller.value = 0.0;
  }

  void _handleDragEnd(DragEndDetails details) {
    _dragTarget = null;
  }

  @override
  Widget build(BuildContext context) {
    final Size screenSize = MediaQuery.of(context).size;
    if (_screenSize == null || _screenSize != screenSize) {
      _screenSize = screenSize;
      _begin = Rect.fromLTWH(
        screenSize.width * 0.5,
        screenSize.height * 0.2,
        screenSize.width * 0.4,
        screenSize.height * 0.2,
      );
      _end = Rect.fromLTWH(
        screenSize.width * 0.1,
        screenSize.height * 0.4,
        screenSize.width * 0.3,
        screenSize.height * 0.3,
      );
    }

    final MaterialRectArcTween arc = MaterialRectArcTween(begin: _begin, end: _end);
    return RawGestureDetector(
      behavior: _dragTarget == null ? HitTestBehavior.deferToChild : HitTestBehavior.opaque,
      gestures: <Type, GestureRecognizerFactory>{
        ImmediateMultiDragGestureRecognizer:
            GestureRecognizerFactoryWithHandlers<ImmediateMultiDragGestureRecognizer>(
              () => ImmediateMultiDragGestureRecognizer(),
              (ImmediateMultiDragGestureRecognizer instance) {
                instance.onStart = _handleOnStart;
              },
            ),
      },
      child: ClipRect(
        child: CustomPaint(
          key: _painterKey,
          foregroundPainter: _RectangleDemoPainter(repaint: _animation, arc: arc),
          // Watch out: if this IgnorePointer is left out, then gestures that
          // fail _RectDemoPainter.hitTest() will still be recognized because
          // they do overlap this child, which is as big as the CustomPaint.
          child: IgnorePointer(
            child: Padding(
              padding: const EdgeInsets.all(16.0),
              child: Text(
                'Tap the refresh button to run the animation. Drag the rectangles '
                "to change the animation's path.",
                style: Theme.of(context).textTheme.bodySmall!.copyWith(fontSize: 16.0),
              ),
            ),
          ),
        ),
      ),
    );
  }
}

typedef _DemoBuilder = Widget Function(_ArcDemo demo);

class _ArcDemo {
  _ArcDemo(this.title, this.builder, TickerProvider vsync)
    : controller = AnimationController(duration: const Duration(milliseconds: 500), vsync: vsync),
      key = GlobalKey(debugLabel: title);

  final String title;
  final _DemoBuilder builder;
  final AnimationController controller;
  final GlobalKey key;
}

class AnimationDemo extends StatefulWidget {
  const AnimationDemo({super.key});

  @override
  State<AnimationDemo> createState() => _AnimationDemoState();
}

class _AnimationDemoState extends State<AnimationDemo> with TickerProviderStateMixin {
  late final List<_ArcDemo> _allDemos = <_ArcDemo>[
    _ArcDemo('POINT', (_ArcDemo demo) {
      return _PointDemo(key: demo.key, controller: demo.controller);
    }, this),
    _ArcDemo('RECTANGLE', (_ArcDemo demo) {
      return _RectangleDemo(key: demo.key, controller: demo.controller);
    }, this),
  ];

  Future<void> _play(_ArcDemo demo) async {
    await demo.controller.forward();
    if (demo.key.currentState != null && demo.key.currentState!.mounted) {
      demo.controller.reverse();
    }
  }

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: _allDemos.length,
      child: Scaffold(
        appBar: AppBar(
          title: const Text('Animation'),
          bottom: TabBar(
            tabs: _allDemos.map<Tab>((_ArcDemo demo) => Tab(text: demo.title)).toList(),
          ),
        ),
        floatingActionButton: Builder(
          builder: (BuildContext context) {
            return FloatingActionButton(
              child: const Icon(Icons.refresh),
              onPressed: () {
                _play(_allDemos[DefaultTabController.of(context).index]);
              },
            );
          },
        ),
        body: TabBarView(
          children: _allDemos.map<Widget>((_ArcDemo demo) => demo.builder(demo)).toList(),
        ),
      ),
    );
  }
}

void main() {
  runApp(const MaterialApp(home: AnimationDemo()));
}
