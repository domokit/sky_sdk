// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

/// Flutter code sample for [SelectableRegion].

void main() => runApp(const SelectableRegionExampleApp());

class SelectableRegionExampleApp extends StatelessWidget {
  const SelectableRegionExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: SelectableRegion(
        selectionControls: materialTextSelectionControls,
        child: Scaffold(
          appBar: AppBar(title: const Text('SelectableRegion Sample')),
          body: const Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                Text('Select this icon', style: TextStyle(fontSize: 30)),
                SizedBox(height: 10),
                MySelectableAdapter(child: Icon(Icons.key, size: 30)),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class MySelectableAdapter extends StatelessWidget {
  const MySelectableAdapter({super.key, required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) {
    final SelectionRegistrar? registrar = SelectionContainer.maybeOf(context);
    if (registrar == null) {
      return child;
    }
    return MouseRegion(
      cursor: SystemMouseCursors.text,
      child: _SelectableAdapter(registrar: registrar, child: child),
    );
  }
}

class _SelectableAdapter extends SingleChildRenderObjectWidget {
  const _SelectableAdapter({required this.registrar, required Widget child}) : super(child: child);

  final SelectionRegistrar registrar;

  @override
  _RenderSelectableAdapter createRenderObject(BuildContext context) {
    return _RenderSelectableAdapter(DefaultSelectionStyle.of(context).selectionColor!, registrar);
  }

  @override
  void updateRenderObject(BuildContext context, _RenderSelectableAdapter renderObject) {
    renderObject
      ..selectionColor = DefaultSelectionStyle.of(context).selectionColor!
      ..registrar = registrar;
  }
}

class _RenderSelectableAdapter extends RenderProxyBox with Selectable, SelectionRegistrant {
  _RenderSelectableAdapter(Color selectionColor, SelectionRegistrar registrar)
    : _selectionColor = selectionColor,
      _geometry = ValueNotifier<SelectionGeometry>(_noSelection) {
    this.registrar = registrar;
    _geometry.addListener(markNeedsPaint);
  }

  static const SelectionGeometry _noSelection = SelectionGeometry(
    status: SelectionStatus.none,
    hasContent: true,
  );
  final ValueNotifier<SelectionGeometry> _geometry;

  Color get selectionColor => _selectionColor;
  Color _selectionColor;
  set selectionColor(Color value) {
    if (_selectionColor == value) {
      return;
    }
    _selectionColor = value;
    markNeedsPaint();
  }

  // ValueListenable APIs

  @override
  void addListener(VoidCallback listener) => _geometry.addListener(listener);

  @override
  void removeListener(VoidCallback listener) => _geometry.removeListener(listener);

  @override
  SelectionGeometry get value => _geometry.value;

  // Selectable APIs.

  @override
  List<Rect> get boundingBoxes => <Rect>[paintBounds];

  // Adjust this value to enlarge or shrink the selection highlight.
  static const double _padding = 10.0;
  Rect _getSelectionHighlightRect() {
    return Rect.fromLTWH(
      0 - _padding,
      0 - _padding,
      size.width + _padding * 2,
      size.height + _padding * 2,
    );
  }

  Offset? _start;
  Offset? _end;
  void _updateGeometry() {
    if (_start == null || _end == null) {
      _geometry.value = _noSelection;
      return;
    }
    final Rect renderObjectRect = Rect.fromLTWH(0, 0, size.width, size.height);
    final Rect selectionRect = Rect.fromPoints(_start!, _end!);
    if (renderObjectRect.intersect(selectionRect).isEmpty) {
      _geometry.value = _noSelection;
    } else {
      final Rect selectionRect = _getSelectionHighlightRect();
      final SelectionPoint firstSelectionPoint = SelectionPoint(
        localPosition: selectionRect.bottomLeft,
        lineHeight: selectionRect.size.height,
        handleType: TextSelectionHandleType.left,
      );
      final SelectionPoint secondSelectionPoint = SelectionPoint(
        localPosition: selectionRect.bottomRight,
        lineHeight: selectionRect.size.height,
        handleType: TextSelectionHandleType.right,
      );
      final bool isReversed;
      if (_start!.dy > _end!.dy) {
        isReversed = true;
      } else if (_start!.dy < _end!.dy) {
        isReversed = false;
      } else {
        isReversed = _start!.dx > _end!.dx;
      }
      _geometry.value = SelectionGeometry(
        status: SelectionStatus.uncollapsed,
        hasContent: true,
        startSelectionPoint: isReversed ? secondSelectionPoint : firstSelectionPoint,
        endSelectionPoint: isReversed ? firstSelectionPoint : secondSelectionPoint,
        selectionRects: <Rect>[selectionRect],
      );
    }
  }

  @override
  SelectionResult dispatchSelectionEvent(SelectionEvent event) {
    SelectionResult result = SelectionResult.none;
    switch (event.type) {
      case SelectionEventType.startEdgeUpdate:
      case SelectionEventType.endEdgeUpdate:
        final Rect renderObjectRect = Rect.fromLTWH(0, 0, size.width, size.height);
        // Normalize offset in case it is out side of the rect.
        final Offset point = globalToLocal((event as SelectionEdgeUpdateEvent).globalPosition);
        final Offset adjustedPoint = SelectionUtils.adjustDragOffset(renderObjectRect, point);
        if (event.type == SelectionEventType.startEdgeUpdate) {
          _start = adjustedPoint;
        } else {
          _end = adjustedPoint;
        }
        result = SelectionUtils.getResultBasedOnRect(renderObjectRect, point);
      case SelectionEventType.clear:
        _start = _end = null;
      case SelectionEventType.selectAll:
      case SelectionEventType.selectWord:
      case SelectionEventType.selectParagraph:
        _start = Offset.zero;
        _end = Offset.infinite;
      case SelectionEventType.granularlyExtendSelection:
        result = SelectionResult.end;
        final GranularlyExtendSelectionEvent extendSelectionEvent =
            event as GranularlyExtendSelectionEvent;
        // Initialize the offset it there is no ongoing selection.
        if (_start == null || _end == null) {
          if (extendSelectionEvent.forward) {
            _start = _end = Offset.zero;
          } else {
            _start = _end = Offset.infinite;
          }
        }
        // Move the corresponding selection edge.
        final Offset newOffset = extendSelectionEvent.forward ? Offset.infinite : Offset.zero;
        if (extendSelectionEvent.isEnd) {
          if (newOffset == _end) {
            result = extendSelectionEvent.forward ? SelectionResult.next : SelectionResult.previous;
          }
          _end = newOffset;
        } else {
          if (newOffset == _start) {
            result = extendSelectionEvent.forward ? SelectionResult.next : SelectionResult.previous;
          }
          _start = newOffset;
        }
      case SelectionEventType.directionallyExtendSelection:
        result = SelectionResult.end;
        final DirectionallyExtendSelectionEvent extendSelectionEvent =
            event as DirectionallyExtendSelectionEvent;
        // Convert to local coordinates.
        final double horizontalBaseLine = globalToLocal(Offset(event.dx, 0)).dx;
        final Offset newOffset;
        final bool forward;
        switch (extendSelectionEvent.direction) {
          case SelectionExtendDirection.backward:
          case SelectionExtendDirection.previousLine:
            forward = false;
            // Initialize the offset it there is no ongoing selection.
            if (_start == null || _end == null) {
              _start = _end = Offset.infinite;
            }
            // Move the corresponding selection edge.
            if (extendSelectionEvent.direction == SelectionExtendDirection.previousLine ||
                horizontalBaseLine < 0) {
              newOffset = Offset.zero;
            } else {
              newOffset = Offset.infinite;
            }
          case SelectionExtendDirection.nextLine:
          case SelectionExtendDirection.forward:
            forward = true;
            // Initialize the offset it there is no ongoing selection.
            if (_start == null || _end == null) {
              _start = _end = Offset.zero;
            }
            // Move the corresponding selection edge.
            if (extendSelectionEvent.direction == SelectionExtendDirection.nextLine ||
                horizontalBaseLine > size.width) {
              newOffset = Offset.infinite;
            } else {
              newOffset = Offset.zero;
            }
        }
        if (extendSelectionEvent.isEnd) {
          if (newOffset == _end) {
            result = forward ? SelectionResult.next : SelectionResult.previous;
          }
          _end = newOffset;
        } else {
          if (newOffset == _start) {
            result = forward ? SelectionResult.next : SelectionResult.previous;
          }
          _start = newOffset;
        }
    }
    _updateGeometry();
    return result;
  }

  // This method is called when users want to copy selected content in this
  // widget into clipboard.
  @override
  SelectedContent? getSelectedContent() {
    return value.hasSelection ? const SelectedContent(plainText: 'Custom Text') : null;
  }

  @override
  SelectedContentRange? getSelection() {
    if (!value.hasSelection) {
      return null;
    }
    return const SelectedContentRange(startOffset: 0, endOffset: 1);
  }

  @override
  int get contentLength => 1;

  LayerLink? _startHandle;
  LayerLink? _endHandle;

  @override
  void pushHandleLayers(LayerLink? startHandle, LayerLink? endHandle) {
    if (_startHandle == startHandle && _endHandle == endHandle) {
      return;
    }
    _startHandle = startHandle;
    _endHandle = endHandle;
    markNeedsPaint();
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    super.paint(context, offset);
    if (!_geometry.value.hasSelection) {
      return;
    }
    // Draw the selection highlight.
    final Paint selectionPaint =
        Paint()
          ..style = PaintingStyle.fill
          ..color = _selectionColor;
    context.canvas.drawRect(_getSelectionHighlightRect().shift(offset), selectionPaint);

    // Push the layer links if any.
    if (_startHandle != null) {
      context.pushLayer(
        LeaderLayer(link: _startHandle!, offset: offset + value.startSelectionPoint!.localPosition),
        (PaintingContext context, Offset offset) {},
        Offset.zero,
      );
    }
    if (_endHandle != null) {
      context.pushLayer(
        LeaderLayer(link: _endHandle!, offset: offset + value.endSelectionPoint!.localPosition),
        (PaintingContext context, Offset offset) {},
        Offset.zero,
      );
    }
  }

  @override
  void dispose() {
    _geometry.dispose();
    _startHandle = null;
    _endHandle = null;
    super.dispose();
  }
}
