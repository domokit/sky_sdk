// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

/// Flutter code sample for [SelectionContainer].

void main() => runApp(const SelectionContainerExampleApp());

class SelectionContainerExampleApp extends StatelessWidget {
  const SelectionContainerExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: SelectionArea(
        child: Scaffold(
          appBar: AppBar(title: const Text('SelectionContainer Sample')),
          body: const Center(
            child: SelectionAllOrNoneContainer(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[Text('Row 1'), Text('Row 2'), Text('Row 3')],
              ),
            ),
          ),
        ),
      ),
    );
  }
}

class SelectionAllOrNoneContainer extends StatefulWidget {
  const SelectionAllOrNoneContainer({super.key, required this.child});

  final Widget child;

  @override
  State<StatefulWidget> createState() => _SelectionAllOrNoneContainerState();
}

class _SelectionAllOrNoneContainerState extends State<SelectionAllOrNoneContainer> {
  final SelectAllOrNoneContainerDelegate delegate = SelectAllOrNoneContainerDelegate();

  @override
  void dispose() {
    delegate.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return SelectionContainer(delegate: delegate, child: widget.child);
  }
}

class SelectAllOrNoneContainerDelegate extends MultiSelectableSelectionContainerDelegate {
  Offset? _adjustedStartEdge;
  Offset? _adjustedEndEdge;
  bool _isSelected = false;

  // This method is called when newly added selectable is in the current
  // selected range.
  @override
  void ensureChildUpdated(Selectable selectable) {
    if (_isSelected) {
      dispatchSelectionEventToChild(selectable, const SelectAllSelectionEvent());
    }
  }

  @override
  SelectionResult handleSelectWord(SelectWordSelectionEvent event) {
    // Treat select word as select all.
    return handleSelectAll(const SelectAllSelectionEvent());
  }

  @override
  SelectionResult handleSelectionEdgeUpdate(SelectionEdgeUpdateEvent event) {
    final Rect containerRect = Rect.fromLTWH(0, 0, containerSize.width, containerSize.height);
    final Matrix4 globalToLocal = getTransformTo(null)..invert();
    final Offset localOffset = MatrixUtils.transformPoint(globalToLocal, event.globalPosition);
    final Offset adjustOffset = SelectionUtils.adjustDragOffset(containerRect, localOffset);
    if (event.type == SelectionEventType.startEdgeUpdate) {
      _adjustedStartEdge = adjustOffset;
    } else {
      _adjustedEndEdge = adjustOffset;
    }
    // Select all content if the selection rect intercepts with the rect.
    if (_adjustedStartEdge != null && _adjustedEndEdge != null) {
      final Rect selectionRect = Rect.fromPoints(_adjustedStartEdge!, _adjustedEndEdge!);
      if (!selectionRect.intersect(containerRect).isEmpty) {
        handleSelectAll(const SelectAllSelectionEvent());
      } else {
        super.handleClearSelection(const ClearSelectionEvent());
      }
    } else {
      super.handleClearSelection(const ClearSelectionEvent());
    }
    return SelectionUtils.getResultBasedOnRect(containerRect, localOffset);
  }

  @override
  SelectionResult handleClearSelection(ClearSelectionEvent event) {
    _adjustedStartEdge = null;
    _adjustedEndEdge = null;
    _isSelected = false;
    return super.handleClearSelection(event);
  }

  @override
  SelectionResult handleSelectAll(SelectAllSelectionEvent event) {
    _isSelected = true;
    return super.handleSelectAll(event);
  }
}
