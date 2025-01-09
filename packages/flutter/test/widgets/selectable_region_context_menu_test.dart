// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('browser') // This file contains web-only library.
library;

import 'dart:js_interop';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:web/web.dart' as web;

extension on web.HTMLCollection {
  Iterable<web.Element?> get iterable =>
      Iterable<web.Element?>.generate(length, (int index) => item(index));
}

extension on web.CSSRuleList {
  Iterable<web.CSSRule?> get iterable =>
      Iterable<web.CSSRule?>.generate(length, (int index) => item(index));
}

void main() {
  web.HTMLElement? element;
  PlatformSelectableRegionContextMenu.debugOverrideRegisterViewFactory = (
    String viewType,
    Object Function(int viewId) fn, {
    bool isVisible = true,
  }) {
    element = fn(0) as web.HTMLElement;
    // The element needs to be attached to the document body to receive mouse
    // events.
    web.document.body!.append(element! as JSAny);
  };
  // This force register the dom element.
  PlatformSelectableRegionContextMenu(child: const Placeholder());
  PlatformSelectableRegionContextMenu.debugOverrideRegisterViewFactory = null;

  test('DOM element is set up correctly', () async {
    expect(element, isNotNull);
    expect(element!.style.width, '100%');
    expect(element!.style.height, '100%');
    expect(element!.classList.length, 1);
    final String className = element!.className;

    expect(web.document.head!.children.iterable, isNotEmpty);
    bool foundStyle = false;
    for (final web.Element? element in web.document.head!.children.iterable) {
      expect(element, isNotNull);
      if (element!.tagName != 'STYLE') {
        continue;
      }
      final web.CSSRuleList? rules = (element as web.HTMLStyleElement).sheet?.rules;
      if (rules != null) {
        foundStyle = rules.iterable.any((web.CSSRule? rule) => rule!.cssText.contains(className));
      }
      if (foundStyle) {
        break;
      }
    }
    expect(foundStyle, isTrue);
  });

  testWidgets('right click can trigger select word', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    final UniqueKey spy = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: SelectableRegion(
          focusNode: focusNode,
          selectionControls: materialTextSelectionControls,
          child: SelectionSpy(key: spy),
        ),
      ),
    );
    expect(element, isNotNull);

    focusNode.requestFocus();
    await tester.pump();

    // Dispatch right click.
    element!.dispatchEvent(
      web.MouseEvent('mousedown', web.MouseEventInit(button: 2, clientX: 200, clientY: 300)),
    );
    final RenderSelectionSpy renderSelectionSpy = tester.renderObject<RenderSelectionSpy>(
      find.byKey(spy),
    );
    expect(renderSelectionSpy.events, isNotEmpty);

    SelectWordSelectionEvent? selectWordEvent;
    for (final SelectionEvent event in renderSelectionSpy.events) {
      if (event is SelectWordSelectionEvent) {
        selectWordEvent = event;
        break;
      }
    }
    expect(selectWordEvent, isNotNull);
    expect((selectWordEvent!.globalPosition.dx - 200).abs() < precisionErrorTolerance, isTrue);
    expect((selectWordEvent.globalPosition.dy - 300).abs() < precisionErrorTolerance, isTrue);
  });
}

class SelectionSpy extends LeafRenderObjectWidget {
  const SelectionSpy({super.key});

  @override
  RenderObject createRenderObject(BuildContext context) {
    return RenderSelectionSpy(SelectionContainer.maybeOf(context));
  }

  @override
  void updateRenderObject(BuildContext context, covariant RenderObject renderObject) {}
}

class RenderSelectionSpy extends RenderProxyBox with Selectable, SelectionRegistrant {
  RenderSelectionSpy(SelectionRegistrar? registrar) {
    this.registrar = registrar;
  }

  final Set<VoidCallback> listeners = <VoidCallback>{};
  List<SelectionEvent> events = <SelectionEvent>[];

  @override
  Size get size => _size;
  Size _size = Size.zero;

  @override
  List<Rect> get boundingBoxes => _boundingBoxes;
  final List<Rect> _boundingBoxes = <Rect>[];

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    _size = Size(constraints.maxWidth, constraints.maxHeight);
    _boundingBoxes.add(Rect.fromLTWH(0.0, 0.0, constraints.maxWidth, constraints.maxHeight));
    return _size;
  }

  @override
  void addListener(VoidCallback listener) => listeners.add(listener);

  @override
  void removeListener(VoidCallback listener) => listeners.remove(listener);

  @override
  SelectionResult dispatchSelectionEvent(SelectionEvent event) {
    events.add(event);
    return SelectionResult.end;
  }

  @override
  SelectedContent? getSelectedContent() {
    return const SelectedContent(plainText: 'content');
  }

  @override
  SelectedContentRange? getSelection() {
    return null;
  }

  @override
  int get contentLength => 1;

  @override
  final SelectionGeometry value = const SelectionGeometry(
    hasContent: true,
    status: SelectionStatus.uncollapsed,
    startSelectionPoint: SelectionPoint(
      localPosition: Offset.zero,
      lineHeight: 0.0,
      handleType: TextSelectionHandleType.left,
    ),
    endSelectionPoint: SelectionPoint(
      localPosition: Offset.zero,
      lineHeight: 0.0,
      handleType: TextSelectionHandleType.left,
    ),
  );

  @override
  void pushHandleLayers(LayerLink? startHandle, LayerLink? endHandle) {}
}
