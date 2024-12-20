// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('$MouseCursor', () {
    test('sets correct `cursor` style on root element', () {
      final DomElement rootViewElement = createDomElement('div');
      final MouseCursor mouseCursor = MouseCursor(rootViewElement);

      // TODO(mdebbar): This should be `rootViewElement`.
      //                https://github.com/flutter/flutter/issues/140226
      final DomElement cursorTarget = domDocument.body!;

      mouseCursor.activateSystemCursor('alias');
      expect(cursorTarget.style.cursor, 'alias');

      mouseCursor.activateSystemCursor('move');
      expect(cursorTarget.style.cursor, 'move');

      mouseCursor.activateSystemCursor('precise');
      expect(cursorTarget.style.cursor, 'crosshair');

      mouseCursor.activateSystemCursor('resizeDownRight');
      expect(cursorTarget.style.cursor, 'se-resize');

      mouseCursor.activateSystemCursor('basic');
      expect(cursorTarget.style.cursor, isEmpty);
    });

    test('handles unknown cursor type', () {
      final DomElement rootViewElement = createDomElement('div');
      final MouseCursor mouseCursor = MouseCursor(rootViewElement);

      // TODO(mdebbar): This should be `rootViewElement`.
      //                https://github.com/flutter/flutter/issues/140226
      final DomElement cursorTarget = domDocument.body!;

      mouseCursor.activateSystemCursor('unknown');
      expect(cursorTarget.style.cursor, isEmpty);

      mouseCursor.activateSystemCursor(null);
    });
  });
}
