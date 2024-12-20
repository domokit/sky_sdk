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
  group('PlatformViewManager', () {
    const int viewId = 6;

    group('createPlatformViewSlot', () {
      test('can render slot, even for views that might have never been rendered before', () async {
        final DomElement slot = createPlatformViewSlot(viewId);
        expect(slot, isNotNull);
        expect(slot.querySelector('slot'), isNotNull);
      });

      test('rendered markup contains required attributes', () async {
        final DomElement slot = createPlatformViewSlot(viewId);
        expect(
          slot.style.pointerEvents,
          'auto',
          reason: 'Should re-enable pointer events for the contents of the view.',
        );
        final DomElement innerSlot = slot.querySelector('slot')!;
        expect(
          innerSlot.getAttribute('name'),
          contains('$viewId'),
          reason: 'The name attribute of the inner SLOT tag must refer to the viewId.',
        );
      });
    });
  });

  test('getPlatformViewSlotName', () {
    expect(getPlatformViewSlotName(42), 'flt-pv-slot-42');
  });

  test('getPlatformViewDomId', () {
    expect(getPlatformViewDomId(42), 'flt-pv-42');
  });
}
