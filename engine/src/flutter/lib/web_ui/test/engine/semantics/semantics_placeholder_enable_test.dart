// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine.dart';

import '../../common/test_initialization.dart';

EngineSemantics semantics() => EngineSemantics.instance;

void main() {
  internalBootstrapBrowserTest(() {
    return testMain;
  });
}

Future<void> testMain() async {
  setUpImplicitView();

  test('EngineSemantics is enabled via a placeholder click', () async {
    expect(semantics().semanticsEnabled, isFalse);

    // Synthesize a click on the placeholder.
    final DomElement placeholder = domDocument.querySelector('flt-semantics-placeholder')!;

    expect(placeholder.isConnected, isTrue);

    final DomRect rect = placeholder.getBoundingClientRect();
    placeholder.dispatchEvent(
      createDomMouseEvent('click', <Object?, Object?>{
        'clientX': (rect.left + (rect.right - rect.left) / 2).floor(),
        'clientY': (rect.top + (rect.bottom - rect.top) / 2).floor(),
      }),
    );

    // On mobile semantics is enabled asynchronously.
    if (isMobile) {
      while (placeholder.isConnected!) {
        await Future<void>.delayed(const Duration(milliseconds: 50));
      }
    }
    expect(semantics().semanticsEnabled, isTrue);
    expect(placeholder.isConnected, isFalse);
  });
}
