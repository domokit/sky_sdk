// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine.dart';

import '../../common/test_initialization.dart';
import 'semantics_tester.dart';

EngineSemantics semantics() => EngineSemantics.instance;
EngineSemanticsOwner owner() => EnginePlatformDispatcher.instance.implicitView!.semantics;

void main() {
  internalBootstrapBrowserTest(() {
    return testMain;
  });
}

Future<void> testMain() async {
  setUpImplicitView();

  test('EngineSemanticsOwner auto-enables semantics on update', () async {
    expect(semantics().semanticsEnabled, isFalse);
    expect(EnginePlatformDispatcher.instance.accessibilityFeatures.accessibleNavigation, isFalse);

    final DomElement placeholder = domDocument.querySelector('flt-semantics-placeholder')!;

    expect(placeholder.isConnected, isTrue);

    // Sending a semantics update should auto-enable engine semantics.
    final SemanticsTester tester = SemanticsTester(owner());
    tester.updateNode(id: 0);
    tester.apply();

    expect(semantics().semanticsEnabled, isTrue);
    expect(EnginePlatformDispatcher.instance.accessibilityFeatures.accessibleNavigation, isTrue);

    // The placeholder should be removed
    expect(placeholder.isConnected, isFalse);
  });
}
