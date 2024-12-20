// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

const String _kDefaultCssFont = '14px monospace';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  late DomHTMLStyleElement styleElement;

  setUp(() {
    styleElement = createDomHTMLStyleElement(null);
    applyGlobalCssRulesToSheet(styleElement, defaultCssFont: _kDefaultCssFont);
  });

  tearDown(() {
    styleElement.remove();
  });

  test('createDomHTMLStyleElement sets a nonce value, when passed', () {
    expect(styleElement.nonce, isEmpty);

    final DomHTMLStyleElement style = createDomHTMLStyleElement('a-nonce-value');
    expect(style.nonce, 'a-nonce-value');
  });

  test('(Self-test) hasCssRule can extract rules', () {
    final bool hasRule = hasCssRule(
      styleElement,
      selector: '.flt-text-editing::placeholder',
      declaration: 'opacity: 0',
    );

    final bool hasFakeRule = hasCssRule(
      styleElement,
      selector: 'input::selection',
      declaration: 'color: #fabada;',
    );

    expect(hasRule, isTrue);
    expect(hasFakeRule, isFalse);
  });

  test('Attaches styling to remove password reveal icons on Edge', () {
    // Check that style.sheet! contains input::-ms-reveal rule
    final bool hidesRevealIcons = hasCssRule(
      styleElement,
      selector: 'input::-ms-reveal',
      declaration: 'display: none',
    );

    final bool codeRanInFakeyBrowser = hasCssRule(
      styleElement,
      selector: 'input.fallback-for-fakey-browser-in-ci',
      declaration: 'display: none',
    );

    if (codeRanInFakeyBrowser) {
      print('Please, fix https://github.com/flutter/flutter/issues/116302');
    }

    expect(
      hidesRevealIcons || codeRanInFakeyBrowser,
      isTrue,
      reason: 'In Edge, stylesheet must contain "input::-ms-reveal" rule.',
    );
  }, skip: !isEdge);

  test('Does not attach the Edge-specific style tag on non-Edge browsers', () {
    // Check that style.sheet! contains input::-ms-reveal rule
    final bool hidesRevealIcons = hasCssRule(
      styleElement,
      selector: 'input::-ms-reveal',
      declaration: 'display: none',
    );

    expect(hidesRevealIcons, isFalse);
  }, skip: isEdge);

  test(
    'Attaches styles to hide the autofill overlay for browsers that support it',
    () {
      final String vendorPrefix = (isSafari || isFirefox) ? '' : '-webkit-';
      final bool autofillOverlay = hasCssRule(
        styleElement,
        selector: '.transparentTextEditing:${vendorPrefix}autofill',
        declaration: 'opacity: 0 !important',
      );
      final bool autofillOverlayHovered = hasCssRule(
        styleElement,
        selector: '.transparentTextEditing:${vendorPrefix}autofill:hover',
        declaration: 'opacity: 0 !important',
      );
      final bool autofillOverlayFocused = hasCssRule(
        styleElement,
        selector: '.transparentTextEditing:${vendorPrefix}autofill:focus',
        declaration: 'opacity: 0 !important',
      );
      final bool autofillOverlayActive = hasCssRule(
        styleElement,
        selector: '.transparentTextEditing:${vendorPrefix}autofill:active',
        declaration: 'opacity: 0 !important',
      );

      expect(autofillOverlay, isTrue);
      expect(autofillOverlayHovered, isTrue);
      expect(autofillOverlayFocused, isTrue);
      expect(autofillOverlayActive, isTrue);
    },
    skip: !browserHasAutofillOverlay(),
  );
}

/// Finds out whether a given CSS Rule ([selector] { [declaration]; }) exists in a [styleElement].
bool hasCssRule(
  DomHTMLStyleElement styleElement, {
  required String selector,
  required String declaration,
}) {
  domDocument.body!.append(styleElement);
  assert(styleElement.sheet != null);

  // regexr.com/740ff
  final RegExp ruleLike = RegExp('[^{]*(?:$selector)[^{]*{[^}]*(?:$declaration)[^}]*}');

  final DomCSSStyleSheet sheet = styleElement.sheet! as DomCSSStyleSheet;

  // Check that the cssText of any rule matches the ruleLike RegExp.
  final bool result = sheet.cssRules
      .map((DomCSSRule rule) => rule.cssText)
      .any((String rule) => ruleLike.hasMatch(rule));

  styleElement.remove();
  return result;
}
