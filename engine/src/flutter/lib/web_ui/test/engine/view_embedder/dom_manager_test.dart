// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@JS()
library dom_manager_test; // We need this to mess with the ShadowDOM.

import 'dart:js_interop';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../../common/matchers.dart';

void main() {
  internalBootstrapBrowserTest(() => doTests);
}

void doTests() {
  group('DomManager', () {
    test('DOM tree looks right', () {
      final DomManager domManager = DomManager(devicePixelRatio: 3.0);

      // Check tag names.

      expect(domManager.rootElement.tagName, equalsIgnoringCase(DomManager.flutterViewTagName));
      expect(domManager.platformViewsHost.tagName, equalsIgnoringCase(DomManager.glassPaneTagName));
      expect(
        domManager.textEditingHost.tagName,
        equalsIgnoringCase(DomManager.textEditingHostTagName),
      );
      expect(domManager.semanticsHost.tagName, equalsIgnoringCase(DomManager.semanticsHostTagName));

      // Check parent-child relationships.

      final List<DomElement> rootChildren = domManager.rootElement.children.toList();
      expect(rootChildren.length, 4);
      expect(rootChildren[0], domManager.platformViewsHost);
      expect(rootChildren[1], domManager.textEditingHost);
      expect(rootChildren[2], domManager.semanticsHost);
      expect(rootChildren[3].tagName, equalsIgnoringCase('style'));

      final List<DomElement> shadowChildren =
          domManager.renderingHost.childNodes.cast<DomElement>().toList();
      expect(shadowChildren.length, 2);
      expect(shadowChildren[0], domManager.sceneHost);
      expect(shadowChildren[1].tagName, equalsIgnoringCase('style'));
    });

    test(
      'hide placeholder text for textfield',
      () {
        final DomManager domManager = DomManager(devicePixelRatio: 3.0);
        domDocument.body!.append(domManager.rootElement);

        final DomHTMLInputElement regularTextField = createDomHTMLInputElement();
        regularTextField.placeholder = 'Now you see me';
        domManager.rootElement.appendChild(regularTextField);

        regularTextField.focusWithoutScroll();
        DomCSSStyleDeclaration? style = domWindow.getComputedStyle(
          domManager.rootElement.querySelector('input')!,
          '::placeholder',
        );
        expect(style, isNotNull);
        expect(style.opacity, isNot('0'));

        final DomHTMLInputElement textField = createDomHTMLInputElement();
        textField.placeholder = 'Now you dont';
        textField.classList.add('flt-text-editing');
        domManager.rootElement.appendChild(textField);

        textField.focusWithoutScroll();
        style = domWindow.getComputedStyle(
          domManager.rootElement.querySelector('input.flt-text-editing')!,
          '::placeholder',
        );
        expect(style, isNotNull);
        expect(style.opacity, '0');

        domManager.rootElement.remove();

        // For some reason, only Firefox is able to correctly compute styles for
        // the `::placeholder` pseudo-element.
      },
      skip: ui_web.browser.browserEngine != ui_web.BrowserEngine.firefox,
    );
  });

  group('Shadow root', () {
    test('throws when shadowDom is not available', () {
      final dynamic oldAttachShadow = attachShadow;
      expect(oldAttachShadow, isNotNull);

      attachShadow = null; // Break ShadowDOM

      expect(() => DomManager(devicePixelRatio: 3.0), throwsAssertionError);
      attachShadow = oldAttachShadow; // Restore ShadowDOM
    });

    test('Initializes and attaches a shadow root', () {
      final DomManager domManager = DomManager(devicePixelRatio: 3.0);

      expect(domInstanceOfString(domManager.renderingHost, 'ShadowRoot'), isTrue);
      expect(domManager.renderingHost.host, domManager.platformViewsHost);
      expect(domManager.renderingHost, domManager.platformViewsHost.shadowRoot);

      // The shadow root should be initialized with correct parameters.
      expect(domManager.renderingHost.mode, 'open');
      if (ui_web.browser.browserEngine != ui_web.BrowserEngine.firefox &&
          ui_web.browser.browserEngine != ui_web.BrowserEngine.webkit) {
        // Older versions of Safari and Firefox don't support this flag yet.
        // See: https://caniuse.com/mdn-api_shadowroot_delegatesfocus
        expect(domManager.renderingHost.delegatesFocus, isFalse);
      }
    });

    test('Attaches a stylesheet to the shadow root', () {
      final DomManager domManager = DomManager(devicePixelRatio: 3.0);
      final DomElement? style = domManager.renderingHost.querySelector('#flt-internals-stylesheet');

      expect(style, isNotNull);
      expect(style!.tagName, equalsIgnoringCase('style'));
      expect(style.parentNode, domManager.renderingHost);
    });

    test('setScene', () {
      final DomManager domManager = DomManager(devicePixelRatio: 3.0);

      final DomElement sceneHost = domManager.renderingHost.querySelector('flt-scene-host')!;

      final DomElement scene1 = createDomElement('flt-scene');
      domManager.setScene(scene1);
      expect(sceneHost.children, <DomElement>[scene1]);

      // Insert the same scene again.
      domManager.setScene(scene1);
      expect(sceneHost.children, <DomElement>[scene1]);

      // Insert a different scene.
      final DomElement scene2 = createDomElement('flt-scene');
      domManager.setScene(scene2);
      expect(sceneHost.children, <DomElement>[scene2]);
      expect(scene1.parent, isNull);
    });
  });
}

@JS('Element.prototype.attachShadow')
external JSAny? get _attachShadow;
dynamic get attachShadow => _attachShadow?.toObjectShallow;

@JS('Element.prototype.attachShadow')
external set _attachShadow(JSAny? x);
set attachShadow(Object? x) => _attachShadow = x?.toJSAnyShallow;
