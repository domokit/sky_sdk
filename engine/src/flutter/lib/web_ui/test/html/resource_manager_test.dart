// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  final DomElement hostElement = createDomHTMLDivElement();
  late DomManager domManager;
  late ResourceManager resourceManager;

  setUp(() {
    domManager = DomManager(devicePixelRatio: 3);
    hostElement.appendChild(domManager.rootElement);
    resourceManager = ResourceManager(domManager);
  });

  tearDown(() {
    hostElement.clearChildren();
  });

  test('prepends resources host as sibling to root element (webkit)', () {
    ui_web.browser.debugBrowserEngineOverride = ui_web.BrowserEngine.webkit;

    // Resource host hasn't been inserted yet.
    expect(
      hostElement.children.map((DomElement e) => e.tagName.toLowerCase()),
      isNot(contains(ResourceManager.resourcesHostTagName.toLowerCase())),
    );

    final List<DomElement> resources = <DomElement>[
      createDomHTMLDivElement()..setAttribute('test-resource', 'r1'),
      createDomHTMLDivElement()..setAttribute('test-resource', 'r2'),
      createDomHTMLDivElement()..setAttribute('test-resource', 'r3'),
    ];
    resources.forEach(resourceManager.addResource);

    final DomElement resourcesHost = hostElement.firstElementChild!;
    expect(resourcesHost.tagName.toLowerCase(), ResourceManager.resourcesHostTagName.toLowerCase());
    // Make sure the resources were correctly inserted into the host.
    expect(resourcesHost.children, resources);

    ui_web.browser.debugBrowserEngineOverride = null;
  });

  test('prepends resources host inside the shadow root (non-webkit)', () {
    ui_web.browser.debugBrowserEngineOverride = ui_web.BrowserEngine.blink;

    // Resource host hasn't been inserted yet.
    expect(
      hostElement.children.map((DomElement e) => e.tagName.toLowerCase()),
      isNot(contains(ResourceManager.resourcesHostTagName.toLowerCase())),
    );

    final List<DomElement> resources = <DomElement>[
      createDomHTMLDivElement()..setAttribute('test-resource', 'r1'),
      createDomHTMLDivElement()..setAttribute('test-resource', 'r2'),
      createDomHTMLDivElement()..setAttribute('test-resource', 'r3'),
    ];
    resources.forEach(resourceManager.addResource);

    final DomElement resourcesHost = domManager.renderingHost.firstElementChild!;
    expect(resourcesHost.tagName.toLowerCase(), ResourceManager.resourcesHostTagName.toLowerCase());
    // Make sure the resources were correctly inserted into the host.
    expect(resourcesHost.children, resources);

    ui_web.browser.debugBrowserEngineOverride = null;
  });

  test('can remove resource', () {
    final DomHTMLDivElement resource = createDomHTMLDivElement();
    resourceManager.addResource(resource);
    final DomElement? resourceRoot = resource.parent;
    expect(resourceRoot, isNotNull);
    expect(resourceRoot!.childNodes.length, 1);
    resourceManager.removeResource(resource);
    expect(resourceRoot.childNodes.length, 0);
  });
}
