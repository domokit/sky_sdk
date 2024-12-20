// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine/browser_detection.dart';
import 'package:ui/src/engine/dom.dart';
import 'package:ui/src/engine/semantics.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('$DesktopSemanticsEnabler', () {
    late DesktopSemanticsEnabler desktopSemanticsEnabler;
    late DomElement? placeholder;

    setUp(() {
      EngineSemantics.instance.semanticsEnabled = false;
      desktopSemanticsEnabler = DesktopSemanticsEnabler();
      placeholder = desktopSemanticsEnabler.prepareAccessibilityPlaceholder();
      domDocument.body!.append(placeholder!);
    });

    tearDown(() {
      expect(placeholder, isNotNull, reason: 'Expected the test to create a placeholder');
      placeholder!.remove();
      EngineSemantics.instance.semanticsEnabled = false;
    });

    test('prepare accessibility placeholder', () async {
      expect(placeholder!.getAttribute('role'), 'button');
      expect(placeholder!.getAttribute('aria-live'), 'polite');
      expect(placeholder!.getAttribute('tabindex'), '0');

      domDocument.body!.append(placeholder!);

      expect(domDocument.getElementsByTagName('flt-semantics-placeholder'), isNotEmpty);

      expect(placeholder!.getBoundingClientRect().height, 1);
      expect(placeholder!.getBoundingClientRect().width, 1);
      expect(placeholder!.getBoundingClientRect().top, -1);
      expect(placeholder!.getBoundingClientRect().left, -1);
    });

    test('Not relevant events should be forwarded to the framework', () async {
      // Attach the placeholder to dom.
      domDocument.body!.append(placeholder!);

      DomEvent event = createDomEvent('Event', 'mousemove');
      bool shouldForwardToFramework = desktopSemanticsEnabler.tryEnableSemantics(event);

      expect(shouldForwardToFramework, isTrue);

      // Pointer events are not defined in webkit.
      if (ui_web.browser.browserEngine != ui_web.BrowserEngine.webkit) {
        event = createDomEvent('Event', 'pointermove');
        shouldForwardToFramework = desktopSemanticsEnabler.tryEnableSemantics(event);

        expect(shouldForwardToFramework, isTrue);
      }
    });

    test(
      'Relevant events targeting placeholder should not be forwarded to the framework',
      () async {
        final DomEvent event = createDomEvent('Event', 'mousedown');
        placeholder!.dispatchEvent(event);

        final bool shouldForwardToFramework = desktopSemanticsEnabler.tryEnableSemantics(event);

        expect(shouldForwardToFramework, isFalse);
      },
    );

    test('disposes of the placeholder', () {
      domDocument.body!.append(placeholder!);

      expect(placeholder!.isConnected, isTrue);
      desktopSemanticsEnabler.dispose();
      expect(placeholder!.isConnected, isFalse);
    });
  }, skip: isMobile);

  group(
    '$MobileSemanticsEnabler',
    () {
      late MobileSemanticsEnabler mobileSemanticsEnabler;
      DomElement? placeholder;

      setUp(() {
        EngineSemantics.instance.semanticsEnabled = false;
        mobileSemanticsEnabler = MobileSemanticsEnabler();
        placeholder = mobileSemanticsEnabler.prepareAccessibilityPlaceholder();
        domDocument.body!.append(placeholder!);
      });

      tearDown(() {
        placeholder!.remove();
        EngineSemantics.instance.semanticsEnabled = false;
      });

      test('prepare accessibility placeholder', () async {
        expect(placeholder!.getAttribute('role'), 'button');

        // Placeholder should cover all the screen on a mobile device.
        final num bodyHeight = domWindow.innerHeight!;
        final num bodyWidth = domWindow.innerWidth!;

        expect(placeholder!.getBoundingClientRect().height, bodyHeight);
        expect(placeholder!.getBoundingClientRect().width, bodyWidth);
      });

      test('Non-relevant events should be forwarded to the framework', () async {
        final DomEvent event = createDomPointerEvent('pointermove');

        final bool shouldForwardToFramework = mobileSemanticsEnabler.tryEnableSemantics(event);

        expect(shouldForwardToFramework, isTrue);
      });

      test('Enables semantics when receiving a relevant event', () {
        expect(mobileSemanticsEnabler.semanticsActivationTimer, isNull);

        // Send a click off center
        placeholder!.dispatchEvent(
          createDomMouseEvent('click', <Object?, Object?>{'clientX': 0, 'clientY': 0}),
        );
        expect(mobileSemanticsEnabler.semanticsActivationTimer, isNull);

        // Send a click at center
        final DomRect activatingElementRect = placeholder!.getBoundingClientRect();
        final int midX =
            (activatingElementRect.left +
                    (activatingElementRect.right - activatingElementRect.left) / 2)
                .toInt();
        final int midY =
            (activatingElementRect.top +
                    (activatingElementRect.bottom - activatingElementRect.top) / 2)
                .toInt();
        placeholder!.dispatchEvent(
          createDomMouseEvent('click', <Object?, Object?>{'clientX': midX, 'clientY': midY}),
        );
        expect(mobileSemanticsEnabler.semanticsActivationTimer, isNotNull);
      });
    },
    // We can run `MobileSemanticsEnabler` tests in mobile browsers and in desktop Chrome.
    skip: isDesktop && ui_web.browser.browserEngine != ui_web.BrowserEngine.blink,
  );
}
