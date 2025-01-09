// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../dom.dart';
import 'semantics.dart';

/// The maximum [semanticsActivationAttempts] before we give up waiting for
/// the user to enable semantics.
///
/// This number is arbitrary and can be adjusted if it doesn't work well.
const int kMaxSemanticsActivationAttempts = 20;

/// After an event related to semantics activation has been received, we consume
/// the consecutive events on the engine. Do not send them to the framework.
/// For example when a 'mousedown' targeting a placeholder received following
/// 'mouseup' is also not sent to the framework.
/// Otherwise these events can cause unintended gestures on the framework side.
const Duration _periodToConsumeEvents = Duration(milliseconds: 300);

/// The message in the label for the placeholder element used to enable
/// accessibility.
///
/// This uses US English as the default message. Set this value prior to
/// calling `runApp` to translate to another language.
String placeholderMessage = 'Enable accessibility';

/// A helper for [EngineSemanticsOwner].
///
/// [SemanticsHelper] prepares and placeholder to enable semantics.
///
/// It decides if an event is purely semantics enabling related or a regular
/// event which should be forwarded to the framework.
///
/// It does this by using a [SemanticsEnabler]. The [SemanticsEnabler]
/// implementation is chosen using form factor type.
///
/// See [DesktopSemanticsEnabler], [MobileSemanticsEnabler].
class SemanticsHelper {
  SemanticsEnabler _semanticsEnabler =
      ui_web.browser.isDesktop ? DesktopSemanticsEnabler() : MobileSemanticsEnabler();

  @visibleForTesting
  set semanticsEnabler(SemanticsEnabler semanticsEnabler) {
    _semanticsEnabler = semanticsEnabler;
  }

  bool shouldEnableSemantics(DomEvent event) {
    return _semanticsEnabler.shouldEnableSemantics(event);
  }

  DomElement prepareAccessibilityPlaceholder() {
    return _semanticsEnabler.prepareAccessibilityPlaceholder();
  }

  /// Stops waiting for the user to enable semantics and removes the
  /// placeholder.
  ///
  /// This is used when semantics is enabled programmatically and therefore the
  /// placehodler is no longer needed.
  void dispose() {
    _semanticsEnabler.dispose();
  }
}

@visibleForTesting
abstract class SemanticsEnabler {
  /// Whether to enable semantics.
  ///
  /// Semantics should be enabled if the web engine is no longer waiting for
  /// extra signals from the user events. See [isWaitingToEnableSemantics].
  ///
  /// Or if the received [DomEvent] is suitable/enough for enabling the
  /// semantics. See [tryEnableSemantics].
  bool shouldEnableSemantics(DomEvent event) {
    if (!isWaitingToEnableSemantics) {
      // Forward to framework as normal.
      return true;
    } else {
      return tryEnableSemantics(event);
    }
  }

  /// Attempts to activate semantics.
  ///
  /// Returns true if the `event` is not related to semantics activation and
  /// should be forwarded to the framework.
  bool tryEnableSemantics(DomEvent event);

  /// Creates the placeholder for accessibility.
  ///
  /// Puts it inside the glasspane.
  ///
  /// On focus the element announces that accessibility can be enabled by
  /// tapping/clicking. (Announcement depends on the assistive technology)
  DomElement prepareAccessibilityPlaceholder();

  /// Whether platform is still considering enabling semantics.
  ///
  /// At this stage a relevant set of events are always assessed to see if
  /// they activate the semantics.
  ///
  /// If not they are sent to framework as normal events.
  bool get isWaitingToEnableSemantics;

  /// Stops waiting for the user to enable semantics and removes the placeholder.
  void dispose();
}

/// The desktop semantics enabler uses a simpler strategy compared to mobile.
///
/// A placeholder element is created completely outside the view and is not
/// reachable via touch or mouse. Assistive technology can still find it either
/// using keyboard shortcuts or via next/previous touch gesture (for touch
/// screens). This simplification removes the need for pointer event
/// disambiguation or timers. The placeholder simply waits for a click event
/// and enables semantics.
@visibleForTesting
class DesktopSemanticsEnabler extends SemanticsEnabler {
  /// A temporary placeholder used to capture a request to activate semantics.
  DomElement? _semanticsPlaceholder;

  /// Whether we are waiting for the user to enable semantics.
  @override
  bool get isWaitingToEnableSemantics => _semanticsPlaceholder != null;

  @override
  bool tryEnableSemantics(DomEvent event) {
    // Semantics may be enabled programmatically. If there's a race between that
    // and the DOM event, we may end up here while there's no longer a placeholder
    // to work with.
    if (!isWaitingToEnableSemantics) {
      return true;
    }

    if (EngineSemantics.instance.semanticsEnabled) {
      // Semantics already enabled, forward to framework as normal.
      return true;
    }

    // In touch screen laptops, the touch is received as a mouse click
    const Set<String> kInterestingEventTypes = <String>{
      'click',
      'keyup',
      'keydown',
      'mouseup',
      'mousedown',
      'pointerdown',
      'pointerup',
    };

    if (!kInterestingEventTypes.contains(event.type)) {
      // The event is not relevant, forward to framework as normal.
      return true;
    }

    // Check for the event target.
    final bool enableConditionPassed = event.target == _semanticsPlaceholder;

    if (!enableConditionPassed) {
      // This was not a semantics activating event; forward as normal.
      return true;
    }

    EngineSemantics.instance.semanticsEnabled = true;
    dispose();
    return false;
  }

  @override
  DomElement prepareAccessibilityPlaceholder() {
    final DomElement placeholder =
        _semanticsPlaceholder = createDomElement('flt-semantics-placeholder');

    // Only listen to "click" because other kinds of events are reported via
    // PointerBinding.
    placeholder.addEventListener(
      'click',
      createDomEventListener((DomEvent event) {
        tryEnableSemantics(event);
      }),
      true,
    );

    // Adding roles to semantics placeholder. 'aria-live' will make sure that
    // the content is announced to the assistive technology user as soon as the
    // page receives focus. 'tabindex' makes sure the button is the first
    // target of tab. 'aria-label' is used to define the placeholder message
    // to the assistive technology user.
    placeholder
      ..setAttribute('role', 'button')
      ..setAttribute('aria-live', 'polite')
      ..setAttribute('tabindex', '0')
      ..setAttribute('aria-label', placeholderMessage);

    // The placeholder sits just outside the window so only AT can reach it.
    placeholder.style
      ..position = 'absolute'
      ..left = '-1px'
      ..top = '-1px'
      ..width = '1px'
      ..height = '1px';
    return placeholder;
  }

  @override
  void dispose() {
    _semanticsPlaceholder?.remove();
    _semanticsPlaceholder = null;
  }
}

@visibleForTesting
class MobileSemanticsEnabler extends SemanticsEnabler {
  /// We do not immediately enable semantics when the user requests it, but
  /// instead wait for a short period of time before doing it. This is because
  /// the request comes as an event targeted on the [_semanticsPlaceholder].
  /// This event, depending on the browser, comes as a burst of events.
  /// For example, Safari on IOS sends "touchstart", "touchend", and "click".
  /// So during a short time period we consume all events and prevent forwarding
  /// to the framework. Otherwise, the events will be interpreted twice, once as
  /// a request to activate semantics, and a second time by Flutter's gesture
  /// recognizers.
  @visibleForTesting
  Timer? semanticsActivationTimer;

  /// A temporary placeholder used to capture a request to activate semantics.
  DomElement? _semanticsPlaceholder;

  /// The number of events we processed that could potentially activate
  /// semantics.
  int semanticsActivationAttempts = 0;

  /// Instructs [_tryEnableSemantics] to remove [_semanticsPlaceholder].
  ///
  /// For Blink browser engine the placeholder is removed upon any next event.
  ///
  /// For Webkit browser engine the placeholder is removed upon the next
  /// "touchend" event. This is to prevent Safari from swallowing the event
  /// that happens on an element that's being removed. Blink doesn't have
  /// this issue.
  bool _schedulePlaceholderRemoval = false;

  /// Whether we are waiting for the user to enable semantics.
  @override
  bool get isWaitingToEnableSemantics => _semanticsPlaceholder != null;

  @override
  bool tryEnableSemantics(DomEvent event) {
    // Semantics may be enabled programmatically. If there's a race between that
    // and the DOM event, we may end up here while there's no longer a placeholder
    // to work with.
    if (!isWaitingToEnableSemantics) {
      return true;
    }

    if (_schedulePlaceholderRemoval) {
      // The event type can also be click for VoiceOver.
      final bool removeNow =
          ui_web.browser.browserEngine != ui_web.BrowserEngine.webkit ||
          event.type == 'touchend' ||
          event.type == 'pointerup' ||
          event.type == 'click';
      if (removeNow) {
        dispose();
      }
      return true;
    }

    if (EngineSemantics.instance.semanticsEnabled) {
      // Semantics already enabled, forward to framework as normal.
      return true;
    }

    semanticsActivationAttempts += 1;
    if (semanticsActivationAttempts >= kMaxSemanticsActivationAttempts) {
      // We have received multiple user events, none of which resulted in
      // semantics activation. This is a signal that the user is not interested
      // in semantics, and so we will stop waiting for it.
      _schedulePlaceholderRemoval = true;
      return true;
    }

    // ios-safari browsers which starts sending `pointer` events instead of
    // `touch` events. (Tested with 12.1 which uses touch events vs 13.5
    // which uses pointer events.)
    const Set<String> kInterestingEventTypes = <String>{
      'click',
      'touchstart',
      'touchend',
      'pointerdown',
      'pointermove',
      'pointerup',
    };

    if (!kInterestingEventTypes.contains(event.type)) {
      // The event is not relevant, forward to framework as normal.
      return true;
    }

    if (semanticsActivationTimer != null) {
      // We are in a waiting period to activate a timer. While the timer is
      // active we should consume events pertaining to semantics activation.
      // Otherwise the event will also be interpreted by the framework and
      // potentially result in activating a gesture in the app.
      return false;
    }

    // Look at where exactly (within 1 pixel) the event landed. If it landed
    // exactly in the middle of the placeholder we interpret it as a signal
    // to enable accessibility. This is because when VoiceOver and TalkBack
    // generate a tap it lands it in the middle of the focused element. This
    // method is a bit flawed in that a user's finger could theoretically land
    // in the middle of the element too. However, the chance of that happening
    // is very small. Even low-end phones typically have >2 million pixels
    // (e.g. Moto G4). It is very unlikely that a user will land their finger
    // exactly in the middle. In the worst case an unlucky user would
    // accidentally enable accessibility and the app will be slightly slower
    // than normal, but the app will continue functioning as normal. Our
    // semantics tree is designed to not interfere with Flutter's gesture
    // detection.
    bool enableConditionPassed = false;
    late final DomPoint activationPoint;

    switch (event.type) {
      case 'click':
        final DomMouseEvent click = event as DomMouseEvent;
        activationPoint = click.offset;
      case 'touchstart':
      case 'touchend':
        final DomTouchEvent touchEvent = event as DomTouchEvent;
        activationPoint = touchEvent.changedTouches.first.client;
      case 'pointerdown':
      case 'pointerup':
        final DomPointerEvent touch = event as DomPointerEvent;
        activationPoint = touch.client;
      default:
        // The event is not relevant, forward to framework as normal.
        return true;
    }

    final DomRect activatingElementRect = _semanticsPlaceholder!.getBoundingClientRect();
    final double midX =
        activatingElementRect.left + (activatingElementRect.right - activatingElementRect.left) / 2;
    final double midY =
        activatingElementRect.top + (activatingElementRect.bottom - activatingElementRect.top) / 2;
    final double deltaX = activationPoint.x.toDouble() - midX;
    final double deltaY = activationPoint.y.toDouble() - midY;
    final double deltaSquared = deltaX * deltaX + deltaY * deltaY;
    if (deltaSquared < 1.0) {
      enableConditionPassed = true;
    }

    if (enableConditionPassed) {
      assert(semanticsActivationTimer == null);
      _schedulePlaceholderRemoval = true;
      semanticsActivationTimer = Timer(_periodToConsumeEvents, () {
        dispose();
        EngineSemantics.instance.semanticsEnabled = true;
      });
      return false;
    }

    // This was not a semantics activating event; forward as normal.
    return true;
  }

  @override
  DomElement prepareAccessibilityPlaceholder() {
    final DomElement placeholder =
        _semanticsPlaceholder = createDomElement('flt-semantics-placeholder');

    // Only listen to "click" because other kinds of events are reported via
    // PointerBinding.
    placeholder.addEventListener(
      'click',
      createDomEventListener((DomEvent event) {
        tryEnableSemantics(event);
      }),
      true,
    );

    placeholder
      ..setAttribute('role', 'button')
      ..setAttribute('aria-label', placeholderMessage);
    placeholder.style
      ..position = 'absolute'
      ..left = '0'
      ..top = '0'
      ..right = '0'
      ..bottom = '0';

    return placeholder;
  }

  @override
  void dispose() {
    _semanticsPlaceholder?.remove();
    _semanticsPlaceholder = null;
    semanticsActivationTimer = null;
  }
}
