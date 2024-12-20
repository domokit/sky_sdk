// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

/// Implements vertical and horizontal scrolling functionality for semantics
/// objects.
///
/// Scrolling is implemented using a "joystick" method. The absolute value of
/// "scrollTop" in HTML is not important. We only need to know in whether the
/// value changed in the positive or negative direction. If it changes in the
/// positive direction we send a [ui.SemanticsAction.scrollUp]. Otherwise, we
/// send [ui.SemanticsAction.scrollDown]. The actual scrolling is then handled
/// by the framework and we receive a [ui.SemanticsUpdate] containing the new
/// [scrollPosition] and child positions.
///
/// "scrollTop" or "scrollLeft" is always reset to an arbitrarily chosen non-
/// zero "neutral" scroll position value. This is done so we have a
/// predictable range of DOM scroll position values. When the amount of
/// contents is less than the size of the viewport the browser snaps
/// "scrollTop" back to zero. If there is more content than available in the
/// viewport "scrollTop" may take positive values.
class SemanticScrollable extends SemanticRole {
  SemanticScrollable(SemanticsObject semanticsObject)
    : super.withBasics(
        SemanticRoleKind.scrollable,
        semanticsObject,
        preferredLabelRepresentation: LabelRepresentation.ariaLabel,
      ) {
    // Mark as group to prevent the browser from merging this element along with
    // all the children into one giant node. This is what happened with the
    // repro provided in https://github.com/flutter/flutter/issues/130950.
    setAriaRole('group');
  }

  /// Disables browser-driven scrolling in the presence of pointer events.
  GestureModeCallback? _gestureModeListener;

  /// DOM element used as a workaround for: https://github.com/flutter/flutter/issues/104036
  ///
  /// When the assistive technology gets to the last element of the scrollable
  /// list, the browser thinks the scrollable area doesn't have any more content,
  /// so it overrides the value of "scrollTop"/"scrollLeft" with zero. As a result,
  /// the user can't scroll back up/left.
  ///
  /// As a workaround, we add this DOM element and set its size to
  /// [canonicalNeutralScrollPosition] so the browser believes
  /// that the scrollable area still has some more content, and doesn't override
  /// scrollTop/scrollLetf with zero.
  final DomElement _scrollOverflowElement = createDomElement('flt-semantics-scroll-overflow');

  /// Listens to HTML "scroll" gestures detected by the browser.
  ///
  /// This gesture is converted to [ui.SemanticsAction.scrollUp] or
  /// [ui.SemanticsAction.scrollDown], depending on the direction.
  DomEventListener? _scrollListener;

  /// The value of the "scrollTop" or "scrollLeft" property of this object's
  /// [element] that has zero offset relative to the [scrollPosition].
  int _effectiveNeutralScrollPosition = 0;

  /// Responds to browser-detected "scroll" gestures.
  void _recomputeScrollPosition() {
    if (_domScrollPosition != _effectiveNeutralScrollPosition) {
      if (!EngineSemantics.instance.shouldAcceptBrowserGesture('scroll')) {
        return;
      }
      final bool doScrollForward = _domScrollPosition > _effectiveNeutralScrollPosition;
      _neutralizeDomScrollPosition();
      semanticsObject.recomputePositionAndSize();

      final int semanticsId = semanticsObject.id;
      if (doScrollForward) {
        if (semanticsObject.isVerticalScrollContainer) {
          EnginePlatformDispatcher.instance.invokeOnSemanticsAction(
            viewId,
            semanticsId,
            ui.SemanticsAction.scrollUp,
            null,
          );
        } else {
          assert(semanticsObject.isHorizontalScrollContainer);
          EnginePlatformDispatcher.instance.invokeOnSemanticsAction(
            viewId,
            semanticsId,
            ui.SemanticsAction.scrollLeft,
            null,
          );
        }
      } else {
        if (semanticsObject.isVerticalScrollContainer) {
          EnginePlatformDispatcher.instance.invokeOnSemanticsAction(
            viewId,
            semanticsId,
            ui.SemanticsAction.scrollDown,
            null,
          );
        } else {
          assert(semanticsObject.isHorizontalScrollContainer);
          EnginePlatformDispatcher.instance.invokeOnSemanticsAction(
            viewId,
            semanticsId,
            ui.SemanticsAction.scrollRight,
            null,
          );
        }
      }
    }
  }

  @override
  void initState() {
    // Scrolling is controlled by setting overflow-y/overflow-x to 'scroll`. The
    // default overflow = "visible" needs to be unset.
    semanticsObject.element.style.overflow = '';

    _scrollOverflowElement.style
      ..position = 'absolute'
      ..transformOrigin = '0 0 0'
      // Ignore pointer events since this is a dummy element.
      ..pointerEvents = 'none';
    append(_scrollOverflowElement);
  }

  @override
  void update() {
    super.update();

    semanticsObject.owner.addOneTimePostUpdateCallback(() {
      _neutralizeDomScrollPosition();
      semanticsObject.recomputePositionAndSize();
    });

    if (_scrollListener == null) {
      // We need to set touch-action:none explicitly here, despite the fact
      // that we already have it on the <body> tag because overflow:scroll
      // still causes the browser to take over pointer events in order to
      // process scrolling. We don't want that when scrolling is handled by
      // the framework.
      //
      // This is effective only in Chrome. Safari does not implement this
      // CSS property. In Safari the `PointerBinding` uses `preventDefault`
      // to prevent browser scrolling.
      element.style.touchAction = 'none';
      _gestureModeDidChange();

      // Memoize the tear-off because Dart does not guarantee that two
      // tear-offs of a method on the same instance will produce the same
      // object.
      _gestureModeListener = (_) {
        _gestureModeDidChange();
      };
      EngineSemantics.instance.addGestureModeListener(_gestureModeListener!);

      _scrollListener = createDomEventListener((_) {
        _recomputeScrollPosition();
      });
      addEventListener('scroll', _scrollListener);
    }
  }

  /// The value of "scrollTop" or "scrollLeft", depending on the scroll axis.
  int get _domScrollPosition {
    if (semanticsObject.isVerticalScrollContainer) {
      return element.scrollTop.toInt();
    } else {
      assert(semanticsObject.isHorizontalScrollContainer);
      return element.scrollLeft.toInt();
    }
  }

  /// Resets the scroll position (top or left) to the neutral value.
  ///
  /// The scroll position of the scrollable HTML node that's considered to
  /// have zero offset relative to Flutter's notion of scroll position is
  /// referred to as "neutral scroll position".
  ///
  /// We always set the scroll position to a non-zero value in order to
  /// be able to scroll in the negative direction. When scrollTop/scrollLeft is
  /// zero the browser will refuse to scroll back even when there is more
  /// content available.
  void _neutralizeDomScrollPosition() {
    // This value is arbitrary.
    const int canonicalNeutralScrollPosition = 10;
    final ui.Rect? rect = semanticsObject.rect;
    if (rect == null) {
      printWarning('Warning! the rect attribute of semanticsObject is null');
      return;
    }
    if (semanticsObject.isVerticalScrollContainer) {
      // Place the _scrollOverflowElement at the end of the content and
      // make sure that when we neutralize the scrolling position,
      // it doesn't scroll into the visible area.
      final int verticalOffset = rect.height.ceil() + canonicalNeutralScrollPosition;
      _scrollOverflowElement.style
        ..transform = 'translate(0px,${verticalOffset}px)'
        ..width = '${rect.width.round()}px'
        ..height = '${canonicalNeutralScrollPosition}px';

      element.scrollTop = canonicalNeutralScrollPosition.toDouble();
      // Read back because the effective value depends on the amount of content.
      _effectiveNeutralScrollPosition = element.scrollTop.toInt();
      semanticsObject
        ..verticalContainerAdjustment = _effectiveNeutralScrollPosition.toDouble()
        ..horizontalContainerAdjustment = 0.0;
    } else {
      // Place the _scrollOverflowElement at the end of the content and
      // make sure that when we neutralize the scrolling position,
      // it doesn't scroll into the visible area.
      final int horizontalOffset = rect.width.ceil() + canonicalNeutralScrollPosition;
      _scrollOverflowElement.style
        ..transform = 'translate(${horizontalOffset}px,0px)'
        ..width = '${canonicalNeutralScrollPosition}px'
        ..height = '${rect.height.round()}px';

      element.scrollLeft = canonicalNeutralScrollPosition.toDouble();
      // Read back because the effective value depends on the amount of content.
      _effectiveNeutralScrollPosition = element.scrollLeft.toInt();
      semanticsObject
        ..verticalContainerAdjustment = 0.0
        ..horizontalContainerAdjustment = _effectiveNeutralScrollPosition.toDouble();
    }
  }

  void _gestureModeDidChange() {
    switch (EngineSemantics.instance.gestureMode) {
      case GestureMode.browserGestures:
        // overflow:scroll will cause the browser report "scroll" events when
        // the accessibility focus shifts outside the visible bounds.
        //
        // Note that on Android overflow:hidden also works. However, we prefer
        // "scroll" because it works both on Android and iOS.
        if (semanticsObject.isVerticalScrollContainer) {
          element.style.overflowY = 'scroll';
        } else {
          assert(semanticsObject.isHorizontalScrollContainer);
          element.style.overflowX = 'scroll';
        }
      case GestureMode.pointerEvents:
        // We use "hidden" instead of "scroll" so that the browser does
        // not "steal" pointer events. Flutter gesture recognizers need
        // all pointer events in order to recognize gestures correctly.
        if (semanticsObject.isVerticalScrollContainer) {
          element.style.overflowY = 'hidden';
        } else {
          assert(semanticsObject.isHorizontalScrollContainer);
          element.style.overflowX = 'hidden';
        }
    }
  }

  @override
  void dispose() {
    super.dispose();
    final DomCSSStyleDeclaration style = element.style;
    assert(_gestureModeListener != null);
    style.removeProperty('overflowY');
    style.removeProperty('overflowX');
    style.removeProperty('touch-action');
    if (_scrollListener != null) {
      removeEventListener('scroll', _scrollListener);
      _scrollListener = null;
    }
    if (_gestureModeListener != null) {
      EngineSemantics.instance.removeGestureModeListener(_gestureModeListener!);
      _gestureModeListener = null;
    }
  }

  @override
  bool focusAsRouteDefault() => focusable?.focusAsRouteDefault() ?? false;
}
