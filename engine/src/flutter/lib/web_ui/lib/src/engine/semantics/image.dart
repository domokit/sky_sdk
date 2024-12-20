// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../dom.dart';
import 'semantics.dart';

/// Represents semantic objects that deliver information in a visual manner.
///
/// Uses aria img role to convey this semantic information to the element.
///
/// Screen-readers takes advantage of "aria-label" to describe the visual.
class SemanticImage extends SemanticRole {
  SemanticImage(SemanticsObject semanticsObject)
    : super.blank(SemanticRoleKind.image, semanticsObject) {
    // The following behaviors can coexist with images. `LabelAndValue` is
    // not used because this behavior uses special auxiliary elements to
    // supply ARIA labels.
    // TODO(yjbanov): reevaluate usage of aux elements, https://github.com/flutter/flutter/issues/129317
    addFocusManagement();
    addLiveRegion();
    addRouteName();
    addTappable();
    addSelectableBehavior();
  }

  @override
  bool focusAsRouteDefault() => focusable?.focusAsRouteDefault() ?? false;

  /// The element with role="img" and aria-label could block access to all
  /// children elements, therefore create an auxiliary element and  describe the
  /// image in that if the semantic object have child nodes.
  DomElement? _auxiliaryImageElement;

  @override
  void update() {
    super.update();

    if (semanticsObject.isVisualOnly && semanticsObject.hasChildren) {
      if (_auxiliaryImageElement == null) {
        _auxiliaryImageElement = domDocument.createElement('flt-semantics-img');
        // Absolute positioning and sizing of leaf text elements confuses
        // VoiceOver. So we let the browser size the value node. The node will
        // still have a bigger tap area. However, if the node is a parent to
        // other nodes, then VoiceOver behaves as expected with absolute
        // positioning and sizing.
        if (semanticsObject.hasChildren) {
          _auxiliaryImageElement!.style
            ..position = 'absolute'
            ..top = '0'
            ..left = '0'
            ..width = '${semanticsObject.rect!.width}px'
            ..height = '${semanticsObject.rect!.height}px';
        }
        _auxiliaryImageElement!.style.fontSize = '6px';
        append(_auxiliaryImageElement!);
      }

      _auxiliaryImageElement!.setAttribute('role', 'img');
      _setLabel(_auxiliaryImageElement);
    } else if (semanticsObject.isVisualOnly) {
      setAriaRole('img');
      _setLabel(element);
      _cleanUpAuxiliaryElement();
    } else {
      _cleanUpAuxiliaryElement();
      _cleanupElement();
    }
  }

  void _setLabel(DomElement? element) {
    if (semanticsObject.hasLabel) {
      element!.setAttribute('aria-label', semanticsObject.label!);
    }
  }

  void _cleanUpAuxiliaryElement() {
    if (_auxiliaryImageElement != null) {
      _auxiliaryImageElement!.remove();
      _auxiliaryImageElement = null;
    }
  }

  void _cleanupElement() {
    removeAttribute('aria-label');
  }

  @override
  void dispose() {
    super.dispose();
    _cleanUpAuxiliaryElement();
    _cleanupElement();
  }
}
