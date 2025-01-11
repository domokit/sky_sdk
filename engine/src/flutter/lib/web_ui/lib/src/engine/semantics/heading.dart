// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../dom.dart';
import 'label_and_value.dart';
import 'semantics.dart';

/// Renders semantics objects as headings with the corresponding
/// level (h1 ... h6).
class SemanticHeading extends SemanticRole {
  SemanticHeading(SemanticsObject semanticsObject)
    : super.blank(EngineSemanticsRole.heading, semanticsObject) {
    addFocusManagement();
    addLiveRegion();
    addRouteName();
    addLabelAndValue(preferredRepresentation: LabelRepresentation.domText);
    addSelectableBehavior();
  }

  @override
  DomElement createElement() {
    final element = createDomElement('h${semanticsObject.effectiveHeadingLevel}');
    element.style
      // Browser adds default non-zero margins/paddings to <h*> tags, which
      // affects the size of the element. As the element size is fully defined
      // by semanticsObject.rect, the extra margins/paddings must be zeroed out.
      ..margin = '0'
      ..padding = '0'
      // The 10px size was picked empirically. By default the browser will scale
      // the font size based on the heading level. Font size should not be
      // important in semantics since rendering is done via the render tree.
      // Speculatively locking the font size to something not too big and not
      // too small, which will hopefully satisfy whoever is consuming the DOM
      // tree, be it a screen reader or a web crawler. However, if there's a
      // good reason to do otherwise, feel free to revise this code.
      ..fontSize = '10px';
    return element;
  }

  /// Focuses on this heading element if it turns out to be the default element
  /// of a route.
  ///
  /// Normally, heading elements are not focusable as they do not receive
  /// keyboard input. However, when a route is pushed (e.g. a dialog pops up),
  /// then it may be desirable to move the screen reader focus to the heading
  /// that explains the contents of the route to the user. This method makes the
  /// element artificially focusable and moves the screen reader focus to it.
  ///
  /// That said, if the node is formally focusable, then the focus is
  /// transferred using [Focusable].
  @override
  bool focusAsRouteDefault() {
    if (semanticsObject.isFocusable) {
      final focusable = this.focusable;
      if (focusable != null) {
        return focusable.focusAsRouteDefault();
      }
    }

    labelAndValue!.focusAsRouteDefault();
    return true;
  }
}
