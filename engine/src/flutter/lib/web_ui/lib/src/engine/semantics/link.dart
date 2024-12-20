// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../dom.dart';
import '../semantics.dart';

/// Provides accessibility for links.
class SemanticLink extends SemanticRole {
  SemanticLink(SemanticsObject semanticsObject)
    : super.withBasics(
        SemanticRoleKind.link,
        semanticsObject,
        preferredLabelRepresentation: LabelRepresentation.domText,
      ) {
    addTappable();
  }

  @override
  DomElement createElement() {
    final DomElement element = domDocument.createElement('a');
    element.style.display = 'block';
    return element;
  }

  @override
  void update() {
    super.update();

    if (semanticsObject.isLinkUrlDirty) {
      if (semanticsObject.hasLinkUrl) {
        element.setAttribute('href', semanticsObject.linkUrl!);
      } else {
        element.removeAttribute('href');
      }
    }
  }

  @override
  bool focusAsRouteDefault() => focusable?.focusAsRouteDefault() ?? false;
}
