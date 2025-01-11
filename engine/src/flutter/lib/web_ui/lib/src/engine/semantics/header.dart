// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../dom.dart';
import 'label_and_value.dart';
import 'semantics.dart';

/// Renders a semantic header.
///
/// A header is a group of nodes that together introduce the content of the
/// current screen or page.
///
/// Uses the `<header>` element, which implies ARIA role "banner".
///
/// See also:
///   * https://developer.mozilla.org/en-US/docs/Web/HTML/Element/header
///   * https://developer.mozilla.org/en-US/docs/Web/Accessibility/ARIA/Roles/banner_role
class SemanticHeader extends SemanticRole {
  SemanticHeader(SemanticsObject semanticsObject)
    : super.withBasics(
        EngineSemanticsRole.header,
        semanticsObject,

        // Why use sizedSpan?
        //
        // On an empty <header> aria-label alone will read the label but also add
        // "empty banner". Additionally, if the label contains information that's
        // meant to be crawlable, it will be lost by moving into aria-label, because
        // most crawlers ignore ARIA labels.
        //
        // Using DOM text, such as <header>DOM text</header> causes the browser to
        // generate two a11y nodes, one for the <header> element, and one for the
        // "DOM text" text node. The text node is sized according to the text size,
        // and does not match the size of the <header> element, which is the same
        // issue as https://github.com/flutter/flutter/issues/146774.
        preferredLabelRepresentation: LabelRepresentation.sizedSpan,
      );

  @override
  DomElement createElement() => createDomElement('header');

  @override
  bool focusAsRouteDefault() => focusable?.focusAsRouteDefault() ?? false;
}
