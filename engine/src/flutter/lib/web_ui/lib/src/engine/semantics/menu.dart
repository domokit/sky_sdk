// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'label_and_value.dart';
import 'semantics.dart';

/// Indicates a combo box element.
///
/// Uses aria combobox role to convey this semantic information to the element.
///
/// Screen-readers takes advantage of "aria-label" to describe the visual.
class SemanticComboBox extends SemanticRole {
  SemanticComboBox(SemanticsObject semanticsObject)
    : super.withBasics(
        EngineSemanticsRole.comboBox,
        semanticsObject,
        preferredLabelRepresentation: LabelRepresentation.ariaLabel,
      ) {
    setAriaRole('combobox');
    setAttribute('aria-expanded', 'false');
  }

  @override
  void update() {
    super.update();
    if (semanticsObject.isExpanded) {
      setAttribute('aria-expanded', 'true');
    } else {
      setAttribute('aria-expanded', 'false');
    }
  }

  @override
  bool focusAsRouteDefault() => focusable?.focusAsRouteDefault() ?? false;
}
