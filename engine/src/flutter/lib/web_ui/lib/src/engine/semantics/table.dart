// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'label_and_value.dart';
import 'semantics.dart';

/// Indicates a table element.
///
/// Uses aria table role to convey this semantic information to the element.
///
/// Screen-readers takes advantage of "aria-label" to describe the visual.
class SemanticTable extends SemanticRole {
  SemanticTable(SemanticsObject semanticsObject)
    : super.withBasics(
        EngineSemanticsRole.table,
        semanticsObject,
        preferredLabelRepresentation: LabelRepresentation.ariaLabel,
      ) {
    setAriaRole('table');
  }

  @override
  bool focusAsRouteDefault() => focusable?.focusAsRouteDefault() ?? false;
}

/// Indicates a table cell element.
///
/// Uses aria cell role to convey this semantic information to the element.
///
/// Screen-readers takes advantage of "aria-label" to describe the visual.
class SemanticCell extends SemanticRole {
  SemanticCell(SemanticsObject semanticsObject)
    : super.withBasics(
        EngineSemanticsRole.cell,
        semanticsObject,
        preferredLabelRepresentation: LabelRepresentation.ariaLabel,
      ) {
    setAriaRole('cell');
  }

  @override
  bool focusAsRouteDefault() => focusable?.focusAsRouteDefault() ?? false;
}

/// Indicates a table row element.
///
/// Uses aria row role to convey this semantic information to the element.
///
/// Screen-readers takes advantage of "aria-label" to describe the visual.
class SemanticRow extends SemanticRole {
  SemanticRow(SemanticsObject semanticsObject)
    : super.withBasics(
        EngineSemanticsRole.row,
        semanticsObject,
        preferredLabelRepresentation: LabelRepresentation.ariaLabel,
      ) {
    setAriaRole('row');
  }

  @override
  bool focusAsRouteDefault() => focusable?.focusAsRouteDefault() ?? false;
}

/// Indicates a table column header element.
///
/// Uses aria columnheader role to convey this semantic information to the element.
///
/// Screen-readers takes advantage of "aria-label" to describe the visual.
class SemanticColumnHeader extends SemanticRole {
  SemanticColumnHeader(SemanticsObject semanticsObject)
    : super.withBasics(
        EngineSemanticsRole.columnHeader,
        semanticsObject,
        preferredLabelRepresentation: LabelRepresentation.ariaLabel,
      ) {
    setAriaRole('columnheader');
  }

  @override
  bool focusAsRouteDefault() => focusable?.focusAsRouteDefault() ?? false;
}
