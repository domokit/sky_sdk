// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

import 'material.dart';
import 'material_localizations.dart';
import 'scaffold.dart' show Scaffold, ScaffoldMessenger;

// Examples can assume:
// late BuildContext context;

/// Asserts that the given context has a [Material] ancestor within the closest
/// [LookupBoundary].
///
/// Used by many Material Design widgets to make sure that they are
/// only used in contexts where they can print ink onto some material.
///
/// To call this function, use the following pattern, typically in the
/// relevant Widget's build method:
///
/// ```dart
/// assert(debugCheckHasMaterial(context));
/// ```
///
/// Always place this before any early returns, so that the invariant is checked
/// in all cases. This prevents bugs from hiding until a particular codepath is
/// hit.
///
/// This method can be expensive (it walks the element tree).
///
/// Does nothing if asserts are disabled. Always returns true.
bool debugCheckHasMaterial(BuildContext context) {
  assert(() {
    if (LookupBoundary.findAncestorWidgetOfExactType<Material>(context) == null) {
      final bool hiddenByBoundary = LookupBoundary.debugIsHidingAncestorWidgetOfExactType<Material>(
        context,
      );
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary(
          'No Material widget found${hiddenByBoundary ? ' within the closest LookupBoundary' : ''}.',
        ),
        if (hiddenByBoundary)
          ErrorDescription(
            'There is an ancestor Material widget, but it is hidden by a LookupBoundary.',
          ),
        ErrorDescription(
          '${context.widget.runtimeType} widgets require a Material '
          'widget ancestor within the closest LookupBoundary.\n'
          'In Material Design, most widgets are conceptually "printed" on '
          "a sheet of material. In Flutter's material library, that "
          'material is represented by the Material widget. It is the '
          'Material widget that renders ink splashes, for instance. '
          'Because of this, many material library widgets require that '
          'there be a Material widget in the tree above them.',
        ),
        ErrorHint(
          'To introduce a Material widget, you can either directly '
          'include one, or use a widget that contains Material itself, '
          'such as a Card, Dialog, Drawer, or Scaffold.',
        ),
        ...context.describeMissingAncestor(expectedAncestorType: Material),
      ]);
    }
    return true;
  }());
  return true;
}

/// Asserts that the given context has a [Localizations] ancestor that contains
/// a [MaterialLocalizations] delegate.
///
/// Used by many Material Design widgets to make sure that they are
/// only used in contexts where they have access to localizations.
///
/// To call this function, use the following pattern, typically in the
/// relevant Widget's build method:
///
/// ```dart
/// assert(debugCheckHasMaterialLocalizations(context));
/// ```
///
/// Always place this before any early returns, so that the invariant is checked
/// in all cases. This prevents bugs from hiding until a particular codepath is
/// hit.
///
/// This function has the side-effect of establishing an inheritance
/// relationship with the nearest [Localizations] widget (see
/// [BuildContext.dependOnInheritedWidgetOfExactType]). This is ok if the caller
/// always also calls [Localizations.of] or [Localizations.localeOf].
///
/// Does nothing if asserts are disabled. Always returns true.
bool debugCheckHasMaterialLocalizations(BuildContext context) {
  assert(() {
    if (Localizations.of<MaterialLocalizations>(context, MaterialLocalizations) == null) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('No MaterialLocalizations found.'),
        ErrorDescription(
          '${context.widget.runtimeType} widgets require MaterialLocalizations '
          'to be provided by a Localizations widget ancestor.',
        ),
        ErrorDescription(
          'The material library uses Localizations to generate messages, '
          'labels, and abbreviations.',
        ),
        ErrorHint(
          'To introduce a MaterialLocalizations, either use a '
          'MaterialApp at the root of your application to include them '
          'automatically, or add a Localization widget with a '
          'MaterialLocalizations delegate.',
        ),
        ...context.describeMissingAncestor(expectedAncestorType: MaterialLocalizations),
      ]);
    }
    return true;
  }());
  return true;
}

/// Asserts that the given context has a [Scaffold] ancestor.
///
/// Used by various widgets to make sure that they are only used in an
/// appropriate context.
///
/// To invoke this function, use the following pattern, typically in the
/// relevant Widget's build method:
///
/// ```dart
/// assert(debugCheckHasScaffold(context));
/// ```
///
/// Always place this before any early returns, so that the invariant is checked
/// in all cases. This prevents bugs from hiding until a particular codepath is
/// hit.
///
/// This method can be expensive (it walks the element tree).
///
/// Does nothing if asserts are disabled. Always returns true.
bool debugCheckHasScaffold(BuildContext context) {
  assert(() {
    if (context.widget is! Scaffold && context.findAncestorWidgetOfExactType<Scaffold>() == null) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('No Scaffold widget found.'),
        ErrorDescription(
          '${context.widget.runtimeType} widgets require a Scaffold widget ancestor.',
        ),
        ...context.describeMissingAncestor(expectedAncestorType: Scaffold),
        ErrorHint(
          'Typically, the Scaffold widget is introduced by the MaterialApp or '
          'WidgetsApp widget at the top of your application widget tree.',
        ),
      ]);
    }
    return true;
  }());
  return true;
}

/// Asserts that the given context has a [ScaffoldMessenger] ancestor.
///
/// Used by various widgets to make sure that they are only used in an
/// appropriate context.
///
/// To invoke this function, use the following pattern, typically in the
/// relevant Widget's build method:
///
/// ```dart
/// assert(debugCheckHasScaffoldMessenger(context));
/// ```
///
/// Always place this before any early returns, so that the invariant is checked
/// in all cases. This prevents bugs from hiding until a particular codepath is
/// hit.
///
/// This method can be expensive (it walks the element tree).
///
/// Does nothing if asserts are disabled. Always returns true.
bool debugCheckHasScaffoldMessenger(BuildContext context) {
  assert(() {
    if (context.findAncestorWidgetOfExactType<ScaffoldMessenger>() == null) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('No ScaffoldMessenger widget found.'),
        ErrorDescription(
          '${context.widget.runtimeType} widgets require a ScaffoldMessenger widget ancestor.',
        ),
        ...context.describeMissingAncestor(expectedAncestorType: ScaffoldMessenger),
        ErrorHint(
          'Typically, the ScaffoldMessenger widget is introduced by the MaterialApp '
          'at the top of your application widget tree.',
        ),
      ]);
    }
    return true;
  }());
  return true;
}
