// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'framework.dart';

/// Annotation used to mark functions that return a widget preview.
///
/// NOTE: this interface is not stable and **will change**.
///
/// {@tool snippet}
///
/// Functions annotated with `@Preview()` must return a `WidgetPreview`
/// and be public, top-level functions.
///
/// ```dart
/// @Preview()
/// WidgetPreview widgetPreview() {
///   return const WidgetPreview(name: 'Preview 1', child: Text('Foo'));
/// }
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [WidgetPreview], a data class used to specify widget previews.
// TODO(bkonyi): link to actual documentation when available.
class Preview {
  /// Annotation used to mark functions that return widget previews.
  const Preview();
}

/// Wraps a [Widget], initializing various state and properties to allow for
/// previewing of the [Widget] in the widget previewer.
///
/// WARNING: This interface is not stable and **will change**.
///
/// See also:
///
///  * [Preview], an annotation class used to mark functions returning widget
///    previews.
// TODO(bkonyi): link to actual documentation when available.
class WidgetPreview {
  /// Wraps [child] in a [WidgetPreview] instance that applies some set of
  /// properties.
  const WidgetPreview({
    required this.child,
    this.name,
    this.width,
    this.height,
    this.textScaleFactor,
  });

  /// A description to be displayed alongside the preview.
  ///
  /// If not provided, no name will be associated with the preview.
  final String? name;

  /// The [Widget] to be rendered in the preview.
  final Widget child;

  /// Artificial width constraint to be applied to the [child].
  ///
  /// If not provided, the previewed widget will attempt to set its own width
  /// constraints and may result in an unbounded constraint error.
  final double? width;

  /// Artificial height constraint to be applied to the [child].
  ///
  /// If not provided, the previewed widget will attempt to set its own height
  /// constraints and may result in an unbounded constraint error.
  final double? height;

  /// Applies font scaling to text within the [child].
  ///
  /// If not provided, the default text scaling factor provided by [MediaQuery]
  /// will be used.
  final double? textScaleFactor;
}
