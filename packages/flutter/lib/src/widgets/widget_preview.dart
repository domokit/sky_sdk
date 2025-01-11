// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

/// Annotation used to mark functions that return widget previews.
///
/// {@tool snippet}
///
/// Functions annotated with `@Preview()` should return a `List<WidgetPreview>`
/// and be public, top-level functions.
///
/// ```dart
/// @Preview()
/// List<WidgetPreview> myFirstPreview() {
///   return <WidgetPreview>[
///     WidgetPreview(
///       name: 'Preview 1',
///       child: const Text('Foo'),
///     ),
///     WidgetPreview(
///       name: 'Preview 2',
///       child: MyWidget(),
///     ),
///   ];
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
  final String? name;

  /// The [Widget] to be rendered in the preview.
  final Widget child;

  /// Artificial width constraint to be applied to the [child].
  final double? width;

  /// Artificial height constraint to be applied to the [child].
  final double? height;

  /// Applies font scaling to text within the [child].
  final double? textScaleFactor;
}
