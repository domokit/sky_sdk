// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:ui/src/engine/dom.dart';
import 'package:ui/src/engine/view_embedder/display_dpr_stream.dart';
import 'package:ui/src/engine/window.dart';
import 'package:ui/ui.dart' as ui show Size;

import 'custom_element_dimensions_provider.dart';
import 'full_page_dimensions_provider.dart';

/// This class provides the dimensions of the "viewport" in which the app is rendered.
///
/// Similarly to the `EmbeddingStrategy`, this class is specialized to handle
/// different sources of information:
///
/// * [FullPageDimensionsProvider] - The default behavior, uses the VisualViewport
///   API to measure, and react to, the dimensions of the full browser window.
/// * [CustomElementDimensionsProvider] - Uses a custom html Element as the source
///   of dimensions, and the ResizeObserver to notify the app of changes.
///
/// All the measurements returned from this class are potentially *expensive*,
/// and should be cached as needed. Every call to every method on this class
/// WILL perform actual DOM measurements.
abstract class DimensionsProvider {
  DimensionsProvider();

  /// Creates the appropriate DimensionsProvider depending on the incoming [hostElement].
  factory DimensionsProvider.create({DomElement? hostElement}) {
    if (hostElement != null) {
      return CustomElementDimensionsProvider(
        hostElement,
        onDprChange: DisplayDprStream.instance.dprChanged,
      );
    } else {
      return FullPageDimensionsProvider();
    }
  }

  /// Returns the [ui.Size] of the "viewport".
  ///
  /// This function is expensive. It triggers browser layout if there are
  /// pending DOM writes.
  ui.Size computePhysicalSize();

  /// Returns the [ViewPadding] of the keyboard insets (if present).
  ViewPadding computeKeyboardInsets(double physicalHeight, bool isEditingOnMobile);

  /// Returns a Stream with the changes to [ui.Size] (when cheap to get).
  ///
  /// Currently this Stream always returns `null` measurements because the
  /// resize event that we use for [FullPageDimensionsProvider] does not contain
  /// the new size, so users of this Stream everywhere immediately retrieve the
  /// new `physicalSize` from the window.
  ///
  /// The [CustomElementDimensionsProvider] *could* broadcast the new size, but
  /// to keep both implementations consistent (and their consumers), for now all
  /// events from this Stream are going to be `null` (until we find a performant
  /// way to retrieve the dimensions in full-page mode).
  Stream<ui.Size?> get onResize;

  /// Whether the [DimensionsProvider] instance has been closed or not.
  @visibleForTesting
  bool isClosed = false;

  /// Clears any resources grabbed by the DimensionsProvider instance.
  ///
  /// All internal event handlers will be disconnected, and the [onResize] Stream
  /// will be closed.
  @mustCallSuper
  void close() {
    isClosed = true;
  }
}
