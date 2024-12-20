// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// The Flutter painting library.
///
/// To use, import `package:flutter/painting.dart`.
///
/// This library includes a variety of classes that wrap the Flutter
/// engine's painting API for more specialized purposes, such as painting scaled
/// images, interpolating between shadows, painting borders around boxes, etc.
///
/// In particular:
///
///  * Use the [TextPainter] class for painting text.
///  * Use [Decoration] (and more concretely [BoxDecoration]) for
///    painting boxes.
///
/// @docImport 'src/painting/box_decoration.dart';
/// @docImport 'src/painting/decoration.dart';
/// @docImport 'src/painting/text_painter.dart';
library painting;

export 'dart:ui'
    show PlaceholderAlignment, Shadow, TextHeightBehavior, TextLeadingDistribution, kTextHeightNone;

export 'src/painting/alignment.dart';
export 'src/painting/basic_types.dart';
export 'src/painting/beveled_rectangle_border.dart';
export 'src/painting/binding.dart';
export 'src/painting/border_radius.dart';
export 'src/painting/borders.dart';
export 'src/painting/box_border.dart';
export 'src/painting/box_decoration.dart';
export 'src/painting/box_fit.dart';
export 'src/painting/box_shadow.dart';
export 'src/painting/circle_border.dart';
export 'src/painting/clip.dart';
export 'src/painting/colors.dart';
export 'src/painting/continuous_rectangle_border.dart';
export 'src/painting/debug.dart';
export 'src/painting/decoration.dart';
export 'src/painting/decoration_image.dart';
export 'src/painting/edge_insets.dart';
export 'src/painting/flutter_logo.dart';
export 'src/painting/fractional_offset.dart';
export 'src/painting/geometry.dart';
export 'src/painting/gradient.dart';
export 'src/painting/image_cache.dart';
export 'src/painting/image_decoder.dart';
export 'src/painting/image_provider.dart';
export 'src/painting/image_resolution.dart';
export 'src/painting/image_stream.dart';
export 'src/painting/inline_span.dart';
export 'src/painting/linear_border.dart';
export 'src/painting/matrix_utils.dart';
export 'src/painting/notched_shapes.dart';
export 'src/painting/oval_border.dart';
export 'src/painting/paint_utilities.dart';
export 'src/painting/placeholder_span.dart';
export 'src/painting/rounded_rectangle_border.dart';
export 'src/painting/shader_warm_up.dart';
export 'src/painting/shape_decoration.dart';
export 'src/painting/stadium_border.dart';
export 'src/painting/star_border.dart';
export 'src/painting/strut_style.dart';
export 'src/painting/text_painter.dart';
export 'src/painting/text_scaler.dart';
export 'src/painting/text_span.dart';
export 'src/painting/text_style.dart';
