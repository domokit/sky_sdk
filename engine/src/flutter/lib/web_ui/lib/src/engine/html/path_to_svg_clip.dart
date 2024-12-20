// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../dom.dart';
import '../svg.dart';
import 'path/path.dart';
import 'path/path_to_svg.dart';

/// Counter used for generating clip path id inside an svg <defs> tag.
int _clipIdCounter = 0;

/// Used for clipping and filter svg resources.
///
/// Position needs to be absolute since these svgs are sandwiched between
/// canvas elements and can cause layout shifts otherwise.
final SVGSVGElement kSvgResourceHeader =
    createSVGSVGElement()
      ..setAttribute('width', 0)
      ..setAttribute('height', 0)
      ..style.position = 'absolute';

/// Converts Path to svg element that contains a clip-path definition.
///
/// Calling this method updates [_clipIdCounter]. The HTML id of the generated
/// clip is set to "svgClip${_clipIdCounter}", e.g. "svgClip123".
SVGSVGElement pathToSvgClipPath(
  ui.Path path, {
  double offsetX = 0,
  double offsetY = 0,
  double scaleX = 1.0,
  double scaleY = 1.0,
}) {
  _clipIdCounter += 1;
  final SVGSVGElement root = kSvgResourceHeader.cloneNode(false) as SVGSVGElement;
  final SVGDefsElement defs = createSVGDefsElement();
  root.append(defs);

  final String clipId = 'svgClip$_clipIdCounter';
  final SVGClipPathElement clipPath = createSVGClipPathElement();
  defs.append(clipPath);
  clipPath.id = clipId;

  final SVGPathElement svgPath = createSVGPathElement();
  clipPath.append(svgPath);
  svgPath.setAttribute('fill', '#FFFFFF');

  // Firefox objectBoundingBox fails to scale to 1x1 units, instead use
  // no clipPathUnits but write the path in target units.
  if (ui_web.browser.browserEngine != ui_web.BrowserEngine.firefox) {
    clipPath.setAttribute('clipPathUnits', 'objectBoundingBox');
    svgPath.setAttribute('transform', 'scale($scaleX, $scaleY)');
  }
  if (path.fillType == ui.PathFillType.evenOdd) {
    svgPath.setAttribute('clip-rule', 'evenodd');
  } else {
    svgPath.setAttribute('clip-rule', 'nonzero');
  }
  svgPath.setAttribute(
    'd',
    pathToSvg((path as SurfacePath).pathRef, offsetX: offsetX, offsetY: offsetY),
  );
  return root;
}

String createSvgClipUrl() => 'url(#svgClip$_clipIdCounter)';

/// Resets clip ids. Used for testing by [debugForgetFrameScene] API.
void resetSvgClipIds() {
  _clipIdCounter = 0;
}
