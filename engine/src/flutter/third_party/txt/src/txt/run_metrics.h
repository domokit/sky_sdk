/*
 * Copyright 2017 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LIB_TXT_SRC_RUN_METRICS_H_
#define LIB_TXT_SRC_RUN_METRICS_H_

#include "text_style.h"
#include "third_party/skia/include/core/SkFontMetrics.h"

namespace txt {

// Contains the font metrics and TextStyle of a unique run.
class RunMetrics {
 public:
  explicit RunMetrics(const TextStyle* style) : text_style(style) {}

  RunMetrics(const TextStyle* style, const SkFontMetrics& metrics)
      : text_style(style), font_metrics(metrics) {}

  const TextStyle* text_style;

  // SkFontMetrics contains the following metrics:
  //
  // * Top                 distance to reserve above baseline
  // * Ascent              distance to reserve below baseline
  // * Descent             extent below baseline
  // * Bottom              extent below baseline
  // * Leading             distance to add between lines
  // * AvgCharWidth        average character width
  // * MaxCharWidth        maximum character width
  // * XMin                minimum x
  // * XMax                maximum x
  // * XHeight             height of lower-case 'x'
  // * CapHeight           height of an upper-case letter
  // * UnderlineThickness  underline thickness
  // * UnderlinePosition   underline position relative to baseline
  // * StrikeoutThickness  strikeout thickness
  // * StrikeoutPosition   strikeout position relative to baseline
  SkFontMetrics font_metrics;
};

}  // namespace txt

#endif  // LIB_TXT_SRC_RUN_METRICS_H_
