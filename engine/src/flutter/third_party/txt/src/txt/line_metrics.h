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

#ifndef LIB_TXT_SRC_LINE_METRICS_H_
#define LIB_TXT_SRC_LINE_METRICS_H_

#include <map>
#include <vector>

#include "run_metrics.h"

namespace txt {

class LineMetrics {
 public:
  // The following fields are used in the layout process itself.

  // The indexes in the text buffer the line begins and ends.
  size_t start_index = 0;
  size_t end_index = 0;
  size_t end_excluding_whitespace = 0;
  size_t end_including_newline = 0;
  bool hard_break = false;

  // The following fields are tracked after or during layout to provide to
  // the user as well as for computing bounding boxes.

  // The final computed ascent and descent for the line. This can be impacted by
  // the strut, height, scaling, as well as outlying runs that are very tall.
  //
  // The top edge is `baseline - ascent` and the bottom edge is `baseline +
  // descent`. Ascent and descent are provided as positive numbers. Raw numbers
  // for specific runs of text can be obtained in run_metrics_map. These values
  // are the cumulative metrics for the entire line.
  double ascent = 0.0;
  double descent = 0.0;
  double unscaled_ascent = 0.0;
  // Total height of the paragraph including the current line.
  //
  // The height of the current line is `round(ascent + descent)`.
  double height = 0.0;
  // Width of the line.
  double width = 0.0;
  // The left edge of the line. The right edge can be obtained with `left +
  // width`
  double left = 0.0;
  // The y position of the baseline for this line from the top of the paragraph.
  double baseline = 0.0;
  // Zero indexed line number.
  size_t line_number = 0;

  // Mapping between text index ranges and the FontMetrics associated with
  // them. The first run will be keyed under start_index. The metrics here
  // are before layout and are the base values we calculate from.
  std::map<size_t, RunMetrics> run_metrics;

  LineMetrics();

  LineMetrics(size_t start,
              size_t end,
              size_t end_excluding_whitespace,
              size_t end_including_newline,
              bool hard_break)
      : start_index(start),
        end_index(end),
        end_excluding_whitespace(end_excluding_whitespace),
        end_including_newline(end_including_newline),
        hard_break(hard_break) {}
};

}  // namespace txt

#endif  // LIB_TXT_SRC_LINE_METRICS_H_
