/*
 * Copyright 2019 Google, Inc.
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

#include "font_features.h"

#include <sstream>

namespace txt {

void FontFeatures::SetFeature(std::string tag, int value) {
  feature_map_[tag] = value;
}

std::string FontFeatures::GetFeatureSettings() const {
  if (feature_map_.empty())
    return "";

  std::ostringstream stream;

  for (const auto& kv : feature_map_) {
    if (stream.tellp()) {
      stream << ',';
    }
    stream << kv.first << '=' << kv.second;
  }

  return stream.str();
}

const std::map<std::string, int>& FontFeatures::GetFontFeatures() const {
  return feature_map_;
}

void FontVariations::SetAxisValue(std::string tag, float value) {
  axis_map_[tag] = value;
}

const std::map<std::string, float>& FontVariations::GetAxisValues() const {
  return axis_map_;
}

}  // namespace txt
