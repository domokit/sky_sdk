/*
 * Copyright 2019 Google Inc.
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

#ifndef LIB_TXT_SRC_FONT_FEATURES_H_
#define LIB_TXT_SRC_FONT_FEATURES_H_

#include <map>
#include <string>
#include <vector>

namespace txt {

// Feature tags that can be applied in a text style to control how a font
// selects glyphs.
class FontFeatures {
 public:
  void SetFeature(std::string tag, int value);

  std::string GetFeatureSettings() const;

  const std::map<std::string, int>& GetFontFeatures() const;

 private:
  std::map<std::string, int> feature_map_;
};

// Axis tags and values that can be applied in a text style to control the
// attributes of variable fonts.
class FontVariations {
 public:
  void SetAxisValue(std::string tag, float value);

  const std::map<std::string, float>& GetAxisValues() const;

 private:
  std::map<std::string, float> axis_map_;
};

}  // namespace txt

#endif  // LIB_TXT_SRC_FONT_FEATURE_H_
