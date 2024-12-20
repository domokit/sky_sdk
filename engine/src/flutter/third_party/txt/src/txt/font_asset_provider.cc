/*
 * Copyright 2018 Google Inc.
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

#include <algorithm>
#include <string>

#include "txt/font_asset_provider.h"

namespace txt {

// Return a canonicalized version of a family name that is suitable for
// matching.
std::string FontAssetProvider::CanonicalFamilyName(std::string family_name) {
  std::string result(family_name.length(), 0);

  // Convert ASCII characters to lower case.
  std::transform(family_name.begin(), family_name.end(), result.begin(),
                 [](char c) { return (c & 0x80) ? c : ::tolower(c); });

  return result;
}

}  // namespace txt
