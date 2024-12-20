// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_CONTAINER_H_
#define FLUTTER_FML_CONTAINER_H_

#include <functional>
#include <map>

namespace fml {

template <
    class Collection =
        std::unordered_map<class Key, class Value, class Hash, class Equal>>
void erase_if(Collection& container,
              std::function<bool(typename Collection::iterator)> predicate) {
  auto it = container.begin();
  while (it != container.end()) {
    if (predicate(it)) {
      it = container.erase(it);
      continue;
    }
    it++;
  }
}

}  // namespace fml

#endif  // FLUTTER_FML_CONTAINER_H_
