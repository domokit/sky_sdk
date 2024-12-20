// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GEOMETRY_TYPE_TRAITS_H_
#define FLUTTER_IMPELLER_GEOMETRY_TYPE_TRAITS_H_

#include <type_traits>

namespace impeller {

template <class F,
          class I,
          class = std::enable_if_t<std::is_floating_point_v<F> &&
                                   std::is_integral_v<I>>>
struct MixedOp_ : public std::true_type {};

template <class F, class I>
using MixedOp = typename MixedOp_<F, I>::type;

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_GEOMETRY_TYPE_TRAITS_H_
