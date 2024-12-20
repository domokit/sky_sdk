// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_CONTEXT_OPTIONS_H_
#define FLUTTER_SHELL_COMMON_CONTEXT_OPTIONS_H_

#if !SLIMPELLER

#include <optional>

#include "flutter/fml/macros.h"
#include "third_party/skia/include/gpu/ganesh/GrContextOptions.h"

namespace flutter {

enum class ContextType {
  /// The context is used to render to a texture or renderbuffer.
  kRender,
  /// The context will only be used to transfer resources to and from device
  /// memory. No rendering will be performed using this context.
  kResource,
};

//------------------------------------------------------------------------------
/// @brief      Initializes GrContextOptions with values suitable for Flutter.
///             The options can be further tweaked before a GrContext is created
///             from these options.
///
/// @param[in]  type  The type of context that will be created using these
///                   options.
/// @param[in]  type  The client rendering API that will be wrapped using a
///                   context with these options. This argument is only required
///                   if the context is going to be used with a particular
///                   client rendering API.
///
/// @return     The default graphics context options.
///
GrContextOptions MakeDefaultContextOptions(
    ContextType type,
    std::optional<GrBackendApi> api = std::nullopt);

}  // namespace flutter

#endif  //  !SLIMPELLER

#endif  // FLUTTER_SHELL_COMMON_CONTEXT_OPTIONS_H_
