// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_CONTEXT_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_CONTEXT_H_

#include <functional>

#include "impeller/display_list/aiks_context.h"
#include "impeller/renderer/context.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"

namespace impeller::interop {

class Context final
    : public Object<Context, IMPELLER_INTERNAL_HANDLE_NAME(ImpellerContext)> {
 public:
  class BackendData;

  static ScopedObject<Context> CreateOpenGLES(
      std::function<void*(const char* gl_proc_name)> proc_address_callback);

  explicit Context(std::shared_ptr<impeller::Context> context,
                   std::shared_ptr<BackendData> backend_data);

  ~Context() override;

  Context(const Context&) = delete;

  Context& operator=(const Context&) = delete;

  bool IsValid() const;

  std::shared_ptr<impeller::Context> GetContext() const;

  AiksContext& GetAiksContext();

 private:
  impeller::AiksContext context_;
  std::shared_ptr<BackendData> backend_data_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_CONTEXT_H_
