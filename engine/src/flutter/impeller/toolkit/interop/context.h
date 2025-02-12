// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_CONTEXT_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_CONTEXT_H_

#include "impeller/display_list/aiks_context.h"
#include "impeller/renderer/context.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"

namespace impeller::interop {

class Context
    : public Object<Context, IMPELLER_INTERNAL_HANDLE_NAME(ImpellerContext)> {
 public:
  ~Context() override;

  Context(const Context&) = delete;

  Context& operator=(const Context&) = delete;

  bool IsValid() const;

  std::shared_ptr<impeller::Context> GetContext() const;

  AiksContext& GetAiksContext();

  bool IsBackend(impeller::Context::BackendType type) const;

  bool IsGL() const;

  bool IsMetal() const;

  bool IsVulkan() const;

 protected:
  explicit Context(std::shared_ptr<impeller::Context> context);

 private:
  impeller::AiksContext context_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_CONTEXT_H_
