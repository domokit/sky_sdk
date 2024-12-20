// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/glvk/proc_table.h"

#include "impeller/base/validation.h"

namespace impeller::glvk {

ProcTable::ProcTable(const Resolver& resolver) {
  if (!resolver) {
    return;
  }

  auto error_fn = reinterpret_cast<PFNGLGETERRORPROC>(resolver("glGetError"));
  if (!error_fn) {
    VALIDATION_LOG << "Could not resolve " << "glGetError";
    return;
  }

#define GLVK_PROC(proc_ivar)                                    \
  if (auto fn_ptr = resolver(proc_ivar.name)) {                 \
    proc_ivar.function =                                        \
        reinterpret_cast<decltype(proc_ivar.function)>(fn_ptr); \
    proc_ivar.error_fn = error_fn;                              \
  } else {                                                      \
    VALIDATION_LOG << "Could not resolve " << proc_ivar.name;   \
    return;                                                     \
  }

  FOR_EACH_GLVK_PROC(GLVK_PROC);

#undef GLVK_PROC

  is_valid_ = true;
}

ProcTable::~ProcTable() = default;

bool ProcTable::IsValid() const {
  return is_valid_;
}

}  // namespace impeller::glvk
