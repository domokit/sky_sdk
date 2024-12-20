// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_DL_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_DL_H_

#include "flutter/display_list/display_list.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"

namespace impeller::interop {

class DisplayList final
    : public Object<DisplayList,
                    IMPELLER_INTERNAL_HANDLE_NAME(ImpellerDisplayList)> {
 public:
  explicit DisplayList(sk_sp<flutter::DisplayList> display_list);

  ~DisplayList() override;

  DisplayList(const DisplayList&) = delete;

  DisplayList& operator=(const DisplayList&) = delete;

  bool IsValid() const;

  const sk_sp<flutter::DisplayList> GetDisplayList() const;

 private:
  sk_sp<flutter::DisplayList> display_list_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_DL_H_
