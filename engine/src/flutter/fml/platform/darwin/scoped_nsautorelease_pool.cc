// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/darwin/scoped_nsautorelease_pool.h"

#include <objc/message.h>
#include <objc/runtime.h>

namespace {
typedef id (*msg_send)(void*, SEL);
}  // anonymous namespace

namespace fml {

ScopedNSAutoreleasePool::ScopedNSAutoreleasePool() {
  autorelease_pool_ = reinterpret_cast<msg_send>(objc_msgSend)(
      objc_getClass("NSAutoreleasePool"), sel_getUid("new"));
}

ScopedNSAutoreleasePool::~ScopedNSAutoreleasePool() {
  reinterpret_cast<msg_send>(objc_msgSend)(autorelease_pool_,
                                           sel_getUid("drain"));
}

}  // namespace fml
