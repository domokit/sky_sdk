// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/memory/thread_checker.h"

namespace fml {

thread_local bool ThreadChecker::disable_next_failure_ = false;

}
