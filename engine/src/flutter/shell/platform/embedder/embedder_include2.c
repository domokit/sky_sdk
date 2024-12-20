// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/embedder.h"

// This file is the same as embedder_include.c and ensures that static methods
// don't end up in public API header. This will cause duplicate symbols when the
// header in imported in the multiple translation units in the embedder.
