// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef ACCESSIBILITY_BASE_MACROS_H_
#define ACCESSIBILITY_BASE_MACROS_H_

#define BASE_DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&) = delete;           \
  TypeName& operator=(const TypeName&) = delete

#endif  // ACCESSIBILITY_BASE_MACROS_H_
