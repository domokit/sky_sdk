// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_COMMON_MACROS_H_
#define FLUTTER_COMMON_MACROS_H_

#if SLIMPELLER

#define ONLY_IN_SLIMPELLER(code) code
#define NOT_SLIMPELLER(code)

#else  // SLIMPELLER

#define ONLY_IN_SLIMPELLER(code)
#define NOT_SLIMPELLER(code) code

#endif  // SLIMPELLER

#endif  // FLUTTER_COMMON_MACROS_H_
