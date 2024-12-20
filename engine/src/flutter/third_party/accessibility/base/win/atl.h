// Copyright 2018 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_WIN_ATL_H_
#define BASE_WIN_ATL_H_

// atlwin.h relies on std::void_t, but libc++ doesn't define it unless
// _LIBCPP_STD_VER > 14.  Workaround this by manually defining it.
#include <type_traits>
#if defined(_LIBCPP_STD_VER) && _LIBCPP_STD_VER <= 14
namespace std {
template <class...>
using void_t = void;
}
#endif

#include <atlbase.h>
#include <atlcom.h>
#include <atlctl.h>
#include <atlhost.h>
#include <atlsecurity.h>
#include <atlwin.h>

#endif  // BASE_WIN_ATL_H_
