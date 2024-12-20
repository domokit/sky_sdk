// Copyright (c) 2011 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_BASE_WIN_ATL_MODULE_H_
#define UI_BASE_WIN_ATL_MODULE_H_

namespace ui {
namespace win {

// Ensure that we have exactly one ATL module registered. It's safe to
// call this more than once. ATL functions will crash if there's no
// ATL module registered, or if you try to register two of them, so
// dynamically registering one if needed makes it much easier for us
// to support different build configurations like multi-dll without
// worrying about which side of a module boundary each ATL module object
// belongs on.
//
// This function must be implemented in this header file rather than a
// source file so that it's inlined into the module where it's included,
// rather than in the "ui" module.
inline void CreateATLModuleIfNeeded() {
  if (_pAtlModule == NULL) {
    // This creates the module and automatically updates _pAtlModule.
    new CComModule;
  }
}

}  // namespace win
}  // namespace ui

#endif  // UI_BASE_WIN_ATL_MODULE_H_
