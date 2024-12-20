// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOWSX_SHIM_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOWSX_SHIM_H_

// The Win32 platform header <windowsx.h> contains some macros for
// common function names. To work around that, windowsx.h is not to be
// included directly, and instead this file should be included. If one
// of the removed Win32 macros is wanted, use the expanded form
// manually instead.

#ifdef _INC_WINDOWS_X
#error "There is an include of windowsx.h in the code. Use windowsx_shim.h"
#endif  // _INC_WINDOWS_X

#include <windowsx.h>

#undef GetNextSibling  // Same as GetWindow(hwnd, GW_HWNDNEXT)
#undef GetFirstChild   // Same as GetTopWindow(hwnd)
#undef IsMaximized     // Defined to IsZoomed, use IsZoomed directly instead
#undef IsMinimized     // Defined to IsIconic, use IsIconic directly instead
#undef IsRestored      // Macro to check that neither WS_MINIMIZE, nor
                       // WS_MAXIMIZE is set in the GetWindowStyle return
                       // value.

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOWSX_SHIM_H_
