// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Precompiled header for Chromium project on Windows, not used by
// other build configurations. Using precompiled headers speeds the
// build up significantly, around 1/4th on VS 2010 on an HP Z600 with 12
// GB of memory.
//
// Numeric comments beside includes are the number of times they were
// included under src/chrome/browser on 2011/8/20, which was used as a
// baseline for deciding what to include in the PCH. Includes without
// a numeric comment are generally included at least 5 times. It may
// be possible to tweak the speed of the build by commenting out or
// removing some of the less frequently used headers.

#if defined(BUILD_PRECOMPILE_H_)
#error You shouldn't include the precompiled header file more than once.
#endif

#define BUILD_PRECOMPILE_H_

#define _USE_MATH_DEFINES

// The Windows header needs to come before almost all the other
// Windows-specific headers.
#include <Windows.h>
#include <dwmapi.h>
#include <shellapi.h>
#include <wtypes.h>  // 2

// Defines in atlbase.h cause conflicts; if we could figure out how
// this family of headers can be included in the PCH, it might speed
// up the build as several of them are used frequently.
/*
#include <atlbase.h>
#include <atlapp.h>
#include <atlcom.h>
#include <atlcrack.h>  // 2
#include <atlctrls.h>  // 2
#include <atlmisc.h>  // 2
#include <atlsafe.h>  // 1
#include <atltheme.h>  // 1
#include <atlwin.h>  // 2
*/

// Objbase.h and other files that rely on it bring in [ #define
// interface struct ] which can cause problems in a multi-platform
// build like Chrome's. #undef-ing it does not work as there are
// currently 118 targets that break if we do this, so leaving out of
// the precompiled header for now.
//#include <commctrl.h>  // 2
//#include <commdlg.h>  // 3
//#include <cryptuiapi.h>  // 2
//#include <Objbase.h>  // 2
//#include <objidl.h>  // 1
//#include <ole2.h>  // 1
//#include <oleacc.h>  // 2
//#include <oleauto.h>  // 1
//#include <oleidl.h>  // 1
//#include <propkey.h>  // 2
//#include <propvarutil.h>  // 2
//#include <pstore.h>  // 2
//#include <shlguid.h>  // 1
//#include <shlwapi.h>  // 1
//#include <shobjidl.h>  // 4
//#include <urlhist.h>  // 2

// Caused other conflicts in addition to the 'interface' issue above.
// #include <shlobj.h>

#include <errno.h>
#include <fcntl.h>
#include <limits.h>  // 4
#include <math.h>
#include <memory.h>  // 1
#include <signal.h>
#include <stdarg.h>  // 1
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>  // 4

#include <algorithm>
#include <bitset>  // 3
#include <cmath>
#include <cstddef>
#include <cstdio>  // 3
#include <cstdlib>  // 2
#include <cstring>
#include <deque>
#include <fstream>  // 3
#include <functional>
#include <iomanip>  // 2
#include <iosfwd>  // 2
#include <iterator>
#include <limits>
#include <list>
#include <map>
#include <numeric>  // 2
#include <ostream>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <utility>
#include <vector>
