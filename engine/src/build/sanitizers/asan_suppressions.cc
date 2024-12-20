// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file contains the default suppressions for AddressSanitizer.
// It should only be used under very limited circumstances such as suppressing
// a report caused by an interceptor call in a system-installed library.

#if defined(ADDRESS_SANITIZER)

// Please make sure the code below declares a single string variable
// kASanDefaultSuppressions which contains ASan suppressions delimited by
// newlines.
char kASanDefaultSuppressions[] =
// http://crbug.com/178677
"interceptor_via_lib:libsqlite3.so\n"

// PLEASE READ ABOVE BEFORE ADDING NEW SUPPRESSIONS.

// End of suppressions.
;  // Please keep this semicolon.

#endif  // ADDRESS_SANITIZER
