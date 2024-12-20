// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_COLOR_UTILS_H_
#define BASE_COLOR_UTILS_H_

/** Returns alpha byte from color value.
 */
#define ColorGetA(color) (((color) >> 24) & 0xFF)

/** Returns red component of color, from zero to 255.
 */
#define ColorGetR(color) (((color) >> 16) & 0xFF)

/** Returns green component of color, from zero to 255.
 */
#define ColorGetG(color) (((color) >> 8) & 0xFF)

/** Returns blue component of color, from zero to 255.
 */
#define ColorGetB(color) (((color) >> 0) & 0xFF)

#endif  // BASE_COLOR_UTILS_H_
