# Copyright 2013 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# The following are unicode (string) constants that were previously defined in
# "colorama". They are inlined here to avoid a dependency, as this is the only
# call site and it's unlikely we'll need to change them.
FOREGROUND_YELLOW = '\x1b[33m'
FOREGROUND_MAGENTA = '\x1b[35m'
FOREGROUND_BLUE = '\x1b[34m'
FOREGROUND_RESET = '\x1b[39m'
STYLE_RESET_ALL = '\x1b[0m'
STYLE_DIM = '\x1b[2m'
STYLE_BRIGHT = '\x1b[1m'
