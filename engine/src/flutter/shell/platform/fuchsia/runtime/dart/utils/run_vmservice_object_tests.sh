#!/boot/bin/sh
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

set -x

# Kill any existing dart runners.
killall dart* || true

# Start up a dart runner app that is guaranteed to be JIT, non-product, and
# won't terminate.
run -d fuchsia-pkg://fuchsia.com/hello_dart_jit#meta/hello_dart_jit.cmx

# Wait for it to come up.
sleep 2

# Find the path to its vm service port.
# NB: This is the command used by the Flutter host-side command line tools.
# If the use of 'find' here breaks, the Flutter host-side command line tools
# will also be broken. If this is intentional, then craft and land a Flutter PR.
# See: https://github.com/flutter/flutter/commit/b18a2b1794606a6cddb6d7f3486557e473e3bfbb
# Then, land a hard transition to GI that updates Flutter and this test.
FIND_RESULT=`find /hub -name vmservice-port | grep dart_jit_runner`

echo "find result:\n${FIND_RESULT}"

killall dart* || true
if [ -z "${FIND_RESULT}" ]; then
  echo "FAILURE: Dart VM service not found in the Hub!"
  exit 1
fi
exit 0
