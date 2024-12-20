# Copyright 2015 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# Helper script to run dump_syms on Chrome Linux executables and strip
# them if needed.

import os
import subprocess
import sys

if len(sys.argv) != 5:
  print("dump_app_syms.py <dump_syms_exe> <strip_binary>")
  print("                 <binary_with_symbols> <symbols_output>")
  sys.exit(1)

dumpsyms = sys.argv[1]
strip_binary = sys.argv[2]
infile = sys.argv[3]
outfile = sys.argv[4]

# Dump only when the output file is out-of-date.
if not os.path.isfile(outfile) or \
   os.stat(outfile).st_mtime > os.stat(infile).st_mtime:
  with open(outfile, 'w') as outfileobj:
    subprocess.check_call([dumpsyms, '-r', infile], stdout=outfileobj)

if strip_binary != '0':
  subprocess.check_call(['strip', infile])
