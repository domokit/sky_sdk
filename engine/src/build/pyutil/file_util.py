# Copyright (c) 2012 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import errno
import os
import uuid

"""Creates a directory and its parents (i.e. `mkdir -p`).

If the directory already exists, does nothing."""
def mkdir_p(path):
  try:
    os.makedirs(path)
  except OSError as exc:
    if exc.errno == errno.EEXIST and os.path.isdir(path):
      pass
    else:
      raise


"""Creates or ovewrites a symlink from `link` to `target`."""
def symlink(target, link):
  mkdir_p(os.path.dirname(link))
  tmp_link = link + '.tmp.' + uuid.uuid4().hex
  try:
    os.remove(tmp_link)
  except OSError:
    pass
  os.symlink(target, tmp_link)
  try:
    os.rename(tmp_link, link)
  except FileExistsError:
    try:
      os.remove(tmp_link)
    except OSError:
      pass
