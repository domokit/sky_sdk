#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#
# This script contains helper function(s) for supporting both
# python 2 and python 3 infrastructure code.

ENCODING = 'UTF-8'


def byte_str_decode(str_or_bytes):
  """Returns a string if given either a string or bytes.

    TODO: This function should be removed when the supported python
    version is only python 3.

    Args:
        str_or_bytes (string or bytes) we want to convert or return as
        the possible value changes depending on the version of python
        used.
    """
  return str_or_bytes if isinstance(str_or_bytes, str) else str_or_bytes.decode(ENCODING)
