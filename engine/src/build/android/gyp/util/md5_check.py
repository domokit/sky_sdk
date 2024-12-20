# Copyright 2013 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import hashlib
import os


def CallAndRecordIfStale(
    function, record_path=None, input_paths=None, input_strings=None,
    force=False):
  """Calls function if the md5sum of the input paths/strings has changed.

  The md5sum of the inputs is compared with the one stored in record_path. If
  this has changed (or the record doesn't exist), function will be called and
  the new md5sum will be recorded.

  If force is True, the function will be called regardless of whether the
  md5sum is out of date.
  """
  if not input_paths:
    input_paths = []
  if not input_strings:
    input_strings = []
  md5_checker = _Md5Checker(
      record_path=record_path,
      input_paths=input_paths,
      input_strings=input_strings)
  if force or md5_checker.IsStale():
    function()
    md5_checker.Write()


def _UpdateMd5ForFile(md5, path, block_size=2**16):
  with open(path, 'rb') as infile:
    while True:
      data = infile.read(block_size)
      if not data:
        break
      md5.update(data)


def _UpdateMd5ForDirectory(md5, dir_path):
  for root, _, files in os.walk(dir_path):
    for f in files:
      _UpdateMd5ForFile(md5, os.path.join(root, f))


def _UpdateMd5ForPath(md5, path):
  if os.path.isdir(path):
    _UpdateMd5ForDirectory(md5, path)
  else:
    _UpdateMd5ForFile(md5, path)


class _Md5Checker(object):
  def __init__(self, record_path=None, input_paths=None, input_strings=None):
    if not input_paths:
      input_paths = []
    if not input_strings:
      input_strings = []

    assert record_path.endswith('.stamp'), (
        'record paths must end in \'.stamp\' so that they are easy to find '
        'and delete')

    self.record_path = record_path

    md5 = hashlib.md5()
    for i in sorted(input_paths):
      _UpdateMd5ForPath(md5, i)
    for s in input_strings:
      md5.update(s.encode('utf-8'))
    self.new_digest = md5.hexdigest()

    self.old_digest = ''
    if os.path.exists(self.record_path):
      with open(self.record_path, 'r') as old_record:
        self.old_digest = old_record.read()

  def IsStale(self):
    return self.old_digest != self.new_digest

  def Write(self):
    with open(self.record_path, 'w') as new_record:
      new_record.write(self.new_digest)
