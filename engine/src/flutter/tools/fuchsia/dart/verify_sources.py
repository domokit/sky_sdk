#!/usr/bin/env python3

# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import os
import sys
import pathlib


def main():
  parser = argparse.ArgumentParser(
      "Verifies that all .dart files are included in sources, and sources don't include nonexsitent files"
  )
  parser.add_argument(
      "--source_dir", help="Path to the directory containing the package sources", required=True
  )
  parser.add_argument("--stamp", help="File to touch when source checking succeeds", required=True)
  parser.add_argument("sources", help="source files", nargs=argparse.REMAINDER)
  args = parser.parse_args()

  actual_sources = set()
  # Get all dart sources from source directory.
  src_dir_path = pathlib.Path(args.source_dir)
  for (dirpath, dirnames, filenames) in os.walk(src_dir_path, topdown=True):
    relpath_to_src_root = pathlib.Path(dirpath).relative_to(src_dir_path)
    actual_sources.update(
        os.path.normpath(relpath_to_src_root.joinpath(filename))
        for filename in filenames
        if pathlib.Path(filename).suffix == ".dart"
    )

  expected_sources = set(args.sources)
  # It is possible for sources to include dart files outside of source_dir.
  actual_sources.update([
      s for s in (expected_sources - actual_sources) if src_dir_path.joinpath(s).resolve().exists()
  ],)

  if actual_sources == expected_sources:
    with open(args.stamp, "w") as stamp:
      stamp.write("Success!")
    return 0

  def sources_to_abs_path(sources):
    return sorted(str(src_dir_path.joinpath(s)) for s in sources)

  missing_sources = actual_sources - expected_sources
  if missing_sources:
    print(
        '\nSource files found that were missing from the "sources" parameter:\n{}\n'.format(
            "\n".join(sources_to_abs_path(missing_sources))
        ),
    )
  nonexistent_sources = expected_sources - actual_sources
  if nonexistent_sources:
    print(
        '\nSource files listed in "sources" parameter but not found:\n{}\n'.format(
            "\n".join(sources_to_abs_path(nonexistent_sources))
        ),
    )
  return 1


if __name__ == "__main__":
  sys.exit(main())
