#!/usr/bin/env python3
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# Generates API docs for Flutter embedders and libraries.
import os
import shutil
import tempfile
import zipfile
import sys
import subprocess
from collections import namedtuple

Section = namedtuple('Section', ['title', 'inputs'])

SECTIONS = {
    'ios':
        Section(
            'iOS Embedder', [
                'shell/platform/darwin/ios',
                'shell/platform/darwin/common',
                'shell/platform/common',
            ]
        ),
    'macos':
        Section(
            'macOS Embedder', [
                'shell/platform/darwin/macos',
                'shell/platform/darwin/common',
                'shell/platform/common',
            ]
        ),
    'linux':
        Section('Linux Embedder', [
            'shell/platform/linux',
            'shell/platform/common',
        ]),
    'windows':
        Section('Windows Embedder', [
            'shell/platform/windows',
            'shell/platform/common',
        ]),
    'impeller':
        Section('Impeller', [
            'impeller',
        ]),
}


def generate_doxyfile(section, output_dir, log_file, doxy_file):
  doxyfile = open('docs/Doxyfile.template', 'r').read()
  doxyfile = doxyfile.replace('@@OUTPUT_DIRECTORY@@', output_dir)
  doxyfile = doxyfile.replace('@@LOG_FILE@@', log_file)
  doxyfile = doxyfile.replace('@@INPUT_DIRECTORIES@@', '"{}"'.format('" "'.join(section.inputs)))
  doxyfile = doxyfile.replace('@@PROJECT_NAME@@', 'Flutter {}'.format(section.title))
  doxyfile = doxyfile.replace(
      '@@DOCSET_FEEDNAME@@', 'Flutter {} Documentation'.format(section.title)
  )
  with open(doxy_file, 'w') as f:
    f.write(doxyfile)


def process_section(name, section, destination):
  output_dir = tempfile.mkdtemp(prefix="doxygen")
  log_file = os.path.join(destination, '{}-doxygen.log'.format(name))
  zip_file = os.path.join(destination, '{}-docs.zip'.format(name))
  doxy_file = os.path.join(output_dir, 'Doxyfile')
  generate_doxyfile(section, output_dir, log_file, doxy_file)
  # Update the Doxyfile format to the latest format.
  subprocess.call(['doxygen', '-u'], cwd=output_dir)
  subprocess.call(['doxygen', doxy_file])
  html_dir = os.path.join(output_dir, 'html')
  with zipfile.ZipFile(zip_file, 'w') as zip:
    for root, _, files in os.walk(html_dir):
      for file in files:
        filename = os.path.join(root, file)
        zip.write(filename, os.path.relpath(filename, html_dir))
  print('Wrote ZIP file for {} to {}'.format(section, zip_file))
  print('Preserving log file in {}'.format(log_file))
  shutil.rmtree(output_dir, ignore_errors=True)


def generate_docs(argv):
  if len(argv) != 2:
    print(
        'Error: Argument specifying output directory required. '
        'Directory may be an absolute path, or a relative path from the "src" directory.'
    )
    exit(1)

  destination = argv[1]
  script_path = os.path.realpath(__file__)
  src_path = os.path.dirname(os.path.dirname(os.path.dirname(script_path)))
  # Run commands from the Flutter root dir.
  os.chdir(os.path.join(src_path, 'flutter'))
  # If the argument isn't an absolute path, assume that it is relative to the src dir.
  if not os.path.isabs(destination):
    destination = os.path.join(src_path, destination)
  os.makedirs(destination, exist_ok=True)
  for name, section in SECTIONS.items():
    process_section(name, section, destination)


if __name__ == '__main__':
  generate_docs(sys.argv)
