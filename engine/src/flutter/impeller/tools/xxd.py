# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import errno
import os


def make_directories(path):
  try:
    os.makedirs(path)
  except OSError as exc:
    if exc.errno == errno.EEXIST and os.path.isdir(path):
      pass
    else:
      raise


# Dump the bytes of file into a C translation unit.
# This can be used to embed the file contents into a binary.
def main():
  parser = argparse.ArgumentParser()
  parser.add_argument(
      '--symbol-name', type=str, required=True, help='The name of the symbol referencing the data.'
  )
  parser.add_argument(
      '--output-header',
      type=str,
      required=True,
      help='The header file containing the symbol reference.'
  )
  parser.add_argument(
      '--output-source', type=str, required=True, help='The source file containing the file bytes.'
  )
  parser.add_argument(
      '--source',
      type=str,
      required=True,
      help='The source file whose contents to embed in the output source file.'
  )

  args = parser.parse_args()

  assert os.path.exists(args.source)

  output_header = os.path.abspath(args.output_header)
  output_source = os.path.abspath(args.output_source)
  output_header_basename = output_header[output_header.rfind('/') + 1:]

  make_directories(os.path.dirname(output_header))
  make_directories(os.path.dirname(output_source))

  with open(args.source, 'rb') as source, open(output_source, 'w') as output:
    data_len = 0
    output.write(f'#include "{output_header_basename}"\n')
    output.write('#include <cstddef>\n')
    output.write(
        f'alignas(std::max_align_t) const unsigned char impeller_{args.symbol_name}_data[] =\n'
    )
    output.write('{\n')
    while True:
      byte = source.read(1)
      if not byte:
        break
      data_len += 1
      output.write(f'{ord(byte)},')
    output.write('};\n')
    output.write(f'const unsigned long impeller_{args.symbol_name}_length = {data_len};\n')

  with open(output_header, 'w') as output:
    output.write('#pragma once\n')
    output.write('#ifdef __cplusplus\n')
    output.write('extern "C" {\n')
    output.write('#endif\n\n')

    output.write(f'extern const unsigned char impeller_{args.symbol_name}_data[];\n')
    output.write(f'extern const unsigned long impeller_{args.symbol_name}_length;\n\n')

    output.write('#ifdef __cplusplus\n')
    output.write('}\n')
    output.write('#endif\n')


if __name__ == '__main__':
  main()
