#!/usr/bin/env vpython3
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import difflib
import json
import os
import sys

# This script detects performance impacting changes to shaders.
#
# When the GN build is configured with the path to the `malioc` tool, the
# results of its analysis will be placed under `out/$CONFIG/gen/malioc` in
# separate .json files. That path should be supplied to this script as the
# `--after` argument. This script compares those results against previous
# results in a golden file checked in to the tree under
# `flutter/impeller/tools/malioc.json`. That file should be passed to this
# script as the `--before` argument. To create or update the golden file,
# passing the `--update` flag will cause the data from the `--after` path to
# overwrite the file at the `--before` path.
#
# Configure and build:
# $ flutter/tools/gn --malioc-path path/to/malioc
# $ ninja -C out/host_debug
#
# Analyze
# $ flutter/impeller/tools/malioc_diff.py \
#   --before flutter/impeller/tools/malioc.json \
#   --after out/host_debug/gen/malioc
#
# If there are differences between before and after, whether positive or
# negative, the exit code for this script will be 1, and 0 otherwise.

SRC_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)

CORES = [
    'Mali-G78',  # Pixel 6 / 2020
    'Mali-T880',  # 2016
]

# Path to the engine root checkout. This is used to calculate absolute
# paths if relative ones are passed to the script.
BUILD_ROOT_DIR = os.path.abspath(os.path.join(os.path.realpath(__file__), '..', '..', '..', '..'))


def parse_args(argv):
  parser = argparse.ArgumentParser(
      description='A script that compares before/after malioc analysis results',
  )
  parser.add_argument(
      '--after',
      '-a',
      type=str,
      help='The path to a directory tree containing new malioc results in json files.',
  )
  parser.add_argument(
      '--before',
      '-b',
      type=str,
      help='The path to a json file containing existing malioc results.',
  )
  parser.add_argument(
      '--after-relative-to-src',
      type=str,
      help=(
          'A relative path calculated from the engine src directory to '
          'a directory tree containing new malioc results in json files'
      ),
  )
  parser.add_argument(
      '--before-relative-to-src',
      type=str,
      help=(
          'A relative path calculated from the engine src directory to '
          'a json file containing existing malioc results in json files'
      ),
  )
  parser.add_argument(
      '--print-diff',
      '-p',
      default=False,
      action='store_true',
      help='Print a unified diff to stdout when differences are found.',
  )
  parser.add_argument(
      '--update',
      '-u',
      default=False,
      action='store_true',
      help='Write results from the --after tree to the --before file.',
  )
  parser.add_argument(
      '--verbose',
      '-v',
      default=False,
      action='store_true',
      help='Emit verbose output.',
  )
  return parser.parse_args(argv)


def validate_args(args):
  if not args.after and not args.after_relative_to_src:
    print('--after argument or --after-relative-to-src must be specified.')
    return False

  if not args.before and not args.before_relative_to_src:
    print('--before argument or --before-relative-to-src must be specified.')
    return False

  # Generate full paths if relative ones are provided with before and
  # after taking precedence.
  args.before = (args.before or os.path.join(BUILD_ROOT_DIR, args.before_relative_to_src))
  args.after = (args.after or os.path.join(BUILD_ROOT_DIR, args.after_relative_to_src))

  if not args.after or not os.path.isdir(args.after):
    print('The --after argument must refer to a directory.')
    return False
  if not args.before or (not args.update and not os.path.isfile(args.before)):
    print('The --before argument must refer to an existing file.')
    return False
  return True


# Reads the 'performance' section of the malioc analysis results.
def read_malioc_file_performance(performance_json):
  performance = {}
  performance['pipelines'] = performance_json['pipelines']

  longest_path_cycles = performance_json['longest_path_cycles']
  performance['longest_path_cycles'] = longest_path_cycles['cycle_count']
  performance['longest_path_bound_pipelines'] = longest_path_cycles['bound_pipelines']

  shortest_path_cycles = performance_json['shortest_path_cycles']
  performance['shortest_path_cycles'] = shortest_path_cycles['cycle_count']
  performance['shortest_path_bound_pipelines'] = shortest_path_cycles['bound_pipelines']

  total_cycles = performance_json['total_cycles']
  performance['total_cycles'] = total_cycles['cycle_count']
  performance['total_bound_pipelines'] = total_cycles['bound_pipelines']
  return performance


# Parses the json output from malioc, which follows the schema defined in
# `mali_offline_compiler/samples/json_schemas/performance-schema.json`.
def read_malioc_file(malioc_tree, json_file):
  with open(json_file, 'r') as file:
    json_obj = json.load(file)

  build_gen_dir = os.path.dirname(malioc_tree)

  results = []
  for shader in json_obj['shaders']:
    # Ignore cores not in the allowlist above.
    if shader['hardware']['core'] not in CORES:
      continue
    result = {}
    filename = os.path.relpath(shader['filename'], build_gen_dir)
    if filename.startswith('../..'):
      filename = filename[6:]
    if filename.startswith('../'):
      filename = filename[3:]
    result['filename'] = filename
    result['core'] = shader['hardware']['core']
    result['type'] = shader['shader']['type']
    for prop in shader['properties']:
      result[prop['name']] = prop['value']

    result['variants'] = {}
    for variant in shader['variants']:
      variant_result = {}
      for prop in variant['properties']:
        variant_result[prop['name']] = prop['value']

      performance_json = variant['performance']
      performance = read_malioc_file_performance(performance_json)
      variant_result['performance'] = performance
      result['variants'][variant['name']] = variant_result
    results.append(result)

  return results


# Parses a tree of malioc performance json files.
#
# The parsing results are returned in a map keyed by the shader file name, whose
# values are maps keyed by the core type. The values in these maps are the
# performance properties of the shader on the core reported by malioc. This
# structure allows for a fast lookup and comparison against the golen file.
def read_malioc_tree(malioc_tree):
  results = {}
  for root, _, files in os.walk(malioc_tree):
    for file in files:
      if not file.endswith('.json'):
        continue
      full_path = os.path.join(root, file)
      for shader in read_malioc_file(malioc_tree, full_path):
        if shader['filename'] not in results:
          results[shader['filename']] = {}
        results[shader['filename']][shader['core']] = shader
  return results


# Converts a list to a string in which each list element is left-aligned in
# a space of `width` characters, and separated by `sep`. The separator does not
# count against the `width`. If `width` is 0, then the width is unconstrained.
def pretty_list(lst, fmt='s', sep='', width=12):
  formats = ['{:<{width}{fmt}}' if ele is not None else '{:<{width}s}' for ele in lst]
  sanitized_list = [x if x is not None else 'null' for x in lst]
  return (sep.join(formats)).format(width='' if width == 0 else width, fmt=fmt, *sanitized_list)


def compare_performance(variant, before, after):
  cycles = [['longest_path_cycles', 'longest_path_bound_pipelines'],
            ['shortest_path_cycles', 'shortest_path_bound_pipelines'],
            ['total_cycles', 'total_bound_pipelines']]
  differences = []
  for cycle in cycles:
    if before[cycle[0]] == after[cycle[0]]:
      continue
    before_cycles = before[cycle[0]]
    before_bounds = before[cycle[1]]
    after_cycles = after[cycle[0]]
    after_bounds = after[cycle[1]]
    differences += [
        '{} in variant {}\n{}{}\n{:<8}{}{}\n{:<8}{}{}\n'.format(
            cycle[0],
            variant,
            ' ' * 8,
            pretty_list(before['pipelines'] + ['bound']),  # Column labels.
            'before',
            pretty_list(before_cycles, fmt='f'),
            pretty_list(before_bounds, sep=',', width=0),
            'after',
            pretty_list(after_cycles, fmt='f'),
            pretty_list(after_bounds, sep=',', width=0),
        )
    ]
  return differences


def compare_variants(befores, afters):
  differences = []
  for variant_name, before_variant in befores.items():
    if variant_name in afters:
      after_variant = afters[variant_name]
      for variant_key, before_variant_val in before_variant.items():
        after_variant_val = after_variant[variant_key]
        if variant_key == 'performance':
          differences += compare_performance(variant_name, before_variant_val, after_variant_val)
        elif before_variant_val != after_variant_val:
          differences += [
              'In variant {}:\n  {vkey}: {} <- before\n  {vkey}: {} <- after'.format(
                  variant_name,
                  before_variant_val,
                  after_variant_val,
                  vkey=variant_key,
              )
          ]
  return differences


# Compares two shaders. Prints a report and returns True if there are
# differences, and returns False otherwise.
def compare_shaders(malioc_tree, before_shader, after_shader):
  differences = []
  for key, before_val in before_shader.items():
    after_val = after_shader[key]
    if key == 'variants':
      differences += compare_variants(before_val, after_val)
    elif key == 'performance':
      differences += compare_performance('Default', before_val, after_val)
    elif before_val != after_val:
      differences += ['{}:\n  {} <- before\n  {} <- after'.format(key, before_val, after_val)]

  if bool(differences):
    build_gen_dir = os.path.dirname(malioc_tree)
    filename = before_shader['filename']
    core = before_shader['core']
    typ = before_shader['type']
    print('Changes found in shader {} on core {}:'.format(filename, core))
    for diff in differences:
      print(diff)
    print(
        '\nFor a full report, run:\n  $ malioc --{} --core {} {}/{}\n'.format(
            typ.lower(), core, build_gen_dir, filename
        )
    )

  return bool(differences)


def main(argv):
  args = parse_args(argv[1:])
  if not validate_args(args):
    return 1

  after_json = read_malioc_tree(args.after)
  if not bool(after_json):
    print('Did not find any malioc results under {}.'.format(args.after))
    return 1

  if args.update:
    # Write the new results to the file given by --before, then exit.
    with open(args.before, 'w') as file:
      json.dump(after_json, file, sort_keys=True, indent=2)
    return 0

  with open(args.before, 'r') as file:
    before_json = json.load(file)

  changed = False
  for filename, shaders in before_json.items():
    if filename not in after_json.keys():
      print('Shader "{}" has been removed.'.format(filename))
      changed = True
      continue
    for core, before_shader in shaders.items():
      if core not in after_json[filename].keys():
        continue
      after_shader = after_json[filename][core]
      if compare_shaders(args.after, before_shader, after_shader):
        changed = True

  for filename, shaders in after_json.items():
    if filename not in before_json:
      print('Shader "{}" is new.'.format(filename))
      changed = True

  if changed:
    print(
        'There are new shaders, shaders have been removed, or performance '
        'changes to existing shaders. The golden file must be updated after a '
        'build of android_debug_unopt using the --malioc-path flag to the '
        'flutter/tools/gn script.\n\n'
        '$ ./flutter/impeller/tools/malioc_diff.py --before {} --after {} --update'.format(
            args.before, args.after
        )
    )
    if args.print_diff:
      before_lines = json.dumps(before_json, sort_keys=True, indent=2).splitlines(keepends=True)
      after_lines = json.dumps(after_json, sort_keys=True, indent=2).splitlines(keepends=True)
      before_path = os.path.relpath(os.path.abspath(args.before), start=SRC_ROOT)
      diff = difflib.unified_diff(before_lines, after_lines, fromfile=before_path)
      print('\nYou can alternately apply the diff below:')
      print('patch -p0 <<DONE')
      print(*diff, sep='')
      print('DONE')

  return 1 if changed else 0


if __name__ == '__main__':
  sys.exit(main(sys.argv))
