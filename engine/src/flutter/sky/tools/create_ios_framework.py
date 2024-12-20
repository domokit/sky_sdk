#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# Generates and zip the ios flutter framework including the architecture
# dependent snapshot.

import argparse
import os
import sys

from create_xcframework import create_xcframework  # pylint: disable=import-error
import sky_utils  # pylint: disable=import-error


def main():
  parser = argparse.ArgumentParser(
      description=(
          'Creates Flutter.framework, Flutter.xcframework and '
          'copies architecture-dependent gen_snapshot binaries to output dir'
      )
  )

  parser.add_argument('--dst', type=str, required=True)
  parser.add_argument('--x64-out-dir', type=str)
  parser.add_argument('--arm64-out-dir', type=str, required=True)
  parser.add_argument('--simulator-x64-out-dir', type=str, required=True)
  parser.add_argument('--simulator-arm64-out-dir', type=str, required=False)
  parser.add_argument('--strip', action='store_true', default=False)
  parser.add_argument('--dsym', action='store_true', default=False)

  args = parser.parse_args()

  dst = (args.dst if os.path.isabs(args.dst) else sky_utils.buildroot_relative_path(args.dst))
  arm64_out_dir = (
      args.arm64_out_dir if os.path.isabs(args.arm64_out_dir) else
      sky_utils.buildroot_relative_path(args.arm64_out_dir)
  )

  x64_out_dir = None
  if args.x64_out_dir:
    x64_out_dir = (
        args.x64_out_dir
        if os.path.isabs(args.x64_out_dir) else sky_utils.buildroot_relative_path(args.x64_out_dir)
    )

  simulator_x64_out_dir = None
  if args.simulator_x64_out_dir:
    simulator_x64_out_dir = (
        args.simulator_x64_out_dir if os.path.isabs(args.simulator_x64_out_dir) else
        sky_utils.buildroot_relative_path(args.simulator_x64_out_dir)
    )

  framework = os.path.join(dst, 'Flutter.framework')
  simulator_framework = os.path.join(dst, 'sim', 'Flutter.framework')
  arm64_framework = os.path.join(arm64_out_dir, 'Flutter.framework')
  simulator_x64_framework = os.path.join(simulator_x64_out_dir, 'Flutter.framework')

  simulator_arm64_out_dir = None
  if args.simulator_arm64_out_dir:
    simulator_arm64_out_dir = (
        args.simulator_arm64_out_dir if os.path.isabs(args.simulator_arm64_out_dir) else
        sky_utils.buildroot_relative_path(args.simulator_arm64_out_dir)
    )

  if args.simulator_arm64_out_dir is not None:
    simulator_arm64_framework = os.path.join(simulator_arm64_out_dir, 'Flutter.framework')

  sky_utils.assert_directory(arm64_framework, 'iOS arm64 framework')
  sky_utils.assert_directory(simulator_arm64_framework, 'iOS arm64 simulator framework')
  sky_utils.assert_directory(simulator_x64_framework, 'iOS x64 simulator framework')
  create_framework(
      args, dst, framework, arm64_framework, simulator_framework, simulator_x64_framework,
      simulator_arm64_framework
  )

  extension_safe_dst = os.path.join(dst, 'extension_safe')
  create_extension_safe_framework(
      args, extension_safe_dst, '%s_extension_safe' % arm64_out_dir,
      '%s_extension_safe' % simulator_x64_out_dir, '%s_extension_safe' % simulator_arm64_out_dir
  )

  # Copy gen_snapshot binary to destination directory.
  if arm64_out_dir:
    gen_snapshot = os.path.join(arm64_out_dir, 'gen_snapshot_arm64')
    sky_utils.copy_binary(gen_snapshot, os.path.join(dst, 'gen_snapshot_arm64'))
  if x64_out_dir:
    gen_snapshot = os.path.join(x64_out_dir, 'gen_snapshot_x64')
    sky_utils.copy_binary(gen_snapshot, os.path.join(dst, 'gen_snapshot_x64'))

  zip_archive(dst, args)
  return 0


def create_extension_safe_framework( # pylint: disable=too-many-arguments
    args, dst, arm64_out_dir, simulator_x64_out_dir, simulator_arm64_out_dir
):
  framework = os.path.join(dst, 'Flutter.framework')
  simulator_framework = os.path.join(dst, 'sim', 'Flutter.framework')
  arm64_framework = os.path.join(arm64_out_dir, 'Flutter.framework')
  simulator_x64_framework = os.path.join(simulator_x64_out_dir, 'Flutter.framework')
  simulator_arm64_framework = os.path.join(simulator_arm64_out_dir, 'Flutter.framework')

  if not os.path.isdir(arm64_framework):
    print('Cannot find extension safe iOS arm64 Framework at %s' % arm64_framework)
    return 1

  if not os.path.isdir(simulator_x64_framework):
    print('Cannot find extension safe iOS x64 simulator Framework at %s' % simulator_x64_framework)
    return 1

  create_framework(
      args, dst, framework, arm64_framework, simulator_framework, simulator_x64_framework,
      simulator_arm64_framework
  )
  return 0


def create_framework(  # pylint: disable=too-many-arguments
    args, dst, framework, arm64_framework, simulator_framework,
    simulator_x64_framework, simulator_arm64_framework
):
  arm64_dylib = os.path.join(arm64_framework, 'Flutter')
  simulator_x64_dylib = os.path.join(simulator_x64_framework, 'Flutter')
  simulator_arm64_dylib = os.path.join(simulator_arm64_framework, 'Flutter')
  sky_utils.assert_file(arm64_dylib, 'iOS arm64 dylib')
  sky_utils.assert_file(simulator_arm64_dylib, 'iOS simulator arm64 dylib')
  sky_utils.assert_file(simulator_x64_dylib, 'iOS simulator x64 dylib')

  # Compute dsym output paths, if enabled.
  framework_dsym = None
  if args.dsym:
    framework_dsym = framework + '.dSYM'

  # Emit the framework for physical devices.
  sky_utils.copy_tree(arm64_framework, framework)
  framework_binary = os.path.join(framework, 'Flutter')
  process_framework(args, dst, framework_binary, framework_dsym)

  # Emit the framework for simulators.
  if args.simulator_arm64_out_dir is not None:
    sky_utils.copy_tree(simulator_arm64_framework, simulator_framework)
    simulator_framework_binary = os.path.join(simulator_framework, 'Flutter')
    sky_utils.lipo([simulator_x64_dylib, simulator_arm64_dylib], simulator_framework_binary)
    process_framework(args, dst, simulator_framework_binary, dsym=None)
  else:
    simulator_framework = simulator_x64_framework

  # Create XCFramework from the arm-only fat framework and the arm64/x64
  # simulator frameworks, or just the x64 simulator framework if only that one
  # exists.
  xcframeworks = [simulator_framework, framework]
  dsyms = {framework: framework_dsym} if args.dsym else None
  create_xcframework(location=dst, name='Flutter', frameworks=xcframeworks, dsyms=dsyms)

  sky_utils.lipo([arm64_dylib, simulator_x64_dylib], framework_binary)
  process_framework(args, dst, framework_binary, framework_dsym)
  return 0


def zip_archive(dst, args):
  # pylint: disable=line-too-long

  # When updating with_entitlements and without_entitlements,
  # `binariesWithoutEntitlements` and `signedXcframeworks` should be updated in
  # the framework's `verifyCodeSignedTestRunner`.
  #
  # See: https://github.com/flutter/flutter/blob/62382c7b83a16b3f48dc06c19a47f6b8667005a5/dev/bots/suite_runners/run_verify_binaries_codesigned_tests.dart#L82-L130

  # Binaries that must be codesigned and require entitlements for particular APIs.
  with_entitlements = ['gen_snapshot_arm64']
  with_entitlements_file = os.path.join(dst, 'entitlements.txt')
  sky_utils.write_codesign_config(with_entitlements_file, with_entitlements)

  # Binaries that must be codesigned and DO NOT require entitlements.
  without_entitlements = [
      'Flutter.xcframework/ios-arm64/Flutter.framework/Flutter',
      'Flutter.xcframework/ios-arm64_x86_64-simulator/Flutter.framework/Flutter',
      'extension_safe/Flutter.xcframework/ios-arm64/Flutter.framework/Flutter',
      'extension_safe/Flutter.xcframework/ios-arm64_x86_64-simulator/Flutter.framework/Flutter',
  ]
  without_entitlements_file = os.path.join(dst, 'without_entitlements.txt')
  sky_utils.write_codesign_config(without_entitlements_file, without_entitlements)

  # Binaries that will not be codesigned.
  unsigned_binaries = []
  if args.dsym:
    unsigned_binaries.extend([
        'Flutter.xcframework/ios-arm64/dSYMs/Flutter.framework.dSYM/Contents/Resources/DWARF/Flutter',
        'extension_safe/Flutter.xcframework/ios-arm64/dSYMs/Flutter.framework.dSYM/Contents/Resources/DWARF/Flutter',
    ])
  unsigned_binaries_file = os.path.join(dst, 'unsigned_binaries.txt')
  sky_utils.write_codesign_config(unsigned_binaries_file, unsigned_binaries)
  # pylint: enable=line-too-long

  zip_contents = [
      'gen_snapshot_arm64',
      'Flutter.xcframework',
      'entitlements.txt',
      'without_entitlements.txt',
      'unsigned_binaries.txt',
      'extension_safe/Flutter.xcframework',
  ]
  sky_utils.assert_valid_codesign_config(
      dst, zip_contents, with_entitlements, without_entitlements, unsigned_binaries
  )
  sky_utils.create_zip(dst, 'artifacts.zip', zip_contents)


def process_framework(args, dst, framework_binary, dsym):
  if dsym:
    sky_utils.extract_dsym(framework_binary, dsym)

  if args.strip:
    unstripped_out = os.path.join(dst, 'Flutter.unstripped')
    sky_utils.strip_binary(framework_binary, unstripped_copy_path=unstripped_out)


if __name__ == '__main__':
  sys.exit(main())
