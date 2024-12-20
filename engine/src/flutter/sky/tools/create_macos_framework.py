#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import shutil
import sys
import os

from create_xcframework import create_xcframework  # pylint: disable=import-error
import sky_utils  # pylint: disable=import-error


def main():
  parser = argparse.ArgumentParser(
      description='Creates FlutterMacOS.framework and FlutterMacOS.xcframework for macOS'
  )

  parser.add_argument('--dst', type=str, required=True)
  parser.add_argument('--arm64-out-dir', type=str, required=True)
  parser.add_argument('--x64-out-dir', type=str, required=True)
  parser.add_argument('--strip', action='store_true', default=False)
  parser.add_argument('--dsym', action='store_true', default=False)
  parser.add_argument('--zip', action='store_true', default=False)

  args = parser.parse_args()

  dst = args.dst if os.path.isabs(args.dst) else sky_utils.buildroot_relative_path(args.dst)

  arm64_out_dir = (
      args.arm64_out_dir if os.path.isabs(args.arm64_out_dir) else
      sky_utils.buildroot_relative_path(args.arm64_out_dir)
  )

  x64_out_dir = (
      args.x64_out_dir
      if os.path.isabs(args.x64_out_dir) else sky_utils.buildroot_relative_path(args.x64_out_dir)
  )

  arm64_framework = os.path.join(arm64_out_dir, 'FlutterMacOS.framework')
  if not os.path.isdir(arm64_framework):
    print('Cannot find macOS arm64 Framework at %s' % arm64_framework)
    return 1

  x64_framework = os.path.join(x64_out_dir, 'FlutterMacOS.framework')
  if not os.path.isdir(x64_framework):
    print('Cannot find macOS x64 Framework at %s' % x64_framework)
    return 1

  arm64_dylib = sky_utils.get_mac_framework_dylib_path(arm64_framework)
  if not os.path.isfile(arm64_dylib):
    print('Cannot find macOS arm64 dylib at %s' % arm64_dylib)
    return 1

  x64_dylib = sky_utils.get_mac_framework_dylib_path(x64_framework)
  if not os.path.isfile(x64_dylib):
    print('Cannot find macOS x64 dylib at %s' % x64_dylib)
    return 1

  fat_framework = os.path.join(dst, 'FlutterMacOS.framework')
  sky_utils.create_fat_macos_framework(args, dst, fat_framework, arm64_framework, x64_framework)

  # Create XCFramework from the arm64 and x64 fat framework.
  xcframeworks = [fat_framework]
  dsyms = {fat_framework: fat_framework + '.dSYM'} if args.dsym else None
  create_xcframework(location=dst, name='FlutterMacOS', frameworks=xcframeworks, dsyms=dsyms)

  if args.zip:
    zip_framework(dst, args)

  return 0


def zip_framework(dst, args):
  # pylint: disable=line-too-long
  # When updating with_entitlements and without_entitlements,
  # `binariesWithoutEntitlements` and `signedXcframeworks` should be updated in
  # the framework's `verifyCodeSignedTestRunner`.
  #
  # See: https://github.com/flutter/flutter/blob/62382c7b83a16b3f48dc06c19a47f6b8667005a5/dev/bots/suite_runners/run_verify_binaries_codesigned_tests.dart#L82-L130
  framework_dst = os.path.join(dst, 'FlutterMacOS.framework')
  sky_utils.write_codesign_config(os.path.join(framework_dst, 'entitlements.txt'), [])
  sky_utils.write_codesign_config(
      os.path.join(framework_dst, 'without_entitlements.txt'),
      [
          # TODO(cbracken): Remove the zip file from the path when outer zip is removed.
          'FlutterMacOS.framework.zip/Versions/A/FlutterMacOS'
      ]
  )
  sky_utils.create_zip(framework_dst, 'FlutterMacOS.framework.zip', ['.'])
  # pylint: enable=line-too-long

  # Double zip to make it consistent with legacy artifacts.
  # TODO(fujino): remove this once https://github.com/flutter/flutter/issues/125067 is resolved
  sky_utils.create_zip(
      framework_dst,
      'FlutterMacOS.framework_.zip',
      [
          'FlutterMacOS.framework.zip',
          # TODO(cbracken): Move these files to inner zip before removing the outer zip.
          'entitlements.txt',
          'without_entitlements.txt',
      ]
  )

  # Overwrite the FlutterMacOS.framework.zip with the double-zipped archive.
  final_src_path = os.path.join(framework_dst, 'FlutterMacOS.framework_.zip')
  final_dst_path = os.path.join(dst, 'FlutterMacOS.framework.zip')
  shutil.move(final_src_path, final_dst_path)

  zip_xcframework_archive(dst, args)


def zip_xcframework_archive(dst, args):
  # pylint: disable=line-too-long

  # When updating with_entitlements and without_entitlements,
  # `binariesWithoutEntitlements` and `signedXcframeworks` should be updated in
  # the framework's `verifyCodeSignedTestRunner`.
  #
  # See: https://github.com/flutter/flutter/blob/62382c7b83a16b3f48dc06c19a47f6b8667005a5/dev/bots/suite_runners/run_verify_binaries_codesigned_tests.dart#L82-L130

  # Binaries that must be codesigned and require entitlements for particular APIs.
  with_entitlements = []
  with_entitlements_file = os.path.join(dst, 'entitlements.txt')
  sky_utils.write_codesign_config(with_entitlements_file, with_entitlements)

  # Binaries that must be codesigned and DO NOT require entitlements.
  without_entitlements = [
      'FlutterMacOS.xcframework/macos-arm64_x86_64/FlutterMacOS.framework/Versions/A/FlutterMacOS',
  ]
  without_entitlements_file = os.path.join(dst, 'without_entitlements.txt')
  sky_utils.write_codesign_config(without_entitlements_file, without_entitlements)

  # Binaries that will not be codesigned.
  unsigned_binaries = []
  if args.dsym:
    unsigned_binaries.extend([
        'FlutterMacOS.xcframework/macos-arm64_x86_64/dSYMs/FlutterMacOS.framework.dSYM/Contents/Resources/DWARF/FlutterMacOS',
    ])
  unsigned_binaries_file = os.path.join(dst, 'unsigned_binaries.txt')
  sky_utils.write_codesign_config(unsigned_binaries_file, unsigned_binaries)
  # pylint: enable=line-too-long

  zip_contents = [
      'FlutterMacOS.xcframework',
      'entitlements.txt',
      'without_entitlements.txt',
      'unsigned_binaries.txt',
  ]
  sky_utils.assert_valid_codesign_config(
      dst, zip_contents, with_entitlements, without_entitlements, unsigned_binaries
  )
  sky_utils.create_zip(dst, 'framework.zip', zip_contents)


if __name__ == '__main__':
  sys.exit(main())
