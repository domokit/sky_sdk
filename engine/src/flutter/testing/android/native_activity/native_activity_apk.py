# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import os
import subprocess
import sys
import zipfile


def run_command_checked(command, env=None):
  try:
    env = env if env is not None else os.environ
    subprocess.check_output(command, stderr=subprocess.STDOUT, text=True, env=env)
  except subprocess.CalledProcessError as cpe:
    print(cpe.output)
    raise cpe


def is_mac():
  return sys.platform == 'darwin'


def java_home():
  script_path = os.path.dirname(os.path.realpath(__file__))
  if is_mac():
    return os.path.join(
        script_path, '..', '..', '..', 'third_party', 'java', 'openjdk', 'Contents', 'Home'
    )
  return os.path.join(script_path, '..', 'third_party', 'java', 'openjdk')


def java_bin():
  return os.path.join(java_home(), 'bin')


def main():
  parser = argparse.ArgumentParser()

  parser.add_argument('--aapt2-bin', type=str, required=True, help='The path to the aapt2 binary.')
  parser.add_argument(
      '--zipalign-bin', type=str, required=True, help='The path to the zipalign binary.'
  )
  parser.add_argument(
      '--apksigner-bin', type=str, required=True, help='The path to the apksigner binary.'
  )
  parser.add_argument(
      '--android-manifest', type=str, required=True, help='The path to the AndroidManifest.xml.'
  )
  parser.add_argument('--android-jar', type=str, required=True, help='The path to android.jar.')
  parser.add_argument('--output-path', type=str, required=True, help='The path to the output apk.')
  parser.add_argument(
      '--library', type=str, required=True, help='The path to the library to put in the apk.'
  )
  parser.add_argument(
      '--keystore', type=str, required=True, help='The path to the debug keystore to sign the apk.'
  )
  parser.add_argument(
      '--gen-dir', type=str, required=True, help='The directory for generated files.'
  )
  parser.add_argument(
      '--android-abi', type=str, required=True, help='The android ABI of the library.'
  )

  args = parser.parse_args()

  library_file = os.path.basename(args.library)
  apk_name = os.path.basename(args.output_path)

  unaligned_apk_path = os.path.join(args.gen_dir, '%s.unaligned' % apk_name)
  unsigned_apk_path = os.path.join(args.gen_dir, '%s.unsigned' % apk_name)
  apk_path = args.output_path

  java_path = ':'.join([java_bin(), os.environ['PATH']])
  env = dict(os.environ, PATH=java_path, JAVA_HOME=java_home())

  # Create the skeleton of the APK using aapt2.
  aapt2_command = [
      args.aapt2_bin,
      'link',
      '-I',
      args.android_jar,
      '--manifest',
      args.android_manifest,
      '-o',
      unaligned_apk_path,
  ]
  run_command_checked(aapt2_command, env=env)

  # Stuff the library in the APK which is just a regular ZIP file. Libraries are not compressed.
  with zipfile.ZipFile(unaligned_apk_path, 'a', compression=zipfile.ZIP_STORED) as zipf:
    zipf.write(args.library, 'lib/%s/%s' % (args.android_abi, library_file))

  # Align the dylib to a page boundary.
  zipalign_command = [
      args.zipalign_bin,
      '-p',  # Page align the dylib
      '-f',  # overwrite output if exists
      '4',  # 32-bit alignment
      unaligned_apk_path,
      unsigned_apk_path,
  ]
  run_command_checked(zipalign_command, env=env)

  # Sign the APK.
  apksigner_command = [
      args.apksigner_bin, 'sign', '--ks', args.keystore, '--ks-pass', 'pass:android', '--out',
      apk_path, unsigned_apk_path
  ]
  run_command_checked(apksigner_command, env=env)

  # Remove the intermediates so the out directory isn't full of large files.
  os.remove(unaligned_apk_path)
  os.remove(unsigned_apk_path)

  return 0


if __name__ == '__main__':
  sys.exit(main())
