# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# This python script uses `pub get --offline` to fill in
# .dart_tool/package_config.json files for Dart packages in the tree whose
# dependencies should be entirely resolved without requesting data from pub.dev.
# This allows us to be certain that the Dart code we are pulling for these
# packages is explicitly fetched by `gclient sync` rather than implicitly
# fetched by pub version solving, and pub fetching transitive dependencies.

import json
import os
import subprocess
import sys

THIS_DIR = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(THIS_DIR, '..', 'third_party', 'pyyaml', 'lib'))
import yaml  # pylint: disable=import-error, wrong-import-position

SRC_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ENGINE_DIR = os.path.join(SRC_ROOT, 'flutter')

ALL_PACKAGES = [
    os.path.join(ENGINE_DIR),
    os.path.join(ENGINE_DIR, 'ci'),
    os.path.join(ENGINE_DIR, 'flutter_frontend_server'),
    os.path.join(ENGINE_DIR, 'impeller', 'tessellator', 'dart'),
    os.path.join(ENGINE_DIR, 'shell', 'vmservice'),
    os.path.join(ENGINE_DIR, 'testing', 'benchmark'),
    os.path.join(ENGINE_DIR, 'testing', 'dart'),
    os.path.join(ENGINE_DIR, 'testing', 'scenario_app'),
    os.path.join(ENGINE_DIR, 'testing', 'skia_gold_client'),
    os.path.join(ENGINE_DIR, 'testing', 'smoke_test_failure'),
    os.path.join(ENGINE_DIR, 'testing', 'symbols'),
    os.path.join(ENGINE_DIR, 'tools', 'android_lint'),
    os.path.join(ENGINE_DIR, 'tools', 'api_check'),
    os.path.join(ENGINE_DIR, 'tools', 'build_bucket_golden_scraper'),
    os.path.join(ENGINE_DIR, 'tools', 'clang_tidy'),
    os.path.join(ENGINE_DIR, 'tools', 'clangd_check'),
    os.path.join(ENGINE_DIR, 'tools', 'compare_goldens'),
    os.path.join(ENGINE_DIR, 'tools', 'const_finder'),
    os.path.join(ENGINE_DIR, 'tools', 'dir_contents_diff'),
    os.path.join(ENGINE_DIR, 'tools', 'engine_tool'),
    os.path.join(ENGINE_DIR, 'tools', 'gen_web_locale_keymap'),
    os.path.join(ENGINE_DIR, 'tools', 'githooks'),
    os.path.join(ENGINE_DIR, 'tools', 'golden_tests_harvester'),
    os.path.join(ENGINE_DIR, 'tools', 'header_guard_check'),
    os.path.join(ENGINE_DIR, 'tools', 'licenses'),
    os.path.join(ENGINE_DIR, 'tools', 'path_ops', 'dart'),
    os.path.join(ENGINE_DIR, 'tools', 'pkg', 'engine_build_configs'),
    os.path.join(ENGINE_DIR, 'tools', 'pkg', 'engine_repo_tools'),
    os.path.join(ENGINE_DIR, 'tools', 'pkg', 'git_repo_tools'),
    os.path.join(ENGINE_DIR, 'tools', 'pkg', 'process_fakes'),
]


def fetch_package(pub, package):
  try:
    subprocess.check_output(pub, cwd=package, stderr=subprocess.STDOUT)
  except subprocess.CalledProcessError as err:
    print(
        '"%s" failed in "%s" with status %d:\n%s' %
        (' '.join(pub), package, err.returncode, err.output)
    )
    return 1
  return 0


def package_uses_workspace_resolution(package):
  pubspec = os.path.join(package, 'pubspec.yaml')

  with open(pubspec) as pubspec_file:
    return yaml.safe_load(pubspec_file).get('resolution') == 'workspace'


def check_package_config(package):
  package_config = os.path.join(package, '.dart_tool', 'package_config.json')
  pub_count = 0
  with open(package_config) as config_file:
    data_dict = json.load(config_file)
    packages_data = data_dict['packages']
    for package_data in packages_data:
      package_uri = package_data['rootUri']
      package_name = package_data['name']
      if '.pub-cache' in package_uri and ('pub.dartlang.org' in package_uri or
                                          'pub.dev' in package_uri):
        print('Error: package "%s" was fetched from pub' % package_name)
        pub_count = pub_count + 1
  if pub_count > 0:
    print('Error: %d packages were fetched from pub for %s' % (pub_count, package))
    print(
        'Please fix the pubspec.yaml for %s '
        'so that all dependencies are path dependencies' % package
    )
  return pub_count


EXCLUDED_DIRS = [
    os.path.join(ENGINE_DIR, 'lib'),
    os.path.join(ENGINE_DIR, 'prebuilts'),
    os.path.join(ENGINE_DIR, 'shell', 'platform', 'fuchsia'),
    os.path.join(ENGINE_DIR, 'shell', 'vmservice'),
    os.path.join(ENGINE_DIR, 'sky', 'packages'),
    os.path.join(ENGINE_DIR, 'third_party'),
    os.path.join(ENGINE_DIR, 'web_sdk'),
]


# Returns a list of paths to directories containing pubspec.yaml files that
# are not listed in ALL_PACKAGES. Directory trees under the paths in
# EXCLUDED_DIRS are skipped.
def find_unlisted_packages():
  unlisted = []
  for root, dirs, files in os.walk(ENGINE_DIR):
    excluded = []
    for dirname in dirs:
      full_dirname = os.path.join(root, dirname)
      if full_dirname in EXCLUDED_DIRS:
        excluded.append(dirname)
    for exclude in excluded:
      dirs.remove(exclude)
    for filename in files:
      if filename == 'pubspec.yaml':
        if root not in ALL_PACKAGES:
          unlisted.append(root)
  return unlisted


def delete_config_files():
  # Find all package_config.json that are not under version control.
  gitcmd = ['git', 'ls-files', '-o', '**/.dart_tool/package_config.json']
  files_to_delete = subprocess.check_output(
      gitcmd, cwd=ENGINE_DIR, stderr=subprocess.STDOUT, text=True
  ).splitlines()
  for file in files_to_delete:
    file_path = os.path.join(ENGINE_DIR, file)
    if os.path.exists(file_path):
      print('Deleting %s...' % file)
      os.remove(file_path)


def main():
  # Intentionally use the Dart SDK prebuilt instead of the Flutter prebuilt
  # (i.e. prebuilts/{platform}/dart-sdk/bin/dart) because the script has to run
  # in a monorepo build *before* the newer Dart SDK has been built from source.
  dart_sdk_bin = os.path.join(
      SRC_ROOT, 'flutter', 'third_party', 'dart', 'tools', 'sdks', 'dart-sdk', 'bin'
  )

  # Delete all package_config.json files. These may be stale.
  # Required ones will be regenerated fresh below.
  delete_config_files()

  # Ensure all relevant packages are listed in ALL_PACKAGES.
  unlisted = find_unlisted_packages()
  if len(unlisted) > 0:
    for pkg in unlisted:
      print('The Dart package "%s" must be checked in flutter/tools/pub_get_offline.py' % pkg)
    return 1

  dart = 'dart'
  if os.name == 'nt':
    dart = 'dart.exe'
  pubcmd = [os.path.join(dart_sdk_bin, dart), 'pub', '--suppress-analytics', 'get', '--offline']

  pub_count = 0
  for package in ALL_PACKAGES:
    if fetch_package(pubcmd, package) != 0:
      return 1
    if not package_uses_workspace_resolution(package):
      pub_count = pub_count + check_package_config(package)

  if pub_count > 0:
    return 1

  return 0


if __name__ == '__main__':
  sys.exit(main())
