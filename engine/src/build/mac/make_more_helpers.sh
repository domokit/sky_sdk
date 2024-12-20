#!/bin/bash

# Copyright (c) 2012 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# Usage: make_more_helpers.sh <directory_within_contents> <app_name>
#
# This script creates additional helper .app bundles for Chromium, based on
# the existing helper .app bundle, changing their Mach-O header's flags to
# enable and disable various features. Based on Chromium Helper.app, it will
# create Chromium Helper EH.app, which has the MH_NO_HEAP_EXECUTION bit
# cleared to support Chromium child processes that require an executable heap,
# and Chromium Helper NP.app, which has the MH_PIE bit cleared to support
# Chromium child processes that cannot tolerate ASLR.
#
# This script expects to be called from the chrome_exe target as a postbuild,
# and operates directly within the built-up browser app's versioned directory.
#
# Each helper is adjusted by giving it the proper bundle name, renaming the
# executable, adjusting several Info.plist keys, and changing the executable's
# Mach-O flags.

set -eu

make_helper() {
  local containing_dir="${1}"
  local app_name="${2}"
  local feature="${3}"
  local flags="${4}"

  local helper_name="${app_name} Helper"
  local helper_stem="${containing_dir}/${helper_name}"
  local original_helper="${helper_stem}.app"
  if [[ ! -d "${original_helper}" ]]; then
    echo "${0}: error: ${original_helper} is a required directory" >& 2
    exit 1
  fi
  local original_helper_exe="${original_helper}/Contents/MacOS/${helper_name}"
  if [[ ! -f "${original_helper_exe}" ]]; then
    echo "${0}: error: ${original_helper_exe} is a required file" >& 2
    exit 1
  fi

  local feature_helper="${helper_stem} ${feature}.app"

  rsync -acC --delete --include '*.so' "${original_helper}/" "${feature_helper}"

  local helper_feature="${helper_name} ${feature}"
  local helper_feature_exe="${feature_helper}/Contents/MacOS/${helper_feature}"
  mv "${feature_helper}/Contents/MacOS/${helper_name}" "${helper_feature_exe}"

  local change_flags="$(dirname "${0}")/change_mach_o_flags.py"
  "${change_flags}" ${flags} "${helper_feature_exe}"

  local feature_info="${feature_helper}/Contents/Info"
  local feature_info_plist="${feature_info}.plist"

  defaults write "${feature_info}" "CFBundleDisplayName" "${helper_feature}"
  defaults write "${feature_info}" "CFBundleExecutable" "${helper_feature}"

  cfbundleid="$(defaults read "${feature_info}" "CFBundleIdentifier")"
  feature_cfbundleid="${cfbundleid}.${feature}"
  defaults write "${feature_info}" "CFBundleIdentifier" "${feature_cfbundleid}"

  cfbundlename="$(defaults read "${feature_info}" "CFBundleName")"
  feature_cfbundlename="${cfbundlename} ${feature}"
  defaults write "${feature_info}" "CFBundleName" "${feature_cfbundlename}"

  # As usual, defaults might have put the plist into whatever format excites
  # it, but Info.plists get converted back to the expected XML format.
  plutil -convert xml1 "${feature_info_plist}"

  # `defaults` also changes the file permissions, so make the file
  # world-readable again.
  chmod a+r "${feature_info_plist}"
}

if [[ ${#} -ne 2 ]]; then
  echo "usage: ${0} <directory_within_contents> <app_name>" >& 2
  exit 1
fi

DIRECTORY_WITHIN_CONTENTS="${1}"
APP_NAME="${2}"

CONTENTS_DIR="${BUILT_PRODUCTS_DIR}/${CONTENTS_FOLDER_PATH}"
CONTAINING_DIR="${CONTENTS_DIR}/${DIRECTORY_WITHIN_CONTENTS}"

make_helper "${CONTAINING_DIR}" "${APP_NAME}" "EH" "--executable-heap"
make_helper "${CONTAINING_DIR}" "${APP_NAME}" "NP" "--no-pie"
