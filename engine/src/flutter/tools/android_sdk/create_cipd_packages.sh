#!/bin/bash

# This script requires depot_tools to be on path.

print_usage () {
  echo "Usage:"
  echo "  ./create_cipd_packages.sh <VERSION_TAG> [PATH_TO_SDK_DIR]"
  echo "    Downloads, packages, and uploads Android SDK packages where:"
  echo "      - VERSION_TAG is the tag of the cipd packages, e.g. 28r6 or 31v1. Must contain"
  echo "                    only lowercase letters and numbers."
  echo "      - PATH_TO_SDK_DIR is the path to the sdk folder. If omitted, this defaults to"
  echo "                      your ANDROID_SDK_ROOT environment variable."
  echo "  ./create_cipd_packages.sh list"
  echo "    Lists the available packages for use in 'packages.txt'"
  echo ""
  echo "This script downloads the packages specified in packages.txt and uploads"
  echo "them to CIPD for linux, mac, and windows."
  echo "To confirm you have write permissions run 'cipd acl-check flutter/android/sdk/all/ -writer'."
  echo ""
  echo "Manage the packages to download in 'packages.txt'. You can use"
  echo "'sdkmanager --list --include_obsolete' in cmdline-tools to list all available packages."
  echo "Packages should be listed in the format of <package-name>:<directory-to-upload>."
  echo "For example, build-tools;31.0.0:build-tools"
  echo "Multiple directories to upload can be specified by delimiting by additional ':'"
  echo ""
  echo "This script expects the cmdline-tools to be installed in your specified PATH_TO_SDK_DIR"
  echo "and should only be run on linux or macos hosts."
}

first_argument=$1
# Validate version or argument is provided.
if [[ $first_argument == "" ]]; then
  print_usage
  exit 1
fi

# Validate version contains only lower case letters and numbers.
if ! [[ $first_argument =~ ^[[:lower:][:digit:]]+$ ]]; then
  echo "Version tag can only consist of lower case letters and digits.";
  print_usage
  exit 1
fi

# Validate environment has cipd installed.
if [[ `which cipd` == "" ]]; then
  echo "'cipd' command not found. depot_tools should be on the path."
  exit 1
fi

sdk_path=${2:-$ANDROID_SDK_ROOT}

# Validate directory contains all SDK packages
if [[ ! -d "$sdk_path" ]]; then
  echo "Android SDK at '$sdk_path' not found."
  print_usage
  exit 1
fi

# Validate caller has cipd.
if [[ ! -d "$sdk_path/cmdline-tools" ]]; then
  echo "SDK directory does not contain $sdk_path/cmdline-tools."
  print_usage
  exit 1
fi

platforms=("linux" "macosx" "windows")
package_file_name="packages.txt"

# Find the sdkmanager in cmdline-tools. We default to using latest if available.
sdkmanager_path="$sdk_path/cmdline-tools/latest/bin/sdkmanager"
find_results=()
while IFS= read -r line; do
  find_results+=("$line")
done < <(find "$sdk_path/cmdline-tools" -name sdkmanager)
i=0
while [ ! -f "$sdkmanager_path" ]; do
  if [ $i -ge ${#find_results[@]} ]; then
    echo "Unable to find sdkmanager in the SDK directory. Please ensure cmdline-tools is installed."
    exit 1
  fi
  sdkmanager_path="${find_results[$i]}"
  echo $sdkmanager_path
  ((i++))
done

# list available packages
if [ $first_argument == "list" ]; then
  $sdkmanager_path  --list --include_obsolete
  exit 0
fi

# We create a new temporary SDK directory because the default working directory
# tends to not update/re-download packages if they are being used. This guarantees
# a clean install of Android SDK.
temp_dir=`mktemp -d -t android_sdkXXXX`

for platform in "${platforms[@]}"; do
  sdk_root="$temp_dir/sdk_$platform"
  upload_dir="$temp_dir/upload_$platform"
  echo "Creating temporary working directory for $platform: $sdk_root"
  mkdir $sdk_root
  mkdir $upload_dir
  mkdir $upload_dir/sdk
  export REPO_OS_OVERRIDE=$platform

  # Download all the packages with sdkmanager.
  for package in $(cat $package_file_name); do
    echo $package
    split=(${package//:/ })
    echo "Installing ${split[0]}"
    yes "y" | $sdkmanager_path --sdk_root=$sdk_root ${split[0]}

    # We copy only the relevant directories to a temporary dir
    # for upload. sdkmanager creates extra files that we don't need.
    array_length=${#split[@]}
    for (( i=1; i<${array_length}; i++ )); do
      cp -r "$sdk_root/${split[$i]}" "$upload_dir/sdk"
    done
  done

  # Special treatment for NDK to move to expected directory.
  # Instead of the ndk being in `sdk/ndk/<major>.<minor>.<patch>/`, it will be
  # in `ndk/`.
  # This simplifies the build scripts, and enables version difference between
  # the Dart and Flutter build while reusing the same build rules.
  # See https://github.com/flutter/flutter/issues/136666#issuecomment-1805467578
  mv $upload_dir/sdk/ndk $upload_dir/ndk-bundle
  ndk_sub_paths=`find $upload_dir/ndk-bundle -maxdepth 1 -type d`
  ndk_sub_paths_arr=($ndk_sub_paths)
  mv ${ndk_sub_paths_arr[1]} $upload_dir/ndk
  rm -rf $upload_dir/ndk-bundle

  if [[ ! -d "$upload_dir/ndk" ]]; then
    echo "Failure to bundle ndk for platform"
    exit 1
  fi

  # Accept all licenses to ensure they are generated and uploaded.
  yes "y" | $sdkmanager_path --licenses --sdk_root=$sdk_root
  cp -r "$sdk_root/licenses" "$upload_dir/sdk"

  archs=("amd64")
  if [[ $platform == "macosx" ]]; then
    # Upload an arm64 version for M1 macs
    archs=("amd64" "arm64")
  fi
  for arch in "${archs[@]}"; do
    # Mac uses a different sdkmanager name than the platform name used in gn.
    cipd_name="$platform-$arch"
    if [[ $platform == "macosx" ]]; then
      cipd_name="mac-$arch"
    fi

    echo "Uploading $upload_dir as $cipd_name to CIPD"
    cipd create -in $upload_dir -name "flutter/android/sdk/all/$cipd_name" -install-mode copy -tag version:$first_argument
  done

  rm -rf $sdk_root
  rm -rf $upload_dir

  # This variable changes the behvaior of sdkmanager.
  # Unset to clean up after script.
  unset REPO_OS_OVERRIDE
done
rm -rf $temp_dir
