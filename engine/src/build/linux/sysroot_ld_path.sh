#!/bin/sh
# Copyright (c) 2013 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# Reads etc/ld.so.conf and/or etc/ld.so.conf.d/*.conf and returns the
# appropriate linker flags.
#
#  sysroot_ld_path.sh /abspath/to/sysroot
#

log_error_and_exit() {
  echo $0: $@
  exit 1
}

process_entry() {
  if [ -z "$1" ] || [ -z "$2" ]; then
    log_error_and_exit "bad arguments to process_entry()"
  fi
  local root="$1"
  local localpath="$2"

  echo $localpath | grep -qs '^/'
  if [ $? -ne 0 ]; then
    log_error_and_exit $localpath does not start with /
  fi
  local entry="$root$localpath"
  echo -L$entry
  echo -Wl,-rpath-link=$entry
}

process_ld_so_conf() {
  if [ -z "$1" ] || [ -z "$2" ]; then
    log_error_and_exit "bad arguments to process_ld_so_conf()"
  fi
  local root="$1"
  local ld_so_conf="$2"

  # ld.so.conf may include relative include paths. pushd is a bashism.
  local saved_pwd=$(pwd)
  cd $(dirname "$ld_so_conf")

  cat "$ld_so_conf" | \
    while read ENTRY; do
      echo "$ENTRY" | grep -qs ^include
      if [ $? -eq 0 ]; then
        local included_files=$(echo "$ENTRY" | sed 's/^include //')
        echo "$included_files" | grep -qs ^/
        if [ $? -eq 0 ]; then
          if ls $root$included_files >/dev/null 2>&1 ; then
            for inc_file in $root$included_files; do
              process_ld_so_conf "$root" "$inc_file"
            done
          fi
        else
          if ls $(pwd)/$included_files >/dev/null 2>&1 ; then
            for inc_file in $(pwd)/$included_files; do
              process_ld_so_conf "$root" "$inc_file"
            done
          fi
        fi
        continue
      fi

      echo "$ENTRY" | grep -qs ^/
      if [ $? -eq 0 ]; then
        process_entry "$root" "$ENTRY"
      fi
    done

  # popd is a bashism
  cd "$saved_pwd"
}

# Main

if [ $# -ne 1 ]; then
  echo Usage $0 /abspath/to/sysroot
  exit 1
fi

echo $1 | grep -qs ' '
if [ $? -eq 0 ]; then
  log_error_and_exit $1 contains whitespace.
fi

LD_SO_CONF="$1/etc/ld.so.conf"
LD_SO_CONF_D="$1/etc/ld.so.conf.d"

if [ -e "$LD_SO_CONF" ]; then
  process_ld_so_conf "$1" "$LD_SO_CONF" | xargs echo
elif [ -e "$LD_SO_CONF_D" ]; then
  find "$LD_SO_CONF_D" -maxdepth 1 -name '*.conf' -print -quit > /dev/null
  if [ $? -eq 0 ]; then
    for entry in $LD_SO_CONF_D/*.conf; do
      process_ld_so_conf "$1" "$entry"
    done | xargs echo
  fi
fi
