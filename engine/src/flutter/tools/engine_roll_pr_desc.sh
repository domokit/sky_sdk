#!/bin/bash

if [[ $1 == '' ]]; then
  echo 'Usage: engine_roll_pr_desc.sh <from git hash>..<to git hash>'
  exit 1
fi
git log --oneline --no-merges --no-color $1 | sed 's/^/flutter\/engine@/g' |  sed -e 's/(\(#[0-9]*)\)/\(flutter\/engine\1/g'
