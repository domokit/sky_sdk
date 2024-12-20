#!/bin/bash
set -e
set -x

if [[ -z $ENGINE_PATH ]]
then
  echo "Please set ENGINE_PATH environment variable."
  exit 1
fi


# Go to the engine git repo to get the date of the latest commit.
cd $ENGINE_PATH/src/flutter

ENGINE_COMMIT=`git rev-parse HEAD`
echo "Using engine commit: $ENGINE_COMMIT"

if [[ -z $FLUTTER_CLONE_REPO_PATH ]]
then
  echo "Please set FLUTTER_CLONE_REPO_PATH environment variable."
  exit 1
else
  cd $FLUTTER_CLONE_REPO_PATH
fi

if [[ $GIT_BRANCH =~ ^flutter-.*-candidate.*$ ]]
then
  # Coming from presubmit and assuming the correct branch has been already checked out.
  COMMIT_NO=`git rev-parse HEAD`
else
  # Try to get release branch from the checkout.
  RELEASE_BRANCH=`git branch -a --contains $ENGINE_COMMIT | grep 'flutter-.*-candidate.*' || true`
  if [[ -z $RELEASE_BRANCH ]]
  then
    # If this is not a release branch commit get latest commit's time for the engine repo.
    # Use date based on local time otherwise timezones might get mixed.
    LATEST_COMMIT_TIME_ENGINE=`git log -1 --date=local --format="%cd"`
    echo "Latest commit time on engine found as $LATEST_COMMIT_TIME_ENGINE"

    # Get the time of the youngest commit older than engine commit.
    # Git log uses commit date not the author date.
    # Before makes the comparison considering the timezone as well.
    COMMIT_NO=`git log --before="$LATEST_COMMIT_TIME_ENGINE" -n 1 | grep commit | cut -d ' ' -f2`
  else
    COMMIT_NO=`git rev-parse $RELEASE_BRANCH`
    git checkout $RELEASE_BRANCH
  fi
fi

echo "Using the flutter/flutter commit $COMMIT_NO";
git reset --hard $COMMIT_NO
# Write the commit number to a file. This file will be read by the LUCI recipe.
echo "$COMMIT_NO" >> flutter_ref.txt

# Print out the flutter version for troubleshooting
$FLUTTER_CLONE_REPO_PATH/bin/flutter --version -v
