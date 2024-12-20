# Updating the Gradle version used in flutter/engine repo

The instructions in this README explain how to create a CIPD package that
contains the [Gradle](https://gradle.org/) build-time dependency of the Android embedding of the engine.
The Android embedder is shipped to Flutter end-users, but Gradle is not.

## Requirements

1. Ensure that you have [Depot tools](http://commondatastorage.googleapis.com/chrome-infra-docs/flat/depot_tools/docs/html/depot_tools_tutorial.html#_setting_up)
on your path. If you have a flutter/engine checkout, then you should already have it on your path.
2. Ensure that you have write access for CIPD. See go/flutter-luci-cipd for instructions on how to
request access.

## Steps to download and verify new Gradle version
These steps use Gradle version 7.5.1 as an example. Please replace 7.5.1 with the actual
Gradle version that you wish to use.

1. Download the new version of Gradle you'd like from [the Gradle website](https://gradle.org/releases/).
Please download the "complete" version.
2. Verify the checksum. To do this, first check the checksum of the complete (-all) ZIP Gradle version you
downloaded from https://gradle.org/release-checksums/. Then, run `shasum -a 256 gradle-7.5.1-all.zip` to
check the checksum of the Gradle version you downloaded. Verify that the checksum outputted by this
command and the one from the Gradle website match.
3. Unzip the Gradle download into a folder by running `unzip gradle-7.5.1-all.zip`.

## Steps to upload new Gradle version to CIPD
These steps use Gradle version 7.5.1 as an example. Please replace 7.5.1 with the actual
Gradle version that you downloaded and verified.

1. Authenticate with CIPD by running `cipd auth-login`.
2. Run `cipd create -in gradle-7.5.1 -install-mode copy -tag version:7.5.1 -name flutter/gradle` to
upload the new Gradle version to CIPD.
3. Update the `engine/src/flutter/DEPS` Gradle entry (which should look something like [this](https://github.com/flutter/engine/blob/4caaab9f2502481b606b930abeea4a361022fa16/DEPS#L732-L743))
to contain the tag from the command above (version:7.5.1).
4. Run `gclient sync` to verify that the dependency can be fetched.

## Useful links
* CIPD Gradle package: https://chrome-infra-packages.appspot.com/p/flutter/gradle/+/
* Gradle releases: https://gradle.org/releases/
* Gradle distribution and wrapper JAR checksum reference: https://gradle.org/release-checksums/