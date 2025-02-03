When new Android versions become available, the following steps should be taken in order to fully support the new API version in the Flutter engine. These steps only change the engine to build against and target the new API and do not guarantee that everything works with the changes in the new Android API.

## Update the [Buildroot](https://github.com/flutter/buildroot):

In `build/config/android/config.gni`, edit `default_android_sdk_version` and `default_android_sdk_build_tools_version` to the new Android version.

**NOTE:** There is an ongoing effort to move the `flutter/buildroot` repo into the `flutter/engine` repo, so this file may have moved by the time you are following these instructions.

## Upload new SDK and other dependencies to [CIPD](https://chrome-infra-packages.appspot.com/p/flutter/android):

Flutter now includes a script to download, package, and upload the Android SDK to CIPD. These CIPD packages are then used as dependencies by the Flutter engine and recipes (go/flutter-luci-recipes) so that there is a stable archived version of the Android SDK to depend on. The script is located in the Flutter engine repo under `tools/android-sdk/create_cipd_packages.sh`.

To upload packages to CIPD (either with the script or manually), the  `flutter-cipd-writers` role is required in order to complete this operation. Googlers can apply for access [here](https://grants.corp.google.com/#/grants?request=8h%2Fflutter-cipd-writers). Once this role is granted, `cipd auth-login` must be run in order for cipd to update the user's available roles.

Edit `tools/android-sdk/packages.txt` to refer to the updated versions you want. The format for each line in packages.txt is `<package_name>:<subdirectory_to_upload>`. Typically, each `<package_name>` should be updated to the latest available version which can be found with the `sdkmanager --list --include_obsolete`. `sdkmanager` can be found in your `commandline-tools` package of the android sdk. Additionally, set the `ANDROID_SDK_ROOT` environment variable to your local Android SDK installation.

The script must be run on a Linux or Mac host. Run:

    `$ cd tools/android-sdk && ./create_cipd_packages.sh <new_version_tag> <path_to_your_local_android_sdk>`

This script will download and re-upload the entire SDK, so it may take a long time to complete. `cmdline-tools` should be installed in your local SDK as the script uses `sdkmanager`. Once the CIPD packages are finished uploading, you can update the SDK version tag used in `.ci.yaml`, `DEPS`, and elsewhere as described [below](#update-the-engine).

It is no longer recommended to upload CIPD Android SDK packages manually, but if it must be done, run the following commands to zip and upload each package to CIPD:

    `$ cipd create -in <your-android-dir>/Android/sdk/<some_package> -name flutter/android/sdk/<some_package> -tag version:<new-version-tag>`

Typically, `<your-android-dir>` is in your home directory under `~/Library/Android`. The `<new-version-tag>` is what you will use to specify the new package you uploaded in the Flutter engine `DEPS` file.

## Update the [Engine](https://github.com/flutter/engine):

### Update the compile and target SDK versions we use

Modify the following files as described:
* `DEPS`: Roll buildroot hash to that of the PR you used to [update the buildroot](#update-the-buildroot)
* `DEPS`: Change the version parameter under `flutter/android/sdk/all/${{platform}}` to the newly uploaded CIPD version tag, e.g. `'version': 'version:30r2'`
* `tools/javadoc/gen_javadoc.py`: Bump the reference to `android-XX` in `classpath` to the latest version.
* `tools/cipd/android_embedding_bundle/build.gradle`: Bump `compileSdkVersion XX` to the latest version.
* `shell/platform/android/test_runner/build.gradle`: Bump `compileSdkVersion XX` to the latest version.
* `shell/platform/android/AndroidManifest.xml`: Bump `android:targetSdkVersion=XX` to the latest version.
* `testing/android/native_activity/native_activity.gni`: Bump the reference to `build-tools/XX` in `android_buildtools` to the latest **build-tools** version and the reference to `android-XX` in `android_jar` to the latest version.

This list may become outdated, so be sure to change any references to the old SDK version to the latest version in `build.gradle` files across the repo.

## Next Steps: Update the Framework, Examples and Samples

* Templates in [the framework](https://github.com/flutter/flutter): Change `targetSdkVersion` in various `build.gradle.tmpl` files to use the new API version
* [Examples](https://github.com/flutter/flutter/tree/main/examples) and [samples](https://github.com/flutter/samples): Change `targetSdkVersion` in `android/app/build.gradle` for each project to the new API version.
