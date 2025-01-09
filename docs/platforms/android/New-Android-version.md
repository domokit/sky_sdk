# Support a new Android API level Flutter (go/flutter-android-new-api-level)

## Objective

Provides a list of areas to consider and examples of former work for how to update Flutter to support a new version of the Android API. This happens every fall and Flutter developers expect to build against the latest versions quickly after they are available.

### Overview

#### Bump compile and target SDK versions in samples

Samples, especially add to app samples, represent apps that mirror the first types of users we see adopt new Android APIs.

Example PR: https://github.com/flutter/samples/pull/2368

#### New Android features

New Android features can require a broad spectrum of work. Some will require nothing from flutter. Some will require a lot of work, such as the support for “back preview”. The Android team generally needs to be aware and schedule work ahead of time.

#### Update Gradle/AGP support

Sometimes newer versions of gradle are required to build without warning against a new version of Android API. The warning looks like:

```
WARNING:We recommend using a newer Android Gradle plugin to use compileSdkPreview = "somenamedversion"
This Android Gradle plugin (X.X.X) was tested up to compileSdk = XX
```

Bump the gradle version used in the engine.

#### Update Robolectric

Robolectric is a dependency that allows us to write unit tests that run on a local development machine against the Android API surface without being on an Android device.
Update what version of Robolectric we use in framework, engine and packages.

Example flutter/packages PR: https://github.com/flutter/packages/pull/4018, Example flutter/engine PR: https://github.com/flutter/engine/pull/42965

#### Update CI

Update emulator and/or physical device testing to use the new Android API version for framework, engine and packages.

Firebase test lab emulators and physical devices need support from the firebase team.
Run command `gcloud firebase test Android models list` [documentation](https://firebase.google.com/docs/test-lab/Android/available-testing-devices) to see available devices.

Flutter managed emulators (engine) require a new AVD image to one that supports the new API. To do this, first locate the desired Android Virtual Device (AVD) from https://chrome-infra-packages.appspot.com/p/chromium/tools/android/avd/linux-amd64/. You should look at the most recently updated AVD and verify that it has the desired generic_android<API#>.textpb for the API version you are modified the engine to support. Then, determine its Instance Identifier.

In the framework, update the following `.ci.yaml` entry:

```yaml
  linux_android_emu:
    properties:
      contexts: >-
        [
          "android_virtual_device"
        ]
      dependencies: >-
        [
          ...
          {"dependency": "android_virtual_device", "version": "android_<API#>_google_apis_x64.textpb"},
          {"dependency": "avd_cipd_version", "version": "build_id:<Instance ID>"},
        ]
      ...
```

In the engine, you may follow [Upgrading Engine's Android API version](https://github.com/flutter/flutter/blob/main/docs/platforms/android/Upgrading-Engine's-Android-API-version.md) to update the AVD dependency.

Example Framework PR: https://github.com/flutter/flutter/pull/152498, Example Engine PR: https://github.com/flutter/engine/pull/54186

In flutter/packages, modify https://github.com/flutter/packages/blob/main/.cirrus.yml “firebase_test_lab_script”
Specifically the value for “--device” `./script/tool_runner.sh firebase-test-lab --device model=redfin,version=30 --exclude=script/configs/exclude_integration_Android.yaml`.

Example PR: https://github.com/flutter/packages/pull/4430

#### Update documentation

Update documentation page to indicate the new API is tested
https://docs.flutter.dev/reference/supported-platforms.

#### Modify defaults

**In flutter/flutter:** Update default compile SDK version and target SDK version to the new API value.
[Code here](../../../packages/flutter_tools/gradle/src/main/groovy/flutter.groovy).
Follow comments in that file to update other locations that are assumed to be the same.
Example bumping min SDK which is similar but different: https://github.com/flutter/flutter/pull/125515

**In flutter/buildroot:**

- Upload new Android SDK version to CIPD (steps at end of document)
- Update default Android SDK version
https://github.com/flutter/buildroot/blob/7984a08044b94bfb4c466ac881c6b56fdfe9148b/build/config/Android/config.gni#L19

**In flutter/engine:**

- Update Android SDK in DEPS file
https://github.com/flutter/engine/blob/9289cb6a36aa86990e3ffe0f20324dafa38e7c11/DEPS#L731

- Update buildroot version in DEPS file to consume the changes in the flutter/buildroot steps above
https://github.com/flutter/engine/blob/9289cb6a36aa86990e3ffe0f20324dafa38e7c11/DEPS#L260

**In flutter/packages:**
- Set examples to build with the new API.
- Update `create_all_packages` to use new api as compile sdk [source](https://github.com/flutter/packages/blob/3515abab07d0bb2441277f43c2411c9b5e4ecf94/script/tool/lib/src/create_all_packages_app_command.dart#L245-L249).

#### Test “Integration Test” package

Integration test is a package shipped in the Flutter tool for running integration tests on Flutter apps. Ensure that the integration test package has an example that targets the new API level on the most recent published stable version of the Flutter tool.

#### Related documents

[Emulators for Flutter Android Testing (PUBLICLY SHARED)](https://docs.google.com/document/d/10wYUcLcSTF4Epg2EUGoBqOkkOe4zxKHvYKjXFZAOgGs/edit?resourcekey=0-pltjPvEtVezXDADMbUwFHQ)

### Additional notes

#### Upload new SDK version to CIPD and consume in buildroot

See [Upgrading Engine's Android API version](https://github.com/flutter/flutter/blob/main/docs/platforms/android/Upgrading-Engine's-Android-API-version.md) for instructions, as this work is also required to build the engine against the new Android version.
