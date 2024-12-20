Native Activity
===============

Executables packaged as native activities in an Android APK. These activities
contain no Java code.

To create an APK of your existing `exectuable` target, replace `exectuable` with
`native_activity_apk` from the `native_activity.gni` template and give it an
`apk_name`.

## Example

```
native_activity_apk("apk_unittests") {
  apk_name = "toolkit_unittests"

  testonly = true

  sources = [ "toolkit_android_unittests.cc" ]

  deps = [
    ":unittests_lib",
    "//flutter/testing/android/native_activity:gtest_activity",
  ]
}
```

One of the translation units in must contain an implementation of
`flutter::NativeActivityMain`. The `gtest_activity` target contains an
implementation of an activity that run GoogleTests. That can be used off the
shelf.
