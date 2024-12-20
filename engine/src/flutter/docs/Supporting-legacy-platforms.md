## Building ARMv7 (iOS) & armeabi v7a (Android) with Xcode10

In Xcode10, the i386 architecture is deprecated for macOS, so building the Flutter engine for armv7/armeabi-v7a fails. Specifically, libraries like CoreFoundation contain only code for the x86_64 architecture.

![iOS ARMv7](https://user-images.githubusercontent.com/817851/45751101-e7a54980-bc43-11e8-833f-b6458c9a4762.png)

![Android armeabi-v7a](https://user-images.githubusercontent.com/817851/45751099-e70cb300-bc43-11e8-97fa-a877dff5449d.png)

To address this, get the MacOS 10.13 SDK from Xcode 9.x from [Apple](https://developer.apple.com/download/more/), and extract the SDK components from the `.xip` file. Uncompress the SDK into `/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs` and name the SDK `MacOSX10.13.sdk`:

![Uncompressed SDK in Xcode10](https://user-images.githubusercontent.com/817851/45752211-47512400-bc47-11e8-88fe-b738ac53831f.png)

To check if the logic is fine, run command below:

```bash
python your-flutter-engine-path/engine/src/build/mac/find_sdk.py 10.12
```

When `find_sdk.py` return 10.13, the ninja build will succeed for gen_snapshot (i386), Flutter.framework (ARMv7) and libflutter.so (armeabi-v7a).

## Build Flutter engine for 32bit iOS simulator on modern Mac(x86_64)

To build the Flutter engine for iOS simulator on a modern Mac(x86_64), the gn command will generate a `target_cpu` value with x64. Henceforth, the Flutter.framework and gen_snapshot will be x86_64.
However, sometimes you may want to develop Flutter on a 32bit simulator(like iPhone5), you will need both Flutter.framework and gen_snapshot to be i386.

Follow instruction below to change the default behavior in gn command:
1. Edit your-flutter-engine-path/engine/src/flutter/tools/gn
![Edit gn](https://user-images.githubusercontent.com/817851/49006557-57840300-f1a4-11e8-850a-d019dc854bbd.png)