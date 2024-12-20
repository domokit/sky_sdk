# Android Vulkan Validation Layers

If you want to run Vulkan Validation Layers with a custom engine build you need
to add the `--enable-vulkan-validation-layers` to the `gn` invocation to make
sure the layers are built and injected into the Flutter jar.

Example:

```sh
flutter/tools/gn \
  --runtime-mode=debug \
  --enable-vulkan-validation-layers \
  --no-lto \
  --unoptimized \
  --android \
  --android-cpu=arm64
```

Then adding the following field to the
`android/app/src/main/AndroidManifest.xml` under the `<application>` tag will turn
them on:

```xml
<meta-data
    android:name="io.flutter.embedding.android.EnableVulkanValidation"
    android:value="true" />
```
