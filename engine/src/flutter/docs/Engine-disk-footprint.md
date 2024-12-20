## Treemaps

For each commit to [flutter/engine](https://github.com/flutter/engine) the Chromebots generate treemaps illustrating the sizes of the individual components within release builds of `libflutter.so`. The treemap is uploaded to Google Cloud Storage and linked from the [LUCI](https://ci.chromium.org/p/flutter/g/engine/console) console: Select a "Linux aot" build and search for "Open Treemap".

Alternatively, a link to a treemap can be constructed as follows:

`https://storage.googleapis.com/flutter_infra_release/flutter/<REVISION>/<VARIANT>/sizes/index.html` where:
* `<REVISION>` is the git hash from [flutter/engine](https://github.com/flutter/engine) for which you want the treemap, and
* `<VARIANT>` can be any android release build, e.g. `android-arm-release` or `android-arm64-release`.

## Benchmarks

In [devicelab](https://github.com/flutter/flutter/tree/main/dev/devicelab) we run various benchmarks to track the APK/IPA sizes and various (engine) artifacts contained within. These benchmarks run for every commit to [flutter/flutter](https://github.com/flutter/flutter) and are visible on our [build dashboard](https://flutter-dashboard.appspot.com/). The most relevant benchmarks for engine size are:

* APK/IPA size of Flutter Gallery
  * Android: `flutter_gallery_android__compile/release_size_bytes`
  * iOS: `flutter_gallery_ios__compile/release_size_bytes`
* APK/IPA size of minimal hello_world app
  * Android: `hello_world_android__compile/release_size_bytes`
  * iOS: `hello_world_ios__compile/release_size_bytes`
* Size of bundled `icudtl.dat`
  * Compressed in APK: `hello_world_android__compile/icudtl_compressed_bytes`
  * Uncompressed: `hello_world_android__compile/icudtl_uncompressed_bytes`
* Size of bundled `libflutter.so` (release mode)
  * Compressed in APK: `hello_world_android__compile/libflutter_compressed_bytes`
  * Uncompressed: `hello_world_android__compile/libflutter_uncompressed_bytes`
* Size of VM & isolate snapshots (data and instructions)
  * Compressed in APK: `hello_world_android__compile/snapshot_compressed_bytes`
  * Uncompressed: `hello_world_android__compile/snapshot_uncompressed_bytes`

## Comparing AOT Snapshot Sizes

A detailed comparison of AOT snapshot sizes can be performed using the [instructions documented here](./benchmarks/Comparing-AOT-Snapshot-Sizes.md).