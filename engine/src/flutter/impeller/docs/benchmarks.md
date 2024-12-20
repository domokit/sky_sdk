# Impeller Benchmarks

Here are some noteworthy benchmarks related to Impeller performance:

- **New Gallery** - Runs through the Flutter Gallery with a driver test.
  - Pixel 7 Pro
    - Vulkan: [dashboard](https://flutter-flutter-perf.skia.org/e/?queries=device_type%3DPixel_7_Pro%26test%3Dnew_gallery_impeller__transition_perf)
    - OpenGLES: [dashboard](https://flutter-flutter-perf.skia.org/e/?queries=device_type%3DPixel_7_Pro%26test%3Dnew_gallery_opengles_impeller__transition_perf)
    - Skia vs Vulkan Impeller - frame raster time stats: [dashboard](https://flutter-flutter-perf.skia.org/e/?queries=device_type%3DPixel_7_Pro%26sub_result%3D90th_percentile_frame_rasterizer_time_millis%26sub_result%3D99th_percentile_frame_rasterizer_time_millis%26sub_result%3Daverage_frame_rasterizer_time_millis%26sub_result%3Dworst_frame_rasterizer_time_millis%26test%3Dnew_gallery__transition_perf%26test%3Dnew_gallery_impeller__transition_perf)
    - Vulkan vs OpenGLES - average rasterizer time: [dashboard](https://flutter-flutter-perf.skia.org/e/?queries=device_type%3DPixel_7_Pro%26sub_result%3Daverage_frame_rasterizer_time_millis%26test%3Dnew_gallery_impeller__transition_perf%26test%3Dnew_gallery_opengles_impeller__transition_perf)
  - Samsung S10
    - Vulkan: [dashboard](https://flutter-flutter-perf.skia.org/e/?keys=X777844777514c7b34e736eadbc5dd002)
    - Skia vs Vulkan Impeller - 90th percentile frame rasterizer time: [dashboard](https://flutter-flutter-perf.skia.org/e/?begin=1707934850&end=1708021250&queries=device_type%3DSM-A025V%26sub_result%3D90th_percentile_frame_rasterizer_time_millis%26test%3Dnew_gallery__transition_perf%26test%3Dnew_gallery_impeller__transition_perf)
  - Moto G4 (OpenGLES): [dashboard](https://flutter-flutter-perf.skia.org/e/?keys=Xaeae5aa39c9028be43e8a9ad40540bd8)
  - iPhone 11
    - Metal: [dashboard](https://flutter-flutter-perf.skia.org/e/?keys=X9d52e54d0ac32151cc10feca61ea34cc)
    - Skia vs Metal Impeller - 90th percentile frame rasterizer time: [dashboard](https://flutter-flutter-perf.skia.org/e/?keys=X836c18b955eb83a9102a4391672f37e0)

- **Animated Blur Backdrop Filter** - A driver test that scrolls to a screen and
  animates a Blur Backdrop filter to get progressively blurrier.  This covers a
  gap in the "New Gallery" tests we've seen in places like Wonderous.
  - Pixel 7 Pro
    - Vulkan: [dashboard](https://flutter-flutter-perf.skia.org/e/?queries=device_type%3DPixel_7_Pro%26test%3Danimated_blur_backdrop_filter_perf__timeline_summary)
    - OpenGLES: [dashboard](https://flutter-flutter-perf.skia.org/e/?queries=device_type%3DPixel_7_Pro%26test%3Danimated_blur_backdrop_filter_perf_opengles__timeline_summary)
    - Vulkan vs OpenGLES - Average rasterizer time: [dashboard](https://flutter-flutter-perf.skia.org/e/?queries=device_type%3DPixel_7_Pro%26sub_result%3Daverage_frame_rasterizer_time_millis%26test%3Danimated_blur_backdrop_filter_perf__timeline_summary%26test%3Danimated_blur_backdrop_filter_perf_opengles__timeline_summary)
  - iPhone 11 (Metal): [dashboard](https://flutter-flutter-perf.skia.org/e/?queries=test%3Danimated_blur_backdrop_filter_perf_ios__timeline_summary)

- **Animated Advanced Blend** - A driver test like the Animated Blur test, but
  is displaying a handful of advanced blurs since it represents a specific case
  exercised in Wonderous.
  - Pixel 7 Pro
    - Vulkan: [dashboard](https://flutter-flutter-perf.skia.org/e/?queries=test%3Danimated_advanced_blend_perf__timeline_summary)
    - OpenGLES: [dashboard](https://flutter-flutter-perf.skia.org/e/?queries=test%3Danimated_advanced_blend_perf_opengles__timeline_summary)
    - Vulkan vs OpenGLES - Average rasterizer time: [dashboard](https://flutter-flutter-perf.skia.org/e/?queries=device_type%3DPixel_7_Pro%26sub_result%3Daverage_frame_rasterizer_time_millis%26test%3Danimated_advanced_blend_perf__timeline_summary%26test%3Danimated_advanced_blend_perf_opengles__timeline_summary)
  - iPhone 11 (Metal): [dashboard](https://flutter-flutter-perf.skia.org/e/?keys=X65477f5b5026c0d5ee8fee79122427ab)

- **Backdrop Filter Perf** - A driver test that isolates better the performance
  of blurs.
  - iPhone 11 (Metal) - Average rasterizer time: [dashboard](https://flutter-flutter-perf.skia.org/e/?queries=sub_result%3Daverage_frame_rasterizer_time_millis%26test%3Dbackdrop_filter_perf_ios__timeline_summary&xbaroffset=38815)
