// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert' show LineSplitter, json, utf8;
import 'dart:ffi' show Abi;
import 'dart:io';
import 'dart:math' as math;

import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;
import 'package:xml/xml.dart';

import '../framework/devices.dart';
import '../framework/framework.dart';
import '../framework/host_agent.dart';
import '../framework/task_result.dart';
import '../framework/utils.dart';

/// Must match flutter_driver/lib/src/common.dart.
///
/// Redefined here to avoid taking a dependency on flutter_driver.
String testOutputDirectory(String testDirectory) {
  return Platform.environment['FLUTTER_TEST_OUTPUTS_DIR'] ?? '$testDirectory/build';
}

TaskFunction createComplexLayoutScrollPerfTest({
  bool measureCpuGpu = true,
  bool badScroll = false,
  bool? enableImpeller,
  bool forceOpenGLES = false,
}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/complex_layout',
    badScroll ? 'test_driver/scroll_perf_bad.dart' : 'test_driver/scroll_perf.dart',
    'complex_layout_scroll_perf',
    measureCpuGpu: measureCpuGpu,
    enableImpeller: enableImpeller,
    forceOpenGLES: forceOpenGLES,
  ).run;
}

TaskFunction createTilesScrollPerfTest({bool? enableImpeller}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/complex_layout',
    'test_driver/scroll_perf.dart',
    'tiles_scroll_perf',
    enableImpeller: enableImpeller,
  ).run;
}

TaskFunction createUiKitViewScrollPerfTest({bool? enableImpeller}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/platform_views_layout',
    'test_driver/uikit_view_scroll_perf.dart',
    'platform_views_scroll_perf',
    testDriver: 'test_driver/scroll_perf_test.dart',
    needsFullTimeline: false,
    enableImpeller: enableImpeller,
    enableMergedPlatformThread: true,
  ).run;
}

TaskFunction createUiKitViewScrollPerfAdBannersTest({bool? enableImpeller}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/platform_views_layout',
    'test_driver/uikit_view_scroll_perf_ad_banners.dart',
    'platform_views_scroll_perf_ad_banners',
    testDriver: 'test_driver/scroll_perf_ad_banners_test.dart',
    needsFullTimeline: false,
    enableImpeller: enableImpeller,
  ).run;
}

TaskFunction createUiKitViewScrollPerfBottomAdBannerTest({bool? enableImpeller}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/platform_views_layout',
    'test_driver/uikit_view_scroll_perf_bottom_ad_banner.dart',
    'platform_views_scroll_perf_bottom_ad_banner',
    testDriver: 'test_driver/scroll_perf_bottom_ad_banner_test.dart',
    needsFullTimeline: false,
    enableImpeller: enableImpeller,
  ).run;
}

TaskFunction createUiKitViewScrollPerfNonIntersectingTest({bool? enableImpeller}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/platform_views_layout',
    'test_driver/uikit_view_scroll_perf_non_intersecting.dart',
    'platform_views_scroll_perf_non_intersecting',
    testDriver: 'test_driver/scroll_perf_non_intersecting_test.dart',
    needsFullTimeline: false,
    enableImpeller: enableImpeller,
  ).run;
}

TaskFunction createAndroidTextureScrollPerfTest({bool? enableImpeller}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/platform_views_layout',
    'test_driver/android_view_scroll_perf.dart',
    'platform_views_scroll_perf',
    testDriver: 'test_driver/scroll_perf_test.dart',
    needsFullTimeline: false,
    enableImpeller: enableImpeller,
    enableMergedPlatformThread: true,
  ).run;
}

TaskFunction createAndroidViewScrollPerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/platform_views_layout_hybrid_composition',
    'test_driver/android_view_scroll_perf.dart',
    'platform_views_scroll_perf_hybrid_composition',
    testDriver: 'test_driver/scroll_perf_test.dart',
    enableMergedPlatformThread: true,
  ).run;
}

TaskFunction createHomeScrollPerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/integration_tests/flutter_gallery',
    'test_driver/scroll_perf.dart',
    'home_scroll_perf',
  ).run;
}

TaskFunction createCullOpacityPerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'cull_opacity_perf',
    testDriver: 'test_driver/cull_opacity_perf_test.dart',
  ).run;
}

TaskFunction createCullOpacityPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/cull_opacity_perf_e2e.dart',
  ).run;
}

TaskFunction createCubicBezierPerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'cubic_bezier_perf',
    testDriver: 'test_driver/cubic_bezier_perf_test.dart',
  ).run;
}

TaskFunction createCubicBezierPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/cubic_bezier_perf_e2e.dart',
  ).run;
}

TaskFunction createBackdropFilterPerfTest({bool measureCpuGpu = true, bool? enableImpeller}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'backdrop_filter_perf',
    measureCpuGpu: measureCpuGpu,
    testDriver: 'test_driver/backdrop_filter_perf_test.dart',
    saveTraceFile: true,
    enableImpeller: enableImpeller,
    disablePartialRepaint: true,
  ).run;
}

TaskFunction createAnimationWithMicrotasksPerfTest({bool measureCpuGpu = true}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'animation_with_microtasks_perf',
    measureCpuGpu: measureCpuGpu,
    testDriver: 'test_driver/animation_with_microtasks_perf_test.dart',
    saveTraceFile: true,
  ).run;
}

TaskFunction createBackdropFilterPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/backdrop_filter_perf_e2e.dart',
  ).run;
}

TaskFunction createPostBackdropFilterPerfTest({bool measureCpuGpu = true}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'post_backdrop_filter_perf',
    measureCpuGpu: measureCpuGpu,
    testDriver: 'test_driver/post_backdrop_filter_perf_test.dart',
    saveTraceFile: true,
  ).run;
}

TaskFunction createSimpleAnimationPerfTest({bool measureCpuGpu = true, bool? enableImpeller}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'simple_animation_perf',
    measureCpuGpu: measureCpuGpu,
    testDriver: 'test_driver/simple_animation_perf_test.dart',
    saveTraceFile: true,
    enableImpeller: enableImpeller,
  ).run;
}

TaskFunction createAnimatedPlaceholderPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/animated_placeholder_perf_e2e.dart',
  ).run;
}

TaskFunction createPictureCachePerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'picture_cache_perf',
    testDriver: 'test_driver/picture_cache_perf_test.dart',
  ).run;
}

TaskFunction createPictureCachePerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/picture_cache_perf_e2e.dart',
  ).run;
}

TaskFunction createPictureCacheComplexityScoringPerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'picture_cache_complexity_scoring_perf',
    testDriver: 'test_driver/picture_cache_complexity_scoring_perf_test.dart',
  ).run;
}

TaskFunction createOpenPayScrollPerfTest({bool measureCpuGpu = true}) {
  return PerfTest(
    openpayDirectory.path,
    'test_driver/scroll_perf.dart',
    'openpay_scroll_perf',
    measureCpuGpu: measureCpuGpu,
    testDriver: 'test_driver/scroll_perf_test.dart',
    saveTraceFile: true,
  ).run;
}

TaskFunction createFlutterGalleryStartupTest({
  String target = 'lib/main.dart',
  Map<String, String>? runEnvironment,
}) {
  return StartupTest(
    '${flutterDirectory.path}/dev/integration_tests/flutter_gallery',
    target: target,
    runEnvironment: runEnvironment,
  ).run;
}

TaskFunction createComplexLayoutStartupTest() {
  return StartupTest('${flutterDirectory.path}/dev/benchmarks/complex_layout').run;
}

TaskFunction createFlutterGalleryCompileTest() {
  return CompileTest('${flutterDirectory.path}/dev/integration_tests/flutter_gallery').run;
}

TaskFunction createHelloWorldCompileTest() {
  return CompileTest(
    '${flutterDirectory.path}/examples/hello_world',
    reportPackageContentSizes: true,
  ).run;
}

TaskFunction createWebCompileTest() {
  return const WebCompileTest().run;
}

TaskFunction createFlutterViewStartupTest() {
  return StartupTest('${flutterDirectory.path}/examples/flutter_view', reportMetrics: false).run;
}

TaskFunction createPlatformViewStartupTest() {
  return StartupTest('${flutterDirectory.path}/examples/platform_view', reportMetrics: false).run;
}

TaskFunction createBasicMaterialCompileTest() {
  return () async {
    const String sampleAppName = 'sample_flutter_app';
    final Directory sampleDir = dir('${Directory.systemTemp.path}/$sampleAppName');

    rmTree(sampleDir);

    await inDirectory<void>(Directory.systemTemp, () async {
      await flutter('create', options: <String>['--template=app', sampleAppName]);
    });

    if (!sampleDir.existsSync()) {
      throw 'Failed to create default Flutter app in ${sampleDir.path}';
    }

    return CompileTest(sampleDir.path).run();
  };
}

TaskFunction createTextfieldPerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'textfield_perf',
    testDriver: 'test_driver/textfield_perf_test.dart',
  ).run;
}

TaskFunction createTextfieldPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/textfield_perf_e2e.dart',
  ).run;
}

TaskFunction createVeryLongPictureScrollingPerfE2ETest({required bool enableImpeller}) {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/very_long_picture_scrolling_perf_e2e.dart',
    enableImpeller: enableImpeller,
  ).run;
}

TaskFunction createSlidersPerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'sliders_perf',
    testDriver: 'test_driver/sliders_perf_test.dart',
  ).run;
}

TaskFunction createFullscreenTextfieldPerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'fullscreen_textfield_perf',
    testDriver: 'test_driver/fullscreen_textfield_perf_test.dart',
  ).run;
}

TaskFunction createFullscreenTextfieldPerfE2ETest({bool? enableImpeller}) {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/fullscreen_textfield_perf_e2e.dart',
    enableImpeller: enableImpeller,
  ).run;
}

TaskFunction createClipperCachePerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/clipper_cache_perf_e2e.dart',
  ).run;
}

TaskFunction createColorFilterAndFadePerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'color_filter_and_fade_perf',
    testDriver: 'test_driver/color_filter_and_fade_perf_test.dart',
    saveTraceFile: true,
  ).run;
}

TaskFunction createColorFilterAndFadePerfE2ETest({bool? enableImpeller}) {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/color_filter_and_fade_perf_e2e.dart',
    enableImpeller: enableImpeller,
  ).run;
}

TaskFunction createColorFilterCachePerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/color_filter_cache_perf_e2e.dart',
  ).run;
}

TaskFunction createColorFilterWithUnstableChildPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/color_filter_with_unstable_child_perf_e2e.dart',
  ).run;
}

TaskFunction createShaderMaskCachePerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/shader_mask_cache_perf_e2e.dart',
  ).run;
}

TaskFunction createFadingChildAnimationPerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'fading_child_animation_perf',
    testDriver: 'test_driver/fading_child_animation_perf_test.dart',
    saveTraceFile: true,
  ).run;
}

TaskFunction createImageFilteredTransformAnimationPerfTest({bool? enableImpeller}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'imagefiltered_transform_animation_perf',
    testDriver: 'test_driver/imagefiltered_transform_animation_perf_test.dart',
    saveTraceFile: true,
    enableImpeller: enableImpeller,
  ).run;
}

TaskFunction createsMultiWidgetConstructPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/multi_widget_construction_perf_e2e.dart',
  ).run;
}

TaskFunction createListTextLayoutPerfE2ETest({bool? enableImpeller}) {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/list_text_layout_perf_e2e.dart',
    enableImpeller: enableImpeller,
  ).run;
}

TaskFunction createsScrollSmoothnessPerfTest() {
  final String testDirectory = '${flutterDirectory.path}/dev/benchmarks/complex_layout';
  const String testTarget = 'test/measure_scroll_smoothness.dart';
  return () {
    return inDirectory<TaskResult>(testDirectory, () async {
      final Device device = await devices.workingDevice;
      await device.unlock();
      final String deviceId = device.deviceId;
      await flutter('packages', options: <String>['get']);

      await flutter(
        'drive',
        options: <String>[
          '--no-android-gradle-daemon',
          '-v',
          '--verbose-system-logs',
          '--profile',
          '-t',
          testTarget,
          '-d',
          deviceId,
        ],
      );
      final Map<String, dynamic> data =
          json.decode(
                file(
                  '${testOutputDirectory(testDirectory)}/scroll_smoothness_test.json',
                ).readAsStringSync(),
              )
              as Map<String, dynamic>;

      final Map<String, dynamic> result = <String, dynamic>{};
      void addResult(dynamic data, String suffix) {
        assert(data is Map<String, dynamic>);
        if (data is Map<String, dynamic>) {
          const List<String> metricKeys = <String>[
            'janky_count',
            'average_abs_jerk',
            'dropped_frame_count',
          ];
          for (final String key in metricKeys) {
            result[key + suffix] = data[key];
          }
        }
      }

      addResult(data['resample on with 90Hz input'], '_with_resampler_90Hz');
      addResult(data['resample on with 59Hz input'], '_with_resampler_59Hz');
      addResult(data['resample off with 90Hz input'], '_without_resampler_90Hz');
      addResult(data['resample off with 59Hz input'], '_without_resampler_59Hz');

      return TaskResult.success(result, benchmarkScoreKeys: result.keys.toList());
    });
  };
}

TaskFunction createFramePolicyIntegrationTest() {
  final String testDirectory = '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks';
  const String testTarget = 'test/frame_policy.dart';
  return () {
    return inDirectory<TaskResult>(testDirectory, () async {
      final Device device = await devices.workingDevice;
      await device.unlock();
      final String deviceId = device.deviceId;
      await flutter('packages', options: <String>['get']);

      await flutter(
        'drive',
        options: <String>[
          '--no-android-gradle-daemon',
          '-v',
          '--verbose-system-logs',
          '--profile',
          '-t',
          testTarget,
          '-d',
          deviceId,
        ],
      );
      final Map<String, dynamic> data =
          json.decode(
                file(
                  '${testOutputDirectory(testDirectory)}/frame_policy_event_delay.json',
                ).readAsStringSync(),
              )
              as Map<String, dynamic>;
      final Map<String, dynamic> fullLiveData = data['fullyLive'] as Map<String, dynamic>;
      final Map<String, dynamic> benchmarkLiveData = data['benchmarkLive'] as Map<String, dynamic>;
      final Map<String, dynamic> dataFormatted = <String, dynamic>{
        'average_delay_fullyLive_millis': fullLiveData['average_delay_millis'],
        'average_delay_benchmarkLive_millis': benchmarkLiveData['average_delay_millis'],
        '90th_percentile_delay_fullyLive_millis': fullLiveData['90th_percentile_delay_millis'],
        '90th_percentile_delay_benchmarkLive_millis':
            benchmarkLiveData['90th_percentile_delay_millis'],
      };

      return TaskResult.success(dataFormatted, benchmarkScoreKeys: dataFormatted.keys.toList());
    });
  };
}

TaskFunction createOpacityPeepholeOneRectPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/opacity_peephole_one_rect_perf_e2e.dart',
  ).run;
}

TaskFunction createOpacityPeepholeColOfRowsPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/opacity_peephole_col_of_rows_perf_e2e.dart',
  ).run;
}

TaskFunction createOpacityPeepholeOpacityOfGridPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/opacity_peephole_opacity_of_grid_perf_e2e.dart',
  ).run;
}

TaskFunction createOpacityPeepholeGridOfOpacityPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/opacity_peephole_grid_of_opacity_perf_e2e.dart',
  ).run;
}

TaskFunction createOpacityPeepholeFadeTransitionTextPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/opacity_peephole_fade_transition_text_perf_e2e.dart',
  ).run;
}

TaskFunction createOpacityPeepholeGridOfAlphaSaveLayersPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/opacity_peephole_grid_of_alpha_savelayers_perf_e2e.dart',
  ).run;
}

TaskFunction createOpacityPeepholeColOfAlphaSaveLayerRowsPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/opacity_peephole_col_of_alpha_savelayer_rows_perf_e2e.dart',
  ).run;
}

TaskFunction createGradientDynamicPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/gradient_dynamic_perf_e2e.dart',
  ).run;
}

TaskFunction createGradientConsistentPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/gradient_consistent_perf_e2e.dart',
  ).run;
}

TaskFunction createGradientStaticPerfE2ETest() {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/gradient_static_perf_e2e.dart',
  ).run;
}

TaskFunction createAnimatedAdvancedBlendPerfTest({bool? enableImpeller, bool? forceOpenGLES}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'animated_advanced_blend_perf',
    enableImpeller: enableImpeller,
    forceOpenGLES: forceOpenGLES,
    testDriver: 'test_driver/animated_advanced_blend_perf_test.dart',
    saveTraceFile: true,
  ).run;
}

TaskFunction createRRectBlurPerfTest({bool? enableImpeller, bool? forceOpenGLES}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'rrect_blur_perf',
    enableImpeller: enableImpeller,
    forceOpenGLES: forceOpenGLES,
    testDriver: 'test_driver/rrect_blur_perf_test.dart',
    saveTraceFile: true,
  ).run;
}

TaskFunction createAnimatedBlurBackropFilterPerfTest({bool? enableImpeller, bool? forceOpenGLES}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'animated_blur_backdrop_filter_perf',
    enableImpeller: enableImpeller,
    forceOpenGLES: forceOpenGLES,
    testDriver: 'test_driver/animated_blur_backdrop_filter_perf_test.dart',
    saveTraceFile: true,
  ).run;
}

TaskFunction createDrawPointsPerfTest({bool? enableImpeller}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'draw_points_perf',
    enableImpeller: enableImpeller,
    testDriver: 'test_driver/draw_points_perf_test.dart',
    saveTraceFile: true,
  ).run;
}

TaskFunction createDrawAtlasPerfTest({bool? forceOpenGLES}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'draw_atlas_perf',
    enableImpeller: true,
    testDriver: 'test_driver/draw_atlas_perf_test.dart',
    saveTraceFile: true,
    forceOpenGLES: forceOpenGLES,
  ).run;
}

TaskFunction createDrawVerticesPerfTest({bool? forceOpenGLES}) {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'draw_vertices_perf',
    enableImpeller: true,
    testDriver: 'test_driver/draw_vertices_perf_test.dart',
    saveTraceFile: true,
    forceOpenGLES: forceOpenGLES,
  ).run;
}

TaskFunction createPathTessellationStaticPerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'tessellation_perf_static',
    enableImpeller: true,
    testDriver: 'test_driver/path_tessellation_static_perf_test.dart',
    saveTraceFile: true,
  ).run;
}

TaskFunction createPathTessellationDynamicPerfTest() {
  return PerfTest(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test_driver/run_app.dart',
    'tessellation_perf_dynamic',
    enableImpeller: true,
    testDriver: 'test_driver/path_tessellation_dynamic_perf_test.dart',
    saveTraceFile: true,
  ).run;
}

TaskFunction createAnimatedComplexOpacityPerfE2ETest({bool? enableImpeller}) {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/animated_complex_opacity_perf_e2e.dart',
    enableImpeller: enableImpeller,
  ).run;
}

TaskFunction createAnimatedComplexImageFilteredPerfE2ETest({bool? enableImpeller}) {
  return PerfTest.e2e(
    '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
    'test/animated_complex_image_filtered_perf_e2e.dart',
    enableImpeller: enableImpeller,
  ).run;
}

Map<String, dynamic> _average(List<Map<String, dynamic>> results, int iterations) {
  final Map<String, dynamic> tally = <String, dynamic>{};
  for (final Map<String, dynamic> item in results) {
    item.forEach((String key, dynamic value) {
      if (tally.containsKey(key)) {
        tally[key] = (tally[key] as int) + (value as int);
      } else {
        tally[key] = value;
      }
    });
  }
  tally.forEach((String key, dynamic value) {
    tally[key] = (value as int) ~/ iterations;
  });
  return tally;
}

/// Opens the file at testDirectory + 'ios/Runner/Info.plist'
/// and adds additional manifest settings.
void _updateManifestSettings(
  String testDirectory, {
  required bool disablePartialRepaint,
  required bool platformThreadMerged,
}) {
  final String manifestPath = path.join(testDirectory, 'ios', 'Runner', 'Info.plist');
  final File file = File(manifestPath);

  if (!file.existsSync()) {
    throw Exception('Info.plist not found at $manifestPath');
  }

  final String xmlStr = file.readAsStringSync();
  final XmlDocument xmlDoc = XmlDocument.parse(xmlStr);
  final List<(String, String)> keyPairs = <(String, String)>[
    if (disablePartialRepaint) ('FLTDisablePartialRepaint', disablePartialRepaint.toString()),
    if (platformThreadMerged) ('FLTEnableMergedPlatformUIThread', platformThreadMerged.toString()),
  ];

  final XmlElement applicationNode = xmlDoc.findAllElements('dict').first;

  // Check if the meta-data node already exists.
  for (final (String key, String value) in keyPairs) {
    applicationNode.children.add(
      XmlElement(XmlName('key'), <XmlAttribute>[], <XmlNode>[XmlText(key)], false),
    );
    applicationNode.children.add(XmlElement(XmlName(value)));
  }

  file.writeAsStringSync(xmlDoc.toXmlString(pretty: true, indent: '    '));
}

Future<void> _resetPlist(String testDirectory) async {
  final String manifestPath = path.join(testDirectory, 'ios', 'Runner', 'Info.plist');
  final File file = File(manifestPath);

  if (!file.existsSync()) {
    throw Exception('Info.plist not found at $manifestPath');
  }

  await exec('git', <String>['checkout', file.path]);
}

void _addMetadataToManifest(String testDirectory, List<(String, String)> keyPairs) {
  final String manifestPath = path.join(
    testDirectory,
    'android',
    'app',
    'src',
    'main',
    'AndroidManifest.xml',
  );
  final File file = File(manifestPath);

  if (!file.existsSync()) {
    throw Exception('AndroidManifest.xml not found at $manifestPath');
  }

  final String xmlStr = file.readAsStringSync();
  final XmlDocument xmlDoc = XmlDocument.parse(xmlStr);
  final XmlElement applicationNode = xmlDoc.findAllElements('application').first;

  // Check if the meta-data node already exists.
  for (final (String key, String value) in keyPairs) {
    final Iterable<XmlElement> existingMetaData = applicationNode
        .findAllElements('meta-data')
        .where((XmlElement node) => node.getAttribute('android:name') == key);

    if (existingMetaData.isNotEmpty) {
      final XmlElement existingEntry = existingMetaData.first;
      existingEntry.setAttribute('android:value', value);
    } else {
      final XmlElement metaData = XmlElement(XmlName('meta-data'), <XmlAttribute>[
        XmlAttribute(XmlName('android:name'), key),
        XmlAttribute(XmlName('android:value'), value),
      ]);
      applicationNode.children.add(metaData);
    }
  }

  file.writeAsStringSync(xmlDoc.toXmlString(pretty: true, indent: '    '));
}

void _addMergedPlatformThreadSupportToManifest(String testDirectory) {
  final List<(String, String)> keyPairs = <(String, String)>[
    ('io.flutter.embedding.android.EnableMergedPlatformUIThread', 'true'),
  ];
  _addMetadataToManifest(testDirectory, keyPairs);
}

/// Opens the file at testDirectory + 'android/app/src/main/AndroidManifest.xml'
/// <meta-data
///   android:name="io.flutter.embedding.android.EnableVulkanGPUTracing"
///   android:value="true" />
void _addVulkanGPUTracingToManifest(String testDirectory) {
  final List<(String, String)> keyPairs = <(String, String)>[
    ('io.flutter.embedding.android.EnableVulkanGPUTracing', 'true'),
  ];
  _addMetadataToManifest(testDirectory, keyPairs);
}

/// Opens the file at testDirectory + 'android/app/src/main/AndroidManifest.xml'
/// and adds the following entry to the application.
/// <meta-data
///   android:name="io.flutter.embedding.android.ImpellerBackend"
///   android:value="opengles" />
/// <meta-data
///   android:name="io.flutter.embedding.android.EnableOpenGLGPUTracing"
///   android:value="true" />
void _addOpenGLESToManifest(String testDirectory) {
  final List<(String, String)> keyPairs = <(String, String)>[
    ('io.flutter.embedding.android.ImpellerBackend', 'opengles'),
    ('io.flutter.embedding.android.EnableOpenGLGPUTracing', 'true'),
  ];
  _addMetadataToManifest(testDirectory, keyPairs);
}

Future<void> _resetManifest(String testDirectory) async {
  final String manifestPath = path.join(
    testDirectory,
    'android',
    'app',
    'src',
    'main',
    'AndroidManifest.xml',
  );
  final File file = File(manifestPath);

  if (!file.existsSync()) {
    throw Exception('AndroidManifest.xml not found at $manifestPath');
  }

  await exec('git', <String>['checkout', file.path]);
}

/// Measure application startup performance.
class StartupTest {
  const StartupTest(
    this.testDirectory, {
    this.reportMetrics = true,
    this.target = 'lib/main.dart',
    this.runEnvironment,
  });

  final String testDirectory;
  final bool reportMetrics;
  final String target;
  final Map<String, String>? runEnvironment;

  Future<TaskResult> run() async {
    return inDirectory<TaskResult>(testDirectory, () async {
      final Device device = await devices.workingDevice;
      await device.unlock();
      const int iterations = 5;
      final List<Map<String, dynamic>> results = <Map<String, dynamic>>[];

      section('Building application');
      String? applicationBinaryPath;
      switch (deviceOperatingSystem) {
        case DeviceOperatingSystem.android:
          await flutter(
            'build',
            options: <String>[
              'apk',
              '-v',
              '--profile',
              '--target-platform=android-arm,android-arm64',
              '--target=$target',
            ],
          );
          applicationBinaryPath = '$testDirectory/build/app/outputs/flutter-apk/app-profile.apk';
        case DeviceOperatingSystem.androidArm:
          await flutter(
            'build',
            options: <String>[
              'apk',
              '-v',
              '--profile',
              '--target-platform=android-arm',
              '--target=$target',
            ],
          );
          applicationBinaryPath = '$testDirectory/build/app/outputs/flutter-apk/app-profile.apk';
        case DeviceOperatingSystem.androidArm64:
          await flutter(
            'build',
            options: <String>[
              'apk',
              '-v',
              '--profile',
              '--target-platform=android-arm64',
              '--target=$target',
            ],
          );
          applicationBinaryPath = '$testDirectory/build/app/outputs/flutter-apk/app-profile.apk';
        case DeviceOperatingSystem.fake:
        case DeviceOperatingSystem.fuchsia:
        case DeviceOperatingSystem.linux:
          break;
        case DeviceOperatingSystem.ios:
        case DeviceOperatingSystem.macos:
          await flutter(
            'build',
            options: <String>[
              if (deviceOperatingSystem == DeviceOperatingSystem.ios) 'ios' else 'macos',
              '-v',
              '--profile',
              '--target=$target',
              if (deviceOperatingSystem == DeviceOperatingSystem.ios) '--no-publish-port',
            ],
          );
          final String buildRoot = path.join(testDirectory, 'build');
          applicationBinaryPath = _findDarwinAppInBuildDirectory(buildRoot);
        case DeviceOperatingSystem.windows:
          await flutter(
            'build',
            options: <String>['windows', '-v', '--profile', '--target=$target'],
          );
          final String basename = path.basename(testDirectory);
          final String arch = Abi.current() == Abi.windowsX64 ? 'x64' : 'arm64';
          applicationBinaryPath = path.join(
            testDirectory,
            'build',
            'windows',
            arch,
            'runner',
            'Profile',
            '$basename.exe',
          );
      }

      const int maxFailures = 3;
      int currentFailures = 0;
      for (int i = 0; i < iterations; i += 1) {
        // Startup should not take more than a few minutes. After 10 minutes,
        // take a screenshot to help debug.
        final Timer timer = Timer(const Duration(minutes: 10), () async {
          print('Startup not completed within 10 minutes. Taking a screenshot...');
          await _flutterScreenshot(
            device.deviceId,
            'screenshot_startup_${DateTime.now().toLocal().toIso8601String()}.png',
          );
        });
        final int result = await flutter(
          'run',
          options: <String>[
            '--no-android-gradle-daemon',
            '--no-publish-port',
            '--verbose',
            '--profile',
            '--trace-startup',
            '--target=$target',
            '-d',
            device.deviceId,
            if (applicationBinaryPath != null) '--use-application-binary=$applicationBinaryPath',
          ],
          environment: runEnvironment,
          canFail: true,
        );
        timer.cancel();
        if (result == 0) {
          final Map<String, dynamic> data =
              json.decode(
                    file(
                      '${testOutputDirectory(testDirectory)}/start_up_info.json',
                    ).readAsStringSync(),
                  )
                  as Map<String, dynamic>;
          results.add(data);
        } else {
          currentFailures += 1;
          await _flutterScreenshot(
            device.deviceId,
            'screenshot_startup_failure_$currentFailures.png',
          );
          i -= 1;
          if (currentFailures == maxFailures) {
            return TaskResult.failure('Application failed to start $maxFailures times');
          }
        }

        await device.uninstallApp();
      }

      final Map<String, dynamic> averageResults = _average(results, iterations);

      if (!reportMetrics) {
        return TaskResult.success(averageResults);
      }

      return TaskResult.success(
        averageResults,
        benchmarkScoreKeys: <String>['timeToFirstFrameMicros', 'timeToFirstFrameRasterizedMicros'],
      );
    });
  }

  Future<void> _flutterScreenshot(String deviceId, String screenshotName) async {
    if (hostAgent.dumpDirectory != null) {
      await flutter(
        'screenshot',
        options: <String>[
          '-d',
          deviceId,
          '--out',
          hostAgent.dumpDirectory!.childFile(screenshotName).path,
        ],
        canFail: true,
      );
    }
  }
}

/// A one-off test to verify that devtools starts in profile mode.
class DevtoolsStartupTest {
  const DevtoolsStartupTest(this.testDirectory);

  final String testDirectory;

  Future<TaskResult> run() async {
    return inDirectory<TaskResult>(testDirectory, () async {
      final Device device = await devices.workingDevice;

      section('Building application');
      String? applicationBinaryPath;
      switch (deviceOperatingSystem) {
        case DeviceOperatingSystem.android:
          await flutter(
            'build',
            options: <String>[
              'apk',
              '-v',
              '--profile',
              '--target-platform=android-arm,android-arm64',
            ],
          );
          applicationBinaryPath = '$testDirectory/build/app/outputs/flutter-apk/app-profile.apk';
        case DeviceOperatingSystem.androidArm:
          await flutter(
            'build',
            options: <String>['apk', '-v', '--profile', '--target-platform=android-arm'],
          );
          applicationBinaryPath = '$testDirectory/build/app/outputs/flutter-apk/app-profile.apk';
        case DeviceOperatingSystem.androidArm64:
          await flutter(
            'build',
            options: <String>['apk', '-v', '--profile', '--target-platform=android-arm64'],
          );
          applicationBinaryPath = '$testDirectory/build/app/outputs/flutter-apk/app-profile.apk';
        case DeviceOperatingSystem.ios:
          await flutter('build', options: <String>['ios', '-v', '--profile']);
          applicationBinaryPath = _findDarwinAppInBuildDirectory(
            '$testDirectory/build/ios/iphoneos',
          );
        case DeviceOperatingSystem.fake:
        case DeviceOperatingSystem.fuchsia:
        case DeviceOperatingSystem.linux:
        case DeviceOperatingSystem.macos:
        case DeviceOperatingSystem.windows:
          break;
      }

      final Process process = await startFlutter(
        'run',
        options: <String>[
          '--no-android-gradle-daemon',
          '--no-publish-port',
          '--verbose',
          '--profile',
          '-d',
          device.deviceId,
          if (applicationBinaryPath != null) '--use-application-binary=$applicationBinaryPath',
        ],
      );
      final Completer<void> completer = Completer<void>();
      bool sawLine = false;
      process.stdout.transform(utf8.decoder).transform(const LineSplitter()).listen((String line) {
        print('[STDOUT]: $line');
        // Wait for devtools output.
        if (line.contains('The Flutter DevTools debugger and profiler')) {
          sawLine = true;
          completer.complete();
        }
      });
      bool didExit = false;
      unawaited(
        process.exitCode.whenComplete(() {
          didExit = true;
        }),
      );
      await Future.any(<Future<void>>[
        completer.future,
        Future<void>.delayed(const Duration(minutes: 5)),
        process.exitCode,
      ]);
      if (!didExit) {
        process.stdin.writeln('q');
        await process.exitCode;
      }

      await device.uninstallApp();

      if (sawLine) {
        return TaskResult.success(null, benchmarkScoreKeys: <String>[]);
      }
      return TaskResult.failure(
        'Did not see line "The Flutter DevTools debugger and profiler" in output',
      );
    });
  }
}

/// A callback function to be used to mock the flutter drive command in PerfTests.
///
/// The `options` contains all the arguments in the `flutter drive` command in PerfTests.
typedef FlutterDriveCallback = void Function(List<String> options);

/// Measures application runtime performance, specifically per-frame
/// performance.
class PerfTest {
  const PerfTest(
    this.testDirectory,
    this.testTarget,
    this.timelineFileName, {
    this.measureCpuGpu = true,
    this.measureMemory = true,
    this.measureTotalGCTime = true,
    this.saveTraceFile = false,
    this.testDriver,
    this.needsFullTimeline = true,
    this.benchmarkScoreKeys,
    this.dartDefine = '',
    String? resultFilename,
    this.device,
    this.flutterDriveCallback,
    this.timeoutSeconds,
    this.enableImpeller,
    this.forceOpenGLES,
    this.disablePartialRepaint = false,
    this.enableMergedPlatformThread = false,
    this.createPlatforms = const <String>[],
  }) : _resultFilename = resultFilename;

  const PerfTest.e2e(
    this.testDirectory,
    this.testTarget, {
    this.measureCpuGpu = false,
    this.measureMemory = false,
    this.measureTotalGCTime = false,
    this.testDriver = 'test_driver/e2e_test.dart',
    this.needsFullTimeline = false,
    this.benchmarkScoreKeys = _kCommonScoreKeys,
    this.dartDefine = '',
    String resultFilename = 'e2e_perf_summary',
    this.device,
    this.flutterDriveCallback,
    this.timeoutSeconds,
    this.enableImpeller,
    this.forceOpenGLES,
    this.disablePartialRepaint = false,
    this.enableMergedPlatformThread = false,
    this.createPlatforms = const <String>[],
  }) : saveTraceFile = false,
       timelineFileName = null,
       _resultFilename = resultFilename;

  /// The directory where the app under test is defined.
  final String testDirectory;

  /// The main entry-point file of the application, as run on the device.
  final String testTarget;
  // The prefix name of the filename such as `<timelineFileName>.timeline_summary.json`.
  final String? timelineFileName;
  String get traceFilename => '$timelineFileName.timeline';
  String get resultFilename => _resultFilename ?? '$timelineFileName.timeline_summary';
  final String? _resultFilename;

  /// The test file to run on the host.
  final String? testDriver;

  /// Whether to collect CPU and GPU metrics.
  final bool measureCpuGpu;

  /// Whether to collect memory metrics.
  final bool measureMemory;

  /// Whether to summarize total GC time on the UI thread from the timeline.
  final bool measureTotalGCTime;

  /// Whether to collect full timeline, meaning if `--trace-startup` flag is needed.
  final bool needsFullTimeline;

  /// Whether to save the trace timeline file `*.timeline.json`.
  final bool saveTraceFile;

  /// The device to test on.
  ///
  /// If null, the device is selected depending on the current environment.
  final Device? device;

  /// The function called instead of the actually `flutter drive`.
  ///
  /// If it is not `null`, `flutter drive` will not happen in the PerfTests.
  final FlutterDriveCallback? flutterDriveCallback;

  /// Whether the perf test should enable Impeller.
  final bool? enableImpeller;

  /// Whether the perf test force Impeller's OpenGLES backend.
  final bool? forceOpenGLES;

  /// Whether partial repaint functionality should be disabled (iOS only).
  final bool disablePartialRepaint;

  /// Whether the UI thread should be the platform thread.
  final bool enableMergedPlatformThread;

  /// Number of seconds to time out the test after, allowing debug callbacks to run.
  final int? timeoutSeconds;

  /// The keys of the values that need to be reported.
  ///
  /// If it's `null`, then report:
  /// ```Dart
  /// <String>[
  ///   'average_frame_build_time_millis',
  ///   'worst_frame_build_time_millis',
  ///   '90th_percentile_frame_build_time_millis',
  ///   '99th_percentile_frame_build_time_millis',
  ///   'average_frame_rasterizer_time_millis',
  ///   'worst_frame_rasterizer_time_millis',
  ///   '90th_percentile_frame_rasterizer_time_millis',
  ///   '99th_percentile_frame_rasterizer_time_millis',
  ///   'average_vsync_transitions_missed',
  ///   '90th_percentile_vsync_transitions_missed',
  ///   '99th_percentile_vsync_transitions_missed',
  ///   if (measureCpuGpu) 'average_cpu_usage',
  ///   if (measureCpuGpu) 'average_gpu_usage',
  /// ]
  /// ```
  final List<String>? benchmarkScoreKeys;

  /// Additional flags for `--dart-define` to control the test
  final String dartDefine;

  /// Additional platforms to create with `flutter create` before running
  /// the test.
  final List<String> createPlatforms;

  Future<TaskResult> run() {
    return internalRun();
  }

  @protected
  Future<TaskResult> internalRun({String? existingApp}) {
    return inDirectory<TaskResult>(testDirectory, () async {
      late Device selectedDevice;
      selectedDevice = device ?? await devices.workingDevice;
      await selectedDevice.unlock();
      await selectedDevice.toggleFixedPerformanceMode(true);

      final String deviceId = selectedDevice.deviceId;
      final String? localEngine = localEngineFromEnv;
      final String? localEngineHost = localEngineHostFromEnv;
      final String? localEngineSrcPath = localEngineSrcPathFromEnv;

      if (createPlatforms.isNotEmpty) {
        // Ensure that the platform-specific manifests are freshly created and
        // do not contain any settings from previous runs.
        await exec('git', <String>['clean', '-f', testDirectory]);

        await flutter(
          'create',
          options: <String>['--platforms', createPlatforms.join(','), '--no-overwrite', '.'],
        );
      }

      bool changedPlist = false;
      bool changedManifest = false;

      Future<void> resetManifest() async {
        if (!changedManifest) {
          return;
        }
        try {
          await _resetManifest(testDirectory);
        } catch (err) {
          print('Caught exception while trying to reset AndroidManifest: $err');
        }
      }

      Future<void> resetPlist() async {
        if (!changedPlist) {
          return;
        }
        try {
          await _resetPlist(testDirectory);
        } catch (err) {
          print('Caught exception while trying to reset Info.plist: $err');
        }
      }

      try {
        if (enableImpeller ?? false) {
          changedManifest = true;
          _addVulkanGPUTracingToManifest(testDirectory);
          if (forceOpenGLES ?? false) {
            _addOpenGLESToManifest(testDirectory);
          }
          if (enableMergedPlatformThread) {
            _addMergedPlatformThreadSupportToManifest(testDirectory);
          }
        }
        if (disablePartialRepaint || enableMergedPlatformThread) {
          changedPlist = true;
          _updateManifestSettings(
            testDirectory,
            disablePartialRepaint: disablePartialRepaint,
            platformThreadMerged: enableMergedPlatformThread,
          );
        }

        final List<String> options = <String>[
          if (localEngine != null) ...<String>['--local-engine', localEngine],
          if (localEngineHost != null) ...<String>['--local-engine-host', localEngineHost],
          if (localEngineSrcPath != null) ...<String>[
            '--local-engine-src-path',
            localEngineSrcPath,
          ],
          '--no-android-gradle-daemon',
          '-v',
          '--verbose-system-logs',
          '--profile',
          if (timeoutSeconds != null) ...<String>['--timeout', timeoutSeconds.toString()],
          if (needsFullTimeline) '--trace-startup', // Enables "endless" timeline event buffering.
          '-t', testTarget,
          if (testDriver != null) ...<String>['--driver', testDriver!],
          if (existingApp != null) ...<String>['--use-existing-app', existingApp],
          if (dartDefine.isNotEmpty) ...<String>['--dart-define', dartDefine],
          if (enableImpeller != null && enableImpeller!) '--enable-impeller',
          if (enableImpeller != null && !enableImpeller!) '--no-enable-impeller',
          '-d',
          deviceId,
        ];
        if (flutterDriveCallback != null) {
          flutterDriveCallback!(options);
        } else {
          await flutter('drive', options: options);
        }
      } finally {
        await resetManifest();
        await resetPlist();
        await selectedDevice.toggleFixedPerformanceMode(false);
      }

      final Map<String, dynamic> data =
          json.decode(
                file(
                  '${testOutputDirectory(testDirectory)}/$resultFilename.json',
                ).readAsStringSync(),
              )
              as Map<String, dynamic>;

      if (data['frame_count'] as int < 5) {
        return TaskResult.failure(
          'Timeline contains too few frames: ${data['frame_count']}. Possibly '
          'trace events are not being captured.',
        );
      }

      final bool recordGPU;
      switch (deviceOperatingSystem) {
        case DeviceOperatingSystem.ios:
          recordGPU = true;
        case DeviceOperatingSystem.android:
        case DeviceOperatingSystem.androidArm:
        case DeviceOperatingSystem.androidArm64:
          recordGPU = true;
        case DeviceOperatingSystem.fake:
        case DeviceOperatingSystem.fuchsia:
        case DeviceOperatingSystem.linux:
        case DeviceOperatingSystem.macos:
        case DeviceOperatingSystem.windows:
          recordGPU = false;
      }

      // TODO(liyuqian): Remove isAndroid restriction once
      // https://github.com/flutter/flutter/issues/61567 is fixed.
      final bool isAndroid = deviceOperatingSystem == DeviceOperatingSystem.android;
      return TaskResult.success(
        data,
        detailFiles: <String>[
          if (saveTraceFile) '${testOutputDirectory(testDirectory)}/$traceFilename.json',
        ],
        benchmarkScoreKeys:
            benchmarkScoreKeys ??
            <String>[
              ..._kCommonScoreKeys,
              'average_vsync_transitions_missed',
              '90th_percentile_vsync_transitions_missed',
              '99th_percentile_vsync_transitions_missed',
              if (measureCpuGpu && !isAndroid) ...<String>[
                // See https://github.com/flutter/flutter/issues/68888
                if (data['average_cpu_usage'] != null) 'average_cpu_usage',
                if (data['average_gpu_usage'] != null) 'average_gpu_usage',
              ],
              if (measureMemory && !isAndroid) ...<String>[
                // See https://github.com/flutter/flutter/issues/68888
                if (data['average_memory_usage'] != null) 'average_memory_usage',
                if (data['90th_percentile_memory_usage'] != null) '90th_percentile_memory_usage',
                if (data['99th_percentile_memory_usage'] != null) '99th_percentile_memory_usage',
              ],
              if (measureTotalGCTime) 'total_ui_gc_time',
              if (data['30hz_frame_percentage'] != null) '30hz_frame_percentage',
              if (data['60hz_frame_percentage'] != null) '60hz_frame_percentage',
              if (data['80hz_frame_percentage'] != null) '80hz_frame_percentage',
              if (data['90hz_frame_percentage'] != null) '90hz_frame_percentage',
              if (data['120hz_frame_percentage'] != null) '120hz_frame_percentage',
              if (data['illegal_refresh_rate_frame_count'] != null)
                'illegal_refresh_rate_frame_count',
              if (recordGPU) ...<String>[
                // GPU Frame Time.
                if (data['average_gpu_frame_time'] != null) 'average_gpu_frame_time',
                if (data['90th_percentile_gpu_frame_time'] != null)
                  '90th_percentile_gpu_frame_time',
                if (data['99th_percentile_gpu_frame_time'] != null)
                  '99th_percentile_gpu_frame_time',
                if (data['worst_gpu_frame_time'] != null) 'worst_gpu_frame_time',
                // GPU Memory.
                if (data['average_gpu_memory_mb'] != null) 'average_gpu_memory_mb',
                if (data['90th_percentile_gpu_memory_mb'] != null) '90th_percentile_gpu_memory_mb',
                if (data['99th_percentile_gpu_memory_mb'] != null) '99th_percentile_gpu_memory_mb',
                if (data['worst_gpu_memory_mb'] != null) 'worst_gpu_memory_mb',
              ],
            ],
      );
    });
  }
}

const List<String> _kCommonScoreKeys = <String>[
  'average_frame_build_time_millis',
  'worst_frame_build_time_millis',
  '90th_percentile_frame_build_time_millis',
  '99th_percentile_frame_build_time_millis',
  'average_frame_rasterizer_time_millis',
  'worst_frame_rasterizer_time_millis',
  '90th_percentile_frame_rasterizer_time_millis',
  '99th_percentile_frame_rasterizer_time_millis',
  'old_gen_gc_count',
];

/// Measures how long it takes to compile a Flutter app to JavaScript and how
/// big the compiled code is.
class WebCompileTest {
  const WebCompileTest();

  Future<TaskResult> run() async {
    final Map<String, Object> metrics = <String, Object>{};

    metrics.addAll(
      await runSingleBuildTest(
        directory: '${flutterDirectory.path}/examples/hello_world',
        metric: 'hello_world',
      ),
    );

    metrics.addAll(
      await runSingleBuildTest(
        directory: '${flutterDirectory.path}/dev/integration_tests/flutter_gallery',
        metric: 'flutter_gallery',
      ),
    );

    const String sampleAppName = 'sample_flutter_app';
    final Directory sampleDir = dir('${Directory.systemTemp.path}/$sampleAppName');

    rmTree(sampleDir);

    await inDirectory<void>(Directory.systemTemp, () async {
      await flutter('create', options: <String>['--template=app', sampleAppName]);
    });

    metrics.addAll(
      await runSingleBuildTest(directory: sampleDir.path, metric: 'basic_material_app'),
    );

    return TaskResult.success(metrics, benchmarkScoreKeys: metrics.keys.toList());
  }

  /// Run a single web compile test and return its metrics.
  ///
  /// Run a single web compile test for the app under [directory], and store
  /// its metrics with prefix [metric].
  static Future<Map<String, int>> runSingleBuildTest({
    required String directory,
    required String metric,
    bool measureBuildTime = false,
  }) {
    return inDirectory<Map<String, int>>(directory, () async {
      final Map<String, int> metrics = <String, int>{};

      await flutter('clean');
      await flutter('packages', options: <String>['get']);
      final Stopwatch? watch = measureBuildTime ? Stopwatch() : null;
      watch?.start();
      await evalFlutter(
        'build',
        options: <String>['web', '-v', '--release', '--no-pub', '--no-web-resources-cdn'],
      );
      watch?.stop();
      final String buildDir = path.join(directory, 'build', 'web');
      metrics.addAll(
        await getSize(
          directories: <String, String>{'web_build_dir': buildDir},
          files: <String, String>{
            'dart2js': path.join(buildDir, 'main.dart.js'),
            'canvaskit_wasm': path.join(buildDir, 'canvaskit', 'canvaskit.wasm'),
            'canvaskit_js': path.join(buildDir, 'canvaskit', 'canvaskit.js'),
            'flutter_js': path.join(buildDir, 'flutter.js'),
          },
          metric: metric,
        ),
      );

      if (measureBuildTime) {
        metrics['${metric}_dart2js_millis'] = watch!.elapsedMilliseconds;
      }

      return metrics;
    });
  }

  /// Obtains the size and gzipped size of both [dartBundleFile] and [buildDir].
  static Future<Map<String, int>> getSize({
    /// Mapping of metric key name to file system path for directories to measure
    Map<String, String> directories = const <String, String>{},

    /// Mapping of metric key name to file system path for files to measure
    Map<String, String> files = const <String, String>{},
    required String metric,
  }) async {
    const String kGzipCompressionLevel = '-9';
    final Map<String, int> sizeMetrics = <String, int>{};

    final Directory tempDir = Directory.systemTemp.createTempSync('perf_tests_gzips');
    try {
      for (final MapEntry<String, String> entry in files.entries) {
        final String key = entry.key;
        final String filePath = entry.value;
        sizeMetrics['${metric}_${key}_uncompressed_bytes'] = File(filePath).lengthSync();

        await Process.run('gzip', <String>['--keep', kGzipCompressionLevel, filePath]);
        // gzip does not provide a CLI option to specify an output file, so
        // instead just move the output file to the temp dir
        final File compressedFile = File(
          '$filePath.gz',
        ).renameSync(path.join(tempDir.absolute.path, '$key.gz'));
        sizeMetrics['${metric}_${key}_compressed_bytes'] = compressedFile.lengthSync();
      }

      for (final MapEntry<String, String> entry in directories.entries) {
        final String key = entry.key;
        final String dirPath = entry.value;

        final String tarball = path.join(tempDir.absolute.path, '$key.tar');
        await Process.run('tar', <String>['--create', '--verbose', '--file=$tarball', dirPath]);
        sizeMetrics['${metric}_${key}_uncompressed_bytes'] = File(tarball).lengthSync();

        // get size of compressed build directory
        await Process.run('gzip', <String>['--keep', kGzipCompressionLevel, tarball]);
        sizeMetrics['${metric}_${key}_compressed_bytes'] = File('$tarball.gz').lengthSync();
      }
    } finally {
      try {
        tempDir.deleteSync(recursive: true);
      } on FileSystemException {
        print('Failed to delete ${tempDir.path}.');
      }
    }
    return sizeMetrics;
  }
}

/// Measures how long it takes to compile a Flutter app and how big the compiled
/// code is.
class CompileTest {
  const CompileTest(this.testDirectory, {this.reportPackageContentSizes = false});

  final String testDirectory;
  final bool reportPackageContentSizes;

  Future<TaskResult> run() async {
    return inDirectory<TaskResult>(testDirectory, () async {
      await flutter('packages', options: <String>['get']);

      // "initial" compile required downloading and creating the `android/.gradle` directory while "full"
      // compiles only run `flutter clean` between runs.
      final Map<String, dynamic> compileInitialRelease = await _compileApp(deleteGradleCache: true);
      final Map<String, dynamic> compileFullRelease = await _compileApp(deleteGradleCache: false);
      final Map<String, dynamic> compileInitialDebug = await _compileDebug(
        clean: true,
        deleteGradleCache: true,
        metricKey: 'debug_initial_compile_millis',
      );
      final Map<String, dynamic> compileFullDebug = await _compileDebug(
        clean: true,
        deleteGradleCache: false,
        metricKey: 'debug_full_compile_millis',
      );
      // Build again without cleaning, should be faster.
      final Map<String, dynamic> compileSecondDebug = await _compileDebug(
        clean: false,
        deleteGradleCache: false,
        metricKey: 'debug_second_compile_millis',
      );

      final Map<String, dynamic> metrics = <String, dynamic>{
        ...compileInitialRelease,
        ...compileFullRelease,
        ...compileInitialDebug,
        ...compileFullDebug,
        ...compileSecondDebug,
      };

      final File mainDart = File('$testDirectory/lib/main.dart');
      if (mainDart.existsSync()) {
        final List<int> bytes = mainDart.readAsBytesSync();
        // "Touch" the file
        mainDart.writeAsStringSync(' ', mode: FileMode.append, flush: true);
        // Build after "edit" without clean should be faster than first build
        final Map<String, dynamic> compileAfterEditDebug = await _compileDebug(
          clean: false,
          deleteGradleCache: false,
          metricKey: 'debug_compile_after_edit_millis',
        );
        metrics.addAll(compileAfterEditDebug);
        // Revert the changes
        mainDart.writeAsBytesSync(bytes, flush: true);
      }

      return TaskResult.success(metrics, benchmarkScoreKeys: metrics.keys.toList());
    });
  }

  Future<Map<String, dynamic>> _compileApp({required bool deleteGradleCache}) async {
    await flutter('clean');
    if (deleteGradleCache) {
      final Directory gradleCacheDir = Directory('$testDirectory/android/.gradle');
      rmTree(gradleCacheDir);
    }
    final Stopwatch watch = Stopwatch();
    int releaseSizeInBytes;
    final List<String> options = <String>['--release'];
    final Map<String, dynamic> metrics = <String, dynamic>{};

    switch (deviceOperatingSystem) {
      case DeviceOperatingSystem.ios:
      case DeviceOperatingSystem.macos:
        unawaited(stderr.flush());
        late final String deviceId;
        if (deviceOperatingSystem == DeviceOperatingSystem.ios) {
          deviceId = 'ios';
        } else if (deviceOperatingSystem == DeviceOperatingSystem.macos) {
          deviceId = 'macos';
        } else {
          throw Exception('Attempted to run darwin compile workflow with $deviceOperatingSystem');
        }

        options.insert(0, deviceId);
        options.add('--tree-shake-icons');
        options.add('--split-debug-info=infos/');
        watch.start();
        await flutter(
          'build',
          options: options,
          environment: <String, String>{
            // iOS 12.1 and lower did not have Swift ABI compatibility so Swift apps embedded the Swift runtime.
            // https://developer.apple.com/documentation/xcode-release-notes/swift-5-release-notes-for-xcode-10_2#App-Thinning
            // The gallery pulls in Swift plugins. Set lowest version to 12.2 to avoid benchmark noise.
            // This should be removed when when Flutter's minimum supported version is >12.2.
            'FLUTTER_XCODE_IPHONEOS_DEPLOYMENT_TARGET': '12.2',
          },
        );
        watch.stop();
        final Directory buildDirectory = dir(path.join(cwd, 'build'));
        final String? appPath = _findDarwinAppInBuildDirectory(buildDirectory.path);
        if (appPath == null) {
          throw 'Failed to find app bundle in ${buildDirectory.path}';
        }
        // Validate changes in Dart snapshot format and data layout do not
        // change compression size. This also simulates the size of an IPA on iOS.
        await exec('tar', <String>['-zcf', 'build/app.tar.gz', appPath]);
        releaseSizeInBytes = await file('$cwd/build/app.tar.gz').length();
        if (reportPackageContentSizes) {
          final Map<String, Object> sizeMetrics = await getSizesFromDarwinApp(
            appPath: appPath,
            operatingSystem: deviceOperatingSystem,
          );
          metrics.addAll(sizeMetrics);
        }
      case DeviceOperatingSystem.android:
      case DeviceOperatingSystem.androidArm:
        options.insert(0, 'apk');
        options.add('--target-platform=android-arm');
        options.add('--tree-shake-icons');
        options.add('--split-debug-info=infos/');
        watch.start();
        await flutter('build', options: options);
        watch.stop();
        final String apkPath = '$cwd/build/app/outputs/flutter-apk/app-release.apk';
        final File apk = file(apkPath);
        releaseSizeInBytes = apk.lengthSync();
        if (reportPackageContentSizes) {
          metrics.addAll(await getSizesFromApk(apkPath));
        }
      case DeviceOperatingSystem.androidArm64:
        options.insert(0, 'apk');
        options.add('--target-platform=android-arm64');
        options.add('--tree-shake-icons');
        options.add('--split-debug-info=infos/');
        watch.start();
        await flutter('build', options: options);
        watch.stop();
        final String apkPath = '$cwd/build/app/outputs/flutter-apk/app-release.apk';
        final File apk = file(apkPath);
        releaseSizeInBytes = apk.lengthSync();
        if (reportPackageContentSizes) {
          metrics.addAll(await getSizesFromApk(apkPath));
        }
      case DeviceOperatingSystem.fake:
        throw Exception('Unsupported option for fake devices');
      case DeviceOperatingSystem.fuchsia:
        throw Exception('Unsupported option for Fuchsia devices');
      case DeviceOperatingSystem.linux:
        throw Exception('Unsupported option for Linux devices');
      case DeviceOperatingSystem.windows:
        unawaited(stderr.flush());
        options.insert(0, 'windows');
        options.add('--tree-shake-icons');
        options.add('--split-debug-info=infos/');
        watch.start();
        await flutter('build', options: options);
        watch.stop();
        final String basename = path.basename(cwd);
        final String arch = Abi.current() == Abi.windowsX64 ? 'x64' : 'arm64';
        final String exePath = path.join(
          cwd,
          'build',
          'windows',
          arch,
          'runner',
          'release',
          '$basename.exe',
        );
        final File exe = file(exePath);
        // On Windows, we do not produce a single installation package file,
        // rather a directory containing an .exe and .dll files.
        // The release size is set to the size of the produced .exe file
        releaseSizeInBytes = exe.lengthSync();
    }

    metrics.addAll(<String, dynamic>{
      'release_${deleteGradleCache ? 'initial' : 'full'}_compile_millis': watch.elapsedMilliseconds,
      'release_size_bytes': releaseSizeInBytes,
    });

    return metrics;
  }

  Future<Map<String, dynamic>> _compileDebug({
    required bool deleteGradleCache,
    required bool clean,
    required String metricKey,
  }) async {
    if (clean) {
      await flutter('clean');
    }
    if (deleteGradleCache) {
      final Directory gradleCacheDir = Directory('$testDirectory/android/.gradle');
      rmTree(gradleCacheDir);
    }
    final Stopwatch watch = Stopwatch();
    final List<String> options = <String>['--debug'];
    switch (deviceOperatingSystem) {
      case DeviceOperatingSystem.ios:
        options.insert(0, 'ios');
      case DeviceOperatingSystem.android:
      case DeviceOperatingSystem.androidArm:
        options.insert(0, 'apk');
        options.add('--target-platform=android-arm');
      case DeviceOperatingSystem.androidArm64:
        options.insert(0, 'apk');
        options.add('--target-platform=android-arm64');
      case DeviceOperatingSystem.fake:
        throw Exception('Unsupported option for fake devices');
      case DeviceOperatingSystem.fuchsia:
        throw Exception('Unsupported option for Fuchsia devices');
      case DeviceOperatingSystem.linux:
        throw Exception('Unsupported option for Linux devices');
      case DeviceOperatingSystem.macos:
        unawaited(stderr.flush());
        options.insert(0, 'macos');
      case DeviceOperatingSystem.windows:
        unawaited(stderr.flush());
        options.insert(0, 'windows');
    }
    watch.start();
    await flutter('build', options: options);
    watch.stop();

    return <String, dynamic>{metricKey: watch.elapsedMilliseconds};
  }

  static Future<Map<String, Object>> getSizesFromDarwinApp({
    required String appPath,
    required DeviceOperatingSystem operatingSystem,
  }) async {
    late final File flutterFramework;
    late final String frameworkDirectory;
    switch (deviceOperatingSystem) {
      case DeviceOperatingSystem.ios:
        frameworkDirectory = path.join(appPath, 'Frameworks');
        flutterFramework = File(path.join(frameworkDirectory, 'Flutter.framework', 'Flutter'));
      case DeviceOperatingSystem.macos:
        frameworkDirectory = path.join(appPath, 'Contents', 'Frameworks');
        flutterFramework = File(
          path.join(frameworkDirectory, 'FlutterMacOS.framework', 'FlutterMacOS'),
        ); // https://github.com/flutter/flutter/issues/70413
      case DeviceOperatingSystem.android:
      case DeviceOperatingSystem.androidArm:
      case DeviceOperatingSystem.androidArm64:
      case DeviceOperatingSystem.fake:
      case DeviceOperatingSystem.fuchsia:
      case DeviceOperatingSystem.linux:
      case DeviceOperatingSystem.windows:
        throw Exception('Called ${CompileTest.getSizesFromDarwinApp} with $operatingSystem.');
    }

    final File appFramework = File(path.join(frameworkDirectory, 'App.framework', 'App'));

    return <String, Object>{
      'app_framework_uncompressed_bytes': await appFramework.length(),
      'flutter_framework_uncompressed_bytes': await flutterFramework.length(),
    };
  }

  static Future<Map<String, dynamic>> getSizesFromApk(String apkPath) async {
    final String output = await eval('unzip', <String>['-v', apkPath]);
    final List<String> lines = output.split('\n');
    final Map<String, _UnzipListEntry> fileToMetadata = <String, _UnzipListEntry>{};

    // First three lines are header, last two lines are footer.
    for (int i = 3; i < lines.length - 2; i++) {
      final _UnzipListEntry entry = _UnzipListEntry.fromLine(lines[i]);
      fileToMetadata[entry.path] = entry;
    }

    final _UnzipListEntry libflutter = fileToMetadata['lib/armeabi-v7a/libflutter.so']!;
    final _UnzipListEntry libapp = fileToMetadata['lib/armeabi-v7a/libapp.so']!;
    final _UnzipListEntry license = fileToMetadata['assets/flutter_assets/NOTICES.Z']!;

    return <String, dynamic>{
      'libflutter_uncompressed_bytes': libflutter.uncompressedSize,
      'libflutter_compressed_bytes': libflutter.compressedSize,
      'libapp_uncompressed_bytes': libapp.uncompressedSize,
      'libapp_compressed_bytes': libapp.compressedSize,
      'license_uncompressed_bytes': license.uncompressedSize,
      'license_compressed_bytes': license.compressedSize,
    };
  }
}

/// Measure application memory usage.
class MemoryTest {
  MemoryTest(this.project, this.test, this.package, {this.requiresTapToStart = false});

  final String project;
  final String test;
  final String package;
  final bool requiresTapToStart;

  /// Completes when the log line specified in the last call to
  /// [prepareForNextMessage] is seen by `adb logcat`.
  Future<void>? get receivedNextMessage => _receivedNextMessage?.future;
  Completer<void>? _receivedNextMessage;
  String? _nextMessage;

  /// Prepares the [receivedNextMessage] future such that it will complete
  /// when `adb logcat` sees a log line with the given `message`.
  void prepareForNextMessage(String message) {
    _nextMessage = message;
    _receivedNextMessage = Completer<void>();
  }

  int get iterationCount => 10;

  Device? get device => _device;
  Device? _device;

  Future<TaskResult> run() {
    return inDirectory<TaskResult>(project, () async {
      // This test currently only works on Android, because device.logcat,
      // device.getMemoryStats, etc, aren't implemented for iOS.

      _device = await devices.workingDevice;
      await device!.unlock();
      await flutter('packages', options: <String>['get']);

      final StreamSubscription<String> adb = device!.logcat.listen((String data) {
        if (data.contains('==== MEMORY BENCHMARK ==== $_nextMessage ====')) {
          _receivedNextMessage?.complete();
        }
      });

      for (int iteration = 0; iteration < iterationCount; iteration += 1) {
        print('running memory test iteration $iteration...');
        _startMemoryUsage = null;
        await useMemory();
        assert(_startMemoryUsage != null);
        assert(_startMemory.length == iteration + 1);
        assert(_endMemory.length == iteration + 1);
        assert(_diffMemory.length == iteration + 1);
        print('terminating...');
        await device!.stop(package);
        await Future<void>.delayed(const Duration(milliseconds: 10));
      }

      await adb.cancel();
      await device!.uninstallApp();

      final ListStatistics startMemoryStatistics = ListStatistics(_startMemory);
      final ListStatistics endMemoryStatistics = ListStatistics(_endMemory);
      final ListStatistics diffMemoryStatistics = ListStatistics(_diffMemory);

      final Map<String, dynamic> memoryUsage = <String, dynamic>{
        ...startMemoryStatistics.asMap('start'),
        ...endMemoryStatistics.asMap('end'),
        ...diffMemoryStatistics.asMap('diff'),
      };

      _device = null;
      _startMemory.clear();
      _endMemory.clear();
      _diffMemory.clear();

      return TaskResult.success(memoryUsage, benchmarkScoreKeys: memoryUsage.keys.toList());
    });
  }

  /// Starts the app specified by [test] on the [device].
  ///
  /// The [run] method will terminate it by its package name ([package]).
  Future<void> launchApp() async {
    prepareForNextMessage('READY');
    print('launching $project$test on device...');
    await flutter(
      'run',
      options: <String>['--verbose', '--release', '--no-resident', '-d', device!.deviceId, test],
    );
    print('awaiting "ready" message...');
    await receivedNextMessage;
  }

  /// Taps the application and looks for acknowledgement.
  ///
  /// This is used by several tests to ensure scrolling gestures are installed.
  Future<void> tapNotification() async {
    // Keep "tapping" the device till it responds with the string we expect,
    // or throw an error instead of tying up the infrastructure for 30 minutes.
    prepareForNextMessage('TAPPED');
    bool tapped = false;
    int tapCount = 0;
    await Future.any(<Future<void>>[
      () async {
        while (true) {
          if (tapped) {
            break;
          }
          tapCount += 1;
          print('tapping device... [$tapCount]');
          await device!.tap(100, 100);
          await Future<void>.delayed(const Duration(milliseconds: 100));
        }
      }(),
      () async {
        print('awaiting "tapped" message... (timeout: 10 seconds)');
        try {
          await receivedNextMessage?.timeout(const Duration(seconds: 10));
        } finally {
          tapped = true;
        }
      }(),
    ]);
  }

  /// To change the behavior of the test, override this.
  ///
  /// Make sure to call recordStart() and recordEnd() once each in that order.
  ///
  /// By default it just launches the app, records memory usage, taps the device,
  /// awaits a DONE notification, and records memory usage again.
  Future<void> useMemory() async {
    await launchApp();
    await recordStart();
    if (requiresTapToStart) {
      await tapNotification();
    }

    prepareForNextMessage('DONE');
    print('awaiting "done" message...');
    await receivedNextMessage;

    await recordEnd();
  }

  final List<int> _startMemory = <int>[];
  final List<int> _endMemory = <int>[];
  final List<int> _diffMemory = <int>[];

  Map<String, dynamic>? _startMemoryUsage;

  @protected
  Future<void> recordStart() async {
    assert(_startMemoryUsage == null);
    print('snapshotting memory usage...');
    _startMemoryUsage = await device!.getMemoryStats(package);
  }

  @protected
  Future<void> recordEnd() async {
    assert(_startMemoryUsage != null);
    print('snapshotting memory usage...');
    final Map<String, dynamic> endMemoryUsage = await device!.getMemoryStats(package);
    _startMemory.add(_startMemoryUsage!['total_kb'] as int);
    _endMemory.add(endMemoryUsage['total_kb'] as int);
    _diffMemory.add((endMemoryUsage['total_kb'] as int) - (_startMemoryUsage!['total_kb'] as int));
  }
}

class DevToolsMemoryTest {
  DevToolsMemoryTest(this.project, this.driverTest);

  final String project;
  final String driverTest;

  Future<TaskResult> run() {
    return inDirectory<TaskResult>(project, () async {
      _device = await devices.workingDevice;
      await _device.unlock();

      await flutter(
        'drive',
        driveWithDds: true,
        options: <String>[
          '-d',
          _device.deviceId,
          '--profile',
          '--profile-memory',
          _kJsonFileName,
          '--no-publish-port',
          '-v',
          driverTest,
        ],
      );

      final Map<String, dynamic> data =
          json.decode(file('$project/$_kJsonFileName').readAsStringSync()) as Map<String, dynamic>;
      final List<dynamic> samples =
          (data['samples'] as Map<String, dynamic>)['data'] as List<dynamic>;
      int maxRss = 0;
      int maxAdbTotal = 0;
      for (final Map<String, dynamic> sample in samples.cast<Map<String, dynamic>>()) {
        if (sample['rss'] != null) {
          maxRss = math.max(maxRss, sample['rss'] as int);
        }
        if (sample['adb_memoryInfo'] != null) {
          maxAdbTotal = math.max(
            maxAdbTotal,
            (sample['adb_memoryInfo'] as Map<String, dynamic>)['Total'] as int,
          );
        }
      }

      return TaskResult.success(
        <String, dynamic>{'maxRss': maxRss, 'maxAdbTotal': maxAdbTotal},
        benchmarkScoreKeys: <String>['maxRss', 'maxAdbTotal'],
      );
    });
  }

  late Device _device;

  static const String _kJsonFileName = 'devtools_memory.json';
}

enum ReportedDurationTestFlavor { debug, profile, release }

String _reportedDurationTestToString(ReportedDurationTestFlavor flavor) {
  return switch (flavor) {
    ReportedDurationTestFlavor.debug => 'debug',
    ReportedDurationTestFlavor.profile => 'profile',
    ReportedDurationTestFlavor.release => 'release',
  };
}

class ReportedDurationTest {
  ReportedDurationTest(this.flavor, this.project, this.test, this.package, this.durationPattern);

  final ReportedDurationTestFlavor flavor;
  final String project;
  final String test;
  final String package;
  final RegExp durationPattern;

  final Completer<int> durationCompleter = Completer<int>();

  int get iterationCount => 10;

  Device? get device => _device;
  Device? _device;

  Future<TaskResult> run() {
    return inDirectory<TaskResult>(project, () async {
      // This test currently only works on Android, because device.logcat,
      // device.getMemoryStats, etc, aren't implemented for iOS.

      _device = await devices.workingDevice;
      await device!.unlock();
      await flutter('packages', options: <String>['get']);

      final StreamSubscription<String> adb = device!.logcat.listen((String data) {
        if (durationPattern.hasMatch(data)) {
          durationCompleter.complete(int.parse(durationPattern.firstMatch(data)!.group(1)!));
        }
      });
      print('launching $project$test on device...');
      await flutter(
        'run',
        options: <String>[
          '--verbose',
          '--no-publish-port',
          '--no-fast-start',
          '--${_reportedDurationTestToString(flavor)}',
          '--no-resident',
          '-d',
          device!.deviceId,
          test,
        ],
      );

      final int duration = await durationCompleter.future;
      print('terminating...');
      await device!.stop(package);
      await adb.cancel();

      _device = null;

      final Map<String, dynamic> reportedDuration = <String, dynamic>{'duration': duration};
      _device = null;

      return TaskResult.success(
        reportedDuration,
        benchmarkScoreKeys: reportedDuration.keys.toList(),
      );
    });
  }
}

/// Holds simple statistics of an odd-lengthed list of integers.
class ListStatistics {
  factory ListStatistics(Iterable<int> data) {
    assert(data.isNotEmpty);
    assert(data.length.isOdd);
    final List<int> sortedData = data.toList()..sort();
    return ListStatistics._(
      sortedData.first,
      sortedData.last,
      sortedData[(sortedData.length - 1) ~/ 2],
    );
  }

  const ListStatistics._(this.min, this.max, this.median);

  final int min;
  final int max;
  final int median;

  Map<String, int> asMap(String prefix) {
    return <String, int>{'$prefix-min': min, '$prefix-max': max, '$prefix-median': median};
  }
}

class _UnzipListEntry {
  factory _UnzipListEntry.fromLine(String line) {
    final List<String> data = line.trim().split(RegExp(r'\s+'));
    assert(data.length == 8);
    return _UnzipListEntry._(
      uncompressedSize: int.parse(data[0]),
      compressedSize: int.parse(data[2]),
      path: data[7],
    );
  }

  _UnzipListEntry._({
    required this.uncompressedSize,
    required this.compressedSize,
    required this.path,
  }) : assert(compressedSize <= uncompressedSize);

  final int uncompressedSize;
  final int compressedSize;
  final String path;
}

/// Wait for up to 1 hour for the file to appear.
Future<File> waitForFile(String path) async {
  for (int i = 0; i < 180; i += 1) {
    final File file = File(path);
    print('looking for ${file.path}');
    if (file.existsSync()) {
      return file;
    }
    await Future<void>.delayed(const Duration(seconds: 20));
  }
  throw StateError('Did not find vmservice out file after 1 hour');
}

String? _findDarwinAppInBuildDirectory(String searchDirectory) {
  for (final FileSystemEntity entity in Directory(searchDirectory).listSync(recursive: true)) {
    if (entity.path.endsWith('.app')) {
      return entity.path;
    }
  }
  return null;
}
