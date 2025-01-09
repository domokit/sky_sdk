// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'common.dart';
import 'src/animated_advanced_blend.dart';
import 'src/animated_blur_backdrop_filter.dart';
import 'src/animated_complex_image_filtered.dart';
import 'src/animated_complex_opacity.dart';
import 'src/animated_image.dart';
import 'src/animated_placeholder.dart';
import 'src/animation_with_microtasks.dart';
import 'src/backdrop_filter.dart';
import 'src/clipper_cache.dart';
import 'src/color_filter_and_fade.dart';
import 'src/color_filter_cache.dart';
import 'src/color_filter_with_unstable_child.dart';
import 'src/cubic_bezier.dart';
import 'src/cull_opacity.dart';
import 'src/draw_atlas.dart';
import 'src/draw_points.dart';
import 'src/draw_vertices.dart';
import 'src/filtered_child_animation.dart';
import 'src/fullscreen_textfield.dart';
import 'src/gradient_perf.dart';
import 'src/heavy_grid_view.dart';
import 'src/large_image_changer.dart';
import 'src/large_images.dart';
import 'src/list_text_layout.dart';
import 'src/multi_widget_construction.dart';
import 'src/opacity_peephole.dart';
import 'src/path_tessellation.dart';
import 'src/picture_cache.dart';
import 'src/picture_cache_complexity_scoring.dart';
import 'src/post_backdrop_filter.dart';
import 'src/raster_cache_use_memory.dart';
import 'src/rrect_blur.dart' show RRectBlur;
import 'src/shader_mask_cache.dart';
import 'src/simple_animation.dart';
import 'src/simple_scroll.dart';
import 'src/sliders.dart';
import 'src/text.dart';
import 'src/very_long_picture_scrolling.dart';

const String kMacrobenchmarks = 'Macrobenchmarks';

void main() => runApp(const MacrobenchmarksApp());

class MacrobenchmarksApp extends StatelessWidget {
  const MacrobenchmarksApp({super.key, this.initialRoute = '/'});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: kMacrobenchmarks,
      initialRoute: initialRoute,
      routes: <String, WidgetBuilder>{
        '/': (BuildContext context) => const HomePage(),
        kCullOpacityRouteName: (BuildContext context) => const CullOpacityPage(),
        kCubicBezierRouteName: (BuildContext context) => const CubicBezierPage(),
        kBackdropFilterRouteName: (BuildContext context) => const BackdropFilterPage(),
        kPostBackdropFilterRouteName: (BuildContext context) => const PostBackdropFilterPage(),
        kSimpleAnimationRouteName: (BuildContext context) => const SimpleAnimationPage(),
        kPictureCacheRouteName: (BuildContext context) => const PictureCachePage(),
        kPictureCacheComplexityScoringRouteName:
            (BuildContext context) => const PictureCacheComplexityScoringPage(),
        kLargeImageChangerRouteName: (BuildContext context) => const LargeImageChangerPage(),
        kLargeImagesRouteName: (BuildContext context) => const LargeImagesPage(),
        kTextRouteName: (BuildContext context) => const TextPage(),
        kPathTessellationRouteName: (BuildContext context) => const PathTessellationPage(),
        kFullscreenTextRouteName: (BuildContext context) => const TextFieldPage(),
        kAnimatedPlaceholderRouteName: (BuildContext context) => const AnimatedPlaceholderPage(),
        kClipperCacheRouteName: (BuildContext context) => const ClipperCachePage(),
        kColorFilterAndFadeRouteName: (BuildContext context) => const ColorFilterAndFadePage(),
        kColorFilterCacheRouteName: (BuildContext context) => const ColorFilterCachePage(),
        kColorFilterWithUnstableChildName:
            (BuildContext context) => const ColorFilterWithUnstableChildPage(),
        kFadingChildAnimationRouteName:
            (BuildContext context) => const FilteredChildAnimationPage(FilterType.opacity),
        kImageFilteredTransformAnimationRouteName:
            (BuildContext context) => const FilteredChildAnimationPage(FilterType.rotateFilter),
        kMultiWidgetConstructionRouteName:
            (BuildContext context) => const MultiWidgetConstructTable(10, 20),
        kHeavyGridViewRouteName: (BuildContext context) => const HeavyGridViewPage(),
        kRasterCacheUseMemory: (BuildContext context) => const RasterCacheUseMemory(),
        kShaderMaskCacheRouteName: (BuildContext context) => const ShaderMaskCachePage(),
        kSimpleScrollRouteName: (BuildContext context) => const SimpleScroll(),
        kAnimationWithMicrotasksRouteName:
            (BuildContext context) => const AnimationWithMicrotasks(),
        kAnimatedImageRouteName: (BuildContext context) => const AnimatedImagePage(),
        kOpacityPeepholeRouteName: (BuildContext context) => const OpacityPeepholePage(),
        ...opacityPeepholeRoutes,
        kGradientPerfRouteName: (BuildContext context) => const GradientPerfHomePage(),
        ...gradientPerfRoutes,
        kAnimatedComplexOpacityPerfRouteName:
            (BuildContext context) => const AnimatedComplexOpacity(),
        kListTextLayoutRouteName: (BuildContext context) => const ColumnOfText(),
        kAnimatedComplexImageFilteredPerfRouteName:
            (BuildContext context) => const AnimatedComplexImageFiltered(),
        kAnimatedBlurBackdropFilter: (BuildContext context) => const AnimatedBlurBackdropFilter(),
        kSlidersRouteName: (BuildContext context) => const SlidersPage(),
        kDrawPointsPageRougeName: (BuildContext context) => const DrawPointsPage(),
        kDrawVerticesPageRouteName: (BuildContext context) => const DrawVerticesPage(),
        kDrawAtlasPageRouteName: (BuildContext context) => const DrawAtlasPage(),
        kAnimatedAdvancedBlend: (BuildContext context) => const AnimatedAdvancedBlend(),
        kRRectBlurRouteName: (BuildContext context) => const RRectBlur(),
        kVeryLongPictureScrollingRouteName:
            (BuildContext context) => const VeryLongPictureScrollingPerf(),
      },
    );
  }

  final String initialRoute;
}

class HomePage extends StatelessWidget {
  const HomePage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text(kMacrobenchmarks)),
      body: ListView(
        key: const Key(kScrollableName),
        children: <Widget>[
          ElevatedButton(
            key: const Key(kCullOpacityRouteName),
            child: const Text('Cull opacity'),
            onPressed: () {
              Navigator.pushNamed(context, kCullOpacityRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kCubicBezierRouteName),
            child: const Text('Cubic Bezier'),
            onPressed: () {
              Navigator.pushNamed(context, kCubicBezierRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kBackdropFilterRouteName),
            child: const Text('Backdrop Filter'),
            onPressed: () {
              Navigator.pushNamed(context, kBackdropFilterRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kPostBackdropFilterRouteName),
            child: const Text('Post Backdrop Filter'),
            onPressed: () {
              Navigator.pushNamed(context, kPostBackdropFilterRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kSimpleAnimationRouteName),
            child: const Text('Simple Animation'),
            onPressed: () {
              Navigator.pushNamed(context, kSimpleAnimationRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kPictureCacheRouteName),
            child: const Text('Picture Cache'),
            onPressed: () {
              Navigator.pushNamed(context, kPictureCacheRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kPictureCacheComplexityScoringRouteName),
            child: const Text('Picture Cache Complexity Scoring'),
            onPressed: () {
              Navigator.pushNamed(context, kPictureCacheComplexityScoringRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kLargeImagesRouteName),
            child: const Text('Large Images'),
            onPressed: () {
              Navigator.pushNamed(context, kLargeImagesRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kPathTessellationRouteName),
            child: const Text('Path Tessellation'),
            onPressed: () {
              Navigator.pushNamed(context, kPathTessellationRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kTextRouteName),
            child: const Text('Text'),
            onPressed: () {
              Navigator.pushNamed(context, kTextRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kFullscreenTextRouteName),
            child: const Text('Fullscreen Text'),
            onPressed: () {
              Navigator.pushNamed(context, kFullscreenTextRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kAnimatedPlaceholderRouteName),
            child: const Text('Animated Placeholder'),
            onPressed: () {
              Navigator.pushNamed(context, kAnimatedPlaceholderRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kClipperCacheRouteName),
            child: const Text('Clipper Cache'),
            onPressed: () {
              Navigator.pushNamed(context, kClipperCacheRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kColorFilterAndFadeRouteName),
            child: const Text('Color Filter and Fade'),
            onPressed: () {
              Navigator.pushNamed(context, kColorFilterAndFadeRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kColorFilterCacheRouteName),
            child: const Text('Color Filter Cache'),
            onPressed: () {
              Navigator.pushNamed(context, kColorFilterCacheRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kColorFilterWithUnstableChildName),
            child: const Text('Color Filter with Unstable Child'),
            onPressed: () {
              Navigator.pushNamed(context, kColorFilterWithUnstableChildName);
            },
          ),
          ElevatedButton(
            key: const Key(kRasterCacheUseMemory),
            child: const Text('RasterCache Use Memory'),
            onPressed: () {
              Navigator.pushNamed(context, kRasterCacheUseMemory);
            },
          ),
          ElevatedButton(
            key: const Key(kShaderMaskCacheRouteName),
            child: const Text('Shader Mask Cache'),
            onPressed: () {
              Navigator.pushNamed(context, kShaderMaskCacheRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kFadingChildAnimationRouteName),
            child: const Text('Fading Child Animation'),
            onPressed: () {
              Navigator.pushNamed(context, kFadingChildAnimationRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kImageFilteredTransformAnimationRouteName),
            child: const Text('ImageFiltered Transform Animation'),
            onPressed: () {
              Navigator.pushNamed(context, kImageFilteredTransformAnimationRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kMultiWidgetConstructionRouteName),
            child: const Text('Widget Construction and Destruction'),
            onPressed: () {
              Navigator.pushNamed(context, kMultiWidgetConstructionRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kHeavyGridViewRouteName),
            child: const Text('Heavy Grid View'),
            onPressed: () {
              Navigator.pushNamed(context, kHeavyGridViewRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kLargeImageChangerRouteName),
            child: const Text('Large Image Changer'),
            onPressed: () {
              Navigator.pushNamed(context, kLargeImageChangerRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kAnimationWithMicrotasksRouteName),
            child: const Text('Animation With Microtasks'),
            onPressed: () {
              Navigator.pushNamed(context, kAnimationWithMicrotasksRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kAnimatedImageRouteName),
            child: const Text('Animated Image'),
            onPressed: () {
              Navigator.pushNamed(context, kAnimatedImageRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kOpacityPeepholeRouteName),
            child: const Text('Opacity Peephole tests'),
            onPressed: () {
              Navigator.pushNamed(context, kOpacityPeepholeRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kGradientPerfRouteName),
            child: const Text('Gradient performance tests'),
            onPressed: () {
              Navigator.pushNamed(context, kGradientPerfRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kAnimatedComplexOpacityPerfRouteName),
            child: const Text('Animated complex opacity perf'),
            onPressed: () {
              Navigator.pushNamed(context, kAnimatedComplexOpacityPerfRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kAnimatedComplexImageFilteredPerfRouteName),
            child: const Text('Animated complex image filtered perf'),
            onPressed: () {
              Navigator.pushNamed(context, kAnimatedComplexImageFilteredPerfRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kListTextLayoutRouteName),
            child: const Text('A list with lots of text'),
            onPressed: () {
              Navigator.pushNamed(context, kListTextLayoutRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kAnimatedBlurBackdropFilter),
            child: const Text('An animating backdrop filter'),
            onPressed: () {
              Navigator.pushNamed(context, kAnimatedBlurBackdropFilter);
            },
          ),
          ElevatedButton(
            key: const Key(kSlidersRouteName),
            child: const Text('Sliders'),
            onPressed: () {
              Navigator.pushNamed(context, kSlidersRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kDrawPointsPageRougeName),
            child: const Text('Draw Points'),
            onPressed: () {
              Navigator.pushNamed(context, kDrawPointsPageRougeName);
            },
          ),
          ElevatedButton(
            key: const Key(kDrawVerticesPageRouteName),
            child: const Text('Draw Vertices'),
            onPressed: () {
              Navigator.pushNamed(context, kDrawVerticesPageRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kDrawAtlasPageRouteName),
            child: const Text('Draw Atlas'),
            onPressed: () {
              Navigator.pushNamed(context, kDrawAtlasPageRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kAnimatedAdvancedBlend),
            child: const Text('Animated Advanced Blend'),
            onPressed: () {
              Navigator.pushNamed(context, kAnimatedAdvancedBlend);
            },
          ),
          ElevatedButton(
            key: const Key(kRRectBlurRouteName),
            child: const Text('Rounded Rect Blur'),
            onPressed: () {
              Navigator.pushNamed(context, kRRectBlurRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kVeryLongPictureScrollingRouteName),
            child: const Text('Very Long Picture Scrolling'),
            onPressed: () {
              Navigator.pushNamed(context, kVeryLongPictureScrollingRouteName);
            },
          ),
        ],
      ),
    );
  }
}
