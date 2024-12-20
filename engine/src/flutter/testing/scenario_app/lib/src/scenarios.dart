// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'animated_color_square.dart';
import 'bogus_font_text.dart';
import 'darwin_app_extension_scenario.dart';
import 'darwin_system_font.dart';
import 'get_bitmap_scenario.dart';
import 'initial_route_reply.dart';
import 'locale_initialization.dart';
import 'platform_view.dart';
import 'poppable_screen.dart';
import 'scenario.dart';
import 'solid_blue.dart';
import 'texture.dart';
import 'touches_scenario.dart';

typedef _ScenarioFactory = Scenario Function(FlutterView view);

int _viewId = 0;

Map<String, _ScenarioFactory> _scenarios = <String, _ScenarioFactory>{
  'animated_color_square': (FlutterView view) => AnimatedColorSquareScenario(view),
  'solid_blue': (FlutterView view) => SolidBlueScenario(view),
  'locale_initialization': (FlutterView view) => LocaleInitialization(view),
  'platform_view': (FlutterView view) => PlatformViewScenario(view, id: _viewId++),
  'platform_view_no_overlay_intersection':
      (FlutterView view) => PlatformViewNoOverlayIntersectionScenario(view, id: _viewId++),
  'platform_view_larger_than_display_size':
      (FlutterView view) => PlatformViewLargerThanDisplaySize(view, id: _viewId++),
  'platform_view_partial_intersection':
      (FlutterView view) => PlatformViewPartialIntersectionScenario(view, id: _viewId++),
  'platform_view_two_intersecting_overlays':
      (FlutterView view) => PlatformViewTwoIntersectingOverlaysScenario(view, id: _viewId++),
  'platform_view_one_overlay_two_intersecting_overlays':
      (FlutterView view) =>
          PlatformViewOneOverlayTwoIntersectingOverlaysScenario(view, id: _viewId++),
  'platform_view_multiple_without_overlays':
      (FlutterView view) =>
          MultiPlatformViewWithoutOverlaysScenario(view, firstId: _viewId++, secondId: _viewId++),
  'platform_view_max_overlays':
      (FlutterView view) => PlatformViewMaxOverlaysScenario(view, id: _viewId++),
  'platform_view_surrounding_layers_fractional_coordinate':
      (FlutterView view) =>
          PlatformViewSurroundingLayersFractionalCoordinateScenario(view, id: _viewId++),
  'platform_view_partial_intersection_fractional_coordinate':
      (FlutterView view) =>
          PlatformViewPartialIntersectionFractionalCoordinateScenario(view, id: _viewId++),
  'platform_view_cliprect': (FlutterView view) => PlatformViewClipRectScenario(view, id: _viewId++),
  'platform_view_cliprect_multiple_clips':
      (FlutterView view) => PlatformViewClipRectMultipleClipsScenario(view, id: _viewId++),
  'platform_view_cliprect_with_transform':
      (FlutterView view) => PlatformViewClipRectWithTransformScenario(view, id: _viewId++),
  'platform_view_cliprect_with_transform_multiple_clips':
      (FlutterView view) =>
          PlatformViewClipRectWithTransformMultipleClipsScenario(view, id: _viewId++),
  'platform_view_cliprect_after_moved':
      (FlutterView view) => PlatformViewClipRectAfterMovedScenario(view, id: _viewId++),
  'platform_view_cliprect_after_moved_multiple_clips':
      (FlutterView view) =>
          PlatformViewClipRectAfterMovedMultipleClipsScenario(view, id: _viewId++),
  'platform_view_cliprrect':
      (FlutterView view) => PlatformViewClipRRectScenario(view, id: _viewId++),
  'platform_view_cliprrect_multiple_clips':
      (FlutterView view) => PlatformViewClipRRectMultipleClipsScenario(view, id: _viewId++),
  'platform_view_large_cliprrect':
      (FlutterView view) => PlatformViewLargeClipRRectScenario(view, id: _viewId++),
  'platform_view_large_cliprrect_multiple_clips':
      (FlutterView view) => PlatformViewLargeClipRRectMultipleClipsScenario(view, id: _viewId++),
  'platform_view_cliprrect_with_transform':
      (FlutterView view) => PlatformViewClipRRectWithTransformScenario(view, id: _viewId++),
  'platform_view_cliprrect_with_transform_multiple_clips':
      (FlutterView view) =>
          PlatformViewClipRRectWithTransformMultipleClipsScenario(view, id: _viewId++),
  'platform_view_large_cliprrect_with_transform':
      (FlutterView view) => PlatformViewLargeClipRRectWithTransformScenario(view, id: _viewId++),
  'platform_view_large_cliprrect_with_transform_multiple_clips':
      (FlutterView view) =>
          PlatformViewLargeClipRRectWithTransformMultipleClipsScenario(view, id: _viewId++),
  'platform_view_clippath': (FlutterView view) => PlatformViewClipPathScenario(view, id: _viewId++),
  'platform_view_clippath_multiple_clips':
      (FlutterView view) => PlatformViewClipPathMultipleClipsScenario(view, id: _viewId++),
  'platform_view_clippath_with_transform':
      (FlutterView view) => PlatformViewClipPathWithTransformScenario(view, id: _viewId++),
  'platform_view_clippath_with_transform_multiple_clips':
      (FlutterView view) =>
          PlatformViewClipPathWithTransformMultipleClipsScenario(view, id: _viewId++),
  'platform_view_transform':
      (FlutterView view) => PlatformViewTransformScenario(view, id: _viewId++),
  'platform_view_opacity': (FlutterView view) => PlatformViewOpacityScenario(view, id: _viewId++),
  'platform_view_with_other_backdrop_filter':
      (FlutterView view) => PlatformViewWithOtherBackDropFilter(view, id: _viewId++),
  'two_platform_views_with_other_backdrop_filter':
      (FlutterView view) =>
          TwoPlatformViewsWithOtherBackDropFilter(view, firstId: _viewId++, secondId: _viewId++),
  'platform_view_with_negative_backdrop_filter':
      (FlutterView view) => PlatformViewWithNegativeBackDropFilter(view, id: _viewId++),
  'platform_view_multiple':
      (FlutterView view) =>
          MultiPlatformViewScenario(view, firstId: _viewId++, secondId: _viewId++),
  'platform_view_multiple_background_foreground':
      (FlutterView view) => MultiPlatformViewBackgroundForegroundScenario(
        view,
        firstId: _viewId++,
        secondId: _viewId++,
      ),
  'non_full_screen_flutter_view_platform_view':
      (FlutterView view) => NonFullScreenFlutterViewPlatformViewScenario(view, id: _viewId++),
  'poppable_screen': (FlutterView view) => PoppableScreenScenario(view),
  'platform_view_rotate': (FlutterView view) => PlatformViewScenario(view, id: _viewId++),
  'platform_view_gesture_reject_eager':
      (FlutterView view) => PlatformViewForTouchIOSScenario(view, id: _viewId++, accept: false),
  'platform_view_gesture_accept':
      (FlutterView view) => PlatformViewForTouchIOSScenario(view, id: _viewId++, accept: true),
  'platform_view_gesture_reject_after_touches_ended':
      (FlutterView view) => PlatformViewForTouchIOSScenario(
        view,
        id: _viewId++,
        accept: false,
        rejectUntilTouchesEnded: true,
      ),
  'platform_view_gesture_accept_with_overlapping_platform_views':
      (FlutterView view) => PlatformViewForOverlappingPlatformViewsScenario(
        view,
        foregroundId: _viewId++,
        backgroundId: _viewId++,
      ),
  'platform_view_scrolling_under_widget':
      (FlutterView view) => PlatformViewScrollingUnderWidget(
        view,
        firstPlatformViewId: _viewId++,
        lastPlatformViewId: _viewId += 16,
      ),
  'platform_views_with_clips_scrolling':
      (FlutterView view) => PlatformViewsWithClipsScrolling(
        view,
        firstPlatformViewId: _viewId++,
        lastPlatformViewId: _viewId += 16,
      ),
  'platform_views_with_clips_scrolling_multiple_clips':
      (FlutterView view) => PlatformViewsWithClipsScrollingMultipleClips(
        view,
        firstPlatformViewId: _viewId++,
        lastPlatformViewId: _viewId += 16,
      ),
  'two_platform_view_clip_rect':
      (FlutterView view) => TwoPlatformViewClipRect(view, firstId: _viewId++, secondId: _viewId++),
  'two_platform_view_clip_rect_multiple_clips':
      (FlutterView view) =>
          TwoPlatformViewClipRectMultipleClips(view, firstId: _viewId++, secondId: _viewId++),
  'two_platform_view_clip_rrect':
      (FlutterView view) => TwoPlatformViewClipRRect(view, firstId: _viewId++, secondId: _viewId++),
  'two_platform_view_clip_rrect_multiple_clips':
      (FlutterView view) =>
          TwoPlatformViewClipRRectMultipleClips(view, firstId: _viewId++, secondId: _viewId++),
  'two_platform_view_clip_path':
      (FlutterView view) => TwoPlatformViewClipPath(view, firstId: _viewId++, secondId: _viewId++),
  'two_platform_view_clip_path_multiple_clips':
      (FlutterView view) =>
          TwoPlatformViewClipPathMultipleClips(view, firstId: _viewId++, secondId: _viewId++),
  'tap_status_bar': (FlutterView view) => TouchesScenario(view),
  'initial_route_reply': (FlutterView view) => InitialRouteReply(view),
  'platform_view_with_continuous_texture':
      (FlutterView view) => PlatformViewWithContinuousTexture(view, id: _viewId++),
  'bogus_font_text': (FlutterView view) => BogusFontText(view),
  'spawn_engine_works': (FlutterView view) => BogusFontText(view),
  'pointer_events': (FlutterView view) => TouchesScenario(view),
  'display_texture': (FlutterView view) => DisplayTexture(view),
  'get_bitmap': (FlutterView view) => GetBitmapScenario(view),
  'app_extension': (FlutterView view) => DarwinAppExtensionScenario(view),
  'darwin_system_font': (FlutterView view) => DarwinSystemFont(view),
};

Map<String, dynamic> _currentScenarioParams = <String, dynamic>{};

Scenario? _currentScenarioInstance;

/// Loads an scenario.
/// The map must contain a `name` entry, which equals to the name of the scenario.
void loadScenario(Map<String, dynamic> scenario, FlutterView view) {
  final String scenarioName = scenario['name'] as String;
  assert(_scenarios[scenarioName] != null);
  _currentScenarioParams = scenario;

  if (_currentScenarioInstance != null) {
    _currentScenarioInstance!.unmount();
  }

  _currentScenarioInstance = _scenarios[scenario['name']]!(view);
  view.platformDispatcher.scheduleFrame();
  print('Loading scenario $scenarioName');
}

/// Gets the loaded [Scenario].
Scenario? get currentScenario {
  return _currentScenarioInstance;
}

/// Gets the parameters passed to the app over the channel.
Map<String, dynamic> get scenarioParams {
  return Map<String, dynamic>.from(_currentScenarioParams);
}
