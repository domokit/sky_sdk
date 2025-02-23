// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart' show defaultTargetPlatform;
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart' show timeDilation;
import 'package:scoped_model/scoped_model.dart';
import 'package:url_launcher/url_launcher.dart';

import '../demo/shrine/model/app_state_model.dart';
import 'demos.dart';
import 'home.dart';
import 'options.dart';
import 'scales.dart';
import 'themes.dart';
import 'updater.dart';

class GalleryApp extends StatefulWidget {
  const GalleryApp({
    super.key,
    this.updateUrlFetcher,
    this.enablePerformanceOverlay = true,
    this.enableRasterCacheImagesCheckerboard = true,
    this.enableOffscreenLayersCheckerboard = true,
    this.onSendFeedback,
    this.testMode = false,
  });

  final UpdateUrlFetcher? updateUrlFetcher;
  final bool enablePerformanceOverlay;
  final bool enableRasterCacheImagesCheckerboard;
  final bool enableOffscreenLayersCheckerboard;
  final VoidCallback? onSendFeedback;
  final bool testMode;

  @override
  State<GalleryApp> createState() => _GalleryAppState();
}

class _GalleryAppState extends State<GalleryApp> {
  GalleryOptions? _options;
  Timer? _timeDilationTimer;
  late final AppStateModel model = AppStateModel()..loadProducts();

  Map<String, WidgetBuilder> _buildRoutes() {
    // For a different example of how to set up an application routing table
    // using named routes, consider the example in the Navigator class documentation:
    // https://api.flutter.dev/flutter/widgets/Navigator-class.html
    return <String, WidgetBuilder>{
      for (final GalleryDemo demo in kAllGalleryDemos) demo.routeName: demo.buildRoute,
    };
  }

  @override
  void initState() {
    super.initState();
    _options = GalleryOptions(
      themeMode: ThemeMode.system,
      textScaleFactor: kAllGalleryTextScaleValues[0],
      visualDensity: kAllGalleryVisualDensityValues[0],
      timeDilation: timeDilation,
      platform: defaultTargetPlatform,
    );
  }

  @override
  void reassemble() {
    _options = _options!.copyWith(platform: defaultTargetPlatform);
    super.reassemble();
  }

  @override
  void dispose() {
    _timeDilationTimer?.cancel();
    _timeDilationTimer = null;
    super.dispose();
  }

  void _handleOptionsChanged(GalleryOptions newOptions) {
    setState(() {
      if (_options!.timeDilation != newOptions.timeDilation) {
        _timeDilationTimer?.cancel();
        _timeDilationTimer = null;
        if (newOptions.timeDilation > 1.0) {
          // We delay the time dilation change long enough that the user can see
          // that UI has started reacting and then we slam on the brakes so that
          // they see that the time is in fact now dilated.
          _timeDilationTimer = Timer(const Duration(milliseconds: 150), () {
            timeDilation = newOptions.timeDilation;
          });
        } else {
          timeDilation = newOptions.timeDilation;
        }
      }

      _options = newOptions;
    });
  }

  Widget _applyTextScaleFactor(Widget child) {
    return Builder(
      builder: (BuildContext context) {
        final double? textScaleFactor = _options!.textScaleFactor!.scale;
        return MediaQuery.withClampedTextScaling(
          minScaleFactor: textScaleFactor ?? 0.0,
          maxScaleFactor: textScaleFactor ?? double.infinity,
          child: child,
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    Widget home = GalleryHome(
      testMode: widget.testMode,
      optionsPage: GalleryOptionsPage(
        options: _options,
        onOptionsChanged: _handleOptionsChanged,
        onSendFeedback:
            widget.onSendFeedback ??
            () {
              launchUrl(
                Uri.parse('https://github.com/flutter/flutter/issues/new/choose'),
                mode: LaunchMode.externalApplication,
              );
            },
      ),
    );

    if (widget.updateUrlFetcher != null) {
      home = Updater(updateUrlFetcher: widget.updateUrlFetcher!, child: home);
    }

    return ScopedModel<AppStateModel>(
      model: model,
      child: MaterialApp(
        // The automatically applied scrollbars on desktop can cause a crash for
        // demos where many scrollables are all attached to the same
        // PrimaryScrollController. The gallery needs to be migrated before
        // enabling this. https://github.com/flutter/gallery/issues/523
        scrollBehavior: const MaterialScrollBehavior().copyWith(scrollbars: false),
        theme: kLightGalleryTheme.copyWith(
          platform: _options!.platform,
          visualDensity: _options!.visualDensity!.visualDensity,
        ),
        darkTheme: kDarkGalleryTheme.copyWith(
          platform: _options!.platform,
          visualDensity: _options!.visualDensity!.visualDensity,
        ),
        themeMode: _options!.themeMode,
        title: 'Flutter Gallery',
        color: Colors.grey,
        showPerformanceOverlay: _options!.showPerformanceOverlay,
        checkerboardOffscreenLayers: _options!.showOffscreenLayersCheckerboard,
        checkerboardRasterCacheImages: _options!.showRasterCacheImagesCheckerboard,
        routes: _buildRoutes(),
        builder: (BuildContext context, Widget? child) {
          return Directionality(
            textDirection: _options!.textDirection,
            child: _applyTextScaleFactor(
              // Specifically use a blank Cupertino theme here and do not transfer
              // over the Material primary color etc except the brightness to
              // showcase standard iOS looks.
              Builder(
                builder: (BuildContext context) {
                  return CupertinoTheme(
                    data: CupertinoThemeData(brightness: Theme.brightnessOf(context)),
                    child: child!,
                  );
                },
              ),
            ),
          );
        },
        home: home,
      ),
    );
  }
}
