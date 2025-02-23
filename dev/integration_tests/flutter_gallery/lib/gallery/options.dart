// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'about.dart';
import 'scales.dart';

@immutable
class GalleryOptions {
  const GalleryOptions({
    this.themeMode,
    this.textScaleFactor,
    this.visualDensity,
    this.textDirection = TextDirection.ltr,
    this.timeDilation = 1.0,
    this.platform,
    this.showOffscreenLayersCheckerboard = false,
    this.showRasterCacheImagesCheckerboard = false,
    this.showPerformanceOverlay = false,
  });

  final ThemeMode? themeMode;
  final GalleryTextScaleValue? textScaleFactor;
  final GalleryVisualDensityValue? visualDensity;
  final TextDirection textDirection;
  final double timeDilation;
  final TargetPlatform? platform;
  final bool showPerformanceOverlay;
  final bool showRasterCacheImagesCheckerboard;
  final bool showOffscreenLayersCheckerboard;

  GalleryOptions copyWith({
    ThemeMode? themeMode,
    GalleryTextScaleValue? textScaleFactor,
    GalleryVisualDensityValue? visualDensity,
    TextDirection? textDirection,
    double? timeDilation,
    TargetPlatform? platform,
    bool? showPerformanceOverlay,
    bool? showRasterCacheImagesCheckerboard,
    bool? showOffscreenLayersCheckerboard,
  }) {
    return GalleryOptions(
      themeMode: themeMode ?? this.themeMode,
      textScaleFactor: textScaleFactor ?? this.textScaleFactor,
      visualDensity: visualDensity ?? this.visualDensity,
      textDirection: textDirection ?? this.textDirection,
      timeDilation: timeDilation ?? this.timeDilation,
      platform: platform ?? this.platform,
      showPerformanceOverlay: showPerformanceOverlay ?? this.showPerformanceOverlay,
      showOffscreenLayersCheckerboard:
          showOffscreenLayersCheckerboard ?? this.showOffscreenLayersCheckerboard,
      showRasterCacheImagesCheckerboard:
          showRasterCacheImagesCheckerboard ?? this.showRasterCacheImagesCheckerboard,
    );
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is GalleryOptions &&
        other.themeMode == themeMode &&
        other.textScaleFactor == textScaleFactor &&
        other.visualDensity == visualDensity &&
        other.textDirection == textDirection &&
        other.platform == platform &&
        other.showPerformanceOverlay == showPerformanceOverlay &&
        other.showRasterCacheImagesCheckerboard == showRasterCacheImagesCheckerboard &&
        other.showOffscreenLayersCheckerboard == showRasterCacheImagesCheckerboard;
  }

  @override
  int get hashCode => Object.hash(
    themeMode,
    textScaleFactor,
    visualDensity,
    textDirection,
    timeDilation,
    platform,
    showPerformanceOverlay,
    showRasterCacheImagesCheckerboard,
    showOffscreenLayersCheckerboard,
  );

  @override
  String toString() {
    return '$runtimeType($themeMode)';
  }
}

const double _kItemHeight = 48.0;
const EdgeInsetsDirectional _kItemPadding = EdgeInsetsDirectional.only(start: 56.0);

class _OptionsItem extends StatelessWidget {
  const _OptionsItem({this.child});

  final Widget? child;

  @override
  Widget build(BuildContext context) {
    final double textScaleFactor = MediaQuery.textScalerOf(context).textScaleFactor;

    return MergeSemantics(
      child: Container(
        constraints: BoxConstraints(minHeight: _kItemHeight * textScaleFactor),
        padding: _kItemPadding,
        alignment: AlignmentDirectional.centerStart,
        child: DefaultTextStyle(
          style: DefaultTextStyle.of(context).style,
          maxLines: 2,
          overflow: TextOverflow.fade,
          child: IconTheme(data: Theme.of(context).primaryIconTheme, child: child!),
        ),
      ),
    );
  }
}

class _BooleanItem extends StatelessWidget {
  const _BooleanItem(this.title, this.value, this.onChanged, {this.switchKey});

  final String title;
  final bool value;
  final ValueChanged<bool> onChanged;
  // [switchKey] is used for accessing the switch from driver tests.
  final Key? switchKey;

  @override
  Widget build(BuildContext context) {
    final bool isDark = Theme.brightnessOf(context) == Brightness.dark;
    return _OptionsItem(
      child: Row(
        children: <Widget>[
          Expanded(child: Text(title)),
          Switch(
            key: switchKey,
            value: value,
            onChanged: onChanged,
            activeColor: const Color(0xFF39CEFD),
            activeTrackColor: isDark ? Colors.white30 : Colors.black26,
          ),
        ],
      ),
    );
  }
}

class _ActionItem extends StatelessWidget {
  const _ActionItem(this.text, this.onTap);

  final String text;
  final VoidCallback? onTap;

  @override
  Widget build(BuildContext context) {
    return _OptionsItem(child: _TextButton(onPressed: onTap, child: Text(text)));
  }
}

class _TextButton extends StatelessWidget {
  const _TextButton({this.onPressed, this.child});

  final VoidCallback? onPressed;
  final Widget? child;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    return TextButton(
      style: TextButton.styleFrom(
        foregroundColor: theme.colorScheme.onPrimary,
        textStyle: theme.textTheme.titleMedium,
        padding: EdgeInsets.zero,
      ),
      onPressed: onPressed,
      child: child!,
    );
  }
}

class _Heading extends StatelessWidget {
  const _Heading(this.text);

  final String text;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    return _OptionsItem(
      child: DefaultTextStyle(
        style: theme.textTheme.titleLarge!.copyWith(
          fontFamily: 'GoogleSans',
          color: theme.colorScheme.onPrimary,
          fontWeight: FontWeight.w700,
        ),
        child: Semantics(header: true, child: Text(text)),
      ),
    );
  }
}

class _ThemeModeItem extends StatelessWidget {
  const _ThemeModeItem(this.options, this.onOptionsChanged);

  final GalleryOptions? options;
  final ValueChanged<GalleryOptions>? onOptionsChanged;

  static final Map<ThemeMode, String> modeLabels = <ThemeMode, String>{
    ThemeMode.system: 'System Default',
    ThemeMode.light: 'Light',
    ThemeMode.dark: 'Dark',
  };

  @override
  Widget build(BuildContext context) {
    return _OptionsItem(
      child: Row(
        children: <Widget>[
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                const Text('Theme'),
                Text(
                  modeLabels[options!.themeMode!]!,
                  style: Theme.of(context).primaryTextTheme.bodyMedium,
                ),
              ],
            ),
          ),
          PopupMenuButton<ThemeMode>(
            padding: const EdgeInsetsDirectional.only(end: 16.0),
            icon: const Icon(Icons.arrow_drop_down),
            initialValue: options!.themeMode,
            itemBuilder: (BuildContext context) {
              return ThemeMode.values.map<PopupMenuItem<ThemeMode>>((ThemeMode mode) {
                return PopupMenuItem<ThemeMode>(value: mode, child: Text(modeLabels[mode]!));
              }).toList();
            },
            onSelected: (ThemeMode mode) {
              onOptionsChanged!(options!.copyWith(themeMode: mode));
            },
          ),
        ],
      ),
    );
  }
}

class _TextScaleFactorItem extends StatelessWidget {
  const _TextScaleFactorItem(this.options, this.onOptionsChanged);

  final GalleryOptions? options;
  final ValueChanged<GalleryOptions>? onOptionsChanged;

  @override
  Widget build(BuildContext context) {
    return _OptionsItem(
      child: Row(
        children: <Widget>[
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                const Text('Text size'),
                Text(
                  options!.textScaleFactor!.label,
                  style: Theme.of(context).primaryTextTheme.bodyMedium,
                ),
              ],
            ),
          ),
          PopupMenuButton<GalleryTextScaleValue>(
            padding: const EdgeInsetsDirectional.only(end: 16.0),
            icon: const Icon(Icons.arrow_drop_down),
            itemBuilder: (BuildContext context) {
              return kAllGalleryTextScaleValues.map<PopupMenuItem<GalleryTextScaleValue>>((
                GalleryTextScaleValue scaleValue,
              ) {
                return PopupMenuItem<GalleryTextScaleValue>(
                  value: scaleValue,
                  child: Text(scaleValue.label),
                );
              }).toList();
            },
            onSelected: (GalleryTextScaleValue scaleValue) {
              onOptionsChanged!(options!.copyWith(textScaleFactor: scaleValue));
            },
          ),
        ],
      ),
    );
  }
}

class _VisualDensityItem extends StatelessWidget {
  const _VisualDensityItem(this.options, this.onOptionsChanged);

  final GalleryOptions? options;
  final ValueChanged<GalleryOptions>? onOptionsChanged;

  @override
  Widget build(BuildContext context) {
    return _OptionsItem(
      child: Row(
        children: <Widget>[
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                const Text('Visual density'),
                Text(
                  options!.visualDensity!.label,
                  style: Theme.of(context).primaryTextTheme.bodyMedium,
                ),
              ],
            ),
          ),
          PopupMenuButton<GalleryVisualDensityValue>(
            padding: const EdgeInsetsDirectional.only(end: 16.0),
            icon: const Icon(Icons.arrow_drop_down),
            itemBuilder: (BuildContext context) {
              return kAllGalleryVisualDensityValues.map<PopupMenuItem<GalleryVisualDensityValue>>((
                GalleryVisualDensityValue densityValue,
              ) {
                return PopupMenuItem<GalleryVisualDensityValue>(
                  value: densityValue,
                  child: Text(densityValue.label),
                );
              }).toList();
            },
            onSelected: (GalleryVisualDensityValue densityValue) {
              onOptionsChanged!(options!.copyWith(visualDensity: densityValue));
            },
          ),
        ],
      ),
    );
  }
}

class _TextDirectionItem extends StatelessWidget {
  const _TextDirectionItem(this.options, this.onOptionsChanged);

  final GalleryOptions? options;
  final ValueChanged<GalleryOptions>? onOptionsChanged;

  @override
  Widget build(BuildContext context) {
    return _BooleanItem('Force RTL', options!.textDirection == TextDirection.rtl, (bool value) {
      onOptionsChanged!(
        options!.copyWith(textDirection: value ? TextDirection.rtl : TextDirection.ltr),
      );
    }, switchKey: const Key('text_direction'));
  }
}

class _TimeDilationItem extends StatelessWidget {
  const _TimeDilationItem(this.options, this.onOptionsChanged);

  final GalleryOptions? options;
  final ValueChanged<GalleryOptions>? onOptionsChanged;

  @override
  Widget build(BuildContext context) {
    return _BooleanItem('Slow motion', options!.timeDilation != 1.0, (bool value) {
      onOptionsChanged!(options!.copyWith(timeDilation: value ? 20.0 : 1.0));
    }, switchKey: const Key('slow_motion'));
  }
}

class _PlatformItem extends StatelessWidget {
  const _PlatformItem(this.options, this.onOptionsChanged);

  final GalleryOptions? options;
  final ValueChanged<GalleryOptions>? onOptionsChanged;

  String _platformLabel(TargetPlatform platform) {
    return switch (platform) {
      TargetPlatform.android => 'Mountain View',
      TargetPlatform.fuchsia => 'Fuchsia',
      TargetPlatform.iOS => 'Cupertino',
      TargetPlatform.linux => 'Material Desktop (linux)',
      TargetPlatform.macOS => 'Material Desktop (macOS)',
      TargetPlatform.windows => 'Material Desktop (Windows)',
    };
  }

  @override
  Widget build(BuildContext context) {
    return _OptionsItem(
      child: Row(
        children: <Widget>[
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                const Text('Platform mechanics'),
                Text(
                  _platformLabel(options!.platform!),
                  style: Theme.of(context).primaryTextTheme.bodyMedium,
                ),
              ],
            ),
          ),
          PopupMenuButton<TargetPlatform>(
            padding: const EdgeInsetsDirectional.only(end: 16.0),
            icon: const Icon(Icons.arrow_drop_down),
            itemBuilder: (BuildContext context) {
              return TargetPlatform.values.map((TargetPlatform platform) {
                return PopupMenuItem<TargetPlatform>(
                  value: platform,
                  child: Text(_platformLabel(platform)),
                );
              }).toList();
            },
            onSelected: (TargetPlatform platform) {
              onOptionsChanged!(options!.copyWith(platform: platform));
            },
          ),
        ],
      ),
    );
  }
}

class GalleryOptionsPage extends StatelessWidget {
  const GalleryOptionsPage({super.key, this.options, this.onOptionsChanged, this.onSendFeedback});

  final GalleryOptions? options;
  final ValueChanged<GalleryOptions>? onOptionsChanged;
  final VoidCallback? onSendFeedback;

  List<Widget> _enabledDiagnosticItems() {
    // Boolean showFoo options with a value of null: don't display
    // the showFoo option at all.
    if (options == null) {
      return const <Widget>[];
    }

    return <Widget>[
      const Divider(),
      const _Heading('Diagnostics'),
      _BooleanItem('Highlight offscreen layers', options!.showOffscreenLayersCheckerboard, (
        bool value,
      ) {
        onOptionsChanged!(options!.copyWith(showOffscreenLayersCheckerboard: value));
      }),
      _BooleanItem('Highlight raster cache images', options!.showRasterCacheImagesCheckerboard, (
        bool value,
      ) {
        onOptionsChanged!(options!.copyWith(showRasterCacheImagesCheckerboard: value));
      }),
      _BooleanItem('Show performance overlay', options!.showPerformanceOverlay, (bool value) {
        onOptionsChanged!(options!.copyWith(showPerformanceOverlay: value));
      }),
    ];
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);

    return DefaultTextStyle(
      style: theme.primaryTextTheme.titleMedium!,
      child: ListView(
        padding: const EdgeInsets.only(bottom: 124.0),
        children: <Widget>[
          const _Heading('Display'),
          _ThemeModeItem(options, onOptionsChanged),
          _TextScaleFactorItem(options, onOptionsChanged),
          _VisualDensityItem(options, onOptionsChanged),
          _TextDirectionItem(options, onOptionsChanged),
          _TimeDilationItem(options, onOptionsChanged),
          const Divider(),
          const ExcludeSemantics(child: _Heading('Platform mechanics')),
          _PlatformItem(options, onOptionsChanged),
          ..._enabledDiagnosticItems(),
          const Divider(),
          const _Heading('Flutter gallery'),
          _ActionItem('About Flutter Gallery', () {
            showGalleryAboutDialog(context);
          }),
          _ActionItem('Send feedback', onSendFeedback),
        ],
      ),
    );
  }
}
