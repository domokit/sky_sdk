// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart' show timeDilation;
import 'package:flutter/services.dart' show SystemUiOverlayStyle;
import '../constants.dart';

enum CustomTextDirection { localeBased, ltr, rtl }

// See http://en.wikipedia.org/wiki/Right-to-left
const List<String> rtlLanguages = <String>[
  'ar', // Arabic
  'fa', // Farsi
  'he', // Hebrew
  'ps', // Pashto
  'ur', // Urdu
];

// Fake locale to represent the system Locale option.
const Locale systemLocaleOption = Locale('system');

Locale? _deviceLocale;

Locale? get deviceLocale => _deviceLocale;

set deviceLocale(Locale? locale) {
  _deviceLocale ??= locale;
}

@immutable
class GalleryOptions {
  const GalleryOptions({
    required this.themeMode,
    required double? textScaleFactor,
    required this.customTextDirection,
    required Locale? locale,
    required this.timeDilation,
    required this.platform,
    required this.isTestMode,
  }) : _textScaleFactor = textScaleFactor ?? 1.0,
       _locale = locale;

  final ThemeMode themeMode;
  final double _textScaleFactor;
  final CustomTextDirection customTextDirection;
  final Locale? _locale;
  final double timeDilation;
  final TargetPlatform? platform;
  final bool isTestMode; // True for integration tests.

  // We use a sentinel value to indicate the system text scale option. By
  // default, return the actual text scale factor, otherwise return the
  // sentinel value.
  double textScaleFactor(BuildContext context, {bool useSentinel = false}) {
    if (_textScaleFactor == systemTextScaleFactorOption) {
      return useSentinel
          ? systemTextScaleFactorOption
          // ignore: deprecated_member_use
          : MediaQuery.of(context).textScaleFactor;
    } else {
      return _textScaleFactor;
    }
  }

  Locale? get locale => _locale ?? deviceLocale;

  /// Returns a text direction based on the [CustomTextDirection] setting.
  /// If it is based on locale and the locale cannot be determined, returns
  /// null.
  TextDirection? resolvedTextDirection() {
    switch (customTextDirection) {
      case CustomTextDirection.localeBased:
        final String? language = locale?.languageCode.toLowerCase();
        if (language == null) {
          return null;
        }
        return rtlLanguages.contains(language) ? TextDirection.rtl : TextDirection.ltr;
      case CustomTextDirection.rtl:
        return TextDirection.rtl;
      case CustomTextDirection.ltr:
        return TextDirection.ltr;
    }
  }

  /// Returns a [SystemUiOverlayStyle] based on the [ThemeMode] setting.
  /// In other words, if the theme is dark, returns light; if the theme is
  /// light, returns dark.
  SystemUiOverlayStyle resolvedSystemUiOverlayStyle() {
    final Brightness brightness = switch (themeMode) {
      ThemeMode.light => Brightness.light,
      ThemeMode.dark => Brightness.dark,
      ThemeMode.system => WidgetsBinding.instance.platformDispatcher.platformBrightness,
    };
    return switch (brightness) {
      Brightness.light => SystemUiOverlayStyle.dark,
      Brightness.dark => SystemUiOverlayStyle.light,
    };
  }

  GalleryOptions copyWith({
    ThemeMode? themeMode,
    double? textScaleFactor,
    CustomTextDirection? customTextDirection,
    Locale? locale,
    double? timeDilation,
    TargetPlatform? platform,
    bool? isTestMode,
  }) {
    return GalleryOptions(
      themeMode: themeMode ?? this.themeMode,
      textScaleFactor: textScaleFactor ?? _textScaleFactor,
      customTextDirection: customTextDirection ?? this.customTextDirection,
      locale: locale ?? this.locale,
      timeDilation: timeDilation ?? this.timeDilation,
      platform: platform ?? this.platform,
      isTestMode: isTestMode ?? this.isTestMode,
    );
  }

  @override
  bool operator ==(Object other) =>
      other is GalleryOptions &&
      themeMode == other.themeMode &&
      _textScaleFactor == other._textScaleFactor &&
      customTextDirection == other.customTextDirection &&
      locale == other.locale &&
      timeDilation == other.timeDilation &&
      platform == other.platform &&
      isTestMode == other.isTestMode;

  @override
  int get hashCode => Object.hash(
    themeMode,
    _textScaleFactor,
    customTextDirection,
    locale,
    timeDilation,
    platform,
    isTestMode,
  );

  static GalleryOptions of(BuildContext context) {
    final _ModelBindingScope scope =
        context.dependOnInheritedWidgetOfExactType<_ModelBindingScope>()!;
    return scope.modelBindingState.currentModel;
  }

  static void update(BuildContext context, GalleryOptions newModel) {
    final _ModelBindingScope scope =
        context.dependOnInheritedWidgetOfExactType<_ModelBindingScope>()!;
    scope.modelBindingState.updateModel(newModel);
  }
}

// Applies text GalleryOptions to a widget
class ApplyTextOptions extends StatelessWidget {
  const ApplyTextOptions({super.key, required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) {
    final GalleryOptions options = GalleryOptions.of(context);
    final TextDirection? textDirection = options.resolvedTextDirection();
    final double textScaleFactor = options.textScaleFactor(context);

    final Widget widget = MediaQuery(
      data: MediaQuery.of(context).copyWith(
        // ignore: deprecated_member_use
        textScaleFactor: textScaleFactor,
      ),
      child: child,
    );
    return textDirection == null
        ? widget
        : Directionality(textDirection: textDirection, child: widget);
  }
}

// Everything below is boilerplate except code relating to time dilation.
// See https://medium.com/flutter/managing-flutter-application-state-with-inheritedwidgets-1140452befe1

class _ModelBindingScope extends InheritedWidget {
  const _ModelBindingScope({required this.modelBindingState, required super.child});

  final _ModelBindingState modelBindingState;

  @override
  bool updateShouldNotify(_ModelBindingScope oldWidget) => true;
}

class ModelBinding extends StatefulWidget {
  const ModelBinding({super.key, required this.initialModel, required this.child});

  final GalleryOptions initialModel;
  final Widget child;

  @override
  State<ModelBinding> createState() => _ModelBindingState();
}

class _ModelBindingState extends State<ModelBinding> {
  late GalleryOptions currentModel;
  Timer? _timeDilationTimer;

  @override
  void initState() {
    super.initState();
    currentModel = widget.initialModel;
  }

  @override
  void dispose() {
    _timeDilationTimer?.cancel();
    _timeDilationTimer = null;
    super.dispose();
  }

  void handleTimeDilation(GalleryOptions newModel) {
    if (currentModel.timeDilation != newModel.timeDilation) {
      _timeDilationTimer?.cancel();
      _timeDilationTimer = null;
      if (newModel.timeDilation > 1) {
        // We delay the time dilation change long enough that the user can see
        // that UI has started reacting and then we slam on the brakes so that
        // they see that the time is in fact now dilated.
        _timeDilationTimer = Timer(const Duration(milliseconds: 150), () {
          timeDilation = newModel.timeDilation;
        });
      } else {
        timeDilation = newModel.timeDilation;
      }
    }
  }

  void updateModel(GalleryOptions newModel) {
    if (newModel != currentModel) {
      handleTimeDilation(newModel);
      setState(() {
        currentModel = newModel;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return _ModelBindingScope(modelBindingState: this, child: widget.child);
  }
}
