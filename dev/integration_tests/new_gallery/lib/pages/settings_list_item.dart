// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:collection';

import 'package:flutter/material.dart';

// Common constants between SlowMotionSetting and SettingsListItem.
final BorderRadius settingItemBorderRadius = BorderRadius.circular(10);
const EdgeInsetsDirectional settingItemHeaderMargin = EdgeInsetsDirectional.fromSTEB(32, 0, 32, 8);

class DisplayOption {
  DisplayOption(this.title, {this.subtitle});
  final String title;
  final String? subtitle;
}

class ToggleSetting extends StatelessWidget {
  const ToggleSetting({
    super.key,
    required this.text,
    required this.value,
    required this.onChanged,
  });
  final String text;
  final bool value;
  final void Function(bool) onChanged;

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    final TextTheme textTheme = Theme.of(context).textTheme;

    return Semantics(
      container: true,
      child: Container(
        margin: settingItemHeaderMargin,
        child: Material(
          shape: RoundedRectangleBorder(borderRadius: settingItemBorderRadius),
          color: colorScheme.secondary,
          clipBehavior: Clip.antiAlias,
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: <Widget>[
              Expanded(
                child: Padding(
                  padding: const EdgeInsets.all(16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[
                      SelectableText(
                        text,
                        style: textTheme.titleMedium!.apply(color: colorScheme.onSurface),
                      ),
                    ],
                  ),
                ),
              ),
              Padding(
                padding: const EdgeInsetsDirectional.only(end: 8),
                child: Switch(activeColor: colorScheme.primary, value: value, onChanged: onChanged),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class SettingsListItem<T> extends StatefulWidget {
  const SettingsListItem({
    super.key,
    required this.optionsMap,
    required this.title,
    required this.selectedOption,
    required this.onOptionChanged,
    required this.onTapSetting,
    required this.isExpanded,
  });

  final LinkedHashMap<T, DisplayOption> optionsMap;
  final String title;
  final T selectedOption;
  final ValueChanged<T> onOptionChanged;
  final void Function() onTapSetting;
  final bool isExpanded;

  @override
  State<SettingsListItem<T?>> createState() => _SettingsListItemState<T?>();
}

class _SettingsListItemState<T> extends State<SettingsListItem<T?>>
    with SingleTickerProviderStateMixin {
  static final Animatable<double> _easeInTween = CurveTween(curve: Curves.easeIn);
  static const Duration _expandDuration = Duration(milliseconds: 150);
  late AnimationController _controller;
  late Animation<double> _childrenHeightFactor;
  late Animation<double> _headerChevronRotation;
  late Animation<double> _headerSubtitleHeight;
  late Animation<EdgeInsetsGeometry> _headerMargin;
  late Animation<EdgeInsetsGeometry> _headerPadding;
  late Animation<EdgeInsetsGeometry> _childrenPadding;
  late Animation<BorderRadius?> _headerBorderRadius;

  // For ease of use. Correspond to the keys and values of `widget.optionsMap`.
  late Iterable<T?> _options;
  late Iterable<DisplayOption> _displayOptions;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(duration: _expandDuration, vsync: this);
    _childrenHeightFactor = _controller.drive(_easeInTween);
    _headerChevronRotation = Tween<double>(begin: 0, end: 0.5).animate(_controller);
    _headerMargin = EdgeInsetsGeometryTween(
      begin: settingItemHeaderMargin,
      end: EdgeInsets.zero,
    ).animate(_controller);
    _headerPadding = EdgeInsetsGeometryTween(
      begin: const EdgeInsetsDirectional.fromSTEB(16, 10, 0, 10),
      end: const EdgeInsetsDirectional.fromSTEB(32, 18, 32, 20),
    ).animate(_controller);
    _headerSubtitleHeight = _controller.drive(Tween<double>(begin: 1.0, end: 0.0));
    _childrenPadding = EdgeInsetsGeometryTween(
      begin: const EdgeInsets.symmetric(horizontal: 32),
      end: EdgeInsets.zero,
    ).animate(_controller);
    _headerBorderRadius = BorderRadiusTween(
      begin: settingItemBorderRadius,
      end: BorderRadius.zero,
    ).animate(_controller);

    if (widget.isExpanded) {
      _controller.value = 1.0;
    }

    _options = widget.optionsMap.keys;
    _displayOptions = widget.optionsMap.values;
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  void _handleExpansion() {
    if (widget.isExpanded) {
      _controller.forward();
    } else {
      _controller.reverse().then<void>((void value) {
        if (!mounted) {
          return;
        }
      });
    }
  }

  Widget _buildHeaderWithChildren(BuildContext context, Widget? child) {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        _CategoryHeader(
          margin: _headerMargin.value,
          padding: _headerPadding.value,
          borderRadius: _headerBorderRadius.value!,
          subtitleHeight: _headerSubtitleHeight,
          chevronRotation: _headerChevronRotation,
          title: widget.title,
          subtitle: widget.optionsMap[widget.selectedOption]?.title ?? '',
          onTap: () => widget.onTapSetting(),
        ),
        Padding(
          padding: _childrenPadding.value,
          child: ClipRect(child: Align(heightFactor: _childrenHeightFactor.value, child: child)),
        ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    _handleExpansion();
    final ThemeData theme = Theme.of(context);

    return AnimatedBuilder(
      animation: _controller.view,
      builder: _buildHeaderWithChildren,
      child: Container(
        constraints: const BoxConstraints(maxHeight: 384),
        margin: const EdgeInsetsDirectional.only(start: 24, bottom: 40),
        decoration: BoxDecoration(
          border: BorderDirectional(
            start: BorderSide(width: 2, color: theme.colorScheme.background),
          ),
        ),
        child: ListView.builder(
          shrinkWrap: true,
          itemCount: widget.isExpanded ? _options.length : 0,
          itemBuilder: (BuildContext context, int index) {
            final DisplayOption displayOption = _displayOptions.elementAt(index);
            return RadioListTile<T?>(
              value: _options.elementAt(index),
              title: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  Text(
                    displayOption.title,
                    style: theme.textTheme.bodyLarge!.copyWith(
                      color: Theme.of(context).colorScheme.onPrimary,
                    ),
                  ),
                  if (displayOption.subtitle != null)
                    Text(
                      displayOption.subtitle!,
                      style: theme.textTheme.bodyLarge!.copyWith(
                        fontSize: 12,
                        color: Theme.of(context).colorScheme.onPrimary.withOpacity(0.8),
                      ),
                    ),
                ],
              ),
              groupValue: widget.selectedOption,
              onChanged: (T? newOption) => widget.onOptionChanged(newOption),
              activeColor: Theme.of(context).colorScheme.primary,
              dense: true,
            );
          },
        ),
      ),
    );
  }
}

class _CategoryHeader extends StatelessWidget {
  const _CategoryHeader({
    this.margin,
    required this.padding,
    required this.borderRadius,
    required this.subtitleHeight,
    required this.chevronRotation,
    required this.title,
    required this.subtitle,
    this.onTap,
  });

  final EdgeInsetsGeometry? margin;
  final EdgeInsetsGeometry padding;
  final BorderRadiusGeometry borderRadius;
  final String title;
  final String subtitle;
  final Animation<double> subtitleHeight;
  final Animation<double> chevronRotation;
  final GestureTapCallback? onTap;

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    final TextTheme textTheme = Theme.of(context).textTheme;
    return Container(
      margin: margin,
      child: Material(
        shape: RoundedRectangleBorder(borderRadius: borderRadius),
        color: colorScheme.secondary,
        clipBehavior: Clip.antiAlias,
        child: InkWell(
          onTap: onTap,
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: <Widget>[
              Expanded(
                child: Padding(
                  padding: padding,
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[
                      Text(
                        title,
                        style: textTheme.titleMedium!.apply(color: colorScheme.onSurface),
                      ),
                      SizeTransition(
                        sizeFactor: subtitleHeight,
                        child: Text(
                          subtitle,
                          maxLines: 1,
                          overflow: TextOverflow.ellipsis,
                          style: textTheme.labelSmall!.apply(color: colorScheme.primary),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
              Padding(
                padding: const EdgeInsetsDirectional.only(start: 8, end: 24),
                child: RotationTransition(
                  turns: chevronRotation,
                  child: const Icon(Icons.arrow_drop_down),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
