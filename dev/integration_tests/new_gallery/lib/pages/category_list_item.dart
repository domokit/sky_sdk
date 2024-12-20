// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../constants.dart';
import '../data/demos.dart';
import '../gallery_localizations.dart';
import '../layout/adaptive.dart';
import 'demo.dart';

typedef CategoryHeaderTapCallback = void Function(bool shouldOpenList);

class CategoryListItem extends StatefulWidget {
  const CategoryListItem({
    super.key,
    this.restorationId,
    required this.category,
    required this.imageString,
    this.demos = const <GalleryDemo>[],
    this.initiallyExpanded = false,
    this.onTap,
  });

  final GalleryDemoCategory category;
  final String? restorationId;
  final String imageString;
  final List<GalleryDemo> demos;
  final bool initiallyExpanded;
  final CategoryHeaderTapCallback? onTap;

  @override
  State<CategoryListItem> createState() => _CategoryListItemState();
}

class _CategoryListItemState extends State<CategoryListItem> with SingleTickerProviderStateMixin {
  static final Animatable<double> _easeInTween = CurveTween(curve: Curves.easeIn);
  static const Duration _expandDuration = Duration(milliseconds: 200);
  late AnimationController _controller;
  late Animation<double> _childrenHeightFactor;
  late Animation<double> _headerChevronOpacity;
  late Animation<double> _headerHeight;
  late Animation<EdgeInsetsGeometry> _headerMargin;
  late Animation<EdgeInsetsGeometry> _headerImagePadding;
  late Animation<EdgeInsetsGeometry> _childrenPadding;
  late Animation<BorderRadius?> _headerBorderRadius;

  @override
  void initState() {
    super.initState();

    _controller = AnimationController(duration: _expandDuration, vsync: this);
    _controller.addStatusListener((AnimationStatus status) {
      setState(() {});
    });

    _childrenHeightFactor = _controller.drive(_easeInTween);
    _headerChevronOpacity = _controller.drive(_easeInTween);
    _headerHeight = Tween<double>(begin: 80, end: 96).animate(_controller);
    _headerMargin = EdgeInsetsGeometryTween(
      begin: const EdgeInsets.fromLTRB(32, 8, 32, 8),
      end: EdgeInsets.zero,
    ).animate(_controller);
    _headerImagePadding = EdgeInsetsGeometryTween(
      begin: const EdgeInsets.all(8),
      end: const EdgeInsetsDirectional.fromSTEB(16, 8, 8, 8),
    ).animate(_controller);
    _childrenPadding = EdgeInsetsGeometryTween(
      begin: const EdgeInsets.symmetric(horizontal: 32),
      end: EdgeInsets.zero,
    ).animate(_controller);
    _headerBorderRadius = BorderRadiusTween(
      begin: BorderRadius.circular(10),
      end: BorderRadius.zero,
    ).animate(_controller);

    if (widget.initiallyExpanded) {
      _controller.value = 1.0;
    }
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  void _handleTap() {
    if (_controller.isDismissed) {
      _controller.forward();
      if (widget.onTap != null) {
        widget.onTap!(true);
      }
    } else {
      _controller.reverse();
      if (widget.onTap != null) {
        widget.onTap!(false);
      }
    }
  }

  Widget _buildHeaderWithChildren(BuildContext context, Widget? child) {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        _CategoryHeader(
          margin: _headerMargin.value,
          imagePadding: _headerImagePadding.value,
          borderRadius: _headerBorderRadius.value!,
          height: _headerHeight.value,
          chevronOpacity: _headerChevronOpacity.value,
          imageString: widget.imageString,
          category: widget.category,
          onTap: _handleTap,
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
    return AnimatedBuilder(
      animation: _controller.view,
      builder: _buildHeaderWithChildren,
      child:
          _controller.isDismissed
              ? null
              : _ExpandedCategoryDemos(category: widget.category, demos: widget.demos),
    );
  }
}

class _CategoryHeader extends StatelessWidget {
  const _CategoryHeader({
    this.margin,
    required this.imagePadding,
    required this.borderRadius,
    this.height,
    required this.chevronOpacity,
    required this.imageString,
    required this.category,
    this.onTap,
  });

  final EdgeInsetsGeometry? margin;
  final EdgeInsetsGeometry imagePadding;
  final double? height;
  final BorderRadiusGeometry borderRadius;
  final String imageString;
  final GalleryDemoCategory category;
  final double chevronOpacity;
  final GestureTapCallback? onTap;

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    return Container(
      margin: margin,
      child: Material(
        shape: RoundedRectangleBorder(borderRadius: borderRadius),
        color: colorScheme.onBackground,
        clipBehavior: Clip.antiAlias,
        child: SizedBox(
          width: MediaQuery.of(context).size.width,
          child: InkWell(
            // Makes integration tests possible.
            key: ValueKey<String>('${category.name}CategoryHeader'),
            onTap: onTap,
            child: Row(
              children: <Widget>[
                Expanded(
                  child: Wrap(
                    crossAxisAlignment: WrapCrossAlignment.center,
                    children: <Widget>[
                      Padding(
                        padding: imagePadding,
                        child: FadeInImage(
                          image: AssetImage(imageString, package: 'flutter_gallery_assets'),
                          placeholder: MemoryImage(kTransparentImage),
                          fadeInDuration: entranceAnimationDuration,
                          width: 64,
                          height: 64,
                          excludeFromSemantics: true,
                        ),
                      ),
                      Padding(
                        padding: const EdgeInsetsDirectional.only(start: 8),
                        child: Text(
                          category.displayTitle(GalleryLocalizations.of(context)!)!,
                          style: Theme.of(
                            context,
                          ).textTheme.headlineSmall!.apply(color: colorScheme.onSurface),
                        ),
                      ),
                    ],
                  ),
                ),
                Opacity(
                  opacity: chevronOpacity,
                  child:
                      chevronOpacity != 0
                          ? Padding(
                            padding: const EdgeInsetsDirectional.only(start: 8, end: 32),
                            child: Icon(Icons.keyboard_arrow_up, color: colorScheme.onSurface),
                          )
                          : null,
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class _ExpandedCategoryDemos extends StatelessWidget {
  const _ExpandedCategoryDemos({required this.category, required this.demos});

  final GalleryDemoCategory category;
  final List<GalleryDemo> demos;

  @override
  Widget build(BuildContext context) {
    return Column(
      // Makes integration tests possible.
      key: ValueKey<String>('${category.name}DemoList'),
      children: <Widget>[
        for (final GalleryDemo demo in demos) CategoryDemoItem(demo: demo),
        const SizedBox(height: 12), // Extra space below.
      ],
    );
  }
}

class CategoryDemoItem extends StatelessWidget {
  const CategoryDemoItem({super.key, required this.demo});

  final GalleryDemo demo;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    return Material(
      // Makes integration tests possible.
      key: ValueKey<String>(demo.describe),
      color: Theme.of(context).colorScheme.surface,
      child: MergeSemantics(
        child: InkWell(
          onTap: () {
            Navigator.of(context).restorablePushNamed('${DemoPage.baseRoute}/${demo.slug}');
          },
          child: Padding(
            padding: EdgeInsetsDirectional.only(
              start: 32,
              top: 20,
              end: isDisplayDesktop(context) ? 16 : 8,
            ),
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                Icon(demo.icon, color: colorScheme.primary),
                const SizedBox(width: 40),
                Flexible(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      Text(
                        demo.title,
                        style: textTheme.titleMedium!.apply(color: colorScheme.onSurface),
                      ),
                      Text(
                        demo.subtitle,
                        style: textTheme.labelSmall!.apply(
                          color: colorScheme.onSurface.withOpacity(0.5),
                        ),
                      ),
                      const SizedBox(height: 20),
                      Divider(
                        thickness: 1,
                        height: 1,
                        color: Theme.of(context).colorScheme.background,
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
