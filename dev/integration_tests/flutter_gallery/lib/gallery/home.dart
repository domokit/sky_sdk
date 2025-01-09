// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:developer';
import 'dart:math' as math;

import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'backdrop.dart';
import 'demos.dart';

const String _kGalleryAssetsPackage = 'flutter_gallery_assets';
const Color _kFlutterBlue = Color(0xFF003D75);
const double _kDemoItemHeight = 64.0;
const Duration _kFrontLayerSwitchDuration = Duration(milliseconds: 300);

class _FlutterLogo extends StatelessWidget {
  const _FlutterLogo();

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Container(
        width: 34.0,
        height: 34.0,
        decoration: const BoxDecoration(
          image: DecorationImage(
            image: AssetImage('logos/flutter_white/logo.png', package: _kGalleryAssetsPackage),
          ),
        ),
      ),
    );
  }
}

class _CategoryItem extends StatelessWidget {
  const _CategoryItem({this.category, this.onTap});

  final GalleryDemoCategory? category;
  final VoidCallback? onTap;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final bool isDark = theme.brightness == Brightness.dark;

    // This repaint boundary prevents the entire _CategoriesPage from being
    // repainted when the button's ink splash animates.
    return RepaintBoundary(
      child: RawMaterialButton(
        hoverColor: theme.primaryColor.withOpacity(0.05),
        splashColor: theme.primaryColor.withOpacity(0.12),
        highlightColor: Colors.transparent,
        onPressed: onTap,
        child: Column(
          mainAxisAlignment: MainAxisAlignment.end,
          children: <Widget>[
            Padding(
              padding: const EdgeInsets.all(6.0),
              child: Icon(category!.icon, size: 60.0, color: isDark ? Colors.white : _kFlutterBlue),
            ),
            const SizedBox(height: 10.0),
            Container(
              height: 48.0,
              alignment: Alignment.center,
              child: Text(
                category!.name,
                textAlign: TextAlign.center,
                style: theme.textTheme.titleMedium!.copyWith(
                  fontFamily: 'GoogleSans',
                  color: isDark ? Colors.white : _kFlutterBlue,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _CategoriesPage extends StatelessWidget {
  const _CategoriesPage({this.categories, this.onCategoryTap});

  final Iterable<GalleryDemoCategory>? categories;
  final ValueChanged<GalleryDemoCategory>? onCategoryTap;

  @override
  Widget build(BuildContext context) {
    const double aspectRatio = 160.0 / 180.0;
    final List<GalleryDemoCategory> categoriesList = categories!.toList();
    final int columnCount = (MediaQuery.of(context).orientation == Orientation.portrait) ? 2 : 3;

    return Semantics(
      scopesRoute: true,
      namesRoute: true,
      label: 'categories',
      explicitChildNodes: true,
      child: SingleChildScrollView(
        key: const PageStorageKey<String>('categories'),
        child: LayoutBuilder(
          builder: (BuildContext context, BoxConstraints constraints) {
            final double columnWidth = constraints.biggest.width / columnCount.toDouble();
            final double rowHeight = math.min(225.0, columnWidth * aspectRatio);
            final int rowCount = (categories!.length + columnCount - 1) ~/ columnCount;

            // This repaint boundary prevents the inner contents of the front layer
            // from repainting when the backdrop toggle triggers a repaint on the
            // LayoutBuilder.
            return RepaintBoundary(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: List<Widget>.generate(rowCount, (int rowIndex) {
                  final int columnCountForRow =
                      rowIndex == rowCount - 1
                          ? categories!.length - columnCount * math.max<int>(0, rowCount - 1)
                          : columnCount;

                  return Row(
                    children: List<Widget>.generate(columnCountForRow, (int columnIndex) {
                      final int index = rowIndex * columnCount + columnIndex;
                      final GalleryDemoCategory category = categoriesList[index];

                      return SizedBox(
                        width: columnWidth,
                        height: rowHeight,
                        child: _CategoryItem(
                          category: category,
                          onTap: () {
                            onCategoryTap!(category);
                          },
                        ),
                      );
                    }),
                  );
                }),
              ),
            );
          },
        ),
      ),
    );
  }
}

class _DemoItem extends StatelessWidget {
  const _DemoItem({this.demo});

  final GalleryDemo? demo;

  void _launchDemo(BuildContext context) {
    if (demo != null) {
      Timeline.instantSync(
        'Start Transition',
        arguments: <String, String>{'from': '/', 'to': demo!.routeName},
      );
      Navigator.pushNamed(context, demo!.routeName);
    }
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final bool isDark = theme.brightness == Brightness.dark;
    // The fontSize to use for computing the heuristic UI scaling factor.
    const double defaultFontSize = 14.0;
    final double containerScalingFactor =
        MediaQuery.textScalerOf(context).scale(defaultFontSize) / defaultFontSize;
    return RawMaterialButton(
      splashColor: theme.primaryColor.withOpacity(0.12),
      highlightColor: Colors.transparent,
      onPressed: () {
        _launchDemo(context);
      },
      child: Container(
        constraints: BoxConstraints(minHeight: _kDemoItemHeight * containerScalingFactor),
        child: Row(
          children: <Widget>[
            Container(
              width: 56.0,
              height: 56.0,
              alignment: Alignment.center,
              child: Icon(demo!.icon, size: 24.0, color: isDark ? Colors.white : _kFlutterBlue),
            ),
            Expanded(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[
                  Text(
                    demo!.title,
                    style: theme.textTheme.titleMedium!.copyWith(
                      color: isDark ? Colors.white : const Color(0xFF202124),
                    ),
                  ),
                  if (demo!.subtitle != null)
                    Text(
                      demo!.subtitle!,
                      style: theme.textTheme.bodyMedium!.copyWith(
                        color: isDark ? Colors.white : const Color(0xFF60646B),
                      ),
                    ),
                ],
              ),
            ),
            const SizedBox(width: 44.0),
          ],
        ),
      ),
    );
  }
}

class _DemosPage extends StatelessWidget {
  const _DemosPage(this.category);

  final GalleryDemoCategory? category;

  @override
  Widget build(BuildContext context) {
    // When overriding ListView.padding, it is necessary to manually handle
    // safe areas.
    final double windowBottomPadding = MediaQuery.of(context).padding.bottom;
    return KeyedSubtree(
      key: const ValueKey<String>('GalleryDemoList'), // So the tests can find this ListView
      child: Semantics(
        scopesRoute: true,
        namesRoute: true,
        label: category!.name,
        explicitChildNodes: true,
        child: ListView(
          dragStartBehavior: DragStartBehavior.down,
          key: PageStorageKey<String>(category!.name),
          padding: EdgeInsets.only(top: 8.0, bottom: windowBottomPadding),
          children:
              kGalleryCategoryToDemos[category!]!.map<Widget>((GalleryDemo demo) {
                return _DemoItem(demo: demo);
              }).toList(),
        ),
      ),
    );
  }
}

class GalleryHome extends StatefulWidget {
  const GalleryHome({super.key, this.testMode = false, this.optionsPage});

  final Widget? optionsPage;
  final bool testMode;

  // In checked mode our MaterialApp will show the default "debug" banner.
  // Otherwise show the "preview" banner.
  static bool showPreviewBanner = true;

  @override
  State<GalleryHome> createState() => _GalleryHomeState();
}

class _GalleryHomeState extends State<GalleryHome> with SingleTickerProviderStateMixin {
  static final GlobalKey<ScaffoldState> _scaffoldKey = GlobalKey<ScaffoldState>();
  late AnimationController _controller;
  GalleryDemoCategory? _category;

  static Widget _topHomeLayout(Widget? currentChild, List<Widget> previousChildren) {
    return Stack(
      alignment: Alignment.topCenter,
      children: <Widget>[...previousChildren, if (currentChild != null) currentChild],
    );
  }

  static const AnimatedSwitcherLayoutBuilder _centerHomeLayout =
      AnimatedSwitcher.defaultLayoutBuilder;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      duration: const Duration(milliseconds: 600),
      debugLabel: 'preview banner',
      vsync: this,
    )..forward();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final bool isDark = theme.brightness == Brightness.dark;
    final MediaQueryData media = MediaQuery.of(context);
    final bool centerHome = media.orientation == Orientation.portrait && media.size.height < 800.0;

    const Curve switchOutCurve = Interval(0.4, 1.0, curve: Curves.fastOutSlowIn);
    const Curve switchInCurve = Interval(0.4, 1.0, curve: Curves.fastOutSlowIn);

    Widget home = Scaffold(
      key: _scaffoldKey,
      backgroundColor: isDark ? _kFlutterBlue : theme.primaryColor,
      body: SafeArea(
        bottom: false,
        child: PopScope<Object?>(
          canPop: _category == null,
          onPopInvokedWithResult: (bool didPop, Object? result) {
            if (didPop) {
              return;
            }
            // Pop the category page if Android back button is pressed.
            setState(() => _category = null);
          },
          child: Backdrop(
            backTitle: const Text('Options'),
            backLayer: widget.optionsPage,
            frontAction: AnimatedSwitcher(
              duration: _kFrontLayerSwitchDuration,
              switchOutCurve: switchOutCurve,
              switchInCurve: switchInCurve,
              child:
                  _category == null
                      ? const _FlutterLogo()
                      : IconButton(
                        icon: const BackButtonIcon(),
                        tooltip: 'Back',
                        onPressed: () => setState(() => _category = null),
                      ),
            ),
            frontTitle: AnimatedSwitcher(
              duration: _kFrontLayerSwitchDuration,
              child: _category == null ? const Text('Flutter gallery') : Text(_category!.name),
            ),
            frontHeading: widget.testMode ? null : Container(height: 24.0),
            frontLayer: AnimatedSwitcher(
              duration: _kFrontLayerSwitchDuration,
              switchOutCurve: switchOutCurve,
              switchInCurve: switchInCurve,
              layoutBuilder: centerHome ? _centerHomeLayout : _topHomeLayout,
              child:
                  _category != null
                      ? _DemosPage(_category)
                      : _CategoriesPage(
                        categories: kAllGalleryDemoCategories,
                        onCategoryTap: (GalleryDemoCategory category) {
                          setState(() => _category = category);
                        },
                      ),
            ),
          ),
        ),
      ),
    );

    assert(() {
      GalleryHome.showPreviewBanner = false;
      return true;
    }());

    if (GalleryHome.showPreviewBanner) {
      home = Stack(
        fit: StackFit.expand,
        children: <Widget>[
          home,
          FadeTransition(
            opacity: CurvedAnimation(parent: _controller, curve: Curves.easeInOut),
            child: const Banner(message: 'PREVIEW', location: BannerLocation.topEnd),
          ),
        ],
      );
    }
    home = AnnotatedRegion<SystemUiOverlayStyle>(value: SystemUiOverlayStyle.light, child: home);

    return home;
  }
}
