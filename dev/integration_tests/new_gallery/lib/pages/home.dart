// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:url_launcher/url_launcher.dart';

import '../constants.dart';
import '../data/demos.dart';
import '../data/gallery_options.dart';
import '../gallery_localizations.dart';
import '../layout/adaptive.dart';
import '../studies/crane/colors.dart';
import '../studies/crane/routes.dart' as crane_routes;
import '../studies/fortnightly/routes.dart' as fortnightly_routes;
import '../studies/rally/colors.dart';
import '../studies/rally/routes.dart' as rally_routes;
import '../studies/reply/routes.dart' as reply_routes;
import '../studies/shrine/colors.dart';
import '../studies/shrine/routes.dart' as shrine_routes;
import '../studies/starter/routes.dart' as starter_app_routes;
import 'category_list_item.dart';
import 'settings.dart';
import 'splash.dart';

const double _horizontalPadding = 32.0;
const double _horizontalDesktopPadding = 81.0;
const double _carouselHeightMin = 240.0;
const double _carouselItemDesktopMargin = 8.0;
const double _carouselItemMobileMargin = 4.0;
const double _carouselItemWidth = 296.0;

class ToggleSplashNotification extends Notification {}

class HomePage extends StatelessWidget {
  const HomePage({super.key});

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    final Map<String, GalleryDemo> studyDemos = Demos.studies(localizations);
    final List<Widget> carouselCards = <Widget>[
      _CarouselCard(
        demo: studyDemos['reply'],
        asset: const AssetImage('assets/studies/reply_card.png', package: 'flutter_gallery_assets'),
        assetColor: const Color(0xFF344955),
        assetDark: const AssetImage(
          'assets/studies/reply_card_dark.png',
          package: 'flutter_gallery_assets',
        ),
        assetDarkColor: const Color(0xFF1D2327),
        textColor: Colors.white,
        studyRoute: reply_routes.homeRoute,
      ),
      _CarouselCard(
        demo: studyDemos['shrine'],
        asset: const AssetImage(
          'assets/studies/shrine_card.png',
          package: 'flutter_gallery_assets',
        ),
        assetColor: const Color(0xFFFEDBD0),
        assetDark: const AssetImage(
          'assets/studies/shrine_card_dark.png',
          package: 'flutter_gallery_assets',
        ),
        assetDarkColor: const Color(0xFF543B3C),
        textColor: shrineBrown900,
        studyRoute: shrine_routes.loginRoute,
      ),
      _CarouselCard(
        demo: studyDemos['rally'],
        textColor: RallyColors.accountColors[0],
        asset: const AssetImage('assets/studies/rally_card.png', package: 'flutter_gallery_assets'),
        assetColor: const Color(0xFFD1F2E6),
        assetDark: const AssetImage(
          'assets/studies/rally_card_dark.png',
          package: 'flutter_gallery_assets',
        ),
        assetDarkColor: const Color(0xFF253538),
        studyRoute: rally_routes.loginRoute,
      ),
      _CarouselCard(
        demo: studyDemos['crane'],
        asset: const AssetImage('assets/studies/crane_card.png', package: 'flutter_gallery_assets'),
        assetColor: const Color(0xFFFBF6F8),
        assetDark: const AssetImage(
          'assets/studies/crane_card_dark.png',
          package: 'flutter_gallery_assets',
        ),
        assetDarkColor: const Color(0xFF591946),
        textColor: cranePurple700,
        studyRoute: crane_routes.defaultRoute,
      ),
      _CarouselCard(
        demo: studyDemos['fortnightly'],
        asset: const AssetImage(
          'assets/studies/fortnightly_card.png',
          package: 'flutter_gallery_assets',
        ),
        assetColor: Colors.white,
        assetDark: const AssetImage(
          'assets/studies/fortnightly_card_dark.png',
          package: 'flutter_gallery_assets',
        ),
        assetDarkColor: const Color(0xFF1F1F1F),
        studyRoute: fortnightly_routes.defaultRoute,
      ),
      _CarouselCard(
        demo: studyDemos['starterApp'],
        asset: const AssetImage(
          'assets/studies/starter_card.png',
          package: 'flutter_gallery_assets',
        ),
        assetColor: const Color(0xFFFAF6FE),
        assetDark: const AssetImage(
          'assets/studies/starter_card_dark.png',
          package: 'flutter_gallery_assets',
        ),
        assetDarkColor: const Color(0xFF3F3D45),
        textColor: Colors.black,
        studyRoute: starter_app_routes.defaultRoute,
      ),
    ];

    if (isDesktop) {
      // Desktop layout
      final List<_DesktopCategoryItem> desktopCategoryItems = <_DesktopCategoryItem>[
        _DesktopCategoryItem(
          category: GalleryDemoCategory.material,
          asset: const AssetImage(
            'assets/icons/material/material.png',
            package: 'flutter_gallery_assets',
          ),
          demos: Demos.materialDemos(localizations),
        ),
        _DesktopCategoryItem(
          category: GalleryDemoCategory.cupertino,
          asset: const AssetImage(
            'assets/icons/cupertino/cupertino.png',
            package: 'flutter_gallery_assets',
          ),
          demos: Demos.cupertinoDemos(localizations),
        ),
        _DesktopCategoryItem(
          category: GalleryDemoCategory.other,
          asset: const AssetImage(
            'assets/icons/reference/reference.png',
            package: 'flutter_gallery_assets',
          ),
          demos: Demos.otherDemos(localizations),
        ),
      ];

      return Scaffold(
        body: ListView(
          // Makes integration tests possible.
          key: const ValueKey<String>('HomeListView'),
          primary: true,
          padding: const EdgeInsetsDirectional.only(top: firstHeaderDesktopTopPadding),
          children: <Widget>[
            _DesktopHomeItem(child: _GalleryHeader()),
            _DesktopCarousel(height: _carouselHeight(0.7, context), children: carouselCards),
            _DesktopHomeItem(child: _CategoriesHeader()),
            SizedBox(
              height: 585,
              child: _DesktopHomeItem(
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: spaceBetween(28, desktopCategoryItems),
                ),
              ),
            ),
            const SizedBox(height: 81),
            _DesktopHomeItem(
              child: Row(
                children: <Widget>[
                  MouseRegion(
                    cursor: SystemMouseCursors.click,
                    child: GestureDetector(
                      onTap: () async {
                        final Uri url = Uri.parse('https://flutter.dev');
                        if (await canLaunchUrl(url)) {
                          await launchUrl(url);
                        }
                      },
                      excludeFromSemantics: true,
                      child: FadeInImage(
                        image:
                            Theme.of(context).colorScheme.brightness == Brightness.dark
                                ? const AssetImage(
                                  'assets/logo/flutter_logo.png',
                                  package: 'flutter_gallery_assets',
                                )
                                : const AssetImage(
                                  'assets/logo/flutter_logo_color.png',
                                  package: 'flutter_gallery_assets',
                                ),
                        placeholder: MemoryImage(kTransparentImage),
                        fadeInDuration: entranceAnimationDuration,
                      ),
                    ),
                  ),
                  const Expanded(
                    child: Wrap(
                      crossAxisAlignment: WrapCrossAlignment.center,
                      alignment: WrapAlignment.end,
                      children: <Widget>[
                        SettingsAbout(),
                        SettingsFeedback(),
                        SettingsAttribution(),
                      ],
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 109),
          ],
        ),
      );
    } else {
      // Mobile layout
      return Scaffold(
        body: _AnimatedHomePage(
          restorationId: 'animated_page',
          isSplashPageAnimationFinished: SplashPageAnimation.of(context)!.isFinished,
          carouselCards: carouselCards,
        ),
      );
    }
  }

  List<Widget> spaceBetween(double paddingBetween, List<Widget> children) {
    return <Widget>[
      for (int index = 0; index < children.length; index++) ...<Widget>[
        Flexible(child: children[index]),
        if (index < children.length - 1) SizedBox(width: paddingBetween),
      ],
    ];
  }
}

class _GalleryHeader extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Header(
      color: Theme.of(context).colorScheme.primaryContainer,
      text: GalleryLocalizations.of(context)!.homeHeaderGallery,
    );
  }
}

class _CategoriesHeader extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Header(
      color: Theme.of(context).colorScheme.primary,
      text: GalleryLocalizations.of(context)!.homeHeaderCategories,
    );
  }
}

class Header extends StatelessWidget {
  const Header({super.key, required this.color, required this.text});

  final Color color;
  final String text;

  @override
  Widget build(BuildContext context) {
    return Align(
      alignment: AlignmentDirectional.centerStart,
      child: Padding(
        padding: EdgeInsets.only(
          top: isDisplayDesktop(context) ? 63 : 15,
          bottom: isDisplayDesktop(context) ? 21 : 11,
        ),
        child: SelectableText(
          text,
          style: Theme.of(context).textTheme.headlineMedium!.apply(
            color: color,
            fontSizeDelta: isDisplayDesktop(context) ? desktopDisplay1FontDelta : 0,
          ),
        ),
      ),
    );
  }
}

class _AnimatedHomePage extends StatefulWidget {
  const _AnimatedHomePage({
    required this.restorationId,
    required this.carouselCards,
    required this.isSplashPageAnimationFinished,
  });

  final String restorationId;
  final List<Widget> carouselCards;
  final bool isSplashPageAnimationFinished;

  @override
  _AnimatedHomePageState createState() => _AnimatedHomePageState();
}

class _AnimatedHomePageState extends State<_AnimatedHomePage>
    with RestorationMixin, SingleTickerProviderStateMixin {
  late AnimationController _animationController;
  Timer? _launchTimer;
  final RestorableBool _isMaterialListExpanded = RestorableBool(false);
  final RestorableBool _isCupertinoListExpanded = RestorableBool(false);
  final RestorableBool _isOtherListExpanded = RestorableBool(false);

  @override
  String get restorationId => widget.restorationId;

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_isMaterialListExpanded, 'material_list');
    registerForRestoration(_isCupertinoListExpanded, 'cupertino_list');
    registerForRestoration(_isOtherListExpanded, 'other_list');
  }

  @override
  void initState() {
    super.initState();
    _animationController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 800),
    );

    if (widget.isSplashPageAnimationFinished) {
      // To avoid the animation from running when changing the window size from
      // desktop to mobile, we do not animate our widget if the
      // splash page animation is finished on initState.
      _animationController.value = 1.0;
    } else {
      // Start our animation halfway through the splash page animation.
      _launchTimer = Timer(halfSplashPageAnimationDuration, () {
        _animationController.forward();
      });
    }
  }

  @override
  void dispose() {
    _animationController.dispose();
    _launchTimer?.cancel();
    _launchTimer = null;
    _isMaterialListExpanded.dispose();
    _isCupertinoListExpanded.dispose();
    _isOtherListExpanded.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    final bool isTestMode = GalleryOptions.of(context).isTestMode;
    return Stack(
      children: <Widget>[
        ListView(
          // Makes integration tests possible.
          key: const ValueKey<String>('HomeListView'),
          primary: true,
          restorationId: 'home_list_view',
          children: <Widget>[
            const SizedBox(height: 8),
            Container(
              margin: const EdgeInsets.symmetric(horizontal: _horizontalPadding),
              child: _GalleryHeader(),
            ),
            _MobileCarousel(
              animationController: _animationController,
              restorationId: 'home_carousel',
              children: widget.carouselCards,
            ),
            Container(
              margin: const EdgeInsets.symmetric(horizontal: _horizontalPadding),
              child: _CategoriesHeader(),
            ),
            _AnimatedCategoryItem(
              startDelayFraction: 0.00,
              controller: _animationController,
              child: CategoryListItem(
                key: const PageStorageKey<GalleryDemoCategory>(GalleryDemoCategory.material),
                restorationId: 'home_material_category_list',
                category: GalleryDemoCategory.material,
                imageString: 'assets/icons/material/material.png',
                demos: Demos.materialDemos(localizations),
                initiallyExpanded: _isMaterialListExpanded.value || isTestMode,
                onTap: (bool shouldOpenList) {
                  _isMaterialListExpanded.value = shouldOpenList;
                },
              ),
            ),
            _AnimatedCategoryItem(
              startDelayFraction: 0.05,
              controller: _animationController,
              child: CategoryListItem(
                key: const PageStorageKey<GalleryDemoCategory>(GalleryDemoCategory.cupertino),
                restorationId: 'home_cupertino_category_list',
                category: GalleryDemoCategory.cupertino,
                imageString: 'assets/icons/cupertino/cupertino.png',
                demos: Demos.cupertinoDemos(localizations),
                initiallyExpanded: _isCupertinoListExpanded.value || isTestMode,
                onTap: (bool shouldOpenList) {
                  _isCupertinoListExpanded.value = shouldOpenList;
                },
              ),
            ),
            _AnimatedCategoryItem(
              startDelayFraction: 0.10,
              controller: _animationController,
              child: CategoryListItem(
                key: const PageStorageKey<GalleryDemoCategory>(GalleryDemoCategory.other),
                restorationId: 'home_other_category_list',
                category: GalleryDemoCategory.other,
                imageString: 'assets/icons/reference/reference.png',
                demos: Demos.otherDemos(localizations),
                initiallyExpanded: _isOtherListExpanded.value || isTestMode,
                onTap: (bool shouldOpenList) {
                  _isOtherListExpanded.value = shouldOpenList;
                },
              ),
            ),
          ],
        ),
        Align(
          alignment: Alignment.topCenter,
          child: GestureDetector(
            onVerticalDragEnd: (DragEndDetails details) {
              if (details.velocity.pixelsPerSecond.dy > 200) {
                ToggleSplashNotification().dispatch(context);
              }
            },
            child: SafeArea(
              child: Container(
                height: 40,
                // If we don't set the color, gestures are not detected.
                color: Colors.transparent,
              ),
            ),
          ),
        ),
      ],
    );
  }
}

class _DesktopHomeItem extends StatelessWidget {
  const _DesktopHomeItem({required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) {
    return Align(
      child: Container(
        constraints: const BoxConstraints(maxWidth: maxHomeItemWidth),
        padding: const EdgeInsets.symmetric(horizontal: _horizontalDesktopPadding),
        child: child,
      ),
    );
  }
}

class _DesktopCategoryItem extends StatelessWidget {
  const _DesktopCategoryItem({required this.category, required this.asset, required this.demos});

  final GalleryDemoCategory category;
  final ImageProvider asset;
  final List<GalleryDemo> demos;

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    return Material(
      borderRadius: BorderRadius.circular(10),
      clipBehavior: Clip.antiAlias,
      color: colorScheme.surface,
      child: Semantics(
        container: true,
        child: FocusTraversalGroup(
          policy: WidgetOrderTraversalPolicy(),
          child: Column(
            children: <Widget>[
              _DesktopCategoryHeader(category: category, asset: asset),
              Divider(height: 2, thickness: 2, color: colorScheme.background),
              Flexible(
                child: ListView.builder(
                  // Makes integration tests possible.
                  key: ValueKey<String>('${category.name}DemoList'),
                  primary: false,
                  itemBuilder:
                      (BuildContext context, int index) => CategoryDemoItem(demo: demos[index]),
                  itemCount: demos.length,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class _DesktopCategoryHeader extends StatelessWidget {
  const _DesktopCategoryHeader({required this.category, required this.asset});

  final GalleryDemoCategory category;
  final ImageProvider asset;

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    return Material(
      // Makes integration tests possible.
      key: ValueKey<String>('${category.name}CategoryHeader'),
      color: colorScheme.onBackground,
      child: Row(
        children: <Widget>[
          Padding(
            padding: const EdgeInsets.all(10),
            child: FadeInImage(
              image: asset,
              placeholder: MemoryImage(kTransparentImage),
              fadeInDuration: entranceAnimationDuration,
              width: 64,
              height: 64,
              excludeFromSemantics: true,
            ),
          ),
          Flexible(
            child: Padding(
              padding: const EdgeInsetsDirectional.only(start: 8),
              child: Semantics(
                header: true,
                child: SelectableText(
                  category.displayTitle(GalleryLocalizations.of(context)!)!,
                  style: Theme.of(
                    context,
                  ).textTheme.headlineSmall!.apply(color: colorScheme.onSurface),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

/// Animates the category item to stagger in. The [_AnimatedCategoryItem.startDelayFraction]
/// gives a delay in the unit of a fraction of the whole animation duration,
/// which is defined in [_AnimatedHomePageState].
class _AnimatedCategoryItem extends StatelessWidget {
  _AnimatedCategoryItem({
    required double startDelayFraction,
    required this.controller,
    required this.child,
  }) : topPaddingAnimation = Tween<double>(begin: 60.0, end: 0.0).animate(
         CurvedAnimation(
           parent: controller,
           curve: Interval(
             0.000 + startDelayFraction,
             0.400 + startDelayFraction,
             curve: Curves.ease,
           ),
         ),
       );

  final Widget child;
  final AnimationController controller;
  final Animation<double> topPaddingAnimation;

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: controller,
      builder: (BuildContext context, Widget? child) {
        return Padding(padding: EdgeInsets.only(top: topPaddingAnimation.value), child: child);
      },
      child: child,
    );
  }
}

/// Animates the carousel to come in from the right.
class _AnimatedCarousel extends StatelessWidget {
  _AnimatedCarousel({required this.child, required this.controller})
    : startPositionAnimation = Tween<double>(begin: 1.0, end: 0.0).animate(
        CurvedAnimation(
          parent: controller,
          curve: const Interval(0.200, 0.800, curve: Curves.ease),
        ),
      );

  final Widget child;
  final AnimationController controller;
  final Animation<double> startPositionAnimation;

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        return Stack(
          children: <Widget>[
            SizedBox(height: _carouselHeight(.4, context)),
            AnimatedBuilder(
              animation: controller,
              builder: (BuildContext context, Widget? child) {
                return PositionedDirectional(
                  start: constraints.maxWidth * startPositionAnimation.value,
                  child: child!,
                );
              },
              child: SizedBox(
                height: _carouselHeight(.4, context),
                width: constraints.maxWidth,
                child: child,
              ),
            ),
          ],
        );
      },
    );
  }
}

/// Animates a carousel card to come in from the right.
class _AnimatedCarouselCard extends StatelessWidget {
  _AnimatedCarouselCard({required this.child, required this.controller})
    : startPaddingAnimation = Tween<double>(begin: _horizontalPadding, end: 0.0).animate(
        CurvedAnimation(
          parent: controller,
          curve: const Interval(0.900, 1.000, curve: Curves.ease),
        ),
      );

  final Widget child;
  final AnimationController controller;
  final Animation<double> startPaddingAnimation;

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: controller,
      builder: (BuildContext context, Widget? child) {
        return Padding(
          padding: EdgeInsetsDirectional.only(start: startPaddingAnimation.value),
          child: child,
        );
      },
      child: child,
    );
  }
}

class _MobileCarousel extends StatefulWidget {
  const _MobileCarousel({
    required this.animationController,
    this.restorationId,
    required this.children,
  });

  final AnimationController animationController;
  final String? restorationId;
  final List<Widget> children;

  @override
  _MobileCarouselState createState() => _MobileCarouselState();
}

class _MobileCarouselState extends State<_MobileCarousel>
    with RestorationMixin, SingleTickerProviderStateMixin {
  late PageController _controller;

  final RestorableInt _currentPage = RestorableInt(0);

  @override
  String? get restorationId => widget.restorationId;

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_currentPage, 'carousel_page');
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    // The viewPortFraction is calculated as the width of the device minus the
    // padding.
    final double width = MediaQuery.of(context).size.width;
    const double padding = _carouselItemMobileMargin * 2;
    _controller = PageController(
      initialPage: _currentPage.value,
      viewportFraction: (_carouselItemWidth + padding) / width,
    );
  }

  @override
  void dispose() {
    _controller.dispose();
    _currentPage.dispose();
    super.dispose();
  }

  Widget builder(int index) {
    final AnimatedBuilder carouselCard = AnimatedBuilder(
      animation: _controller,
      builder: (BuildContext context, Widget? child) {
        double value;
        if (_controller.position.haveDimensions) {
          value = _controller.page! - index;
        } else {
          // If haveDimensions is false, use _currentPage to calculate value.
          value = (_currentPage.value - index).toDouble();
        }
        // .3 is an approximation of the curve used in the design.
        value = (1 - (value.abs() * .3)).clamp(0, 1).toDouble();
        value = Curves.easeOut.transform(value);

        return Transform.scale(scale: value, child: child);
      },
      child: widget.children[index],
    );

    // We only want the second card to be animated.
    if (index == 1) {
      return _AnimatedCarouselCard(controller: widget.animationController, child: carouselCard);
    } else {
      return carouselCard;
    }
  }

  @override
  Widget build(BuildContext context) {
    return _AnimatedCarousel(
      controller: widget.animationController,
      child: PageView.builder(
        // Makes integration tests possible.
        key: const ValueKey<String>('studyDemoList'),
        onPageChanged: (int value) {
          setState(() {
            _currentPage.value = value;
          });
        },
        controller: _controller,
        pageSnapping: false,
        itemCount: widget.children.length,
        itemBuilder: (BuildContext context, int index) => builder(index),
        allowImplicitScrolling: true,
      ),
    );
  }
}

/// This creates a horizontally scrolling [ListView] of items.
///
/// This class uses a [ListView] with a custom [ScrollPhysics] to enable
/// snapping behavior. A [PageView] was considered but does not allow for
/// multiple pages visible without centering the first page.
class _DesktopCarousel extends StatefulWidget {
  const _DesktopCarousel({required this.height, required this.children});

  final double height;
  final List<Widget> children;

  @override
  _DesktopCarouselState createState() => _DesktopCarouselState();
}

class _DesktopCarouselState extends State<_DesktopCarousel> {
  late ScrollController _controller;

  @override
  void initState() {
    super.initState();
    _controller = ScrollController();
    _controller.addListener(() {
      setState(() {});
    });
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    bool showPreviousButton = false;
    bool showNextButton = true;
    // Only check this after the _controller has been attached to the ListView.
    if (_controller.hasClients) {
      showPreviousButton = _controller.offset > 0;
      showNextButton = _controller.offset < _controller.position.maxScrollExtent;
    }

    final bool isDesktop = isDisplayDesktop(context);

    return Align(
      child: Container(
        height: widget.height,
        constraints: const BoxConstraints(maxWidth: maxHomeItemWidth),
        child: Stack(
          children: <Widget>[
            ListView.builder(
              padding: EdgeInsets.symmetric(
                horizontal:
                    isDesktop
                        ? _horizontalDesktopPadding - _carouselItemDesktopMargin
                        : _horizontalPadding - _carouselItemMobileMargin,
              ),
              scrollDirection: Axis.horizontal,
              primary: false,
              physics: const _SnappingScrollPhysics(),
              controller: _controller,
              itemExtent: _carouselItemWidth,
              itemCount: widget.children.length,
              itemBuilder:
                  (BuildContext context, int index) => Padding(
                    padding: const EdgeInsets.symmetric(vertical: 8.0),
                    child: widget.children[index],
                  ),
            ),
            if (showPreviousButton)
              _DesktopPageButton(
                onTap: () {
                  _controller.animateTo(
                    _controller.offset - _carouselItemWidth,
                    duration: const Duration(milliseconds: 200),
                    curve: Curves.easeInOut,
                  );
                },
              ),
            if (showNextButton)
              _DesktopPageButton(
                isEnd: true,
                onTap: () {
                  _controller.animateTo(
                    _controller.offset + _carouselItemWidth,
                    duration: const Duration(milliseconds: 200),
                    curve: Curves.easeInOut,
                  );
                },
              ),
          ],
        ),
      ),
    );
  }
}

/// Scrolling physics that snaps to the new item in the [_DesktopCarousel].
class _SnappingScrollPhysics extends ScrollPhysics {
  const _SnappingScrollPhysics({super.parent});

  @override
  _SnappingScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return _SnappingScrollPhysics(parent: buildParent(ancestor));
  }

  double _getTargetPixels(ScrollMetrics position, Tolerance tolerance, double velocity) {
    final double itemWidth = position.viewportDimension / 4;
    double item = position.pixels / itemWidth;
    if (velocity < -tolerance.velocity) {
      item -= 0.5;
    } else if (velocity > tolerance.velocity) {
      item += 0.5;
    }
    return math.min(item.roundToDouble() * itemWidth, position.maxScrollExtent);
  }

  @override
  Simulation? createBallisticSimulation(ScrollMetrics position, double velocity) {
    if ((velocity <= 0.0 && position.pixels <= position.minScrollExtent) ||
        (velocity >= 0.0 && position.pixels >= position.maxScrollExtent)) {
      return super.createBallisticSimulation(position, velocity);
    }
    final Tolerance tolerance = toleranceFor(position);
    final double target = _getTargetPixels(position, tolerance, velocity);
    if (target != position.pixels) {
      return ScrollSpringSimulation(
        spring,
        position.pixels,
        target,
        velocity,
        tolerance: tolerance,
      );
    }
    return null;
  }

  @override
  bool get allowImplicitScrolling => true;
}

class _DesktopPageButton extends StatelessWidget {
  const _DesktopPageButton({this.isEnd = false, this.onTap});

  final bool isEnd;
  final GestureTapCallback? onTap;

  @override
  Widget build(BuildContext context) {
    const double buttonSize = 58.0;
    const double padding = _horizontalDesktopPadding - buttonSize / 2;
    return ExcludeSemantics(
      child: Align(
        alignment: isEnd ? AlignmentDirectional.centerEnd : AlignmentDirectional.centerStart,
        child: Container(
          width: buttonSize,
          height: buttonSize,
          margin: EdgeInsetsDirectional.only(start: isEnd ? 0 : padding, end: isEnd ? padding : 0),
          child: Tooltip(
            message:
                isEnd
                    ? MaterialLocalizations.of(context).nextPageTooltip
                    : MaterialLocalizations.of(context).previousPageTooltip,
            child: Material(
              color: Colors.black.withOpacity(0.5),
              shape: const CircleBorder(),
              clipBehavior: Clip.antiAlias,
              child: InkWell(
                onTap: onTap,
                child: Icon(
                  isEnd ? Icons.arrow_forward_ios : Icons.arrow_back_ios,
                  color: Colors.white,
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}

class _CarouselCard extends StatelessWidget {
  const _CarouselCard({
    required this.demo,
    this.asset,
    this.assetDark,
    this.assetColor,
    this.assetDarkColor,
    this.textColor,
    required this.studyRoute,
  });

  final GalleryDemo? demo;
  final ImageProvider? asset;
  final ImageProvider? assetDark;
  final Color? assetColor;
  final Color? assetDarkColor;
  final Color? textColor;
  final String studyRoute;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    final bool isDark = Theme.of(context).colorScheme.brightness == Brightness.dark;
    final ImageProvider<Object>? asset = isDark ? assetDark : this.asset;
    final Color? assetColor = isDark ? assetDarkColor : this.assetColor;
    final Color? textColor = isDark ? Colors.white.withOpacity(0.87) : this.textColor;
    final bool isDesktop = isDisplayDesktop(context);

    return Container(
      padding: EdgeInsets.symmetric(
        horizontal: isDesktop ? _carouselItemDesktopMargin : _carouselItemMobileMargin,
      ),
      margin: const EdgeInsets.symmetric(vertical: 16.0),
      height: _carouselHeight(0.7, context),
      width: _carouselItemWidth,
      child: Material(
        // Makes integration tests possible.
        key: ValueKey<String>(demo!.describe),
        color: assetColor,
        elevation: 4,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
        clipBehavior: Clip.antiAlias,
        child: Stack(
          fit: StackFit.expand,
          children: <Widget>[
            if (asset != null)
              FadeInImage(
                image: asset,
                placeholder: MemoryImage(kTransparentImage),
                fit: BoxFit.cover,
                height: _carouselHeightMin,
                fadeInDuration: entranceAnimationDuration,
              ),
            Padding(
              padding: const EdgeInsetsDirectional.fromSTEB(16, 0, 16, 16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                mainAxisAlignment: MainAxisAlignment.end,
                children: <Widget>[
                  Text(
                    demo!.title,
                    style: textTheme.bodySmall!.apply(color: textColor),
                    maxLines: 3,
                    overflow: TextOverflow.visible,
                  ),
                  Text(
                    demo!.subtitle,
                    style: textTheme.labelSmall!.apply(color: textColor),
                    maxLines: 5,
                    overflow: TextOverflow.visible,
                  ),
                ],
              ),
            ),
            Positioned.fill(
              child: Material(
                color: Colors.transparent,
                child: InkWell(
                  onTap: () {
                    Navigator.of(
                      context,
                    ).popUntil((Route<void> route) => route.settings.name == '/');
                    Navigator.of(context).restorablePushNamed(studyRoute);
                  },
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

double _carouselHeight(double scaleFactor, BuildContext context) => math.max(
  _carouselHeightMin * GalleryOptions.of(context).textScaleFactor(context) * scaleFactor,
  _carouselHeightMin,
);

/// Wrap the studies with this to display a back button and allow the user to
/// exit them at any time.
class StudyWrapper extends StatefulWidget {
  const StudyWrapper({
    super.key,
    required this.study,
    this.alignment = AlignmentDirectional.bottomStart,
    this.hasBottomNavBar = false,
  });

  final Widget study;
  final bool hasBottomNavBar;
  final AlignmentDirectional alignment;

  @override
  State<StudyWrapper> createState() => _StudyWrapperState();
}

class _StudyWrapperState extends State<StudyWrapper> {
  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    final TextTheme textTheme = Theme.of(context).textTheme;
    return ApplyTextOptions(
      child: Stack(
        children: <Widget>[
          Semantics(
            sortKey: const OrdinalSortKey(1),
            child: RestorationScope(restorationId: 'study_wrapper', child: widget.study),
          ),
          SafeArea(
            child: Align(
              alignment: widget.alignment,
              child: Padding(
                padding: EdgeInsets.symmetric(
                  horizontal: 16.0,
                  vertical: widget.hasBottomNavBar ? kBottomNavigationBarHeight + 16.0 : 16.0,
                ),
                child: Semantics(
                  sortKey: const OrdinalSortKey(0),
                  label: GalleryLocalizations.of(context)!.backToGallery,
                  button: true,
                  enabled: true,
                  excludeSemantics: true,
                  child: FloatingActionButton.extended(
                    heroTag: _BackButtonHeroTag(),
                    key: const ValueKey<String>('Back'),
                    onPressed: () {
                      Navigator.of(
                        context,
                      ).popUntil((Route<void> route) => route.settings.name == '/');
                    },
                    icon: IconTheme(
                      data: IconThemeData(color: colorScheme.onPrimary),
                      child: const BackButtonIcon(),
                    ),
                    label: Text(
                      MaterialLocalizations.of(context).backButtonTooltip,
                      style: textTheme.labelLarge!.apply(color: colorScheme.onPrimary),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _BackButtonHeroTag {}
