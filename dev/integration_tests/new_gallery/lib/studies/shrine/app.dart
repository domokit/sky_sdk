// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:scoped_model/scoped_model.dart';

import '../../data/gallery_options.dart';
import '../../gallery_localizations.dart';
import '../../layout/adaptive.dart';
import 'backdrop.dart';
import 'category_menu_page.dart';
import 'expanding_bottom_sheet.dart';
import 'home.dart';
import 'login.dart';
import 'model/app_state_model.dart';
import 'model/product.dart';
import 'page_status.dart';
import 'routes.dart' as routes;
import 'scrim.dart';
import 'supplemental/layout_cache.dart';
import 'theme.dart';

class ShrineApp extends StatefulWidget {
  const ShrineApp({super.key});

  static const String loginRoute = routes.loginRoute;
  static const String homeRoute = routes.homeRoute;

  @override
  State<ShrineApp> createState() => _ShrineAppState();
}

class _ShrineAppState extends State<ShrineApp> with TickerProviderStateMixin, RestorationMixin {
  // Controller to coordinate both the opening/closing of backdrop and sliding
  // of expanding bottom sheet
  late AnimationController _controller;

  // Animation Controller for expanding/collapsing the cart menu.
  late AnimationController _expandingController;

  final _RestorableAppStateModel _model = _RestorableAppStateModel();
  final RestorableDouble _expandingTabIndex = RestorableDouble(0);
  final RestorableDouble _tabIndex = RestorableDouble(1);
  final Map<String, List<List<int>>> _layouts = <String, List<List<int>>>{};

  @override
  String get restorationId => 'shrine_app_state';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_model, 'app_state_model');
    registerForRestoration(_tabIndex, 'tab_index');
    registerForRestoration(_expandingTabIndex, 'expanding_tab_index');
    _controller.value = _tabIndex.value;
    _expandingController.value = _expandingTabIndex.value;
  }

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 450),
      value: 1,
    );
    // Save state restoration animation values only when the cart page
    // fully opens or closes.
    _controller.addStatusListener((AnimationStatus status) {
      if (!status.isAnimating) {
        _tabIndex.value = _controller.value;
      }
    });
    _expandingController = AnimationController(
      duration: const Duration(milliseconds: 500),
      vsync: this,
    );
    // Save state restoration animation values only when the menu page
    // fully opens or closes.
    _expandingController.addStatusListener((AnimationStatus status) {
      if (!status.isAnimating) {
        _expandingTabIndex.value = _expandingController.value;
      }
    });
  }

  @override
  void dispose() {
    _controller.dispose();
    _expandingController.dispose();
    _tabIndex.dispose();
    _expandingTabIndex.dispose();
    super.dispose();
  }

  Widget mobileBackdrop() {
    return Backdrop(
      frontLayer: const ProductPage(),
      backLayer: CategoryMenuPage(onCategoryTap: () => _controller.forward()),
      frontTitle: const Text('SHRINE'),
      backTitle: Text(GalleryLocalizations.of(context)!.shrineMenuCaption),
      controller: _controller,
    );
  }

  Widget desktopBackdrop() {
    return const DesktopBackdrop(frontLayer: ProductPage(), backLayer: CategoryMenuPage());
  }

  // Closes the bottom sheet if it is open.
  Future<bool> _onWillPop() async {
    if (_expandingController.isForwardOrCompleted) {
      await _expandingController.reverse();
      return false;
    }

    return true;
  }

  @override
  Widget build(BuildContext context) {
    final Widget home = LayoutCache(
      layouts: _layouts,
      child: PageStatus(
        menuController: _controller,
        cartController: _expandingController,
        child: LayoutBuilder(
          builder:
              (BuildContext context, BoxConstraints constraints) => HomePage(
                backdrop: isDisplayDesktop(context) ? desktopBackdrop() : mobileBackdrop(),
                scrim: Scrim(controller: _expandingController),
                expandingBottomSheet: ExpandingBottomSheet(
                  hideController: _controller,
                  expandingController: _expandingController,
                ),
              ),
        ),
      ),
    );

    return ScopedModel<AppStateModel>(
      model: _model.value,
      // ignore: deprecated_member_use
      child: WillPopScope(
        onWillPop: _onWillPop,
        child: MaterialApp(
          // By default on desktop, scrollbars are applied by the
          // ScrollBehavior. This overrides that. All vertical scrollables in
          // the gallery need to be audited before enabling this feature,
          // see https://github.com/flutter/gallery/issues/541
          scrollBehavior: const MaterialScrollBehavior().copyWith(scrollbars: false),
          restorationScopeId: 'shrineApp',
          title: 'Shrine',
          debugShowCheckedModeBanner: false,
          initialRoute: ShrineApp.loginRoute,
          routes: <String, WidgetBuilder>{
            ShrineApp.loginRoute: (BuildContext context) => const LoginPage(),
            ShrineApp.homeRoute: (BuildContext context) => home,
          },
          theme: shrineTheme.copyWith(platform: GalleryOptions.of(context).platform),
          // L10n settings.
          localizationsDelegates: GalleryLocalizations.localizationsDelegates,
          supportedLocales: GalleryLocalizations.supportedLocales,
          locale: GalleryOptions.of(context).locale,
        ),
      ),
    );
  }
}

class _RestorableAppStateModel extends RestorableListenable<AppStateModel> {
  @override
  AppStateModel createDefaultValue() => AppStateModel()..loadProducts();

  @override
  AppStateModel fromPrimitives(Object? data) {
    final AppStateModel appState = AppStateModel()..loadProducts();
    final Map<String, dynamic> appData = Map<String, dynamic>.from(data! as Map<dynamic, dynamic>);

    // Reset selected category.
    final int categoryIndex = appData['category_index'] as int;
    appState.setCategory(categories[categoryIndex]);

    // Reset cart items.
    final Map<dynamic, dynamic> cartItems = appData['cart_data'] as Map<dynamic, dynamic>;
    cartItems.forEach((dynamic id, dynamic quantity) {
      appState.addMultipleProductsToCart(id as int, quantity as int);
    });

    return appState;
  }

  @override
  Object toPrimitives() {
    return <String, dynamic>{
      'cart_data': value.productsInCart,
      'category_index': categories.indexOf(value.selectedCategory),
    };
  }
}
