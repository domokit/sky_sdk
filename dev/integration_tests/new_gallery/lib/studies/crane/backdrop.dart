// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_staggered_grid_view/flutter_staggered_grid_view.dart';

import '../../data/gallery_options.dart';
import '../../gallery_localizations.dart';
import '../../layout/adaptive.dart';
import '../../layout/image_placeholder.dart';
import 'backlayer.dart';
import 'border_tab_indicator.dart';
import 'colors.dart';
import 'header_form.dart';
import 'item_cards.dart';
import 'model/data.dart';
import 'model/destination.dart';

class _FrontLayer extends StatefulWidget {
  const _FrontLayer({
    required this.title,
    required this.index,
    required this.mobileTopOffset,
    required this.restorationId,
  });

  final String title;
  final int index;
  final double mobileTopOffset;
  final String restorationId;

  @override
  _FrontLayerState createState() => _FrontLayerState();
}

class _FrontLayerState extends State<_FrontLayer> {
  List<Destination>? destinations;

  static const double frontLayerBorderRadius = 16.0;
  static const EdgeInsets bottomPadding = EdgeInsets.only(bottom: 120);

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    // We use didChangeDependencies because the initialization involves an
    // InheritedWidget (for localization). However, we don't need to get
    // destinations again when, say, resizing the window.
    if (destinations == null) {
      if (widget.index == 0) {
        destinations = getFlyDestinations(context);
      }
      if (widget.index == 1) {
        destinations = getSleepDestinations(context);
      }
      if (widget.index == 2) {
        destinations = getEatDestinations(context);
      }
    }
  }

  Widget _header() {
    return Align(
      alignment: AlignmentDirectional.centerStart,
      child: Padding(
        padding: const EdgeInsets.only(top: 20, bottom: 22),
        child: SelectableText(widget.title, style: Theme.of(context).textTheme.titleSmall),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);
    final bool isSmallDesktop = isDisplaySmallDesktop(context);
    final int crossAxisCount = isDesktop ? 4 : 1;

    return FocusTraversalGroup(
      policy: ReadingOrderTraversalPolicy(),
      child: Padding(
        padding: isDesktop ? EdgeInsets.zero : EdgeInsets.only(top: widget.mobileTopOffset),
        child: PhysicalShape(
          elevation: 16,
          color: cranePrimaryWhite,
          clipper: const ShapeBorderClipper(
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.only(
                topLeft: Radius.circular(frontLayerBorderRadius),
                topRight: Radius.circular(frontLayerBorderRadius),
              ),
            ),
          ),
          child: Padding(
            padding:
                isDesktop
                    ? EdgeInsets.symmetric(
                      horizontal: isSmallDesktop ? appPaddingSmall : appPaddingLarge,
                    ).add(bottomPadding)
                    : const EdgeInsets.symmetric(horizontal: 20).add(bottomPadding),
            child: Column(
              children: <Widget>[
                _header(),
                Expanded(
                  child: MasonryGridView.count(
                    key: ValueKey<String>('CraneListView-${widget.index}'),
                    restorationId: widget.restorationId,
                    crossAxisCount: crossAxisCount,
                    crossAxisSpacing: 16.0,
                    itemBuilder:
                        (BuildContext context, int index) =>
                            DestinationCard(destination: destinations![index]),
                    itemCount: destinations!.length,
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

/// Builds a Backdrop.
///
/// A Backdrop widget has two layers, front and back. The front layer is shown
/// by default, and slides down to show the back layer, from which a user
/// can make a selection. The user can also configure the titles for when the
/// front or back layer is showing.
class Backdrop extends StatefulWidget {
  const Backdrop({
    super.key,
    required this.frontLayer,
    required this.backLayerItems,
    required this.frontTitle,
    required this.backTitle,
  });
  final Widget frontLayer;
  final List<BackLayerItem> backLayerItems;
  final Widget frontTitle;
  final Widget backTitle;

  @override
  State<Backdrop> createState() => _BackdropState();
}

class _BackdropState extends State<Backdrop> with TickerProviderStateMixin, RestorationMixin {
  final RestorableInt tabIndex = RestorableInt(0);
  late TabController _tabController;
  late Animation<Offset> _flyLayerHorizontalOffset;
  late Animation<Offset> _sleepLayerHorizontalOffset;
  late Animation<Offset> _eatLayerHorizontalOffset;

  // How much the 'sleep' front layer is vertically offset relative to other
  // front layers, in pixels, with the mobile layout.
  static const double _sleepLayerTopOffset = 60.0;

  @override
  String get restorationId => 'tab_non_scrollable_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(tabIndex, 'tab_index');
    _tabController.index = tabIndex.value;
  }

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 3, vsync: this);
    _tabController.addListener(() {
      // When the tab controller's value is updated, make sure to update the
      // tab index value, which is state restorable.
      setState(() {
        tabIndex.value = _tabController.index;
      });
    });

    // Offsets to create a horizontal gap between front layers.
    final Animation<double> tabControllerAnimation = _tabController.animation!;

    _flyLayerHorizontalOffset = tabControllerAnimation.drive(
      Tween<Offset>(begin: Offset.zero, end: const Offset(-0.05, 0)),
    );

    _sleepLayerHorizontalOffset = tabControllerAnimation.drive(
      Tween<Offset>(begin: const Offset(0.05, 0), end: Offset.zero),
    );

    _eatLayerHorizontalOffset = tabControllerAnimation.drive(
      Tween<Offset>(begin: const Offset(0.10, 0), end: const Offset(0.05, 0)),
    );
  }

  @override
  void dispose() {
    _tabController.dispose();
    tabIndex.dispose();
    super.dispose();
  }

  void _handleTabs(int tabIndex) {
    _tabController.animateTo(tabIndex, duration: const Duration(milliseconds: 300));
  }

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);
    final double textScaleFactor = GalleryOptions.of(context).textScaleFactor(context);
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Material(
      color: cranePurple800,
      child: Padding(
        padding: const EdgeInsets.only(top: 12),
        child: FocusTraversalGroup(
          policy: ReadingOrderTraversalPolicy(),
          child: Scaffold(
            backgroundColor: cranePurple800,
            appBar: AppBar(
              automaticallyImplyLeading: false,
              systemOverlayStyle: SystemUiOverlayStyle.light,
              elevation: 0,
              titleSpacing: 0,
              flexibleSpace: CraneAppBar(tabController: _tabController, tabHandler: _handleTabs),
            ),
            body: Stack(
              children: <Widget>[
                BackLayer(tabController: _tabController, backLayerItems: widget.backLayerItems),
                Container(
                  margin: EdgeInsets.only(
                    top:
                        isDesktop
                            ? (isDisplaySmallDesktop(context)
                                    ? textFieldHeight * 3
                                    : textFieldHeight * 2) +
                                20 * textScaleFactor / 2
                            : 175 + 140 * textScaleFactor / 2,
                  ),
                  // To display the middle front layer higher than the others,
                  // we allow the TabBarView to overflow by an offset
                  // (doubled because it technically overflows top & bottom).
                  // The other front layers are top padded by this offset.
                  child: LayoutBuilder(
                    builder: (BuildContext context, BoxConstraints constraints) {
                      return OverflowBox(
                        maxHeight: constraints.maxHeight + _sleepLayerTopOffset * 2,
                        child: TabBarView(
                          physics:
                              isDesktop
                                  ? const NeverScrollableScrollPhysics()
                                  : null, // use default TabBarView physics
                          controller: _tabController,
                          children: <Widget>[
                            SlideTransition(
                              position: _flyLayerHorizontalOffset,
                              child: _FrontLayer(
                                title: localizations.craneFlySubhead,
                                index: 0,
                                mobileTopOffset: _sleepLayerTopOffset,
                                restorationId: 'fly-subhead',
                              ),
                            ),
                            SlideTransition(
                              position: _sleepLayerHorizontalOffset,
                              child: _FrontLayer(
                                title: localizations.craneSleepSubhead,
                                index: 1,
                                mobileTopOffset: 0,
                                restorationId: 'sleep-subhead',
                              ),
                            ),
                            SlideTransition(
                              position: _eatLayerHorizontalOffset,
                              child: _FrontLayer(
                                title: localizations.craneEatSubhead,
                                index: 2,
                                mobileTopOffset: _sleepLayerTopOffset,
                                restorationId: 'eat-subhead',
                              ),
                            ),
                          ],
                        ),
                      );
                    },
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

class CraneAppBar extends StatefulWidget {
  const CraneAppBar({super.key, this.tabHandler, required this.tabController});
  final void Function(int)? tabHandler;
  final TabController tabController;

  @override
  State<CraneAppBar> createState() => _CraneAppBarState();
}

class _CraneAppBarState extends State<CraneAppBar> {
  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);
    final bool isSmallDesktop = isDisplaySmallDesktop(context);
    final double textScaleFactor = GalleryOptions.of(context).textScaleFactor(context);
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return SafeArea(
      child: Padding(
        padding: EdgeInsets.symmetric(
          horizontal: isDesktop && !isSmallDesktop ? appPaddingLarge : appPaddingSmall,
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: <Widget>[
            const ExcludeSemantics(
              child: FadeInImagePlaceholder(
                image: ResizeImage(
                  AssetImage('crane/logo/logo.png', package: 'flutter_gallery_assets'),
                  width: 40,
                  height: 60,
                ),
                placeholder: SizedBox(width: 40, height: 60),
                width: 40,
                height: 60,
              ),
            ),
            Expanded(
              child: Padding(
                padding: const EdgeInsetsDirectional.only(start: 24),
                child: Theme(
                  data: Theme.of(context).copyWith(splashColor: Colors.transparent),
                  child: TabBar(
                    indicator: BorderTabIndicator(
                      indicatorHeight: isDesktop ? 28 : 32,
                      textScaleFactor: textScaleFactor,
                    ),
                    controller: widget.tabController,
                    labelPadding: const EdgeInsets.symmetric(horizontal: 32),
                    isScrollable: true,
                    // left-align tabs on desktop
                    labelStyle: Theme.of(context).textTheme.labelLarge,
                    labelColor: cranePrimaryWhite,
                    physics: const BouncingScrollPhysics(),
                    unselectedLabelColor: cranePrimaryWhite.withOpacity(.6),
                    onTap:
                        (int index) => widget.tabController.animateTo(
                          index,
                          duration: const Duration(milliseconds: 300),
                        ),
                    tabs: <Widget>[
                      Tab(text: localizations.craneFly),
                      Tab(text: localizations.craneSleep),
                      Tab(text: localizations.craneEat),
                    ],
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
