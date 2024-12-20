// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:ui' as ui;

import 'package:animations/animations.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:provider/provider.dart';

import '../../data/gallery_options.dart';
import '../../gallery_localizations.dart';
import '../../layout/adaptive.dart';
import 'app.dart';
import 'bottom_drawer.dart';
import 'colors.dart';
import 'compose_page.dart';
import 'mailbox_body.dart';
import 'model/email_model.dart';
import 'model/email_store.dart';
import 'profile_avatar.dart';
import 'search_page.dart';
import 'waterfall_notched_rectangle.dart';

const String _assetsPackage = 'flutter_gallery_assets';
const String _iconAssetLocation = 'reply/icons';
const String _folderIconAssetLocation = '$_iconAssetLocation/twotone_folder.png';
final GlobalKey<NavigatorState> desktopMailNavKey = GlobalKey<NavigatorState>();
final GlobalKey<NavigatorState> mobileMailNavKey = GlobalKey<NavigatorState>();
const double _kFlingVelocity = 2.0;
const Duration _kAnimationDuration = Duration(milliseconds: 300);

class AdaptiveNav extends StatefulWidget {
  const AdaptiveNav({super.key});

  @override
  State<AdaptiveNav> createState() => _AdaptiveNavState();
}

class _AdaptiveNavState extends State<AdaptiveNav> {
  @override
  void initState() {
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);
    final bool isTablet = isDisplaySmallDesktop(context);
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    final List<_Destination> navigationDestinations = <_Destination>[
      _Destination(
        type: MailboxPageType.inbox,
        textLabel: localizations.replyInboxLabel,
        icon: '$_iconAssetLocation/twotone_inbox.png',
      ),
      _Destination(
        type: MailboxPageType.starred,
        textLabel: localizations.replyStarredLabel,
        icon: '$_iconAssetLocation/twotone_star.png',
      ),
      _Destination(
        type: MailboxPageType.sent,
        textLabel: localizations.replySentLabel,
        icon: '$_iconAssetLocation/twotone_send.png',
      ),
      _Destination(
        type: MailboxPageType.trash,
        textLabel: localizations.replyTrashLabel,
        icon: '$_iconAssetLocation/twotone_delete.png',
      ),
      _Destination(
        type: MailboxPageType.spam,
        textLabel: localizations.replySpamLabel,
        icon: '$_iconAssetLocation/twotone_error.png',
      ),
      _Destination(
        type: MailboxPageType.drafts,
        textLabel: localizations.replyDraftsLabel,
        icon: '$_iconAssetLocation/twotone_drafts.png',
      ),
    ];

    final Map<String, String> folders = <String, String>{
      'Receipts': _folderIconAssetLocation,
      'Pine Elementary': _folderIconAssetLocation,
      'Taxes': _folderIconAssetLocation,
      'Vacation': _folderIconAssetLocation,
      'Mortgage': _folderIconAssetLocation,
      'Freelance': _folderIconAssetLocation,
    };

    if (isDesktop) {
      return _DesktopNav(
        extended: !isTablet,
        destinations: navigationDestinations,
        folders: folders,
        onItemTapped: _onDestinationSelected,
      );
    } else {
      return _MobileNav(
        destinations: navigationDestinations,
        folders: folders,
        onItemTapped: _onDestinationSelected,
      );
    }
  }

  void _onDestinationSelected(int index, MailboxPageType destination) {
    final EmailStore emailStore = Provider.of<EmailStore>(context, listen: false);

    final bool isDesktop = isDisplayDesktop(context);

    emailStore.selectedMailboxPage = destination;

    if (isDesktop) {
      while (desktopMailNavKey.currentState!.canPop()) {
        desktopMailNavKey.currentState!.pop();
      }
    }

    if (emailStore.onMailView) {
      if (!isDesktop) {
        mobileMailNavKey.currentState!.pop();
      }

      emailStore.selectedEmailId = -1;
    }
  }
}

class _DesktopNav extends StatefulWidget {
  const _DesktopNav({
    required this.extended,
    required this.destinations,
    required this.folders,
    required this.onItemTapped,
  });

  final bool extended;
  final List<_Destination> destinations;
  final Map<String, String> folders;
  final void Function(int, MailboxPageType) onItemTapped;

  @override
  _DesktopNavState createState() => _DesktopNavState();
}

class _DesktopNavState extends State<_DesktopNav> with SingleTickerProviderStateMixin {
  late ValueNotifier<bool> _isExtended;

  @override
  void initState() {
    super.initState();
    _isExtended = ValueNotifier<bool>(widget.extended);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Row(
        children: <Widget>[
          Consumer<EmailStore>(
            builder: (BuildContext context, EmailStore model, Widget? child) {
              return LayoutBuilder(
                builder: (BuildContext context, BoxConstraints constraints) {
                  final int selectedIndex = widget.destinations.indexWhere((
                    _Destination destination,
                  ) {
                    return destination.type == model.selectedMailboxPage;
                  });
                  return Container(
                    color: Theme.of(context).navigationRailTheme.backgroundColor,
                    child: SingleChildScrollView(
                      clipBehavior: Clip.antiAlias,
                      child: ConstrainedBox(
                        constraints: BoxConstraints(minHeight: constraints.maxHeight),
                        child: IntrinsicHeight(
                          child: ValueListenableBuilder<bool>(
                            valueListenable: _isExtended,
                            builder: (BuildContext context, bool value, Widget? child) {
                              return NavigationRail(
                                destinations: <NavigationRailDestination>[
                                  for (final _Destination destination in widget.destinations)
                                    NavigationRailDestination(
                                      icon: Material(
                                        key: ValueKey<String>('Reply-${destination.textLabel}'),
                                        color: Colors.transparent,
                                        child: ImageIcon(
                                          AssetImage(destination.icon, package: _assetsPackage),
                                        ),
                                      ),
                                      label: Text(destination.textLabel),
                                    ),
                                ],
                                extended: _isExtended.value,
                                labelType: NavigationRailLabelType.none,
                                leading: _NavigationRailHeader(extended: _isExtended),
                                trailing: _NavigationRailFolderSection(folders: widget.folders),
                                selectedIndex: selectedIndex,
                                onDestinationSelected: (int index) {
                                  widget.onItemTapped(index, widget.destinations[index].type);
                                },
                              );
                            },
                          ),
                        ),
                      ),
                    ),
                  );
                },
              );
            },
          ),
          const VerticalDivider(thickness: 1, width: 1),
          Expanded(
            child: Center(
              child: ConstrainedBox(
                constraints: const BoxConstraints(maxWidth: 1340),
                child: const _SharedAxisTransitionSwitcher(
                  defaultChild: _MailNavigator(child: MailboxBody()),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _NavigationRailHeader extends StatelessWidget {
  const _NavigationRailHeader({required this.extended});

  final ValueNotifier<bool> extended;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    final Animation<double> animation = NavigationRail.extendedAnimation(context);

    return AnimatedBuilder(
      animation: animation,
      builder: (BuildContext context, Widget? child) {
        return Align(
          alignment: AlignmentDirectional.centerStart,
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: <Widget>[
              SizedBox(
                height: 56,
                child: Row(
                  children: <Widget>[
                    const SizedBox(width: 6),
                    InkWell(
                      key: const ValueKey<String>('ReplyLogo'),
                      borderRadius: const BorderRadius.all(Radius.circular(16)),
                      onTap: () {
                        extended.value = !extended.value;
                      },
                      child: Row(
                        children: <Widget>[
                          Transform.rotate(
                            angle: animation.value * math.pi,
                            child: const Icon(
                              Icons.arrow_left,
                              color: ReplyColors.white50,
                              size: 16,
                            ),
                          ),
                          const _ReplyLogo(),
                          const SizedBox(width: 10),
                          Align(
                            alignment: AlignmentDirectional.centerStart,
                            widthFactor: animation.value,
                            child: Opacity(
                              opacity: animation.value,
                              child: Text(
                                'REPLY',
                                style: textTheme.bodyLarge!.copyWith(color: ReplyColors.white50),
                              ),
                            ),
                          ),
                          SizedBox(width: 18 * animation.value),
                        ],
                      ),
                    ),
                    if (animation.value > 0)
                      Opacity(
                        opacity: animation.value,
                        child: const Row(
                          children: <Widget>[
                            SizedBox(width: 18),
                            ProfileAvatar(avatar: 'reply/avatars/avatar_2.jpg', radius: 16),
                            SizedBox(width: 12),
                            Icon(Icons.settings, color: ReplyColors.white50),
                          ],
                        ),
                      ),
                  ],
                ),
              ),
              const SizedBox(height: 20),
              Padding(
                padding: const EdgeInsetsDirectional.only(start: 8),
                child: _ReplyFab(extended: extended.value),
              ),
              const SizedBox(height: 8),
            ],
          ),
        );
      },
    );
  }
}

class _NavigationRailFolderSection extends StatelessWidget {
  const _NavigationRailFolderSection({required this.folders});

  final Map<String, String> folders;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final TextTheme textTheme = theme.textTheme;
    final NavigationRailThemeData navigationRailTheme = theme.navigationRailTheme;
    final Animation<double> animation = NavigationRail.extendedAnimation(context);

    return AnimatedBuilder(
      animation: animation,
      builder: (BuildContext context, Widget? child) {
        return Visibility(
          maintainAnimation: true,
          maintainState: true,
          visible: animation.value > 0,
          child: Opacity(
            opacity: animation.value,
            child: Align(
              widthFactor: animation.value,
              alignment: AlignmentDirectional.centerStart,
              child: SizedBox(
                height: 485,
                width: 256,
                child: ListView(
                  padding: const EdgeInsets.all(12),
                  physics: const NeverScrollableScrollPhysics(),
                  children: <Widget>[
                    const Divider(
                      color: ReplyColors.blue200,
                      thickness: 0.4,
                      indent: 14,
                      endIndent: 16,
                    ),
                    const SizedBox(height: 16),
                    Padding(
                      padding: const EdgeInsetsDirectional.only(start: 16),
                      child: Text(
                        'FOLDERS',
                        style: textTheme.bodySmall!.copyWith(
                          color: navigationRailTheme.unselectedLabelTextStyle!.color,
                        ),
                      ),
                    ),
                    const SizedBox(height: 8),
                    for (final String folder in folders.keys)
                      InkWell(
                        borderRadius: const BorderRadius.all(Radius.circular(36)),
                        onTap: () {},
                        child: Column(
                          children: <Widget>[
                            Row(
                              children: <Widget>[
                                const SizedBox(width: 12),
                                ImageIcon(
                                  AssetImage(folders[folder]!, package: _assetsPackage),
                                  color: navigationRailTheme.unselectedLabelTextStyle!.color,
                                ),
                                const SizedBox(width: 24),
                                Text(
                                  folder,
                                  style: textTheme.bodyLarge!.copyWith(
                                    color: navigationRailTheme.unselectedLabelTextStyle!.color,
                                  ),
                                ),
                                const SizedBox(height: 72),
                              ],
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
      },
    );
  }
}

class _MobileNav extends StatefulWidget {
  const _MobileNav({required this.destinations, required this.folders, required this.onItemTapped});

  final List<_Destination> destinations;
  final Map<String, String> folders;
  final void Function(int, MailboxPageType) onItemTapped;

  @override
  _MobileNavState createState() => _MobileNavState();
}

class _MobileNavState extends State<_MobileNav> with TickerProviderStateMixin {
  final GlobalKey<State<StatefulWidget>> _bottomDrawerKey = GlobalKey(debugLabel: 'Bottom Drawer');
  late AnimationController _drawerController;
  late AnimationController _dropArrowController;
  late AnimationController _bottomAppBarController;
  late Animation<double> _drawerCurve;
  late Animation<double> _dropArrowCurve;
  late Animation<double> _bottomAppBarCurve;

  @override
  void initState() {
    super.initState();
    _drawerController = AnimationController(duration: _kAnimationDuration, value: 0, vsync: this)
      ..addListener(() {
        if (_drawerController.value < 0.01) {
          setState(() {
            //Reload state when drawer is at its smallest to toggle visibility
            //If state is reloaded before this drawer closes abruptly instead
            //of animating.
          });
        }
      });

    _dropArrowController = AnimationController(duration: _kAnimationDuration, vsync: this);

    _bottomAppBarController = AnimationController(
      vsync: this,
      value: 1,
      duration: const Duration(milliseconds: 250),
    );

    _drawerCurve = CurvedAnimation(
      parent: _drawerController,
      curve: Easing.legacy,
      reverseCurve: Easing.legacy.flipped,
    );

    _dropArrowCurve = CurvedAnimation(
      parent: _dropArrowController,
      curve: Easing.legacy,
      reverseCurve: Easing.legacy.flipped,
    );

    _bottomAppBarCurve = CurvedAnimation(
      parent: _bottomAppBarController,
      curve: Easing.legacy,
      reverseCurve: Easing.legacy.flipped,
    );
  }

  @override
  void dispose() {
    _drawerController.dispose();
    _dropArrowController.dispose();
    _bottomAppBarController.dispose();
    super.dispose();
  }

  bool get _bottomDrawerVisible => _drawerController.isForwardOrCompleted;

  void _toggleBottomDrawerVisibility() {
    if (_drawerController.value < 0.4) {
      _drawerController.animateTo(0.4, curve: Easing.legacy);
      _dropArrowController.animateTo(0.35, curve: Easing.legacy);
      return;
    }

    _dropArrowController.forward();
    _drawerController.fling(velocity: _bottomDrawerVisible ? -_kFlingVelocity : _kFlingVelocity);
  }

  double get _bottomDrawerHeight {
    final RenderBox renderBox = _bottomDrawerKey.currentContext!.findRenderObject()! as RenderBox;
    return renderBox.size.height;
  }

  void _handleDragUpdate(DragUpdateDetails details) {
    _drawerController.value -= details.primaryDelta! / _bottomDrawerHeight;
  }

  void _handleDragEnd(DragEndDetails details) {
    if (!_drawerController.isDismissed) {
      return;
    }

    final double flingVelocity = details.velocity.pixelsPerSecond.dy / _bottomDrawerHeight;

    if (flingVelocity < 0.0) {
      _drawerController.fling(velocity: math.max(_kFlingVelocity, -flingVelocity));
    } else if (flingVelocity > 0.0) {
      _dropArrowController.forward();
      _drawerController.fling(velocity: math.min(-_kFlingVelocity, -flingVelocity));
    } else {
      if (_drawerController.value < 0.6) {
        _dropArrowController.forward();
      }
      _drawerController.fling(
        velocity: _drawerController.value < 0.6 ? -_kFlingVelocity : _kFlingVelocity,
      );
    }
  }

  bool _handleScrollNotification(ScrollNotification notification) {
    if (notification case UserScrollNotification(depth: 0)) {
      switch (notification.direction) {
        case ScrollDirection.forward:
          _bottomAppBarController.forward();
        case ScrollDirection.reverse:
          _bottomAppBarController.reverse();
        case ScrollDirection.idle:
          break;
      }
    }
    return false;
  }

  Widget _buildStack(BuildContext context, BoxConstraints constraints) {
    final ui.Size drawerSize = constraints.biggest;
    final double drawerTop = drawerSize.height;

    final Animation<RelativeRect> drawerAnimation = RelativeRectTween(
      begin: RelativeRect.fromLTRB(0.0, drawerTop, 0.0, 0.0),
      end: RelativeRect.fill,
    ).animate(_drawerCurve);

    return Stack(
      clipBehavior: Clip.none,
      key: _bottomDrawerKey,
      children: <Widget>[
        NotificationListener<ScrollNotification>(
          onNotification: _handleScrollNotification,
          child: const _MailNavigator(child: MailboxBody()),
        ),
        MouseRegion(
          cursor: SystemMouseCursors.click,
          child: GestureDetector(
            onTap: () {
              _drawerController.reverse();
              _dropArrowController.reverse();
            },
            child: Visibility(
              maintainAnimation: true,
              maintainState: true,
              visible: _bottomDrawerVisible,
              child: FadeTransition(
                opacity: _drawerCurve,
                child: Container(
                  height: MediaQuery.of(context).size.height,
                  width: MediaQuery.of(context).size.width,
                  color: Theme.of(context).bottomSheetTheme.modalBackgroundColor,
                ),
              ),
            ),
          ),
        ),
        PositionedTransition(
          rect: drawerAnimation,
          child: Visibility(
            visible: _bottomDrawerVisible,
            child: BottomDrawer(
              onVerticalDragUpdate: _handleDragUpdate,
              onVerticalDragEnd: _handleDragEnd,
              leading: Consumer<EmailStore>(
                builder: (BuildContext context, EmailStore model, Widget? child) {
                  return _BottomDrawerDestinations(
                    destinations: widget.destinations,
                    drawerController: _drawerController,
                    dropArrowController: _dropArrowController,
                    selectedMailbox: model.selectedMailboxPage,
                    onItemTapped: widget.onItemTapped,
                  );
                },
              ),
              trailing: _BottomDrawerFolderSection(folders: widget.folders),
            ),
          ),
        ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    return _SharedAxisTransitionSwitcher(
      defaultChild: Scaffold(
        extendBody: true,
        body: LayoutBuilder(builder: _buildStack),
        bottomNavigationBar: Consumer<EmailStore>(
          builder: (BuildContext context, EmailStore model, Widget? child) {
            return _AnimatedBottomAppBar(
              bottomAppBarController: _bottomAppBarController,
              bottomAppBarCurve: _bottomAppBarCurve,
              bottomDrawerVisible: _bottomDrawerVisible,
              drawerController: _drawerController,
              dropArrowCurve: _dropArrowCurve,
              navigationDestinations: widget.destinations,
              selectedMailbox: model.selectedMailboxPage,
              toggleBottomDrawerVisibility: _toggleBottomDrawerVisibility,
            );
          },
        ),
        floatingActionButton:
            _bottomDrawerVisible
                ? null
                : const Padding(padding: EdgeInsetsDirectional.only(bottom: 8), child: _ReplyFab()),
        floatingActionButtonLocation: FloatingActionButtonLocation.centerDocked,
      ),
    );
  }
}

class _AnimatedBottomAppBar extends StatelessWidget {
  const _AnimatedBottomAppBar({
    required this.bottomAppBarController,
    required this.bottomAppBarCurve,
    required this.bottomDrawerVisible,
    required this.drawerController,
    required this.dropArrowCurve,
    required this.navigationDestinations,
    this.selectedMailbox,
    this.toggleBottomDrawerVisibility,
  });

  final AnimationController bottomAppBarController;
  final Animation<double> bottomAppBarCurve;
  final bool bottomDrawerVisible;
  final AnimationController drawerController;
  final Animation<double> dropArrowCurve;
  final List<_Destination> navigationDestinations;
  final MailboxPageType? selectedMailbox;
  final ui.VoidCallback? toggleBottomDrawerVisibility;

  @override
  Widget build(BuildContext context) {
    final Animation<double> fadeOut = Tween<double>(
      begin: 1,
      end: -1,
    ).animate(drawerController.drive(CurveTween(curve: Easing.legacy)));

    return Selector<EmailStore, bool>(
      selector: (BuildContext context, EmailStore emailStore) => emailStore.onMailView,
      builder: (BuildContext context, bool onMailView, Widget? child) {
        bottomAppBarController.forward();

        return SizeTransition(
          sizeFactor: bottomAppBarCurve,
          axisAlignment: -1,
          child: Padding(
            padding: const EdgeInsetsDirectional.only(top: 2),
            child: BottomAppBar(
              shape: const WaterfallNotchedRectangle(),
              notchMargin: 6,
              child: Container(
                color: Colors.transparent,
                height: kToolbarHeight,
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: <Widget>[
                    InkWell(
                      key: const ValueKey<String>('navigation_button'),
                      borderRadius: const BorderRadius.all(Radius.circular(16)),
                      onTap: toggleBottomDrawerVisibility,
                      child: Row(
                        children: <Widget>[
                          const SizedBox(width: 16),
                          RotationTransition(
                            turns: Tween<double>(begin: 0.0, end: 1.0).animate(dropArrowCurve),
                            child: const Icon(Icons.arrow_drop_up, color: ReplyColors.white50),
                          ),
                          const SizedBox(width: 8),
                          const _ReplyLogo(),
                          const SizedBox(width: 10),
                          _FadeThroughTransitionSwitcher(
                            fillColor: Colors.transparent,
                            child:
                                onMailView
                                    ? const SizedBox(width: 48)
                                    : FadeTransition(
                                      opacity: fadeOut,
                                      child: Text(
                                        navigationDestinations.firstWhere((
                                          _Destination destination,
                                        ) {
                                          return destination.type == selectedMailbox;
                                        }).textLabel,
                                        style: Theme.of(
                                          context,
                                        ).textTheme.bodyLarge!.copyWith(color: ReplyColors.white50),
                                      ),
                                    ),
                          ),
                        ],
                      ),
                    ),
                    Expanded(
                      child: ColoredBox(
                        color: Colors.transparent,
                        child: _BottomAppBarActionItems(drawerVisible: bottomDrawerVisible),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        );
      },
    );
  }
}

class _BottomAppBarActionItems extends StatelessWidget {
  const _BottomAppBarActionItems({required this.drawerVisible});

  final bool drawerVisible;

  @override
  Widget build(BuildContext context) {
    return Consumer<EmailStore>(
      builder: (BuildContext context, EmailStore model, Widget? child) {
        final bool onMailView = model.onMailView;
        Color? starIconColor;

        if (onMailView) {
          starIconColor =
              model.isCurrentEmailStarred
                  ? Theme.of(context).colorScheme.secondary
                  : ReplyColors.white50;
        }

        return _FadeThroughTransitionSwitcher(
          fillColor: Colors.transparent,
          child:
              drawerVisible
                  ? Align(
                    key: UniqueKey(),
                    alignment: Alignment.centerRight,
                    child: IconButton(
                      icon: const Icon(Icons.settings),
                      color: ReplyColors.white50,
                      onPressed: () {},
                    ),
                  )
                  : onMailView
                  ? Row(
                    mainAxisAlignment: MainAxisAlignment.end,
                    children: <Widget>[
                      IconButton(
                        key: const ValueKey<String>('star_email_button'),
                        icon: ImageIcon(
                          const AssetImage(
                            '$_iconAssetLocation/twotone_star.png',
                            package: _assetsPackage,
                          ),
                          color: starIconColor,
                        ),
                        onPressed: () {
                          final Email currentEmail = model.currentEmail;
                          if (model.isCurrentEmailStarred) {
                            model.unstarEmail(currentEmail.id);
                          } else {
                            model.starEmail(currentEmail.id);
                          }
                          if (model.selectedMailboxPage == MailboxPageType.starred) {
                            mobileMailNavKey.currentState!.pop();
                            model.selectedEmailId = -1;
                          }
                        },
                        color: ReplyColors.white50,
                      ),
                      IconButton(
                        icon: const ImageIcon(
                          AssetImage(
                            '$_iconAssetLocation/twotone_delete.png',
                            package: _assetsPackage,
                          ),
                        ),
                        onPressed: () {
                          model.deleteEmail(model.selectedEmailId);

                          mobileMailNavKey.currentState!.pop();
                          model.selectedEmailId = -1;
                        },
                        color: ReplyColors.white50,
                      ),
                      IconButton(
                        icon: const Icon(Icons.more_vert),
                        onPressed: () {},
                        color: ReplyColors.white50,
                      ),
                    ],
                  )
                  : Align(
                    alignment: Alignment.centerRight,
                    child: IconButton(
                      key: const ValueKey<String>('ReplySearch'),
                      icon: const Icon(Icons.search),
                      color: ReplyColors.white50,
                      onPressed: () {
                        Provider.of<EmailStore>(context, listen: false).onSearchPage = true;
                      },
                    ),
                  ),
        );
      },
    );
  }
}

class _BottomDrawerDestinations extends StatelessWidget {
  const _BottomDrawerDestinations({
    required this.destinations,
    required this.drawerController,
    required this.dropArrowController,
    required this.selectedMailbox,
    required this.onItemTapped,
  });

  final List<_Destination> destinations;
  final AnimationController drawerController;
  final AnimationController dropArrowController;
  final MailboxPageType selectedMailbox;
  final void Function(int, MailboxPageType) onItemTapped;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final List<Widget> destinationButtons = <Widget>[];

    for (int index = 0; index < destinations.length; index += 1) {
      final _Destination destination = destinations[index];
      destinationButtons.add(
        InkWell(
          key: ValueKey<String>('Reply-${destination.textLabel}'),
          onTap: () {
            drawerController.reverse();
            dropArrowController.forward();
            Future<void>.delayed(
              Duration(
                milliseconds:
                    (drawerController.value == 1 ? 300 : 120) *
                    GalleryOptions.of(context).timeDilation.toInt(),
              ),
              () {
                // Wait until animations are complete to reload the state.
                // Delay scales with the timeDilation value of the gallery.
                onItemTapped(index, destination.type);
              },
            );
          },
          child: ListTile(
            mouseCursor: SystemMouseCursors.click,
            leading: ImageIcon(
              AssetImage(destination.icon, package: _assetsPackage),
              color:
                  destination.type == selectedMailbox
                      ? theme.colorScheme.secondary
                      : theme.navigationRailTheme.unselectedLabelTextStyle!.color,
            ),
            title: Text(
              destination.textLabel,
              style: theme.textTheme.bodyMedium!.copyWith(
                color:
                    destination.type == selectedMailbox
                        ? theme.colorScheme.secondary
                        : theme.navigationRailTheme.unselectedLabelTextStyle!.color,
              ),
            ),
          ),
        ),
      );
    }

    return Column(children: destinationButtons);
  }
}

class _Destination {
  const _Destination({required this.type, required this.textLabel, required this.icon});

  // Which mailbox page to display. For example, 'Starred' or 'Trash'.
  final MailboxPageType type;

  // The localized text label for the inbox.
  final String textLabel;

  // The icon that appears next to the text label for the inbox.
  final String icon;
}

class _BottomDrawerFolderSection extends StatelessWidget {
  const _BottomDrawerFolderSection({required this.folders});

  final Map<String, String> folders;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final NavigationRailThemeData navigationRailTheme = theme.navigationRailTheme;

    return Column(
      children: <Widget>[
        for (final String folder in folders.keys)
          InkWell(
            onTap: () {},
            child: ListTile(
              mouseCursor: SystemMouseCursors.click,
              leading: ImageIcon(
                AssetImage(folders[folder]!, package: _assetsPackage),
                color: navigationRailTheme.unselectedLabelTextStyle!.color,
              ),
              title: Text(
                folder,
                style: theme.textTheme.bodyMedium!.copyWith(
                  color: navigationRailTheme.unselectedLabelTextStyle!.color,
                ),
              ),
            ),
          ),
      ],
    );
  }
}

class _MailNavigator extends StatefulWidget {
  const _MailNavigator({required this.child});

  final Widget child;

  @override
  _MailNavigatorState createState() => _MailNavigatorState();
}

class _MailNavigatorState extends State<_MailNavigator> {
  static const String inboxRoute = '/reply/inbox';

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);

    return Navigator(
      restorationScopeId: 'replyMailNavigator',
      key: isDesktop ? desktopMailNavKey : mobileMailNavKey,
      initialRoute: inboxRoute,
      onGenerateRoute: (RouteSettings settings) {
        switch (settings.name) {
          case inboxRoute:
            return MaterialPageRoute<void>(
              builder: (BuildContext context) {
                return _FadeThroughTransitionSwitcher(
                  fillColor: Theme.of(context).scaffoldBackgroundColor,
                  child: widget.child,
                );
              },
              settings: settings,
            );
          case ReplyApp.composeRoute:
            return ReplyApp.createComposeRoute(settings);
        }
        return null;
      },
    );
  }
}

class _ReplyLogo extends StatelessWidget {
  const _ReplyLogo();

  @override
  Widget build(BuildContext context) {
    return const ImageIcon(
      AssetImage('reply/reply_logo.png', package: _assetsPackage),
      size: 32,
      color: ReplyColors.white50,
    );
  }
}

class _ReplyFab extends StatefulWidget {
  const _ReplyFab({this.extended = false});

  final bool extended;

  @override
  _ReplyFabState createState() => _ReplyFabState();
}

class _ReplyFabState extends State<_ReplyFab> with SingleTickerProviderStateMixin {
  static final UniqueKey fabKey = UniqueKey();
  static const double _mobileFabDimension = 56;

  void onPressed() {
    final bool onSearchPage = Provider.of<EmailStore>(context, listen: false).onSearchPage;
    // Navigator does not have an easy way to access the current
    // route when using a GlobalKey to keep track of NavigatorState.
    // We can use [Navigator.popUntil] in order to access the current
    // route, and check if it is a ComposePage. If it is not a
    // ComposePage and we are not on the SearchPage, then we can push
    // a ComposePage onto our navigator. We return true at the end
    // so nothing is popped.
    desktopMailNavKey.currentState!.popUntil((Route<void> route) {
      final String? currentRoute = route.settings.name;
      if (currentRoute != ReplyApp.composeRoute && !onSearchPage) {
        desktopMailNavKey.currentState!.restorablePushNamed(ReplyApp.composeRoute);
      }
      return true;
    });
  }

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);
    final ThemeData theme = Theme.of(context);
    const CircleBorder circleFabBorder = CircleBorder();

    return Selector<EmailStore, bool>(
      selector: (BuildContext context, EmailStore emailStore) => emailStore.onMailView,
      builder: (BuildContext context, bool onMailView, Widget? child) {
        final _FadeThroughTransitionSwitcher fabSwitcher = _FadeThroughTransitionSwitcher(
          fillColor: Colors.transparent,
          child:
              onMailView
                  ? Icon(Icons.reply_all, key: fabKey, color: Colors.black)
                  : const Icon(Icons.create, color: Colors.black),
        );
        final String tooltip = onMailView ? 'Reply' : 'Compose';

        if (isDesktop) {
          final Animation<double> animation = NavigationRail.extendedAnimation(context);
          return Container(
            height: 56,
            padding: EdgeInsets.symmetric(vertical: ui.lerpDouble(0, 6, animation.value)!),
            child:
                animation.value == 0
                    ? FloatingActionButton(
                      tooltip: tooltip,
                      key: const ValueKey<String>('ReplyFab'),
                      onPressed: onPressed,
                      child: fabSwitcher,
                    )
                    : Align(
                      alignment: AlignmentDirectional.centerStart,
                      child: FloatingActionButton.extended(
                        key: const ValueKey<String>('ReplyFab'),
                        label: Row(
                          children: <Widget>[
                            fabSwitcher,
                            SizedBox(width: 16 * animation.value),
                            Align(
                              alignment: AlignmentDirectional.centerStart,
                              widthFactor: animation.value,
                              child: Text(
                                tooltip.toUpperCase(),
                                style: Theme.of(context).textTheme.headlineSmall!.copyWith(
                                  fontSize: 16,
                                  color: theme.colorScheme.onSecondary,
                                ),
                              ),
                            ),
                          ],
                        ),
                        onPressed: onPressed,
                      ),
                    ),
          );
        } else {
          // TODO(x): State restoration of compose page on mobile is blocked because OpenContainer does not support restorablePush, https://github.com/flutter/gallery/issues/570.
          return OpenContainer(
            openBuilder: (BuildContext context, void Function() closedContainer) {
              return const ComposePage();
            },
            openColor: theme.cardColor,
            closedShape: circleFabBorder,
            closedColor: theme.colorScheme.secondary,
            closedElevation: 6,
            closedBuilder: (BuildContext context, void Function() openContainer) {
              return Tooltip(
                message: tooltip,
                child: InkWell(
                  key: const ValueKey<String>('ReplyFab'),
                  customBorder: circleFabBorder,
                  onTap: openContainer,
                  child: SizedBox(
                    height: _mobileFabDimension,
                    width: _mobileFabDimension,
                    child: Center(child: fabSwitcher),
                  ),
                ),
              );
            },
          );
        }
      },
    );
  }
}

class _FadeThroughTransitionSwitcher extends StatelessWidget {
  const _FadeThroughTransitionSwitcher({required this.fillColor, required this.child});

  final Widget child;
  final Color fillColor;

  @override
  Widget build(BuildContext context) {
    return PageTransitionSwitcher(
      transitionBuilder: (
        Widget child,
        Animation<double> animation,
        Animation<double> secondaryAnimation,
      ) {
        return FadeThroughTransition(
          fillColor: fillColor,
          animation: animation,
          secondaryAnimation: secondaryAnimation,
          child: child,
        );
      },
      child: child,
    );
  }
}

class _SharedAxisTransitionSwitcher extends StatelessWidget {
  const _SharedAxisTransitionSwitcher({required this.defaultChild});

  final Widget defaultChild;

  @override
  Widget build(BuildContext context) {
    return Selector<EmailStore, bool>(
      selector: (BuildContext context, EmailStore emailStore) => emailStore.onSearchPage,
      builder: (BuildContext context, bool onSearchPage, Widget? child) {
        return PageTransitionSwitcher(
          reverse: !onSearchPage,
          transitionBuilder: (
            Widget child,
            Animation<double> animation,
            Animation<double> secondaryAnimation,
          ) {
            return SharedAxisTransition(
              fillColor: Theme.of(context).colorScheme.background,
              animation: animation,
              secondaryAnimation: secondaryAnimation,
              transitionType: SharedAxisTransitionType.scaled,
              child: child,
            );
          },
          child: onSearchPage ? const SearchPage() : defaultChild,
        );
      },
    );
  }
}
