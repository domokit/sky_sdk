// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';

import '../constants.dart';
import '../data/gallery_options.dart';
import '../gallery_localizations.dart';
import '../layout/adaptive.dart';
import 'home.dart';
import 'settings.dart';
import 'settings_icon/icon.dart' as settings_icon;

const double _settingsButtonWidth = 64;
const double _settingsButtonHeightDesktop = 56;
const double _settingsButtonHeightMobile = 40;

class Backdrop extends StatefulWidget {
  const Backdrop({super.key, required this.isDesktop, this.settingsPage, this.homePage});

  final bool isDesktop;
  final Widget? settingsPage;
  final Widget? homePage;

  @override
  State<Backdrop> createState() => _BackdropState();
}

class _BackdropState extends State<Backdrop> with TickerProviderStateMixin {
  late AnimationController _settingsPanelController;
  late AnimationController _iconController;
  late FocusNode _settingsPageFocusNode;
  late ValueNotifier<bool> _isSettingsOpenNotifier;
  late Widget _settingsPage;
  late Widget _homePage;

  @override
  void initState() {
    super.initState();
    _settingsPanelController = AnimationController(
      vsync: this,
      duration:
          widget.isDesktop
              ? settingsPanelMobileAnimationDuration
              : settingsPanelDesktopAnimationDuration,
    );
    _iconController = AnimationController(vsync: this, duration: const Duration(milliseconds: 500));
    _settingsPageFocusNode = FocusNode();
    _isSettingsOpenNotifier = ValueNotifier<bool>(false);
    _settingsPage =
        widget.settingsPage ?? SettingsPage(animationController: _settingsPanelController);
    _homePage = widget.homePage ?? const HomePage();
  }

  @override
  void dispose() {
    _settingsPanelController.dispose();
    _iconController.dispose();
    _settingsPageFocusNode.dispose();
    _isSettingsOpenNotifier.dispose();
    super.dispose();
  }

  void _toggleSettings() {
    // Animate the settings panel to open or close.
    if (_isSettingsOpenNotifier.value) {
      _settingsPanelController.reverse();
      _iconController.reverse();
    } else {
      _settingsPanelController.forward();
      _iconController.forward();
    }
    _isSettingsOpenNotifier.value = !_isSettingsOpenNotifier.value;
  }

  Animation<RelativeRect> _slideDownSettingsPageAnimation(BoxConstraints constraints) {
    return RelativeRectTween(
      begin: RelativeRect.fromLTRB(0, -constraints.maxHeight, 0, 0),
      end: RelativeRect.fill,
    ).animate(
      CurvedAnimation(
        parent: _settingsPanelController,
        curve: const Interval(0.0, 0.4, curve: Curves.ease),
      ),
    );
  }

  Animation<RelativeRect> _slideDownHomePageAnimation(BoxConstraints constraints) {
    return RelativeRectTween(
      begin: RelativeRect.fill,
      end: RelativeRect.fromLTRB(
        0,
        constraints.biggest.height - galleryHeaderHeight,
        0,
        -galleryHeaderHeight,
      ),
    ).animate(
      CurvedAnimation(
        parent: _settingsPanelController,
        curve: const Interval(0.0, 0.4, curve: Curves.ease),
      ),
    );
  }

  Widget _buildStack(BuildContext context, BoxConstraints constraints) {
    final bool isDesktop = isDisplayDesktop(context);

    final Widget settingsPage = ValueListenableBuilder<bool>(
      valueListenable: _isSettingsOpenNotifier,
      builder: (BuildContext context, bool isSettingsOpen, Widget? child) {
        return ExcludeSemantics(
          excluding: !isSettingsOpen,
          child:
              isSettingsOpen
                  ? KeyboardListener(
                    includeSemantics: false,
                    focusNode: _settingsPageFocusNode,
                    onKeyEvent: (KeyEvent event) {
                      if (event.logicalKey == LogicalKeyboardKey.escape) {
                        _toggleSettings();
                      }
                    },
                    child: FocusScope(child: _settingsPage),
                  )
                  : ExcludeFocus(child: _settingsPage),
        );
      },
    );

    final Widget homePage = ValueListenableBuilder<bool>(
      valueListenable: _isSettingsOpenNotifier,
      builder: (BuildContext context, bool isSettingsOpen, Widget? child) {
        return ExcludeSemantics(
          excluding: isSettingsOpen,
          child: FocusTraversalGroup(child: _homePage),
        );
      },
    );

    return AnnotatedRegion<SystemUiOverlayStyle>(
      value: GalleryOptions.of(context).resolvedSystemUiOverlayStyle(),
      child: Stack(
        children: <Widget>[
          if (!isDesktop) ...<Widget>[
            // Slides the settings page up and down from the top of the
            // screen.
            PositionedTransition(
              rect: _slideDownSettingsPageAnimation(constraints),
              child: settingsPage,
            ),
            // Slides the home page up and down below the bottom of the
            // screen.
            PositionedTransition(rect: _slideDownHomePageAnimation(constraints), child: homePage),
          ],
          if (isDesktop) ...<Widget>[
            Semantics(sortKey: const OrdinalSortKey(2), child: homePage),
            ValueListenableBuilder<bool>(
              valueListenable: _isSettingsOpenNotifier,
              builder: (BuildContext context, bool isSettingsOpen, Widget? child) {
                if (isSettingsOpen) {
                  return ExcludeSemantics(
                    child: Listener(
                      onPointerDown: (_) => _toggleSettings(),
                      child: const ModalBarrier(dismissible: false),
                    ),
                  );
                } else {
                  return Container();
                }
              },
            ),
            Semantics(
              sortKey: const OrdinalSortKey(3),
              child: ScaleTransition(
                alignment:
                    Directionality.of(context) == TextDirection.ltr
                        ? Alignment.topRight
                        : Alignment.topLeft,
                scale: CurvedAnimation(
                  parent: _settingsPanelController,
                  curve: Curves.fastOutSlowIn,
                ),
                child: Align(
                  alignment: AlignmentDirectional.topEnd,
                  child: Material(
                    elevation: 7,
                    clipBehavior: Clip.antiAlias,
                    borderRadius: BorderRadius.circular(40),
                    color: Theme.of(context).colorScheme.secondaryContainer,
                    child: Container(
                      constraints: const BoxConstraints(
                        maxHeight: 560,
                        maxWidth: desktopSettingsWidth,
                        minWidth: desktopSettingsWidth,
                      ),
                      child: settingsPage,
                    ),
                  ),
                ),
              ),
            ),
          ],
          _SettingsIcon(
            animationController: _iconController,
            toggleSettings: _toggleSettings,
            isSettingsOpenNotifier: _isSettingsOpenNotifier,
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(builder: _buildStack);
  }
}

class _SettingsIcon extends AnimatedWidget {
  const _SettingsIcon({
    required this.animationController,
    required this.toggleSettings,
    required this.isSettingsOpenNotifier,
  }) : super(listenable: animationController);

  final AnimationController animationController;
  final VoidCallback toggleSettings;
  final ValueNotifier<bool> isSettingsOpenNotifier;

  String _settingsSemanticLabel(bool isOpen, BuildContext context) {
    return isOpen
        ? GalleryLocalizations.of(context)!.settingsButtonCloseLabel
        : GalleryLocalizations.of(context)!.settingsButtonLabel;
  }

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);
    final double safeAreaTopPadding = MediaQuery.of(context).padding.top;

    return Align(
      alignment: AlignmentDirectional.topEnd,
      child: Semantics(
        sortKey: const OrdinalSortKey(1),
        button: true,
        enabled: true,
        label: _settingsSemanticLabel(isSettingsOpenNotifier.value, context),
        child: SizedBox(
          width: _settingsButtonWidth,
          height:
              isDesktop
                  ? _settingsButtonHeightDesktop
                  : _settingsButtonHeightMobile + safeAreaTopPadding,
          child: Material(
            borderRadius: const BorderRadiusDirectional.only(bottomStart: Radius.circular(10)),
            color:
                isSettingsOpenNotifier.value & !animationController.isAnimating
                    ? Colors.transparent
                    : Theme.of(context).colorScheme.secondaryContainer,
            clipBehavior: Clip.antiAlias,
            child: InkWell(
              onTap: () {
                toggleSettings();
                SemanticsService.announce(
                  _settingsSemanticLabel(isSettingsOpenNotifier.value, context),
                  GalleryOptions.of(context).resolvedTextDirection()!,
                );
              },
              child: Padding(
                padding: const EdgeInsetsDirectional.only(start: 3, end: 18),
                child: settings_icon.SettingsIcon(animationController.value),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
