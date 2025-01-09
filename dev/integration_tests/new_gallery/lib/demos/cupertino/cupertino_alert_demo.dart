// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

import '../../data/gallery_options.dart';
import '../../gallery_localizations.dart';
import 'demo_types.dart';

// BEGIN cupertinoAlertDemo

class CupertinoAlertDemo extends StatefulWidget {
  const CupertinoAlertDemo({super.key, required this.type});

  final AlertDemoType type;

  @override
  State<CupertinoAlertDemo> createState() => _CupertinoAlertDemoState();
}

class _CupertinoAlertDemoState extends State<CupertinoAlertDemo> with RestorationMixin {
  RestorableStringN lastSelectedValue = RestorableStringN(null);
  late RestorableRouteFuture<String> _alertDialogRoute;
  late RestorableRouteFuture<String> _alertWithTitleDialogRoute;
  late RestorableRouteFuture<String> _alertWithButtonsDialogRoute;
  late RestorableRouteFuture<String> _alertWithButtonsOnlyDialogRoute;
  late RestorableRouteFuture<String> _modalPopupRoute;

  @override
  String get restorationId => 'cupertino_alert_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(lastSelectedValue, 'last_selected_value');
    registerForRestoration(_alertDialogRoute, 'alert_demo_dialog_route');
    registerForRestoration(_alertWithTitleDialogRoute, 'alert_with_title_press_demo_dialog_route');
    registerForRestoration(
      _alertWithButtonsDialogRoute,
      'alert_with_title_press_demo_dialog_route',
    );
    registerForRestoration(
      _alertWithButtonsOnlyDialogRoute,
      'alert_with_title_press_demo_dialog_route',
    );
    registerForRestoration(_modalPopupRoute, 'modal_popup_route');
  }

  void _setSelectedValue(String value) {
    setState(() {
      lastSelectedValue.value = value;
    });
  }

  @override
  void initState() {
    super.initState();
    _alertDialogRoute = RestorableRouteFuture<String>(
      onPresent: (NavigatorState navigator, Object? arguments) {
        return navigator.restorablePush(_alertDemoDialog);
      },
      onComplete: _setSelectedValue,
    );
    _alertWithTitleDialogRoute = RestorableRouteFuture<String>(
      onPresent: (NavigatorState navigator, Object? arguments) {
        return navigator.restorablePush(_alertWithTitleDialog);
      },
      onComplete: _setSelectedValue,
    );
    _alertWithButtonsDialogRoute = RestorableRouteFuture<String>(
      onPresent: (NavigatorState navigator, Object? arguments) {
        return navigator.restorablePush(_alertWithButtonsDialog);
      },
      onComplete: _setSelectedValue,
    );
    _alertWithButtonsOnlyDialogRoute = RestorableRouteFuture<String>(
      onPresent: (NavigatorState navigator, Object? arguments) {
        return navigator.restorablePush(_alertWithButtonsOnlyDialog);
      },
      onComplete: _setSelectedValue,
    );
    _modalPopupRoute = RestorableRouteFuture<String>(
      onPresent: (NavigatorState navigator, Object? arguments) {
        return navigator.restorablePush(_modalRoute);
      },
      onComplete: _setSelectedValue,
    );
  }

  String _title(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return switch (widget.type) {
      AlertDemoType.alert => localizations.demoCupertinoAlertTitle,
      AlertDemoType.alertTitle => localizations.demoCupertinoAlertWithTitleTitle,
      AlertDemoType.alertButtons => localizations.demoCupertinoAlertButtonsTitle,
      AlertDemoType.alertButtonsOnly => localizations.demoCupertinoAlertButtonsOnlyTitle,
      AlertDemoType.actionSheet => localizations.demoCupertinoActionSheetTitle,
    };
  }

  static Route<String> _alertDemoDialog(BuildContext context, Object? arguments) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return CupertinoDialogRoute<String>(
      context: context,
      builder:
          (BuildContext context) => ApplyTextOptions(
            child: CupertinoAlertDialog(
              title: Text(localizations.dialogDiscardTitle),
              actions: <Widget>[
                CupertinoDialogAction(
                  isDestructiveAction: true,
                  onPressed: () {
                    Navigator.of(context).pop(localizations.cupertinoAlertDiscard);
                  },
                  child: Text(localizations.cupertinoAlertDiscard),
                ),
                CupertinoDialogAction(
                  isDefaultAction: true,
                  onPressed: () => Navigator.of(context).pop(localizations.cupertinoAlertCancel),
                  child: Text(localizations.cupertinoAlertCancel),
                ),
              ],
            ),
          ),
    );
  }

  static Route<String> _alertWithTitleDialog(BuildContext context, Object? arguments) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return CupertinoDialogRoute<String>(
      context: context,
      builder:
          (BuildContext context) => ApplyTextOptions(
            child: CupertinoAlertDialog(
              title: Text(localizations.cupertinoAlertLocationTitle),
              content: Text(localizations.cupertinoAlertLocationDescription),
              actions: <Widget>[
                CupertinoDialogAction(
                  onPressed: () => Navigator.of(context).pop(localizations.cupertinoAlertDontAllow),
                  child: Text(localizations.cupertinoAlertDontAllow),
                ),
                CupertinoDialogAction(
                  onPressed: () => Navigator.of(context).pop(localizations.cupertinoAlertAllow),
                  child: Text(localizations.cupertinoAlertAllow),
                ),
              ],
            ),
          ),
    );
  }

  static Route<String> _alertWithButtonsDialog(BuildContext context, Object? arguments) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return CupertinoDialogRoute<String>(
      context: context,
      builder:
          (BuildContext context) => ApplyTextOptions(
            child: CupertinoDessertDialog(
              title: Text(localizations.cupertinoAlertFavoriteDessert),
              content: Text(localizations.cupertinoAlertDessertDescription),
            ),
          ),
    );
  }

  static Route<String> _alertWithButtonsOnlyDialog(BuildContext context, Object? arguments) {
    return CupertinoDialogRoute<String>(
      context: context,
      builder: (BuildContext context) => const ApplyTextOptions(child: CupertinoDessertDialog()),
    );
  }

  static Route<String> _modalRoute(BuildContext context, Object? arguments) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return CupertinoModalPopupRoute<String>(
      builder:
          (BuildContext context) => ApplyTextOptions(
            child: CupertinoActionSheet(
              title: Text(localizations.cupertinoAlertFavoriteDessert),
              message: Text(localizations.cupertinoAlertDessertDescription),
              actions: <Widget>[
                CupertinoActionSheetAction(
                  onPressed:
                      () => Navigator.of(context).pop(localizations.cupertinoAlertCheesecake),
                  child: Text(localizations.cupertinoAlertCheesecake),
                ),
                CupertinoActionSheetAction(
                  onPressed: () => Navigator.of(context).pop(localizations.cupertinoAlertTiramisu),
                  child: Text(localizations.cupertinoAlertTiramisu),
                ),
                CupertinoActionSheetAction(
                  onPressed: () => Navigator.of(context).pop(localizations.cupertinoAlertApplePie),
                  child: Text(localizations.cupertinoAlertApplePie),
                ),
              ],
              cancelButton: CupertinoActionSheetAction(
                isDefaultAction: true,
                onPressed: () => Navigator.of(context).pop(localizations.cupertinoAlertCancel),
                child: Text(localizations.cupertinoAlertCancel),
              ),
            ),
          ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        automaticallyImplyLeading: false,
        middle: Text(_title(context)),
      ),
      child: Builder(
        builder: (BuildContext context) {
          final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
          final Widget showAlertButton = CupertinoButton.filled(
            onPressed:
                () =>
                    switch (widget.type) {
                      AlertDemoType.alert => _alertDialogRoute,
                      AlertDemoType.alertTitle => _alertWithTitleDialogRoute,
                      AlertDemoType.alertButtons => _alertWithButtonsDialogRoute,
                      AlertDemoType.alertButtonsOnly => _alertWithButtonsOnlyDialogRoute,
                      AlertDemoType.actionSheet => _modalPopupRoute,
                    }.present(),
            child: Text(localizations.cupertinoShowAlert),
          );

          return Column(
            children: <Widget>[
              Expanded(child: Center(child: showAlertButton)),
              if (lastSelectedValue.value != null)
                Padding(
                  padding: const EdgeInsets.all(16),
                  child: Text(
                    localizations.dialogSelectedOption(lastSelectedValue.value!),
                    style: CupertinoTheme.of(context).textTheme.textStyle,
                    textAlign: TextAlign.center,
                  ),
                ),
            ],
          );
        },
      ),
    );
  }
}

class CupertinoDessertDialog extends StatelessWidget {
  const CupertinoDessertDialog({super.key, this.title, this.content});

  final Widget? title;
  final Widget? content;

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return CupertinoAlertDialog(
      title: title,
      content: content,
      actions: <Widget>[
        CupertinoDialogAction(
          onPressed: () {
            Navigator.of(context).pop(localizations.cupertinoAlertCheesecake);
          },
          child: Text(localizations.cupertinoAlertCheesecake),
        ),
        CupertinoDialogAction(
          onPressed: () {
            Navigator.of(context).pop(localizations.cupertinoAlertTiramisu);
          },
          child: Text(localizations.cupertinoAlertTiramisu),
        ),
        CupertinoDialogAction(
          onPressed: () {
            Navigator.of(context).pop(localizations.cupertinoAlertApplePie);
          },
          child: Text(localizations.cupertinoAlertApplePie),
        ),
        CupertinoDialogAction(
          onPressed: () {
            Navigator.of(context).pop(localizations.cupertinoAlertChocolateBrownie);
          },
          child: Text(localizations.cupertinoAlertChocolateBrownie),
        ),
        CupertinoDialogAction(
          isDestructiveAction: true,
          onPressed: () {
            Navigator.of(context).pop(localizations.cupertinoAlertCancel);
          },
          child: Text(localizations.cupertinoAlertCancel),
        ),
      ],
    );
  }
}

// END
