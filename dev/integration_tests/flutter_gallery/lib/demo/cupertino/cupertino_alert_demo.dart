// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

import '../../gallery/demo.dart';

class CupertinoAlertDemo extends StatefulWidget {
  const CupertinoAlertDemo({super.key});

  static const String routeName = '/cupertino/alert';

  @override
  State<CupertinoAlertDemo> createState() => _CupertinoAlertDemoState();
}

class _CupertinoAlertDemoState extends State<CupertinoAlertDemo> {
  String? lastSelectedValue;

  void showDemoDialog({required BuildContext context, Widget? child}) {
    showCupertinoDialog<String>(context: context, builder: (BuildContext context) => child!).then((
      String? value,
    ) {
      if (value != null) {
        setState(() {
          lastSelectedValue = value;
        });
      }
    });
  }

  void showDemoActionSheet({required BuildContext context, Widget? child}) {
    showCupertinoModalPopup<String>(
      context: context,
      builder: (BuildContext context) => child!,
    ).then((String? value) {
      if (value != null) {
        setState(() {
          lastSelectedValue = value;
        });
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: CupertinoNavigationBar(
        middle: const Text('Alerts'),
        // We're specifying a back label here because the previous page is a
        // Material page. CupertinoPageRoutes could auto-populate these back
        // labels.
        previousPageTitle: 'Cupertino',
        trailing: CupertinoDemoDocumentationButton(CupertinoAlertDemo.routeName),
      ),
      child: DefaultTextStyle(
        style: CupertinoTheme.of(context).textTheme.textStyle,
        child: Builder(
          builder: (BuildContext context) {
            return Stack(
              alignment: Alignment.center,
              children: <Widget>[
                CupertinoScrollbar(
                  child: ListView(
                    primary: true,
                    // Add more padding to the normal safe area.
                    padding:
                        const EdgeInsets.symmetric(vertical: 24.0, horizontal: 72.0) +
                        MediaQuery.of(context).padding,
                    children: <Widget>[
                      CupertinoButton.filled(
                        child: const Text('Alert'),
                        onPressed: () => _onAlertPress(context),
                      ),
                      const Padding(padding: EdgeInsets.all(8.0)),
                      CupertinoButton.filled(
                        padding: const EdgeInsets.symmetric(vertical: 16.0, horizontal: 36.0),
                        child: const Text('Alert with Title'),
                        onPressed: () => _onAlertWithTitlePress(context),
                      ),
                      const Padding(padding: EdgeInsets.all(8.0)),
                      CupertinoButton.filled(
                        padding: const EdgeInsets.symmetric(vertical: 16.0, horizontal: 36.0),
                        child: const Text('Alert with Buttons'),
                        onPressed: () => _onAlertWithButtonsPress(context),
                      ),
                      const Padding(padding: EdgeInsets.all(8.0)),
                      CupertinoButton.filled(
                        padding: const EdgeInsets.symmetric(vertical: 16.0, horizontal: 36.0),
                        child: const Text('Alert Buttons Only'),
                        onPressed: () {
                          showDemoDialog(context: context, child: const CupertinoDessertDialog());
                        },
                      ),
                      const Padding(padding: EdgeInsets.all(8.0)),
                      CupertinoButton.filled(
                        padding: const EdgeInsets.symmetric(vertical: 16.0, horizontal: 36.0),
                        child: const Text('Action Sheet'),
                        onPressed: () => _onActionSheetPress(context),
                      ),
                    ],
                  ),
                ),
                if (lastSelectedValue != null)
                  Positioned(bottom: 32.0, child: Text('You selected: $lastSelectedValue')),
              ],
            );
          },
        ),
      ),
    );
  }

  void _onAlertPress(BuildContext context) {
    showDemoDialog(
      context: context,
      child: CupertinoAlertDialog(
        title: const Text('Discard draft?'),
        actions: <Widget>[
          CupertinoDialogAction(
            isDestructiveAction: true,
            child: const Text('Discard'),
            onPressed: () => Navigator.pop(context, 'Discard'),
          ),
          CupertinoDialogAction(
            isDefaultAction: true,
            child: const Text('Cancel'),
            onPressed: () => Navigator.pop(context, 'Cancel'),
          ),
        ],
      ),
    );
  }

  void _onAlertWithTitlePress(BuildContext context) {
    showDemoDialog(
      context: context,
      child: CupertinoAlertDialog(
        title: const Text('Allow "Maps" to access your location while you are using the app?'),
        content: const Text(
          'Your current location will be displayed on the map and used '
          'for directions, nearby search results, and estimated travel times.',
        ),
        actions: <Widget>[
          CupertinoDialogAction(
            child: const Text("Don't Allow"),
            onPressed: () => Navigator.pop(context, 'Disallow'),
          ),
          CupertinoDialogAction(
            child: const Text('Allow'),
            onPressed: () => Navigator.pop(context, 'Allow'),
          ),
        ],
      ),
    );
  }

  void _onAlertWithButtonsPress(BuildContext context) {
    showDemoDialog(
      context: context,
      child: const CupertinoDessertDialog(
        title: Text('Select Favorite Dessert'),
        content: Text(
          'Please select your favorite type of dessert from the '
          'list below. Your selection will be used to customize the suggested '
          'list of eateries in your area.',
        ),
      ),
    );
  }

  void _onActionSheetPress(BuildContext context) {
    showDemoActionSheet(
      context: context,
      child: CupertinoActionSheet(
        title: const Text('Favorite Dessert'),
        message: const Text('Please select the best dessert from the options below.'),
        actions: <Widget>[
          CupertinoActionSheetAction(
            child: const Text('Profiteroles'),
            onPressed: () => Navigator.pop(context, 'Profiteroles'),
          ),
          CupertinoActionSheetAction(
            child: const Text('Cannolis'),
            onPressed: () => Navigator.pop(context, 'Cannolis'),
          ),
          CupertinoActionSheetAction(
            child: const Text('Trifle'),
            onPressed: () => Navigator.pop(context, 'Trifle'),
          ),
        ],
        cancelButton: CupertinoActionSheetAction(
          isDefaultAction: true,
          child: const Text('Cancel'),
          onPressed: () => Navigator.pop(context, 'Cancel'),
        ),
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
    return CupertinoAlertDialog(
      title: title,
      content: content,
      actions: <Widget>[
        CupertinoDialogAction(
          child: const Text('Cheesecake'),
          onPressed: () {
            Navigator.pop(context, 'Cheesecake');
          },
        ),
        CupertinoDialogAction(
          child: const Text('Tiramisu'),
          onPressed: () {
            Navigator.pop(context, 'Tiramisu');
          },
        ),
        CupertinoDialogAction(
          child: const Text('Apple Pie'),
          onPressed: () {
            Navigator.pop(context, 'Apple Pie');
          },
        ),
        CupertinoDialogAction(
          child: const Text("Devil's food cake"),
          onPressed: () {
            Navigator.pop(context, "Devil's food cake");
          },
        ),
        CupertinoDialogAction(
          child: const Text('Banana Split'),
          onPressed: () {
            Navigator.pop(context, 'Banana Split');
          },
        ),
        CupertinoDialogAction(
          child: const Text('Oatmeal Cookie'),
          onPressed: () {
            Navigator.pop(context, 'Oatmeal Cookies');
          },
        ),
        CupertinoDialogAction(
          child: const Text('Chocolate Brownie'),
          onPressed: () {
            Navigator.pop(context, 'Chocolate Brownies');
          },
        ),
        CupertinoDialogAction(
          isDestructiveAction: true,
          child: const Text('Cancel'),
          onPressed: () {
            Navigator.pop(context, 'Cancel');
          },
        ),
      ],
    );
  }
}
