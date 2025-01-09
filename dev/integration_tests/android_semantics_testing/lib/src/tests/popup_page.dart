// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'popup_constants.dart';

export 'popup_constants.dart';

/// A page with a popup menu, a dropdown menu, and a modal alert.
class PopupControlsPage extends StatefulWidget {
  const PopupControlsPage({super.key});

  @override
  State<StatefulWidget> createState() => _PopupControlsPageState();
}

class _PopupControlsPageState extends State<PopupControlsPage> {
  final Key popupKey = const ValueKey<String>(popupKeyValue);
  final Key dropdownKey = const ValueKey<String>(dropdownKeyValue);
  final Key alertKey = const ValueKey<String>(alertKeyValue);

  String popupValue = popupItems.first;
  String dropdownValue = popupItems.first;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(leading: const BackButton(key: ValueKey<String>('back'))),
      body: SafeArea(
        child: Center(
          child: Row(
            mainAxisAlignment: MainAxisAlignment.center,
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              PopupMenuButton<String>(
                key: const ValueKey<String>(popupButtonKeyValue),
                icon: const Icon(Icons.arrow_drop_down),
                itemBuilder: (BuildContext context) {
                  return popupItems.map<PopupMenuItem<String>>((String item) {
                    return PopupMenuItem<String>(
                      key: ValueKey<String>('$popupKeyValue.$item'),
                      value: item,
                      child: Text(item),
                    );
                  }).toList();
                },
                onSelected: (String value) {
                  popupValue = value;
                },
              ),
              DropdownButton<String>(
                key: const ValueKey<String>(dropdownButtonKeyValue),
                value: dropdownValue,
                items:
                    popupItems.map<DropdownMenuItem<String>>((String item) {
                      return DropdownMenuItem<String>(
                        key: ValueKey<String>('$dropdownKeyValue.$item'),
                        value: item,
                        child: Text(item),
                      );
                    }).toList(),
                onChanged: (String? value) {
                  setState(() {
                    dropdownValue = value!;
                  });
                },
              ),
              MaterialButton(
                key: const ValueKey<String>(alertButtonKeyValue),
                child: const Text('Alert'),
                onPressed: () {
                  showDialog<void>(
                    context: context,
                    barrierDismissible: false, // user must tap button!
                    builder: (BuildContext context) {
                      return AlertDialog(
                        key: const ValueKey<String>(alertKeyValue),
                        title: const Text(
                          'Title text',
                          key: ValueKey<String>('$alertKeyValue.Title'),
                        ),
                        content: const SingleChildScrollView(
                          child: ListBody(
                            children: <Widget>[
                              Text(
                                'Body text line 1.',
                                key: ValueKey<String>('$alertKeyValue.Body1'),
                              ),
                              Text(
                                'Body text line 2.',
                                key: ValueKey<String>('$alertKeyValue.Body2'),
                              ),
                            ],
                          ),
                        ),
                        actions: <Widget>[
                          TextButton(
                            child: const Text('OK', key: ValueKey<String>('$alertKeyValue.OK')),
                            onPressed: () {
                              Navigator.of(context).pop();
                            },
                          ),
                        ],
                      );
                    },
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );
  }
}
