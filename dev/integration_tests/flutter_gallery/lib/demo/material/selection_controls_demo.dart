// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

const String _checkboxText =
    'Checkboxes allow the user to select multiple options from a set. '
    "A normal checkbox's value is true or false and a tristate checkbox's "
    'value can also be null.';

const String _checkboxCode = 'selectioncontrols_checkbox';

const String _radioText =
    'Radio buttons allow the user to select one option from a set. Use radio '
    'buttons for exclusive selection if you think that the user needs to see '
    'all available options side-by-side.';

const String _radioCode = 'selectioncontrols_radio';

const String _switchText =
    'On/off switches toggle the state of a single settings option. The option '
    'that the switch controls, as well as the state it’s in, should be made '
    'clear from the corresponding inline label.';

const String _switchCode = 'selectioncontrols_switch';

class SelectionControlsDemo extends StatefulWidget {
  const SelectionControlsDemo({super.key});

  static const String routeName = '/material/selection-controls';

  @override
  State<SelectionControlsDemo> createState() => _SelectionControlsDemoState();
}

class _SelectionControlsDemoState extends State<SelectionControlsDemo> {
  @override
  Widget build(BuildContext context) {
    final List<ComponentDemoTabData> demos = <ComponentDemoTabData>[
      ComponentDemoTabData(
        tabName: 'CHECKBOX',
        description: _checkboxText,
        demoWidget: buildCheckbox(),
        exampleCodeTag: _checkboxCode,
        documentationUrl: 'https://api.flutter.dev/flutter/material/Checkbox-class.html',
      ),
      ComponentDemoTabData(
        tabName: 'RADIO',
        description: _radioText,
        demoWidget: buildRadio(),
        exampleCodeTag: _radioCode,
        documentationUrl: 'https://api.flutter.dev/flutter/material/Radio-class.html',
      ),
      ComponentDemoTabData(
        tabName: 'SWITCH',
        description: _switchText,
        demoWidget: buildSwitch(),
        exampleCodeTag: _switchCode,
        documentationUrl: 'https://api.flutter.dev/flutter/material/Switch-class.html',
      ),
    ];

    return TabbedComponentDemoScaffold(title: 'Selection controls', demos: demos);
  }

  bool? checkboxValueA = true;
  bool? checkboxValueB = false;
  bool? checkboxValueC;
  int? radioValue = 0;
  bool switchValue = false;

  void handleRadioValueChanged(int? value) {
    setState(() {
      radioValue = value;
    });
  }

  Widget buildCheckbox() {
    return Align(
      alignment: const Alignment(0.0, -0.2),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Row(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              Semantics(
                label: 'Checkbox A',
                child: Checkbox(
                  value: checkboxValueA,
                  onChanged: (bool? value) {
                    setState(() {
                      checkboxValueA = value;
                    });
                  },
                ),
              ),
              Semantics(
                label: 'Checkbox B',
                child: Checkbox(
                  value: checkboxValueB,
                  onChanged: (bool? value) {
                    setState(() {
                      checkboxValueB = value;
                    });
                  },
                ),
              ),
              Semantics(
                label: 'Checkbox C',
                child: Checkbox(
                  value: checkboxValueC,
                  tristate: true,
                  onChanged: (bool? value) {
                    setState(() {
                      checkboxValueC = value;
                    });
                  },
                ),
              ),
            ],
          ),
          const Row(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              // Disabled checkboxes
              Checkbox(value: true, onChanged: null),
              Checkbox(value: false, onChanged: null),
              Checkbox(value: null, tristate: true, onChanged: null),
            ],
          ),
        ],
      ),
    );
  }

  Widget buildRadio() {
    return Align(
      alignment: const Alignment(0.0, -0.2),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Row(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              Radio<int>(value: 0, groupValue: radioValue, onChanged: handleRadioValueChanged),
              Radio<int>(value: 1, groupValue: radioValue, onChanged: handleRadioValueChanged),
              Radio<int>(value: 2, groupValue: radioValue, onChanged: handleRadioValueChanged),
            ],
          ),
          // Disabled radio buttons
          const Row(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              Radio<int>(value: 0, groupValue: 0, onChanged: null),
              Radio<int>(value: 1, groupValue: 0, onChanged: null),
              Radio<int>(value: 2, groupValue: 0, onChanged: null),
            ],
          ),
        ],
      ),
    );
  }

  Widget buildSwitch() {
    return Align(
      alignment: const Alignment(0.0, -0.2),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Switch.adaptive(
            value: switchValue,
            onChanged: (bool value) {
              setState(() {
                switchValue = value;
              });
            },
          ),
          // Disabled switches
          const Switch.adaptive(value: true, onChanged: null),
          const Switch.adaptive(value: false, onChanged: null),
        ],
      ),
    );
  }
}
