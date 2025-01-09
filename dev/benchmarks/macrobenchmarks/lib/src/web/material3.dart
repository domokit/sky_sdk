// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

const SizedBox rowDivider = SizedBox(width: 20);
const SizedBox colDivider = SizedBox(height: 10);
const double smallSpacing = 10.0;
const double cardWidth = 115;
const double widthConstraint = 450;
final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

class SingleColumnMaterial3Components extends StatelessWidget {
  const SingleColumnMaterial3Components({super.key, this.scrollController});

  final ScrollController? scrollController;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        key: scaffoldKey,
        body: ListView(
          controller: scrollController,
          children: <Widget>[
            const Actions(),
            colDivider,
            const Communication(),
            colDivider,
            const Containment(),
            colDivider,
            Navigation(scaffoldKey: scaffoldKey),
            colDivider,
            const Selection(),
            colDivider,
            const TextInputs(),
            colDivider,
            Navigation(scaffoldKey: scaffoldKey),
            colDivider,
            const Selection(),
            colDivider,
            const TextInputs(),
          ],
        ),
      ),
    );
  }
}

class TwoColumnMaterial3Components extends StatefulWidget {
  const TwoColumnMaterial3Components({super.key});

  @override
  State<TwoColumnMaterial3Components> createState() => _TwoColumnMaterial3ComponentsState();
}

class _TwoColumnMaterial3ComponentsState extends State<TwoColumnMaterial3Components> {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        key: scaffoldKey,
        body: Row(
          children: <Widget>[
            Expanded(
              child: FirstComponentList(
                showNavBottomBar: true,
                scaffoldKey: scaffoldKey,
                showSecondList: true,
              ),
            ),
            Expanded(child: SecondComponentList(scaffoldKey: scaffoldKey)),
          ],
        ),
      ),
    );
  }
}

class FirstComponentList extends StatelessWidget {
  const FirstComponentList({
    super.key,
    required this.showNavBottomBar,
    required this.scaffoldKey,
    required this.showSecondList,
  });

  final bool showNavBottomBar;
  final GlobalKey<ScaffoldState> scaffoldKey;
  final bool showSecondList;

  @override
  Widget build(BuildContext context) {
    // Fully traverse this list before moving on.
    return FocusTraversalGroup(
      child: ListView(
        padding:
            showSecondList ? const EdgeInsetsDirectional.only(end: smallSpacing) : EdgeInsets.zero,
        children: <Widget>[
          const Actions(),
          colDivider,
          const Communication(),
          colDivider,
          const Containment(),
          if (!showSecondList) ...<Widget>[
            colDivider,
            Navigation(scaffoldKey: scaffoldKey),
            colDivider,
            const Selection(),
            colDivider,
            const TextInputs(),
          ],
        ],
      ),
    );
  }
}

class SecondComponentList extends StatelessWidget {
  const SecondComponentList({super.key, required this.scaffoldKey});

  final GlobalKey<ScaffoldState> scaffoldKey;

  @override
  Widget build(BuildContext context) {
    // Fully traverse this list before moving on.
    return FocusTraversalGroup(
      child: ListView(
        padding: const EdgeInsetsDirectional.only(end: smallSpacing),
        children: <Widget>[
          Navigation(scaffoldKey: scaffoldKey),
          colDivider,
          const Selection(),
          colDivider,
          const TextInputs(),
        ],
      ),
    );
  }
}

class Actions extends StatelessWidget {
  const Actions({super.key});

  @override
  Widget build(BuildContext context) {
    return const ComponentGroupDecoration(
      label: 'Actions',
      children: <Widget>[
        Buttons(),
        FloatingActionButtons(),
        IconToggleButtons(),
        SegmentedButtons(),
      ],
    );
  }
}

class Communication extends StatelessWidget {
  const Communication({super.key});

  @override
  Widget build(BuildContext context) {
    return const ComponentGroupDecoration(
      label: 'Communication',
      children: <Widget>[
        NavigationBars(selectedIndex: 1, isExampleBar: true, isBadgeExample: true),
        ProgressIndicators(),
        SnackBarSection(),
      ],
    );
  }
}

class Containment extends StatelessWidget {
  const Containment({super.key});

  @override
  Widget build(BuildContext context) {
    return const ComponentGroupDecoration(
      label: 'Containment',
      children: <Widget>[BottomSheetSection(), Cards(), Dialogs(), Dividers()],
    );
  }
}

class Navigation extends StatelessWidget {
  const Navigation({super.key, required this.scaffoldKey});

  final GlobalKey<ScaffoldState> scaffoldKey;

  @override
  Widget build(BuildContext context) {
    return ComponentGroupDecoration(
      label: 'Navigation',
      children: <Widget>[
        const BottomAppBars(),
        const NavigationBars(selectedIndex: 0, isExampleBar: true),
        NavigationDrawers(scaffoldKey: scaffoldKey),
        const NavigationRails(),
        const Tabs(),
        const TopAppBars(),
      ],
    );
  }
}

class Selection extends StatelessWidget {
  const Selection({super.key});

  @override
  Widget build(BuildContext context) {
    return const ComponentGroupDecoration(
      label: 'Selection',
      children: <Widget>[Checkboxes(), Chips(), Menus(), Radios(), Sliders(), Switches()],
    );
  }
}

class TextInputs extends StatelessWidget {
  const TextInputs({super.key});

  @override
  Widget build(BuildContext context) {
    return const ComponentGroupDecoration(label: 'Text inputs', children: <Widget>[TextFields()]);
  }
}

class Buttons extends StatefulWidget {
  const Buttons({super.key});

  @override
  State<Buttons> createState() => _ButtonsState();
}

class _ButtonsState extends State<Buttons> {
  @override
  Widget build(BuildContext context) {
    return const ComponentDecoration(
      label: 'Common buttons',
      tooltipMessage:
          'Use ElevatedButton, FilledButton, FilledButton.tonal, OutlinedButton, or TextButton',
      child: SingleChildScrollView(
        scrollDirection: Axis.horizontal,
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          children: <Widget>[
            ButtonsWithoutIcon(isDisabled: false),
            ButtonsWithIcon(),
            ButtonsWithoutIcon(isDisabled: true),
          ],
        ),
      ),
    );
  }
}

class ButtonsWithoutIcon extends StatelessWidget {
  const ButtonsWithoutIcon({super.key, required this.isDisabled});

  final bool isDisabled;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 5.0),
      child: IntrinsicWidth(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: <Widget>[
            ElevatedButton(onPressed: isDisabled ? null : () {}, child: const Text('Elevated')),
            colDivider,
            FilledButton(onPressed: isDisabled ? null : () {}, child: const Text('Filled')),
            colDivider,
            FilledButton.tonal(
              onPressed: isDisabled ? null : () {},
              child: const Text('Filled tonal'),
            ),
            colDivider,
            OutlinedButton(onPressed: isDisabled ? null : () {}, child: const Text('Outlined')),
            colDivider,
            TextButton(onPressed: isDisabled ? null : () {}, child: const Text('Text')),
          ],
        ),
      ),
    );
  }
}

class ButtonsWithIcon extends StatelessWidget {
  const ButtonsWithIcon({super.key});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 10.0),
      child: IntrinsicWidth(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: <Widget>[
            ElevatedButton.icon(
              onPressed: () {},
              icon: const Icon(Icons.add),
              label: const Text('Icon'),
            ),
            colDivider,
            FilledButton.icon(
              onPressed: () {},
              label: const Text('Icon'),
              icon: const Icon(Icons.add),
            ),
            colDivider,
            FilledButton.tonalIcon(
              onPressed: () {},
              label: const Text('Icon'),
              icon: const Icon(Icons.add),
            ),
            colDivider,
            OutlinedButton.icon(
              onPressed: () {},
              icon: const Icon(Icons.add),
              label: const Text('Icon'),
            ),
            colDivider,
            TextButton.icon(
              onPressed: () {},
              icon: const Icon(Icons.add),
              label: const Text('Icon'),
            ),
          ],
        ),
      ),
    );
  }
}

class FloatingActionButtons extends StatelessWidget {
  const FloatingActionButtons({super.key});

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Floating action buttons',
      tooltipMessage: 'Use FloatingActionButton or FloatingActionButton.extended',
      child: Wrap(
        crossAxisAlignment: WrapCrossAlignment.center,
        runSpacing: smallSpacing,
        spacing: smallSpacing,
        children: <Widget>[
          FloatingActionButton.small(
            onPressed: () {},
            tooltip: 'Small',
            child: const Icon(Icons.add),
          ),
          FloatingActionButton.extended(
            onPressed: () {},
            tooltip: 'Extended',
            icon: const Icon(Icons.add),
            label: const Text('Create'),
          ),
          FloatingActionButton(onPressed: () {}, tooltip: 'Standard', child: const Icon(Icons.add)),
          FloatingActionButton.large(
            onPressed: () {},
            tooltip: 'Large',
            child: const Icon(Icons.add),
          ),
        ],
      ),
    );
  }
}

class Cards extends StatelessWidget {
  const Cards({super.key});

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Cards',
      tooltipMessage: 'Use Card',
      child: Wrap(
        alignment: WrapAlignment.spaceEvenly,
        children: <Widget>[
          SizedBox(
            width: cardWidth,
            child: Card(
              child: Container(
                padding: const EdgeInsets.fromLTRB(10, 5, 5, 10),
                child: Column(
                  children: <Widget>[
                    Align(
                      alignment: Alignment.topRight,
                      child: IconButton(icon: const Icon(Icons.more_vert), onPressed: () {}),
                    ),
                    const SizedBox(height: 20),
                    const Align(alignment: Alignment.bottomLeft, child: Text('Elevated')),
                  ],
                ),
              ),
            ),
          ),
          SizedBox(
            width: cardWidth,
            child: Card(
              color: Theme.of(context).colorScheme.surfaceContainerHighest,
              elevation: 0,
              child: Container(
                padding: const EdgeInsets.fromLTRB(10, 5, 5, 10),
                child: Column(
                  children: <Widget>[
                    Align(
                      alignment: Alignment.topRight,
                      child: IconButton(icon: const Icon(Icons.more_vert), onPressed: () {}),
                    ),
                    const SizedBox(height: 20),
                    const Align(alignment: Alignment.bottomLeft, child: Text('Filled')),
                  ],
                ),
              ),
            ),
          ),
          SizedBox(
            width: cardWidth,
            child: Card(
              elevation: 0,
              shape: RoundedRectangleBorder(
                side: BorderSide(color: Theme.of(context).colorScheme.outline),
                borderRadius: const BorderRadius.all(Radius.circular(12)),
              ),
              child: Container(
                padding: const EdgeInsets.fromLTRB(10, 5, 5, 10),
                child: Column(
                  children: <Widget>[
                    Align(
                      alignment: Alignment.topRight,
                      child: IconButton(icon: const Icon(Icons.more_vert), onPressed: () {}),
                    ),
                    const SizedBox(height: 20),
                    const Align(alignment: Alignment.bottomLeft, child: Text('Outlined')),
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _ClearButton extends StatelessWidget {
  const _ClearButton({required this.controller});

  final TextEditingController controller;

  @override
  Widget build(BuildContext context) =>
      IconButton(icon: const Icon(Icons.clear), onPressed: () => controller.clear());
}

class TextFields extends StatefulWidget {
  const TextFields({super.key});

  @override
  State<TextFields> createState() => _TextFieldsState();
}

class _TextFieldsState extends State<TextFields> {
  final TextEditingController _controllerFilled = TextEditingController();
  final TextEditingController _controllerOutlined = TextEditingController();

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Text fields',
      tooltipMessage: 'Use TextField with different InputDecoration',
      child: Column(
        children: <Widget>[
          Padding(
            padding: const EdgeInsets.all(smallSpacing),
            child: TextField(
              controller: _controllerFilled,
              decoration: InputDecoration(
                prefixIcon: const Icon(Icons.search),
                suffixIcon: _ClearButton(controller: _controllerFilled),
                labelText: 'Filled',
                hintText: 'hint text',
                helperText: 'supporting text',
                filled: true,
              ),
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(smallSpacing),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: <Widget>[
                Flexible(
                  child: SizedBox(
                    width: 200,
                    child: TextField(
                      maxLength: 10,
                      maxLengthEnforcement: MaxLengthEnforcement.none,
                      controller: _controllerFilled,
                      decoration: InputDecoration(
                        prefixIcon: const Icon(Icons.search),
                        suffixIcon: _ClearButton(controller: _controllerFilled),
                        labelText: 'Filled',
                        hintText: 'hint text',
                        helperText: 'supporting text',
                        filled: true,
                        errorText: 'error text',
                      ),
                    ),
                  ),
                ),
                const SizedBox(width: smallSpacing),
                Flexible(
                  child: SizedBox(
                    width: 200,
                    child: TextField(
                      controller: _controllerFilled,
                      enabled: false,
                      decoration: InputDecoration(
                        prefixIcon: const Icon(Icons.search),
                        suffixIcon: _ClearButton(controller: _controllerFilled),
                        labelText: 'Disabled',
                        hintText: 'hint text',
                        helperText: 'supporting text',
                        filled: true,
                      ),
                    ),
                  ),
                ),
              ],
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(smallSpacing),
            child: TextField(
              controller: _controllerOutlined,
              decoration: InputDecoration(
                prefixIcon: const Icon(Icons.search),
                suffixIcon: _ClearButton(controller: _controllerOutlined),
                labelText: 'Outlined',
                hintText: 'hint text',
                helperText: 'supporting text',
                border: const OutlineInputBorder(),
              ),
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(smallSpacing),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: <Widget>[
                Flexible(
                  child: SizedBox(
                    width: 200,
                    child: TextField(
                      controller: _controllerOutlined,
                      decoration: InputDecoration(
                        prefixIcon: const Icon(Icons.search),
                        suffixIcon: _ClearButton(controller: _controllerOutlined),
                        labelText: 'Outlined',
                        hintText: 'hint text',
                        helperText: 'supporting text',
                        errorText: 'error text',
                        border: const OutlineInputBorder(),
                        filled: true,
                      ),
                    ),
                  ),
                ),
                const SizedBox(width: smallSpacing),
                Flexible(
                  child: SizedBox(
                    width: 200,
                    child: TextField(
                      controller: _controllerOutlined,
                      enabled: false,
                      decoration: InputDecoration(
                        prefixIcon: const Icon(Icons.search),
                        suffixIcon: _ClearButton(controller: _controllerOutlined),
                        labelText: 'Disabled',
                        hintText: 'hint text',
                        helperText: 'supporting text',
                        border: const OutlineInputBorder(),
                        filled: true,
                      ),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class Dialogs extends StatefulWidget {
  const Dialogs({super.key});

  @override
  State<Dialogs> createState() => _DialogsState();
}

class _DialogsState extends State<Dialogs> {
  void openDialog(BuildContext context) {
    showDialog<void>(
      context: context,
      builder:
          (BuildContext context) => AlertDialog(
            title: const Text('What is a dialog?'),
            content: const Text(
              'A dialog is a type of modal window that appears in front of app content to provide critical information, or prompt for a decision to be made.',
            ),
            actions: <Widget>[
              TextButton(child: const Text('Okay'), onPressed: () => Navigator.of(context).pop()),
              FilledButton(
                child: const Text('Dismiss'),
                onPressed: () => Navigator.of(context).pop(),
              ),
            ],
          ),
    );
  }

  void openFullscreenDialog(BuildContext context) {
    showDialog<void>(
      context: context,
      builder:
          (BuildContext context) => Dialog.fullscreen(
            child: Padding(
              padding: const EdgeInsets.all(20.0),
              child: Scaffold(
                appBar: AppBar(
                  title: const Text('Full-screen dialog'),
                  centerTitle: false,
                  leading: IconButton(
                    icon: const Icon(Icons.close),
                    onPressed: () => Navigator.of(context).pop(),
                  ),
                  actions: <Widget>[
                    TextButton(
                      child: const Text('Close'),
                      onPressed: () => Navigator.of(context).pop(),
                    ),
                  ],
                ),
              ),
            ),
          ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Dialog',
      tooltipMessage: 'Use showDialog with Dialog.fullscreen, AlertDialog, or SimpleDialog',
      child: Wrap(
        alignment: WrapAlignment.spaceBetween,
        children: <Widget>[
          TextButton(
            child: const Text('Show dialog', style: TextStyle(fontWeight: FontWeight.bold)),
            onPressed: () => openDialog(context),
          ),
          TextButton(
            child: const Text(
              'Show full-screen dialog',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
            onPressed: () => openFullscreenDialog(context),
          ),
        ],
      ),
    );
  }
}

class Dividers extends StatelessWidget {
  const Dividers({super.key});

  @override
  Widget build(BuildContext context) {
    return const ComponentDecoration(
      label: 'Dividers',
      tooltipMessage: 'Use Divider or VerticalDivider',
      child: Column(children: <Widget>[Divider(key: Key('divider'))]),
    );
  }
}

class Switches extends StatelessWidget {
  const Switches({super.key});

  @override
  Widget build(BuildContext context) {
    return const ComponentDecoration(
      label: 'Switches',
      tooltipMessage: 'Use SwitchListTile or Switch',
      child: Column(children: <Widget>[SwitchRow(isEnabled: true), SwitchRow(isEnabled: false)]),
    );
  }
}

class SwitchRow extends StatefulWidget {
  const SwitchRow({super.key, required this.isEnabled});

  final bool isEnabled;

  @override
  State<SwitchRow> createState() => _SwitchRowState();
}

class _SwitchRowState extends State<SwitchRow> {
  bool value0 = false;
  bool value1 = true;

  final MaterialStateProperty<Icon?> thumbIcon = MaterialStateProperty.resolveWith<Icon?>((
    Set<MaterialState> states,
  ) {
    if (states.contains(MaterialState.selected)) {
      return const Icon(Icons.check);
    }
    return const Icon(Icons.close);
  });

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: <Widget>[
        Switch(
          value: value0,
          onChanged:
              widget.isEnabled
                  ? (bool value) {
                    setState(() {
                      value0 = value;
                    });
                  }
                  : null,
        ),
        Switch(
          thumbIcon: thumbIcon,
          value: value1,
          onChanged:
              widget.isEnabled
                  ? (bool value) {
                    setState(() {
                      value1 = value;
                    });
                  }
                  : null,
        ),
      ],
    );
  }
}

class Checkboxes extends StatefulWidget {
  const Checkboxes({super.key});

  @override
  State<Checkboxes> createState() => _CheckboxesState();
}

class _CheckboxesState extends State<Checkboxes> {
  bool? isChecked0 = true;
  bool? isChecked1;
  bool? isChecked2 = false;

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Checkboxes',
      tooltipMessage: 'Use CheckboxListTile or Checkbox',
      child: Column(
        children: <Widget>[
          CheckboxListTile(
            tristate: true,
            value: isChecked0,
            title: const Text('Option 1'),
            onChanged: (bool? value) {
              setState(() {
                isChecked0 = value;
              });
            },
          ),
          CheckboxListTile(
            tristate: true,
            value: isChecked1,
            title: const Text('Option 2'),
            onChanged: (bool? value) {
              setState(() {
                isChecked1 = value;
              });
            },
          ),
          CheckboxListTile(
            tristate: true,
            value: isChecked2,
            title: const Text('Option 3'),
            onChanged: (bool? value) {
              setState(() {
                isChecked2 = value;
              });
            },
          ),
          const CheckboxListTile(
            tristate: true,
            title: Text('Option 4'),
            value: true,
            onChanged: null,
          ),
        ],
      ),
    );
  }
}

enum Value { first, second }

class Radios extends StatefulWidget {
  const Radios({super.key});

  @override
  State<Radios> createState() => _RadiosState();
}

enum Options { option1, option2, option3 }

class _RadiosState extends State<Radios> {
  Options? _selectedOption = Options.option1;

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Radio buttons',
      tooltipMessage: 'Use RadioListTile<T> or Radio<T>',
      child: Column(
        children: <Widget>[
          RadioListTile<Options>(
            title: const Text('Option 1'),
            value: Options.option1,
            groupValue: _selectedOption,
            onChanged: (Options? value) {
              setState(() {
                _selectedOption = value;
              });
            },
          ),
          RadioListTile<Options>(
            title: const Text('Option 2'),
            value: Options.option2,
            groupValue: _selectedOption,
            onChanged: (Options? value) {
              setState(() {
                _selectedOption = value;
              });
            },
          ),
          RadioListTile<Options>(
            title: const Text('Option 3'),
            value: Options.option3,
            groupValue: _selectedOption,
            onChanged: null,
          ),
        ],
      ),
    );
  }
}

class ProgressIndicators extends StatefulWidget {
  const ProgressIndicators({super.key});

  @override
  State<ProgressIndicators> createState() => _ProgressIndicatorsState();
}

class _ProgressIndicatorsState extends State<ProgressIndicators> {
  bool playProgressIndicator = false;

  @override
  Widget build(BuildContext context) {
    final double? progressValue = playProgressIndicator ? null : 0.7;

    return ComponentDecoration(
      label: 'Progress indicators',
      tooltipMessage: 'Use CircularProgressIndicator or LinearProgressIndicator',
      child: Column(
        children: <Widget>[
          Row(
            children: <Widget>[
              IconButton(
                isSelected: playProgressIndicator,
                selectedIcon: const Icon(Icons.pause),
                icon: const Icon(Icons.play_arrow),
                onPressed: () {
                  setState(() {
                    playProgressIndicator = !playProgressIndicator;
                  });
                },
              ),
              Expanded(
                child: Row(
                  children: <Widget>[
                    rowDivider,
                    CircularProgressIndicator(value: progressValue),
                    rowDivider,
                    Expanded(child: LinearProgressIndicator(value: progressValue)),
                    rowDivider,
                  ],
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}

const List<NavigationDestination> appBarDestinations = <NavigationDestination>[
  NavigationDestination(
    tooltip: '',
    icon: Icon(Icons.widgets_outlined),
    label: 'Components',
    selectedIcon: Icon(Icons.widgets),
  ),
  NavigationDestination(
    tooltip: '',
    icon: Icon(Icons.format_paint_outlined),
    label: 'Color',
    selectedIcon: Icon(Icons.format_paint),
  ),
  NavigationDestination(
    tooltip: '',
    icon: Icon(Icons.text_snippet_outlined),
    label: 'Typography',
    selectedIcon: Icon(Icons.text_snippet),
  ),
  NavigationDestination(
    tooltip: '',
    icon: Icon(Icons.invert_colors_on_outlined),
    label: 'Elevation',
    selectedIcon: Icon(Icons.opacity),
  ),
];

const List<Widget> exampleBarDestinations = <Widget>[
  NavigationDestination(
    tooltip: '',
    icon: Icon(Icons.explore_outlined),
    label: 'Explore',
    selectedIcon: Icon(Icons.explore),
  ),
  NavigationDestination(
    tooltip: '',
    icon: Icon(Icons.pets_outlined),
    label: 'Pets',
    selectedIcon: Icon(Icons.pets),
  ),
  NavigationDestination(
    tooltip: '',
    icon: Icon(Icons.account_box_outlined),
    label: 'Account',
    selectedIcon: Icon(Icons.account_box),
  ),
];

List<Widget> barWithBadgeDestinations = <Widget>[
  NavigationDestination(
    tooltip: '',
    icon: Badge.count(count: 1000, child: const Icon(Icons.mail_outlined)),
    label: 'Mail',
    selectedIcon: Badge.count(count: 1000, child: const Icon(Icons.mail)),
  ),
  const NavigationDestination(
    tooltip: '',
    icon: Badge(label: Text('10'), child: Icon(Icons.chat_bubble_outline)),
    label: 'Chat',
    selectedIcon: Badge(label: Text('10'), child: Icon(Icons.chat_bubble)),
  ),
  const NavigationDestination(
    tooltip: '',
    icon: Badge(child: Icon(Icons.group_outlined)),
    label: 'Rooms',
    selectedIcon: Badge(child: Icon(Icons.group_rounded)),
  ),
  NavigationDestination(
    tooltip: '',
    icon: Badge.count(count: 3, child: const Icon(Icons.videocam_outlined)),
    label: 'Meet',
    selectedIcon: Badge.count(count: 3, child: const Icon(Icons.videocam)),
  ),
];

class NavigationBars extends StatefulWidget {
  const NavigationBars({
    super.key,
    this.onSelectItem,
    required this.selectedIndex,
    required this.isExampleBar,
    this.isBadgeExample = false,
  });

  final void Function(int)? onSelectItem;
  final int selectedIndex;
  final bool isExampleBar;
  final bool isBadgeExample;

  @override
  State<NavigationBars> createState() => _NavigationBarsState();
}

class _NavigationBarsState extends State<NavigationBars> {
  late int selectedIndex;

  @override
  void initState() {
    super.initState();
    selectedIndex = widget.selectedIndex;
  }

  @override
  void didUpdateWidget(covariant NavigationBars oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.selectedIndex != oldWidget.selectedIndex) {
      selectedIndex = widget.selectedIndex;
    }
  }

  @override
  Widget build(BuildContext context) {
    // App NavigationBar should get first focus.
    Widget navigationBar = Focus(
      autofocus: !(widget.isExampleBar || widget.isBadgeExample),
      child: NavigationBar(
        selectedIndex: selectedIndex,
        onDestinationSelected: (int index) {
          setState(() {
            selectedIndex = index;
          });
          if (!widget.isExampleBar) {
            widget.onSelectItem!(index);
          }
        },
        destinations:
            widget.isExampleBar && widget.isBadgeExample
                ? barWithBadgeDestinations
                : widget.isExampleBar
                ? exampleBarDestinations
                : appBarDestinations,
      ),
    );

    if (widget.isExampleBar && widget.isBadgeExample) {
      navigationBar = ComponentDecoration(
        label: 'Badges',
        tooltipMessage: 'Use Badge or Badge.count',
        child: navigationBar,
      );
    } else if (widget.isExampleBar) {
      navigationBar = ComponentDecoration(
        label: 'Navigation bar',
        tooltipMessage: 'Use NavigationBar',
        child: navigationBar,
      );
    }

    return navigationBar;
  }
}

class IconToggleButtons extends StatefulWidget {
  const IconToggleButtons({super.key});

  @override
  State<IconToggleButtons> createState() => _IconToggleButtonsState();
}

class _IconToggleButtonsState extends State<IconToggleButtons> {
  @override
  Widget build(BuildContext context) {
    return const ComponentDecoration(
      label: 'Icon buttons',
      tooltipMessage: 'Use IconButton',
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: <Widget>[
          Column(
            // Standard IconButton
            children: <Widget>[
              IconToggleButton(isEnabled: true, tooltip: 'Standard'),
              colDivider,
              IconToggleButton(isEnabled: false, tooltip: 'Standard (disabled)'),
            ],
          ),
          Column(
            children: <Widget>[
              // Filled IconButton
              IconToggleButton(
                isEnabled: true,
                tooltip: 'Filled',
                getDefaultStyle: enabledFilledButtonStyle,
              ),
              colDivider,
              IconToggleButton(
                isEnabled: false,
                tooltip: 'Filled (disabled)',
                getDefaultStyle: disabledFilledButtonStyle,
              ),
            ],
          ),
          Column(
            children: <Widget>[
              // Filled Tonal IconButton
              IconToggleButton(
                isEnabled: true,
                tooltip: 'Filled tonal',
                getDefaultStyle: enabledFilledTonalButtonStyle,
              ),
              colDivider,
              IconToggleButton(
                isEnabled: false,
                tooltip: 'Filled tonal (disabled)',
                getDefaultStyle: disabledFilledTonalButtonStyle,
              ),
            ],
          ),
          Column(
            children: <Widget>[
              // Outlined IconButton
              IconToggleButton(
                isEnabled: true,
                tooltip: 'Outlined',
                getDefaultStyle: enabledOutlinedButtonStyle,
              ),
              colDivider,
              IconToggleButton(
                isEnabled: false,
                tooltip: 'Outlined (disabled)',
                getDefaultStyle: disabledOutlinedButtonStyle,
              ),
            ],
          ),
        ],
      ),
    );
  }
}

class IconToggleButton extends StatefulWidget {
  const IconToggleButton({
    required this.isEnabled,
    required this.tooltip,
    this.getDefaultStyle,
    super.key,
  });

  final bool isEnabled;
  final String tooltip;
  final ButtonStyle? Function(bool, ColorScheme)? getDefaultStyle;

  @override
  State<IconToggleButton> createState() => _IconToggleButtonState();
}

class _IconToggleButtonState extends State<IconToggleButton> {
  bool selected = false;

  @override
  Widget build(BuildContext context) {
    final ColorScheme colors = Theme.of(context).colorScheme;
    final VoidCallback? onPressed =
        widget.isEnabled
            ? () {
              setState(() {
                selected = !selected;
              });
            }
            : null;
    final ButtonStyle? style = widget.getDefaultStyle?.call(selected, colors);

    return IconButton(
      visualDensity: VisualDensity.standard,
      isSelected: selected,
      tooltip: widget.tooltip,
      icon: const Icon(Icons.settings_outlined),
      selectedIcon: const Icon(Icons.settings),
      onPressed: onPressed,
      style: style,
    );
  }
}

ButtonStyle enabledFilledButtonStyle(bool selected, ColorScheme colors) {
  return IconButton.styleFrom(
    foregroundColor: selected ? colors.onPrimary : colors.primary,
    backgroundColor: selected ? colors.primary : colors.surfaceContainerHighest,
    disabledForegroundColor: colors.onSurface.withOpacity(0.38),
    disabledBackgroundColor: colors.onSurface.withOpacity(0.12),
    hoverColor: selected ? colors.onPrimary.withOpacity(0.08) : colors.primary.withOpacity(0.08),
    focusColor: selected ? colors.onPrimary.withOpacity(0.12) : colors.primary.withOpacity(0.12),
    highlightColor:
        selected ? colors.onPrimary.withOpacity(0.12) : colors.primary.withOpacity(0.12),
  );
}

ButtonStyle disabledFilledButtonStyle(bool selected, ColorScheme colors) {
  return IconButton.styleFrom(
    disabledForegroundColor: colors.onSurface.withOpacity(0.38),
    disabledBackgroundColor: colors.onSurface.withOpacity(0.12),
  );
}

ButtonStyle enabledFilledTonalButtonStyle(bool selected, ColorScheme colors) {
  return IconButton.styleFrom(
    foregroundColor: selected ? colors.onSecondaryContainer : colors.onSurfaceVariant,
    backgroundColor: selected ? colors.secondaryContainer : colors.surfaceContainerHighest,
    hoverColor:
        selected
            ? colors.onSecondaryContainer.withOpacity(0.08)
            : colors.onSurfaceVariant.withOpacity(0.08),
    focusColor:
        selected
            ? colors.onSecondaryContainer.withOpacity(0.12)
            : colors.onSurfaceVariant.withOpacity(0.12),
    highlightColor:
        selected
            ? colors.onSecondaryContainer.withOpacity(0.12)
            : colors.onSurfaceVariant.withOpacity(0.12),
  );
}

ButtonStyle disabledFilledTonalButtonStyle(bool selected, ColorScheme colors) {
  return IconButton.styleFrom(
    disabledForegroundColor: colors.onSurface.withOpacity(0.38),
    disabledBackgroundColor: colors.onSurface.withOpacity(0.12),
  );
}

ButtonStyle enabledOutlinedButtonStyle(bool selected, ColorScheme colors) {
  return IconButton.styleFrom(
    backgroundColor: selected ? colors.inverseSurface : null,
    hoverColor:
        selected
            ? colors.onInverseSurface.withOpacity(0.08)
            : colors.onSurfaceVariant.withOpacity(0.08),
    focusColor:
        selected
            ? colors.onInverseSurface.withOpacity(0.12)
            : colors.onSurfaceVariant.withOpacity(0.12),
    highlightColor:
        selected ? colors.onInverseSurface.withOpacity(0.12) : colors.onSurface.withOpacity(0.12),
    side: BorderSide(color: colors.outline),
  ).copyWith(
    foregroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return colors.onInverseSurface;
      }
      if (states.contains(MaterialState.pressed)) {
        return colors.onSurface;
      }
      return null;
    }),
  );
}

ButtonStyle disabledOutlinedButtonStyle(bool selected, ColorScheme colors) {
  return IconButton.styleFrom(
    disabledForegroundColor: colors.onSurface.withOpacity(0.38),
    disabledBackgroundColor: selected ? colors.onSurface.withOpacity(0.12) : null,
    side: selected ? null : BorderSide(color: colors.outline.withOpacity(0.12)),
  );
}

class Chips extends StatefulWidget {
  const Chips({super.key});

  @override
  State<Chips> createState() => _ChipsState();
}

class _ChipsState extends State<Chips> {
  bool isFiltered = true;

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Chips',
      tooltipMessage:
          'Use ActionChip, FilterChip, or InputChip. \nActionChip can also be used for suggestion chip',
      child: Column(
        children: <Widget>[
          Wrap(
            spacing: smallSpacing,
            runSpacing: smallSpacing,
            children: <Widget>[
              ActionChip(
                label: const Text('Assist'),
                avatar: const Icon(Icons.event),
                onPressed: () {},
              ),
              FilterChip(
                label: const Text('Filter'),
                selected: isFiltered,
                onSelected: (bool selected) {
                  setState(() => isFiltered = selected);
                },
              ),
              InputChip(label: const Text('Input'), onPressed: () {}, onDeleted: () {}),
              ActionChip(label: const Text('Suggestion'), onPressed: () {}),
            ],
          ),
          colDivider,
          Wrap(
            spacing: smallSpacing,
            runSpacing: smallSpacing,
            children: <Widget>[
              const ActionChip(label: Text('Assist'), avatar: Icon(Icons.event)),
              FilterChip(label: const Text('Filter'), selected: isFiltered, onSelected: null),
              InputChip(label: const Text('Input'), onDeleted: () {}, isEnabled: false),
              const ActionChip(label: Text('Suggestion')),
            ],
          ),
        ],
      ),
    );
  }
}

class SegmentedButtons extends StatelessWidget {
  const SegmentedButtons({super.key});

  @override
  Widget build(BuildContext context) {
    return const ComponentDecoration(
      label: 'Segmented buttons',
      tooltipMessage: 'Use SegmentedButton<T>',
      child: Column(children: <Widget>[SingleChoice(), colDivider, MultipleChoice()]),
    );
  }
}

enum Calendar { day, week, month, year }

class SingleChoice extends StatefulWidget {
  const SingleChoice({super.key});

  @override
  State<SingleChoice> createState() => _SingleChoiceState();
}

class _SingleChoiceState extends State<SingleChoice> {
  Calendar calendarView = Calendar.day;

  @override
  Widget build(BuildContext context) {
    return SegmentedButton<Calendar>(
      segments: const <ButtonSegment<Calendar>>[
        ButtonSegment<Calendar>(
          value: Calendar.day,
          label: Text('Day'),
          icon: Icon(Icons.calendar_view_day),
        ),
        ButtonSegment<Calendar>(
          value: Calendar.week,
          label: Text('Week'),
          icon: Icon(Icons.calendar_view_week),
        ),
        ButtonSegment<Calendar>(
          value: Calendar.month,
          label: Text('Month'),
          icon: Icon(Icons.calendar_view_month),
        ),
        ButtonSegment<Calendar>(
          value: Calendar.year,
          label: Text('Year'),
          icon: Icon(Icons.calendar_today),
        ),
      ],
      selected: <Calendar>{calendarView},
      onSelectionChanged: (Set<Calendar> newSelection) {
        setState(() {
          // By default there is only a single segment that can be
          // selected at one time, so its value is always the first
          // item in the selected set.
          calendarView = newSelection.first;
        });
      },
    );
  }
}

enum Sizes { extraSmall, small, medium, large, extraLarge }

class MultipleChoice extends StatefulWidget {
  const MultipleChoice({super.key});

  @override
  State<MultipleChoice> createState() => _MultipleChoiceState();
}

class _MultipleChoiceState extends State<MultipleChoice> {
  Set<Sizes> selection = <Sizes>{Sizes.large, Sizes.extraLarge};

  @override
  Widget build(BuildContext context) {
    return SegmentedButton<Sizes>(
      segments: const <ButtonSegment<Sizes>>[
        ButtonSegment<Sizes>(value: Sizes.extraSmall, label: Text('XS')),
        ButtonSegment<Sizes>(value: Sizes.small, label: Text('S')),
        ButtonSegment<Sizes>(value: Sizes.medium, label: Text('M')),
        ButtonSegment<Sizes>(value: Sizes.large, label: Text('L')),
        ButtonSegment<Sizes>(value: Sizes.extraLarge, label: Text('XL')),
      ],
      selected: selection,
      onSelectionChanged: (Set<Sizes> newSelection) {
        setState(() {
          selection = newSelection;
        });
      },
      multiSelectionEnabled: true,
    );
  }
}

class SnackBarSection extends StatelessWidget {
  const SnackBarSection({super.key});

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Snackbar',
      tooltipMessage: 'Use ScaffoldMessenger.of(context).showSnackBar with SnackBar',
      child: TextButton(
        onPressed: () {
          final SnackBar snackBar = SnackBar(
            behavior: SnackBarBehavior.floating,
            width: 400.0,
            content: const Text('This is a snackbar'),
            action: SnackBarAction(label: 'Close', onPressed: () {}),
          );

          ScaffoldMessenger.of(context).hideCurrentSnackBar();
          ScaffoldMessenger.of(context).showSnackBar(snackBar);
        },
        child: const Text('Show snackbar', style: TextStyle(fontWeight: FontWeight.bold)),
      ),
    );
  }
}

class BottomSheetSection extends StatefulWidget {
  const BottomSheetSection({super.key});

  @override
  State<BottomSheetSection> createState() => _BottomSheetSectionState();
}

class _BottomSheetSectionState extends State<BottomSheetSection> {
  bool isNonModalBottomSheetOpen = false;
  PersistentBottomSheetController? _nonModalBottomSheetController;

  @override
  Widget build(BuildContext context) {
    List<Widget> buttonList = <Widget>[
      IconButton(onPressed: () {}, icon: const Icon(Icons.share_outlined)),
      IconButton(onPressed: () {}, icon: const Icon(Icons.add)),
      IconButton(onPressed: () {}, icon: const Icon(Icons.delete_outline)),
      IconButton(onPressed: () {}, icon: const Icon(Icons.archive_outlined)),
      IconButton(onPressed: () {}, icon: const Icon(Icons.settings_outlined)),
      IconButton(onPressed: () {}, icon: const Icon(Icons.favorite_border)),
    ];
    const List<Text> labelList = <Text>[
      Text('Share'),
      Text('Add to'),
      Text('Trash'),
      Text('Archive'),
      Text('Settings'),
      Text('Favorite'),
    ];

    buttonList = List<Widget>.generate(
      buttonList.length,
      (int index) => Padding(
        padding: const EdgeInsets.fromLTRB(20.0, 30.0, 20.0, 20.0),
        child: Column(children: <Widget>[buttonList[index], labelList[index]]),
      ),
    );

    return ComponentDecoration(
      label: 'Bottom sheet',
      tooltipMessage: 'Use showModalBottomSheet<T> or showBottomSheet<T>',
      child: Wrap(
        alignment: WrapAlignment.spaceEvenly,
        children: <Widget>[
          TextButton(
            child: const Text(
              'Show modal bottom sheet',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
            onPressed: () {
              showModalBottomSheet<void>(
                context: context,
                constraints: const BoxConstraints(maxWidth: 640),
                builder: (BuildContext context) {
                  return SizedBox(
                    height: 150,
                    child: Padding(
                      padding: const EdgeInsets.symmetric(horizontal: 32.0),
                      child: ListView(
                        shrinkWrap: true,
                        scrollDirection: Axis.horizontal,
                        children: buttonList,
                      ),
                    ),
                  );
                },
              );
            },
          ),
          TextButton(
            child: Text(
              isNonModalBottomSheetOpen ? 'Hide bottom sheet' : 'Show bottom sheet',
              style: const TextStyle(fontWeight: FontWeight.bold),
            ),
            onPressed: () {
              if (isNonModalBottomSheetOpen) {
                _nonModalBottomSheetController?.close();
                setState(() {
                  isNonModalBottomSheetOpen = false;
                });
                return;
              } else {
                setState(() {
                  isNonModalBottomSheetOpen = true;
                });
              }

              _nonModalBottomSheetController = showBottomSheet(
                elevation: 8.0,
                context: context,
                constraints: const BoxConstraints(maxWidth: 640),
                builder: (BuildContext context) {
                  return SizedBox(
                    height: 150,
                    child: Padding(
                      padding: const EdgeInsets.symmetric(horizontal: 32.0),
                      child: ListView(
                        shrinkWrap: true,
                        scrollDirection: Axis.horizontal,
                        children: buttonList,
                      ),
                    ),
                  );
                },
              );
            },
          ),
        ],
      ),
    );
  }
}

class BottomAppBars extends StatelessWidget {
  const BottomAppBars({super.key});

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Bottom app bar',
      tooltipMessage: 'Use BottomAppBar',
      child: Column(
        children: <Widget>[
          SizedBox(
            height: 80,
            child: Scaffold(
              floatingActionButton: FloatingActionButton(
                onPressed: () {},
                elevation: 0.0,
                child: const Icon(Icons.add),
              ),
              floatingActionButtonLocation: FloatingActionButtonLocation.endContained,
              bottomNavigationBar: BottomAppBar(
                child: Row(
                  children: <Widget>[
                    const IconButtonAnchorExample(),
                    IconButton(tooltip: 'Search', icon: const Icon(Icons.search), onPressed: () {}),
                    IconButton(
                      tooltip: 'Favorite',
                      icon: const Icon(Icons.favorite),
                      onPressed: () {},
                    ),
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class IconButtonAnchorExample extends StatelessWidget {
  const IconButtonAnchorExample({super.key});

  @override
  Widget build(BuildContext context) {
    return MenuAnchor(
      builder: (BuildContext context, MenuController controller, Widget? child) {
        return IconButton(
          onPressed: () {
            if (controller.isOpen) {
              controller.close();
            } else {
              controller.open();
            }
          },
          icon: const Icon(Icons.more_vert),
        );
      },
      menuChildren: <Widget>[
        MenuItemButton(child: const Text('Menu 1'), onPressed: () {}),
        MenuItemButton(child: const Text('Menu 2'), onPressed: () {}),
        SubmenuButton(
          menuChildren: <Widget>[
            MenuItemButton(onPressed: () {}, child: const Text('Menu 3.1')),
            MenuItemButton(onPressed: () {}, child: const Text('Menu 3.2')),
            MenuItemButton(onPressed: () {}, child: const Text('Menu 3.3')),
          ],
          child: const Text('Menu 3'),
        ),
      ],
    );
  }
}

class ButtonAnchorExample extends StatelessWidget {
  const ButtonAnchorExample({super.key});

  @override
  Widget build(BuildContext context) {
    return MenuAnchor(
      builder: (BuildContext context, MenuController controller, Widget? child) {
        return FilledButton.tonal(
          onPressed: () {
            if (controller.isOpen) {
              controller.close();
            } else {
              controller.open();
            }
          },
          child: const Text('Show menu'),
        );
      },
      menuChildren: <Widget>[
        MenuItemButton(
          leadingIcon: const Icon(Icons.people_alt_outlined),
          child: const Text('Item 1'),
          onPressed: () {},
        ),
        MenuItemButton(
          leadingIcon: const Icon(Icons.remove_red_eye_outlined),
          child: const Text('Item 2'),
          onPressed: () {},
        ),
        MenuItemButton(
          leadingIcon: const Icon(Icons.refresh),
          onPressed: () {},
          child: const Text('Item 3'),
        ),
      ],
    );
  }
}

class NavigationDrawers extends StatelessWidget {
  const NavigationDrawers({super.key, required this.scaffoldKey});
  final GlobalKey<ScaffoldState> scaffoldKey;

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Navigation drawer',
      tooltipMessage: 'Use NavigationDrawer. For modal navigation drawers, see Scaffold.endDrawer',
      child: Column(
        children: <Widget>[
          const SizedBox(height: 520, child: NavigationDrawerSection()),
          colDivider,
          colDivider,
          TextButton(
            child: const Text(
              'Show modal navigation drawer',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
            onPressed: () {
              scaffoldKey.currentState!.openEndDrawer();
            },
          ),
        ],
      ),
    );
  }
}

class NavigationDrawerSection extends StatefulWidget {
  const NavigationDrawerSection({super.key});

  @override
  State<NavigationDrawerSection> createState() => _NavigationDrawerSectionState();
}

class _NavigationDrawerSectionState extends State<NavigationDrawerSection> {
  int navDrawerIndex = 0;

  @override
  Widget build(BuildContext context) {
    return NavigationDrawer(
      onDestinationSelected: (int selectedIndex) {
        setState(() {
          navDrawerIndex = selectedIndex;
        });
      },
      selectedIndex: navDrawerIndex,
      children: <Widget>[
        Padding(
          padding: const EdgeInsets.fromLTRB(28, 16, 16, 10),
          child: Text('Mail', style: Theme.of(context).textTheme.titleSmall),
        ),
        ...destinations.map((ExampleDestination destination) {
          return NavigationDrawerDestination(
            label: Text(destination.label),
            icon: destination.icon,
            selectedIcon: destination.selectedIcon,
          );
        }),
        const Divider(indent: 28, endIndent: 28),
        Padding(
          padding: const EdgeInsets.fromLTRB(28, 16, 16, 10),
          child: Text('Labels', style: Theme.of(context).textTheme.titleSmall),
        ),
        ...labelDestinations.map((ExampleDestination destination) {
          return NavigationDrawerDestination(
            label: Text(destination.label),
            icon: destination.icon,
            selectedIcon: destination.selectedIcon,
          );
        }),
      ],
    );
  }
}

class ExampleDestination {
  const ExampleDestination(this.label, this.icon, this.selectedIcon);

  final String label;
  final Widget icon;
  final Widget selectedIcon;
}

const List<ExampleDestination> destinations = <ExampleDestination>[
  ExampleDestination('Inbox', Icon(Icons.inbox_outlined), Icon(Icons.inbox)),
  ExampleDestination('Outbox', Icon(Icons.send_outlined), Icon(Icons.send)),
  ExampleDestination('Favorites', Icon(Icons.favorite_outline), Icon(Icons.favorite)),
  ExampleDestination('Trash', Icon(Icons.delete_outline), Icon(Icons.delete)),
];

const List<ExampleDestination> labelDestinations = <ExampleDestination>[
  ExampleDestination('Family', Icon(Icons.bookmark_border), Icon(Icons.bookmark)),
  ExampleDestination('School', Icon(Icons.bookmark_border), Icon(Icons.bookmark)),
  ExampleDestination('Work', Icon(Icons.bookmark_border), Icon(Icons.bookmark)),
];

class NavigationRails extends StatelessWidget {
  const NavigationRails({super.key});

  @override
  Widget build(BuildContext context) {
    return const ComponentDecoration(
      label: 'Navigation rail',
      tooltipMessage: 'Use NavigationRail',
      child: IntrinsicWidth(child: SizedBox(height: 420, child: NavigationRailSection())),
    );
  }
}

class NavigationRailSection extends StatefulWidget {
  const NavigationRailSection({super.key});

  @override
  State<NavigationRailSection> createState() => _NavigationRailSectionState();
}

class _NavigationRailSectionState extends State<NavigationRailSection> {
  int navRailIndex = 0;

  @override
  Widget build(BuildContext context) {
    return NavigationRail(
      onDestinationSelected: (int selectedIndex) {
        setState(() {
          navRailIndex = selectedIndex;
        });
      },
      elevation: 4,
      leading: FloatingActionButton(child: const Icon(Icons.create), onPressed: () {}),
      groupAlignment: 0.0,
      selectedIndex: navRailIndex,
      labelType: NavigationRailLabelType.selected,
      destinations: <NavigationRailDestination>[
        ...destinations.map((ExampleDestination destination) {
          return NavigationRailDestination(
            label: Text(destination.label),
            icon: destination.icon,
            selectedIcon: destination.selectedIcon,
          );
        }),
      ],
    );
  }
}

class Tabs extends StatefulWidget {
  const Tabs({super.key});

  @override
  State<Tabs> createState() => _TabsState();
}

class _TabsState extends State<Tabs> with TickerProviderStateMixin {
  late TabController _tabController;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 3, vsync: this);
  }

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Tabs',
      tooltipMessage: 'Use TabBar',
      child: SizedBox(
        height: 80,
        child: Scaffold(
          appBar: AppBar(
            bottom: TabBar(
              controller: _tabController,
              tabs: const <Widget>[
                Tab(
                  icon: Icon(Icons.videocam_outlined),
                  text: 'Video',
                  iconMargin: EdgeInsets.zero,
                ),
                Tab(icon: Icon(Icons.photo_outlined), text: 'Photos', iconMargin: EdgeInsets.zero),
                Tab(icon: Icon(Icons.audiotrack_sharp), text: 'Audio', iconMargin: EdgeInsets.zero),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class TopAppBars extends StatelessWidget {
  const TopAppBars({super.key});

  static final List<IconButton> actions = <IconButton>[
    IconButton(icon: const Icon(Icons.attach_file), onPressed: () {}),
    IconButton(icon: const Icon(Icons.event), onPressed: () {}),
    IconButton(icon: const Icon(Icons.more_vert), onPressed: () {}),
  ];

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Top app bars',
      tooltipMessage: 'Use AppBar, SliverAppBar, SliverAppBar.medium, or  SliverAppBar.large',
      child: Column(
        children: <Widget>[
          AppBar(
            title: const Text('Center-aligned'),
            leading: const BackButton(),
            actions: <Widget>[
              IconButton(
                iconSize: 32,
                icon: const Icon(Icons.account_circle_outlined),
                onPressed: () {},
              ),
            ],
            centerTitle: true,
          ),
          colDivider,
          AppBar(
            title: const Text('Small'),
            leading: const BackButton(),
            actions: actions,
            centerTitle: false,
          ),
          colDivider,
          SizedBox(
            height: 100,
            child: CustomScrollView(
              slivers: <Widget>[
                SliverAppBar.medium(
                  title: const Text('Medium'),
                  leading: const BackButton(),
                  actions: actions,
                ),
                const SliverFillRemaining(),
              ],
            ),
          ),
          colDivider,
          SizedBox(
            height: 130,
            child: CustomScrollView(
              slivers: <Widget>[
                SliverAppBar.large(
                  title: const Text('Large'),
                  leading: const BackButton(),
                  actions: actions,
                ),
                const SliverFillRemaining(),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class Menus extends StatefulWidget {
  const Menus({super.key});

  @override
  State<Menus> createState() => _MenusState();
}

class _MenusState extends State<Menus> {
  final TextEditingController colorController = TextEditingController();
  final TextEditingController iconController = TextEditingController();
  IconLabel? selectedIcon = IconLabel.smile;
  ColorLabel? selectedColor;

  @override
  Widget build(BuildContext context) {
    final List<DropdownMenuEntry<ColorLabel>> colorEntries = <DropdownMenuEntry<ColorLabel>>[];
    for (final ColorLabel color in ColorLabel.values) {
      colorEntries.add(
        DropdownMenuEntry<ColorLabel>(
          value: color,
          label: color.label,
          enabled: color.label != 'Grey',
        ),
      );
    }

    final List<DropdownMenuEntry<IconLabel>> iconEntries = <DropdownMenuEntry<IconLabel>>[];
    for (final IconLabel icon in IconLabel.values) {
      iconEntries.add(DropdownMenuEntry<IconLabel>(value: icon, label: icon.label));
    }

    return ComponentDecoration(
      label: 'Menus',
      tooltipMessage: 'Use MenuAnchor or DropdownMenu<T>',
      child: Column(
        children: <Widget>[
          const Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[ButtonAnchorExample(), rowDivider, IconButtonAnchorExample()],
          ),
          colDivider,
          Wrap(
            alignment: WrapAlignment.spaceAround,
            runAlignment: WrapAlignment.center,
            crossAxisAlignment: WrapCrossAlignment.center,
            spacing: smallSpacing,
            runSpacing: smallSpacing,
            children: <Widget>[
              DropdownMenu<ColorLabel>(
                controller: colorController,
                label: const Text('Color'),
                enableFilter: true,
                dropdownMenuEntries: colorEntries,
                inputDecorationTheme: const InputDecorationTheme(filled: true),
                onSelected: (ColorLabel? color) {
                  setState(() {
                    selectedColor = color;
                  });
                },
              ),
              DropdownMenu<IconLabel>(
                initialSelection: IconLabel.smile,
                controller: iconController,
                leadingIcon: const Icon(Icons.search),
                label: const Text('Icon'),
                dropdownMenuEntries: iconEntries,
                onSelected: (IconLabel? icon) {
                  setState(() {
                    selectedIcon = icon;
                  });
                },
              ),
              Icon(selectedIcon?.icon, color: selectedColor?.color ?? Colors.grey.withOpacity(0.5)),
            ],
          ),
        ],
      ),
    );
  }
}

enum ColorLabel {
  blue('Blue', Colors.blue),
  pink('Pink', Colors.pink),
  green('Green', Colors.green),
  yellow('Yellow', Colors.yellow),
  grey('Grey', Colors.grey);

  const ColorLabel(this.label, this.color);
  final String label;
  final Color color;
}

enum IconLabel {
  smile('Smile', Icons.sentiment_satisfied_outlined),
  cloud('Cloud', Icons.cloud_outlined),
  brush('Brush', Icons.brush_outlined),
  heart('Heart', Icons.favorite);

  const IconLabel(this.label, this.icon);
  final String label;
  final IconData icon;
}

class Sliders extends StatefulWidget {
  const Sliders({super.key});

  @override
  State<Sliders> createState() => _SlidersState();
}

class _SlidersState extends State<Sliders> {
  double sliderValue0 = 30.0;
  double sliderValue1 = 20.0;

  @override
  Widget build(BuildContext context) {
    return ComponentDecoration(
      label: 'Sliders',
      tooltipMessage: 'Use Slider or RangeSlider',
      child: Column(
        children: <Widget>[
          Slider(
            max: 100,
            value: sliderValue0,
            onChanged: (double value) {
              setState(() {
                sliderValue0 = value;
              });
            },
          ),
          const SizedBox(height: 20),
          Slider(
            max: 100,
            divisions: 5,
            value: sliderValue1,
            label: sliderValue1.round().toString(),
            onChanged: (double value) {
              setState(() {
                sliderValue1 = value;
              });
            },
          ),
        ],
      ),
    );
  }
}

class ComponentDecoration extends StatefulWidget {
  const ComponentDecoration({
    super.key,
    required this.label,
    required this.child,
    this.tooltipMessage = '',
  });

  final String label;
  final Widget child;
  final String? tooltipMessage;

  @override
  State<ComponentDecoration> createState() => _ComponentDecorationState();
}

class _ComponentDecorationState extends State<ComponentDecoration> {
  final FocusNode focusNode = FocusNode();

  @override
  Widget build(BuildContext context) {
    return RepaintBoundary(
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: smallSpacing),
        child: Column(
          children: <Widget>[
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                Text(widget.label, style: Theme.of(context).textTheme.titleSmall),
                Tooltip(
                  message: widget.tooltipMessage,
                  child: const Padding(
                    padding: EdgeInsets.symmetric(horizontal: 5.0),
                    child: Icon(Icons.info_outline, size: 16),
                  ),
                ),
              ],
            ),
            ConstrainedBox(
              constraints: const BoxConstraints.tightFor(width: widthConstraint),
              // Tapping within the a component card should request focus
              // for that component's children.
              child: Focus(
                focusNode: focusNode,
                canRequestFocus: true,
                child: GestureDetector(
                  onTapDown: (_) {
                    focusNode.requestFocus();
                  },
                  behavior: HitTestBehavior.opaque,
                  child: Card(
                    elevation: 0,
                    shape: RoundedRectangleBorder(
                      side: BorderSide(color: Theme.of(context).colorScheme.outlineVariant),
                      borderRadius: const BorderRadius.all(Radius.circular(12)),
                    ),
                    child: Padding(
                      padding: const EdgeInsets.symmetric(horizontal: 5.0, vertical: 20.0),
                      child: Center(child: widget.child),
                    ),
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

class ComponentGroupDecoration extends StatelessWidget {
  const ComponentGroupDecoration({super.key, required this.label, required this.children});

  final String label;
  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    // Fully traverse this component group before moving on
    return FocusTraversalGroup(
      child: Card(
        margin: EdgeInsets.zero,
        elevation: 0,
        color: Theme.of(context).colorScheme.surfaceContainerHighest.withOpacity(0.3),
        child: Padding(
          padding: const EdgeInsets.symmetric(vertical: 20.0),
          child: Center(
            child: Column(
              children: <Widget>[
                Text(label, style: Theme.of(context).textTheme.titleLarge),
                colDivider,
                ...children,
              ],
            ),
          ),
        ),
      ),
    );
  }
}
