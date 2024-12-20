// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

class BottomAppBarDemo extends StatefulWidget {
  const BottomAppBarDemo({super.key});

  static const String routeName = '/material/bottom_app_bar';

  @override
  State createState() => _BottomAppBarDemoState();
}

// Flutter generally frowns upon abbreviation however this class uses two
// abbreviations extensively: "fab" for floating action button, and "bab"
// for bottom application bar.

class _BottomAppBarDemoState extends State<BottomAppBarDemo> {
  static final GlobalKey<ScaffoldMessengerState> _scaffoldMessengerKey =
      GlobalKey<ScaffoldMessengerState>();

  // FAB shape

  static const _ChoiceValue<Widget> kNoFab = _ChoiceValue<Widget>(
    title: 'None',
    label: 'do not show a floating action button',
  );

  static const _ChoiceValue<Widget> kCircularFab = _ChoiceValue<Widget>(
    title: 'Circular',
    label: 'circular floating action button',
    value: FloatingActionButton(
      onPressed: _showSnackbar,
      backgroundColor: Colors.orange,
      child: Icon(Icons.add, semanticLabel: 'Action'),
    ),
  );

  static const _ChoiceValue<Widget> kDiamondFab = _ChoiceValue<Widget>(
    title: 'Diamond',
    label: 'diamond shape floating action button',
    value: _DiamondFab(onPressed: _showSnackbar, child: Icon(Icons.add, semanticLabel: 'Action')),
  );

  // Notch

  static const _ChoiceValue<bool> kShowNotchTrue = _ChoiceValue<bool>(
    title: 'On',
    label: 'show bottom appbar notch',
    value: true,
  );

  static const _ChoiceValue<bool> kShowNotchFalse = _ChoiceValue<bool>(
    title: 'Off',
    label: 'do not show bottom appbar notch',
    value: false,
  );

  // FAB Position

  static const _ChoiceValue<FloatingActionButtonLocation> kFabEndDocked =
      _ChoiceValue<FloatingActionButtonLocation>(
        title: 'Attached - End',
        label: 'floating action button is docked at the end of the bottom app bar',
        value: FloatingActionButtonLocation.endDocked,
      );

  static const _ChoiceValue<FloatingActionButtonLocation> kFabCenterDocked =
      _ChoiceValue<FloatingActionButtonLocation>(
        title: 'Attached - Center',
        label: 'floating action button is docked at the center of the bottom app bar',
        value: FloatingActionButtonLocation.centerDocked,
      );

  static const _ChoiceValue<FloatingActionButtonLocation> kFabEndFloat =
      _ChoiceValue<FloatingActionButtonLocation>(
        title: 'Free - End',
        label: 'floating action button floats above the end of the bottom app bar',
        value: FloatingActionButtonLocation.endFloat,
      );

  static const _ChoiceValue<FloatingActionButtonLocation> kFabCenterFloat =
      _ChoiceValue<FloatingActionButtonLocation>(
        title: 'Free - Center',
        label: 'floating action button is floats above the center of the bottom app bar',
        value: FloatingActionButtonLocation.centerFloat,
      );

  static void _showSnackbar() {
    const String text =
        "When the Scaffold's floating action button location changes, "
        'the floating action button animates to its new position. '
        'The BottomAppBar adapts its shape appropriately.';
    _scaffoldMessengerKey.currentState!.showSnackBar(const SnackBar(content: Text(text)));
  }

  // App bar color

  static const List<_NamedColor> kBabColors = <_NamedColor>[
    _NamedColor(null, 'Clear'),
    _NamedColor(Color(0xFFFFC100), 'Orange'),
    _NamedColor(Color(0xFF91FAFF), 'Light Blue'),
    _NamedColor(Color(0xFF00D1FF), 'Cyan'),
    _NamedColor(Color(0xFF00BCFF), 'Cerulean'),
    _NamedColor(Color(0xFF009BEE), 'Blue'),
  ];

  _ChoiceValue<Widget> _fabShape = kCircularFab;
  _ChoiceValue<bool> _showNotch = kShowNotchTrue;
  _ChoiceValue<FloatingActionButtonLocation> _fabLocation = kFabEndDocked;
  Color? _babColor = kBabColors.first.color;

  void _onShowNotchChanged(_ChoiceValue<bool>? value) {
    setState(() {
      _showNotch = value!;
    });
  }

  void _onFabShapeChanged(_ChoiceValue<Widget>? value) {
    setState(() {
      _fabShape = value!;
    });
  }

  void _onFabLocationChanged(_ChoiceValue<FloatingActionButtonLocation>? value) {
    setState(() {
      _fabLocation = value!;
    });
  }

  void _onBabColorChanged(Color? value) {
    setState(() {
      _babColor = value;
    });
  }

  @override
  Widget build(BuildContext context) {
    return ScaffoldMessenger(
      key: _scaffoldMessengerKey,
      child: Builder(
        builder:
            (BuildContext context) => Scaffold(
              appBar: AppBar(
                title: const Text('Bottom app bar'),
                elevation: 0.0,
                actions: <Widget>[
                  MaterialDemoDocumentationButton(BottomAppBarDemo.routeName),
                  IconButton(
                    icon: const Icon(Icons.sentiment_very_satisfied, semanticLabel: 'Update shape'),
                    onPressed: () {
                      setState(() {
                        _fabShape = _fabShape == kCircularFab ? kDiamondFab : kCircularFab;
                      });
                    },
                  ),
                ],
              ),
              body: Scrollbar(
                child: ListView(
                  primary: true,
                  padding: const EdgeInsets.only(bottom: 88.0),
                  children: <Widget>[
                    const _Heading('FAB Shape'),

                    _RadioItem<Widget>(kCircularFab, _fabShape, _onFabShapeChanged),
                    _RadioItem<Widget>(kDiamondFab, _fabShape, _onFabShapeChanged),
                    _RadioItem<Widget>(kNoFab, _fabShape, _onFabShapeChanged),

                    const Divider(),
                    const _Heading('Notch'),

                    _RadioItem<bool>(kShowNotchTrue, _showNotch, _onShowNotchChanged),
                    _RadioItem<bool>(kShowNotchFalse, _showNotch, _onShowNotchChanged),

                    const Divider(),
                    const _Heading('FAB Position'),

                    _RadioItem<FloatingActionButtonLocation>(
                      kFabEndDocked,
                      _fabLocation,
                      _onFabLocationChanged,
                    ),
                    _RadioItem<FloatingActionButtonLocation>(
                      kFabCenterDocked,
                      _fabLocation,
                      _onFabLocationChanged,
                    ),
                    _RadioItem<FloatingActionButtonLocation>(
                      kFabEndFloat,
                      _fabLocation,
                      _onFabLocationChanged,
                    ),
                    _RadioItem<FloatingActionButtonLocation>(
                      kFabCenterFloat,
                      _fabLocation,
                      _onFabLocationChanged,
                    ),

                    const Divider(),
                    const _Heading('App bar color'),

                    _ColorsItem(kBabColors, _babColor, _onBabColorChanged),
                  ],
                ),
              ),
              floatingActionButton: _fabShape.value,
              floatingActionButtonLocation: _fabLocation.value,
              bottomNavigationBar: _DemoBottomAppBar(
                color: _babColor,
                fabLocation: _fabLocation.value,
                shape: _selectNotch(),
              ),
            ),
      ),
    );
  }

  NotchedShape? _selectNotch() {
    if (!_showNotch.value!) {
      return null;
    }
    if (_fabShape == kCircularFab) {
      return const CircularNotchedRectangle();
    }
    if (_fabShape == kDiamondFab) {
      return const _DiamondNotchedRectangle();
    }
    return null;
  }
}

class _ChoiceValue<T> {
  const _ChoiceValue({this.value, this.title, this.label});

  final T? value;
  final String? title;
  final String? label; // For the Semantics widget that contains title

  @override
  String toString() => '$runtimeType("$title")';
}

class _RadioItem<T> extends StatelessWidget {
  const _RadioItem(this.value, this.groupValue, this.onChanged);

  final _ChoiceValue<T> value;
  final _ChoiceValue<T> groupValue;
  final ValueChanged<_ChoiceValue<T>?> onChanged;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    return Container(
      height: 56.0,
      padding: const EdgeInsetsDirectional.only(start: 16.0),
      alignment: AlignmentDirectional.centerStart,
      child: MergeSemantics(
        child: Row(
          children: <Widget>[
            Radio<_ChoiceValue<T>>(value: value, groupValue: groupValue, onChanged: onChanged),
            Expanded(
              child: Semantics(
                container: true,
                button: true,
                label: value.label,
                child: GestureDetector(
                  behavior: HitTestBehavior.opaque,
                  onTap: () {
                    onChanged(value);
                  },
                  child: Text(value.title!, style: theme.textTheme.titleMedium),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _NamedColor {
  const _NamedColor(this.color, this.name);

  final Color? color;
  final String name;
}

class _ColorsItem extends StatelessWidget {
  const _ColorsItem(this.colors, this.selectedColor, this.onChanged);

  final List<_NamedColor> colors;
  final Color? selectedColor;
  final ValueChanged<Color?> onChanged;

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children:
          colors.map<Widget>((_NamedColor namedColor) {
            return RawMaterialButton(
              onPressed: () {
                onChanged(namedColor.color);
              },
              constraints: const BoxConstraints.tightFor(width: 32.0, height: 32.0),
              fillColor: namedColor.color,
              shape: CircleBorder(
                side: BorderSide(
                  color: namedColor.color == selectedColor ? Colors.black : const Color(0xFFD5D7DA),
                  width: 2.0,
                ),
              ),
              child: Semantics(value: namedColor.name, selected: namedColor.color == selectedColor),
            );
          }).toList(),
    );
  }
}

class _Heading extends StatelessWidget {
  const _Heading(this.text);

  final String text;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    return Container(
      height: 48.0,
      padding: const EdgeInsetsDirectional.only(start: 56.0),
      alignment: AlignmentDirectional.centerStart,
      child: Text(text, style: theme.textTheme.bodyLarge),
    );
  }
}

class _DemoBottomAppBar extends StatelessWidget {
  const _DemoBottomAppBar({this.color, this.fabLocation, this.shape});

  final Color? color;
  final FloatingActionButtonLocation? fabLocation;
  final NotchedShape? shape;

  static final List<FloatingActionButtonLocation> kCenterLocations = <FloatingActionButtonLocation>[
    FloatingActionButtonLocation.centerDocked,
    FloatingActionButtonLocation.centerFloat,
  ];

  @override
  Widget build(BuildContext context) {
    return BottomAppBar(
      color: color,
      shape: shape,
      child: Row(
        children: <Widget>[
          IconButton(
            icon: const Icon(Icons.menu, semanticLabel: 'Show bottom sheet'),
            onPressed: () {
              showModalBottomSheet<void>(
                context: context,
                builder: (BuildContext context) => const _DemoDrawer(),
              );
            },
          ),
          if (kCenterLocations.contains(fabLocation)) const Expanded(child: SizedBox()),
          IconButton(
            icon: const Icon(Icons.search, semanticLabel: 'show search action'),
            onPressed: () {
              ScaffoldMessenger.of(
                context,
              ).showSnackBar(const SnackBar(content: Text('This is a dummy search action.')));
            },
          ),
          IconButton(
            icon: Icon(
              Theme.of(context).platform == TargetPlatform.iOS ? Icons.more_horiz : Icons.more_vert,
              semanticLabel: 'Show menu actions',
            ),
            onPressed: () {
              ScaffoldMessenger.of(
                context,
              ).showSnackBar(const SnackBar(content: Text('This is a dummy menu action.')));
            },
          ),
        ],
      ),
    );
  }
}

// A drawer that pops up from the bottom of the screen.
class _DemoDrawer extends StatelessWidget {
  const _DemoDrawer();

  @override
  Widget build(BuildContext context) {
    return const Drawer(
      child: Column(
        children: <Widget>[
          ListTile(leading: Icon(Icons.search), title: Text('Search')),
          ListTile(leading: Icon(Icons.threed_rotation), title: Text('3D')),
        ],
      ),
    );
  }
}

// A diamond-shaped floating action button.
class _DiamondFab extends StatelessWidget {
  const _DiamondFab({this.child, this.onPressed});

  final Widget? child;
  final VoidCallback? onPressed;

  @override
  Widget build(BuildContext context) {
    return Material(
      shape: const _DiamondBorder(),
      color: Colors.orange,
      elevation: 6.0,
      child: InkWell(
        onTap: onPressed,
        child: SizedBox(
          width: 56.0,
          height: 56.0,
          child: IconTheme.merge(
            data: IconThemeData(color: Theme.of(context).colorScheme.secondary),
            child: child!,
          ),
        ),
      ),
    );
  }
}

class _DiamondNotchedRectangle implements NotchedShape {
  const _DiamondNotchedRectangle();

  @override
  Path getOuterPath(Rect host, Rect? guest) {
    if (!host.overlaps(guest!)) {
      return Path()..addRect(host);
    }
    assert(guest.width > 0.0);

    final Rect intersection = guest.intersect(host);
    // We are computing a "V" shaped notch, as in this diagram:
    //    -----\****   /-----
    //          \     /
    //           \   /
    //            \ /
    //
    //  "-" marks the top edge of the bottom app bar.
    //  "\" and "/" marks the notch outline
    //
    //  notchToCenter is the horizontal distance between the guest's center and
    //  the host's top edge where the notch starts (marked with "*").
    //  We compute notchToCenter by similar triangles:
    final double notchToCenter = intersection.height * (guest.height / 2.0) / (guest.width / 2.0);

    return Path()
      ..moveTo(host.left, host.top)
      ..lineTo(guest.center.dx - notchToCenter, host.top)
      ..lineTo(guest.left + guest.width / 2.0, guest.bottom)
      ..lineTo(guest.center.dx + notchToCenter, host.top)
      ..lineTo(host.right, host.top)
      ..lineTo(host.right, host.bottom)
      ..lineTo(host.left, host.bottom)
      ..close();
  }
}

class _DiamondBorder extends ShapeBorder {
  const _DiamondBorder();

  @override
  EdgeInsetsGeometry get dimensions {
    return EdgeInsets.zero;
  }

  @override
  Path getInnerPath(Rect rect, {TextDirection? textDirection}) {
    return getOuterPath(rect, textDirection: textDirection);
  }

  @override
  Path getOuterPath(Rect rect, {TextDirection? textDirection}) {
    return Path()
      ..moveTo(rect.left + rect.width / 2.0, rect.top)
      ..lineTo(rect.right, rect.top + rect.height / 2.0)
      ..lineTo(rect.left + rect.width / 2.0, rect.bottom)
      ..lineTo(rect.left, rect.top + rect.height / 2.0)
      ..close();
  }

  @override
  void paint(Canvas canvas, Rect rect, {TextDirection? textDirection}) {}

  // This border doesn't support scaling.
  @override
  ShapeBorder scale(double t) {
    return this;
  }
}
