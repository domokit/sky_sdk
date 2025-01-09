// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SafeArea].
///
/// The app is wrapped with [Insets] (defined below)
/// to simulate a mobile device with a notched screen.

void main() => runApp(const Insets());

class SafeAreaExampleApp extends StatelessWidget {
  const SafeAreaExampleApp({super.key});

  static const Color spring = Color(0xFF00FF80);
  static final ColorScheme colors = ColorScheme.fromSeed(seedColor: spring);
  static final ThemeData theme = ThemeData(
    colorScheme: colors,
    sliderTheme: SliderThemeData(
      trackHeight: 8,
      activeTrackColor: colors.primary.withValues(alpha: 0.5),
      thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 12),
    ),
    scaffoldBackgroundColor: const Color(0xFFD0FFE8),
    listTileTheme: const ListTileThemeData(
      tileColor: Colors.white70,
      visualDensity: VisualDensity(horizontal: -4, vertical: -4),
    ),
    appBarTheme: AppBarTheme(
      backgroundColor: colors.secondary,
      foregroundColor: colors.onSecondary,
    ),
  );

  static final AppBar appBar = AppBar(title: const Text('SafeArea Demo'));

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: theme,
      debugShowCheckedModeBanner: false,
      home: Builder(
        builder:
            (BuildContext context) => Scaffold(
              appBar: Toggle.appBar.of(context) ? appBar : null,
              body: const DefaultTextStyle(
                style: TextStyle(fontSize: 16, fontWeight: FontWeight.w600, color: Colors.black),
                child: Center(child: SafeAreaExample()),
              ),
            ),
      ),
    );
  }
}

class SafeAreaExample extends StatelessWidget {
  const SafeAreaExample({super.key});

  static final Widget controls = Column(
    children: <Widget>[
      const SizedBox(height: 6),
      Builder(
        builder:
            (BuildContext context) => Text(
              Toggle.safeArea.of(context) ? 'safe area!' : 'no safe area',
              style: const TextStyle(fontSize: 24),
            ),
      ),
      const Spacer(flex: 2),
      for (final Value data in Value.allValues) ...data.controls,
    ],
  );

  @override
  Widget build(BuildContext context) {
    final bool hasSafeArea = Toggle.safeArea.of(context);

    return SafeArea(
      top: hasSafeArea,
      bottom: hasSafeArea,
      left: hasSafeArea,
      right: hasSafeArea,
      child: controls,
    );
  }
}

sealed class Value implements Enum {
  Object _getValue(covariant Model<Value> model);

  List<Widget> get controls;

  static const List<Value> allValues = <Value>[...Inset.values, ...Toggle.values];
}

enum Inset implements Value {
  top('top notch'),
  sides('side padding'),
  bottom('bottom indicator');

  const Inset(this.label);

  final String label;

  @override
  double _getValue(_InsetModel model) => switch (this) {
    top => model.insets.top,
    sides => model.insets.left,
    bottom => model.insets.bottom,
  };

  double of(BuildContext context) => _getValue(Model.of<_InsetModel>(context, this));

  @override
  List<Widget> get controls => <Widget>[
    Text(label),
    Builder(
      builder:
          (BuildContext context) => Slider(
            max: 50,
            value: of(context),
            onChanged: (double newValue) {
              InsetsState.instance.changeInset(this, newValue);
            },
          ),
    ),
    const Spacer(),
  ];
}

enum Toggle implements Value {
  appBar('Build an AppBar?'),
  safeArea("Wrap Scaffold's body with SafeArea?");

  const Toggle(this.label);

  final String label;

  @override
  bool _getValue(_ToggleModel model) => switch (this) {
    appBar => model.buildAppBar,
    safeArea => model.buildSafeArea,
  };

  bool of(BuildContext context) => _getValue(Model.of<_ToggleModel>(context, this));

  @override
  List<Widget> get controls => <Widget>[
    Builder(
      builder:
          (BuildContext context) => SwitchListTile(
            title: Text(label),
            value: of(context),
            onChanged: (bool value) {
              InsetsState.instance.toggle(this, value);
            },
          ),
    ),
  ];
}

abstract class Model<E extends Value> extends InheritedModel<E> {
  const Model({super.key, required super.child});

  static M of<M extends Model<Value>>(BuildContext context, Value value) {
    return context.dependOnInheritedWidgetOfExactType<M>(aspect: value)!;
  }

  @override
  bool updateShouldNotify(Model<E> oldWidget) => true;

  @override
  bool updateShouldNotifyDependent(Model<E> oldWidget, Set<E> dependencies) {
    return dependencies.any((E data) => data._getValue(this) != data._getValue(oldWidget));
  }
}

class _InsetModel extends Model<Inset> {
  const _InsetModel({required this.insets, required super.child});

  final EdgeInsets insets;
}

class _ToggleModel extends Model<Toggle> {
  _ToggleModel({required Set<Toggle> togglers, required super.child})
    : buildAppBar = togglers.contains(Toggle.appBar),
      buildSafeArea = togglers.contains(Toggle.safeArea);

  final bool buildAppBar;
  final bool buildSafeArea;
}

class Insets extends UniqueWidget<InsetsState> {
  const Insets() : super(key: const GlobalObjectKey<InsetsState>('insets'));

  @override
  InsetsState createState() => InsetsState();
}

class InsetsState extends State<Insets> {
  static InsetsState get instance => const Insets().currentState!;

  EdgeInsets insets = const EdgeInsets.fromLTRB(8, 25, 8, 12);
  void changeInset(Inset inset, double value) {
    setState(() {
      insets = switch (inset) {
        Inset.top => insets.copyWith(top: value),
        Inset.sides => insets.copyWith(left: value, right: value),
        Inset.bottom => insets.copyWith(bottom: value),
      };
    });
  }

  final Set<Toggle> _togglers = <Toggle>{};
  void toggle(Toggle toggler, bool value) {
    setState(() {
      value ? _togglers.add(toggler) : _togglers.remove(toggler);
    });
  }

  @override
  Widget build(BuildContext context) {
    final Widget topNotch = ClipRRect(
      borderRadius: BorderRadius.vertical(bottom: Radius.circular(insets.top)),
      child: SizedBox(
        height: insets.top,
        child: const FractionallySizedBox(
          widthFactor: 1 / 2,
          child: ColoredBox(color: Colors.black),
        ),
      ),
    );
    final Widget bottomIndicator = SizedBox(
      width: double.infinity,
      height: insets.bottom,
      child: const FractionallySizedBox(
        heightFactor: 0.5,
        widthFactor: 0.5,
        child: PhysicalShape(
          clipper: ShapeBorderClipper(shape: StadiumBorder()),
          color: Color(0xC0000000),
          child: SizedBox.expand(),
        ),
      ),
    );
    final Widget sideBar = SizedBox(
      width: insets.left,
      height: double.infinity,
      child: const IgnorePointer(child: ColoredBox(color: Colors.black12)),
    );

    final Widget app = _ToggleModel(
      togglers: _togglers,
      child: Builder(
        builder:
            (BuildContext context) => MediaQuery(
              data: MediaQuery.of(context).copyWith(
                viewInsets: EdgeInsets.only(top: insets.top),
                viewPadding: insets,
                padding: insets,
              ),
              child: const SafeAreaExampleApp(),
            ),
      ),
    );

    return _InsetModel(
      insets: insets,
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            app,
            Align(alignment: Alignment.topCenter, child: topNotch),
            Align(alignment: Alignment.bottomCenter, child: bottomIndicator),
            Align(alignment: Alignment.centerLeft, child: sideBar),
            Align(alignment: Alignment.centerRight, child: sideBar),
          ],
        ),
      ),
    );
  }
}
