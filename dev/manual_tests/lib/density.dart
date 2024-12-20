// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart' show timeDilation;

final Map<int, Color> m2SwatchColors = <int, Color>{
  50: const Color(0xfff2e7fe),
  100: const Color(0xffd7b7fd),
  200: const Color(0xffbb86fc),
  300: const Color(0xff9e55fc),
  400: const Color(0xff7f22fd),
  500: const Color(0xff6200ee),
  600: const Color(0xff4b00d1),
  700: const Color(0xff3700b3),
  800: const Color(0xff270096),
  900: const Color(0xff270096),
};
final MaterialColor m2Swatch = MaterialColor(m2SwatchColors[500]!.value, m2SwatchColors);

void main() => runApp(const MyApp());

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  static const String _title = 'Density Test';

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(title: _title, home: MyHomePage());
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key});

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class OptionModel extends ChangeNotifier {
  double get size => _size;
  double _size = 1.0;
  set size(double size) {
    if (size != _size) {
      _size = size;
      notifyListeners();
    }
  }

  VisualDensity get density => _density;
  VisualDensity _density = VisualDensity.standard;
  set density(VisualDensity density) {
    if (density != _density) {
      _density = density;
      notifyListeners();
    }
  }

  bool get enable => _enable;
  bool _enable = true;
  set enable(bool enable) {
    if (enable != _enable) {
      _enable = enable;
      notifyListeners();
    }
  }

  bool get slowAnimations => _slowAnimations;
  bool _slowAnimations = false;
  set slowAnimations(bool slowAnimations) {
    if (slowAnimations != _slowAnimations) {
      _slowAnimations = slowAnimations;
      notifyListeners();
    }
  }

  bool get rtl => _rtl;
  bool _rtl = false;
  set rtl(bool rtl) {
    if (rtl != _rtl) {
      _rtl = rtl;
      notifyListeners();
    }
  }

  bool get longText => _longText;
  bool _longText = false;

  void reset() {
    final OptionModel defaultModel = OptionModel();
    _size = defaultModel.size;
    _enable = defaultModel.enable;
    _slowAnimations = defaultModel.slowAnimations;
    _longText = defaultModel.longText;
    _density = defaultModel.density;
    _rtl = defaultModel.rtl;
    notifyListeners();
  }
}

class LabeledCheckbox extends StatelessWidget {
  const LabeledCheckbox({super.key, required this.label, this.onChanged, this.value});

  final String label;
  final ValueChanged<bool?>? onChanged;
  final bool? value;

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[Checkbox(onChanged: onChanged, value: value), Text(label)],
    );
  }
}

class Options extends StatefulWidget {
  const Options(this.model, {super.key});

  final OptionModel model;

  @override
  State<Options> createState() => _OptionsState();
}

class _OptionsState extends State<Options> {
  @override
  void initState() {
    super.initState();
    widget.model.addListener(_modelChanged);
  }

  @override
  void didUpdateWidget(Options oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.model != oldWidget.model) {
      oldWidget.model.removeListener(_modelChanged);
      widget.model.addListener(_modelChanged);
    }
  }

  @override
  void dispose() {
    super.dispose();
    widget.model.removeListener(_modelChanged);
  }

  void _modelChanged() {
    setState(() {});
  }

  double sliderValue = 0.0;

  String _densityToProfile(VisualDensity density) {
    return switch (density) {
      VisualDensity.standard => 'standard',
      VisualDensity.compact => 'compact',
      VisualDensity.comfortable => 'comfortable',
      _ => 'custom',
    };
  }

  VisualDensity _profileToDensity(String? profile) {
    return switch (profile) {
      'standard' => VisualDensity.standard,
      'comfortable' => VisualDensity.comfortable,
      'compact' => VisualDensity.compact,
      'custom' || _ => widget.model.density,
    };
  }

  @override
  Widget build(BuildContext context) {
    final SliderThemeData controlTheme = SliderTheme.of(context).copyWith(
      thumbColor: Colors.grey[50],
      activeTickMarkColor: Colors.deepPurple[200],
      activeTrackColor: Colors.deepPurple[300],
      inactiveTrackColor: Colors.grey[50],
    );

    return Padding(
      padding: const EdgeInsets.fromLTRB(5.0, 0.0, 5.0, 10.0),
      child: Builder(
        builder: (BuildContext context) {
          return DefaultTextStyle(
            style: TextStyle(color: Colors.grey[50]),
            child: Column(
              children: <Widget>[
                Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: Row(
                    children: <Widget>[
                      const Text('Text Scale'),
                      Expanded(
                        child: SliderTheme(
                          data: controlTheme,
                          child: Slider(
                            label: '${widget.model.size}',
                            min: 0.5,
                            max: 3.0,
                            onChanged: (double value) {
                              widget.model.size = value;
                            },
                            value: widget.model.size,
                          ),
                        ),
                      ),
                      Text(
                        widget.model.size.toStringAsFixed(3),
                        style: TextStyle(color: Colors.grey[50]),
                      ),
                    ],
                  ),
                ),
                Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: Row(
                    children: <Widget>[
                      const Text('X Density'),
                      Expanded(
                        child: SliderTheme(
                          data: controlTheme,
                          child: Slider(
                            label: widget.model.density.horizontal.toStringAsFixed(1),
                            min: VisualDensity.minimumDensity,
                            max: VisualDensity.maximumDensity,
                            onChanged: (double value) {
                              widget.model.density = widget.model.density.copyWith(
                                horizontal: value,
                                vertical: widget.model.density.vertical,
                              );
                            },
                            value: widget.model.density.horizontal,
                          ),
                        ),
                      ),
                      Text(
                        widget.model.density.horizontal.toStringAsFixed(3),
                        style: TextStyle(color: Colors.grey[50]),
                      ),
                    ],
                  ),
                ),
                Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: Row(
                    children: <Widget>[
                      const Text('Y Density'),
                      Expanded(
                        child: SliderTheme(
                          data: controlTheme,
                          child: Slider(
                            label: widget.model.density.vertical.toStringAsFixed(1),
                            min: VisualDensity.minimumDensity,
                            max: VisualDensity.maximumDensity,
                            onChanged: (double value) {
                              widget.model.density = widget.model.density.copyWith(
                                horizontal: widget.model.density.horizontal,
                                vertical: value,
                              );
                            },
                            value: widget.model.density.vertical,
                          ),
                        ),
                      ),
                      Text(
                        widget.model.density.vertical.toStringAsFixed(3),
                        style: TextStyle(color: Colors.grey[50]),
                      ),
                    ],
                  ),
                ),
                Wrap(
                  alignment: WrapAlignment.center,
                  crossAxisAlignment: WrapCrossAlignment.center,
                  children: <Widget>[
                    Theme(
                      data: Theme.of(context).copyWith(canvasColor: Colors.grey[600]),
                      child: DropdownButton<String>(
                        style: TextStyle(color: Colors.grey[50]),
                        isDense: true,
                        onChanged: (String? value) {
                          widget.model.density = _profileToDensity(value);
                        },
                        items: const <DropdownMenuItem<String>>[
                          DropdownMenuItem<String>(value: 'standard', child: Text('Standard')),
                          DropdownMenuItem<String>(
                            value: 'comfortable',
                            child: Text('Comfortable'),
                          ),
                          DropdownMenuItem<String>(value: 'compact', child: Text('Compact')),
                          DropdownMenuItem<String>(value: 'custom', child: Text('Custom')),
                        ],
                        value: _densityToProfile(widget.model.density),
                      ),
                    ),
                    LabeledCheckbox(
                      label: 'Enabled',
                      onChanged: (bool? checked) {
                        widget.model.enable = checked ?? false;
                      },
                      value: widget.model.enable,
                    ),
                    LabeledCheckbox(
                      label: 'Slow',
                      onChanged: (bool? checked) {
                        widget.model.slowAnimations = checked ?? false;
                        Future<void>.delayed(const Duration(milliseconds: 150)).then((_) {
                          if (widget.model.slowAnimations) {
                            timeDilation = 20.0;
                          } else {
                            timeDilation = 1.0;
                          }
                        });
                      },
                      value: widget.model.slowAnimations,
                    ),
                    LabeledCheckbox(
                      label: 'RTL',
                      onChanged: (bool? checked) {
                        widget.model.rtl = checked ?? false;
                      },
                      value: widget.model.rtl,
                    ),
                    MaterialButton(
                      onPressed: () {
                        widget.model.reset();
                        sliderValue = 0.0;
                      },
                      child: Text('Reset', style: TextStyle(color: Colors.grey[50])),
                    ),
                  ],
                ),
              ],
            ),
          );
        },
      ),
    );
  }
}

class _ControlTile extends StatelessWidget {
  const _ControlTile({required this.label, required this.child});

  final String label;
  final Widget child;

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Padding(
        padding: const EdgeInsets.all(8.0),
        child: Column(
          children: <Widget>[
            Align(
              alignment: AlignmentDirectional.topStart,
              child: Text(label, textAlign: TextAlign.start),
            ),
            child,
          ],
        ),
      ),
    );
  }
}

class _MyHomePageState extends State<MyHomePage> {
  static final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
  final OptionModel _model = OptionModel();
  final TextEditingController textController = TextEditingController();

  @override
  void initState() {
    super.initState();
    _model.addListener(_modelChanged);
  }

  @override
  void dispose() {
    super.dispose();
    _model.removeListener(_modelChanged);
  }

  void _modelChanged() {
    setState(() {});
  }

  double sliderValue = 0.0;
  List<bool> checkboxValues = <bool>[false, false, false, false];
  List<IconData> iconValues = <IconData>[Icons.arrow_back, Icons.play_arrow, Icons.arrow_forward];
  List<String> chipValues = <String>['Potato', 'Computer'];
  int radioValue = 0;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = ThemeData(primarySwatch: m2Swatch);
    final Widget label = Text(_model.rtl ? 'اضغط علي' : 'Press Me');
    textController.text =
        _model.rtl
            ? 'يعتمد القرار الجيد على المعرفة وليس على الأرقام.'
            : 'A good decision is based on knowledge and not on numbers.';

    final List<Widget> tiles = <Widget>[
      _ControlTile(
        label: _model.rtl ? 'حقل النص' : 'List Tile',
        child: SizedBox(
          width: 400,
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              ListTile(
                title: Text(
                  _model.rtl ? 'هذا عنوان طويل نسبيا' : 'This is a relatively long title',
                ),
                onTap: () {},
              ),
              ListTile(
                title: Text(_model.rtl ? 'هذا عنوان قصير' : 'This is a short title'),
                subtitle: Text(
                  _model.rtl ? 'هذا عنوان فرعي مناسب.' : 'This is an appropriate subtitle.',
                ),
                trailing: const Icon(Icons.check_box),
                onTap: () {},
              ),
              ListTile(
                title: Text(_model.rtl ? 'هذا عنوان قصير' : 'This is a short title'),
                subtitle: Text(
                  _model.rtl ? 'هذا عنوان فرعي مناسب.' : 'This is an appropriate subtitle.',
                ),
                leading: const Icon(Icons.check_box),
                dense: true,
                onTap: () {},
              ),
              ListTile(
                title: Text(_model.rtl ? 'هذا عنوان قصير' : 'This is a short title'),
                subtitle: Text(
                  _model.rtl ? 'هذا عنوان فرعي مناسب.' : 'This is an appropriate subtitle.',
                ),
                dense: true,
                leading: const Icon(Icons.add_box),
                trailing: const Icon(Icons.check_box),
                onTap: () {},
              ),
              ListTile(
                title: Text(_model.rtl ? 'هذا عنوان قصير' : 'This is a short title'),
                subtitle: Text(
                  _model.rtl ? 'هذا عنوان فرعي مناسب.' : 'This is an appropriate subtitle.',
                ),
                isThreeLine: true,
                leading: const Icon(Icons.add_box),
                trailing: const Icon(Icons.check_box),
                onTap: () {},
              ),
            ],
          ),
        ),
      ),
      _ControlTile(
        label: _model.rtl ? 'حقل النص' : 'Text Field',
        child: SizedBox(
          width: 300,
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              TextField(
                controller: textController,
                decoration: const InputDecoration(
                  hintText: 'Hint',
                  helperText: 'Helper',
                  labelText: 'Label',
                  border: OutlineInputBorder(),
                ),
              ),
              TextField(controller: textController),
              TextField(controller: textController, maxLines: 3),
            ],
          ),
        ),
      ),
      _ControlTile(
        label: _model.rtl ? 'رقائق' : 'Chips',
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: List<Widget>.generate(chipValues.length, (int index) {
            return InputChip(
              onPressed: _model.enable ? () {} : null,
              onDeleted: _model.enable ? () {} : null,
              label: Text(chipValues[index]),
              deleteIcon: const Icon(Icons.delete),
              avatar: const Icon(Icons.play_arrow),
            );
          }),
        ),
      ),
      _ControlTile(
        label: _model.rtl ? 'زر المواد' : 'Material Button',
        child: MaterialButton(
          color: m2Swatch[200],
          onPressed: _model.enable ? () {} : null,
          child: label,
        ),
      ),
      _ControlTile(
        label: _model.rtl ? 'زر مسطح' : 'Text Button',
        child: TextButton(
          style: TextButton.styleFrom(
            foregroundColor: Colors.white,
            backgroundColor: m2Swatch[200],
          ),
          onPressed: _model.enable ? () {} : null,
          child: label,
        ),
      ),
      _ControlTile(
        label: _model.rtl ? 'أثارت زر' : 'Elevated Button',
        child: ElevatedButton(
          style: TextButton.styleFrom(backgroundColor: m2Swatch[200]),
          onPressed: _model.enable ? () {} : null,
          child: label,
        ),
      ),
      _ControlTile(
        label: _model.rtl ? 'زر المخطط التفصيلي' : 'Outlined Button',
        child: OutlinedButton(onPressed: _model.enable ? () {} : null, child: label),
      ),
      _ControlTile(
        label: _model.rtl ? 'خانات الاختيار' : 'Checkboxes',
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: List<Widget>.generate(checkboxValues.length, (int index) {
            return Checkbox(
              onChanged:
                  _model.enable
                      ? (bool? value) {
                        setState(() {
                          checkboxValues[index] = value ?? false;
                        });
                      }
                      : null,
              value: checkboxValues[index],
            );
          }),
        ),
      ),
      _ControlTile(
        label: _model.rtl ? 'زر الراديو' : 'Radio Button',
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: List<Widget>.generate(4, (int index) {
            return Radio<int>(
              onChanged:
                  _model.enable
                      ? (int? value) {
                        setState(() {
                          radioValue = value!;
                        });
                      }
                      : null,
              groupValue: radioValue,
              value: index,
            );
          }),
        ),
      ),
      _ControlTile(
        label: _model.rtl ? 'زر الأيقونة' : 'Icon Button',
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: List<Widget>.generate(iconValues.length, (int index) {
            return IconButton(
              onPressed: _model.enable ? () {} : null,
              icon: Icon(iconValues[index]),
            );
          }),
        ),
      ),
    ];

    return SafeArea(
      child: Theme(
        data: theme,
        child: Scaffold(
          key: scaffoldKey,
          appBar: AppBar(
            title: const Text('Density'),
            bottom: PreferredSize(
              preferredSize: const Size.fromHeight(220.0),
              child: Options(_model),
            ),
            backgroundColor: const Color(0xff323232),
          ),
          body: DefaultTextStyle(
            style: const TextStyle(
              color: Colors.black,
              fontSize: 14.0,
              fontFamily: 'Roboto',
              fontStyle: FontStyle.normal,
            ),
            child: Theme(
              data: Theme.of(context).copyWith(visualDensity: _model.density),
              child: Directionality(
                textDirection: _model.rtl ? TextDirection.rtl : TextDirection.ltr,
                child: Builder(
                  builder: (BuildContext context) {
                    final MediaQueryData mediaQueryData = MediaQuery.of(context);
                    return MediaQuery(
                      data: mediaQueryData.copyWith(textScaler: TextScaler.linear(_model.size)),
                      child: SizedBox.expand(child: ListView(children: tiles)),
                    );
                  },
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
