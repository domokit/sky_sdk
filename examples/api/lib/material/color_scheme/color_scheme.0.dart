// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ColorScheme].

const Widget divider = SizedBox(height: 10);

void main() => runApp(const ColorSchemeExample());

class ColorSchemeExample extends StatefulWidget {
  const ColorSchemeExample({super.key});

  @override
  State<ColorSchemeExample> createState() => _ColorSchemeExampleState();
}

class _ColorSchemeExampleState extends State<ColorSchemeExample> {
  Color selectedColor = ColorSeed.baseColor.color;
  Brightness selectedBrightness = Brightness.light;
  double selectedContrast = 0.0;
  static const List<DynamicSchemeVariant> schemeVariants = DynamicSchemeVariant.values;

  void updateTheme(Brightness brightness, Color color, double contrastLevel) {
    setState(() {
      selectedBrightness = brightness;
      selectedColor = color;
      selectedContrast = contrastLevel;
    });
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(
          seedColor: selectedColor,
          brightness: selectedBrightness,
          contrastLevel: selectedContrast,
        ),
      ),
      home: Scaffold(
        appBar: AppBar(
          title: const Text('ColorScheme'),
          actions: <Widget>[
            SettingsButton(
              selectedColor: selectedColor,
              selectedBrightness: selectedBrightness,
              selectedContrast: selectedContrast,
              updateTheme: updateTheme,
            ),
          ],
        ),
        body: SingleChildScrollView(
          child: Padding(
            padding: const EdgeInsets.only(top: 5),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                SingleChildScrollView(
                  scrollDirection: Axis.horizontal,
                  child: Row(
                    children:
                        List<Widget>.generate(schemeVariants.length, (int index) {
                          return ColorSchemeVariantColumn(
                            selectedColor: selectedColor,
                            brightness: selectedBrightness,
                            schemeVariant: schemeVariants[index],
                            contrastLevel: selectedContrast,
                          );
                        }).toList(),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class Settings extends StatefulWidget {
  const Settings({
    super.key,
    required this.updateTheme,
    required this.selectedBrightness,
    required this.selectedContrast,
    required this.selectedColor,
  });

  final Brightness selectedBrightness;
  final double selectedContrast;
  final Color selectedColor;

  final void Function(Brightness, Color, double) updateTheme;

  @override
  State<Settings> createState() => _SettingsState();
}

class _SettingsState extends State<Settings> {
  late Brightness selectedBrightness = widget.selectedBrightness;
  late Color selectedColor = widget.selectedColor;
  late double selectedContrast = widget.selectedContrast;

  @override
  Widget build(BuildContext context) {
    return Theme(
      data: Theme.of(context).copyWith(
        colorScheme: ColorScheme.fromSeed(
          seedColor: selectedColor,
          contrastLevel: selectedContrast,
          brightness: selectedBrightness,
        ),
      ),
      child: ConstrainedBox(
        constraints: const BoxConstraints(maxHeight: 200),
        child: Padding(
          padding: const EdgeInsets.all(20.0),
          child: ListView(
            children: <Widget>[
              Center(child: Text('Settings', style: Theme.of(context).textTheme.titleLarge)),
              Row(
                children: <Widget>[
                  const Text('Brightness: '),
                  Switch(
                    value: selectedBrightness == Brightness.light,
                    onChanged: (bool value) {
                      setState(() {
                        selectedBrightness = value ? Brightness.light : Brightness.dark;
                      });
                      widget.updateTheme(selectedBrightness, selectedColor, selectedContrast);
                    },
                  ),
                ],
              ),
              Wrap(
                crossAxisAlignment: WrapCrossAlignment.center,
                children: <Widget>[
                  const Text('Seed color: '),
                  ...List<Widget>.generate(ColorSeed.values.length, (int index) {
                    final Color itemColor = ColorSeed.values[index].color;
                    return IconButton(
                      icon:
                          selectedColor == ColorSeed.values[index].color
                              ? Icon(Icons.circle, color: itemColor)
                              : Icon(Icons.circle_outlined, color: itemColor),
                      onPressed: () {
                        setState(() {
                          selectedColor = itemColor;
                        });
                        widget.updateTheme(selectedBrightness, selectedColor, selectedContrast);
                      },
                    );
                  }),
                ],
              ),
              Row(
                children: <Widget>[
                  const Text('Contrast level: '),
                  Expanded(
                    child: Slider(
                      divisions: 4,
                      label: selectedContrast.toString(),
                      min: -1,
                      value: selectedContrast,
                      onChanged: (double value) {
                        setState(() {
                          selectedContrast = value;
                        });
                        widget.updateTheme(selectedBrightness, selectedColor, selectedContrast);
                      },
                    ),
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class ColorSchemeVariantColumn extends StatelessWidget {
  const ColorSchemeVariantColumn({
    super.key,
    this.schemeVariant = DynamicSchemeVariant.tonalSpot,
    this.brightness = Brightness.light,
    this.contrastLevel = 0.0,
    required this.selectedColor,
  });

  final DynamicSchemeVariant schemeVariant;
  final Brightness brightness;
  final double contrastLevel;
  final Color selectedColor;

  @override
  Widget build(BuildContext context) {
    return ConstrainedBox(
      constraints: const BoxConstraints.tightFor(width: 250),
      child: Column(
        children: <Widget>[
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 15),
            child: Text(
              schemeVariant.name == 'tonalSpot'
                  ? '${schemeVariant.name} (Default)'
                  : schemeVariant.name,
              style: const TextStyle(fontWeight: FontWeight.bold),
            ),
          ),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 15),
            child: ColorSchemeView(
              colorScheme: ColorScheme.fromSeed(
                seedColor: selectedColor,
                brightness: brightness,
                contrastLevel: contrastLevel,
                dynamicSchemeVariant: schemeVariant,
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class ColorSchemeView extends StatelessWidget {
  const ColorSchemeView({super.key, required this.colorScheme});

  final ColorScheme colorScheme;

  @override
  Widget build(BuildContext context) {
    return Column(
      children: <Widget>[
        ColorGroup(
          children: <ColorChip>[
            ColorChip('primary', colorScheme.primary, colorScheme.onPrimary),
            ColorChip('onPrimary', colorScheme.onPrimary, colorScheme.primary),
            ColorChip(
              'primaryContainer',
              colorScheme.primaryContainer,
              colorScheme.onPrimaryContainer,
            ),
            ColorChip(
              'onPrimaryContainer',
              colorScheme.onPrimaryContainer,
              colorScheme.primaryContainer,
            ),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip('primaryFixed', colorScheme.primaryFixed, colorScheme.onPrimaryFixed),
            ColorChip('onPrimaryFixed', colorScheme.onPrimaryFixed, colorScheme.primaryFixed),
            ColorChip(
              'primaryFixedDim',
              colorScheme.primaryFixedDim,
              colorScheme.onPrimaryFixedVariant,
            ),
            ColorChip(
              'onPrimaryFixedVariant',
              colorScheme.onPrimaryFixedVariant,
              colorScheme.primaryFixedDim,
            ),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip('secondary', colorScheme.secondary, colorScheme.onSecondary),
            ColorChip('onSecondary', colorScheme.onSecondary, colorScheme.secondary),
            ColorChip(
              'secondaryContainer',
              colorScheme.secondaryContainer,
              colorScheme.onSecondaryContainer,
            ),
            ColorChip(
              'onSecondaryContainer',
              colorScheme.onSecondaryContainer,
              colorScheme.secondaryContainer,
            ),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip('secondaryFixed', colorScheme.secondaryFixed, colorScheme.onSecondaryFixed),
            ColorChip('onSecondaryFixed', colorScheme.onSecondaryFixed, colorScheme.secondaryFixed),
            ColorChip(
              'secondaryFixedDim',
              colorScheme.secondaryFixedDim,
              colorScheme.onSecondaryFixedVariant,
            ),
            ColorChip(
              'onSecondaryFixedVariant',
              colorScheme.onSecondaryFixedVariant,
              colorScheme.secondaryFixedDim,
            ),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip('tertiary', colorScheme.tertiary, colorScheme.onTertiary),
            ColorChip('onTertiary', colorScheme.onTertiary, colorScheme.tertiary),
            ColorChip(
              'tertiaryContainer',
              colorScheme.tertiaryContainer,
              colorScheme.onTertiaryContainer,
            ),
            ColorChip(
              'onTertiaryContainer',
              colorScheme.onTertiaryContainer,
              colorScheme.tertiaryContainer,
            ),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip('tertiaryFixed', colorScheme.tertiaryFixed, colorScheme.onTertiaryFixed),
            ColorChip('onTertiaryFixed', colorScheme.onTertiaryFixed, colorScheme.tertiaryFixed),
            ColorChip(
              'tertiaryFixedDim',
              colorScheme.tertiaryFixedDim,
              colorScheme.onTertiaryFixedVariant,
            ),
            ColorChip(
              'onTertiaryFixedVariant',
              colorScheme.onTertiaryFixedVariant,
              colorScheme.tertiaryFixedDim,
            ),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip('error', colorScheme.error, colorScheme.onError),
            ColorChip('onError', colorScheme.onError, colorScheme.error),
            ColorChip('errorContainer', colorScheme.errorContainer, colorScheme.onErrorContainer),
            ColorChip('onErrorContainer', colorScheme.onErrorContainer, colorScheme.errorContainer),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip('surfaceDim', colorScheme.surfaceDim, colorScheme.onSurface),
            ColorChip('surface', colorScheme.surface, colorScheme.onSurface),
            ColorChip('surfaceBright', colorScheme.surfaceBright, colorScheme.onSurface),
            ColorChip(
              'surfaceContainerLowest',
              colorScheme.surfaceContainerLowest,
              colorScheme.onSurface,
            ),
            ColorChip(
              'surfaceContainerLow',
              colorScheme.surfaceContainerLow,
              colorScheme.onSurface,
            ),
            ColorChip('surfaceContainer', colorScheme.surfaceContainer, colorScheme.onSurface),
            ColorChip(
              'surfaceContainerHigh',
              colorScheme.surfaceContainerHigh,
              colorScheme.onSurface,
            ),
            ColorChip(
              'surfaceContainerHighest',
              colorScheme.surfaceContainerHighest,
              colorScheme.onSurface,
            ),
            ColorChip('onSurface', colorScheme.onSurface, colorScheme.surface),
            ColorChip(
              'onSurfaceVariant',
              colorScheme.onSurfaceVariant,
              colorScheme.surfaceContainerHighest,
            ),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip('outline', colorScheme.outline, null),
            ColorChip('shadow', colorScheme.shadow, null),
            ColorChip('inverseSurface', colorScheme.inverseSurface, colorScheme.onInverseSurface),
            ColorChip('onInverseSurface', colorScheme.onInverseSurface, colorScheme.inverseSurface),
            ColorChip('inversePrimary', colorScheme.inversePrimary, colorScheme.primary),
          ],
        ),
      ],
    );
  }
}

class ColorGroup extends StatelessWidget {
  const ColorGroup({super.key, required this.children});

  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    return RepaintBoundary(
      child: Card(clipBehavior: Clip.antiAlias, child: Column(children: children)),
    );
  }
}

class ColorChip extends StatelessWidget {
  const ColorChip(this.label, this.color, this.onColor, {super.key});

  final Color color;
  final Color? onColor;
  final String label;

  static Color contrastColor(Color color) {
    final Brightness brightness = ThemeData.estimateBrightnessForColor(color);
    return brightness == Brightness.dark ? Colors.white : Colors.black;
  }

  @override
  Widget build(BuildContext context) {
    final Color labelColor = onColor ?? contrastColor(color);
    return ColoredBox(
      color: color,
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Row(
          children: <Expanded>[Expanded(child: Text(label, style: TextStyle(color: labelColor)))],
        ),
      ),
    );
  }
}

enum ColorSeed {
  baseColor('M3 Baseline', Color(0xff6750a4)),
  indigo('Indigo', Colors.indigo),
  blue('Blue', Colors.blue),
  teal('Teal', Colors.teal),
  green('Green', Colors.green),
  yellow('Yellow', Colors.yellow),
  orange('Orange', Colors.orange),
  deepOrange('Deep Orange', Colors.deepOrange),
  pink('Pink', Colors.pink),
  brightBlue('Bright Blue', Color(0xFF0000FF)),
  brightGreen('Bright Green', Color(0xFF00FF00)),
  brightRed('Bright Red', Color(0xFFFF0000));

  const ColorSeed(this.label, this.color);
  final String label;
  final Color color;
}

class SettingsButton extends StatelessWidget {
  const SettingsButton({
    super.key,
    required this.updateTheme,
    required this.selectedBrightness,
    required this.selectedContrast,
    required this.selectedColor,
  });

  final Brightness selectedBrightness;
  final double selectedContrast;
  final Color selectedColor;

  final void Function(Brightness, Color, double) updateTheme;

  @override
  Widget build(BuildContext context) {
    return IconButton(
      icon: const Icon(Icons.settings),
      onPressed: () {
        showModalBottomSheet<void>(
          barrierColor: Colors.transparent,
          context: context,
          builder: (BuildContext context) {
            return Settings(
              selectedColor: selectedColor,
              selectedBrightness: selectedBrightness,
              selectedContrast: selectedContrast,
              updateTheme: updateTheme,
            );
          },
        );
      },
    );
  }
}
