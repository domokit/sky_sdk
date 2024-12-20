// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ColorScheme.fromImageProvider] with content-based dynamic color.

const Widget divider = SizedBox(height: 10);
const double narrowScreenWidthThreshold = 400;

void main() => runApp(const DynamicColorExample());

class DynamicColorExample extends StatefulWidget {
  const DynamicColorExample({this.loadColorScheme, super.key});

  static const List<ImageProvider> images = <NetworkImage>[
    NetworkImage(
      'https://flutter.github.io/assets-for-api-docs/assets/material/content_based_color_scheme_1.png',
    ),
    NetworkImage(
      'https://flutter.github.io/assets-for-api-docs/assets/material/content_based_color_scheme_2.png',
    ),
    NetworkImage(
      'https://flutter.github.io/assets-for-api-docs/assets/material/content_based_color_scheme_3.png',
    ),
    NetworkImage(
      'https://flutter.github.io/assets-for-api-docs/assets/material/content_based_color_scheme_4.png',
    ),
    NetworkImage(
      'https://flutter.github.io/assets-for-api-docs/assets/material/content_based_color_scheme_5.png',
    ),
    NetworkImage(
      'https://flutter.github.io/assets-for-api-docs/assets/material/content_based_color_scheme_6.png',
    ),
  ];

  final Future<ColorScheme> Function(ImageProvider<Object> provider, Brightness brightness)?
  loadColorScheme;

  @override
  State<DynamicColorExample> createState() => _DynamicColorExampleState();
}

class _DynamicColorExampleState extends State<DynamicColorExample> {
  late ColorScheme currentColorScheme;
  String currentHyperlinkImage = '';
  late int selectedImage;
  late bool isLight;
  late bool isLoading;

  @override
  void initState() {
    super.initState();
    selectedImage = 0;
    isLight = true;
    isLoading = true;
    currentColorScheme = const ColorScheme.light();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _updateImage(DynamicColorExample.images[selectedImage]);
      isLoading = false;
    });
  }

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = currentColorScheme;
    final Color selectedColor = currentColorScheme.primary;

    final ThemeData lightTheme = ThemeData(
      colorSchemeSeed: selectedColor,
      brightness: Brightness.light,
      useMaterial3: false,
    );
    final ThemeData darkTheme = ThemeData(
      colorSchemeSeed: selectedColor,
      brightness: Brightness.dark,
      useMaterial3: false,
    );

    Widget schemeLabel(String brightness, ColorScheme colorScheme) {
      return Padding(
        padding: const EdgeInsets.symmetric(vertical: 15),
        child: Text(
          brightness,
          style: TextStyle(fontWeight: FontWeight.bold, color: colorScheme.onSecondaryContainer),
        ),
      );
    }

    Widget schemeView(ThemeData theme) {
      return Padding(
        padding: const EdgeInsets.symmetric(horizontal: 15),
        child: ColorSchemeView(colorScheme: theme.colorScheme),
      );
    }

    return MaterialApp(
      theme: ThemeData(useMaterial3: true, colorScheme: colorScheme),
      debugShowCheckedModeBanner: false,
      home: Builder(
        builder:
            (BuildContext context) => Scaffold(
              appBar: AppBar(
                title: const Text('Content Based Dynamic Color'),
                backgroundColor: colorScheme.primary,
                foregroundColor: colorScheme.onPrimary,
                actions: <Widget>[
                  const Icon(Icons.light_mode),
                  Switch(
                    activeColor: colorScheme.primary,
                    activeTrackColor: colorScheme.surface,
                    inactiveTrackColor: colorScheme.onSecondary,
                    value: isLight,
                    onChanged: (bool value) {
                      setState(() {
                        isLight = value;
                        _updateImage(DynamicColorExample.images[selectedImage]);
                      });
                    },
                  ),
                ],
              ),
              body: Center(
                child:
                    isLoading
                        ? const CircularProgressIndicator()
                        : ColoredBox(
                          color: colorScheme.secondaryContainer,
                          child: Column(
                            children: <Widget>[
                              divider,
                              _imagesRow(context, DynamicColorExample.images, colorScheme),
                              divider,
                              Expanded(
                                child: ColoredBox(
                                  color: colorScheme.surface,
                                  child: LayoutBuilder(
                                    builder: (BuildContext context, BoxConstraints constraints) {
                                      if (constraints.maxWidth < narrowScreenWidthThreshold) {
                                        return SingleChildScrollView(
                                          child: Column(
                                            children: <Widget>[
                                              divider,
                                              schemeLabel('Light ColorScheme', colorScheme),
                                              schemeView(lightTheme),
                                              divider,
                                              divider,
                                              schemeLabel('Dark ColorScheme', colorScheme),
                                              schemeView(darkTheme),
                                            ],
                                          ),
                                        );
                                      } else {
                                        return SingleChildScrollView(
                                          child: Padding(
                                            padding: const EdgeInsets.only(top: 5),
                                            child: Column(
                                              children: <Widget>[
                                                Row(
                                                  children: <Widget>[
                                                    Expanded(
                                                      child: Column(
                                                        children: <Widget>[
                                                          schemeLabel(
                                                            'Light ColorScheme',
                                                            colorScheme,
                                                          ),
                                                          schemeView(lightTheme),
                                                        ],
                                                      ),
                                                    ),
                                                    Expanded(
                                                      child: Column(
                                                        children: <Widget>[
                                                          schemeLabel(
                                                            'Dark ColorScheme',
                                                            colorScheme,
                                                          ),
                                                          schemeView(darkTheme),
                                                        ],
                                                      ),
                                                    ),
                                                  ],
                                                ),
                                              ],
                                            ),
                                          ),
                                        );
                                      }
                                    },
                                  ),
                                ),
                              ),
                            ],
                          ),
                        ),
              ),
            ),
      ),
    );
  }

  Future<void> _updateImage(ImageProvider provider) async {
    final ColorScheme newColorScheme;
    if (widget.loadColorScheme != null) {
      newColorScheme = await widget.loadColorScheme!(
        provider,
        isLight ? Brightness.light : Brightness.dark,
      );
    } else {
      newColorScheme = await ColorScheme.fromImageProvider(
        provider: provider,
        brightness: isLight ? Brightness.light : Brightness.dark,
      );
    }
    if (!mounted) {
      return;
    }
    setState(() {
      selectedImage = DynamicColorExample.images.indexOf(provider);
      currentColorScheme = newColorScheme;
    });
  }

  // For small screens, have two rows of image selection. For wide screens,
  // fit them onto one row.
  Widget _imagesRow(BuildContext context, List<ImageProvider> images, ColorScheme colorScheme) {
    final double windowHeight = MediaQuery.of(context).size.height;
    final double windowWidth = MediaQuery.of(context).size.width;
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: LayoutBuilder(
        builder: (BuildContext context, BoxConstraints constraints) {
          if (constraints.maxWidth > 800) {
            return _adaptiveLayoutImagesRow(images, colorScheme, windowHeight);
          } else {
            return Column(
              children: <Widget>[
                _adaptiveLayoutImagesRow(images.sublist(0, 3), colorScheme, windowWidth),
                _adaptiveLayoutImagesRow(images.sublist(3), colorScheme, windowWidth),
              ],
            );
          }
        },
      ),
    );
  }

  Widget _adaptiveLayoutImagesRow(
    List<ImageProvider> images,
    ColorScheme colorScheme,
    double windowWidth,
  ) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.center,
      children:
          images
              .map(
                (ImageProvider image) => Flexible(
                  flex: (images.length / 3).floor(),
                  child: GestureDetector(
                    onTap: () => _updateImage(image),
                    child: Card(
                      color:
                          DynamicColorExample.images.indexOf(image) == selectedImage
                              ? colorScheme.primaryContainer
                              : colorScheme.surface,
                      child: Padding(
                        padding: const EdgeInsets.all(5.0),
                        child: ConstrainedBox(
                          constraints: BoxConstraints(maxWidth: windowWidth * .25),
                          child: ClipRRect(
                            borderRadius: BorderRadius.circular(8.0),
                            child: Image(image: image),
                          ),
                        ),
                      ),
                    ),
                  ),
                ),
              )
              .toList(),
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
            ColorChip(label: 'primary', color: colorScheme.primary, onColor: colorScheme.onPrimary),
            ColorChip(
              label: 'onPrimary',
              color: colorScheme.onPrimary,
              onColor: colorScheme.primary,
            ),
            ColorChip(
              label: 'primaryContainer',
              color: colorScheme.primaryContainer,
              onColor: colorScheme.onPrimaryContainer,
            ),
            ColorChip(
              label: 'onPrimaryContainer',
              color: colorScheme.onPrimaryContainer,
              onColor: colorScheme.primaryContainer,
            ),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip(
              label: 'secondary',
              color: colorScheme.secondary,
              onColor: colorScheme.onSecondary,
            ),
            ColorChip(
              label: 'onSecondary',
              color: colorScheme.onSecondary,
              onColor: colorScheme.secondary,
            ),
            ColorChip(
              label: 'secondaryContainer',
              color: colorScheme.secondaryContainer,
              onColor: colorScheme.onSecondaryContainer,
            ),
            ColorChip(
              label: 'onSecondaryContainer',
              color: colorScheme.onSecondaryContainer,
              onColor: colorScheme.secondaryContainer,
            ),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip(
              label: 'tertiary',
              color: colorScheme.tertiary,
              onColor: colorScheme.onTertiary,
            ),
            ColorChip(
              label: 'onTertiary',
              color: colorScheme.onTertiary,
              onColor: colorScheme.tertiary,
            ),
            ColorChip(
              label: 'tertiaryContainer',
              color: colorScheme.tertiaryContainer,
              onColor: colorScheme.onTertiaryContainer,
            ),
            ColorChip(
              label: 'onTertiaryContainer',
              color: colorScheme.onTertiaryContainer,
              onColor: colorScheme.tertiaryContainer,
            ),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip(label: 'error', color: colorScheme.error, onColor: colorScheme.onError),
            ColorChip(label: 'onError', color: colorScheme.onError, onColor: colorScheme.error),
            ColorChip(
              label: 'errorContainer',
              color: colorScheme.errorContainer,
              onColor: colorScheme.onErrorContainer,
            ),
            ColorChip(
              label: 'onErrorContainer',
              color: colorScheme.onErrorContainer,
              onColor: colorScheme.errorContainer,
            ),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip(label: 'surface', color: colorScheme.surface, onColor: colorScheme.onSurface),
            ColorChip(
              label: 'onSurface',
              color: colorScheme.onSurface,
              onColor: colorScheme.surface,
            ),
            ColorChip(
              label: 'onSurfaceVariant',
              color: colorScheme.onSurfaceVariant,
              onColor: colorScheme.surfaceContainerHighest,
            ),
          ],
        ),
        divider,
        ColorGroup(
          children: <ColorChip>[
            ColorChip(label: 'outline', color: colorScheme.outline),
            ColorChip(label: 'shadow', color: colorScheme.shadow),
            ColorChip(
              label: 'inverseSurface',
              color: colorScheme.inverseSurface,
              onColor: colorScheme.onInverseSurface,
            ),
            ColorChip(
              label: 'onInverseSurface',
              color: colorScheme.onInverseSurface,
              onColor: colorScheme.inverseSurface,
            ),
            ColorChip(
              label: 'inversePrimary',
              color: colorScheme.inversePrimary,
              onColor: colorScheme.primary,
            ),
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
  const ColorChip({super.key, required this.color, required this.label, this.onColor});

  final Color color;
  final Color? onColor;
  final String label;

  static Color contrastColor(Color color) {
    final Brightness brightness = ThemeData.estimateBrightnessForColor(color);
    return switch (brightness) {
      Brightness.dark => Colors.white,
      Brightness.light => Colors.black,
    };
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
