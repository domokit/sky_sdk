// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [TextButton].

void main() {
  runApp(const TextButtonExampleApp());
}

class TextButtonExampleApp extends StatefulWidget {
  const TextButtonExampleApp({super.key});

  @override
  State<TextButtonExampleApp> createState() => _TextButtonExampleAppState();
}

class _TextButtonExampleAppState extends State<TextButtonExampleApp> {
  bool darkMode = false;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      themeMode: darkMode ? ThemeMode.dark : ThemeMode.light,
      theme: ThemeData(brightness: Brightness.light),
      darkTheme: ThemeData(brightness: Brightness.dark),
      home: Scaffold(
        body: Padding(
          padding: const EdgeInsets.all(16),
          child: TextButtonExample(
            darkMode: darkMode,
            updateDarkMode: (bool value) {
              setState(() {
                darkMode = value;
              });
            },
          ),
        ),
      ),
    );
  }
}

class TextButtonExample extends StatefulWidget {
  const TextButtonExample({super.key, required this.darkMode, required this.updateDarkMode});

  final bool darkMode;
  final ValueChanged<bool> updateDarkMode;

  @override
  State<TextButtonExample> createState() => _TextButtonExampleState();
}

class _TextButtonExampleState extends State<TextButtonExample> {
  TextDirection textDirection = TextDirection.ltr;
  ThemeMode themeMode = ThemeMode.light;
  late final ScrollController scrollController;
  Future<void>? currentAction;

  static const Widget verticalSpacer = SizedBox(height: 16);
  static const Widget horizontalSpacer = SizedBox(width: 32);

  static const ImageProvider grassImage = NetworkImage(
    'https://flutter.github.io/assets-for-api-docs/assets/material/text_button_grass.jpeg',
  );
  static const ImageProvider defaultImage = NetworkImage(
    'https://flutter.github.io/assets-for-api-docs/assets/material/text_button_nhu_default.png',
  );
  static const ImageProvider hoveredImage = NetworkImage(
    'https://flutter.github.io/assets-for-api-docs/assets/material/text_button_nhu_hovered.png',
  );
  static const ImageProvider pressedImage = NetworkImage(
    'https://flutter.github.io/assets-for-api-docs/assets/material/text_button_nhu_pressed.png',
  );
  static const ImageProvider runningImage = NetworkImage(
    'https://flutter.github.io/assets-for-api-docs/assets/material/text_button_nhu_end.png',
  );

  @override
  void initState() {
    scrollController = ScrollController();
    super.initState();
  }

  @override
  void dispose() {
    scrollController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final ColorScheme colorScheme = theme.colorScheme;

    // Adapt colors that are not part of the color scheme to
    // the current dark/light mode. Used to define TextButton #7's
    // gradients.
    final (Color color1, Color color2, Color color3) = switch (colorScheme.brightness) {
      Brightness.light => (
        Colors.blue.withOpacity(1.0),
        Colors.orange.withOpacity(1.0),
        Colors.yellow.withOpacity(1.0),
      ),
      Brightness.dark => (
        Colors.purple.withOpacity(1.0),
        Colors.cyan.withOpacity(1.0),
        Colors.yellow.withOpacity(1.0),
      ),
    };

    // This gradient's appearance reflects the button's state.
    // Always return a gradient decoration so that AnimatedContainer
    // can interpolate in between. Used by TextButton #7.
    Decoration? statesToDecoration(Set<WidgetState> states) {
      if (states.contains(WidgetState.pressed)) {
        return BoxDecoration(
          gradient: LinearGradient(colors: <Color>[color2, color2]), // solid fill
        );
      }
      return BoxDecoration(
        gradient: LinearGradient(
          colors: switch (states.contains(WidgetState.hovered)) {
            true => <Color>[color1, color2],
            false => <Color>[color2, color1],
          },
        ),
      );
    }

    // To make this method a little easier to read, the buttons that
    // appear in the two columns to the right of the demo switches
    // Card are broken out below.

    final List<Widget> columnOneButtons = <Widget>[
      TextButton(onPressed: () {}, child: const Text('Enabled')),
      verticalSpacer,

      const TextButton(onPressed: null, child: Text('Disabled')),
      verticalSpacer,

      TextButton.icon(
        onPressed: () {},
        icon: const Icon(Icons.access_alarm),
        label: const Text('TextButton.icon #1'),
      ),
      verticalSpacer,

      // Override the foreground and background colors.
      //
      // In this example, and most of the ones that follow, we're using
      // the TextButton.styleFrom() convenience method to create a ButtonStyle.
      // The styleFrom method is a little easier because it creates
      // ButtonStyle WidgetStateProperty parameters for you.
      // In this case, Specifying foregroundColor overrides the text,
      // icon and overlay (splash and highlight) colors a little differently
      // depending on the button's state. BackgroundColor is just the background
      // color for all states.
      TextButton.icon(
        style: TextButton.styleFrom(
          foregroundColor: colorScheme.onError,
          backgroundColor: colorScheme.error,
        ),
        onPressed: () {},
        icon: const Icon(Icons.access_alarm),
        label: const Text('TextButton.icon #2'),
      ),
      verticalSpacer,

      // Override the button's shape and its border.
      //
      // In this case we've specified a shape that has border - the
      // RoundedRectangleBorder's side parameter. If the styleFrom
      // side parameter was also specified, or if the TextButtonTheme
      // defined above included a side parameter, then that would
      // override the RoundedRectangleBorder's side.
      TextButton(
        style: TextButton.styleFrom(
          shape: RoundedRectangleBorder(
            borderRadius: const BorderRadius.all(Radius.circular(8)),
            side: BorderSide(color: colorScheme.primary, width: 5),
          ),
        ),
        onPressed: () {},
        child: const Text('TextButton #3'),
      ),
      verticalSpacer,

      // Override overlay: the ink splash and highlight colors.
      //
      // The styleFrom method turns the specified overlayColor
      // into a value MaterialStyleProperty<Color> ButtonStyle.overlay
      // value that uses opacities depending on the button's state.
      // If the overlayColor was Colors.transparent, no splash
      // or highlights would be shown.
      TextButton(
        style: TextButton.styleFrom(overlayColor: Colors.yellow),
        onPressed: () {},
        child: const Text('TextButton #4'),
      ),
    ];

    final List<Widget> columnTwoButtons = <Widget>[
      // Override the foregroundBuilder: apply a ShaderMask.
      //
      // Apply a ShaderMask to the button's child. This kind of thing
      // can be applied to one button easily enough by just wrapping the
      // button's child directly. However to affect all buttons in this
      // way you can specify a similar foregroundBuilder in a TextButton
      // theme or the MaterialApp theme's ThemeData.textButtonTheme.
      TextButton(
        style: TextButton.styleFrom(
          foregroundBuilder: (BuildContext context, Set<WidgetState> states, Widget? child) {
            return ShaderMask(
              shaderCallback: (Rect bounds) {
                return LinearGradient(
                  begin: Alignment.bottomCenter,
                  end: Alignment.topCenter,
                  colors: <Color>[colorScheme.primary, colorScheme.onPrimary],
                ).createShader(bounds);
              },
              blendMode: BlendMode.srcATop,
              child: child,
            );
          },
        ),
        onPressed: () {},
        child: const Text('TextButton #5'),
      ),
      verticalSpacer,

      // Override the foregroundBuilder: add an underline.
      //
      // Add a border around button's child. In this case the
      // border only appears when the button is hovered or pressed
      // (if it's pressed it's always hovered too). Not that this
      // border is different than the one specified with the styleFrom
      // side parameter (or the ButtonStyle.side property). The foregroundBuilder
      // is applied to a widget that contains the child and has already
      // included the button's padding. It is unaffected by the button's shape.
      // The styleFrom side parameter controls the button's outermost border and it
      // outlines the button's shape.
      TextButton(
        style: TextButton.styleFrom(
          foregroundBuilder: (BuildContext context, Set<WidgetState> states, Widget? child) {
            return DecoratedBox(
              decoration: BoxDecoration(
                border:
                    states.contains(WidgetState.hovered)
                        ? Border(bottom: BorderSide(color: colorScheme.primary))
                        : const Border(), // essentially "no border"
              ),
              child: child,
            );
          },
        ),
        onPressed: () {},
        child: const Text('TextButton #6'),
      ),
      verticalSpacer,

      // Override the backgroundBuilder to add a state specific gradient background
      // and add an outline that only appears when the button is hovered or pressed.
      //
      // The gradient background decoration is computed by the statesToDecoration()
      // method. The gradient flips horizontally when the button is hovered (watch
      // closely). Because we want the outline to only appear when the button is hovered
      // we can't use the styleFrom() side parameter, because that creates the same
      // outline for all states. The ButtonStyle.copyWith() method is used to add
      // a WidgetState<BorderSide?> property that does the right thing.
      //
      // The gradient background is translucent - all of the colors have opacity 0.5 -
      // so the overlay's splash and highlight colors are visible even though they're
      // drawn on the Material widget that's effectively behind the background. The
      // border is also translucent, so if you look carefully, you'll see that the
      // background - which is part of the button's Material but is drawn on top of the
      // the background gradient - shows through the border.
      TextButton(
        onPressed: () {},
        style: TextButton.styleFrom(
          overlayColor: color2,
          backgroundBuilder: (BuildContext context, Set<WidgetState> states, Widget? child) {
            return AnimatedContainer(
              duration: const Duration(milliseconds: 500),
              decoration: statesToDecoration(states),
              child: child,
            );
          },
        ).copyWith(
          side: WidgetStateProperty.resolveWith<BorderSide?>((Set<WidgetState> states) {
            if (states.contains(WidgetState.hovered)) {
              return BorderSide(width: 3, color: color3);
            }
            return null; // defer to the default
          }),
        ),
        child: const Text('TextButton #7'),
      ),
      verticalSpacer,

      // Override the backgroundBuilder to add a grass image background.
      //
      // The image is clipped to the button's shape. We've included an Ink widget
      // because the background image is opaque and would otherwise obscure the splash
      // and highlight overlays that are painted on the button's Material widget
      // by default. They're drawn on the Ink widget instead. The foreground color
      // was overridden as well because white shows up a little better on the mottled
      // green background.
      TextButton(
        onPressed: () {},
        style: TextButton.styleFrom(
          foregroundColor: Colors.white,
          backgroundBuilder: (BuildContext context, Set<WidgetState> states, Widget? child) {
            return Ink(
              decoration: const BoxDecoration(
                image: DecorationImage(image: grassImage, fit: BoxFit.cover),
              ),
              child: child,
            );
          },
        ),
        child: const Text('TextButton #8'),
      ),
      verticalSpacer,

      // Override the foregroundBuilder to specify images for the button's pressed
      // hovered and default states. We switch to an additional image while the
      // button's callback is "running".
      //
      // This is an example of completely changing the default appearance of a button
      // by specifying images for each state and by turning off the overlays by
      // overlayColor: Colors.transparent. AnimatedContainer takes care of the
      // fade in and out segues between images.
      //
      // This foregroundBuilder function ignores its child parameter. Unfortunately
      // TextButton's child parameter is required, so we still have
      // to provide one.
      TextButton(
        onPressed: () async {
          // This is slightly complicated so that if the user presses the button
          // while the current Future.delayed action is running, the currentAction
          // flag is only reset to null after the _new_ action completes.
          late final Future<void> thisAction;
          thisAction = Future<void>.delayed(const Duration(seconds: 1), () {
            if (currentAction == thisAction) {
              setState(() {
                currentAction = null;
              });
            }
          });
          setState(() {
            currentAction = thisAction;
          });
        },
        style: TextButton.styleFrom(
          overlayColor: Colors.transparent,
          foregroundBuilder: (BuildContext context, Set<WidgetState> states, Widget? child) {
            late final ImageProvider image;
            if (currentAction != null) {
              image = runningImage;
            } else if (states.contains(WidgetState.pressed)) {
              image = pressedImage;
            } else if (states.contains(WidgetState.hovered)) {
              image = hoveredImage;
            } else {
              image = defaultImage;
            }
            return AnimatedContainer(
              width: 64,
              height: 64,
              duration: const Duration(milliseconds: 300),
              curve: Curves.fastOutSlowIn,
              decoration: BoxDecoration(image: DecorationImage(image: image, fit: BoxFit.contain)),
            );
          },
        ),
        child: const Text('This child is not used'),
      ),
    ];

    return Row(
      children: <Widget>[
        // The dark/light and LTR/RTL switches. We use the updateDarkMode function
        // provided by the parent TextButtonExampleApp to rebuild the MaterialApp
        // in the appropriate dark/light ThemeMdoe. The directionality of the rest
        // of the UI is controlled by the Directionality widget below, and the
        // textDirection local state variable.
        TextButtonExampleSwitches(
          darkMode: widget.darkMode,
          updateDarkMode: widget.updateDarkMode,
          textDirection: textDirection,
          updateRTL: (bool value) {
            setState(() {
              textDirection = value ? TextDirection.rtl : TextDirection.ltr;
            });
          },
        ),
        horizontalSpacer,

        Expanded(
          child: Scrollbar(
            controller: scrollController,
            thumbVisibility: true,
            child: SingleChildScrollView(
              scrollDirection: Axis.horizontal,
              controller: scrollController,
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                mainAxisSize: MainAxisSize.min,
                children: <Widget>[
                  Directionality(
                    textDirection: textDirection,
                    child: Column(children: columnOneButtons),
                  ),
                  horizontalSpacer,

                  Directionality(
                    textDirection: textDirection,
                    child: Column(children: columnTwoButtons),
                  ),
                  horizontalSpacer,
                ],
              ),
            ),
          ),
        ),
      ],
    );
  }
}

class TextButtonExampleSwitches extends StatelessWidget {
  const TextButtonExampleSwitches({
    super.key,
    required this.darkMode,
    required this.updateDarkMode,
    required this.textDirection,
    required this.updateRTL,
  });

  final bool darkMode;
  final ValueChanged<bool> updateDarkMode;
  final TextDirection textDirection;
  final ValueChanged<bool> updateRTL;

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: IntrinsicWidth(
          child: Column(
            children: <Widget>[
              Row(
                children: <Widget>[
                  const Expanded(child: Text('Dark Mode')),
                  const SizedBox(width: 4),
                  Switch(value: darkMode, onChanged: updateDarkMode),
                ],
              ),
              const SizedBox(height: 16),
              Row(
                children: <Widget>[
                  const Expanded(child: Text('RTL Text')),
                  const SizedBox(width: 4),
                  Switch(value: textDirection == TextDirection.rtl, onChanged: updateRTL),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }
}
