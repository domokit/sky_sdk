// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class FortnightlyDemo extends StatelessWidget {
  const FortnightlyDemo({super.key});

  static const String routeName = '/fortnightly';

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Fortnightly Demo',
      theme: _fortnightlyTheme,
      home: Scaffold(
        body: Stack(
          children: <Widget>[
            const FruitPage(),
            SafeArea(
              child: ShortAppBar(
                onBackPressed: () {
                  Navigator.pop(context);
                },
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class ShortAppBar extends StatelessWidget {
  const ShortAppBar({super.key, this.onBackPressed});

  final VoidCallback? onBackPressed;

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: 96,
      height: 50,
      child: Material(
        color: Theme.of(context).colorScheme.surface,
        elevation: 4,
        shape: const BeveledRectangleBorder(
          borderRadius: BorderRadius.only(bottomRight: Radius.circular(22)),
        ),
        child: Row(
          children: <Widget>[
            IconButton(
              icon: const Icon(Icons.arrow_back),
              tooltip: 'Back',
              onPressed: onBackPressed,
            ),
            const SizedBox(width: 12),
            Image.asset(
              'logos/fortnightly/fortnightly_logo.png',
              package: 'flutter_gallery_assets',
            ),
          ],
        ),
      ),
    );
  }
}

class FruitPage extends StatelessWidget {
  const FruitPage({super.key});

  static final String paragraph1 = '''
Have you ever held a quince? It's strange;
covered in a fuzz somewhere between peach skin and a spider web. And it's
hard as soft lumber. You'd be forgiven for thinking it's veneered Larch-wood.
But inhale the aroma and you'll instantly know you have something wonderful.
Its scent can fill a room for days. And all this before you've even cooked it.
'''.replaceAll('\n', ' ');

  static final String paragraph2 = '''
Pomegranates on the other hand have become
almost ubiquitous. You can find its juice in any bodega, Walmart, and even some
gas stations. But at what cost? The pomegranate juice craze of the aughts made
"megafarmers" Lynda and Stewart Resnick billions. Unfortunately, it takes a lot
of water to make that much pomegranate juice. Water the Resnicks get from their
majority stake in the Kern Water Bank. How did one family come to hold control
over water meant for the whole central valley of California? The story will shock you.
'''.replaceAll('\n', ' ');

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).primaryTextTheme;

    return SingleChildScrollView(
      child: SafeArea(
        top: false,
        child: ColoredBox(
          color: Theme.of(context).colorScheme.surface,
          child: Column(
            children: <Widget>[
              Container(
                constraints: const BoxConstraints.expand(height: 248),
                child: Image.asset(
                  'food/fruits.png',
                  package: 'flutter_gallery_assets',
                  fit: BoxFit.fitWidth,
                ),
              ),
              const SizedBox(height: 17),
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 8),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: <Widget>[
                    Row(
                      children: <Widget>[
                        Text('US', style: textTheme.labelSmall),
                        Text(
                          ' ¬ ',
                          // TODO(larche): Replace textTheme.headline2.color with a ColorScheme value when known.
                          style: textTheme.labelSmall!.apply(color: textTheme.displayMedium!.color),
                        ),
                        Text('CULTURE', style: textTheme.labelSmall),
                      ],
                    ),
                    const SizedBox(height: 10),
                    Text(
                      'Quince for Wisdom, Persimmon for Luck, Pomegranate for Love',
                      style: textTheme.headlineMedium,
                    ),
                    const SizedBox(height: 10),
                    Text(
                      'How these crazy fruits sweetened our hearts, relationships, '
                      'and puffed pastries',
                      style: textTheme.bodyMedium,
                    ),
                    Padding(
                      padding: const EdgeInsets.symmetric(vertical: 16),
                      child: Row(
                        children: <Widget>[
                          const CircleAvatar(
                            backgroundImage: ExactAssetImage(
                              'people/square/trevor.png',
                              package: 'flutter_gallery_assets',
                            ),
                            radius: 20,
                          ),
                          const SizedBox(width: 12),
                          Text('by', style: textTheme.displayMedium),
                          const SizedBox(width: 4),
                          const Text(
                            'Connor Eghan',
                            style: TextStyle(
                              fontFamily: 'Merriweather',
                              fontSize: 18,
                              fontWeight: FontWeight.w500,
                              color: Colors.black,
                            ),
                          ),
                        ],
                      ),
                    ),
                    Text('$paragraph1\n\n$paragraph2', style: textTheme.bodyLarge),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

final ThemeData _fortnightlyTheme = _buildFortnightlyTheme();

ThemeData _buildFortnightlyTheme() {
  final ThemeData base = ThemeData.light();
  return base.copyWith(
    primaryTextTheme: _buildTextTheme(base.primaryTextTheme),
    scaffoldBackgroundColor: Colors.white,
  );
}

TextTheme _buildTextTheme(TextTheme base) {
  TextTheme theme = base.apply(bodyColor: Colors.black);
  theme = theme.apply(displayColor: Colors.black);

  theme = theme.copyWith(
    headlineMedium: base.headlineMedium!.copyWith(
      fontFamily: 'Merriweather',
      fontStyle: FontStyle.italic,
      fontSize: 28,
      fontWeight: FontWeight.w800,
      color: Colors.black,
      height: .88,
    ),
    displayMedium: base.displayMedium!.copyWith(
      fontFamily: 'LibreFranklin',
      fontSize: 18,
      fontWeight: FontWeight.w500,
      color: Colors.black.withAlpha(153),
    ),
    headlineSmall: base.headlineSmall!.copyWith(fontWeight: FontWeight.w500),
    bodyMedium: base.bodyMedium!.copyWith(
      fontFamily: 'Merriweather',
      fontSize: 14,
      fontWeight: FontWeight.w300,
      color: const Color(0xFF666666),
      height: 1.11,
    ),
    bodyLarge: base.bodyLarge!.copyWith(
      fontFamily: 'Merriweather',
      fontSize: 16,
      fontWeight: FontWeight.w300,
      color: const Color(0xFF666666),
      height: 1.4,
      letterSpacing: .25,
    ),
    labelSmall: const TextStyle(
      fontFamily: 'LibreFranklin',
      fontSize: 10,
      fontWeight: FontWeight.w700,
      color: Colors.black,
    ),
  );

  return theme;
}
